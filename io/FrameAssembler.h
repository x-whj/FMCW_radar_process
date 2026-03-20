#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "model/RadarConfig.h"

namespace radar
{

    struct AssemblyStatus
    {
        bool started_new_frame = false;
        bool frame_completed = false;
        bool packet_duplicate = false;
        bool packet_invalid = false;
    };

    class FrameAssembler
    {
    public:
        explicit FrameAssembler(const RadarConfig &cfg)
            : cfg_(cfg),
              cpi_bitmap_(static_cast<std::size_t>(std::max(cfg_.num_chirps, 1)), 0)
        {
            prt_buffer_.reserve(cfg_.prt_total_bytes() +
                                static_cast<std::size_t>(std::max(cfg_.udp_payload_bytes, 0)));
        }

        AssemblyStatus push_packet(const uint8_t *payload,
                                   std::size_t payload_len,
                                   uint8_t *dst_frame,
                                   std::size_t payload_capacity)
        {
            AssemblyStatus st;

            if (payload == nullptr || dst_frame == nullptr || payload_len == 0)
            {
                st.packet_invalid = true;
                return st;
            }

            prt_buffer_.insert(prt_buffer_.end(), payload, payload + payload_len);

            while (true)
            {
                if (!prt_header_parsed_)
                {
                    // 把 UDP payload 视为连续字节流，
                    // 在这段字节流里查找下一个合法的 PRT 帧头。
                    resync_to_prt_head();
                    if (prt_buffer_.size() < RadarConfig::kPrtHeaderBytes)
                    {
                        break;
                    }

                    if (!parse_prt_header())
                    {
                        st.packet_invalid = true;
                        if (!prt_buffer_.empty())
                        {
                            prt_buffer_.erase(prt_buffer_.begin());
                        }
                        reset_prt_state();
                        continue;
                    }
                }

                if (prt_buffer_.size() < prt_padded_bytes_)
                {
                    // 当前还没收齐一个完整的 PRT，继续缓存后续 UDP 数据。
                    break;
                }

                const AssemblyStatus prt_status = commit_prt(dst_frame, payload_capacity);
                st.started_new_frame = st.started_new_frame || prt_status.started_new_frame;
                st.frame_completed = st.frame_completed || prt_status.frame_completed;
                st.packet_duplicate = st.packet_duplicate || prt_status.packet_duplicate;
                st.packet_invalid = st.packet_invalid || prt_status.packet_invalid;

                prt_buffer_.erase(prt_buffer_.begin(),
                                  prt_buffer_.begin() + static_cast<std::ptrdiff_t>(prt_padded_bytes_));
                reset_prt_state();
            }

            return st;
        }

        uint32_t current_frame_id() const
        {
            return last_completed_frame_id_;
        }

    private:
        struct PendingPrtHeader
        {
            uint32_t global_prt_count = 0;
            uint16_t prt_num = 0;
            uint16_t prt_samples = 0;
            uint16_t frame_length_bytes = 0;
            uint16_t radar_mode = 0;
            uint16_t data_type = 0;
            uint16_t data_format_bits = 0;
        };

        static uint32_t read_le_u32(const uint8_t *p)
        {
            return static_cast<uint32_t>(p[0]) |
                   (static_cast<uint32_t>(p[1]) << 8) |
                   (static_cast<uint32_t>(p[2]) << 16) |
                   (static_cast<uint32_t>(p[3]) << 24);
        }

        static std::size_t round_up(std::size_t value, std::size_t align)
        {
            if (align == 0)
            {
                return value;
            }
            const std::size_t rem = value % align;
            return rem == 0 ? value : (value + align - rem);
        }

        void reset_prt_state()
        {
            prt_header_parsed_ = false;
            prt_total_bytes_ = 0;
            prt_padded_bytes_ = 0;
            pending_prt_ = {};
        }

        void reset_cpi_state()
        {
            cpi_started_ = false;
            assembling_frame_id_ = 0;
            received_prts_ = 0;
            std::fill(cpi_bitmap_.begin(), cpi_bitmap_.end(), uint8_t{0});
        }

        void start_new_cpi(uint32_t frame_id)
        {
            reset_cpi_state();
            cpi_started_ = true;
            assembling_frame_id_ = frame_id;
        }

        void resync_to_prt_head()
        {
            static constexpr uint8_t kPrtHeadLE[4] = {0x32, 0xCD, 0x55, 0xAA};

            auto it = std::search(prt_buffer_.begin(),
                                  prt_buffer_.end(),
                                  std::begin(kPrtHeadLE),
                                  std::end(kPrtHeadLE));

            if (it == prt_buffer_.end())
            {
                if (prt_buffer_.size() > 3)
                {
                    prt_buffer_.erase(prt_buffer_.begin(),
                                      prt_buffer_.end() - 3);
                }
                return;
            }

            if (it != prt_buffer_.begin())
            {
                prt_buffer_.erase(prt_buffer_.begin(), it);
            }
        }

        bool parse_prt_header()
        {
            const uint8_t *hdr = prt_buffer_.data();
            if (read_le_u32(hdr) != RadarConfig::kPrtFixedHeader)
            {
                return false;
            }

            pending_prt_.global_prt_count = read_le_u32(hdr + 8);

            const uint32_t prt_info = read_le_u32(hdr + 12);
            pending_prt_.prt_num = static_cast<uint16_t>(prt_info >> 16);
            pending_prt_.prt_samples = static_cast<uint16_t>(prt_info & 0xFFFFu);

            const uint32_t frame_info = read_le_u32(hdr + 16);
            pending_prt_.frame_length_bytes = static_cast<uint16_t>(frame_info >> 16);
            pending_prt_.radar_mode = static_cast<uint16_t>(frame_info & 0xFFFFu);

            const uint32_t data_info = read_le_u32(hdr + 20);
            pending_prt_.data_type = static_cast<uint16_t>(data_info >> 16);
            pending_prt_.data_format_bits = static_cast<uint16_t>(data_info & 0xFFFFu);

            // 当前只接受和现有处理链一致的协议子集：
            // 固定采样点数、int16 复数 IQ 数据。
            if (pending_prt_.prt_samples != static_cast<uint16_t>(cfg_.num_samples))
            {
                return false;
            }

            if (pending_prt_.frame_length_bytes != static_cast<uint16_t>(cfg_.prt_data_bytes()))
            {
                return false;
            }

            if (pending_prt_.data_format_bits != 16)
            {
                return false;
            }

            if (pending_prt_.prt_num >= static_cast<uint16_t>(cfg_.num_chirps))
            {
                return false;
            }

            prt_total_bytes_ = RadarConfig::kPrtHeaderBytes +
                               static_cast<std::size_t>(pending_prt_.frame_length_bytes);
            prt_padded_bytes_ = round_up(prt_total_bytes_,
                                         static_cast<std::size_t>(std::max(cfg_.udp_payload_bytes, 0)));
            prt_header_parsed_ = true;
            return true;
        }

        AssemblyStatus commit_prt(uint8_t *dst_frame, std::size_t payload_capacity)
        {
            AssemblyStatus st;

            const std::size_t prt_data_bytes = cfg_.prt_data_bytes();
            if (payload_capacity < cfg_.frame_payload_bytes() ||
                prt_total_bytes_ < RadarConfig::kPrtHeaderBytes + prt_data_bytes)
            {
                st.packet_invalid = true;
                return st;
            }

            if (!cpi_started_)
            {
                if (pending_prt_.prt_num != 0)
                {
                    return st;
                }

                start_new_cpi(pending_prt_.global_prt_count);
                st.started_new_frame = true;
            }
            else if (pending_prt_.prt_num == 0)
            {
                start_new_cpi(pending_prt_.global_prt_count);
                st.started_new_frame = true;
            }

            const std::size_t prt_idx = static_cast<std::size_t>(pending_prt_.prt_num);
            if (prt_idx >= cpi_bitmap_.size())
            {
                st.packet_invalid = true;
                return st;
            }

            if (cpi_bitmap_[prt_idx] != 0)
            {
                st.packet_duplicate = true;
                return st;
            }

            const std::size_t dst_offset = prt_idx * prt_data_bytes;
            if (dst_offset + prt_data_bytes > payload_capacity)
            {
                st.packet_invalid = true;
                return st;
            }

            // GPU 期望拿到的整帧 CPI 数据区布局是：
            // [chirp/PRT][channel][sample][Q,I]
            // 因此这里把每个完整 PRT 的数据区拷贝到由 PRT_Num 指定的位置上。
            std::memcpy(dst_frame + dst_offset,
                        prt_buffer_.data() + RadarConfig::kPrtHeaderBytes,
                        prt_data_bytes);

            cpi_bitmap_[prt_idx] = 1;
            received_prts_++;

            if (received_prts_ == static_cast<std::size_t>(cfg_.num_chirps))
            {
                last_completed_frame_id_ = assembling_frame_id_;
                reset_cpi_state();
                st.frame_completed = true;
            }

            return st;
        }

    private:
        const RadarConfig &cfg_;

        std::vector<uint8_t> prt_buffer_;
        bool prt_header_parsed_ = false;
        std::size_t prt_total_bytes_ = 0;
        std::size_t prt_padded_bytes_ = 0;
        PendingPrtHeader pending_prt_{};

        std::vector<uint8_t> cpi_bitmap_;
        bool cpi_started_ = false;
        uint32_t assembling_frame_id_ = 0;
        uint32_t last_completed_frame_id_ = 0;
        std::size_t received_prts_ = 0;
    };

} // namespace radar
