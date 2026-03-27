#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "io/PrtProtocol.h"
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
                    // 在这段字节流里查找下一个合法的PRT帧头。
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

        const PrtHeaderInfo &current_frame_header() const
        {
            return last_completed_cpi_header_;
        }

    private:
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
            assembling_cpi_header_ = {};
            std::fill(cpi_bitmap_.begin(), cpi_bitmap_.end(), uint8_t{0});
        }

        void start_new_cpi(uint32_t frame_id, const PrtHeaderInfo &header)
        {
            reset_cpi_state();
            cpi_started_ = true;
            assembling_frame_id_ = frame_id;
            // 每个 CPI 只锁存一次首个 PRT 头，
            // 后续算法若需要用雷达角度、CFAR 门限等元数据，
            // 就从这一份 CPI 级别的头信息里取。
            assembling_cpi_header_ = header;
        }

        void resync_to_prt_head()
        {
            radar::resync_to_prt_head(prt_buffer_);
        }

        bool parse_prt_header()
        {
            const uint8_t *hdr = prt_buffer_.data();
            pending_prt_ = decode_prt_header(hdr);
            if (!pending_prt_.has_fixed_head())
            {
                return false;
            }

            const uint16_t prt_samples = pending_prt_.prt_samples();
            const uint16_t frame_length_bytes = pending_prt_.frame_length_bytes();
            const uint16_t data_format_bits = pending_prt_.data_format_bits();
            const uint16_t prt_num = pending_prt_.prt_num();

            // 实际流里有些头字段会保留为 0，此时回退到本地配置。
            if (prt_samples != 0 && prt_samples != static_cast<uint16_t>(cfg_.num_samples))
            {
                return false;
            }

            if (frame_length_bytes != 0 && frame_length_bytes != static_cast<uint16_t>(cfg_.prt_data_bytes()))
            {
                return false;
            }

            if (data_format_bits != 0 && data_format_bits != 16)
            {
                return false;
            }

            if (!cfg_.protocol_prt_num_in_range(static_cast<int>(prt_num)))
            {
                return false;
            }

            const std::size_t effective_frame_length =
                frame_length_bytes != 0 ? static_cast<std::size_t>(frame_length_bytes)
                                        : cfg_.prt_data_bytes();

            prt_total_bytes_ = RadarConfig::kPrtHeaderBytes +
                               effective_frame_length;
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
                if (pending_prt_.prt_num() != static_cast<uint16_t>(cfg_.active_prt_first_num()))
                {
                    return st;
                }

                start_new_cpi(pending_prt_.global_prt_count(), pending_prt_);
                st.started_new_frame = true;
            }
            //宁愿丢掉后续的 PRT 也不接受乱序的 PRT，因为乱序可能是因为 UDP 丢包导致的，接受乱序反而会把错误数据组装成完整帧输出。
            else if (pending_prt_.prt_num() == static_cast<uint16_t>(cfg_.active_prt_first_num()))
            {
                start_new_cpi(pending_prt_.global_prt_count(), pending_prt_);
                st.started_new_frame = true;
            }

            const int prt_num = static_cast<int>(pending_prt_.prt_num());

            if (!cfg_.active_prt_num_in_range(prt_num))
            {
                // 实际协议里尾部若干 PRT 用于切波位，不参与当前 CPI 的算法处理。
                return st;
            }

            const std::size_t prt_idx =
                static_cast<std::size_t>(prt_num - cfg_.active_prt_first_num());
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
                last_completed_cpi_header_ = assembling_cpi_header_;
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
        PrtHeaderInfo pending_prt_{};

        std::vector<uint8_t> cpi_bitmap_;
        bool cpi_started_ = false;
        uint32_t assembling_frame_id_ = 0;
        uint32_t last_completed_frame_id_ = 0;
        std::size_t received_prts_ = 0;
        PrtHeaderInfo assembling_cpi_header_{};
        PrtHeaderInfo last_completed_cpi_header_{};
    };

} // namespace radar
