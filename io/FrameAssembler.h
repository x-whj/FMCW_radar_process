#pragma once
#include <cstdint>
#include <cstring>
#include <vector>
namespace radar
{
#pragma pack(push, 1)
    struct FpgaUdpHeader
    {
        uint32_t frame_id;
        uint32_t packet_idx;
        uint32_t total_packets;
        uint32_t payload_len;
    };
#pragma pack(pop)
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
        FrameAssembler() = default;
        void reset(uint32_t frame_id, std::size_t total_packets, std::size_t frame_payload_bytes)
        {
            current_frame_id_ = frame_id;
            total_packets_ = total_packets;
            frame_payload_bytes_ = frame_payload_bytes;
            received_packets_ = 0;
            received_bytes_ = 0;
            packet_bitmap_.assign(total_packets_, 0);
        }
        AssemblyStatus push_packet(const FpgaUdpHeader &hdr,
                                   const uint8_t *payload,
                                   uint8_t *dst_frame,
                                   std::size_t payload_capacity)
        {
            AssemblyStatus st;
            if (current_frame_id_ != hdr.frame_id)
            {
                reset(hdr.frame_id, hdr.total_packets, payload_capacity);
                st.started_new_frame = true;
            }
            if (hdr.total_packets == 0 || hdr.packet_idx >= total_packets_)
            {
                st.packet_invalid = true;
                return st;
            }
            if (packet_bitmap_[hdr.packet_idx])
            {
                st.packet_duplicate = true;
                return st;
            }
            std::size_t packet_payload_len = hdr.payload_len;
            std::size_t nominal_stride = frame_payload_bytes_ / total_packets_;
            std::size_t offset = hdr.packet_idx * nominal_stride;
            if (offset + packet_payload_len > payload_capacity)
            {
                st.packet_invalid = true;
                return st;
            }
            std::memcpy(dst_frame + offset, payload, packet_payload_len);
            packet_bitmap_[hdr.packet_idx] = 1;
            received_packets_++;
            received_bytes_ += packet_payload_len;
            if (received_packets_ == total_packets_)
            {
                st.frame_completed = true;
            }
            return st;
        }
        uint32_t current_frame_id() const { return current_frame_id_; }
        std::size_t received_packets() const { return received_packets_; }
        std::size_t total_packets() const { return total_packets_; }

    private:
        uint32_t current_frame_id_ = 0xFFFFFFFFu;
        std::size_t total_packets_ = 0;
        std::size_t frame_payload_bytes_ = 0;
        std::size_t received_packets_ = 0;
        std::size_t received_bytes_ = 0;
        std::vector<uint8_t> packet_bitmap_;
    };
} // namespace radar