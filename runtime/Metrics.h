#pragma once
#include <atomic>
#include <cstdint>
namespace radar
{
    struct RuntimeMetrics
    {
        std::atomic<uint64_t> udp_packets_rx{0};
        std::atomic<uint64_t> udp_packets_drop{0};
        std::atomic<uint64_t> udp_packets_dup{0};
        std::atomic<uint64_t> frames_completed{0};
        std::atomic<uint64_t> frames_dropped{0};
        std::atomic<uint64_t> gpu_frames_processed{0};
        std::atomic<uint64_t> gpu_targets_reported{0};
    };
} // namespace radar