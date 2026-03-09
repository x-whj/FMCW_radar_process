#pragma once
#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>
#include "io/FrameAssembler.h"
#include "io/RingBuffer.h"
#include "model/RadarConfig.h"
#include "runtime/Logger.h"
#include "runtime/Metrics.h"
namespace radar
{
    struct RxFrameSlot
    {
        uint8_t *mapped_host = nullptr;
        int16_t *mapped_dev = nullptr;
        uint32_t frame_id = 0;
    };
    template <std::size_t N>
    class UdpReceiver
    {
    public:
        UdpReceiver(const RadarConfig &cfg,
                    RuntimeMetrics &metrics,
                    FixedSlotRing<RxFrameSlot, N> &ring,
                    std::array<RxFrameSlot, N> &slots)
            : cfg_(cfg), metrics_(metrics), ring_(ring), slots_(slots) {}
        ~UdpReceiver() { stop(); }
        bool start();
        void stop();

    private:
        void run();
        const RadarConfig &cfg_;
        RuntimeMetrics &metrics_;
        FixedSlotRing<RxFrameSlot, N> &ring_;
        std::array<RxFrameSlot, N> &slots_;
        std::atomic<bool> running_{false};
        std::thread worker_;
        int sockfd_ = -1;
    };
} // namespace radar
