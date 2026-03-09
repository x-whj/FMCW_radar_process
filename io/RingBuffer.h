#pragma once

#include <array>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <optional>

namespace radar
{

    enum class FrameSlotState
    {
        FREE = 0,
        FILLING,
        READY,
        INFLIGHT
    };

    template <typename Slot, std::size_t N>
    class FixedSlotRing
    {
    public:
        FixedSlotRing()
        {
            for (auto &s : states_)
            {
                s = FrameSlotState::FREE;
            }
        }

        std::optional<std::size_t> acquire_free_slot()
        {
            std::lock_guard<std::mutex> lk(m_);
            for (std::size_t i = 0; i < N; ++i)
            {
                if (states_[i] == FrameSlotState::FREE)
                {
                    states_[i] = FrameSlotState::FILLING;
                    return i;
                }
            }
            return std::nullopt;
        }

        void mark_ready(std::size_t idx)
        {
            {
                std::lock_guard<std::mutex> lk(m_);
                states_[idx] = FrameSlotState::READY;
            }
            cv_.notify_one();
        }

        std::optional<std::size_t> wait_and_acquire_ready()
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait(lk, [&]()
                     {
            for (auto s : states_) {
                if (s == FrameSlotState::READY) {
                    return true;
                }
            }
            return shutdown_; });

            if (shutdown_)
            {
                return std::nullopt;
            }

            for (std::size_t i = 0; i < N; ++i)
            {
                if (states_[i] == FrameSlotState::READY)
                {
                    states_[i] = FrameSlotState::INFLIGHT;
                    return i;
                }
            }

            return std::nullopt;
        }

        void release(std::size_t idx)
        {
            std::lock_guard<std::mutex> lk(m_);
            states_[idx] = FrameSlotState::FREE;
        }

        void shutdown()
        {
            {
                std::lock_guard<std::mutex> lk(m_);
                shutdown_ = true;
            }
            cv_.notify_all();
        }

    private:
        std::array<FrameSlotState, N> states_{};
        std::mutex m_;
        std::condition_variable cv_;
        bool shutdown_ = false;
    };

} // namespace radar