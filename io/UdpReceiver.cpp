#include "io/UdpReceiver.h"
#include <chrono>
#include <arpa/inet.h>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>
namespace radar
{
    template <std::size_t N>
    bool UdpReceiver<N>::start()
    {
        if (running_)
            return true;
        sockfd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0)
        {
            Logger::error("socket creation failed");
            return false;
        }
        int rcvbuf = cfg_.socket_rcvbuf_bytes;
        setsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(cfg_.udp_port);
        if (::bind(sockfd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
        {
            Logger::error("bind failed on port ", cfg_.udp_port);
            ::close(sockfd_);
            sockfd_ = -1;
            return false;
        }
        running_ = true;
        worker_ = std::thread(&UdpReceiver::run, this);
        Logger::info("UDP receiver started on port ", cfg_.udp_port);
        return true;
    }
    template <std::size_t N>
    void UdpReceiver<N>::stop()
    {
        if (!running_)
            return;
        running_ = false;
        if (sockfd_ >= 0)
        {
            ::close(sockfd_);
            sockfd_ = -1;
        }
        if (worker_.joinable())
            worker_.join();
    }
    template <std::size_t N>
    void UdpReceiver<N>::run()
    {
        std::vector<uint8_t> udp_buf(cfg_.mtu_bytes);
        FrameAssembler assembler(cfg_);
        std::optional<std::size_t> filling_slot = ring_.acquire_free_slot();
        if (!filling_slot.has_value())
        {
            Logger::error("no free rx slot when receiver starts");
            running_ = false;
            return;
        }
        while (running_)
        {
            int n = ::recvfrom(sockfd_, udp_buf.data(),
                               static_cast<int>(udp_buf.size()), 0, nullptr, nullptr);
            if (n <= 0)
                continue;
            metrics_.udp_packets_rx.fetch_add(1, std::memory_order_relaxed);

            // The document format no longer uses a custom per-UDP software header.
            // recvfrom() returns one UDP payload chunk from the raw PRT byte stream,
            // and FrameAssembler is responsible for finding PRT boundaries inside it.
            auto st = assembler.push_packet(udp_buf.data(),
                                            static_cast<std::size_t>(n),
                                            slots_[*filling_slot].mapped_host,
                                            cfg_.frame_payload_bytes());
            if (st.packet_invalid)
            {
                metrics_.udp_packets_drop.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            if (st.packet_duplicate)
            {
                metrics_.udp_packets_dup.fetch_add(1, std::memory_order_relaxed);
                continue;
            }
            if (st.frame_completed)
            {
                slots_[*filling_slot].frame_id = assembler.current_frame_id();
                ring_.mark_ready(*filling_slot);
                metrics_.frames_completed.fetch_add(1, std::memory_order_relaxed);
                auto next = ring_.acquire_free_slot();
                if (!next.has_value()){
                    metrics_.frames_dropped.fetch_add(1, std::memory_order_relaxed);
                    Logger::warn("RX ring full, dropping completed frame or waiting for GPU too long");
                    while (running_ && !next.has_value()) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        next = ring_.acquire_free_slot();
                    }
                    if (!running_) break;
                }
                filling_slot = next;
            }
        }
    }
    // 显式实例化
    template class UdpReceiver<3>;
} // namespace radar
