#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "model/RadarConfig.h"

namespace radar
{

    class OfflineReplay
    {
    public:
        OfflineReplay(const RadarConfig &cfg,
                      const std::string &sum_file,
                      const std::string &az_file,
                      const std::string &el_file)
            : cfg_(cfg)
        {
            load_one_channel(sum_file, sum_ch_);
            load_one_channel(az_file, az_ch_);
            load_one_channel(el_file, el_ch_);

            if (sum_ch_.size() != az_ch_.size() || sum_ch_.size() != el_ch_.size())
            {
                throw std::runtime_error("offline replay: channel file sizes do not match");
            }

            samples_per_frame_ =
                static_cast<std::size_t>(cfg_.num_chirps) *
                static_cast<std::size_t>(cfg_.num_samples);

            if (sum_ch_.size() % samples_per_frame_ != 0)
            {
                throw std::runtime_error("offline replay: file size is not an integer number of frames");
            }

            total_frames_ = sum_ch_.size() / samples_per_frame_;
        }

        std::size_t total_frames() const
        {
            return total_frames_;
        }

        // 构造给 GPU unpack 使用的一帧原始 payload。
        // 按文档中的单 PRT 数据区语义拼成整帧 CPI：
        // [chirp][channel][sample][Q,I]
        // 其中 channel 顺序固定为：
        // sum -> az diff -> el diff
        //
        // 注意：文档规定 32-bit 单元内低 16 位是 I，高 16 位是 Q。
        // 接收端按 16-bit little-endian 读取后，内存中的顺序就是 [Q, I]，
        // 这里离线回放显式模拟这一点，后续由 GPU unpack 再交换回 (I, Q)。
        void build_frame_payload(std::size_t frame_idx, std::vector<int16_t> &out_raw_i16) const
        {
            if (frame_idx >= total_frames_)
            {
                throw std::runtime_error("offline replay: frame index out of range");
            }

            out_raw_i16.resize(samples_per_frame_ * cfg_.num_channels * 2);

            const std::size_t frame_offset = frame_idx * samples_per_frame_;
            const std::size_t channel_stride =
                static_cast<std::size_t>(cfg_.num_samples) * 2ULL;
            const std::size_t chirp_stride =
                static_cast<std::size_t>(cfg_.num_channels) * channel_stride;

            for (int chirp = 0; chirp < cfg_.num_chirps; ++chirp)
            {
                const std::size_t chirp_base =
                    static_cast<std::size_t>(chirp) * chirp_stride;

                for (int sample = 0; sample < cfg_.num_samples; ++sample)
                {
                    const std::size_t local_idx =
                        static_cast<std::size_t>(chirp) * static_cast<std::size_t>(cfg_.num_samples) +
                        static_cast<std::size_t>(sample);

                    const std::size_t src_idx = frame_offset + local_idx;

                    const IQPair &sumv = sum_ch_[src_idx];
                    const IQPair &azv = az_ch_[src_idx];
                    const IQPair &elv = el_ch_[src_idx];

                    const std::size_t sample_base =
                        chirp_base + static_cast<std::size_t>(sample) * 2ULL;

                    write_wire_iq(out_raw_i16, sample_base, sumv);
                    write_wire_iq(out_raw_i16, chirp_base + channel_stride + static_cast<std::size_t>(sample) * 2ULL, azv);
                    write_wire_iq(out_raw_i16, chirp_base + 2ULL * channel_stride + static_cast<std::size_t>(sample) * 2ULL, elv);
                }
            }
        }

    private:
        using IQPair = std::pair<float, float>;

        static int16_t float_to_i16(float x)
        {
            if (x > 32767.0f)
                x = 32767.0f;
            if (x < -32768.0f)
                x = -32768.0f;
            return static_cast<int16_t>(x);
        }

        static void write_wire_iq(std::vector<int16_t> &dst,
                                  std::size_t offset_i16,
                                  const IQPair &iq)
        {
            dst[offset_i16 + 0] = float_to_i16(iq.second); // Q
            dst[offset_i16 + 1] = float_to_i16(iq.first);  // I
        }

        // 读取单通道文件：
        // 文件格式为 float IQ 交织：
        // [I0, Q0, I1, Q1, I2, Q2, ...]
        void load_one_channel(const std::string &path, std::vector<IQPair> &out)
        {
            std::ifstream ifs(path, std::ios::binary);
            if (!ifs)
            {
                throw std::runtime_error("offline replay: failed to open file: " + path);
            }

            ifs.seekg(0, std::ios::end);
            const std::streamsize bytes = ifs.tellg();
            ifs.seekg(0, std::ios::beg);

            if (bytes <= 0 || (bytes % sizeof(float)) != 0)
            {
                throw std::runtime_error("offline replay: invalid float file size: " + path);
            }

            std::vector<float> raw(static_cast<std::size_t>(bytes / sizeof(float)));
            if (!ifs.read(reinterpret_cast<char *>(raw.data()), bytes))
            {
                throw std::runtime_error("offline replay: failed to read file: " + path);
            }

            if ((raw.size() % 2) != 0)
            {
                throw std::runtime_error("offline replay: IQ float count must be even: " + path);
            }

            out.resize(raw.size() / 2);
            for (std::size_t i = 0; i < out.size(); ++i)
            {
                out[i] = {raw[2 * i + 0], raw[2 * i + 1]};
            }
        }

    private:
        RadarConfig cfg_;
        std::size_t samples_per_frame_ = 0;
        std::size_t total_frames_ = 0;

        // 每个通道都是线性复数流，顺序是：
        // [frame][chirp][sample]
        std::vector<IQPair> sum_ch_;
        std::vector<IQPair> az_ch_;
        std::vector<IQPair> el_ch_;
    };

} // namespace radar
