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

        // 构造给 GPU unpack 使用的一帧原始 payload
        // 每个 (chirp, sample) 点按如下顺序写入：
        // [sum_I, sum_Q, az_I, az_Q, el_I, el_Q]
        void build_frame_payload(std::size_t frame_idx, std::vector<int16_t> &out_raw_i16) const
        {
            if (frame_idx >= total_frames_)
            {
                throw std::runtime_error("offline replay: frame index out of range");
            }

            out_raw_i16.resize(samples_per_frame_ * cfg_.num_channels * 2);

            const std::size_t frame_offset = frame_idx * samples_per_frame_;
            std::size_t dst = 0;

            // 明确按 [chirp][sample] 的顺序拼
            for (int chirp = 0; chirp < cfg_.num_chirps; ++chirp)
            {
                for (int sample = 0; sample < cfg_.num_samples; ++sample)
                {
                    const std::size_t local_idx =
                        static_cast<std::size_t>(chirp) * static_cast<std::size_t>(cfg_.num_samples) +
                        static_cast<std::size_t>(sample);

                    const std::size_t src_idx = frame_offset + local_idx;

                    const IQPair &sumv = sum_ch_[src_idx];
                    const IQPair &azv = az_ch_[src_idx];
                    const IQPair &elv = el_ch_[src_idx];

                    // ch0 = sum
                    out_raw_i16[dst++] = float_to_i16(sumv.first);
                    out_raw_i16[dst++] = float_to_i16(sumv.second);

                    // ch1 = az diff
                    out_raw_i16[dst++] = float_to_i16(azv.first);
                    out_raw_i16[dst++] = float_to_i16(azv.second);

                    // ch2 = el diff
                    out_raw_i16[dst++] = float_to_i16(elv.first);
                    out_raw_i16[dst++] = float_to_i16(elv.second);
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