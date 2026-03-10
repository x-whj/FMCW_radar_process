#include "gpu/RadarPipeline.h"

#include <stdexcept>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>

namespace radar
{
    static void dump_power_map_to_file(const float *h_power_map,
                                       int num_chirps,
                                       int num_samples,
                                       int frame_index)
    {
        std::ostringstream oss;
        oss << "power_map_frame_"
            << std::setw(3) << std::setfill('0') << frame_index
            << ".bin";

        std::ofstream ofs(oss.str(), std::ios::binary);
        if (!ofs)
        {
            throw std::runtime_error("failed to open dump file: " + oss.str());
        }

        const std::size_t bytes =
            static_cast<std::size_t>(num_chirps) *
            static_cast<std::size_t>(num_samples) *
            sizeof(float);

        ofs.write(reinterpret_cast<const char *>(h_power_map), bytes);
        if (!ofs)
        {
            throw std::runtime_error("failed to write dump file: " + oss.str());
        }
    }

    // DBSCAN 邻域判定（在 range-velocity 平面上做“归一化椭圆距离”）
    static bool dbscan_is_neighbor(const RadarTarget &a,
                                   const RadarTarget &b,
                                   float eps_range_m,
                                   float eps_velocity_mps)
    {
        const float dr = (a.range_m - b.range_m) / eps_range_m;
        const float dv = (a.velocity_mps - b.velocity_mps) / eps_velocity_mps;
        return (dr * dr + dv * dv) <= 1.0f;
    }

    // 在 (range, velocity) 上做 DBSCAN 聚类，每簇输出 1 个代表点（默认选簇内最强点）。
    static int cluster_targets_dbscan(const std::vector<RadarTarget> &input,
                                      const RadarConfig &cfg,
                                      std::vector<RadarTarget> &output)
    {
        output.clear();
        if (input.empty())
        {
            return 0;
        }

        const int n = static_cast<int>(input.size());
        const float eps_range_m = std::max(cfg.dbscan_eps_range_m, 1e-6f);
        const float eps_velocity_mps = std::max(cfg.dbscan_eps_velocity_mps, 1e-6f);
        const int min_points = std::max(cfg.dbscan_min_points, 1);

        std::vector<std::vector<int>> neighbors(n);
        for (int i = 0; i < n; ++i)
        {
            neighbors[i].reserve(n);
            for (int j = 0; j < n; ++j)
            {
                if (dbscan_is_neighbor(input[i], input[j], eps_range_m, eps_velocity_mps))
                {
                    neighbors[i].push_back(j);
                }
            }
        }

        constexpr int kUnassigned = -2;
        constexpr int kNoise = -1;

        std::vector<int> labels(n, kUnassigned);
        std::vector<uint8_t> visited(n, 0);

        int cluster_count = 0;
        for (int i = 0; i < n; ++i)
        {
            if (visited[i])
                continue;

            visited[i] = 1;
            if (static_cast<int>(neighbors[i].size()) < min_points)
            {
                labels[i] = kNoise;
                continue;
            }

            const int cluster_id = cluster_count++;
            labels[i] = cluster_id;

            std::vector<int> queue = neighbors[i];
            std::size_t head = 0;
            while (head < queue.size())
            {
                const int j = queue[head++];
                if (!visited[j])
                {
                    visited[j] = 1;
                    if (static_cast<int>(neighbors[j].size()) >= min_points)
                    {
                        queue.insert(queue.end(), neighbors[j].begin(), neighbors[j].end());
                    }
                }

                if (labels[j] == kUnassigned || labels[j] == kNoise)
                {
                    labels[j] = cluster_id;
                }
            }
        }

        if (cluster_count == 0)
        {
            // When no dense cluster is formed, keep candidates for temporal selection.
            // Forcing strongest-only here can lock to wrong sidelobes/high-velocity artifacts.
            output = input;
            return 0;
        }

        struct ClusterBest
        {
            int best_idx = -1;
            int count = 0;
        };

        std::vector<ClusterBest> clusters(cluster_count);
        for (int i = 0; i < n; ++i)
        {
            const int label = labels[i];
            if (label < 0)
                continue;

            auto &c = clusters[label];
            c.count++;
            if (c.best_idx < 0 ||
                input[i].snr_db > input[c.best_idx].snr_db ||
                (input[i].snr_db == input[c.best_idx].snr_db && input[i].power > input[c.best_idx].power))
            {
                c.best_idx = i;
            }
        }

        output.reserve(cluster_count + (cfg.dbscan_keep_noise ? n : 0));
        for (const auto &c : clusters)
        {
            if (c.best_idx >= 0)
            {
                output.push_back(input[c.best_idx]);
            }
        }

        if (cfg.dbscan_keep_noise)
        {
            for (int i = 0; i < n; ++i)
            {
                if (labels[i] == kNoise)
                {
                    output.push_back(input[i]);
                }
            }
        }

        if (cfg.dbscan_max_clusters > 0 &&
            static_cast<int>(output.size()) > cfg.dbscan_max_clusters)
        {
            std::partial_sort(output.begin(),
                              output.begin() + cfg.dbscan_max_clusters,
                              output.end(),
                              [](const RadarTarget &a, const RadarTarget &b)
                              {
                                  if (a.snr_db != b.snr_db)
                                      return a.snr_db > b.snr_db;
                                  return a.power > b.power;
                              });
            output.resize(cfg.dbscan_max_clusters);
        }

        return cluster_count;
    }

    struct DopplerLineSuppressStats
    {
        int suppressed_points = 0;
        int suppressed_bins = 0;
    };

    static int centered_doppler_bin_from_target(const RadarTarget &target,
                                                const RadarConfig &cfg)
    {
        const float vr = std::fabs(cfg.velocity_resolution_mps);
        if (vr < 1e-6f)
        {
            return 0;
        }
        return static_cast<int>(std::lround(target.velocity_mps / cfg.velocity_resolution_mps));
    }

    static DopplerLineSuppressStats suppress_doppler_line_targets(
        const std::vector<RadarTarget> &input,
        const RadarConfig &cfg,
        std::vector<RadarTarget> &output)
    {
        DopplerLineSuppressStats stats{};
        output.clear();

        if (input.empty() || !cfg.post_doppler_line_suppress_enable)
        {
            output = input;
            return stats;
        }

        const int min_points = std::max(cfg.post_doppler_line_min_points, 2);
        const int keep_per_bin = std::max(cfg.post_doppler_line_keep_per_bin, 1);

        struct BinIndex
        {
            int dbin = 0;
            int idx = -1;
        };

        std::vector<BinIndex> bin_indices;
        bin_indices.reserve(input.size());
        for (int i = 0; i < static_cast<int>(input.size()); ++i)
        {
            BinIndex b;
            b.dbin = centered_doppler_bin_from_target(input[i], cfg);
            b.idx = i;
            bin_indices.push_back(b);
        }

        std::sort(bin_indices.begin(), bin_indices.end(),
                  [](const BinIndex &a, const BinIndex &b)
                  {
                      if (a.dbin != b.dbin)
                          return a.dbin < b.dbin;
                      return a.idx < b.idx;
                  });

        std::vector<uint8_t> keep(input.size(), 1);

        int start = 0;
        while (start < static_cast<int>(bin_indices.size()))
        {
            int end = start + 1;
            const int dbin = bin_indices[start].dbin;
            while (end < static_cast<int>(bin_indices.size()) &&
                   bin_indices[end].dbin == dbin)
            {
                ++end;
            }

            const int group_size = end - start;
            if (group_size >= min_points)
            {
                stats.suppressed_bins++;
                std::vector<int> group;
                group.reserve(group_size);
                for (int i = start; i < end; ++i)
                {
                    group.push_back(bin_indices[i].idx);
                }

                std::sort(group.begin(), group.end(),
                          [&input](int lhs, int rhs)
                          {
                              if (input[lhs].snr_db != input[rhs].snr_db)
                                  return input[lhs].snr_db > input[rhs].snr_db;
                              return input[lhs].power > input[rhs].power;
                          });

                for (int idx : group)
                {
                    keep[idx] = 0;
                }
                const int keep_n = std::min(keep_per_bin, static_cast<int>(group.size()));
                for (int i = 0; i < keep_n; ++i)
                {
                    keep[group[i]] = 1;
                }
            }

            start = end;
        }

        output.reserve(input.size());
        for (int i = 0; i < static_cast<int>(input.size()); ++i)
        {
            if (keep[i])
            {
                output.push_back(input[i]);
            }
        }

        stats.suppressed_points =
            static_cast<int>(input.size()) - static_cast<int>(output.size());
        return stats;
    }

    void upload_slow_time_window(const float *h_window, int count);

    void launch_unpack_and_window(cudaStream_t stream,
                                  const int16_t *raw,
                                  float2 *cube,
                                  int num_channels,
                                  int num_chirps,
                                  int num_samples);

    void launch_sum_channel_power(cudaStream_t stream,
                                  const float2 *cube,
                                  float *power_map,
                                  int sum_channel,
                                  int elements_per_channel);

    void launch_cfar_and_peak_extract_v2(cudaStream_t stream,
                                         const float *power_map,
                                         uint8_t *hit_map,
                                         float *noise_map,
                                         float *threshold_map,
                                         TargetDetection *detections,
                                         int *hit_count,
                                         int *peak_count,
                                         int width,
                                         int height,
                                         int train_x,
                                         int train_y,
                                         int guard_x,
                                         int guard_y,
                                         float pfa,
                                         float min_noise,
                                         float min_cut_power,
                                         int near_range_suppress_bins,
                                         int zero_doppler_suppress_bins,
                                         int peak_half_window,
                                         float peak_min_snr_db,
                                         int max_targets);

    void launch_monopulse(cudaStream_t stream,
                          const float2 *cube,
                          const TargetDetection *dets,
                          RadarTarget *targets,
                          const int *det_count,
                          int max_targets,
                          const RadarConfig &cfg,
                          const ChannelMap &chmap,
                          const MonopulseCalibration &calib);

    RadarPipeline::RadarPipeline(const RadarConfig &cfg,
                                 const ChannelMap &channel_map,
                                 const MonopulseCalibration &calib)
        : cfg_(cfg), chmap_(channel_map), calib_(calib) {}

    RadarPipeline::~RadarPipeline()
    {
        fft_plan_.destroy();
        free_gpu_buffers(buffers_);
        if (stream_)
        {
            cudaStreamDestroy(stream_);
            stream_ = nullptr;
        }
    }

    void RadarPipeline::initialize(const float *h_window_256)
    {
        CUDA_CHECK(cudaStreamCreate(&stream_));
        allocate_gpu_buffers(cfg_, buffers_);
        fft_plan_.create(cfg_, stream_);
        upload_slow_time_window(h_window_256, cfg_.num_chirps);
    }

    int RadarPipeline::process_frame(const int16_t *d_or_mapped_raw,
                                     std::vector<RadarTarget> &out_targets)
    {
        // 一帧主流程（建议按以下阶段读代码）：
        // 1 清理计数与中间缓冲
        // 2 处理输入来源（mapped zero-copy 或 staged memcpy）
        // 3 unpack + slow-time window
        // 4 各通道多普勒 FFT
        // 5 求和通道功率图
        // 6 (可选)导出功率图供 Matlab 对比
        // 7 CFAR + 峰值提取
        // 8 单脉冲角度/速度/距离形成目标列表
        // 9~16 主机侧后处理（过滤/聚类/Doppler线抑制/top-k/单目标时序选择）
        const int plane = cfg_.num_chirps * cfg_.num_samples;
        static int s_dump_frame_index = 0;

        // Stage 1: 每帧开始先清理状态缓冲
        CUDA_CHECK(cudaMemsetAsync(buffers_.d_hit_map,
                                   0,
                                   cfg_.rd_map_elements() * sizeof(uint8_t),
                                   stream_));

        CUDA_CHECK(cudaMemsetAsync(buffers_.d_noise_map,
                                   0,
                                   cfg_.rd_map_bytes(),
                                   stream_));

        CUDA_CHECK(cudaMemsetAsync(buffers_.d_threshold_map,
                                   0,
                                   cfg_.rd_map_bytes(),
                                   stream_));

        CUDA_CHECK(cudaMemsetAsync(buffers_.d_hit_count,
                                   0,
                                   sizeof(int),
                                   stream_));

        CUDA_CHECK(cudaMemsetAsync(buffers_.d_peak_count,
                                   0,
                                   sizeof(int),
                                   stream_));

        const int16_t *raw_input = d_or_mapped_raw;

        // Stage 2: 输入策略
        // prefer_mapped_zero_copy=true: kernel 直接读 mapped host 内存
        // false: 先拷到设备 staging 缓冲，再读
        if (!cfg_.prefer_mapped_zero_copy)
        {
            CUDA_CHECK(cudaMemcpyAsync(buffers_.d_stage_raw,
                                       d_or_mapped_raw,
                                       cfg_.frame_payload_bytes(),
                                       cudaMemcpyHostToDevice,
                                       stream_));
            raw_input = buffers_.d_stage_raw;
        }

        // Stage 3: int16 原始 IQ -> float2 复数立方体，并施加慢时间窗
        launch_unpack_and_window(stream_,
                                 raw_input,
                                 buffers_.d_cube,
                                 cfg_.num_channels,
                                 cfg_.num_chirps,
                                 cfg_.num_samples);

        // Stage 4: 对每个通道分别做多普勒 FFT（沿 chirp 维）
        for (int ch = 0; ch < cfg_.num_channels; ++ch)
        {
            float2 *ch_ptr = buffers_.d_cube + ch * plane;

            CUFFT_CHECK(cufftExecC2C(
                fft_plan_.handle,
                reinterpret_cast<cufftComplex *>(ch_ptr),
                reinterpret_cast<cufftComplex *>(ch_ptr),
                CUFFT_FORWARD));
        }

        // Stage 5: 仅用和通道生成功率图（后续 CFAR 在这张图上做）
        launch_sum_channel_power(stream_,
                                 buffers_.d_cube,
                                 buffers_.d_power_map,
                                 chmap_.sum_primary,
                                 plane);

        // Stage 6: 调试导出功率图，便于和 Matlab 逐帧对齐
        {
            std::vector<float> h_power_map(cfg_.rd_map_elements());

            CUDA_CHECK(cudaMemcpyAsync(h_power_map.data(),
                                       buffers_.d_power_map,
                                       cfg_.rd_map_bytes(),
                                       cudaMemcpyDeviceToHost,
                                       stream_));

            CUDA_CHECK(cudaStreamSynchronize(stream_));

            dump_power_map_to_file(h_power_map.data(),
                                   cfg_.num_chirps,
                                   cfg_.num_samples,
                                   s_dump_frame_index);
            ++s_dump_frame_index;
        }

        // Stage 7: CFAR 命中 + NMS 峰值提取（得到目标候选 bin）
        launch_cfar_and_peak_extract_v2(stream_,
                                        buffers_.d_power_map,
                                        buffers_.d_hit_map,
                                        buffers_.d_noise_map,
                                        buffers_.d_threshold_map,
                                        buffers_.d_detections,
                                        buffers_.d_hit_count,
                                        buffers_.d_peak_count,
                                        cfg_.num_samples,
                                        cfg_.num_chirps,
                                        cfg_.cfar_train_x,
                                        cfg_.cfar_train_y,
                                        cfg_.cfar_guard_x,
                                        cfg_.cfar_guard_y,
                                        cfg_.cfar_pfa,
                                        cfg_.cfar_min_noise,
                                        cfg_.cfar_min_cut_power,
                                        cfg_.cfar_near_range_suppress_bins,
                                        cfg_.cfar_zero_doppler_suppress_bins,
                                        cfg_.cfar_peak_half_window,
                                        cfg_.cfar_peak_min_snr_db,
                                        cfg_.max_targets);

        // Stage 8: 从候选 bin 回查三通道复数数据，计算距离/速度/角度/SNR
        launch_monopulse(stream_,
                         buffers_.d_cube,
                         buffers_.d_detections,
                         buffers_.d_targets,
                         buffers_.d_peak_count,
                         cfg_.max_targets,
                         cfg_,
                         chmap_,
                         calib_);

        // Stage 9: 拷回命中数/峰值数，供日志和后处理使用
        CUDA_CHECK(cudaMemcpyAsync(&h_hit_count_,
                                   buffers_.d_hit_count,
                                   sizeof(int),
                                   cudaMemcpyDeviceToHost,
                                   stream_));

        CUDA_CHECK(cudaMemcpyAsync(&h_peak_count_,
                                   buffers_.d_peak_count,
                                   sizeof(int),
                                   cudaMemcpyDeviceToHost,
                                   stream_));

        // Stage 10: 同步一帧计算完成
        CUDA_CHECK(cudaStreamSynchronize(stream_));

        std::cout << "[CFAR] hits=" << h_hit_count_
                  << " peaks=" << h_peak_count_
                  << std::endl;

        const int valid = (h_peak_count_ > cfg_.max_targets)
                              ? cfg_.max_targets
                              : h_peak_count_;

        std::vector<RadarTarget> raw_targets(valid);

        // Stage 11: 拷回候选目标数组
        if (valid > 0)
        {
            CUDA_CHECK(cudaMemcpy(raw_targets.data(),
                                  buffers_.d_targets,
                                  valid * sizeof(RadarTarget),
                                  cudaMemcpyDeviceToHost));
        }

        // Stage 12: 主机侧基础过滤（距离/SNR/最大速度）
        out_targets.clear();
        out_targets.reserve(valid);

        for (const auto &t : raw_targets)
        {
            if (t.range_m < cfg_.post_min_range_m)
                continue;

            if (t.snr_db < cfg_.post_min_snr_db)
                continue;

            if (std::fabs(t.velocity_mps) > cfg_.post_max_abs_vel_mps)
                continue;

            out_targets.push_back(t);
        }

        // Stage 13: DBSCAN 聚类（减少同一目标附近的重复点）
        if (cfg_.dbscan_enable && !out_targets.empty())
        {
            const int before_count = static_cast<int>(out_targets.size());
            std::vector<RadarTarget> clustered_targets;
            const int cluster_count = cluster_targets_dbscan(out_targets, cfg_, clustered_targets);
            out_targets.swap(clustered_targets);
            std::cout << "[DBSCAN] in=" << before_count
                      << " clusters=" << cluster_count
                      << " out=" << out_targets.size()
                      << std::endl;
        }

        // Stage 14: 同一 Doppler bin 线状杂波抑制（可选）
        if (cfg_.post_doppler_line_suppress_enable && !out_targets.empty())
        {
            const int before_count = static_cast<int>(out_targets.size());
            std::vector<RadarTarget> doppler_suppressed_targets;
            const auto stats = suppress_doppler_line_targets(
                out_targets, cfg_, doppler_suppressed_targets);
            out_targets.swap(doppler_suppressed_targets);
            if (stats.suppressed_points > 0)
            {
                std::cout << "[DOPPLER] in=" << before_count
                          << " suppressed_bins=" << stats.suppressed_bins
                          << " out=" << out_targets.size()
                          << std::endl;
            }
        }

        // Stage 15: 可选仅保留最强 top-k（0 表示不截断）
        if (cfg_.post_top_k > 0 &&
            static_cast<int>(out_targets.size()) > cfg_.post_top_k)
        {
            const auto cmp = [](const RadarTarget &a, const RadarTarget &b)
            {
                if (a.snr_db != b.snr_db)
                    return a.snr_db > b.snr_db;
                return a.power > b.power;
            };
            std::partial_sort(out_targets.begin(),
                              out_targets.begin() + cfg_.post_top_k,
                              out_targets.end(),
                              cmp);
            out_targets.resize(cfg_.post_top_k);
        }

        // Stage 16: 单目标模式（用于单目标数据集），将输出收敛为 0/1 个目标
        if (cfg_.single_target_mode)
        {
            if (out_targets.empty())
            {
                single_track_missed_frames_++;
                if (single_track_missed_frames_ > cfg_.single_track_max_missed_frames)
                {
                    single_track_initialized_ = false;
                }
                return 0;
            }

            float frame_snr_max = out_targets[0].snr_db;
            for (int i = 1; i < static_cast<int>(out_targets.size()); ++i)
            {
                frame_snr_max = std::max(frame_snr_max, out_targets[i].snr_db);
            }
            const bool allow_soft_reseed =
                single_track_initialized_ &&
                (single_track_last_.snr_db + cfg_.single_track_snr_near_max_db < frame_snr_max);

            int best_idx = -1;
            float best_score = -std::numeric_limits<float>::infinity();
            const float dt = std::max(cfg_.single_track_frame_dt_s, 1e-3f);

            for (int i = 0; i < static_cast<int>(out_targets.size()); ++i)
            {
                const auto &t = out_targets[i];
                if (t.snr_db < frame_snr_max - cfg_.single_track_snr_near_max_db)
                {
                    continue;
                }

                float dr = 0.0f;
                float dv = 0.0f;
                float dv_kin = 0.0f;
                float range_penalty = cfg_.single_track_range_penalty;
                float velocity_penalty = cfg_.single_track_velocity_penalty;
                float kinematic_penalty = cfg_.single_track_kinematic_penalty;
                if (single_track_initialized_)
                {
                    dr = std::fabs(t.range_m - single_track_last_.range_m);
                    dv = std::fabs(t.velocity_mps - single_track_last_.velocity_mps);
                    if (!allow_soft_reseed &&
                        (dr > cfg_.single_track_gate_range_m ||
                         dv > cfg_.single_track_gate_velocity_mps))
                    {
                        continue;
                    }

                    const float v_from_range =
                        (t.range_m - single_track_last_.range_m) / dt;
                    dv_kin = std::fabs(t.velocity_mps - v_from_range);
                    if (!allow_soft_reseed &&
                        dv_kin > cfg_.single_track_kinematic_mismatch_mps)
                    {
                        continue;
                    }

                    if (allow_soft_reseed)
                    {
                        range_penalty *= 0.25f;
                        velocity_penalty *= 0.1f;
                        kinematic_penalty = 0.0f;
                    }
                }

                const float score =
                    t.snr_db -
                    range_penalty * dr -
                    velocity_penalty * dv -
                    kinematic_penalty * dv_kin -
                    cfg_.single_track_abs_velocity_penalty * std::fabs(t.velocity_mps);

                if (score > best_score)
                {
                    best_score = score;
                    best_idx = i;
                }
            }

            if (best_idx < 0)
            {
                if (single_track_initialized_)
                {
                    int reacquire_idx = -1;
                    float best_reacquire_cost = std::numeric_limits<float>::infinity();
                    for (int i = 0; i < static_cast<int>(out_targets.size()); ++i)
                    {
                        const auto &t = out_targets[i];
                        if (t.snr_db < frame_snr_max - cfg_.single_track_snr_near_max_db)
                        {
                            continue;
                        }

                        const float dr = std::fabs(t.range_m - single_track_last_.range_m);
                        const float dv = std::fabs(t.velocity_mps - single_track_last_.velocity_mps);
                        if (dv > cfg_.single_track_reacquire_max_velocity_jump_mps)
                        {
                            continue;
                        }

                        const float reacquire_cost = dv + 0.2f * dr;
                        if (reacquire_cost < best_reacquire_cost)
                        {
                            best_reacquire_cost = reacquire_cost;
                            reacquire_idx = i;
                        }
                    }

                    if (reacquire_idx >= 0)
                    {
                        best_idx = reacquire_idx;
                    }
                    else if (single_track_missed_frames_ < cfg_.single_track_max_missed_frames)
                    {
                        single_track_missed_frames_++;
                        out_targets.clear();
                        return 0;
                    }
                }

                if (best_idx < 0)
                {
                    best_idx = 0;
                    for (int i = 1; i < static_cast<int>(out_targets.size()); ++i)
                    {
                        if (out_targets[i].snr_db > out_targets[best_idx].snr_db ||
                            (out_targets[i].snr_db == out_targets[best_idx].snr_db &&
                             out_targets[i].power > out_targets[best_idx].power))
                        {
                            best_idx = i;
                        }
                    }
                }
            }

            const RadarTarget selected = out_targets[best_idx];
            out_targets.clear();
            out_targets.push_back(selected);

            single_track_last_ = selected;
            single_track_initialized_ = true;
            single_track_missed_frames_ = 0;
        }

        return static_cast<int>(out_targets.size());
    }

} // namespace radar
