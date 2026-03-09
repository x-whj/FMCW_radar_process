#include <atomic>
#include <csignal>
#include <cstring>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>

#include <cuda_runtime.h>

#include "ChebWindow256.h"
#include "gpu/RadarPipeline.h"
#include "io/OfflineReplay.h"
#include "model/Calibration.h"
#include "model/ChannelMap.h"
#include "model/RadarConfig.h"
#include "runtime/Logger.h"
#include "runtime/Metrics.h"

namespace
{
    std::atomic<bool> g_running{true};

    void on_sigint(int)
    {
        g_running = false;
    }

    struct PrimaryTrackState
    {
        bool initialized = false;
        float range_m = 0.0f;
        float velocity_mps = 0.0f;
    };

    int select_primary_target_idx(const std::vector<radar::RadarTarget> &targets,
                                  PrimaryTrackState &state)
    {
        if (targets.empty())
        {
            state.initialized = false;
            return -1;
        }

        if (!state.initialized)
        {
            int best = 0;
            for (int i = 1; i < static_cast<int>(targets.size()); ++i)
            {
                if (targets[i].snr_db > targets[best].snr_db)
                {
                    best = i;
                }
            }
            state.initialized = true;
            state.range_m = targets[best].range_m;
            state.velocity_mps = targets[best].velocity_mps;
            return best;
        }

        constexpr float kGateRangeM = 1.5f;
        constexpr float kGateVelMps = 6.0f;
        constexpr float kRangePenalty = 2.0f;
        constexpr float kVelPenalty = 0.7f;
        constexpr float kAbsVelPenalty = 0.15f;
        constexpr float kSnrNearMaxDb = 3.5f;

        float snr_max = targets[0].snr_db;
        for (int i = 1; i < static_cast<int>(targets.size()); ++i)
        {
            if (targets[i].snr_db > snr_max)
            {
                snr_max = targets[i].snr_db;
            }
        }

        int best = -1;
        float best_score = -std::numeric_limits<float>::infinity();

        for (int i = 0; i < static_cast<int>(targets.size()); ++i)
        {
            const auto &t = targets[i];
            if (t.snr_db < snr_max - kSnrNearMaxDb)
            {
                continue;
            }

            const float dr = std::fabs(t.range_m - state.range_m);
            const float dv = std::fabs(t.velocity_mps - state.velocity_mps);

            if (dr > kGateRangeM || dv > kGateVelMps)
            {
                continue;
            }

            const float score = t.snr_db - kRangePenalty * dr - kVelPenalty * dv - kAbsVelPenalty * std::fabs(t.velocity_mps);
            if (score > best_score)
            {
                best_score = score;
                best = i;
            }
        }

        // Gate miss fallback: relock to strongest SNR.
        if (best < 0)
        {
            best = 0;
            for (int i = 1; i < static_cast<int>(targets.size()); ++i)
            {
                if (targets[i].snr_db > targets[best].snr_db)
                {
                    best = i;
                }
            }
        }

        state.range_m = targets[best].range_m;
        state.velocity_mps = targets[best].velocity_mps;
        return best;
    }
} // namespace

int main()
{
    using namespace radar;

    std::signal(SIGINT, on_sigint);

    RadarConfig cfg;
    // Single-target friendly defaults for offline verification.
    cfg.cfar_pfa = 1e-6f;
    cfg.cfar_peak_half_window = 3;          // 7x7 NMS
    cfg.cfar_peak_min_snr_db = 8.0f;
    cfg.cfar_zero_doppler_suppress_bins = 0;
    cfg.post_top_k = 0; // keep all CFAR candidates for debugging
    cfg.post_min_snr_db = 6.0f;
    cfg.post_min_range_m = 0.0f;
    cfg.post_max_abs_vel_mps = 60.0f;
    cfg.dbscan_enable = true;
    cfg.dbscan_eps_range_m = 0.45f;
    cfg.dbscan_eps_velocity_mps = 1.2f;
    cfg.dbscan_min_points = 2;
    cfg.dbscan_keep_noise = true;
    cfg.dbscan_max_clusters = 0;
    cfg.single_target_mode = true;
    cfg.single_track_gate_range_m = 1.5f;
    cfg.single_track_gate_velocity_mps = 4.0f;
    cfg.single_track_snr_near_max_db = 2.5f;
    cfg.single_track_frame_dt_s = 0.1f;
    cfg.single_track_kinematic_mismatch_mps = 6.0f;
    cfg.single_track_reacquire_max_velocity_jump_mps = 7.0f;
    ChannelMap chmap;
    MonopulseCalibration calib;
    RuntimeMetrics metrics;

    // 已确认的三通道语义
    chmap.sum_primary = 0; // output_he.dat
    chmap.diff_az = 1;     // output_cha1.dat
    chmap.diff_el = 2;     // output_cha2.dat
    chmap.has_diff_az = true;
    chmap.has_diff_el = true;

    // 如果你想先只跑前几帧，就改这里
    // 设为 0 表示跑全部帧
    const std::size_t max_frames_limit = 0;

    // 离线文件路径
    const std::string sum_file = "output_he.dat";
    const std::string az_file = "output_cha1.dat";
    const std::string el_file = "output_cha2.dat";
    const std::string primary_track_csv = "offline_primary_track.csv";

    try
    {
        CUDA_CHECK(cudaSetDeviceFlags(cudaDeviceMapHost));

        // 一块输入缓冲区就够了：每次把当前帧 memcpy 进去，再交给 pipeline
        uint8_t *h_mapped = nullptr;
        int16_t *d_mapped = nullptr;

        CUDA_CHECK(cudaHostAlloc(reinterpret_cast<void **>(&h_mapped),
                                 cfg.frame_payload_bytes(),
                                 cudaHostAllocMapped));

        CUDA_CHECK(cudaHostGetDevicePointer(reinterpret_cast<void **>(&d_mapped),
                                            h_mapped,
                                            0));

        RadarPipeline pipeline(cfg, chmap, calib);
        pipeline.initialize(CHEB_WINDOW_256);

        OfflineReplay replay(cfg, sum_file, az_file, el_file);

        Logger::info("Offline replay initialized.");
        Logger::info("  sum_file = ", sum_file);
        Logger::info("  az_file  = ", az_file);
        Logger::info("  el_file  = ", el_file);
        Logger::info("  total_frames = ", replay.total_frames());

        std::ofstream primary_track_ofs(primary_track_csv, std::ios::out | std::ios::trunc);
        if (!primary_track_ofs)
        {
            throw std::runtime_error("failed to open primary track csv: " + primary_track_csv);
        }
        primary_track_ofs << "frame,valid,targets,primary_idx,rbin,dbin_c,dbin_u,range_m,vel_mps,snr_db,az_deg,el_deg\n";
        primary_track_ofs << std::fixed << std::setprecision(6);

        std::vector<int16_t> frame_raw;
        const std::size_t total_frames =
            (max_frames_limit == 0 || max_frames_limit > replay.total_frames())
                ? replay.total_frames()
                : max_frames_limit;
        PrimaryTrackState primary_track;

        for (std::size_t frame_idx = 0; frame_idx < total_frames && g_running; ++frame_idx)
        {
            // 从 3 个文件中抽出第 frame_idx 帧，重新拼成：
            // [sum_I, sum_Q, az_I, az_Q, el_I, el_Q]
            replay.build_frame_payload(frame_idx, frame_raw);

            // 拷到 mapped host buffer
            std::memcpy(h_mapped, frame_raw.data(), cfg.frame_payload_bytes());
            std::atomic_thread_fence(std::memory_order_seq_cst);

            std::vector<RadarTarget> targets;
            const int count = pipeline.process_frame(d_mapped, targets);

            metrics.gpu_frames_processed.fetch_add(1, std::memory_order_relaxed);
            metrics.gpu_targets_reported.fetch_add(static_cast<uint64_t>(count), std::memory_order_relaxed);

            std::cout << "[Offline Frame " << frame_idx << "] targets=" << count << std::endl;
            for (int i = 0; i < count; ++i)
            {
                const auto &t = targets[i];
                const int range_bin = static_cast<int>(std::lround(t.range_m / cfg.range_resolution_m));
                const int doppler_bin_centered = static_cast<int>(std::lround(t.velocity_mps / cfg.velocity_resolution_mps));
                int doppler_bin_unshifted = doppler_bin_centered;
                if (doppler_bin_unshifted < 0)
                {
                    doppler_bin_unshifted += cfg.num_chirps;
                }
                std::cout << "  idx=" << i
                          << " rbin=" << range_bin
                          << " dbin_c=" << doppler_bin_centered
                          << " dbin_u=" << doppler_bin_unshifted
                          << " range=" << t.range_m << " m"
                          << " vel=" << t.velocity_mps << " m/s"
                          << " az=" << t.azimuth_deg << " deg"
                          << " el=" << t.elevation_deg << " deg"
                          << " snr=" << t.snr_db << " dB"
                          << std::endl;
            }

            const int primary_idx = select_primary_target_idx(targets, primary_track);
            if (primary_idx >= 0)
            {
                const auto &p = targets[primary_idx];
                const int pr = static_cast<int>(std::lround(p.range_m / cfg.range_resolution_m));
                const int pdc = static_cast<int>(std::lround(p.velocity_mps / cfg.velocity_resolution_mps));
                int pdu = pdc;
                if (pdu < 0)
                {
                    pdu += cfg.num_chirps;
                }
                std::cout << "  [Primary] idx=" << primary_idx
                          << " rbin=" << pr
                          << " dbin_c=" << pdc
                          << " dbin_u=" << pdu
                          << " range=" << p.range_m << " m"
                          << " vel=" << p.velocity_mps << " m/s"
                          << " snr=" << p.snr_db << " dB"
                          << std::endl;

                primary_track_ofs << frame_idx << ",1," << count << "," << primary_idx << ","
                                  << pr << "," << pdc << "," << pdu << ","
                                  << p.range_m << "," << p.velocity_mps << "," << p.snr_db << ","
                                  << p.azimuth_deg << "," << p.elevation_deg << "\n";
            }
            else
            {
                std::cout << "  [Primary] none" << std::endl;
                primary_track_ofs << frame_idx << ",0," << count << ",-1,-1,-9999,-1,nan,nan,nan,nan,nan\n";
            }
        }

        if (h_mapped)
        {
            cudaFreeHost(h_mapped);
            h_mapped = nullptr;
            d_mapped = nullptr;
        }

        Logger::info("offline replay done");
        Logger::info("  primary_track_csv = ", primary_track_csv);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
