#include <atomic>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
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
#include "tracking/MultiTargetTracker.h"

namespace
{
    std::atomic<bool> g_running{true};

    void on_sigint(int)
    {
        g_running = false;
    }

    enum class TuneProfile
    {
        SingleFriendly,
        MultiTarget
    };

    void apply_single_friendly_profile(radar::RadarConfig &cfg)
    {
        cfg.cfar_pfa = 1e-6f;
        cfg.cfar_peak_half_window = 3;
        cfg.cfar_peak_min_snr_db = 14.0f;
        cfg.cfar_zero_doppler_suppress_bins = 0;
        cfg.post_top_k = 0;
        cfg.post_min_snr_db = 10.0f;
        cfg.post_min_range_m = 0.0f;
        cfg.post_max_abs_vel_mps = 60.0f;
        cfg.post_doppler_line_suppress_enable = true;
        cfg.post_doppler_line_min_points = 4;
        cfg.post_doppler_line_keep_per_bin = 1;

        cfg.dbscan_enable = true;
        cfg.dbscan_eps_range_m = 0.45f;
        cfg.dbscan_eps_velocity_mps = 1.2f;
        cfg.dbscan_min_points = 2;
        cfg.dbscan_keep_noise = true;
        cfg.dbscan_max_clusters = 0;

        cfg.single_target_mode = false;

        cfg.tracking_enable = true;
        cfg.tracking_frame_dt_s = 0.1f;
        cfg.tracking_gate_range_m = 0.6f;
        cfg.tracking_gate_velocity_mps = 1.0f;
        cfg.tracking_process_noise_range_m = 0.30f;
        cfg.tracking_process_noise_velocity_mps = 0.60f;
        cfg.tracking_measurement_noise_range_m = 0.20f;
        cfg.tracking_measurement_noise_velocity_mps = 0.40f;
        cfg.tracking_confirm_hits = 4;
        cfg.tracking_max_missed_frames = 2;
        cfg.tracking_output_tentative = false;
        cfg.tracking_output_only_updated_tracks = true;
        cfg.tracking_output_min_age = 4;
        cfg.tracking_output_min_hits = 4;
        cfg.tracking_max_tracks = 128;
        cfg.tracking_spawn_min_snr_db = 12.0f;
        cfg.tracking_spawn_exclusion_range_m = 0.8f;
        cfg.tracking_spawn_exclusion_velocity_mps = 1.2f;
    }

    void apply_multi_target_profile(radar::RadarConfig &cfg)
    {
        cfg.cfar_pfa = 1e-6f;
        cfg.cfar_peak_half_window = 2;
        cfg.cfar_peak_min_snr_db = 11.5f;
        cfg.cfar_zero_doppler_suppress_bins = 0;
        cfg.post_top_k = 0;
        cfg.post_min_snr_db = 8.0f;
        cfg.post_min_range_m = 0.0f;
        cfg.post_max_abs_vel_mps = 80.0f;
        cfg.post_doppler_line_suppress_enable = true;
        cfg.post_doppler_line_min_points = 5;
        cfg.post_doppler_line_keep_per_bin = 1;

        cfg.dbscan_enable = true;
        cfg.dbscan_eps_range_m = 0.35f;
        cfg.dbscan_eps_velocity_mps = 0.8f;
        cfg.dbscan_min_points = 2;
        cfg.dbscan_keep_noise = true;
        cfg.dbscan_max_clusters = 0;

        cfg.single_target_mode = false;

        cfg.tracking_enable = true;
        cfg.tracking_frame_dt_s = 0.1f;
        cfg.tracking_gate_range_m = 0.9f;
        cfg.tracking_gate_velocity_mps = 1.4f;
        cfg.tracking_process_noise_range_m = 0.45f;
        cfg.tracking_process_noise_velocity_mps = 0.90f;
        cfg.tracking_measurement_noise_range_m = 0.25f;
        cfg.tracking_measurement_noise_velocity_mps = 0.50f;
        cfg.tracking_confirm_hits = 3;
        cfg.tracking_max_missed_frames = 2;
        cfg.tracking_output_tentative = false;
        cfg.tracking_output_only_updated_tracks = true;
        cfg.tracking_output_min_age = 6;
        cfg.tracking_output_min_hits = 6;
        cfg.tracking_max_tracks = 128;
        cfg.tracking_spawn_min_snr_db = 11.0f;
        cfg.tracking_spawn_exclusion_range_m = 0.7f;
        cfg.tracking_spawn_exclusion_velocity_mps = 1.0f;
    }

    TuneProfile parse_profile(int argc, char **argv)
    {
        TuneProfile profile = TuneProfile::SingleFriendly;

        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (arg == "--profile=single")
            {
                profile = TuneProfile::SingleFriendly;
            }
            else if (arg == "--profile=multi")
            {
                profile = TuneProfile::MultiTarget;
            }
            else if (arg == "--profile" && i + 1 < argc)
            {
                const std::string value = argv[++i];
                if (value == "single")
                {
                    profile = TuneProfile::SingleFriendly;
                }
                else if (value == "multi")
                {
                    profile = TuneProfile::MultiTarget;
                }
                else
                {
                    throw std::invalid_argument("unknown --profile value: " + value + " (expected single|multi)");
                }
            }
            else if (arg == "--help" || arg == "-h")
            {
                std::cout << "Usage: ./radar_app [--profile single|multi]\n";
                std::cout << "  single: conservative settings for single-target-like data\n";
                std::cout << "  multi : more permissive settings for multi-target scenes\n";
                std::exit(0);
            }
            else
            {
                throw std::invalid_argument("unknown argument: " + arg);
            }
        }

        return profile;
    }
} // namespace

int main(int argc, char **argv)
{
    using namespace radar;

    std::signal(SIGINT, on_sigint);

    const TuneProfile profile = parse_profile(argc, argv);

    RadarConfig cfg;
    if (profile == TuneProfile::MultiTarget)
    {
        apply_multi_target_profile(cfg);
    }
    else
    {
        apply_single_friendly_profile(cfg);
    }

    ChannelMap chmap;
    MonopulseCalibration calib;
    RuntimeMetrics metrics;

    chmap.sum_primary = 0;
    chmap.diff_az = 1;
    chmap.diff_el = 2;
    chmap.has_diff_az = true;
    chmap.has_diff_el = true;

    const std::size_t max_frames_limit = 0;

    const std::string sum_file = "output_he.dat";
    const std::string az_file = "output_cha1.dat";
    const std::string el_file = "output_cha2.dat";
    const std::string track_csv = "offline_multi_track.csv";

    try
    {
        CUDA_CHECK(cudaSetDeviceFlags(cudaDeviceMapHost));

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
        MultiTargetTracker tracker(cfg);

        Logger::info("Offline replay initialized.");
        Logger::info("  profile = ", (profile == TuneProfile::MultiTarget) ? "multi" : "single");
        Logger::info("  sum_file = ", sum_file);
        Logger::info("  az_file  = ", az_file);
        Logger::info("  el_file  = ", el_file);
        Logger::info("  total_frames = ", replay.total_frames());
        Logger::info("  cfar_peak_min_snr_db = ", cfg.cfar_peak_min_snr_db);
        Logger::info("  post_min_snr_db = ", cfg.post_min_snr_db);
        Logger::info("  dbscan_min_points = ", cfg.dbscan_min_points);
        Logger::info("  tracking_confirm_hits = ", cfg.tracking_confirm_hits);
        Logger::info("  tracking_max_missed_frames = ", cfg.tracking_max_missed_frames);
        Logger::info("  tracking_output_min_age = ", cfg.tracking_output_min_age);
        Logger::info("  tracking_output_min_hits = ", cfg.tracking_output_min_hits);
        Logger::info("  tracking_spawn_min_snr_db = ", cfg.tracking_spawn_min_snr_db);

        std::ofstream track_ofs(track_csv, std::ios::out | std::ios::trunc);
        if (!track_ofs)
        {
            throw std::runtime_error("failed to open track csv: " + track_csv);
        }
        track_ofs << "frame,track_id,confirmed,age,hits,missed,rbin,dbin_c,dbin_u,range_m,vel_mps,snr_db,az_deg,el_deg,power\n";
        track_ofs << std::fixed << std::setprecision(6);

        std::vector<int16_t> frame_raw;
        const std::size_t total_frames =
            (max_frames_limit == 0 || max_frames_limit > replay.total_frames())
                ? replay.total_frames()
                : max_frames_limit;

        for (std::size_t frame_idx = 0; frame_idx < total_frames && g_running; ++frame_idx)
        {
            replay.build_frame_payload(frame_idx, frame_raw);

            std::memcpy(h_mapped, frame_raw.data(), cfg.frame_payload_bytes());
            std::atomic_thread_fence(std::memory_order_seq_cst);

            std::vector<RadarTarget> detections;
            const int detection_count = pipeline.process_frame(d_mapped, detections);

            metrics.gpu_frames_processed.fetch_add(1, std::memory_order_relaxed);
            metrics.gpu_targets_reported.fetch_add(static_cast<uint64_t>(detection_count), std::memory_order_relaxed);

            std::vector<TrackedTarget> tracks;
            int track_count = 0;
            if (cfg.tracking_enable)
            {
                track_count = tracker.update(detections, tracks);
            }

            std::cout << "[Offline Frame " << frame_idx << "] detections=" << detection_count
                      << " tracks=" << track_count << std::endl;

            for (int i = 0; i < detection_count; ++i)
            {
                const auto &t = detections[i];
                const int range_bin = static_cast<int>(std::lround(t.range_m / cfg.range_resolution_m));
                const int doppler_bin_centered = static_cast<int>(std::lround(t.velocity_mps / cfg.velocity_resolution_mps));
                int doppler_bin_unshifted = doppler_bin_centered;
                if (doppler_bin_unshifted < 0)
                {
                    doppler_bin_unshifted += cfg.num_chirps;
                }
                std::cout << "  [Det] idx=" << i
                          << " rbin=" << range_bin
                          << " dbin_c=" << doppler_bin_centered
                          << " dbin_u=" << doppler_bin_unshifted
                          << " range=" << t.range_m << " m"
                          << " vel=" << t.velocity_mps << " m/s"
                          << " snr=" << t.snr_db << " dB"
                          << std::endl;
            }

            for (int i = 0; i < track_count; ++i)
            {
                const auto &tr = tracks[i];
                const auto &t = tr.target;

                const int rbin = static_cast<int>(std::lround(t.range_m / cfg.range_resolution_m));
                const int dbin_c = static_cast<int>(std::lround(t.velocity_mps / cfg.velocity_resolution_mps));
                int dbin_u = dbin_c;
                if (dbin_u < 0)
                {
                    dbin_u += cfg.num_chirps;
                }

                std::cout << "  [Track] id=" << tr.track_id
                          << " conf=" << static_cast<int>(tr.confirmed)
                          << " age=" << tr.age
                          << " hits=" << tr.hit_count
                          << " miss=" << tr.missed_frames
                          << " rbin=" << rbin
                          << " dbin_c=" << dbin_c
                          << " range=" << t.range_m << " m"
                          << " vel=" << t.velocity_mps << " m/s"
                          << " snr=" << t.snr_db << " dB"
                          << std::endl;

                track_ofs << frame_idx << ","
                          << tr.track_id << ","
                          << static_cast<int>(tr.confirmed) << ","
                          << tr.age << ","
                          << tr.hit_count << ","
                          << tr.missed_frames << ","
                          << rbin << ","
                          << dbin_c << ","
                          << dbin_u << ","
                          << t.range_m << ","
                          << t.velocity_mps << ","
                          << t.snr_db << ","
                          << t.azimuth_deg << ","
                          << t.elevation_deg << ","
                          << t.power << "\n";
            }

            if (track_count == 0)
            {
                track_ofs << frame_idx << ",-1,0,0,0,0,-1,-9999,-1,nan,nan,nan,nan,nan,nan\n";
            }
        }

        if (h_mapped)
        {
            cudaFreeHost(h_mapped);
            h_mapped = nullptr;
            d_mapped = nullptr;
        }

        Logger::info("offline replay done");
        Logger::info("  multi_track_csv = ", track_csv);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
