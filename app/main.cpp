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

#include "app/TuneProfiles.h"
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
    using radar::TuneProfile;

    std::atomic<bool> g_running{true};

    void on_sigint(int)
    {
        g_running = false;
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
    apply_tune_profile(cfg, profile);

    ChannelMap chmap = make_default_channel_map();
    MonopulseCalibration calib;
    RuntimeMetrics metrics;

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
        Logger::info("  profile = ", tune_profile_name(profile));
        Logger::info("  input_mode = ", cfg.prefer_mapped_zero_copy ? "zero_copy_mapped_host" : "device_staging_copy");
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
        track_ofs << "frame,track_id,confirmed,age,hits,missed,rbin,dbin_c,dbin_u,range_m,vel_mps,snr_db,az_deg,el_deg,az_err,el_err,power\n";
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

            if (cfg.verbose_frame_logs)
            {
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
                }
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
                          << t.az_error << ","
                          << t.el_error << ","
                          << t.power << "\n";
            }

            if (track_count == 0)
            {
                track_ofs << frame_idx << ",-1,0,0,0,0,-1,-9999,-1,nan,nan,nan,nan,nan,nan,nan,nan\n";
            }
        }

        pipeline.print_signal_timing_summary();

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
