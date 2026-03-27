#include <atomic>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <cstdlib>
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
#include "io/FrameAssembler.h"
#include "io/FrameRuntimeConfigBuilder.h"
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

    struct ReplayOptions
    {
        std::string input_path = "build/output_first_900MB.dat";
        std::string track_csv = "raw_packet_track.csv";
        std::string det_csv = "raw_packet_det.csv";
        TuneProfile profile = TuneProfile::SingleFriendly;
        std::size_t max_frames = 0;
        std::size_t payload_bytes = 1464;
        bool verbose_frame_logs = true;
        bool has_cfar_peak_snr_override = false;
        float cfar_peak_snr_override = 0.0f;
        bool has_post_min_snr_override = false;
        float post_min_snr_override = 0.0f;
        bool enable_power_stats = false;
        bool enable_power_dump = false;
        bool filter_beam_raw = false;
        uint16_t beam_az_raw = 1000;
        uint16_t beam_el_raw = 1000;
    };

    void apply_raw_packet_replay_relaxations(radar::RadarConfig &cfg, radar::TuneProfile profile)
    {
        if (profile != TuneProfile::MultiTarget)
        {
            return;
        }

        // 原始抓包离线验证先用更宽松的一档，方便确认协议接入后是否能出点。
        cfg.cfar_peak_min_snr_db = 8.0f;
        cfg.post_min_snr_db = 6.0f;
        cfg.tracking_spawn_min_snr_db = 8.0f;
    }

    ReplayOptions parse_options(int argc, char **argv)
    {
        ReplayOptions opt;

        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (arg == "--file" && i + 1 < argc)
            {
                opt.input_path = argv[++i];
            }
            else if (arg.rfind("--file=", 0) == 0)
            {
                opt.input_path = arg.substr(7);
            }
            else if (arg == "--track-csv" && i + 1 < argc)
            {
                opt.track_csv = argv[++i];
            }
            else if (arg.rfind("--track-csv=", 0) == 0)
            {
                opt.track_csv = arg.substr(12);
            }
            else if (arg == "--det-csv" && i + 1 < argc)
            {
                opt.det_csv = argv[++i];
            }
            else if (arg.rfind("--det-csv=", 0) == 0)
            {
                opt.det_csv = arg.substr(10);
            }
            else if (arg == "--max-frames" && i + 1 < argc)
            {
                opt.max_frames = static_cast<std::size_t>(std::stoull(argv[++i]));
            }
            else if (arg.rfind("--max-frames=", 0) == 0)
            {
                opt.max_frames = static_cast<std::size_t>(std::stoull(arg.substr(13)));
            }
            else if (arg == "--payload-bytes" && i + 1 < argc)
            {
                opt.payload_bytes = static_cast<std::size_t>(std::stoull(argv[++i]));
            }
            else if (arg.rfind("--payload-bytes=", 0) == 0)
            {
                opt.payload_bytes = static_cast<std::size_t>(std::stoull(arg.substr(16)));
            }
            else if (arg == "--profile" && i + 1 < argc)
            {
                const std::string value = argv[++i];
                if (value == "single")
                {
                    opt.profile = TuneProfile::SingleFriendly;
                }
                else if (value == "multi")
                {
                    opt.profile = TuneProfile::MultiTarget;
                }
                else
                {
                    throw std::invalid_argument("unknown --profile value: " + value + " (expected single|multi)");
                }
            }
            else if (arg.rfind("--profile=", 0) == 0)
            {
                const std::string value = arg.substr(10);
                if (value == "single")
                {
                    opt.profile = TuneProfile::SingleFriendly;
                }
                else if (value == "multi")
                {
                    opt.profile = TuneProfile::MultiTarget;
                }
                else
                {
                    throw std::invalid_argument("unknown --profile value: " + value + " (expected single|multi)");
                }
            }
            else if (arg == "--quiet")
            {
                opt.verbose_frame_logs = false;
            }
            else if (arg == "--power-stats")
            {
                opt.enable_power_stats = true;
            }
            else if (arg == "--dump-power-map")
            {
                opt.enable_power_dump = true;
            }
            else if (arg == "--only-zero-beam")
            {
                opt.filter_beam_raw = true;
                opt.beam_az_raw = 1000;
                opt.beam_el_raw = 1000;
            }
            else if (arg == "--beam-az-raw" && i + 1 < argc)
            {
                opt.filter_beam_raw = true;
                opt.beam_az_raw = static_cast<uint16_t>(std::stoul(argv[++i]));
            }
            else if (arg.rfind("--beam-az-raw=", 0) == 0)
            {
                opt.filter_beam_raw = true;
                opt.beam_az_raw = static_cast<uint16_t>(std::stoul(arg.substr(14)));
            }
            else if (arg == "--beam-el-raw" && i + 1 < argc)
            {
                opt.filter_beam_raw = true;
                opt.beam_el_raw = static_cast<uint16_t>(std::stoul(argv[++i]));
            }
            else if (arg.rfind("--beam-el-raw=", 0) == 0)
            {
                opt.filter_beam_raw = true;
                opt.beam_el_raw = static_cast<uint16_t>(std::stoul(arg.substr(14)));
            }
            else if (arg == "--cfar-peak-snr-db" && i + 1 < argc)
            {
                opt.has_cfar_peak_snr_override = true;
                opt.cfar_peak_snr_override = std::stof(argv[++i]);
            }
            else if (arg.rfind("--cfar-peak-snr-db=", 0) == 0)
            {
                opt.has_cfar_peak_snr_override = true;
                opt.cfar_peak_snr_override = std::stof(arg.substr(20));
            }
            else if (arg == "--post-min-snr-db" && i + 1 < argc)
            {
                opt.has_post_min_snr_override = true;
                opt.post_min_snr_override = std::stof(argv[++i]);
            }
            else if (arg.rfind("--post-min-snr-db=", 0) == 0)
            {
                opt.has_post_min_snr_override = true;
                opt.post_min_snr_override = std::stof(arg.substr(18));
            }
            else if (arg == "--help" || arg == "-h")
            {
                std::cout << "Usage: ./raw_packet_replay [--file PATH] [--profile single|multi]\n";
                std::cout << "                           [--max-frames N] [--payload-bytes N]\n";
                std::cout << "                           [--track-csv PATH] [--det-csv PATH] [--quiet]\n";
                std::cout << "                           [--power-stats] [--dump-power-map]\n";
                std::cout << "                           [--only-zero-beam | --beam-az-raw N --beam-el-raw N]\n";
                std::cout << "                           [--cfar-peak-snr-db X]\n";
                std::cout << "                           [--post-min-snr-db X]\n";
                std::exit(0);
            }
            else
            {
                throw std::invalid_argument("unknown argument: " + arg);
            }
        }

        return opt;
    }
} // namespace

int main(int argc, char **argv)
{
    using namespace radar;

    std::signal(SIGINT, on_sigint);

    const ReplayOptions opt = parse_options(argc, argv);

    RadarConfig cfg;
    cfg.udp_payload_bytes = static_cast<int>(opt.payload_bytes);
    cfg.verbose_frame_logs = opt.verbose_frame_logs;
    cfg.debug_print_power_stats = opt.enable_power_stats;
    cfg.debug_dump_power_map = opt.enable_power_dump;
    apply_tune_profile(cfg, opt.profile);
    apply_raw_packet_replay_relaxations(cfg, opt.profile);

    if (opt.has_cfar_peak_snr_override)
    {
        cfg.cfar_peak_min_snr_db = opt.cfar_peak_snr_override;
    }
    if (opt.has_post_min_snr_override)
    {
        cfg.post_min_snr_db = opt.post_min_snr_override;
    }

    ChannelMap chmap = make_default_channel_map();
    MonopulseCalibration calib;
    RuntimeMetrics metrics;

    try
    {
        std::ifstream input(opt.input_path, std::ios::binary);
        if (!input)
        {
            throw std::runtime_error("failed to open input file: " + opt.input_path);
        }

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
        MultiTargetTracker tracker(cfg);
        FrameAssembler assembler(cfg);

        std::ofstream track_ofs(opt.track_csv, std::ios::out | std::ios::trunc);
        if (!track_ofs)
        {
            throw std::runtime_error("failed to open track csv: " + opt.track_csv);
        }
        track_ofs << "frame_idx,frame_id,track_id,confirmed,age,hits,missed,rbin,dbin_c,dbin_u,range_m,vel_mps,snr_db,az_deg,el_deg,az_err,el_err,power\n";
        track_ofs << std::fixed << std::setprecision(6);

        std::ofstream det_ofs(opt.det_csv, std::ios::out | std::ios::trunc);
        if (!det_ofs)
        {
            throw std::runtime_error("failed to open detection csv: " + opt.det_csv);
        }
        det_ofs << "det_uid,frame_idx,frame_id,det_idx,beam_az_raw,beam_el_raw,beam_az_deg,beam_el_deg,"
                   "rel_az_deg,rel_el_deg,abs_az_deg,abs_el_deg,valid_az,valid_el,"
                   "rbin,dbin_c,dbin_u,range_m,vel_mps,snr_db,power,az_err,el_err\n";
        det_ofs << std::fixed << std::setprecision(6);

        std::vector<uint8_t> packet(cfg.udp_payload_bytes);
        std::size_t processed_frames = 0;
        std::size_t packet_index = 0;
        std::size_t invalid_packets = 0;
        std::size_t duplicate_packets = 0;
        std::size_t filtered_beam_frames = 0;
        std::size_t det_uid = 0;

        Logger::info("Raw packet replay initialized.");
        Logger::info("  profile = ", tune_profile_name(opt.profile));
        Logger::info("  input_file = ", opt.input_path);
        Logger::info("  udp_payload_bytes = ", cfg.udp_payload_bytes);
        Logger::info("  protocol_prt_range = [", cfg.protocol_prt_start_num, ", ",
                     cfg.protocol_prt_last_num(), "]");
        Logger::info("  active_processing_prts = ", cfg.num_chirps);
        Logger::info("  cfar_peak_min_snr_db = ", cfg.cfar_peak_min_snr_db);
        Logger::info("  post_min_snr_db = ", cfg.post_min_snr_db);
        Logger::info("  debug_power_stats = ", cfg.debug_print_power_stats ? "on" : "off");
        Logger::info("  debug_power_dump = ", cfg.debug_dump_power_map ? "on" : "off");
        if (opt.filter_beam_raw)
        {
            Logger::info("  beam filter raw    = (", opt.beam_az_raw, ", ", opt.beam_el_raw, ")");
        }

        while (g_running && input)
        {
            input.read(reinterpret_cast<char *>(packet.data()),
                       static_cast<std::streamsize>(packet.size()));
            const std::size_t got = static_cast<std::size_t>(input.gcount());
            if (got == 0)
            {
                break;
            }

            ++packet_index;
            const AssemblyStatus st = assembler.push_packet(packet.data(),
                                                            got,
                                                            h_mapped,
                                                            cfg.frame_payload_bytes());
            if (st.packet_invalid)
            {
                ++invalid_packets;
                continue;
            }
            if (st.packet_duplicate)
            {
                ++duplicate_packets;
                continue;
            }

            if (!st.frame_completed)
            {
                continue;
            }

            const PrtHeaderInfo &frame_header = assembler.current_frame_header();
            if (opt.filter_beam_raw &&
                (frame_header.radar_angle_a_raw() != opt.beam_az_raw ||
                 frame_header.radar_angle_e_raw() != opt.beam_el_raw))
            {
                ++filtered_beam_frames;
                continue;
            }

            const FrameRuntimeConfig runtime =
                build_frame_runtime_config(assembler.current_frame_id(),
                                           frame_header);

            std::atomic_thread_fence(std::memory_order_seq_cst);

            std::vector<RadarTarget> detections;
            const int detection_count = pipeline.process_frame(d_mapped, detections, &runtime);
            metrics.gpu_frames_processed.fetch_add(1, std::memory_order_relaxed);
            metrics.gpu_targets_reported.fetch_add(static_cast<uint64_t>(detection_count), std::memory_order_relaxed);

            const float beam_az_deg = runtime.has_beam_azimuth ? runtime.beam_azimuth_deg : std::numeric_limits<float>::quiet_NaN();
            const float beam_el_deg = runtime.has_beam_elevation ? runtime.beam_elevation_deg : std::numeric_limits<float>::quiet_NaN();
            const int beam_az_raw = runtime.has_beam_azimuth ? static_cast<int>(frame_header.radar_angle_a_raw()) : -1;
            const int beam_el_raw = runtime.has_beam_elevation ? static_cast<int>(frame_header.radar_angle_e_raw()) : -1;

            for (int i = 0; i < detection_count; ++i)
            {
                const auto &t = detections[static_cast<std::size_t>(i)];
                const int rbin = static_cast<int>(std::lround(t.range_m / cfg.range_resolution_m));
                const int dbin_c = static_cast<int>(std::lround(t.velocity_mps / cfg.velocity_resolution_mps));
                int dbin_u = dbin_c;
                if (dbin_u < 0)
                {
                    dbin_u += cfg.num_chirps;
                }

                const float rel_az_deg =
                    (runtime.has_beam_azimuth && t.valid_az) ? (t.azimuth_deg - runtime.beam_azimuth_deg)
                                                              : std::numeric_limits<float>::quiet_NaN();
                const float rel_el_deg =
                    (runtime.has_beam_elevation && t.valid_el) ? (t.elevation_deg - runtime.beam_elevation_deg)
                                                                : std::numeric_limits<float>::quiet_NaN();

                det_ofs << det_uid++ << ","
                        << processed_frames << ","
                        << runtime.frame_id << ","
                        << i << ","
                        << beam_az_raw << ","
                        << beam_el_raw << ","
                        << beam_az_deg << ","
                        << beam_el_deg << ","
                        << rel_az_deg << ","
                        << rel_el_deg << ","
                        << t.azimuth_deg << ","
                        << t.elevation_deg << ","
                        << static_cast<int>(t.valid_az) << ","
                        << static_cast<int>(t.valid_el) << ","
                        << rbin << ","
                        << dbin_c << ","
                        << dbin_u << ","
                        << t.range_m << ","
                        << t.velocity_mps << ","
                        << t.snr_db << ","
                        << t.power << ","
                        << t.az_error << ","
                        << t.el_error << "\n";
            }

            std::vector<TrackedTarget> tracks;
            int track_count = 0;
            if (cfg.tracking_enable)
            {
                track_count = tracker.update(detections, tracks);
            }

            if (cfg.verbose_frame_logs)
            {
                std::cout << "[Raw Frame " << processed_frames
                          << "] frame_id=" << runtime.frame_id
                          << " prt_global=" << runtime.global_prt_count
                          << " det=" << detection_count
                          << " tracks=" << track_count;
                if (runtime.has_beam_azimuth || runtime.has_beam_elevation)
                {
                    std::cout << " beam=("
                              << (runtime.has_beam_azimuth ? std::to_string(runtime.beam_azimuth_deg) : std::string("nan"))
                              << ","
                              << (runtime.has_beam_elevation ? std::to_string(runtime.beam_elevation_deg) : std::string("nan"))
                              << ")";
                }
                std::cout << std::endl;
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

                track_ofs << processed_frames << ","
                          << runtime.frame_id << ","
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
                track_ofs << processed_frames << ","
                          << runtime.frame_id
                          << ",-1,0,0,0,0,-1,-9999,-1,nan,nan,nan,nan,nan,nan,nan,nan\n";
            }

            ++processed_frames;
            if (opt.max_frames != 0 && processed_frames >= opt.max_frames)
            {
                break;
            }
        }

        pipeline.print_signal_timing_summary();

        if (h_mapped)
        {
            cudaFreeHost(h_mapped);
            h_mapped = nullptr;
            d_mapped = nullptr;
        }

        Logger::info("raw packet replay done");
        Logger::info("  processed_frames = ", processed_frames);
        Logger::info("  udp_chunks_read = ", packet_index);
        Logger::info("  invalid_chunks = ", invalid_packets);
        Logger::info("  duplicate_chunks = ", duplicate_packets);
        Logger::info("  filtered_beam_frames = ", filtered_beam_frames);
        Logger::info("  track_csv = ", opt.track_csv);
        Logger::info("  det_csv = ", opt.det_csv);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Fatal: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
