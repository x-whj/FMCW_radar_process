#pragma once

#include <cstddef>
#include <cstdint>

namespace radar
{

    struct RadarConfig
    {
       
        int num_channels = 3; // 1 sum + 1 az diff + 1 el diff
        int num_chirps = 256;
        int num_samples = 664;

        int cfar_window = 17; // 必须为奇数
        int cfar_guard = 2;
        float cfar_pfa = 1e-5f;             
        float cfar_threshold_scale = 12.0f; // 先保留经验值
        int max_targets = 256;

       
        int cfar_train_x = 8;
        int cfar_train_y = 6;
        int cfar_guard_x = 2;
        int cfar_guard_y = 2;
        int cfar_near_range_suppress_bins = 20;
        int cfar_zero_doppler_suppress_bins = 0;
        int cfar_peak_half_window = 2;   // 2 -> 5x5 NMS
        float cfar_min_noise = 1e-12f;
        float cfar_min_cut_power = 0.0f;
        float cfar_peak_min_snr_db = 12.0f;

       
        float post_min_range_m = 0.0f;
        float post_min_snr_db = 0.0f;
        float post_max_abs_vel_mps = 100.0f;
        int post_top_k = 0; // 0: keep all
      
        bool post_doppler_line_suppress_enable = false;
        int post_doppler_line_min_points = 6;
        int post_doppler_line_keep_per_bin = 1;

       
        bool dbscan_enable = true;
        float dbscan_eps_range_m = 0.45f;
        float dbscan_eps_velocity_mps = 1.2f;
        int dbscan_min_points = 2;
        bool dbscan_keep_noise = true;
        int dbscan_max_clusters = 0;

        bool tracking_enable = true;
        float tracking_frame_dt_s = 0.1f;
        float tracking_gate_range_m = 1.2f;
        float tracking_gate_velocity_mps = 2.0f;
        float tracking_process_noise_range_m = 0.30f;
        float tracking_process_noise_velocity_mps = 0.60f;
        float tracking_measurement_noise_range_m = 0.20f;
        float tracking_measurement_noise_velocity_mps = 0.40f;
        int tracking_confirm_hits = 3;
        int tracking_max_missed_frames = 5;
        bool tracking_output_tentative = false;
        bool tracking_output_only_updated_tracks = true;
        int tracking_output_min_age = 1;
        int tracking_output_min_hits = 1;
        int tracking_max_tracks = 128;
        float tracking_spawn_min_snr_db = 0.0f;
        float tracking_spawn_exclusion_range_m = 0.0f;
        float tracking_spawn_exclusion_velocity_mps = 0.0f;

       
        bool single_target_mode = true;
        float single_track_gate_range_m = 1.5f;
        float single_track_gate_velocity_mps = 6.0f;
        float single_track_snr_near_max_db = 3.5f;
        float single_track_range_penalty = 2.0f;
        float single_track_velocity_penalty = 0.7f;
        float single_track_abs_velocity_penalty = 0.15f;
        float single_track_frame_dt_s = 0.1f;
        float single_track_kinematic_mismatch_mps = 8.0f;
        float single_track_kinematic_penalty = 0.4f;
        float single_track_reacquire_max_velocity_jump_mps = 7.0f;
        int single_track_max_missed_frames = 2;

        // ---------- bin 到物理量的换算 ----------
        // range_m ~= range_bin * range_resolution_m
        // velocity_mps ~= doppler_bin_centered * velocity_resolution_mps
        float range_resolution_m = 0.15f;
        float velocity_resolution_mps = 0.2f;

        int udp_port = 8080;
        int mtu_bytes = 9000;
        int socket_rcvbuf_bytes = 32 * 1024 * 1024;

        int ring_slots = 3;
        bool prefer_mapped_zero_copy = false; // false: 先 memcpyAsync 到 device；true: kernel 直接读 mapped host
        bool debug_dump_power_map = false;
        bool verbose_frame_logs = false;
        //算出一整帧原始UDP pay load总共有字节数
        std::size_t frame_payload_bytes() const
        {
            // raw payload: [channel][I/Q] => int16 interleaved
            return static_cast<std::size_t>(num_channels) *
                   static_cast<std::size_t>(num_chirps) *
                   static_cast<std::size_t>(num_samples) *
                   2ULL * sizeof(int16_t);
        }

        std::size_t cube_complex_count() const
        {
            return static_cast<std::size_t>(num_channels) *
                   static_cast<std::size_t>(num_chirps) *
                   static_cast<std::size_t>(num_samples);
        }
        // 计算立方体缓冲区需要的字节数，单位是字节
        std::size_t cube_bytes() const
        {
            // float2 == 2 * float
            return cube_complex_count() * sizeof(float) * 2ULL;
        }

        std::size_t rd_map_elements() const
        {
            return static_cast<std::size_t>(num_chirps) *
                   static_cast<std::size_t>(num_samples);
        }

        std::size_t rd_map_bytes() const
        {
            return rd_map_elements() * sizeof(float);
        }
    };

    inline int doppler_center_bin(const RadarConfig &cfg)
    {
        return cfg.num_chirps / 2;
    }

    inline float doppler_bin_to_velocity_mps(const RadarConfig &cfg, int doppler_bin_unshifted)
    {
        int centered = doppler_bin_unshifted;
        if (centered >= cfg.num_chirps / 2)
        {
            centered -= cfg.num_chirps;
        }
        return centered * cfg.velocity_resolution_mps;
    }

    inline float range_bin_to_m(const RadarConfig &cfg, int range_bin)
    {
        return range_bin * cfg.range_resolution_m;
    }

} // namespace radar
