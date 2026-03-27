#pragma once

#include <cstddef>
#include <cstdint>

namespace radar
{

    struct RadarConfig
    {
        static constexpr std::size_t kPrtHeaderBytes = 256;
        static constexpr std::uint32_t kPrtFixedHeader = 0xAA55CD32u;
        static constexpr float kLightSpeedMps = 299792458.0f;

       
        int num_channels = 3; // 1 sum + 1 az diff + 1 el diff
        int num_chirps = 256;
        int num_samples = 664;

        int cfar_window = 17; // 必须为奇数
        int cfar_guard = 2;
        float cfar_pfa = 1e-5f;
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

        // ---------- 波形物理参数 ----------
        // 当前按老师给定的实际参数设置：
        // fc = 28.2 GHz, B = 20 MHz, PRT = 51 us
        float carrier_frequency_hz = 28.2e9f;
        float waveform_bandwidth_hz = 20.0e6f;
        float prt_s = 51.0e-6f;

        // ---------- bin 到物理量的换算 ----------
        // range_m ~= range_bin * range_resolution_m
        // velocity_mps ~= doppler_bin_centered * velocity_resolution_mps
        //
        // 这里默认按脉压带宽和 PRT 推导：
        // range_resolution_m = c / (2B)
        // velocity_resolution_mps = lambda / (2 * num_chirps * PRT)
        //
        // 若后续老师确认“664 个 sample 的真实距离门间隔”与 c/(2B) 不同，
        // 再把这两个默认值改为协议/系统真实标定值。
        float range_resolution_m = 299792458.0f / (2.0f * 20.0e6f);
        float velocity_resolution_mps = (299792458.0f / 28.2e9f) / (2.0f * 256.0f * 51.0e-6f);

        int udp_port = 8080;
        int udp_payload_bytes = 1464;
        int mtu_bytes = 9000;
        int socket_rcvbuf_bytes = 32 * 1024 * 1024;

        int ring_slots = 3;
        bool prefer_mapped_zero_copy = false; // false: 先 memcpyAsync 到 device；true: kernel 直接读 mapped host
        bool debug_dump_power_map = false;
        bool debug_print_power_stats = false;
        bool verbose_frame_logs = false;
        int protocol_prt_start_num = 1;
        int protocol_prts_per_cpi = 261;

        int protocol_prt_last_num() const
        {
            return protocol_prt_start_num + protocol_prts_per_cpi - 1;
        }

        int active_prt_first_num() const
        {
            return protocol_prt_start_num;
        }

        int active_prt_last_num() const
        {
            return protocol_prt_start_num + num_chirps - 1;
        }

        bool protocol_prt_num_in_range(int prt_num) const
        {
            return prt_num >= protocol_prt_start_num && prt_num <= protocol_prt_last_num();
        }

        bool active_prt_num_in_range(int prt_num) const
        {
            return prt_num >= active_prt_first_num() && prt_num <= active_prt_last_num();
        }

        // 单个 PRT 数据区大小（不含 256 B PRT 头）：
        // [channel][sample][Q,I]，其中 channel 顺序为 sum/az/el
        std::size_t prt_data_bytes() const
        {
            return static_cast<std::size_t>(num_channels) *
                   static_cast<std::size_t>(num_samples) *
                   2ULL * sizeof(int16_t);
        }

        std::size_t prt_total_bytes() const
        {
            return kPrtHeaderBytes + prt_data_bytes();
        }

        // 一整帧 CPI 原始数据区大小（不含各 PRT 头）：
        // [chirp][channel][sample][Q,I]
        std::size_t frame_payload_bytes() const
        {
            return static_cast<std::size_t>(num_chirps) * prt_data_bytes();
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

        float wavelength_m() const
        {
            return kLightSpeedMps / carrier_frequency_hz;
        }

        float nominal_range_resolution_m() const
        {
            return kLightSpeedMps / (2.0f * waveform_bandwidth_hz);
        }

        float nominal_velocity_resolution_mps() const
        {
            return wavelength_m() / (2.0f * static_cast<float>(num_chirps) * prt_s);
        }

        float nominal_max_abs_velocity_mps() const
        {
            return wavelength_m() / (4.0f * prt_s);
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
