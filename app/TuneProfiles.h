#pragma once

#include "model/ChannelMap.h"
#include "model/RadarConfig.h"

namespace radar
{

    enum class TuneProfile
    {
        SingleFriendly,
        MultiTarget
    };

    inline const char *tune_profile_name(TuneProfile profile)
    {
        return profile == TuneProfile::MultiTarget ? "multi" : "single";
    }

    inline ChannelMap make_default_channel_map()
    {
        ChannelMap chmap;
        chmap.sum_primary = 0;
        chmap.diff_az = 1;
        chmap.diff_el = 2;
        chmap.has_diff_az = true;
        chmap.has_diff_el = true;
        return chmap;
    }

    inline void apply_single_friendly_profile(RadarConfig &cfg)
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

    inline void apply_multi_target_profile(RadarConfig &cfg)
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

    inline void apply_tune_profile(RadarConfig &cfg, TuneProfile profile)
    {
        if (profile == TuneProfile::MultiTarget)
        {
            apply_multi_target_profile(cfg);
            return;
        }

        apply_single_friendly_profile(cfg);
    }

} // namespace radar
