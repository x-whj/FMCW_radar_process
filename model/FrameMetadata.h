#pragma once

#include <cstdint>

namespace radar
{

    struct FrameRuntimeConfig
    {
        uint32_t frame_id = 0;
        uint32_t global_prt_count = 0;
        uint16_t first_prt_num = 0;

        bool has_beam_azimuth = false;
        bool has_beam_elevation = false;
        float beam_azimuth_deg = 0.0f;
        float beam_elevation_deg = 0.0f;

        float cfar_peak_min_snr_db = 0.0f;
    };

} // namespace radar
