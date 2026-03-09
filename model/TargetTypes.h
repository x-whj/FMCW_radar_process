#pragma once
#include <cstdint>
namespace radar
{
    struct TargetDetection
    {
        uint16_t range_bin;
        uint16_t doppler_bin;
        float power;
        float noise;
        float snr_db;
    };
    struct RadarTarget
    {
        float range_m;
        float velocity_mps;
        float azimuth_deg;
        float elevation_deg;
        float snr_db;
        float power;
        uint8_t valid_az;
        uint8_t valid_el;
    };
} // namespace radar