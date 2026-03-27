#pragma once

#include "io/PrtProtocol.h"
#include "model/FrameMetadata.h"
#include "model/RadarConfig.h"

namespace radar
{

    inline FrameRuntimeConfig build_frame_runtime_config(uint32_t frame_id,
                                                         const PrtHeaderInfo &hdr)
    {
        FrameRuntimeConfig runtime;
        runtime.frame_id = frame_id;
        runtime.global_prt_count = hdr.global_prt_count();
        runtime.first_prt_num = hdr.prt_num();

        if (hdr.radar_angle_a_raw() != 0)
        {
            runtime.has_beam_azimuth = true;
            runtime.beam_azimuth_deg = hdr.radar_angle_a_deg();
        }

        if (hdr.radar_angle_e_raw() != 0)
        {
            runtime.has_beam_elevation = true;
            runtime.beam_elevation_deg = hdr.radar_angle_e_deg();
        }

        return runtime;
    }

} // namespace radar
