#pragma once

#include <vector>

#include "gpu/RadarBuffers.h"
#include "gpu/fft/DopplerPlan.h"
#include "model/Calibration.h"
#include "model/ChannelMap.h"
#include "model/RadarConfig.h"
#include "model/TargetTypes.h"

namespace radar
{

    class RadarPipeline
    {
    public:
        RadarPipeline(const RadarConfig &cfg,
                      const ChannelMap &channel_map,
                      const MonopulseCalibration &calib);
        ~RadarPipeline();

        void initialize(const float *h_window_256);

        int process_frame(const int16_t *d_or_mapped_raw, std::vector<RadarTarget> &out_targets);

        cudaStream_t stream() const { return stream_; }

    private:
        RadarConfig cfg_;
        ChannelMap chmap_;
        MonopulseCalibration calib_;

        cudaStream_t stream_ = nullptr;
        GpuBuffers buffers_{};
        DopplerPlan fft_plan_{};

        int h_hit_count_ = 0;
        int h_peak_count_ = 0;

        // For optional single-target temporal gating.
        bool single_track_initialized_ = false;
        RadarTarget single_track_last_{};
        int single_track_missed_frames_ = 0;
    };

} // namespace radar
