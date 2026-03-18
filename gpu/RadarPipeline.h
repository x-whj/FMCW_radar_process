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

    struct SignalTimingSummary
    {
        std::size_t frame_count = 0;
        double total_unpack_window_ms = 0.0;
        double total_fft_ms = 0.0;
        double total_power_ms = 0.0;
        double total_mtd_ms = 0.0;
        double total_cfar_ms = 0.0;
        double total_angle_ms = 0.0;
        double total_signal_ms = 0.0;
    };

    // 一帧处理总控类（把各个 CUDA 内核串起来）。
    // 核心入口：process_frame()
    // 输入：原始一帧 int16 IQ（设备指针或 mapped 指针）
    // 输出：RadarTarget 列表（已做后处理/可选单目标选择）
    class RadarPipeline
    {
    public:
        RadarPipeline(const RadarConfig &cfg,
                      const ChannelMap &channel_map,
                      const MonopulseCalibration &calib);
        ~RadarPipeline();

        void initialize(const float *h_window_256);

        int process_frame(const int16_t *d_or_mapped_raw, std::vector<RadarTarget> &out_targets);
        void print_signal_timing_summary() const;
        const SignalTimingSummary &signal_timing_summary() const { return signal_timing_summary_; }

        cudaStream_t stream() const { return stream_; }

    private:
        RadarConfig cfg_;
        ChannelMap chmap_;
        MonopulseCalibration calib_;

        cudaStream_t stream_ = nullptr;
        GpuBuffers buffers_{};
        DopplerPlan fft_plan_{};
        static constexpr int kTimingEventCount = 7;
        cudaEvent_t timing_events_[kTimingEventCount]{};
        SignalTimingSummary signal_timing_summary_{};

        int h_hit_count_ = 0;
        int h_peak_count_ = 0;

        // For optional single-target temporal gating.
        bool single_track_initialized_ = false;
        RadarTarget single_track_last_{};
        int single_track_missed_frames_ = 0;
    };

} // namespace radar
