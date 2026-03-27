#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math.h>

#include "model/Calibration.h"
#include "model/ChannelMap.h"
#include "model/RadarConfig.h"
#include "model/TargetTypes.h"

namespace radar
{

    __device__ inline float amplitude_ratio_from_sum_diff(const float2 diff, const float2 sum)
    {
        // Amplitude-comparison method:
        // left  = (sum + diff) / 2
        // right = (sum - diff) / 2
        // ratio = (|left| - |right|) / (|left| + |right|)
        const float2 left{
            0.5f * (sum.x + diff.x),
            0.5f * (sum.y + diff.y)};
        const float2 right{
            0.5f * (sum.x - diff.x),
            0.5f * (sum.y - diff.y)};

        const float left_amp = sqrtf(left.x * left.x + left.y * left.y);
        const float right_amp = sqrtf(right.x * right.x + right.y * right.y);
        const float den = left_amp + right_amp + 1e-12f;
        return (left_amp - right_amp) / den;
    }

    __global__ void monopulse_angle_kernel(
        const float2 *__restrict__ cube,
        const TargetDetection *__restrict__ dets,
        RadarTarget *__restrict__ targets,
        const int *__restrict__ det_count,
        int max_targets,
        RadarConfig cfg,
        ChannelMap chmap,
        MonopulseCalibration calib)
    {
        __shared__ int actual_count;

        if (threadIdx.x == 0)
        {
            int c = *det_count;
            actual_count = (c > max_targets) ? max_targets : c;
        }
        __syncthreads();

        const int tid = blockIdx.x * blockDim.x + threadIdx.x;
        if (tid >= actual_count)
        {
            return;
        }

        const TargetDetection det = dets[tid];

        const int r = static_cast<int>(det.range_bin);
        const int d = static_cast<int>(det.doppler_bin);

        const int plane = cfg.num_chirps * cfg.num_samples;
        const int base = d * cfg.num_samples + r;

        // ch0 -> sum channel Σ
        const float2 sumv = cube[chmap.sum_primary * plane + base];

        RadarTarget out{};
        out.range_m = det.range_bin * cfg.range_resolution_m;

        // 当前 RD 图不做显式 fftshift，所以这里按“未shift频谱索引”映射到正负速度
        int centered_doppler = d;
        if (centered_doppler >= cfg.num_chirps / 2)
        {
            centered_doppler -= cfg.num_chirps;
        }
        out.velocity_mps = centered_doppler * cfg.velocity_resolution_mps;

        out.azimuth_deg = 0.0f;
        out.elevation_deg = 0.0f;
        out.az_error = 0.0f;
        out.el_error = 0.0f;
        out.snr_db = det.snr_db;
        out.power = det.power;
        out.valid_az = 0;
        out.valid_el = 0;

        // ch1 -> azimuth difference Δaz (amplitude-comparison angle)
        if (chmap.has_diff_az && calib.az_enabled)
        {
            const float2 diff_az = cube[chmap.diff_az * plane + base];
            const float az_error = amplitude_ratio_from_sum_diff(diff_az, sumv);
            out.az_error = az_error;
            out.azimuth_deg = calib.az_from_error(az_error);
            out.valid_az = 1;
        }

        // ch2 -> elevation difference Δel (amplitude-comparison angle)
        if (chmap.has_diff_el && calib.el_enabled)
        {
            const float2 diff_el = cube[chmap.diff_el * plane + base];
            const float el_error = amplitude_ratio_from_sum_diff(diff_el, sumv);
            out.el_error = el_error;
            out.elevation_deg = calib.el_from_error(el_error);
            out.valid_el = 1;
        }

        targets[tid] = out;
    }

    void launch_monopulse(
        cudaStream_t stream,
        const float2 *cube,
        const TargetDetection *dets,
        RadarTarget *targets,
        const int *det_count,
        int max_targets,
        const RadarConfig &cfg,
        const ChannelMap &chmap,
        const MonopulseCalibration &calib)
    {
        const int threads = 128;
        const int blocks = (max_targets + threads - 1) / threads;

        monopulse_angle_kernel<<<blocks, threads, 0, stream>>>(
            cube,
            dets,
            targets,
            det_count,
            max_targets,
            cfg,
            chmap,
            calib);
    }

} // namespace radar
