#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math.h>

#include "model/TargetTypes.h"

namespace radar
{

    constexpr int CFAR_TILE_X = 16;
    constexpr int CFAR_TILE_Y = 16;
    constexpr int CFAR_MAX_WINDOW = 25;
    constexpr int CFAR_MAX_HALF_WIN = CFAR_MAX_WINDOW / 2;
    constexpr int CFAR_SMEM_X = CFAR_TILE_X + 2 * CFAR_MAX_HALF_WIN + 1;
    constexpr int CFAR_SMEM_Y = CFAR_TILE_Y + 2 * CFAR_MAX_HALF_WIN;
    constexpr int kWarpSize = 32;

    struct CfarRuntimeParam
    {
        int train_x;
        int train_y;
        int guard_x;
        int guard_y;
        float alpha;
        float min_noise;
        float min_cut_power;
        int near_range_suppress_bins;
        int zero_doppler_suppress_bins;
    };

    __device__ __forceinline__ bool is_inside_guard(int dx, int dy, int guard_x, int guard_y)
    {
        return (abs(dx) <= guard_x) && (abs(dy) <= guard_y);
    }

    template <int WarpSize = kWarpSize>
    __device__ __forceinline__ int warp_reduce_sum_int(int v)
    {
#pragma unroll
        for (int mask = WarpSize >> 1; mask >= 1; mask >>= 1)
        {
            v += __shfl_xor_sync(0xffffffff, v, mask);
        }
        return v;
    }

    template <int TrainX, int TrainY, int GuardX, int GuardY>
    __global__ void cfar_ca_2d_power_kernel_fixed(
        const float *__restrict__ power_map,
        uint8_t *__restrict__ hit_map,
        float *__restrict__ noise_map,
        int width,
        int height,
        CfarRuntimeParam p)
    {
        __shared__ float tile[CFAR_SMEM_Y][CFAR_SMEM_X];

        constexpr int half_win_x = TrainX + GuardX;
        constexpr int half_win_y = TrainY + GuardY;
        constexpr int win_x = 2 * half_win_x + 1;
        constexpr int win_y = 2 * half_win_y + 1;

        const int tx = threadIdx.x;
        const int ty = threadIdx.y;

        const int out_x = blockIdx.x * CFAR_TILE_X + tx;
        const int out_y = blockIdx.y * CFAR_TILE_Y + ty;

        const int base_x = blockIdx.x * CFAR_TILE_X - half_win_x;
        const int base_y = blockIdx.y * CFAR_TILE_Y - half_win_y;

        for (int y = ty; y < CFAR_TILE_Y + 2 * half_win_y; y += blockDim.y)
        {
            for (int x = tx; x < CFAR_TILE_X + 2 * half_win_x; x += blockDim.x)
            {
                const int gx = base_x + x;
                const int gy = base_y + y;
                float v = 0.0f;
                if ((unsigned)gx < (unsigned)width && (unsigned)gy < (unsigned)height)
                {
                    v = power_map[gy * width + gx];
                }
                tile[y][x] = v;
            }
        }
        __syncthreads();

        if (out_x >= width || out_y >= height)
            return;

        const int out_idx = out_y * width + out_x;

        if (out_x < p.near_range_suppress_bins)
        {
            hit_map[out_idx] = 0;
            if (noise_map)
                noise_map[out_idx] = 0.0f;
            return;
        }

        if (p.zero_doppler_suppress_bins > 0)
        {
            const int notch = p.zero_doppler_suppress_bins;
            if (out_y < notch || out_y >= height - notch)
            {
                hit_map[out_idx] = 0;
                if (noise_map)
                    noise_map[out_idx] = 0.0f;
                return;
            }
        }

        if (out_x < half_win_x || out_x >= width - half_win_x ||
            out_y < half_win_y || out_y >= height - half_win_y)
        {
            hit_map[out_idx] = 0;
            if (noise_map)
                noise_map[out_idx] = 0.0f;
            return;
        }

        float noise_sum = 0.0f;
        int ref_count = 0;

#pragma unroll
        for (int wy = 0; wy < win_y; ++wy)
        {
            const int dy = wy - half_win_y;
#pragma unroll
            for (int wx = 0; wx < win_x; ++wx)
            {
                const int dx = wx - half_win_x;
                if ((abs(dx) <= GuardX) && (abs(dy) <= GuardY))
                    continue;

                noise_sum += tile[ty + wy][tx + wx];
                ref_count++;
            }
        }

        const float cut = tile[ty + half_win_y][tx + half_win_x];
        const float noise = fmaxf(noise_sum / fmaxf(1.0f, (float)ref_count), p.min_noise);
        const float threshold = p.alpha * noise;
        const bool pass = (cut > threshold) && (cut > p.min_cut_power);

        hit_map[out_idx] = pass ? 1 : 0;
        if (noise_map)
            noise_map[out_idx] = noise;
    }

    __global__ void cfar_ca_2d_power_kernel_v2(
        const float *__restrict__ power_map,
        uint8_t *__restrict__ hit_map,
        float *__restrict__ noise_map,
        float *__restrict__ threshold_map,
        int width,
        int height,
        CfarRuntimeParam p)
    {
        __shared__ float tile[CFAR_SMEM_Y][CFAR_SMEM_X];

        const int tx = threadIdx.x;
        const int ty = threadIdx.y;

        const int half_win_x = p.train_x + p.guard_x;
        const int half_win_y = p.train_y + p.guard_y;
        const int win_x = 2 * half_win_x + 1;
        const int win_y = 2 * half_win_y + 1;

        const int out_x = blockIdx.x * CFAR_TILE_X + tx;
        const int out_y = blockIdx.y * CFAR_TILE_Y + ty;

        const int base_x = blockIdx.x * CFAR_TILE_X - half_win_x;
        const int base_y = blockIdx.y * CFAR_TILE_Y - half_win_y;

        for (int y = ty; y < CFAR_TILE_Y + 2 * half_win_y; y += blockDim.y)
        {
            for (int x = tx; x < CFAR_TILE_X + 2 * half_win_x; x += blockDim.x)
            {
                const int gx = base_x + x;
                const int gy = base_y + y;
                float v = 0.0f;
                if ((unsigned)gx < (unsigned)width && (unsigned)gy < (unsigned)height)
                {
                    v = power_map[gy * width + gx];
                }
                tile[y][x] = v;
            }
        }
        __syncthreads();

        if (out_x >= width || out_y >= height)
            return;

        const int out_idx = out_y * width + out_x;

        // 近距离抑制：先把前 20 个 range bin 全部清掉
        if (out_x < p.near_range_suppress_bins)
        {
            hit_map[out_idx] = 0;
            if (noise_map)
                noise_map[out_idx] = 0.0f;
            if (threshold_map)
                threshold_map[out_idx] = 0.0f;
            return;
        }

        if (p.zero_doppler_suppress_bins > 0)
        {
            const int notch = p.zero_doppler_suppress_bins;
            if (out_y < notch || out_y >= height - notch)
            {
                hit_map[out_idx] = 0;
                if (noise_map)
                    noise_map[out_idx] = 0.0f;
                if (threshold_map)
                    threshold_map[out_idx] = 0.0f;
                return;
            }
        }

        if (out_x < half_win_x || out_x >= width - half_win_x ||
            out_y < half_win_y || out_y >= height - half_win_y)
        {
            hit_map[out_idx] = 0;
            if (noise_map)
                noise_map[out_idx] = 0.0f;
            if (threshold_map)
                threshold_map[out_idx] = 0.0f;
            return;
        }

        float noise_sum = 0.0f;
        int ref_count = 0;

#pragma unroll
        for (int wy = 0; wy < CFAR_MAX_WINDOW; ++wy)
        {
            if (wy >= win_y)
                break;
            const int dy = wy - half_win_y;
#pragma unroll
            for (int wx = 0; wx < CFAR_MAX_WINDOW; ++wx)
            {
                if (wx >= win_x)
                    break;
                const int dx = wx - half_win_x;
                if (is_inside_guard(dx, dy, p.guard_x, p.guard_y))
                    continue;

                noise_sum += tile[ty + wy][tx + wx];
                ref_count++;
            }
        }

        const float cut = tile[ty + half_win_y][tx + half_win_x];
        const float noise = fmaxf(noise_sum / fmaxf(1.0f, (float)ref_count), p.min_noise);
        const float threshold = p.alpha * noise;
        const bool pass = (cut > threshold) && (cut > p.min_cut_power);

        hit_map[out_idx] = pass ? 1 : 0;
        if (noise_map)
            noise_map[out_idx] = noise;
        if (threshold_map)
            threshold_map[out_idx] = threshold;
    }

    template <int NumThreads = 256>
    __global__ void count_hits_kernel(
        const uint8_t *__restrict__ hit_map,
        int *__restrict__ hit_count,
        int width,
        int height)
    {
        constexpr int kWarps = (NumThreads + kWarpSize - 1) / kWarpSize;
        __shared__ int smem[kWarps];

        const int x = blockIdx.x * blockDim.x + threadIdx.x;
        const int y = blockIdx.y * blockDim.y + threadIdx.y;
        const int tid = threadIdx.y * blockDim.x + threadIdx.x;
        const int lane = tid % kWarpSize;
        const int warp = tid / kWarpSize;

        int local_count = 0;
        if (x < width && y < height)
        {
            const int idx = y * width + x;
            local_count = hit_map[idx] ? 1 : 0;
        }

        local_count = warp_reduce_sum_int<kWarpSize>(local_count);
        if (lane == 0)
        {
            smem[warp] = local_count;
        }
        __syncthreads();

        int block_sum = (tid < kWarps) ? smem[tid] : 0;
        if (warp == 0)
        {
            block_sum = warp_reduce_sum_int<kWarps>(block_sum);
        }
        if (tid == 0)
        {
            atomicAdd(hit_count, block_sum);
        }
    }

    __global__ void peak_extract_kernel_v2(
        const float *__restrict__ power_map,
        const float *__restrict__ noise_map,
        const uint8_t *__restrict__ hit_map,
        TargetDetection *__restrict__ detections,
        int *__restrict__ peak_count,
        int width,
        int height,
        int max_targets,
        int peak_half_window,
        float min_snr_db)
    {
        const int x = blockIdx.x * blockDim.x + threadIdx.x;
        const int y = blockIdx.y * blockDim.y + threadIdx.y;

        if (x < 0 || x >= width || y < 0 || y >= height)
            return;

        const int idx = y * width + x;
        if (!hit_map[idx])
            return;

        const float c = power_map[idx];

        bool is_peak = true;
        for (int oy = -peak_half_window; oy <= peak_half_window && is_peak; ++oy)
        {
            for (int ox = -peak_half_window; ox <= peak_half_window; ++ox)
            {
                if (ox == 0 && oy == 0)
                    continue;

                const int nx = x + ox;
                const int ny = y + oy;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                    continue;

                const int nidx = ny * width + nx;
                // NMS should compare against other CFAR hits; otherwise
                // a non-hit high neighbor can suppress all valid hits nearby.
                if (!hit_map[nidx])
                    continue;
                const float v = power_map[nidx];
                if (v > c || (v == c && nidx < idx))
                {
                    is_peak = false;
                }
            }
        }
        if (!is_peak)
            return;

        const float noise = fmaxf(noise_map ? noise_map[idx] : 1e-12f, 1e-12f);
        const float snr_db = 10.0f * log10f(fmaxf(c / noise, 1e-12f));
        if (snr_db < min_snr_db)
            return;

        const int slot = atomicAdd(peak_count, 1);
        if (slot < max_targets)
        {
            detections[slot].range_bin = static_cast<uint16_t>(x);
            detections[slot].doppler_bin = static_cast<uint16_t>(y);
            detections[slot].power = c;
            detections[slot].noise = noise;
            detections[slot].snr_db = snr_db;
        }
    }

    inline float calc_ca_alpha(float pfa, int ref_count)
    {
        if (ref_count <= 0)
            return 1.0f;
        pfa = fminf(fmaxf(pfa, 1e-9f), 1e-2f);
        return ref_count * (powf(pfa, -1.0f / ref_count) - 1.0f);
    }

    void launch_cfar_and_peak_extract_v2(
        cudaStream_t stream,
        const float *power_map,
        uint8_t *hit_map,
        float *noise_map,
        float *threshold_map,
        TargetDetection *detections,
        int *hit_count,
        int *peak_count,
        int width,
        int height,
        int train_x,
        int train_y,
        int guard_x,
        int guard_y,
        float pfa,
        float min_noise,
        float min_cut_power,
        int near_range_suppress_bins,
        int zero_doppler_suppress_bins,
        int peak_half_window,
        float peak_min_snr_db,
        int max_targets,
        bool collect_hit_count)
    {
        const int half_win_x = train_x + guard_x;
        const int half_win_y = train_y + guard_y;
        const int win_x = 2 * half_win_x + 1;
        const int win_y = 2 * half_win_y + 1;
        const int guard_cells = (2 * guard_x + 1) * (2 * guard_y + 1);
        const int total_cells = win_x * win_y;
        const int ref_count = total_cells - guard_cells;

        CfarRuntimeParam p;
        p.train_x = train_x;
        p.train_y = train_y;
        p.guard_x = guard_x;
        p.guard_y = guard_y;
        p.alpha = calc_ca_alpha(pfa, ref_count);
        p.min_noise = min_noise;
        p.min_cut_power = min_cut_power;
        p.near_range_suppress_bins = near_range_suppress_bins;
        p.zero_doppler_suppress_bins = zero_doppler_suppress_bins;

        cudaMemsetAsync(peak_count, 0, sizeof(int), stream);
        if (collect_hit_count && hit_count)
        {
            cudaMemsetAsync(hit_count, 0, sizeof(int), stream);
        }

        dim3 block(CFAR_TILE_X, CFAR_TILE_Y);
        dim3 grid((width + CFAR_TILE_X - 1) / CFAR_TILE_X,
                  (height + CFAR_TILE_Y - 1) / CFAR_TILE_Y);

        if (train_x == 8 && train_y == 6 && guard_x == 2 && guard_y == 2)
        {
            cfar_ca_2d_power_kernel_fixed<8, 6, 2, 2><<<grid, block, 0, stream>>>(
                power_map, hit_map, noise_map, width, height, p);
        }
        else
        {
            cfar_ca_2d_power_kernel_v2<<<grid, block, 0, stream>>>(
                power_map, hit_map, noise_map, threshold_map, width, height, p);
        }

        dim3 block_cnt(16, 16);
        dim3 grid_cnt((width + block_cnt.x - 1) / block_cnt.x,
                      (height + block_cnt.y - 1) / block_cnt.y);

        if (collect_hit_count && hit_count)
        {
            count_hits_kernel<256><<<grid_cnt, block_cnt, 0, stream>>>(
                hit_map, hit_count, width, height);
        }

        peak_extract_kernel_v2<<<grid_cnt, block_cnt, 0, stream>>>(
            power_map, noise_map, hit_map, detections, peak_count, width, height, max_targets, peak_half_window, peak_min_snr_db);
    }

} // namespace radar
