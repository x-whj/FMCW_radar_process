#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cstdint>

namespace radar
{

    __global__ void sum_channel_power_kernel_scalar(
        const float2 *__restrict__ cube,
        float *__restrict__ power_map,
        int sum_channel,
        int elements_per_channel)
    {
        const int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= elements_per_channel)
            return;

        const float2 v = cube[sum_channel * elements_per_channel + idx];
        power_map[idx] = __fmaf_rn(v.x, v.x, v.y * v.y);
    }

    __global__ void sum_channel_power_kernel_f32x2(
        const float2 *__restrict__ cube,
        float *__restrict__ power_map,
        int sum_channel,
        int elements_per_channel)
    {
        const int idx = (blockIdx.x * blockDim.x + threadIdx.x) * 2;
        const float2 *sum_ptr = cube + sum_channel * elements_per_channel;

        if (idx + 1 < elements_per_channel)
        {
            const float4 v = reinterpret_cast<const float4 *>(sum_ptr)[idx / 2];
            float2 out;
            out.x = __fmaf_rn(v.x, v.x, v.y * v.y);
            out.y = __fmaf_rn(v.z, v.z, v.w * v.w);
            reinterpret_cast<float2 *>(power_map)[idx / 2] = out;
            return;
        }

        if (idx < elements_per_channel)
        {
            const float2 v = sum_ptr[idx];
            power_map[idx] = __fmaf_rn(v.x, v.x, v.y * v.y);
        }
    }

    void launch_sum_channel_power(
        cudaStream_t stream,
        const float2 *cube,
        float *power_map,
        int sum_channel,
        int elements_per_channel)
    {
        constexpr int threads = 256;
        const std::uintptr_t sum_ptr_addr =
            reinterpret_cast<std::uintptr_t>(cube + sum_channel * elements_per_channel);
        const bool aligned_for_f32x2 =
            (sum_ptr_addr % alignof(float4) == 0) &&
            (reinterpret_cast<std::uintptr_t>(power_map) % alignof(float2) == 0);

        if (aligned_for_f32x2)
        {
            const int blocks = (elements_per_channel + threads * 2 - 1) / (threads * 2);
            sum_channel_power_kernel_f32x2<<<blocks, threads, 0, stream>>>(
                cube, power_map, sum_channel, elements_per_channel);
            return;
        }

        const int blocks = (elements_per_channel + threads - 1) / threads;
        sum_channel_power_kernel_scalar<<<blocks, threads, 0, stream>>>(
            cube, power_map, sum_channel, elements_per_channel);
    }

} // namespace radar
