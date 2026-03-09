#include <cuda_runtime.h>
#include <device_launch_parameters.h>

namespace radar
{

    __global__ void sum_channel_power_kernel(
        const float2 *__restrict__ cube,
        float *__restrict__ power_map,
        int sum_channel,
        int elements_per_channel)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx >= elements_per_channel)
            return;

        const float2 v = cube[sum_channel * elements_per_channel + idx];
        power_map[idx] = v.x * v.x + v.y * v.y;
    }

    void launch_sum_channel_power(
        cudaStream_t stream,
        const float2 *cube,
        float *power_map,
        int sum_channel,
        int elements_per_channel)
    {
        int threads = 256;
        int blocks = (elements_per_channel + threads - 1) / threads;
        sum_channel_power_kernel<<<blocks, threads, 0, stream>>>(cube, power_map, sum_channel, elements_per_channel);
    }

} // namespace radar