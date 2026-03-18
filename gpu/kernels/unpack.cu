#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdint.h>

namespace radar
{

    // Slow-time window, loaded once from host
    __constant__ float k_slow_time_window[256];

    // Raw payload layout for each (chirp, sample):
    // [ch0_I, ch0_Q, ch1_I, ch1_Q, ch2_I, ch2_Q]
    // where:
    // ch0 -> sum channel Sigma
    // ch1 -> azimuth difference Delta_az
    // ch2 -> elevation difference Delta_el
    //
    // Output cube layout:
    // [channel][chirp][sample]

    __global__ void unpack_and_window_kernel(
        const int16_t *__restrict__ raw,
        float2 *__restrict__ cube,
        int num_channels,
        int num_chirps,
        int num_samples)
    {
        const int sample = blockIdx.x * blockDim.x + threadIdx.x;
        const int chirp = blockIdx.y * blockDim.y + threadIdx.y;

        if (sample >= num_samples || chirp >= num_chirps)
        {
            return;
        }

        const int plane = num_chirps * num_samples;
        const int raw_base = (chirp * num_samples + sample) * (num_channels * 2);
        const float w = k_slow_time_window[chirp];

        for (int ch = 0; ch < num_channels; ++ch)
        {
            const int raw_idx = raw_base + ch * 2;
            const int cube_idx = ch * plane + chirp * num_samples + sample;

            const float i = static_cast<float>(raw[raw_idx + 0]);
            const float q = static_cast<float>(raw[raw_idx + 1]);

            cube[cube_idx] = make_float2(i * w, q * w);
        }
    }

    __global__ void unpack_and_window_kernel_ch3(
        const int16_t *__restrict__ raw,
        float2 *__restrict__ cube,
        int num_chirps,
        int num_samples)
    {
        const int sample = blockIdx.x * blockDim.x + threadIdx.x;
        const int chirp = blockIdx.y * blockDim.y + threadIdx.y;

        if (sample >= num_samples || chirp >= num_chirps)
        {
            return;
        }

        const int plane = num_chirps * num_samples;
        const int idx = chirp * num_samples + sample;
        const int raw_base = idx * 6;
        const float w = k_slow_time_window[chirp];

        cube[idx] = make_float2(static_cast<float>(raw[raw_base + 0]) * w,
                                static_cast<float>(raw[raw_base + 1]) * w);
        cube[plane + idx] = make_float2(static_cast<float>(raw[raw_base + 2]) * w,
                                        static_cast<float>(raw[raw_base + 3]) * w);
        cube[2 * plane + idx] = make_float2(static_cast<float>(raw[raw_base + 4]) * w,
                                            static_cast<float>(raw[raw_base + 5]) * w);
    }

    void upload_slow_time_window(const float *h_window, int count)
    {
        cudaMemcpyToSymbol(k_slow_time_window, h_window, count * sizeof(float));
    }

    void launch_unpack_and_window(
        cudaStream_t stream,
        const int16_t *raw,
        float2 *cube,
        int num_channels,
        int num_chirps,
        int num_samples)
    {
        const dim3 block(32, 8);
        const dim3 grid(
            (num_samples + block.x - 1) / block.x,
            (num_chirps + block.y - 1) / block.y);

        if (num_channels == 3)
        {
            unpack_and_window_kernel_ch3<<<grid, block, 0, stream>>>(
                raw,
                cube,
                num_chirps,
                num_samples);
            return;
        }

        unpack_and_window_kernel<<<grid, block, 0, stream>>>(
            raw,
            cube,
            num_channels,
            num_chirps,
            num_samples);
    }

} // namespace radar
