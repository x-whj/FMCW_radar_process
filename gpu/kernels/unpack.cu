#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdint.h>

namespace radar
{

    // 慢时间窗，由主机侧一次性上传到常量内存
    __constant__ float k_slow_time_window[256];

    // 原始 CPI 数据区布局：
    // [chirp][channel][sample][Q,I]
    // 其中：
    // ch0 -> 和通道 Sigma
    // ch1 -> 方位差通道 Delta_az
    // ch2 -> 俯仰差通道 Delta_el
    //
    // 文档规定单个采样点的 32-bit 小端排列等价于接收端按 16-bit 读取后得到 [Q, I]，
    // 所以这里需要在写入 float2 时交换成 (I, Q)。
    //
    // 输出立方体布局：
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
        const int channel_stride = num_samples * 2;
        const int chirp_stride = num_channels * channel_stride;
        const int chirp_base = chirp * chirp_stride;
        const float w = k_slow_time_window[chirp];

        for (int ch = 0; ch < num_channels; ++ch)
        {
            const int raw_idx = chirp_base + ch * channel_stride + sample * 2;
            const int cube_idx = ch * plane + chirp * num_samples + sample;

            // 线上的顺序是 [Q, I]，这里恢复成算法内部统一使用的 (I, Q)。
            const float q = static_cast<float>(raw[raw_idx + 0]);
            const float i = static_cast<float>(raw[raw_idx + 1]);

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
        const int channel_stride = num_samples * 2;
        const int chirp_stride = channel_stride * 3;
        const int idx = chirp * num_samples + sample;
        const int sample_base = chirp * chirp_stride + sample * 2;
        const float w = k_slow_time_window[chirp];

        // ch0/ch1/ch2 三个通道各自的平面内都按 [Q, I] 存放同一个 sample。
        cube[idx] = make_float2(static_cast<float>(raw[sample_base + 1]) * w,
                                static_cast<float>(raw[sample_base + 0]) * w);
        cube[plane + idx] = make_float2(static_cast<float>(raw[sample_base + channel_stride + 1]) * w,
                                        static_cast<float>(raw[sample_base + channel_stride + 0]) * w);
        cube[2 * plane + idx] = make_float2(static_cast<float>(raw[sample_base + 2 * channel_stride + 1]) * w,
                                            static_cast<float>(raw[sample_base + 2 * channel_stride + 0]) * w);
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
