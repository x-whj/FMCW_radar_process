#pragma once

#include <cuda_runtime.h>
#include <cufft.h>
#include <stdexcept>

#include "model/RadarConfig.h"
#include "model/TargetTypes.h"

namespace radar
{

#define CUDA_CHECK(call)                                                                       \
    do                                                                                         \
    {                                                                                          \
        cudaError_t err__ = (call);                                                            \
        if (err__ != cudaSuccess)                                                              \
        {                                                                                      \
            throw std::runtime_error(std::string("CUDA error: ") + cudaGetErrorString(err__)); \
        }                                                                                      \
    } while (0)

#define CUFFT_CHECK(call)                            \
    do                                               \
    {                                                \
        cufftResult err__ = (call);                  \
        if (err__ != CUFFT_SUCCESS)                  \
        {                                            \
            throw std::runtime_error("CUFFT error"); \
        }                                            \
    } while (0)

    struct GpuBuffers
    {
        float2 *d_cube = nullptr;         // [channel][chirp][sample]
        float *d_power_map = nullptr;     // [chirp][sample]
        uint8_t *d_hit_map = nullptr;     // [chirp][sample]
        float *d_noise_map = nullptr;     // [chirp][sample]
        float *d_threshold_map = nullptr; // [chirp][sample]

        TargetDetection *d_detections = nullptr;
        RadarTarget *d_targets = nullptr;

        int *d_hit_count = nullptr;
        int *d_peak_count = nullptr;

        int16_t *d_stage_raw = nullptr; // 如果 prefer_mapped_zero_copy=false，则用于 staging
    };

    inline void allocate_gpu_buffers(const RadarConfig &cfg, GpuBuffers &b)
    {
        CUDA_CHECK(cudaMalloc(&b.d_cube, cfg.cube_bytes()));
        CUDA_CHECK(cudaMalloc(&b.d_power_map, cfg.rd_map_bytes()));
        CUDA_CHECK(cudaMalloc(&b.d_hit_map, cfg.rd_map_elements() * sizeof(uint8_t)));
        CUDA_CHECK(cudaMalloc(&b.d_noise_map, cfg.rd_map_bytes()));
        CUDA_CHECK(cudaMalloc(&b.d_threshold_map, cfg.rd_map_bytes()));

        CUDA_CHECK(cudaMalloc(&b.d_detections, cfg.max_targets * sizeof(TargetDetection)));
        CUDA_CHECK(cudaMalloc(&b.d_targets, cfg.max_targets * sizeof(RadarTarget)));

        CUDA_CHECK(cudaMalloc(&b.d_hit_count, sizeof(int)));
        CUDA_CHECK(cudaMalloc(&b.d_peak_count, sizeof(int)));

        if (!cfg.prefer_mapped_zero_copy)
        {
            CUDA_CHECK(cudaMalloc(&b.d_stage_raw, cfg.frame_payload_bytes()));
        }
    }

    inline void free_gpu_buffers(GpuBuffers &b)
    {
        cudaFree(b.d_cube);
        cudaFree(b.d_power_map);
        cudaFree(b.d_hit_map);
        cudaFree(b.d_noise_map);
        cudaFree(b.d_threshold_map);

        cudaFree(b.d_detections);
        cudaFree(b.d_targets);

        cudaFree(b.d_hit_count);
        cudaFree(b.d_peak_count);

        cudaFree(b.d_stage_raw);
        b = {};
    }

} // namespace radar