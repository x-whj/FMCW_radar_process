#pragma once
#include <cmath>
namespace radar
{   
    //测角标定参数，目前先用线性模型占位，后续可以替换成更复杂的 LUT
    struct MonopulseCalibration
    {
        bool az_enabled = true;
        bool el_enabled = true;
        // 先给线性占位参数，后续很容易替换成 LUT。
        float az_slope = 30.0f;
        float az_bias = 0.0f;
        float el_slope = 30.0f;
        float el_bias = 0.0f;
        __host__ __device__ float az_from_error(float e) const
        {
            return az_slope * e + az_bias;
        }
        __host__ __device__ float el_from_error(float e) const
        {
            return el_slope * e + el_bias;
        }
    };
} // namespace radar