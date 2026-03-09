#pragma once

#include <cufft.h>

#include "gpu/RadarBuffers.h"
#include "model/RadarConfig.h"

namespace radar
{

    struct DopplerPlan
    {
        cufftHandle handle = 0;

        void create(const RadarConfig &cfg, cudaStream_t stream)
        {
            CUFFT_CHECK(cufftCreate(&handle));
            CUFFT_CHECK(cufftSetStream(handle, stream));

            int n[1] = {cfg.num_chirps};

            // 对单个通道做 plan：
            // 输入布局是 [chirp][sample]
            // 对每个 sample 做一条长度为 num_chirps 的 FFT
            int inembed[1] = {cfg.num_chirps};
            int onembed[1] = {cfg.num_chirps};

            int istride = cfg.num_samples;
            int ostride = cfg.num_samples;

            int idist = 1;
            int odist = 1;

            int howmany = cfg.num_samples;

            CUFFT_CHECK(cufftPlanMany(
                &handle,
                1,
                n,
                inembed,
                istride,
                idist,
                onembed,
                ostride,
                odist,
                CUFFT_C2C,
                howmany));
        }

        void destroy()
        {
            if (handle)
            {
                cufftDestroy(handle);
                handle = 0;
            }
        }
    };

} // namespace radar