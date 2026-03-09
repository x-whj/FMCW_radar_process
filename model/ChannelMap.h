#pragma once

namespace radar
{

    // Confirmed channel semantic:
    // ch0 -> output_he.dat   -> sum channel (Sigma)
    // ch1 -> output_cha1.dat -> azimuth difference (Delta_az)
    // ch2 -> output_cha2.dat -> elevation difference (Delta_el)

    struct ChannelMap
    {
        int sum_primary = 0;
        int diff_az = 1;
        int diff_el = 2;

        bool has_diff_az = true;
        bool has_diff_el = true;
    };

} // namespace radar