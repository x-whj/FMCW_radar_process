#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "model/RadarConfig.h"

namespace radar
{

    inline uint16_t word_hi16(uint32_t word)
    {
        return static_cast<uint16_t>(word >> 16);
    }

    inline uint16_t word_lo16(uint32_t word)
    {
        return static_cast<uint16_t>(word & 0xFFFFu);
    }

    inline uint32_t read_le_u32(const uint8_t *p)
    {
        return static_cast<uint32_t>(p[0]) |
               (static_cast<uint32_t>(p[1]) << 8) |
               (static_cast<uint32_t>(p[2]) << 16) |
               (static_cast<uint32_t>(p[3]) << 24);
    }

    inline float decode_radar_angle_deg(uint16_t raw_angle)
    {
        const int signed_raw = static_cast<int>(static_cast<int16_t>(raw_angle));
        return (signed_raw - 1000) * 0.05f;
    }

    struct PrtHeaderInfo
    {
        static constexpr std::size_t kWordCount = 64;

        std::array<uint32_t, kWordCount> words{};

        uint32_t fixed_head() const { return words[0]; }
        uint32_t device_version_word() const { return words[1]; }
        uint32_t global_prt_count() const { return words[2]; }

        uint16_t device_id() const { return word_hi16(words[1]); }
        uint16_t firmware_version() const { return word_lo16(words[1]); }

        uint16_t prt_num() const { return word_hi16(words[3]); }
        uint16_t prt_samples() const { return word_lo16(words[3]); }

        uint16_t frame_length_bytes() const { return word_hi16(words[4]); }
        uint16_t radar_mode() const { return word_lo16(words[4]); }

        uint16_t data_type() const { return word_hi16(words[5]); }
        uint16_t data_format_bits() const { return word_lo16(words[5]); }

        uint16_t extract_word() const { return word_hi16(words[6]); }
        uint16_t cfar_threshold() const { return word_lo16(words[6]); }

        uint16_t radar_angle_a_begin_raw() const { return word_hi16(words[7]); }
        uint16_t radar_angle_a_end_raw() const { return word_lo16(words[7]); }
        uint16_t radar_angle_e_begin_raw() const { return word_hi16(words[8]); }
        uint16_t radar_angle_e_end_raw() const { return word_lo16(words[8]); }
        uint16_t radar_angle_a_step_raw() const { return word_hi16(words[9]); }
        uint16_t radar_angle_e_step_raw() const { return word_lo16(words[9]); }
        uint16_t radar_angle_a_raw() const { return word_hi16(words[10]); }
        uint16_t radar_angle_e_raw() const { return word_lo16(words[10]); }

        float radar_angle_a_deg() const { return decode_radar_angle_deg(radar_angle_a_raw()); }
        float radar_angle_e_deg() const { return decode_radar_angle_deg(radar_angle_e_raw()); }

        bool has_fixed_head() const
        {
            return fixed_head() == RadarConfig::kPrtFixedHeader;
        }
    };

    inline PrtHeaderInfo decode_prt_header(const uint8_t *hdr)
    {
        PrtHeaderInfo info;
        for (std::size_t word_idx = 0; word_idx < PrtHeaderInfo::kWordCount; ++word_idx)
        {
            info.words[word_idx] = read_le_u32(hdr + word_idx * sizeof(uint32_t));
        }
        return info;
    }

    inline bool starts_with_prt_head(const uint8_t *hdr)
    {
        return read_le_u32(hdr) == RadarConfig::kPrtFixedHeader;
    }

    inline void resync_to_prt_head(std::vector<uint8_t> &buffer)
    {
        static constexpr uint8_t kPrtHeadLE[4] = {0x32, 0xCD, 0x55, 0xAA};

        auto it = std::search(buffer.begin(),
                              buffer.end(),
                              std::begin(kPrtHeadLE),
                              std::end(kPrtHeadLE));

        if (it == buffer.end())
        {
            if (buffer.size() > 3)
            {
                buffer.erase(buffer.begin(), buffer.end() - 3);
            }
            return;
        }

        if (it != buffer.begin())
        {
            buffer.erase(buffer.begin(), it);
        }
    }

} // namespace radar
