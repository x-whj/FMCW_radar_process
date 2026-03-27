#pragma once

#include <cstdint>
#include <vector>

#include "model/RadarConfig.h"
#include "model/TargetTypes.h"

namespace radar
{

    struct TrackedTarget
    {
        uint32_t track_id = 0;
        RadarTarget target{};
        float filtered_range_m = 0.0f;
        float filtered_velocity_mps = 0.0f;
        int age = 0;
        int hit_count = 0;
        int missed_frames = 0;
        uint8_t confirmed = 0;
    };

    class MultiTargetTracker
    {
    public:
        explicit MultiTargetTracker(const RadarConfig &cfg);

        void reset();

        int update(const std::vector<RadarTarget> &detections,
                   std::vector<TrackedTarget> &out_tracks);

    private:
        struct TrackState
        {
            uint32_t id = 0;
            float x_range_m = 0.0f;
            float x_velocity_mps = 0.0f;
            float p00 = 0.0f;
            float p01 = 0.0f;
            float p10 = 0.0f;
            float p11 = 0.0f;
            RadarTarget last_target{};
            int age = 0;
            int hit_count = 0;
            int missed_frames = 0;
            bool confirmed = false;
            bool updated_in_frame = false;
        };

        RadarConfig cfg_;
        uint32_t next_track_id_ = 1;
        std::vector<TrackState> tracks_;

        void predict_track(TrackState &track, float dt) const;
        void update_track(TrackState &track, const RadarTarget &det) const;
    };

} // namespace radar
