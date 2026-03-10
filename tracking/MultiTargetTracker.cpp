#include "tracking/MultiTargetTracker.h"

#include <algorithm>
#include <cmath>

namespace radar
{
    namespace
    {
        constexpr float kMinVariance = 1e-6f;
        constexpr float kMinDet = 1e-9f;

        inline float sq(float x)
        {
            return x * x;
        }

        struct AssocCandidate
        {
            int track_idx = -1;
            int det_idx = -1;
            float cost = 0.0f;
        };
    } // namespace

    MultiTargetTracker::MultiTargetTracker(const RadarConfig &cfg)
        : cfg_(cfg)
    {
    }

    void MultiTargetTracker::reset()
    {
        tracks_.clear();
        next_track_id_ = 1;
    }

    void MultiTargetTracker::predict_track(TrackState &track, float dt) const
    {
        // 2D CV model in range-velocity space.
        track.x_range_m += track.x_velocity_mps * dt;
        track.x_range_m = std::max(track.x_range_m, 0.0f);

        const float p00 = track.p00;
        const float p01 = track.p01;
        const float p10 = track.p10;
        const float p11 = track.p11;

        const float q_range = sq(std::max(cfg_.tracking_process_noise_range_m, 1e-3f));
        const float q_velocity = sq(std::max(cfg_.tracking_process_noise_velocity_mps, 1e-3f));

        track.p00 = p00 + dt * (p01 + p10) + dt * dt * p11 + q_range;
        track.p01 = p01 + dt * p11;
        track.p10 = p10 + dt * p11;
        track.p11 = p11 + q_velocity;

        track.p00 = std::max(track.p00, kMinVariance);
        track.p11 = std::max(track.p11, kMinVariance);
    }

    void MultiTargetTracker::update_track(TrackState &track, const RadarTarget &det) const
    {
        const float z0 = det.range_m;
        const float z1 = det.velocity_mps;

        const float r00 = sq(std::max(cfg_.tracking_measurement_noise_range_m, 1e-3f));
        const float r11 = sq(std::max(cfg_.tracking_measurement_noise_velocity_mps, 1e-3f));

        const float s00 = track.p00 + r00;
        const float s01 = track.p01;
        const float s10 = track.p10;
        const float s11 = track.p11 + r11;

        const float det_s = s00 * s11 - s01 * s10;
        const float inv_det = 1.0f / ((std::fabs(det_s) < kMinDet) ? (det_s < 0.0f ? -kMinDet : kMinDet) : det_s);

        const float is00 = s11 * inv_det;
        const float is01 = -s01 * inv_det;
        const float is10 = -s10 * inv_det;
        const float is11 = s00 * inv_det;

        const float k00 = track.p00 * is00 + track.p01 * is10;
        const float k01 = track.p00 * is01 + track.p01 * is11;
        const float k10 = track.p10 * is00 + track.p11 * is10;
        const float k11 = track.p10 * is01 + track.p11 * is11;

        const float y0 = z0 - track.x_range_m;
        const float y1 = z1 - track.x_velocity_mps;

        track.x_range_m += k00 * y0 + k01 * y1;
        track.x_velocity_mps += k10 * y0 + k11 * y1;
        track.x_range_m = std::max(track.x_range_m, 0.0f);

        const float i_k00 = 1.0f - k00;
        const float i_k01 = -k01;
        const float i_k10 = -k10;
        const float i_k11 = 1.0f - k11;

        const float p00 = i_k00 * track.p00 + i_k01 * track.p10;
        const float p01 = i_k00 * track.p01 + i_k01 * track.p11;
        const float p10 = i_k10 * track.p00 + i_k11 * track.p10;
        const float p11 = i_k10 * track.p01 + i_k11 * track.p11;

        track.p00 = std::max(p00, kMinVariance);
        track.p01 = 0.5f * (p01 + p10);
        track.p10 = track.p01;
        track.p11 = std::max(p11, kMinVariance);

        track.last_target = det;
        track.last_target.range_m = track.x_range_m;
        track.last_target.velocity_mps = track.x_velocity_mps;
    }

    int MultiTargetTracker::update(const std::vector<RadarTarget> &detections,
                                   std::vector<TrackedTarget> &out_tracks)
    {
        out_tracks.clear();

        const float dt = std::max(cfg_.tracking_frame_dt_s, 1e-3f);
        const float gate_range = std::max(cfg_.tracking_gate_range_m, 1e-3f);
        const float gate_velocity = std::max(cfg_.tracking_gate_velocity_mps, 1e-3f);
        const float spawn_min_snr = cfg_.tracking_spawn_min_snr_db;
        const float spawn_excl_range = std::max(cfg_.tracking_spawn_exclusion_range_m, 0.0f);
        const float spawn_excl_velocity = std::max(cfg_.tracking_spawn_exclusion_velocity_mps, 0.0f);

        for (auto &track : tracks_)
        {
            track.updated_in_frame = false;
            predict_track(track, dt);
            track.age++;
        }

        std::vector<int> track_assigned_det(tracks_.size(), -1);
        std::vector<int> det_assigned_track(detections.size(), -1);
        std::vector<AssocCandidate> cands;
        cands.reserve(tracks_.size() * detections.size());

        for (int ti = 0; ti < static_cast<int>(tracks_.size()); ++ti)
        {
            const auto &track = tracks_[ti];
            for (int di = 0; di < static_cast<int>(detections.size()); ++di)
            {
                const auto &det = detections[di];
                const float dr = std::fabs(det.range_m - track.x_range_m);
                const float dv = std::fabs(det.velocity_mps - track.x_velocity_mps);
                if (dr > gate_range || dv > gate_velocity)
                {
                    continue;
                }

                const float cost =
                    sq(dr / gate_range) +
                    sq(dv / gate_velocity);

                AssocCandidate c;
                c.track_idx = ti;
                c.det_idx = di;
                c.cost = cost;
                cands.push_back(c);
            }
        }

        std::sort(cands.begin(), cands.end(), [](const AssocCandidate &a, const AssocCandidate &b)
                  { return a.cost < b.cost; });

        for (const auto &c : cands)
        {
            if (track_assigned_det[c.track_idx] >= 0)
            {
                continue;
            }
            if (det_assigned_track[c.det_idx] >= 0)
            {
                continue;
            }
            track_assigned_det[c.track_idx] = c.det_idx;
            det_assigned_track[c.det_idx] = c.track_idx;
        }

        for (int ti = 0; ti < static_cast<int>(tracks_.size()); ++ti)
        {
            auto &track = tracks_[ti];
            const int di = track_assigned_det[ti];
            if (di >= 0)
            {
                update_track(track, detections[di]);
                track.hit_count++;
                track.missed_frames = 0;
                track.updated_in_frame = true;
                if (track.hit_count >= std::max(cfg_.tracking_confirm_hits, 1))
                {
                    track.confirmed = true;
                }
            }
            else
            {
                track.missed_frames++;
            }
        }

        const float init_r_var = sq(std::max(cfg_.tracking_measurement_noise_range_m, 1e-3f));
        const float init_v_var = sq(std::max(cfg_.tracking_measurement_noise_velocity_mps, 1e-3f));

        for (int di = 0; di < static_cast<int>(detections.size()); ++di)
        {
            if (det_assigned_track[di] >= 0)
            {
                continue;
            }

            const auto &det = detections[di];
            if (det.snr_db < spawn_min_snr)
            {
                continue;
            }

            if (spawn_excl_range > 0.0f || spawn_excl_velocity > 0.0f)
            {
                bool too_close = false;
                for (const auto &track : tracks_)
                {
                    const float dr = std::fabs(det.range_m - track.x_range_m);
                    const float dv = std::fabs(det.velocity_mps - track.x_velocity_mps);

                    const bool in_range_gate = (spawn_excl_range <= 0.0f) || (dr <= spawn_excl_range);
                    const bool in_velocity_gate = (spawn_excl_velocity <= 0.0f) || (dv <= spawn_excl_velocity);
                    if (in_range_gate && in_velocity_gate)
                    {
                        too_close = true;
                        break;
                    }
                }
                if (too_close)
                {
                    continue;
                }
            }

            if (cfg_.tracking_max_tracks > 0 &&
                static_cast<int>(tracks_.size()) >= cfg_.tracking_max_tracks)
            {
                break;
            }

            TrackState t;
            t.id = next_track_id_++;
            t.x_range_m = std::max(det.range_m, 0.0f);
            t.x_velocity_mps = det.velocity_mps;
            t.p00 = init_r_var;
            t.p01 = 0.0f;
            t.p10 = 0.0f;
            t.p11 = init_v_var;
            t.last_target = det;
            t.last_target.range_m = t.x_range_m;
            t.age = 1;
            t.hit_count = 1;
            t.missed_frames = 0;
            t.confirmed = (t.hit_count >= std::max(cfg_.tracking_confirm_hits, 1));
            t.updated_in_frame = true;
            tracks_.push_back(t);
        }

        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                           [this](const TrackState &t)
                           {
                               return t.missed_frames > std::max(cfg_.tracking_max_missed_frames, 0);
                           }),
            tracks_.end());

        out_tracks.reserve(tracks_.size());
        for (const auto &t : tracks_)
        {
            if (!t.confirmed && !cfg_.tracking_output_tentative)
            {
                continue;
            }
            if (cfg_.tracking_output_only_updated_tracks && !t.updated_in_frame)
            {
                continue;
            }
            if (t.age < std::max(cfg_.tracking_output_min_age, 1))
            {
                continue;
            }
            if (t.hit_count < std::max(cfg_.tracking_output_min_hits, 1))
            {
                continue;
            }

            TrackedTarget out;
            out.track_id = t.id;
            out.target = t.last_target;
            out.filtered_range_m = t.x_range_m;
            out.filtered_velocity_mps = t.x_velocity_mps;
            out.target.range_m = t.x_range_m;
            out.target.velocity_mps = t.x_velocity_mps;
            out.age = t.age;
            out.hit_count = t.hit_count;
            out.missed_frames = t.missed_frames;
            out.confirmed = t.confirmed ? 1 : 0;
            out_tracks.push_back(out);
        }

        std::sort(out_tracks.begin(), out_tracks.end(),
                  [](const TrackedTarget &a, const TrackedTarget &b)
                  { return a.track_id < b.track_id; });

        return static_cast<int>(out_tracks.size());
    }

} // namespace radar
