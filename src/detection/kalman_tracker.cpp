#include "ir_sim/detection/kalman_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace ir_sim::detection {

void MultiTargetTracker::KalmanFilter1D::predict(float dt, float q) {
    // State extrapolation: pos = pos + vel * dt
    pos += vel * dt;

    // Covariance extrapolation: P = F * P * F^T + Q
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;

    const float new_p00 = p00 + dt * (p10 + p01 + dt * p11) + q * (dt3 / 3.0f);
    const float new_p01 = p01 + dt * p11 + q * (dt2 * 0.5f);
    const float new_p10 = p10 + dt * p11 + q * (dt2 * 0.5f);
    const float new_p11 = p11 + q * dt;

    p00 = new_p00;
    p01 = new_p01;
    p10 = new_p10;
    p11 = new_p11;
}

void MultiTargetTracker::KalmanFilter1D::update(float z, float r) {
    // Innovation: y = z - H * pos
    const float y = z - pos;
    const float s = p00 + r;
    if (s <= 1.0e-6f) return;

    // Kalman Gain: K = P * H^T * inv(S)
    const float k0 = p00 / s;
    const float k1 = p10 / s;

    // State update: pos = pos + K * y
    pos += k0 * y;
    vel += k1 * y;

    // Covariance update: P = (I - K * H) * P
    const float old_p00 = p00;
    const float old_p01 = p01;

    p00 = (1.0f - k0) * old_p00;
    p01 = (1.0f - k0) * old_p01;
    p10 = p10 - k1 * old_p00;
    p11 = p11 - k1 * old_p01;
}

MultiTargetTracker::MultiTargetTracker()
    : MultiTargetTracker(TrackerParams{}) {}

MultiTargetTracker::MultiTargetTracker(TrackerParams params)
    : params_(params) {}

void MultiTargetTracker::reset() {
    tracks_.clear();
    published_tracks_.clear();
    next_track_id_ = 1;
}

size_t MultiTargetTracker::confirmed_track_count() const noexcept {
    size_t count = 0;
    for (const auto& t : tracks_) {
        if (t.state == TrackState::Confirmed) count++;
    }
    return count;
}

std::vector<Track> MultiTargetTracker::update(const std::vector<core::Detection>& detections, float dt) {
    if (dt <= 0.0f) dt = 0.0333f; // Default 30 Hz

    // 1. Predict all existing tracks forward using Kalman filter
    for (auto& track : tracks_) {
        track.kf_x.predict(dt, params_.process_noise_cov);
        track.kf_y.predict(dt, params_.process_noise_cov);
        track.age_frames++;
        track.time_since_update++;

        if (track.time_since_update > 0 && track.state == TrackState::Confirmed) {
            track.state = TrackState::Lost;
        }
    }

    // 2. Data Association: Match detections to existing tracks (Greedy Nearest Neighbor)
    std::vector<bool> det_matched(detections.size(), false);
    std::vector<bool> track_matched(tracks_.size(), false);

    const float max_dist_sq = params_.max_gating_distance_px * params_.max_gating_distance_px;

    // For each track, find closest detection
    for (size_t t = 0; t < tracks_.size(); ++t) {
        auto& track = tracks_[t];
        const float pred_x = track.kf_x.pos;
        const float pred_y = track.kf_y.pos;

        float best_dist_sq = max_dist_sq;
        int best_det_idx = -1;

        for (size_t d = 0; d < detections.size(); ++d) {
            if (det_matched[d]) continue;

            const auto& det = detections[d];
            const float det_cx = det.bbox.x + det.bbox.width * 0.5f;
            const float det_cy = det.bbox.y + det.bbox.height * 0.5f;

            const float dx = det_cx - pred_x;
            const float dy = det_cy - pred_y;
            const float dist_sq = dx * dx + dy * dy;

            if (dist_sq < best_dist_sq) {
                best_dist_sq = dist_sq;
                best_det_idx = static_cast<int>(d);
            }
        }

        if (best_det_idx >= 0) {
            // Update track with measurement
            det_matched[static_cast<size_t>(best_det_idx)] = true;
            track_matched[t] = true;

            const auto& det = detections[static_cast<size_t>(best_det_idx)];
            const float det_cx = det.bbox.x + det.bbox.width * 0.5f;
            const float det_cy = det.bbox.y + det.bbox.height * 0.5f;

            track.kf_x.update(det_cx, params_.measurement_noise_cov);
            track.kf_y.update(det_cy, params_.measurement_noise_cov);

            track.width = det.bbox.width;
            track.height = det.bbox.height;
            track.scr = det.scr;
            track.total_hits++;
            track.consecutive_hits++;
            track.time_since_update = 0;

            if (track.consecutive_hits >= params_.min_hits_to_confirm) {
                track.state = TrackState::Confirmed;
            }

            // Record history trail
            track.history.push_back({
                static_cast<int32_t>(track.kf_x.pos),
                static_cast<int32_t>(track.kf_y.pos)
            });
            if (track.history.size() > 30) {
                track.history.pop_front();
            }
        } else {
            // Missed detection on this frame
            track.consecutive_hits = 0;
        }
    }

    // 3. Initiate new tentative tracks for unmatched detections
    for (size_t d = 0; d < detections.size(); ++d) {
        if (!det_matched[d]) {
            const auto& det = detections[d];
            const float cx = det.bbox.x + det.bbox.width * 0.5f;
            const float cy = det.bbox.y + det.bbox.height * 0.5f;

            TargetTrackInternal new_track{};
            new_track.id = next_track_id_++;
            new_track.state = TrackState::Tentative;
            new_track.kf_x.pos = cx;
            new_track.kf_y.pos = cy;
            new_track.width = det.bbox.width;
            new_track.height = det.bbox.height;
            new_track.scr = det.scr;
            new_track.history.push_back({static_cast<int32_t>(cx), static_cast<int32_t>(cy)});

            tracks_.push_back(std::move(new_track));
        }
    }

    // 4. Prune tracks that have coasted longer than max_frames_to_coast
    std::erase_if(tracks_, [this](const TargetTrackInternal& t) {
        return t.time_since_update > params_.max_frames_to_coast;
    });

    // 5. Output active tracks for rendering and telemetry
    published_tracks_.clear();
    published_tracks_.reserve(tracks_.size());

    for (const auto& t : tracks_) {
        Track pub_track{};
        pub_track.track_id = t.id;
        pub_track.state = t.state;
        pub_track.bbox.x = t.kf_x.pos - t.width * 0.5f;
        pub_track.bbox.y = t.kf_y.pos - t.height * 0.5f;
        pub_track.bbox.width = t.width;
        pub_track.bbox.height = t.height;
        pub_track.bbox.score = (t.state == TrackState::Confirmed) ? 1.0f : 0.5f;
        pub_track.velocity_pxps = {t.kf_x.vel, t.kf_y.vel, 0.0f};
        pub_track.scr = t.scr;
        pub_track.total_hits = t.total_hits;
        pub_track.consecutive_hits = t.consecutive_hits;
        pub_track.age_frames = t.age_frames;
        pub_track.time_since_update = t.time_since_update;
        pub_track.history = t.history;

        published_tracks_.push_back(std::move(pub_track));
    }

    return published_tracks_;
}

} // namespace ir_sim::detection
