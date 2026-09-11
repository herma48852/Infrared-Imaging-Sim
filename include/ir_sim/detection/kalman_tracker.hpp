#pragma once

#include "ir_sim/core/types.hpp"

#include <cstdint>
#include <deque>
#include <vector>

namespace ir_sim::detection {

enum class TrackState {
    Tentative,  // New track, needs consecutive hits to confirm
    Confirmed,  // Actively tracked, published to telemetry/HUD
    Lost        // Temporarily missed, coasting on Kalman prediction
};

struct Track {
    uint32_t track_id{0};
    TrackState state{TrackState::Tentative};
    core::BoundingBox bbox{};
    core::Vec3 velocity_pxps{0.0f, 0.0f, 0.0f}; // [vx, vy, 0] in pixels/sec
    float scr{0.0f};
    int total_hits{0};
    int consecutive_hits{0};
    int age_frames{0};
    int time_since_update{0};
    std::deque<core::PixelCoord> history; // Recent centroid positions for trajectory trail

    // Estimated ground GPS position
    core::GeoCoordinate estimated_geo{};
};

struct TrackerParams {
    int min_hits_to_confirm{3};    // Require 3 consecutive hits to declare track
    int max_frames_to_coast{5};    // Coast for up to 5 missed frames before deletion
    float max_gating_distance_px{30.0f}; // Max Euclidean distance for measurement association
    float process_noise_cov{4.0f}; // Q covariance
    float measurement_noise_cov{2.0f}; // R covariance
};

/**
 * @brief Multi-Target Kalman Filter Tracker (aerospace SORT / ByteTrack standard).
 * 
 * Tracks multiple simultaneous targets in real time. Maintains track identity,
 * filters measurement jitter, estimates velocity vectors, and coasts through
 * temporary occlusions or low-SCR dropouts.
 */
class MultiTargetTracker {
public:
    MultiTargetTracker();
    explicit MultiTargetTracker(TrackerParams params);

    /**
     * @brief Steps the tracker forward with new frame detections.
     * @param detections List of current frame detections from CFAR / LCM.
     * @param dt Time delta between frames [s].
     * @return List of currently confirmed target tracks.
     */
    std::vector<Track> update(const std::vector<core::Detection>& detections, float dt);

    void reset();

    [[nodiscard]] const std::vector<Track>& active_tracks() const noexcept { return published_tracks_; }
    [[nodiscard]] size_t confirmed_track_count() const noexcept;

private:
    struct KalmanFilter1D {
        float pos{0.0f}; // Position
        float vel{0.0f}; // Velocity
        float p00{10.0f}, p01{0.0f}, p10{0.0f}, p11{10.0f}; // Covariance P

        void predict(float dt, float q);
        void update(float z, float r);
    };

    struct TargetTrackInternal {
        uint32_t id{0};
        TrackState state{TrackState::Tentative};
        KalmanFilter1D kf_x;
        KalmanFilter1D kf_y;
        float width{10.0f};
        float height{10.0f};
        float scr{0.0f};
        int total_hits{1};
        int consecutive_hits{1};
        int age_frames{1};
        int time_since_update{0};
        std::deque<core::PixelCoord> history;
    };

    TrackerParams params_{};
    uint32_t next_track_id_{1};
    std::vector<TargetTrackInternal> tracks_;
    std::vector<Track> published_tracks_;
};

} // namespace ir_sim::detection
