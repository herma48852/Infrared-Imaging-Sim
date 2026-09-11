#include "test_main.hpp"
#include "ir_sim/core/frame_buffer.hpp"
#include "ir_sim/detection/cfar_2d.hpp"
#include "ir_sim/detection/kalman_tracker.hpp"
#include "ir_sim/detection/point_target_lcm.hpp"

#include <random>

using namespace ir_sim::core;
using namespace ir_sim::detection;

TEST_CASE(lcm_point_target_saliency_isolation) {
    PointTargetLCM lcm(64, 64);

    RawFrame14Bit frame(64, 64, 5000); // Uniform background

    // Inject dim point target at (32, 32)
    frame(32, 32) = 5400;

    std::vector<float> saliency(64 * 64, 0.0f);
    lcm.compute_saliency_map(frame.as_span(), saliency, 3);

    // Peak saliency must be at target position (32, 32)
    const float target_saliency = saliency[32 * 64 + 32];
    REQUIRE(target_saliency > 50.0f);

    // Background should have zero saliency response
    REQUIRE(saliency[10 * 64 + 10] == 0.0f);
    REQUIRE(saliency[50 * 64 + 50] == 0.0f);
}

TEST_CASE(cfar_detection_at_varying_scr) {
    CFAR2D cfar(100, 100);

    RawFrame14Bit frame(100, 100);
    // Background noise: mean 5000, std dev 40
    std::mt19937 rng(42);
    std::normal_distribution<float> noise(5000.0f, 40.0f);

    for (size_t i = 0; i < 10000; ++i) {
        frame.as_span()[i] = static_cast<uint16_t>(noise(rng));
    }

    // Target 1 at (30, 30): SCR = 6.0 -> intensity = 5000 + 6 * 40 = 5240
    frame(30, 30) = 5240;

    // Target 2 at (70, 70): SCR = 4.0 -> intensity = 5000 + 4 * 40 = 5160
    frame(70, 70) = 5160;

    const auto detections = cfar.detect(frame.as_span());

    // Both targets must be detected
    REQUIRE(detections.size() >= 2);

    bool found_t1 = false;
    bool found_t2 = false;

    for (const auto& det : detections) {
        if (std::abs(det.bbox.x - 30.0f) <= 2.0f && std::abs(det.bbox.y - 30.0f) <= 2.0f) {
            found_t1 = true;
            REQUIRE(det.scr >= 4.5f); // High SCR
        }
        if (std::abs(det.bbox.x - 70.0f) <= 2.0f && std::abs(det.bbox.y - 70.0f) <= 2.0f) {
            found_t2 = true;
            REQUIRE(det.scr >= 3.0f);
        }
    }

    REQUIRE(found_t1);
    REQUIRE(found_t2);
}

TEST_CASE(kalman_track_continuity_and_velocity_estimation) {
    MultiTargetTracker tracker;

    // Simulate target moving along x with velocity 20 px/s (2 px per 0.1s frame)
    float true_x = 50.0f;
    float true_y = 50.0f;
    constexpr float dt = 0.1f;
    constexpr float vx = 20.0f;

    uint32_t persistent_id = 0;

    for (int frame = 0; frame < 10; ++frame) {
        true_x += vx * dt;

        Detection det{};
        det.bbox.x = true_x;
        det.bbox.y = true_y;
        det.bbox.width = 4.0f;
        det.bbox.height = 4.0f;
        det.scr = 5.5f;

        const auto tracks = tracker.update({det}, dt);

        REQUIRE(tracks.size() == 1);
        if (frame == 0) {
            persistent_id = tracks[0].track_id;
            REQUIRE(tracks[0].state == TrackState::Tentative);
        } else {
            // Track ID must remain continuous
            REQUIRE(tracks[0].track_id == persistent_id);
        }

        if (frame >= 3) {
            // Track confirmed after 3 hits
            REQUIRE(tracks[0].state == TrackState::Confirmed);
        }
    }

    // Velocity should be estimated near 20 px/s
    const auto confirmed_tracks = tracker.active_tracks();
    REQUIRE(confirmed_tracks.size() == 1);
    REQUIRE_NEAR(confirmed_tracks[0].velocity_pxps.x, 20.0f, 4.0f);
    REQUIRE_NEAR(confirmed_tracks[0].velocity_pxps.y, 0.0f, 2.0f);
}

TEST_CASE(kalman_track_coasting_and_pruning) {
    TrackerParams params;
    params.min_hits_to_confirm = 2;
    params.max_frames_to_coast = 3;

    MultiTargetTracker tracker(params);

    // 1. Establish track
    Detection det{};
    det.bbox.x = 100.0f;
    det.bbox.y = 100.0f;
    det.bbox.width = 5.0f;
    det.bbox.height = 5.0f;

    tracker.update({det}, 0.1f);
    auto tracks = tracker.update({det}, 0.1f);
    REQUIRE(tracks.size() == 1);
    REQUIRE(tracks[0].state == TrackState::Confirmed);

    // 2. Target drops out for 2 frames (coasting)
    tracks = tracker.update({}, 0.1f);
    REQUIRE(tracks.size() == 1);
    REQUIRE(tracks[0].state == TrackState::Lost);

    tracks = tracker.update({}, 0.1f);
    REQUIRE(tracks.size() == 1);
    REQUIRE(tracks[0].state == TrackState::Lost);

    // 3. Drop out for 2 more frames (exceeds max_frames_to_coast = 3)
    tracker.update({}, 0.1f);
    tracks = tracker.update({}, 0.1f);

    // Track should now be pruned / deleted
    REQUIRE(tracks.empty());
}
