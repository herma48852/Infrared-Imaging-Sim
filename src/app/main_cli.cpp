#include "ir_sim/app/simulation_pipeline.hpp"
#include "ir_sim/scene/scenario_manifest.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>

using namespace ir_sim;
using namespace ir_sim::app;

namespace {

void print_cli_usage(const char* prog) {
    std::cout << "===============================================================\n"
              << " Antigravity IR Imaging & Tactical Detection Simulation (CLI)\n"
              << "===============================================================\n\n"
              << "Usage: " << prog << " [options]\n\n"
              << "Options:\n"
              << "  --scenario <path>     Path to scenario manifest JSON\n"
              << "  --duration <seconds>  Simulation run duration (default: 5.0s)\n"
              << "  --fps <hz>            Simulation tick rate (default: 30)\n"
              << "  --csv <filepath>      Export per-frame telemetry to CSV\n"
              << "  --mode <orbit|waypoint|pursuit>  Flight mode\n"
              << "  --benchmark           Run full performance profiler & ROC evaluation\n"
              << "  --help                Show this message\n";
}

} // namespace

int main(int argc, char* argv[]) {
    std::string scenario_path = "scenarios/desert_convoy_day.json";
    std::string csv_path = "";
    float duration_sec = 5.0f;
    float fps = 30.0f;
    std::string flight_mode_str = "orbit";
    bool benchmark = true;

    for (int i = 1; i < argc; ++i) {
        std::string_view arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            print_cli_usage(argv[0]);
            return 0;
        } else if (arg == "--scenario" && i + 1 < argc) {
            scenario_path = argv[++i];
        } else if (arg == "--duration" && i + 1 < argc) {
            duration_sec = std::stof(argv[++i]);
        } else if (arg == "--fps" && i + 1 < argc) {
            fps = std::stof(argv[++i]);
        } else if (arg == "--csv" && i + 1 < argc) {
            csv_path = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            flight_mode_str = argv[++i];
        } else if (arg == "--benchmark") {
            benchmark = true;
        }
    }
    (void)benchmark;

    std::cout << "===============================================================\n"
              << " Loading Scenario: " << scenario_path << "\n"
              << "===============================================================\n";

    auto manifest_opt = scene::ScenarioManifest::load_from_file(scenario_path);
    if (!manifest_opt) {
        std::cerr << "Warning: Failed to load " << scenario_path << ", generating default desert convoy.\n";
        manifest_opt = scene::ScenarioManifest{};
        manifest_opt->scenario_name = "desert_convoy_fallback";
    }

    const auto& manifest = *manifest_opt;
    std::cout << "  Scenario Name:  " << manifest.scenario_name << "\n"
              << "  Ambient Temp:   " << manifest.environment.ambient_temp_k << " K\n"
              << "  Solar Flux:     " << manifest.environment.solar_irradiance_w_m2 << " W/m^2\n"
              << "  Terrain Type:   " << manifest.terrain.terrain_type << "\n"
              << "  Target Count:   " << manifest.targets.size() << "\n\n";

    SimulationPipeline pipeline(manifest, 640, 512);

    if (flight_mode_str == "waypoint") {
        pipeline.set_flight_mode(FlightMode::Waypoint);
    } else if (flight_mode_str == "pursuit") {
        pipeline.set_flight_mode(FlightMode::Pursuit);
    } else {
        pipeline.set_flight_mode(FlightMode::Orbit);
    }

    std::ofstream csv_out;
    if (!csv_path.empty()) {
        csv_out.open(csv_path);
        if (csv_out.is_open()) {
            csv_out << "frame,time_sec,drone_lat,drone_lon,drone_alt_m,speed_mps,"
                    << "gimbal_pitch_deg,gt_count,det_count,track_count,latency_total_ms,"
                    << "latency_fpa_ms,latency_isp_ms,latency_cfar_ms,latency_track_ms\n";
        }
    }

    const float dt = 1.0f / fps;
    const int total_frames = static_cast<int>(duration_sec * fps);

    std::cout << "[ir_sim_cli] Executing " << total_frames << " frames at " << fps << " Hz (" << duration_sec << " s)...\n";

    size_t total_gt_occurrences = 0;
    size_t total_detections = 0;
    size_t total_true_positives = 0;
    size_t total_false_alarms = 0;

    float sum_scene_ms = 0.0f;
    float sum_fpa_ms = 0.0f;
    float sum_isp_ms = 0.0f;
    float sum_cfar_ms = 0.0f;
    float sum_track_ms = 0.0f;
    float sum_total_ms = 0.0f;

    for (int frame = 0; frame < total_frames; ++frame) {
        pipeline.step(dt);
        const auto& telem = pipeline.telemetry();
        const auto& lat = telem.latency;

        sum_scene_ms += lat.scene_ms;
        sum_fpa_ms += lat.fpa_ms;
        sum_isp_ms += lat.isp_ms;
        sum_cfar_ms += lat.cfar_ms;
        sum_track_ms += lat.tracker_ms;
        sum_total_ms += lat.total_ms;

        // Association for ROC calculation: match detections to ground truth
        const auto& gt = pipeline.ground_truth_detections();
        const auto& dets = pipeline.current_detections();

        total_gt_occurrences += gt.size();
        total_detections += dets.size();

        std::vector<bool> gt_matched(gt.size(), false);
        for (const auto& det : dets) {
            const float dcx = det.bbox.x + det.bbox.width * 0.5f;
            const float dcy = det.bbox.y + det.bbox.height * 0.5f;

            bool matched = false;
            for (size_t g = 0; g < gt.size(); ++g) {
                if (gt_matched[g]) continue;
                const float gcx = gt[g].bbox.x + gt[g].bbox.width * 0.5f;
                const float gcy = gt[g].bbox.y + gt[g].bbox.height * 0.5f;

                const float dist = std::sqrt((dcx - gcx) * (dcx - gcx) + (dcy - gcy) * (dcy - gcy));
                const float gate_r = std::max(15.0f, std::max(gt[g].bbox.width, gt[g].bbox.height) * 0.75f);
                if (dist <= gate_r) {
                    gt_matched[g] = true;
                    matched = true;
                    total_true_positives++;
                    break;
                }
            }
            if (!matched) {
                total_false_alarms++;
            }
        }

        if (csv_out.is_open()) {
            csv_out << frame << "," << std::fixed << std::setprecision(3)
                    << telem.sim_time_sec << ","
                    << std::setprecision(6) << telem.drone_gps.latitude_deg << ","
                    << telem.drone_gps.longitude_deg << ","
                    << std::setprecision(2) << telem.altitude_agl_m << ","
                    << telem.ground_speed_mps << ","
                    << pipeline.gimbal().current_pitch_deg() << ","
                    << telem.ground_truth_count << ","
                    << telem.detection_count << ","
                    << telem.confirmed_track_count << ","
                    << lat.total_ms << ","
                    << lat.fpa_ms << ","
                    << lat.isp_ms << ","
                    << lat.cfar_ms << ","
                    << lat.tracker_ms << "\n";
        }

        if (frame % 30 == 0) {
            std::cout << "  Frame " << std::setw(4) << frame << " / " << total_frames
                      << " | SimTime: " << std::fixed << std::setprecision(1) << telem.sim_time_sec << "s"
                      << " | Alt: " << std::setw(5) << telem.altitude_agl_m << "m"
                      << " | GT: " << telem.ground_truth_count
                      << " | Dets: " << telem.detection_count
                      << " | Tracks: " << telem.confirmed_track_count
                      << " | Step: " << std::setprecision(2) << lat.total_ms << " ms\n";
        }
    }

    if (csv_out.is_open()) {
        csv_out.close();
        std::cout << "\n[ir_sim_cli] Telemetry successfully exported to: " << csv_path << "\n";
    }

    // Performance & ROC Summary
    const float inv_n = (total_frames > 0) ? (1.0f / static_cast<float>(total_frames)) : 1.0f;
    const float pd = (total_gt_occurrences > 0)
        ? (static_cast<float>(total_true_positives) / static_cast<float>(total_gt_occurrences))
        : 1.0f;
    const float pfa_per_frame = static_cast<float>(total_false_alarms) * inv_n;
    const float mean_total_ms = sum_total_ms * inv_n;
    const float effective_fps = (mean_total_ms > 0.0f) ? (1000.0f / mean_total_ms) : 0.0f;

    std::cout << "\n===============================================================\n"
              << " PERFORMANCE PROFILER & DETECTION BENCHMARK SUMMARY\n"
              << "===============================================================\n"
              << " Total Frames Executed:   " << total_frames << "\n"
              << " Ground Truth Targets:    " << total_gt_occurrences << "\n"
              << " Total System Detections: " << total_detections << "\n"
              << " True Positives:          " << total_true_positives << "\n"
              << " Probability of Det (Pd): " << std::fixed << std::setprecision(4) << pd * 100.0f << " %\n"
              << " False Alarms / Frame:    " << std::setprecision(4) << pfa_per_frame << "\n"
              << " Active Confirmed Tracks: " << pipeline.tracker().confirmed_track_count() << "\n\n"
              << " LATENCY BREAKDOWN (Per 640x512 Frame on Apple Silicon):\n"
              << "   1. Scene Radiative Transfer: " << std::setw(6) << std::setprecision(3) << (sum_scene_ms * inv_n) << " ms\n"
              << "   2. FPA Sensor Transduction:  " << std::setw(6) << std::setprecision(3) << (sum_fpa_ms * inv_n) << " ms\n"
              << "   3. Embedded ISP Pipeline:    " << std::setw(6) << std::setprecision(3) << (sum_isp_ms * inv_n) << " ms\n"
              << "   4. CFAR / LCM Detection:     " << std::setw(6) << std::setprecision(3) << (sum_cfar_ms * inv_n) << " ms\n"
              << "   5. Multi-Target Kalman:      " << std::setw(6) << std::setprecision(3) << (sum_track_ms * inv_n) << " ms\n"
              << "   -----------------------------------------\n"
              << "   TOTAL PIPELINE LATENCY:      " << std::setw(6) << std::setprecision(3) << mean_total_ms << " ms\n"
              << "   MAX THEORETICAL THROUGHPUT:  " << std::setw(6) << std::setprecision(1) << effective_fps << " FPS\n"
              << "===============================================================\n";

    return 0;
}
