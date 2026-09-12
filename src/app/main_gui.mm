#import <Cocoa/Cocoa.h>
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>

#define GLFW_INCLUDE_NONE
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_metal.h"
#include "implot.h"

#include "ir_sim/app/metal_texture_renderer.hpp"
#include "ir_sim/app/simulation_pipeline.hpp"
#include "ir_sim/scene/scenario_manifest.hpp"

#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace ir_sim;
using namespace ir_sim::app;

namespace {

void apply_tactical_defense_theme() {
    ImGuiStyle& style = ImGui::GetStyle();
    ImVec4* colors = style.Colors;

    // Defense dark slate aesthetic
    colors[ImGuiCol_WindowBg]             = ImVec4(0.08f, 0.09f, 0.11f, 0.96f);
    colors[ImGuiCol_ChildBg]              = ImVec4(0.10f, 0.11f, 0.13f, 1.00f);
    colors[ImGuiCol_PopupBg]              = ImVec4(0.08f, 0.09f, 0.11f, 0.98f);
    colors[ImGuiCol_Border]               = ImVec4(0.20f, 0.24f, 0.28f, 0.60f);
    colors[ImGuiCol_BorderShadow]         = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_FrameBg]              = ImVec4(0.14f, 0.16f, 0.19f, 1.00f);
    colors[ImGuiCol_FrameBgHovered]       = ImVec4(0.18f, 0.22f, 0.26f, 1.00f);
    colors[ImGuiCol_FrameBgActive]        = ImVec4(0.22f, 0.28f, 0.33f, 1.00f);
    colors[ImGuiCol_TitleBg]              = ImVec4(0.09f, 0.11f, 0.13f, 1.00f);
    colors[ImGuiCol_TitleBgActive]        = ImVec4(0.12f, 0.15f, 0.18f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed]     = ImVec4(0.06f, 0.07f, 0.09f, 1.00f);
    colors[ImGuiCol_MenuBarBg]            = ImVec4(0.10f, 0.12f, 0.14f, 1.00f);
    colors[ImGuiCol_ScrollbarBg]          = ImVec4(0.08f, 0.09f, 0.11f, 0.60f);
    colors[ImGuiCol_ScrollbarGrab]        = ImVec4(0.22f, 0.26f, 0.30f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.30f, 0.35f, 0.40f, 1.00f);
    colors[ImGuiCol_ScrollbarGrabActive]  = ImVec4(0.38f, 0.44f, 0.50f, 1.00f);
    colors[ImGuiCol_CheckMark]            = ImVec4(0.20f, 0.85f, 0.50f, 1.00f);
    colors[ImGuiCol_SliderGrab]           = ImVec4(0.25f, 0.65f, 0.90f, 1.00f);
    colors[ImGuiCol_SliderGrabActive]     = ImVec4(0.35f, 0.75f, 1.00f, 1.00f);
    colors[ImGuiCol_Button]               = ImVec4(0.16f, 0.20f, 0.25f, 1.00f);
    colors[ImGuiCol_ButtonHovered]        = ImVec4(0.24f, 0.30f, 0.38f, 1.00f);
    colors[ImGuiCol_ButtonActive]         = ImVec4(0.18f, 0.45f, 0.65f, 1.00f);
    colors[ImGuiCol_Header]               = ImVec4(0.18f, 0.23f, 0.28f, 1.00f);
    colors[ImGuiCol_HeaderHovered]        = ImVec4(0.24f, 0.30f, 0.37f, 1.00f);
    colors[ImGuiCol_HeaderActive]         = ImVec4(0.20f, 0.42f, 0.60f, 1.00f);
    colors[ImGuiCol_Separator]            = ImVec4(0.20f, 0.24f, 0.28f, 0.70f);
    colors[ImGuiCol_Tab]                  = ImVec4(0.12f, 0.15f, 0.18f, 1.00f);
    colors[ImGuiCol_TabHovered]           = ImVec4(0.24f, 0.30f, 0.38f, 1.00f);
    colors[ImGuiCol_TabActive]            = ImVec4(0.18f, 0.25f, 0.32f, 1.00f);
    colors[ImGuiCol_Text]                 = ImVec4(0.92f, 0.94f, 0.96f, 1.00f);
    colors[ImGuiCol_TextDisabled]         = ImVec4(0.50f, 0.55f, 0.60f, 1.00f);

    style.WindowRounding    = 4.0f;
    style.ChildRounding     = 4.0f;
    style.FrameRounding     = 3.0f;
    style.PopupRounding     = 4.0f;
    style.ScrollbarRounding = 3.0f;
    style.GrabRounding      = 3.0f;
    style.TabRounding       = 4.0f;
    style.WindowBorderSize  = 1.0f;
    style.FrameBorderSize   = 1.0f;
}

void draw_tactical_hud_overlay(
    ImDrawList* draw_list,
    ImVec2 top_left,
    ImVec2 view_size,
    const SimulationPipeline& pipeline
) {
    const float cx = top_left.x + view_size.x * 0.5f;
    const float cy = top_left.y + view_size.y * 0.5f;

    const auto& telem = pipeline.telemetry();
    const auto& drone = pipeline.drone();
    const auto& gimbal = pipeline.gimbal();

    // 1. Central Optical Reticle & Crosshairs
    const ImU32 reticle_color = IM_COL32(0, 255, 180, 180);
    draw_list->AddLine(ImVec2(cx - 24.0f, cy), ImVec2(cx - 6.0f, cy), reticle_color, 1.5f);
    draw_list->AddLine(ImVec2(cx + 6.0f, cy), ImVec2(cx + 24.0f, cy), reticle_color, 1.5f);
    draw_list->AddLine(ImVec2(cx, cy - 24.0f), ImVec2(cx, cy - 6.0f), reticle_color, 1.5f);
    draw_list->AddLine(ImVec2(cx, cy + 6.0f), ImVec2(cx, cy + 24.0f), reticle_color, 1.5f);
    draw_list->AddCircle(ImVec2(cx, cy), 18.0f, reticle_color, 24, 1.0f);

    // 2. Flight Attitude Pitch Ladder
    const float pitch_deg = drone.pose().attitude.pitch_deg();

    for (int p = -30; p <= 30; p += 10) {
        if (p == 0) continue;
        const float py = cy - (static_cast<float>(p) - pitch_deg) * 3.5f;
        if (py >= top_left.y + 20.0f && py <= top_left.y + view_size.y - 20.0f) {
            const float bar_w = (p % 20 == 0) ? 35.0f : 20.0f;
            draw_list->AddLine(ImVec2(cx - bar_w, py), ImVec2(cx - 10.0f, py), IM_COL32(0, 255, 180, 120), 1.0f);
            draw_list->AddLine(ImVec2(cx + 10.0f, py), ImVec2(cx + bar_w, py), IM_COL32(0, 255, 180, 120), 1.0f);
        }
    }

    // 3. Top Heading Compass Tape
    const float yaw_deg = drone.pose().attitude.yaw_deg();
    const float tape_y = top_left.y + 16.0f;
    draw_list->AddLine(ImVec2(cx - 100.0f, tape_y), ImVec2(cx + 100.0f, tape_y), IM_COL32(0, 255, 180, 140), 1.0f);
    draw_list->AddTriangleFilled(
        ImVec2(cx, tape_y + 6.0f),
        ImVec2(cx - 4.0f, tape_y + 12.0f),
        ImVec2(cx + 4.0f, tape_y + 12.0f),
        IM_COL32(0, 255, 180, 220)
    );

    char hdg_str[32];
    snprintf(hdg_str, sizeof(hdg_str), "%03.0f°", std::fmod(yaw_deg + 360.0f, 360.0f));
    draw_list->AddText(ImVec2(cx - 14.0f, tape_y - 14.0f), IM_COL32(0, 255, 180, 240), hdg_str);

    // 4. Ground Truth BBoxes (Cyan, dashed)
    const float scale_x = view_size.x / 640.0f;
    const float scale_y = view_size.y / 512.0f;

    for (const auto& gt : pipeline.ground_truth_detections()) {
        const float gx1 = top_left.x + gt.bbox.x * scale_x;
        const float gy1 = top_left.y + gt.bbox.y * scale_y;
        const float gx2 = gx1 + gt.bbox.width * scale_x;
        const float gy2 = gy1 + gt.bbox.height * scale_y;
        draw_list->AddRect(ImVec2(gx1, gy1), ImVec2(gx2, gy2), IM_COL32(0, 220, 255, 150), 1.0f);
    }

    // 5. CFAR Detections (Yellow corner cross marks)
    for (const auto& det : pipeline.current_detections()) {
        const float dx1 = top_left.x + det.bbox.x * scale_x;
        const float dy1 = top_left.y + det.bbox.y * scale_y;
        const float dx2 = dx1 + det.bbox.width * scale_x;
        const float dy2 = dy1 + det.bbox.height * scale_y;
        draw_list->AddRect(ImVec2(dx1 - 1.0f, dy1 - 1.0f), ImVec2(dx2 + 1.0f, dy2 + 1.0f), IM_COL32(255, 230, 0, 180), 1.0f);
    }

    // 6. Confirmed Kalman Multi-Target Tracks (Green solid box, ID, velocity vector, trajectory trail)
    for (const auto& track : pipeline.active_tracks()) {
        const float tx1 = top_left.x + track.bbox.x * scale_x;
        const float ty1 = top_left.y + track.bbox.y * scale_y;
        const float tx2 = tx1 + track.bbox.width * scale_x;
        const float ty2 = ty1 + track.bbox.height * scale_y;

        const ImU32 trk_color = (track.state == detection::TrackState::Confirmed)
            ? IM_COL32(40, 255, 80, 255)
            : IM_COL32(255, 140, 0, 200);

        // Bounding box
        draw_list->AddRect(ImVec2(tx1, ty1), ImVec2(tx2, ty2), trk_color, 2.0f, 0, 2.0f);

        // Trajectory History Trail
        if (track.history.size() >= 2) {
            for (size_t h = 0; h + 1 < track.history.size(); ++h) {
                const float hx1 = top_left.x + static_cast<float>(track.history[h].x) * scale_x;
                const float hy1 = top_left.y + static_cast<float>(track.history[h].y) * scale_y;
                const float hx2 = top_left.x + static_cast<float>(track.history[h + 1].x) * scale_x;
                const float hy2 = top_left.y + static_cast<float>(track.history[h + 1].y) * scale_y;
                draw_list->AddLine(ImVec2(hx1, hy1), ImVec2(hx2, hy2), IM_COL32(40, 255, 80, 120), 1.0f);
            }
        }

        // Velocity Vector Arrow
        const float t_cx = (tx1 + tx2) * 0.5f;
        const float t_cy = (ty1 + ty2) * 0.5f;
        const float vx = track.velocity_pxps.x * scale_x * 0.5f;
        const float vy = track.velocity_pxps.y * scale_y * 0.5f;
        draw_list->AddLine(ImVec2(t_cx, t_cy), ImVec2(t_cx + vx, t_cy + vy), trk_color, 2.0f);

        // Label: ID + SCR
        char tag[48];
        snprintf(tag, sizeof(tag), "TRK-%02u [SCR:%.1f]", track.track_id, track.scr);
        draw_list->AddText(ImVec2(tx1, ty1 - 15.0f), trk_color, tag);
    }

    // 7. Corner Tactical Telemetry OSD
    char osd_top[128];
    snprintf(osd_top, sizeof(osd_top), "DRONE POS: [%+.1f, %+.1f, %.1f]m | ALT: %.1f m AGL",
             drone.pose().position_enu_m.x, drone.pose().position_enu_m.y, drone.pose().position_enu_m.z, telem.altitude_agl_m);
    draw_list->AddText(ImVec2(top_left.x + 10.0f, top_left.y + 10.0f), IM_COL32(0, 255, 200, 240), osd_top);

    char osd_left[128];
    snprintf(osd_left, sizeof(osd_left), "ALT: %.1f m AGL | SPD: %.1f m/s | PITCH: %+.1f°",
             telem.altitude_agl_m, telem.ground_speed_mps, gimbal.current_pitch_deg());
    draw_list->AddText(ImVec2(top_left.x + 12.0f, top_left.y + view_size.y - 24.0f), IM_COL32(0, 255, 180, 240), osd_left);

    char osd_right[128];
    snprintf(osd_right, sizeof(osd_right), "GPS: %.5f°, %.5f°",
             telem.drone_gps.latitude_deg, telem.drone_gps.longitude_deg);
    draw_list->AddText(ImVec2(top_left.x + view_size.x - 220.0f, top_left.y + view_size.y - 24.0f), IM_COL32(0, 255, 180, 240), osd_right);
}

class BeamTimelineState {
public:
    [[nodiscard]] bool paused() const noexcept { return paused_; }
    [[nodiscard]] std::size_t snapshot_size() const noexcept { return snapshot_.size(); }

    void toggle(const std::deque<BeamRecord>& live_history) {
        paused_ = !paused_;
        if (paused_) {
            snapshot_.assign(live_history.begin(), live_history.end());
        } else {
            snapshot_.clear();
        }
    }

    [[nodiscard]] const std::vector<BeamRecord>& displayed_history(const std::deque<BeamRecord>& live_history) const {
        if (paused_) {
            return snapshot_;
        }
        static thread_local std::vector<BeamRecord> cached;
        cached.assign(live_history.begin(), live_history.end());
        return cached;
    }

private:
    bool paused_{false};
    std::vector<BeamRecord> snapshot_{};
};

void render_beam_schedule_plot(
    SimulationPipeline& pipeline,
    BeamTimelineState& timeline_state,
    bool& popped_out,
    float plot_height
) {
    const auto& live_history = pipeline.beam_history();

    // 1. Toolbar controls
    const bool pause_clicked = ImGui::SmallButton(timeline_state.paused() ? "RESUME" : "PAUSE");
    if (pause_clicked) {
        timeline_state.toggle(live_history);
    }
    ImGui::SameLine();
    if (timeline_state.paused()) {
        ImGui::TextColored(
            ImVec4(1.0f, 0.75f, 0.30f, 1.0f),
            "FROZEN  %zu samples", timeline_state.snapshot_size());
    } else {
        ImGui::TextColored(
            ImVec4(1.0f, 0.75f, 0.30f, 1.0f),
            "%zu samples", live_history.size());
    }

    ImGui::SameLine();
    const bool is_sector = (pipeline.gimbal().mode() == platform::GimbalMode::SectorScan);
    if (ImGui::SmallButton(is_sector ? "Mode: 360° Scan (0.25 Hz)" : "Mode: GeoLock / Tracking")) {
        if (!is_sector) {
            pipeline.set_gimbal_sector_scan(0.0f, 360.0f, 4.0f, {25.0f, 3.0f, 14.0f});
        } else {
            pipeline.set_gimbal_geolock(pipeline.primary_target_position());
        }
    }

    ImGui::SameLine();
    const float avail_x = ImGui::GetContentRegionAvail().x;
    if (avail_x > 32.0f) {
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + avail_x - 30.0f);
        if (ImGui::SmallButton(popped_out ? "[ - ]" : "[ ]")) {
            popped_out = !popped_out;
        }
    }

    // 2. Dual-axis ImPlot Graph
    const auto& displayed = timeline_state.displayed_history(live_history);
    if (displayed.size() >= 2 &&
        ImPlot::BeginPlot("##beamtl", ImVec2(-1, plot_height), ImPlotFlags_NoMenus)) {
        
        static thread_local std::vector<double> xs, ys, ys2;
        const size_t n = displayed.size();
        xs.resize(n);
        ys.resize(n);
        ys2.resize(n);

        const double t0 = displayed.front().sim_time_sec;
        for (size_t i = 0; i < n; ++i) {
            xs[i] = displayed[i].sim_time_sec - t0;
            ys[i] = displayed[i].az_deg;
            ys2[i] = displayed[i].el_deg;
        }

        const double y1_max = 360.0;
        const double y2_max = is_sector ? 30.0 : 90.0;
        const char* y1_label = "az [0..360°]";
        const char* y2_label = is_sector ? "el [deg]" : "el dep [0..90°]";

        ImPlot::SetupAxes("t [s]", y1_label, 0, 0);
        ImPlot::SetupAxis(ImAxis_Y2, y2_label, ImPlotAxisFlags_AuxDefault);

        const double x_max = std::max(5.0, xs.back() + 0.1);
        ImPlot::SetupAxesLimits(0.0, x_max, 0.0, y1_max, ImPlotCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y2, 0.0, y2_max, ImPlotCond_Always);

        // Azimuth trace (green sawtooth)
        ImPlot::SetAxes(ImAxis_X1, ImAxis_Y1);
        ImPlot::SetNextLineStyle(ImVec4(0.35f, 1.0f, 0.55f, 1.0f), 1.5f);
        ImPlot::PlotLine("beam az", xs.data(), ys.data(), static_cast<int>(xs.size()));

        // Elevation trace (amber stepped)
        ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
        ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.75f, 0.30f, 1.0f), 1.5f);
        ImPlot::PlotLine("beam el", xs.data(), ys2.data(), static_cast<int>(xs.size()));

        ImPlot::EndPlot();
    }
}

} // namespace

int main(int argc, char* argv[]) {
    std::string scenario_path = "scenarios/desert_convoy_day.json";
    for (int i = 1; i < argc; ++i) {
        std::string_view arg = argv[i];
        if (arg == "--scenario" && i + 1 < argc) {
            scenario_path = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options] [scenario.json]\n"
                      << "  --scenario <path>   Path to scenario manifest JSON\n"
                      << "  --help              Display this message\n";
            return 0;
        } else if (!arg.starts_with("-")) {
            scenario_path = arg;
        }
    }

    if (!glfwInit()) {
        std::cerr << "Error: Failed to initialize GLFW\n";
        return 1;
    }

    // Setup Metal on macOS
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window = glfwCreateWindow(1440, 880, "Anduril Tactical IR Sensor & Drone Detection Simulation", NULL, NULL);
    if (!window) {
        std::cerr << "Error: Failed to create GLFW window\n";
        glfwTerminate();
        return 1;
    }

    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    id<MTLCommandQueue> commandQueue = [device newCommandQueue];

    NSWindow* nswin = glfwGetCocoaWindow(window);
    CAMetalLayer* layer = [CAMetalLayer layer];
    layer.device = device;
    layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
    nswin.contentView.layer = layer;
    nswin.contentView.wantsLayer = YES;

    // Setup Dear ImGui and ImPlot
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    apply_tactical_defense_theme();

    ImGui_ImplGlfw_InitForOther(window, true);
    ImGui_ImplMetal_Init(device);

    // Load Scenario
    auto manifest_opt = scene::ScenarioManifest::load_from_file(scenario_path);
    if (!manifest_opt) {
        manifest_opt = scene::ScenarioManifest{};
    }

    constexpr uint32_t CAM_W = 640;
    constexpr uint32_t CAM_H = 512;
    SimulationPipeline pipeline(*manifest_opt, CAM_W, CAM_H);

    // Create 4 Metal Viewport Textures
    auto tex_gt = create_metal_texture((__bridge void*)device, CAM_W, CAM_H);
    auto tex_raw = create_metal_texture((__bridge void*)device, CAM_W, CAM_H);
    auto tex_isp = create_metal_texture((__bridge void*)device, CAM_W, CAM_H);
    auto tex_hud = create_metal_texture((__bridge void*)device, CAM_W, CAM_H);

    MTLRenderPassDescriptor* renderPassDescriptor = [MTLRenderPassDescriptor renderPassDescriptor];

    // UI State
    float netd_slider = 40.0f;
    float orbit_radius = 300.0f;
    float airspeed_cmd = 18.0f;
    float altitude_cmd = 500.0f;
    float cfar_threshold_factor = 3.5f;
    bool enable_jitter = true;
    bool enable_bpr = true;
    bool enable_lcm = true;
    BeamTimelineState timeline_state;
    bool beam_schedule_popped_out = false;
    float sim_speed = 1.0f;
    auto last_wall_time = std::chrono::steady_clock::now();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // 1. Real-time clock synchronization: measure actual wall-clock delta
        const auto now = std::chrono::steady_clock::now();
        float wall_dt = std::chrono::duration<float>(now - last_wall_time).count();
        last_wall_time = now;
        wall_dt = std::clamp(wall_dt, 0.001f, 0.100f);

        const float dt = wall_dt * sim_speed;
        pipeline.enable_bpr = enable_bpr;
        pipeline.enable_lcm_filter = enable_lcm;
        pipeline.step(dt);

        // 2. Upload frames to Metal Textures
        tex_gt->update_from_radiance(pipeline.ground_truth_radiance().as_span());
        tex_raw->update_from_14bit(pipeline.raw_fpa_frame().as_span(), 3000, 7000);
        tex_isp->update_from_grayscale(pipeline.display_frame().as_span());
        tex_hud->update_from_grayscale(pipeline.display_frame().as_span());

        // 3. Begin ImGui Frame
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        layer.drawableSize = CGSizeMake(width, height);
        id<CAMetalDrawable> drawable = [layer nextDrawable];

        id<MTLCommandBuffer> commandBuffer = [commandQueue commandBuffer];
        renderPassDescriptor.colorAttachments[0].texture = drawable.texture;
        renderPassDescriptor.colorAttachments[0].loadAction = MTLLoadActionClear;
        renderPassDescriptor.colorAttachments[0].clearColor = MTLClearColorMake(0.06, 0.07, 0.08, 1.0);
        renderPassDescriptor.colorAttachments[0].storeAction = MTLStoreActionStore;

        ImGui_ImplMetal_NewFrame(renderPassDescriptor);
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Logical display coordinates in points (NOT raw framebuffer pixels)
        const float win_w = io.DisplaySize.x;
        const float win_h = io.DisplaySize.y;

        // Set Fullscreen Docking / Workspace window
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(win_w, win_h));
        ImGui::Begin("GCS Main Canvas", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus);

        const auto& telem = pipeline.telemetry();
        const auto& d_pos = telem.drone_pose.position_enu_m;

        // Top Control Ribbon with live Drone Position updates
        ImGui::TextColored(ImVec4(0.2f, 0.85f, 0.5f, 1.0f), "ANDURIL LATTICE / EO-IR DRONE SIMULATOR");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "| ENU: [%+.1f, %+.1f, %.1f]m", d_pos.x, d_pos.y, d_pos.z);
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1.0f), "| GPS: %.5f°, %.5f° (%.1fm AGL)",
                           telem.drone_gps.latitude_deg, telem.drone_gps.longitude_deg, telem.altitude_agl_m);
        ImGui::SameLine();
        ImGui::Text("| Spd: %.1fm/s", telem.ground_speed_mps);
        ImGui::SameLine();
        ImGui::Text("| Sim: %.1fs", telem.sim_time_sec);
        ImGui::SameLine();
        ImGui::Text("| FPS: %.0f", io.Framerate);
        ImGui::SameLine();
        ImGui::Text("| Tracks: %zu", pipeline.tracker().confirmed_track_count());
        ImGui::Separator();

        // Main Layout: 2 Columns
        // Left Column: 4 Viewports (62% width), Right Column: Controls (38% width)
        const float left_col_w = std::floor(win_w * 0.62f);
        const float right_col_w = win_w - left_col_w - 24.0f;
        const float avail_h = win_h - 48.0f;

        // Symmetric 2x2 Viewport Layout Computation:
        // 2 rows of viewports and 2 columns.
        // Each row: text header (~22px) + image (vp_h) + spacing (~10px).
        // Max height per viewport ensuring BOTH rows [1,2] and [3,4] fit completely:
        const float max_vp_h = std::max(100.0f, (avail_h - 72.0f) * 0.5f);
        const float max_vp_w = std::max(140.0f, (left_col_w - 28.0f) * 0.5f);

        // Maintain 640x512 (5:4 / 1.25) aspect ratio
        float vp_w = max_vp_w;
        float vp_h = vp_w * (512.0f / 640.0f);
        if (vp_h > max_vp_h) {
            vp_h = max_vp_h;
            vp_w = vp_h * (640.0f / 512.0f);
        }

        ImGui::BeginChild("LeftViewportGrid", ImVec2(left_col_w, avail_h), true);
        {
            // Row 1: Viewport 1 (Ground Truth Radiance) & Viewport 2 (Raw 14-bit FPA)
            ImGui::BeginGroup();
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "[1] Ground Truth At-Aperture Radiance");
            ImGui::Image(tex_gt->imgui_texture_id(), ImVec2(vp_w, vp_h));
            ImGui::EndGroup();

            ImGui::SameLine();

            ImGui::BeginGroup();
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "[2] Raw 14-bit FPA (FPN Striping + NETD Noise)");
            ImGui::Image(tex_raw->imgui_texture_id(), ImVec2(vp_w, vp_h));
            ImGui::EndGroup();

            ImGui::Spacing();

            // Row 2: Viewport 3 (Calibrated NUC 8-bit Video) & Viewport 4 (Tactical HUD)
            ImGui::BeginGroup();
            ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.5f, 1.0f), "[3] Embedded ISP Feed (2-Point NUC + CLAHE)");
            ImGui::Image(tex_isp->imgui_texture_id(), ImVec2(vp_w, vp_h));
            ImGui::EndGroup();

            ImGui::SameLine();

            ImGui::BeginGroup();
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.7f, 1.0f), "[4] Tactical Tracking HUD (SORT Multi-Target)");
            const ImVec2 hud_pos = ImGui::GetCursorScreenPos();
            ImGui::Image(tex_hud->imgui_texture_id(), ImVec2(vp_w, vp_h));

            // Overlay HUD graphics directly onto Viewport 4
            draw_tactical_hud_overlay(ImGui::GetWindowDrawList(), hud_pos, ImVec2(vp_w, vp_h), pipeline);
            ImGui::EndGroup();
        }
        ImGui::EndChild();

        ImGui::SameLine();

        // Right Column: Interactive Flight, Gimbal, Sensor & Telemetry Panels
        ImGui::BeginChild("RightControlPanels", ImVec2(right_col_w, avail_h), true);
        {
            if (ImGui::CollapsingHeader("1. Live Drone Navigation & State", ImGuiTreeNodeFlags_DefaultOpen)) {
                const auto& d_pose = telem.drone_pose;

                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Local ENU Position:");
                ImGui::Text("  East  (X): %+.2f m", d_pose.position_enu_m.x);
                ImGui::Text("  North (Y): %+.2f m", d_pose.position_enu_m.y);
                ImGui::Text("  Up    (Z): %+.2f m", d_pose.position_enu_m.z);

                ImGui::Separator();
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "WGS84 GPS Position:");
                ImGui::Text("  Latitude:  %+.6f°", telem.drone_gps.latitude_deg);
                ImGui::Text("  Longitude: %+.6f°", telem.drone_gps.longitude_deg);
                ImGui::Text("  Altitude:  %.1f m MSL (%.1f m AGL)", telem.drone_gps.altitude_msl_m, telem.altitude_agl_m);

                ImGui::Separator();
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Kinematics & Attitude:");
                ImGui::Text("  Ground Speed: %.1f m/s (%.1f km/h)", telem.ground_speed_mps, telem.ground_speed_mps * 3.6f);
                ImGui::Text("  Velocity: [%+.1f, %+.1f, %+.1f] m/s", d_pose.velocity_mps.x, d_pose.velocity_mps.y, d_pose.velocity_mps.z);
                ImGui::Text("  Roll:  %+.1f°", d_pose.attitude.roll_deg());
                ImGui::Text("  Pitch: %+.1f°", d_pose.attitude.pitch_deg());
                ImGui::Text("  Yaw:   %+.1f°", d_pose.attitude.yaw_deg());
            }

            if (ImGui::CollapsingHeader("2. Flight Mode & Teleoperation", ImGuiTreeNodeFlags_DefaultOpen)) {
                // Mode Buttons
                if (ImGui::Button("Waypoint Nav", ImVec2(100, 26))) pipeline.set_flight_mode(FlightMode::Waypoint);
                ImGui::SameLine();
                if (ImGui::Button("Standoff Orbit", ImVec2(100, 26))) pipeline.set_flight_mode(FlightMode::Orbit);
                ImGui::SameLine();
                if (ImGui::Button("Manual Stick", ImVec2(100, 26))) pipeline.set_flight_mode(FlightMode::Manual);
                ImGui::SameLine();
                if (ImGui::Button("Pursuit Lock", ImVec2(100, 26))) pipeline.set_flight_mode(FlightMode::Pursuit);

                const char* mode_name = "Orbit";
                if (pipeline.flight_mode() == FlightMode::Waypoint) mode_name = "Waypoint Route";
                else if (pipeline.flight_mode() == FlightMode::Manual) mode_name = "Manual Teleoperation";
                else if (pipeline.flight_mode() == FlightMode::Pursuit) mode_name = "Autonomous Target Pursuit";
                ImGui::Text("Active Flight Mode: %s", mode_name);

                if (ImGui::SliderFloat("Airspeed [m/s]", &airspeed_cmd, 5.0f, 35.0f, "%.1f m/s")) {
                    pipeline.set_orbit_params({250.0f, 200.0f, altitude_cmd}, orbit_radius, airspeed_cmd);
                }
                if (ImGui::SliderFloat("Orbit Radius [m]", &orbit_radius, 100.0f, 800.0f, "%.0f m")) {
                    pipeline.set_orbit_params({250.0f, 200.0f, altitude_cmd}, orbit_radius, airspeed_cmd);
                }

                ImGui::Separator();
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Simulation Pacing & Speed:");
                ImGui::SliderFloat("Time Scale", &sim_speed, 0.25f, 3.0f, "%.2fx");
                ImGui::SameLine();
                if (ImGui::SmallButton("1.0x Real-Time")) {
                    sim_speed = 1.0f;
                }

                if (pipeline.flight_mode() == FlightMode::Manual) {
                    static float roll_s = 0.0f, pitch_s = 0.0f, yaw_s = 0.0f, throt_s = 0.0f;
                    ImGui::Text("Virtual Joystick (Stick Input):");
                    ImGui::SliderFloat("Roll", &roll_s, -1.0f, 1.0f);
                    ImGui::SliderFloat("Pitch", &pitch_s, -1.0f, 1.0f);
                    ImGui::SliderFloat("Yaw Rate", &yaw_s, -1.0f, 1.0f);
                    ImGui::SliderFloat("Throttle", &throt_s, -1.0f, 1.0f);
                    pipeline.set_manual_sticks(roll_s, pitch_s, yaw_s, throt_s);
                }
            }

            if (ImGui::CollapsingHeader("3. Gimbal & Payload Controls", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::Button("360° Scan (0.25 Hz)", ImVec2(140, 24))) {
                    pipeline.set_gimbal_sector_scan(0.0f, 360.0f, 4.0f, {25.0f, 3.0f, 14.0f});
                }
                ImGui::SameLine();
                if (ImGui::Button("Point Nadir", ImVec2(90, 24))) pipeline.set_gimbal_nadir();
                ImGui::SameLine();
                if (ImGui::Button("GeoLock Target", ImVec2(110, 24))) pipeline.set_gimbal_geolock(pipeline.primary_target_position());

                if (ImGui::Checkbox("Motor Vibration Jitter", &enable_jitter)) {
                    pipeline.gimbal().set_jitter_enabled(enable_jitter);
                }
                static float jitter_amp = 0.20f;
                if (ImGui::SliderFloat("Jitter Amplitude [mrad]", &jitter_amp, 0.0f, 1.0f, "%.2f mrad")) {
                    pipeline.gimbal().set_jitter_amplitude_mrad(jitter_amp);
                }

                ImGui::Separator();
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "BEAM SCHEDULE (Gimbal Slew & Scan Monitor):");
                if (!beam_schedule_popped_out) {
                    render_beam_schedule_plot(pipeline, timeline_state, beam_schedule_popped_out, 200.0f);
                } else {
                    ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.30f, 1.0f), "Popout Window Active: [BEAM SCHEDULE]");
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Re-dock")) {
                        beam_schedule_popped_out = false;
                    }
                }
            }

            if (ImGui::CollapsingHeader("4. Sensor Degradation & Embedded ISP", ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::SliderFloat("NETD Temporal Noise [mK]", &netd_slider, 10.0f, 100.0f, "%.0f mK")) {
                    pipeline.fpa().set_netd_k(netd_slider * 1.0e-3f);
                }

                if (ImGui::SliderFloat("CFAR Threshold Multiplier", &cfar_threshold_factor, 1.5f, 7.0f, "%.1f sigma")) {
                    pipeline.cfar().params().threshold_factor = cfar_threshold_factor;
                }

                ImGui::Checkbox("Bad Pixel Replacement (BPR)", &enable_bpr);
                ImGui::Checkbox("Multiscale LCM Point-Target Filter", &enable_lcm);

                if (ImGui::Button("Trigger 2-Point NUC Calibration", ImVec2(240, 28))) {
                    pipeline.calibrate_nuc(288.15f, 308.15f);
                }
            }

            if (ImGui::CollapsingHeader("5. Tactical Telemetry & Active Tracks", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::Text("GPS Lat: %.6f° | Lon: %.6f°", telem.drone_gps.latitude_deg, telem.drone_gps.longitude_deg);
                ImGui::Text("Altitude: %.1f m AGL | Speed: %.1f m/s", telem.altitude_agl_m, telem.ground_speed_mps);
                ImGui::Separator();

                // Track Table
                if (ImGui::BeginTable("TracksTable", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                    ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, 40.0f);
                    ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 65.0f);
                    ImGui::TableSetupColumn("SCR", ImGuiTableColumnFlags_WidthFixed, 45.0f);
                    ImGui::TableSetupColumn("Vel [px/s]", ImGuiTableColumnFlags_WidthFixed, 75.0f);
                    ImGui::TableSetupColumn("GPS Est", ImGuiTableColumnFlags_WidthStretch);
                    ImGui::TableHeadersRow();

                    for (const auto& track : pipeline.active_tracks()) {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::Text("%u", track.track_id);

                        ImGui::TableSetColumnIndex(1);
                        if (track.state == detection::TrackState::Confirmed) {
                            ImGui::TextColored(ImVec4(0.2f, 0.9f, 0.3f, 1.0f), "CONF");
                        } else {
                            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.1f, 1.0f), "LOST");
                        }

                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("%.1f", track.scr);

                        ImGui::TableSetColumnIndex(3);
                        const float spd = std::sqrt(track.velocity_pxps.x * track.velocity_pxps.x + track.velocity_pxps.y * track.velocity_pxps.y);
                        ImGui::Text("%.1f", spd);

                        ImGui::TableSetColumnIndex(4);
                        ImGui::Text("%.4f, %.4f", track.estimated_geo.latitude_deg, track.estimated_geo.longitude_deg);
                    }
                    ImGui::EndTable();
                }
            }

            if (ImGui::CollapsingHeader("6. Pipeline Latency Breakdown", ImGuiTreeNodeFlags_DefaultOpen)) {
                const auto& lat = pipeline.telemetry().latency;
                ImGui::Text("1. Scene Radiative Transfer: %.2f ms", lat.scene_ms);
                ImGui::Text("2. FPA Transduction/Noise:   %.2f ms", lat.fpa_ms);
                ImGui::Text("3. Embedded ISP (NUC/AGC):    %.2f ms", lat.isp_ms);
                ImGui::Text("4. CFAR / LCM Detection:     %.2f ms", lat.cfar_ms);
                ImGui::Text("5. Multi-Target Kalman:      %.2f ms", lat.tracker_ms);
                ImGui::Separator();
                ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Total Frame Time: %.2f ms (%.0f FPS max)",
                                   lat.total_ms, (lat.total_ms > 0.0f) ? (1000.0f / lat.total_ms) : 0.0f);
            }
        }
        ImGui::EndChild();

        ImGui::End();

        // 3b. Render Popout BEAM SCHEDULE Window if detached
        if (beam_schedule_popped_out) {
            ImGui::SetNextWindowSize(ImVec2(600, 420), ImGuiCond_FirstUseEver);
            if (ImGui::Begin("BEAM SCHEDULE", &beam_schedule_popped_out, ImGuiWindowFlags_NoCollapse)) {
                render_beam_schedule_plot(pipeline, timeline_state, beam_schedule_popped_out, -1);
            }
            ImGui::End();
        }

        // 4. Render ImGui Draw Data via Metal
        ImGui::Render();
        id<MTLRenderCommandEncoder> renderEncoder = [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
        [renderEncoder pushDebugGroup:@"Dear ImGui Rendering"];
        ImGui_ImplMetal_RenderDrawData(ImGui::GetDrawData(), commandBuffer, renderEncoder);
        [renderEncoder popDebugGroup];
        [renderEncoder endEncoding];

        [commandBuffer presentDrawable:drawable];
        [commandBuffer commit];
    }

    // Cleanup
    ImGui_ImplMetal_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
