cmake_minimum_required(VERSION 3.25)
project(InfraredImagingSim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Strict modern compiler flags for Apple Clang
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
    )
    # Fast floating-point and vectorization optimization in release builds
    add_compile_options("$<$<CONFIG:Release>:-O3;-ffast-math>")
endif()

# Core include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# -----------------------------------------------------------------------------
# 1. Core Data Structures Library (ir_sim_core)
# -----------------------------------------------------------------------------
add_library(ir_sim_core
    src/core/types.cpp
)

target_include_directories(ir_sim_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# -----------------------------------------------------------------------------
# 2. Physics & Radiometry Engine Library (ir_sim_physics)
# -----------------------------------------------------------------------------
add_library(ir_sim_physics
    src/physics/planck.cpp
    src/physics/atmosphere.cpp
    src/physics/materials.cpp
)

target_link_libraries(ir_sim_physics PUBLIC ir_sim_core)

# -----------------------------------------------------------------------------
# 3. Drone Platform, Gimbal & Optics Library (ir_sim_platform)
# -----------------------------------------------------------------------------
add_library(ir_sim_platform
    src/platform/drone_platform.cpp
    src/platform/flight_controller.cpp
    src/platform/gimbal.cpp
    src/platform/camera_model.cpp
)

target_link_libraries(ir_sim_platform PUBLIC ir_sim_core)

# -----------------------------------------------------------------------------
# 4. Sensor Transduction & Embedded ISP Library (ir_sim_sensor)
# -----------------------------------------------------------------------------
add_library(ir_sim_sensor
    src/sensor/fpa_sensor.cpp
    src/sensor/isp_pipeline.cpp
)

target_link_libraries(ir_sim_sensor PUBLIC
    ir_sim_physics
    ir_sim_core
)

# -----------------------------------------------------------------------------
# 5. Tactical Detection & Multi-Target Tracker Library (ir_sim_detection)
# -----------------------------------------------------------------------------
add_library(ir_sim_detection
    src/detection/point_target_lcm.cpp
    src/detection/cfar_2d.cpp
    src/detection/kalman_tracker.cpp
)

target_link_libraries(ir_sim_detection PUBLIC
    ir_sim_core
)

# -----------------------------------------------------------------------------
# 6. Scenario Manifest & Synthetic Scene Engine Library (ir_sim_scene)
# -----------------------------------------------------------------------------
add_library(ir_sim_scene
    src/scene/scenario_manifest.cpp
    src/scene/synthetic_scene.cpp
)

target_link_libraries(ir_sim_scene PUBLIC
    ir_sim_platform
    ir_sim_physics
    ir_sim_core
)

# -----------------------------------------------------------------------------
# 7. Standalone Scenario & Target Generator Tool (target_gen)
# -----------------------------------------------------------------------------
add_executable(target_gen
    src/tools/target_gen_main.cpp
)

target_link_libraries(target_gen PRIVATE
    ir_sim_scene
    ir_sim_core
)

# -----------------------------------------------------------------------------
# 8. Unified Simulation Pipeline Library (ir_sim_pipeline)
# -----------------------------------------------------------------------------
add_library(ir_sim_pipeline
    src/app/simulation_pipeline.cpp
)

target_link_libraries(ir_sim_pipeline PUBLIC
    ir_sim_scene
    ir_sim_detection
    ir_sim_sensor
    ir_sim_platform
    ir_sim_physics
    ir_sim_core
)

# -----------------------------------------------------------------------------
# 9. Headless Simulation Benchmark & Profiler Runner (ir_sim_cli)
# -----------------------------------------------------------------------------
add_executable(ir_sim_cli
    src/app/main_cli.cpp
)

target_link_libraries(ir_sim_cli PRIVATE
    ir_sim_pipeline
)

# -----------------------------------------------------------------------------
# 10. Dear ImGui + Metal Interactive GUI Simulator (ir_drone_sim)
# -----------------------------------------------------------------------------
find_package(glfw3 3.3 QUIET)
if (TARGET glfw)
    set(GLFW_TARGET glfw)
else()
    find_library(GLFW_LIB glfw PATHS /opt/homebrew/lib /usr/local/lib)
    set(GLFW_TARGET ${GLFW_LIB})
endif()

add_library(imgui STATIC
    third_party/imgui/imgui.cpp
    third_party/imgui/imgui_draw.cpp
    third_party/imgui/imgui_tables.cpp
    third_party/imgui/imgui_widgets.cpp
    third_party/imgui/backends/imgui_impl_glfw.cpp
    third_party/imgui/backends/imgui_impl_metal.mm
)

target_include_directories(imgui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/backends
    /opt/homebrew/include
    /usr/local/include
)

target_link_libraries(imgui PUBLIC
    ${GLFW_TARGET}
    "-framework Metal"
    "-framework QuartzCore"
    "-framework Cocoa"
    "-framework IOKit"
    "-framework CoreVideo"
)

# Suppress compiler warnings for third-party Dear ImGui code
target_compile_options(imgui PRIVATE -w)

add_executable(ir_drone_sim
    src/app/metal_texture_renderer.mm
    src/app/main_gui.mm
)

target_compile_options(ir_drone_sim PRIVATE -Wno-old-style-cast -Wno-nontrivial-memcall)

target_include_directories(ir_drone_sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui/backends
    /opt/homebrew/include
    /usr/local/include
)

target_link_libraries(ir_drone_sim PRIVATE
    imgui
    ir_sim_pipeline
    ${GLFW_TARGET}
    "-framework Metal"
    "-framework QuartzCore"
    "-framework Cocoa"
    "-framework IOKit"
    "-framework CoreVideo"
)

# -----------------------------------------------------------------------------
# 11. Unit Test Suite (ir_sim_tests)
# -----------------------------------------------------------------------------
enable_testing()

add_executable(ir_sim_tests
    tests/main_tests.cpp
    tests/test_framebuffer.cpp
    tests/test_planck_radiometry.cpp
    tests/test_atmosphere.cpp
    tests/test_materials.cpp
    tests/test_drone_kinematics.cpp
    tests/test_gimbal.cpp
    tests/test_camera_projection.cpp
    tests/test_fpa_sensor.cpp
    tests/test_isp_pipeline.cpp
    tests/test_detection_benchmark.cpp
    tests/test_scenario_manifest.cpp
)

target_include_directories(ir_sim_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/tests
)

target_link_libraries(ir_sim_tests PRIVATE
    ir_sim_pipeline
)

add_test(NAME PhaseUnitTests COMMAND ir_sim_tests)


