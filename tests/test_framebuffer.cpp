#include "test_main.hpp"
#include "ir_sim/core/frame_buffer.hpp"

using namespace ir_sim::core;

TEST_CASE(framebuffer_cache_alignment) {
    FrameBuffer<float> fb(640, 512);
    REQUIRE(!fb.empty());
    REQUIRE(fb.width() == 640);
    REQUIRE(fb.height() == 512);
    REQUIRE(fb.size_elements() == 640 * 512);

    // Verify 64-byte SIMD cache alignment
    const uintptr_t address = reinterpret_cast<uintptr_t>(fb.data());
    REQUIRE(address % SIMD_ALIGNMENT_BYTES == 0);
}

TEST_CASE(framebuffer_zero_copy_span) {
    FrameBuffer<uint16_t> fb(100, 50, 42);
    REQUIRE(fb(0, 0) == 42);
    REQUIRE(fb(99, 49) == 42);

    // Zero-copy std::span mutation
    std::span<uint16_t> span = fb.as_span();
    REQUIRE(span.size() == 5000);
    span[0] = 999;
    span[4999] = 1234;

    REQUIRE(fb(0, 0) == 999);
    REQUIRE(fb(99, 49) == 1234);
}

TEST_CASE(framebuffer_copy_and_move) {
    FrameBuffer<float> fb1(10, 10, 3.14f);
    
    // Copy
    FrameBuffer<float> fb2 = fb1;
    REQUIRE(fb2.width() == 10);
    REQUIRE_NEAR(fb2(5, 5), 3.14f, 1e-5);
    
    fb2(5, 5) = 2.71f;
    REQUIRE_NEAR(fb1(5, 5), 3.14f, 1e-5); // Independent copy

    // Move
    FrameBuffer<float> fb3 = std::move(fb1);
    REQUIRE(fb3.width() == 10);
    REQUIRE_NEAR(fb3(5, 5), 3.14f, 1e-5);
    REQUIRE(fb1.empty());
}

TEST_CASE(framebuffer_min_max) {
    FrameBuffer<uint8_t> fb(20, 20, 128);
    fb(3, 4) = 10;
    fb(15, 12) = 240;

    auto [min_v, max_v] = fb.min_max();
    REQUIRE(min_v == 10);
    REQUIRE(max_v == 240);
}
