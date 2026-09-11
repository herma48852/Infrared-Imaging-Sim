#pragma once

#include <cstdint>
#include <memory>
#include <span>

namespace ir_sim::app {

/**
 * @brief Zero-Copy Metal Texture Wrapper for Dear ImGui Viewports.
 * 
 * Uploads 8-bit grayscale or 32-bit RGBA frames directly to Apple Silicon unified memory
 * using MTLTexture replaceRegion for real-time 60 FPS display without heap thrashing.
 */
class MetalTexture {
public:
    virtual ~MetalTexture() = default;

    /**
     * @brief Uploads 8-bit grayscale frame into RGBA8 texture.
     */
    virtual void update_from_grayscale(std::span<const uint8_t> gray_pixels) = 0;

    /**
     * @brief Uploads 14-bit counts normalized to display.
     */
    virtual void update_from_14bit(std::span<const uint16_t> raw_pixels, uint16_t min_val = 0, uint16_t max_val = 16383) = 0;

    /**
     * @brief Uploads float radiance [W/(m^2 sr)] with colormap / auto-contrast.
     */
    virtual void update_from_radiance(std::span<const float> radiance_pixels) = 0;

    /**
     * @brief Returns the native MTLTexture handle cast to ImTextureID.
     */
    [[nodiscard]] virtual void* imgui_texture_id() const noexcept = 0;

    [[nodiscard]] virtual uint32_t width() const noexcept = 0;
    [[nodiscard]] virtual uint32_t height() const noexcept = 0;
};

[[nodiscard]] std::unique_ptr<MetalTexture> create_metal_texture(
    void* mtl_device,
    uint32_t width,
    uint32_t height
);

} // namespace ir_sim::app
