#import <Metal/Metal.h>
#include "ir_sim/app/metal_texture_renderer.hpp"

#include <algorithm>
#include <vector>

namespace ir_sim::app {

class MetalTextureImpl : public MetalTexture {
public:
    MetalTextureImpl(id<MTLDevice> device, uint32_t width, uint32_t height)
        : width_(width), height_(height) {
        MTLTextureDescriptor* desc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                                                       width:width
                                                                                      height:height
                                                                                   mipmapped:NO];
        desc.usage = MTLTextureUsageShaderRead;
        desc.storageMode = MTLStorageModeShared;

        texture_ = [device newTextureWithDescriptor:desc];
        rgba_scratch_.resize(static_cast<size_t>(width) * static_cast<size_t>(height), 0xFF000000);
    }

    ~MetalTextureImpl() override {
        texture_ = nil;
    }

    void update_from_grayscale(std::span<const uint8_t> gray_pixels) override {
        const size_t count = std::min(gray_pixels.size(), rgba_scratch_.size());

        for (size_t i = 0; i < count; ++i) {
            const uint32_t g = static_cast<uint32_t>(gray_pixels[i]);
            // RGBA8 format: (A << 24) | (B << 16) | (G << 8) | R
            rgba_scratch_[i] = 0xFF000000u | (g << 16) | (g << 8) | g;
        }

        upload_scratch_to_metal();
    }

    void update_from_14bit(
        std::span<const uint16_t> raw_pixels,
        uint16_t min_val,
        uint16_t max_val
    ) override {
        const size_t count = std::min(raw_pixels.size(), rgba_scratch_.size());
        const float range = (max_val > min_val) ? static_cast<float>(max_val - min_val) : 1.0f;
        const float scale = 255.0f / range;

        for (size_t i = 0; i < count; ++i) {
            const float val = static_cast<float>(raw_pixels[i] - min_val);
            const uint32_t g = static_cast<uint32_t>(std::clamp(val * scale, 0.0f, 255.0f));
            rgba_scratch_[i] = 0xFF000000u | (g << 16) | (g << 8) | g;
        }

        upload_scratch_to_metal();
    }

    void update_from_radiance(std::span<const float> radiance_pixels) override {
        const size_t count = std::min(radiance_pixels.size(), rgba_scratch_.size());
        if (count == 0) return;

        float min_rad = radiance_pixels[0];
        float max_rad = radiance_pixels[0];
        for (size_t i = 1; i < count; ++i) {
            if (radiance_pixels[i] < min_rad) min_rad = radiance_pixels[i];
            if (radiance_pixels[i] > max_rad) max_rad = radiance_pixels[i];
        }

        const float range = (max_rad > min_rad) ? (max_rad - min_rad) : 1.0f;
        const float scale = 255.0f / range;

        for (size_t i = 0; i < count; ++i) {
            const float val = (radiance_pixels[i] - min_rad) * scale;
            const uint32_t g = static_cast<uint32_t>(std::clamp(val, 0.0f, 255.0f));
            rgba_scratch_[i] = 0xFF000000u | (g << 16) | (g << 8) | g;
        }

        upload_scratch_to_metal();
    }

    [[nodiscard]] void* imgui_texture_id() const noexcept override {
        return (__bridge void*)texture_;
    }

    [[nodiscard]] uint32_t width() const noexcept override { return width_; }
    [[nodiscard]] uint32_t height() const noexcept override { return height_; }

private:
    uint32_t width_;
    uint32_t height_;
    id<MTLTexture> texture_;
    std::vector<uint32_t> rgba_scratch_;

    void upload_scratch_to_metal() {
        const MTLRegion region = MTLRegionMake2D(0, 0, width_, height_);
        const NSUInteger bytes_per_row = static_cast<NSUInteger>(width_) * sizeof(uint32_t);
        [texture_ replaceRegion:region
                    mipmapLevel:0
                      withBytes:rgba_scratch_.data()
                    bytesPerRow:bytes_per_row];
    }
};

std::unique_ptr<MetalTexture> create_metal_texture(
    void* mtl_device,
    uint32_t width,
    uint32_t height
) {
    id<MTLDevice> device = (__bridge id<MTLDevice>)mtl_device;
    return std::make_unique<MetalTextureImpl>(device, width, height);
}

} // namespace ir_sim::app
