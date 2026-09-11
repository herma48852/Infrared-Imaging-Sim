#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <span>
#include <stdexcept>
#include <type_traits>

namespace ir_sim::core {

// Alignment boundary for cache line and ARM NEON SIMD operations
inline constexpr size_t SIMD_ALIGNMENT_BYTES = 64;

/**
 * @brief Cache-aligned, contiguous 2D FrameBuffer with zero-copy std::span views.
 * 
 * Optimized for real-time video pipelines: avoids dynamic reallocations when
 * reusing pre-allocated frame memory, guarantees 64-byte alignment for SIMD vectorization.
 */
template <typename T>
class FrameBuffer {
    static_assert(std::is_trivially_copyable_v<T>, "FrameBuffer element must be trivially copyable for SIMD/ISP");

public:
    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;
    using reference = T&;
    using const_reference = const T&;

    FrameBuffer() = default;

    FrameBuffer(size_t width, size_t height) {
        allocate(width, height);
    }

    FrameBuffer(size_t width, size_t height, T initial_val) {
        allocate(width, height);
        fill(initial_val);
    }

    // Copy constructor (deep copy)
    FrameBuffer(const FrameBuffer& other) {
        if (other.data_) {
            allocate(other.width_, other.height_);
            std::memcpy(data_, other.data_, size_bytes());
        }
    }

    // Copy assignment
    FrameBuffer& operator=(const FrameBuffer& other) {
        if (this != &other) {
            if (width_ != other.width_ || height_ != other.height_) {
                deallocate();
                if (other.data_) {
                    allocate(other.width_, other.height_);
                }
            }
            if (other.data_ && data_) {
                std::memcpy(data_, other.data_, size_bytes());
            }
        }
        return *this;
    }

    // Move constructor
    FrameBuffer(FrameBuffer&& other) noexcept
        : data_(other.data_),
          width_(other.width_),
          height_(other.height_),
          capacity_elements_(other.capacity_elements_) {
        other.data_ = nullptr;
        other.width_ = 0;
        other.height_ = 0;
        other.capacity_elements_ = 0;
    }

    // Move assignment
    FrameBuffer& operator=(FrameBuffer&& other) noexcept {
        if (this != &other) {
            deallocate();
            data_ = other.data_;
            width_ = other.width_;
            height_ = other.height_;
            capacity_elements_ = other.capacity_elements_;

            other.data_ = nullptr;
            other.width_ = 0;
            other.height_ = 0;
            other.capacity_elements_ = 0;
        }
        return *this;
    }

    ~FrameBuffer() {
        deallocate();
    }

    // Re-allocate or re-use existing capacity
    void resize(size_t new_width, size_t new_height) {
        const size_t new_elements = new_width * new_height;
        if (new_elements > capacity_elements_) {
            deallocate();
            allocate(new_width, new_height);
        } else {
            width_ = new_width;
            height_ = new_height;
        }
    }

    void fill(T value) noexcept {
        if (data_) {
            std::fill_n(data_, size_elements(), value);
        }
    }

    void zero() noexcept {
        if (data_) {
            std::memset(data_, 0, size_bytes());
        }
    }

    // Direct pixel access (unchecked for performance in inner loops)
    [[nodiscard]] reference operator()(size_t x, size_t y) noexcept {
        assert(x < width_ && y < height_ && "FrameBuffer index out of bounds");
        return data_[y * width_ + x];
    }

    [[nodiscard]] const_reference operator()(size_t x, size_t y) const noexcept {
        assert(x < width_ && y < height_ && "FrameBuffer index out of bounds");
        return data_[y * width_ + x];
    }

    // Bounds-checked pixel access
    [[nodiscard]] reference at(size_t x, size_t y) {
        if (x >= width_ || y >= height_) {
            throw std::out_of_range("FrameBuffer::at() index out of range");
        }
        return data_[y * width_ + x];
    }

    [[nodiscard]] const_reference at(size_t x, size_t y) const {
        if (x >= width_ || y >= height_) {
            throw std::out_of_range("FrameBuffer::at() index out of range");
        }
        return data_[y * width_ + x];
    }

    // Zero-copy std::span views for safe Modern C++ interfaces
    [[nodiscard]] std::span<T> as_span() noexcept {
        return {data_, size_elements()};
    }

    [[nodiscard]] std::span<const T> as_span() const noexcept {
        return {data_, size_elements()};
    }

    // Row span view
    [[nodiscard]] std::span<T> row(size_t y) noexcept {
        assert(y < height_ && "Row index out of bounds");
        return {data_ + y * width_, width_};
    }

    [[nodiscard]] std::span<const T> row(size_t y) const noexcept {
        assert(y < height_ && "Row index out of bounds");
        return {data_ + y * width_, width_};
    }

    [[nodiscard]] pointer data() noexcept { return data_; }
    [[nodiscard]] const_pointer data() const noexcept { return data_; }

    [[nodiscard]] size_t width() const noexcept { return width_; }
    [[nodiscard]] size_t height() const noexcept { return height_; }
    [[nodiscard]] size_t size_elements() const noexcept { return width_ * height_; }
    [[nodiscard]] size_t size_bytes() const noexcept { return size_elements() * sizeof(T); }
    [[nodiscard]] bool empty() const noexcept { return data_ == nullptr || size_elements() == 0; }

    // Min and max intensity values (useful for AGC / scaling)
    [[nodiscard]] std::pair<T, T> min_max() const noexcept {
        if (empty()) {
            return {T{}, T{}};
        }
        auto [min_it, max_it] = std::minmax_element(data_, data_ + size_elements());
        return {*min_it, *max_it};
    }

private:
    void allocate(size_t width, size_t height) {
        width_ = width;
        height_ = height;
        capacity_elements_ = width * height;

        if (capacity_elements_ > 0) {
            void* raw_ptr = nullptr;
            const size_t bytes = capacity_elements_ * sizeof(T);
            // POSIX aligned allocation for ARM NEON SIMD efficiency
            const int res = posix_memalign(&raw_ptr, SIMD_ALIGNMENT_BYTES, bytes);
            if (res != 0 || raw_ptr == nullptr) {
                throw std::bad_alloc();
            }
            data_ = static_cast<pointer>(raw_ptr);
        } else {
            data_ = nullptr;
        }
    }

    void deallocate() noexcept {
        if (data_) {
            std::free(data_);
            data_ = nullptr;
        }
        width_ = 0;
        height_ = 0;
        capacity_elements_ = 0;
    }

    pointer data_{nullptr};
    size_t width_{0};
    size_t height_{0};
    size_t capacity_elements_{0};
};

// Aliases for common image formats in thermal simulation
using RadianceFrame = FrameBuffer<float>;       // At-aperture spectral radiance [W/(m^2*sr)]
using TemperatureFrame = FrameBuffer<float>;    // Apparent temperature [Kelvin]
using RawFrame14Bit = FrameBuffer<uint16_t>;    // 14-bit ADC counts [0 - 16383]
using DisplayFrame8Bit = FrameBuffer<uint8_t>;  // 8-bit enhanced display [0 - 255]

} // namespace ir_sim::core
