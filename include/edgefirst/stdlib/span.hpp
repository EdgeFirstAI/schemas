#ifndef EDGEFIRST_STDLIB_SPAN_HPP
#define EDGEFIRST_STDLIB_SPAN_HPP

// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.
//
// Minimal C++17 backport of std::span for older standards. When C++20's
// std::span is available, this header aliases to the standard library type;
// otherwise it provides a two-pointer view over contiguous memory with the
// members required by schemas.hpp.

#include <cstddef>

#if __has_include(<version>)
#include <version>
#endif

#if defined(__cpp_lib_span) && __cpp_lib_span >= 202002L
#  include <span>
namespace edgefirst::stdlib {
    template <class T, std::size_t E = std::dynamic_extent>
    using span = std::span<T, E>;
    using std::dynamic_extent;
}
#else // C++17 fallback

namespace edgefirst::stdlib {

/// Sentinel value for "all remaining elements".
inline constexpr std::size_t dynamic_extent = static_cast<std::size_t>(-1);

/**
 * @brief Minimal non-owning view over a contiguous sequence of T.
 *
 * The class holds two data members: a pointer and a length.  It is
 * trivially copyable, so it can be passed and returned by value with no
 * overhead.  No heap allocation is ever performed.
 */
template <class T>
class span {
public:
    // ---- construction ---------------------------------------------------

    /// Default: empty span (data == nullptr, size == 0).
    constexpr span() noexcept = default;

    /// Pointer + length constructor.
    constexpr span(T* p, std::size_t n) noexcept : data_(p), size_(n) {}

    /// Begin / end iterator-pair constructor.
    constexpr span(T* first, T* last) noexcept
        : data_(first), size_(static_cast<std::size_t>(last - first)) {}

    // Defaulted copy / move.
    constexpr span(span const&) noexcept = default;
    constexpr span& operator=(span const&) noexcept = default;

    // ---- observers ------------------------------------------------------

    /// Pointer to the first element (may be nullptr when empty).
    [[nodiscard]] constexpr T* data() const noexcept { return data_; }

    /// Number of elements in the span.
    [[nodiscard]] constexpr std::size_t size() const noexcept { return size_; }

    /// Size in bytes (size() * sizeof(T)).
    [[nodiscard]] constexpr std::size_t size_bytes() const noexcept {
        return size_ * sizeof(T);
    }

    /// True when size() == 0.
    [[nodiscard]] constexpr bool empty() const noexcept { return size_ == 0; }

    // ---- element access -------------------------------------------------

    /// Unchecked element access.
    [[nodiscard]] constexpr T& operator[](std::size_t i) const noexcept {
        return data_[i];
    }

    // ---- iterators ------------------------------------------------------

    [[nodiscard]] constexpr T* begin() const noexcept { return data_; }
    [[nodiscard]] constexpr T* end()   const noexcept { return data_ + size_; }

    // ---- sub-views ------------------------------------------------------

    /**
     * @brief Returns a sub-span starting at @p offset of length @p count.
     *
     * If @p count is dynamic_extent (the default) the sub-span extends to
     * the end of this span.  No bounds checking is performed.
     */
    [[nodiscard]] constexpr span<T> subspan(
        std::size_t offset,
        std::size_t count = dynamic_extent) const noexcept
    {
        std::size_t n = (count == dynamic_extent) ? (size_ - offset) : count;
        return span<T>(data_ + offset, n);
    }

private:
    T*          data_ = nullptr;
    std::size_t size_ = 0;
};

} // namespace edgefirst::stdlib

#endif  // __cpp_lib_span

#endif  // EDGEFIRST_STDLIB_SPAN_HPP
