// DmaBuffer / DmaBufferView deprecated in 3.1.0; tests retained through
// the deprecation window.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/**
 * @file test_zero_copy.cpp
 * @brief Pointer-identity and allocation-counting invariant tests for the C++ wrapper.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.
 *
 * Part A — Pointer identity: verifies that View field accessors return string_view
 * and span objects whose data pointers lie within the CDR buffer that backs the
 * view handle, proving zero-copy access to variable-length fields.
 *
 * Part B — Allocation counting: overrides global operator new/delete to count
 * C++ heap allocations.  Field accessors on View types must not introduce
 * additional C++ allocations beyond what the construction of the view itself
 * requires.
 *
 * Note: The Rust FFI layer allocates the opaque handle (ros_image_t etc.) via
 * malloc, which is NOT counted by operator new.  The counter therefore measures
 * only C++ wrapper overhead, which should be zero for field accessors.
 */

#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

#include <atomic>
#include <cstdlib>
#include <new>
#include <vector>
#include <cstdint>

namespace ef = edgefirst::schemas;

// ============================================================================
// Part B — Global allocation counter
// ============================================================================

static std::atomic<std::size_t> g_new_count{0};
static std::atomic<std::size_t> g_delete_count{0};

// Custom operator new/delete family — every standard-library variant,
// including the C++17 aligned-allocation overloads and nothrow forms.
//
// Every form is overridden so that all C++ heap allocations (including
// Catch2's internal allocations and any aligned allocations the stdlib
// may do) route through a counted malloc. This keeps ASan's shadow
// bookkeeping self-consistent: every allocation tagged by ASan as
// "malloc" is freed by a matching "free" in our operator delete. The
// Rust FFI layer's ros_*_encode / ros_bytes_free calls go through libc
// malloc/free directly and do not touch these interceptors.
//
// A `malloc(0)` call is permitted by POSIX to return NULL. To avoid
// spurious `bad_alloc` in an allocation-counting test, we normalise
// zero-sized requests to 1 byte before calling __builtin_malloc.

static inline void* counted_malloc(std::size_t size) {
    g_new_count.fetch_add(1, std::memory_order_relaxed);
    if (size == 0) size = 1;  // malloc(0) may legally return NULL.
    return __builtin_malloc(size);
}

static inline void* counted_aligned_malloc(std::size_t size, std::size_t align) {
    g_new_count.fetch_add(1, std::memory_order_relaxed);
    if (size == 0) size = 1;
    // aligned_alloc requires size to be a multiple of alignment; round up.
    std::size_t asize = (size + align - 1) & ~(align - 1);
    return std::aligned_alloc(align, asize);
}

// ---- Plain operator new / delete ----

void* operator new(std::size_t size) {
    void* p = counted_malloc(size);
    if (!p) throw std::bad_alloc();
    return p;
}

void* operator new[](std::size_t size) {
    void* p = counted_malloc(size);
    if (!p) throw std::bad_alloc();
    return p;
}

void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
    return counted_malloc(size);
}

void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
    return counted_malloc(size);
}

void operator delete(void* p) noexcept {
    if (p) { g_delete_count.fetch_add(1, std::memory_order_relaxed); __builtin_free(p); }
}

void operator delete(void* p, std::size_t) noexcept {
    if (p) { g_delete_count.fetch_add(1, std::memory_order_relaxed); __builtin_free(p); }
}

void operator delete[](void* p) noexcept {
    if (p) { g_delete_count.fetch_add(1, std::memory_order_relaxed); __builtin_free(p); }
}

void operator delete[](void* p, std::size_t) noexcept {
    if (p) { g_delete_count.fetch_add(1, std::memory_order_relaxed); __builtin_free(p); }
}

void operator delete(void* p, const std::nothrow_t&) noexcept {
    if (p) { g_delete_count.fetch_add(1, std::memory_order_relaxed); __builtin_free(p); }
}

void operator delete[](void* p, const std::nothrow_t&) noexcept {
    if (p) { g_delete_count.fetch_add(1, std::memory_order_relaxed); __builtin_free(p); }
}

// ---- Aligned operator new / delete (C++17+) ----

void* operator new(std::size_t size, std::align_val_t align) {
    void* p = counted_aligned_malloc(size, static_cast<std::size_t>(align));
    if (!p) throw std::bad_alloc();
    return p;
}

void* operator new[](std::size_t size, std::align_val_t align) {
    void* p = counted_aligned_malloc(size, static_cast<std::size_t>(align));
    if (!p) throw std::bad_alloc();
    return p;
}

void* operator new(std::size_t size, std::align_val_t align, const std::nothrow_t&) noexcept {
    return counted_aligned_malloc(size, static_cast<std::size_t>(align));
}

void* operator new[](std::size_t size, std::align_val_t align, const std::nothrow_t&) noexcept {
    return counted_aligned_malloc(size, static_cast<std::size_t>(align));
}

void operator delete(void* p, std::align_val_t) noexcept {
    if (p) { g_delete_count.fetch_add(1, std::memory_order_relaxed); __builtin_free(p); }
}

void operator delete[](void* p, std::align_val_t) noexcept {
    if (p) { g_delete_count.fetch_add(1, std::memory_order_relaxed); __builtin_free(p); }
}

void operator delete(void* p, std::size_t, std::align_val_t) noexcept {
    if (p) { g_delete_count.fetch_add(1, std::memory_order_relaxed); __builtin_free(p); }
}

void operator delete[](void* p, std::size_t, std::align_val_t) noexcept {
    if (p) { g_delete_count.fetch_add(1, std::memory_order_relaxed); __builtin_free(p); }
}

void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept {
    if (p) { g_delete_count.fetch_add(1, std::memory_order_relaxed); __builtin_free(p); }
}

void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept {
    if (p) { g_delete_count.fetch_add(1, std::memory_order_relaxed); __builtin_free(p); }
}

// ============================================================================
// Golden CDR fixtures (shared with pointer-identity tests below)
// ============================================================================

// kGoldenDetectBytes: Detect with 3 boxes, mirroring the detect_multi_cdr
// fixture used in tests/c/test_edgefirst_msgs.c (Detect_multi.cdr).
//   boxes[0]  label="a",      score≈0.95, track_id="t"
//   boxes[1]  label="person", score≈0.87, track_id="track_long_id"
//   boxes[2]  label="ab",     score=0.50, track_id="abc"
static constexpr std::uint8_t kGoldenDetectBytes[] = {
    0x00,0x01,0x00,0x00,0xd2,0x02,0x96,0x49,0x15,0xcd,0x5b,0x07,0x0b,0x00,0x00,0x00,
    0x74,0x65,0x73,0x74,0x5f,0x66,0x72,0x61,0x6d,0x65,0x00,0x00,0xd2,0x02,0x96,0x49,
    0x15,0xcd,0x5b,0x07,0x00,0x00,0x00,0x00,0x40,0x42,0x0f,0x00,0x00,0x00,0x00,0x00,
    0x80,0x84,0x1e,0x00,0x03,0x00,0x00,0x00,0xcd,0xcc,0xcc,0x3d,0xcd,0xcc,0x4c,0x3e,
    0x00,0x00,0x00,0x3f,0x9a,0x99,0x19,0x3f,0x02,0x00,0x00,0x00,0x61,0x00,0x00,0x00,
    0x33,0x33,0x73,0x3f,0x00,0x00,0xa0,0x40,0x00,0x00,0x80,0x3f,0x02,0x00,0x00,0x00,
    0x74,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x9a,0x99,0x99,0x3e,0xcd,0xcc,0xcc,0x3e,0xcd,0xcc,0x4c,0x3e,0x9a,0x99,0x99,0x3e,
    0x07,0x00,0x00,0x00,0x70,0x65,0x72,0x73,0x6f,0x6e,0x00,0x00,0x52,0xb8,0x5e,0x3f,
    0x00,0x00,0x40,0x41,0x00,0x00,0x40,0x40,0x0e,0x00,0x00,0x00,0x74,0x72,0x61,0x63,
    0x6b,0x5f,0x6c,0x6f,0x6e,0x67,0x5f,0x69,0x64,0x00,0x00,0x00,0x0a,0x00,0x00,0x00,
    0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x33,0x33,0x3f,0xcd,0xcc,0x4c,0x3f,
    0xcd,0xcc,0xcc,0x3d,0xcd,0xcc,0xcc,0x3d,0x03,0x00,0x00,0x00,0x61,0x62,0x00,0x00,
    0x00,0x00,0x00,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,
    0x61,0x62,0x63,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

// kGoldenModelBytes: Model with 1 box (label="car") and 1 mask
// (encoding="raw", height=2, width=4, 8 data bytes, boxed=true).
// Sourced from testdata/cdr/edgefirst_msgs/Model.cdr via test_edgefirst_msgs.c.
static constexpr std::uint8_t kGoldenModelBytes[] = {
    0x00,0x01,0x00,0x00,0xd2,0x02,0x96,0x49,0x15,0xcd,0x5b,0x07,0x0b,0x00,0x00,0x00,
    0x74,0x65,0x73,0x74,0x5f,0x66,0x72,0x61,0x6d,0x65,0x00,0x00,0x00,0x00,0x00,0x00,
    0x40,0x42,0x0f,0x00,0x00,0x00,0x00,0x00,0x40,0x4b,0x4c,0x00,0x00,0x00,0x00,0x00,
    0x20,0xa1,0x07,0x00,0x00,0x00,0x00,0x00,0x40,0x0d,0x03,0x00,0x01,0x00,0x00,0x00,
    0x00,0x00,0x00,0x3f,0x00,0x00,0x00,0x3f,0xcd,0xcc,0xcc,0x3d,0xcd,0xcc,0x4c,0x3e,
    0x04,0x00,0x00,0x00,0x63,0x61,0x72,0x00,0x48,0xe1,0x7a,0x3f,0x00,0x00,0x20,0x41,
    0x00,0x00,0xa0,0x40,0x03,0x00,0x00,0x00,0x74,0x31,0x00,0x00,0x05,0x00,0x00,0x00,
    0x5f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x02,0x00,0x00,0x00,
    0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x08,0x00,0x00,0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x01
};

// ============================================================================
// Part A — Pointer identity: ImageView
// ============================================================================

TEST_CASE("ImageView data borrows from source CDR buffer", "[zero_copy][image]") {
    std::vector<std::uint8_t> pixels(640 * 480 * 3, 42);
    auto img = ef::Image::encode(
        {1, 2}, "cam", 480, 640, "rgb8", false,
        640 * 3, {pixels.data(), pixels.size()});
    REQUIRE(img.has_value());
    auto cdr = img->as_cdr();

    auto view = ef::ImageView::from_cdr(cdr);
    REQUIRE(view.has_value());

    // data() span must point into the CDR buffer, not a copy.
    // Cast through uintptr_t to avoid Catch2 StringMaker<char const*> calling
    // strlen on non-NUL-terminated byte arrays (ASan global-buffer-overflow).
    auto px = view->data();
    REQUIRE(px.size() == pixels.size());

    const auto base_addr = reinterpret_cast<std::uintptr_t>(cdr.data());
    const auto cdr_end   = base_addr + cdr.size();
    const auto px_addr   = reinterpret_cast<std::uintptr_t>(px.data());
    CHECK(px_addr >= base_addr);
    CHECK(px_addr < cdr_end);

    // encoding string_view must also borrow from the CDR buffer.
    auto enc = view->encoding();
    const auto enc_addr = reinterpret_cast<std::uintptr_t>(enc.data());
    CHECK(enc_addr >= base_addr);
    CHECK(enc_addr < cdr_end);
    CHECK(enc == "rgb8");
}

// ============================================================================
// Part A — Pointer identity: HeaderView::frame_id
// ============================================================================

TEST_CASE("HeaderView frame_id borrows from source CDR buffer", "[zero_copy][header]") {
    auto hdr = ef::Header::encode({10, 20}, "lidar_front");
    REQUIRE(hdr.has_value());
    auto cdr = hdr->as_cdr();

    auto view = ef::HeaderView::from_cdr(cdr);
    REQUIRE(view.has_value());

    auto fid = view->frame_id();
    const auto base_addr = reinterpret_cast<std::uintptr_t>(cdr.data());
    const auto fid_addr  = reinterpret_cast<std::uintptr_t>(fid.data());
    CHECK(fid_addr >= base_addr);
    CHECK(fid_addr < base_addr + cdr.size());
    CHECK(fid == "lidar_front");
}

// ============================================================================
// Part A — Pointer identity: CompressedImageView::data
// ============================================================================

TEST_CASE("CompressedImageView data borrows from source CDR buffer", "[zero_copy][compressed_image]") {
    std::vector<std::uint8_t> jpeg(512, 0xFF);
    auto ci = ef::CompressedImage::encode(
        {3, 4}, "cam", "jpeg", {jpeg.data(), jpeg.size()});
    REQUIRE(ci.has_value());
    auto cdr = ci->as_cdr();

    auto view = ef::CompressedImageView::from_cdr(cdr);
    REQUIRE(view.has_value());

    auto d = view->data();
    REQUIRE(d.size() == jpeg.size());

    const auto base_addr = reinterpret_cast<std::uintptr_t>(cdr.data());
    const auto cdr_end   = base_addr + cdr.size();
    const auto d_addr    = reinterpret_cast<std::uintptr_t>(d.data());
    CHECK(d_addr >= base_addr);
    CHECK(d_addr < cdr_end);

    // format string_view must also be within the buffer.
    auto fmt = view->format();
    const auto fmt_addr = reinterpret_cast<std::uintptr_t>(fmt.data());
    CHECK(fmt_addr >= base_addr);
    CHECK(fmt_addr < cdr_end);
    CHECK(fmt == "jpeg");
}

// ============================================================================
// Part A — Pointer identity: MaskView::data and MaskView::encoding
// ============================================================================

TEST_CASE("MaskView accessors borrow from CDR buffer", "[zero_copy][mask]") {
    std::vector<std::uint8_t> mask_data = {0xFF, 0x00, 0xFF, 0x00};
    auto mask = ef::Mask::encode(
        100, 200, static_cast<std::uint32_t>(mask_data.size()),
        "raw", {mask_data.data(), mask_data.size()}, false);
    REQUIRE(mask.has_value());
    auto cdr = mask->as_cdr();  // OwnedBaseNoCdr returns stored encoded bytes

    auto view = ef::MaskView::from_cdr(cdr);
    REQUIRE(view.has_value());

    const auto base_addr = reinterpret_cast<std::uintptr_t>(cdr.data());
    const auto cdr_end   = base_addr + cdr.size();

    // data() blob must borrow from the CDR buffer.
    auto blob = view->data();
    REQUIRE(blob.size() == mask_data.size());
    const auto blob_addr = reinterpret_cast<std::uintptr_t>(blob.data());
    CHECK(blob_addr >= base_addr);
    CHECK(blob_addr < cdr_end);

    // encoding string_view must borrow from the CDR buffer.
    auto enc = view->encoding();
    const auto enc_addr = reinterpret_cast<std::uintptr_t>(enc.data());
    CHECK(enc_addr >= base_addr);
    CHECK(enc_addr < cdr_end);
    CHECK(enc == "raw");
}

// ============================================================================
// Part A — Pointer identity: CompressedVideoView::data and ::format
// ============================================================================

TEST_CASE("CompressedVideoView accessors borrow from CDR buffer", "[zero_copy][compressed_video]") {
    std::vector<std::uint8_t> payload(256, 0xAB);
    auto cv = ef::CompressedVideo::encode(
        {5, 6}, "front_cam",
        {payload.data(), payload.size()}, "h264");
    REQUIRE(cv.has_value());
    auto cdr = cv->as_cdr();

    auto view = ef::CompressedVideoView::from_cdr(cdr);
    REQUIRE(view.has_value());

    const auto base_addr = reinterpret_cast<std::uintptr_t>(cdr.data());
    const auto cdr_end   = base_addr + cdr.size();

    // data() blob must borrow from the CDR buffer.
    auto blob = view->data();
    REQUIRE(blob.size() == payload.size());
    const auto blob_addr = reinterpret_cast<std::uintptr_t>(blob.data());
    CHECK(blob_addr >= base_addr);
    CHECK(blob_addr < cdr_end);

    // format string_view must borrow from the CDR buffer.
    auto fmt = view->format();
    const auto fmt_addr = reinterpret_cast<std::uintptr_t>(fmt.data());
    CHECK(fmt_addr >= base_addr);
    CHECK(fmt_addr < cdr_end);
    CHECK(fmt == "h264");
}

// ============================================================================
// Part A — Pointer identity: DmaBufferView::frame_id
// ============================================================================

TEST_CASE("DmaBufferView accessors borrow from CDR buffer", "[zero_copy][dmabuffer]") {
    auto dma = ef::DmaBuffer::encode(
        {100, 200}, "camera0",
        5678u, 10, 640u, 480u, 1280u, 0x56595559u, 614400u);
    REQUIRE(dma.has_value());
    auto cdr = dma->as_cdr();

    auto view = ef::DmaBufferView::from_cdr(cdr);
    REQUIRE(view.has_value());

    const auto base_addr = reinterpret_cast<std::uintptr_t>(cdr.data());

    // frame_id string_view must borrow from the CDR buffer.
    auto fid = view->frame_id();
    const auto fid_addr = reinterpret_cast<std::uintptr_t>(fid.data());
    CHECK(fid_addr >= base_addr);
    CHECK(fid_addr < base_addr + cdr.size());
    CHECK(fid == "camera0");
}

// ============================================================================
// Part A — Pointer identity: BorrowedBoxView::label and ::track_id
//          via DetectView::boxes() iteration
// ============================================================================

TEST_CASE("BorrowedBoxView accessors borrow from parent CDR buffer", "[zero_copy][detect][borrowed_box]") {
    auto det = ef::DetectView::from_cdr(
        ef::span<const std::uint8_t>{kGoldenDetectBytes, sizeof(kGoldenDetectBytes)});
    REQUIRE(det.has_value());
    REQUIRE(det->boxes_len() == 3u);

    const auto base_addr = reinterpret_cast<std::uintptr_t>(kGoldenDetectBytes);
    const auto end_addr  = base_addr + sizeof(kGoldenDetectBytes);

    for (auto box : det->boxes()) {
        // label() must borrow into the parent CDR buffer.
        auto lbl = box.label();
        const auto lbl_addr = reinterpret_cast<std::uintptr_t>(lbl.data());
        CHECK(lbl_addr >= base_addr);
        CHECK(lbl_addr <  end_addr);

        // track_id() must borrow into the parent CDR buffer.
        auto tid = box.track_id();
        const auto tid_addr = reinterpret_cast<std::uintptr_t>(tid.data());
        CHECK(tid_addr >= base_addr);
        CHECK(tid_addr <  end_addr);
    }
}

// ============================================================================
// Part A — Pointer identity: BorrowedMaskView::data and ::encoding
//          via ModelView::masks() iteration
//
// Use model->as_cdr() as the reference window so the test remains correct if
// the Rust layer's buffer strategy ever changes. Today, as_cdr() returns a
// pointer into the original input span (the Rust layer does not copy on
// decode), but the indirection through the handle insulates the test from
// any future change.
// ============================================================================

TEST_CASE("BorrowedMaskView accessors borrow from parent CDR buffer", "[zero_copy][model][borrowed_mask]") {
    auto model = ef::ModelView::from_cdr(
        ef::span<const std::uint8_t>{kGoldenModelBytes, sizeof(kGoldenModelBytes)});
    REQUIRE(model.has_value());
    REQUIRE(model->masks_len() == 1u);

    // Use the handle's own backing buffer as the reference window.
    auto parent_cdr  = model->as_cdr();
    const auto base_addr = reinterpret_cast<std::uintptr_t>(parent_cdr.data());
    const auto end_addr  = base_addr + parent_cdr.size();

    for (auto mask : model->masks()) {
        // data() blob must borrow into the parent handle's CDR buffer.
        auto blob = mask.data();
        if (!blob.empty()) {
            const auto blob_addr = reinterpret_cast<std::uintptr_t>(blob.data());
            CHECK(blob_addr >= base_addr);
            CHECK(blob_addr <  end_addr);
        }

        // encoding() must borrow into the parent handle's CDR buffer
        // (only checked when the encoding string is non-empty).
        auto enc = mask.encoding();
        if (!enc.empty()) {
            const auto enc_addr = reinterpret_cast<std::uintptr_t>(enc.data());
            CHECK(enc_addr >= base_addr);
            CHECK(enc_addr <  end_addr);
        }
    }
}

// ============================================================================
// Part B — Allocation counting: ImageView field access
//
// The counted region contains ONLY field-accessor calls — no REQUIRE/CHECK
// macros, which would themselves allocate when formatting failure messages.
// The setup (encode + from_cdr) runs outside the counted region so that
// construction allocations (vector, encode buffer) do not pollute the delta.
// ============================================================================

TEST_CASE("ImageView field access allocates zero heap memory", "[zero_copy][allocations]") {
    // Setup — outside counted region.
    std::vector<std::uint8_t> pixels(100, 7);
    auto img = ef::Image::encode(
        {1, 2}, "cam", 10, 10, "mono8", false, 10,
        {pixels.data(), pixels.size()});
    REQUIRE(img.has_value());
    auto cdr = img->as_cdr();
    auto view_result = ef::ImageView::from_cdr(cdr);
    REQUIRE(view_result.has_value());
    auto& view = *view_result;

    // Counted region — ONLY field accesses, no REQUIRE/CHECK inside.
    // We use a volatile sink to prevent dead-code elimination of the accessor
    // calls without making the return types themselves volatile (which would
    // conflict with string_view/span member function qualifiers).
    volatile std::uint8_t sink = 0;
    const std::size_t start_new = g_new_count.load(std::memory_order_relaxed);
    {
        auto w   = view.width();
        auto h   = view.height();
        auto st  = view.step();
        auto enc = view.encoding();
        auto px  = view.data();
        sink ^= static_cast<std::uint8_t>(w  & 0xFF);
        sink ^= static_cast<std::uint8_t>(h  & 0xFF);
        sink ^= static_cast<std::uint8_t>(st & 0xFF);
        sink ^= static_cast<std::uint8_t>(enc.size() & 0xFF);
        if (!px.empty()) sink ^= px[0];
    }
    const std::size_t delta = g_new_count.load(std::memory_order_relaxed) - start_new;
    (void)sink;

    // Assertion — outside counted region (Catch2 may allocate freely here).
    INFO("ImageView field access alloc delta: " << delta);
    CHECK(delta == 0);
}

// ============================================================================
// Part B — Allocation counting: HeaderView field access
// ============================================================================

TEST_CASE("HeaderView field access allocates zero heap memory", "[zero_copy][allocations]") {
    // Setup — outside counted region.
    auto hdr = ef::Header::encode({5, 6}, "radar_top");
    REQUIRE(hdr.has_value());
    auto cdr = hdr->as_cdr();
    auto view_result = ef::HeaderView::from_cdr(cdr);
    REQUIRE(view_result.has_value());
    auto& view = *view_result;

    // Counted region — ONLY field accesses, no REQUIRE/CHECK inside.
    volatile std::uint8_t sink = 0;
    const std::size_t start_new = g_new_count.load(std::memory_order_relaxed);
    {
        auto fid   = view.frame_id();
        auto stamp = view.stamp();
        sink ^= static_cast<std::uint8_t>(fid.size() & 0xFF);
        sink ^= static_cast<std::uint8_t>(stamp.sec & 0xFF);
    }
    const std::size_t delta = g_new_count.load(std::memory_order_relaxed) - start_new;
    (void)sink;

    // Assertion — outside counted region.
    INFO("HeaderView field access alloc delta: " << delta);
    CHECK(delta == 0);
}

// ============================================================================
// Part B — Allocation counting: CompressedImageView field access
// ============================================================================

TEST_CASE("CompressedImageView field access allocates zero heap memory", "[zero_copy][allocations]") {
    // Setup — outside counted region.
    std::vector<std::uint8_t> jpeg(256, 0xFF);
    auto ci = ef::CompressedImage::encode(
        {3, 4}, "cam", "jpeg", {jpeg.data(), jpeg.size()});
    REQUIRE(ci.has_value());
    auto cdr = ci->as_cdr();
    auto view_result = ef::CompressedImageView::from_cdr(cdr);
    REQUIRE(view_result.has_value());
    auto& view = *view_result;

    // Counted region — ONLY field accesses, no REQUIRE/CHECK inside.
    volatile std::uint8_t sink = 0;
    const std::size_t start_new = g_new_count.load(std::memory_order_relaxed);
    {
        auto fmt  = view.format();
        auto d    = view.data();
        auto fid  = view.frame_id();
        sink ^= static_cast<std::uint8_t>(fmt.size() & 0xFF);
        if (!d.empty()) sink ^= d[0];
        sink ^= static_cast<std::uint8_t>(fid.size() & 0xFF);
    }
    const std::size_t delta = g_new_count.load(std::memory_order_relaxed) - start_new;
    (void)sink;

    // Assertion — outside counted region.
    INFO("CompressedImageView field access alloc delta: " << delta);
    CHECK(delta == 0);
}

// ============================================================================
// Part B — Allocation counting: MaskView field access
// ============================================================================

TEST_CASE("MaskView field access allocates zero heap memory", "[zero_copy][allocations]") {
    // Setup — outside counted region.
    std::vector<std::uint8_t> maskdata(16, 0xAB);
    auto m = ef::Mask::encode(
        4u, 4u, static_cast<std::uint32_t>(maskdata.size()),
        "raw", {maskdata.data(), maskdata.size()}, false);
    REQUIRE(m.has_value());
    auto cdr = m->as_cdr();
    auto view_result = ef::MaskView::from_cdr(cdr);
    REQUIRE(view_result.has_value());
    auto& view = *view_result;

    // Counted region — ONLY field accesses, no REQUIRE/CHECK inside.
    volatile std::uint8_t sink = 0;
    const std::size_t start_new = g_new_count.load(std::memory_order_relaxed);
    {
        volatile auto h   = view.height();
        volatile auto w   = view.width();
        auto enc          = view.encoding();
        auto data         = view.data();
        sink ^= static_cast<std::uint8_t>(h & 0xFF);
        sink ^= static_cast<std::uint8_t>(w & 0xFF);
        sink ^= static_cast<std::uint8_t>(enc.size() & 0xFF);
        if (!data.empty()) sink ^= data[0];
    }
    const std::size_t delta = g_new_count.load(std::memory_order_relaxed) - start_new;
    (void)sink;

    // Assertion — outside counted region.
    INFO("MaskView field access alloc delta: " << delta);
    CHECK(delta == 0);
}

// ============================================================================
// Part B — Allocation counting: CompressedVideoView field access
// ============================================================================

TEST_CASE("CompressedVideoView field access allocates zero heap memory", "[zero_copy][allocations]") {
    // Setup — outside counted region.
    std::vector<std::uint8_t> payload(128, 0x55);
    auto cv = ef::CompressedVideo::encode(
        {5, 6}, "front_cam",
        {payload.data(), payload.size()}, "h264");
    REQUIRE(cv.has_value());
    auto cdr = cv->as_cdr();
    auto view_result = ef::CompressedVideoView::from_cdr(cdr);
    REQUIRE(view_result.has_value());
    auto& view = *view_result;

    // Counted region — ONLY field accesses, no REQUIRE/CHECK inside.
    volatile std::uint8_t sink = 0;
    const std::size_t start_new = g_new_count.load(std::memory_order_relaxed);
    {
        auto fmt = view.format();
        auto d   = view.data();
        auto fid = view.frame_id();
        sink ^= static_cast<std::uint8_t>(fmt.size() & 0xFF);
        if (!d.empty()) sink ^= d[0];
        sink ^= static_cast<std::uint8_t>(fid.size() & 0xFF);
    }
    const std::size_t delta = g_new_count.load(std::memory_order_relaxed) - start_new;
    (void)sink;

    // Assertion — outside counted region.
    INFO("CompressedVideoView field access alloc delta: " << delta);
    CHECK(delta == 0);
}

// ============================================================================
// Part B — Allocation counting: DmaBufferView field access
// ============================================================================

TEST_CASE("DmaBufferView field access allocates zero heap memory", "[zero_copy][allocations]") {
    // Setup — outside counted region.
    auto dma = ef::DmaBuffer::encode(
        {100, 200}, "camera0",
        5678u, 10, 640u, 480u, 1280u, 0x56595559u, 614400u);
    REQUIRE(dma.has_value());
    auto cdr = dma->as_cdr();
    auto view_result = ef::DmaBufferView::from_cdr(cdr);
    REQUIRE(view_result.has_value());
    auto& view = *view_result;

    // Counted region — ONLY field accesses, no REQUIRE/CHECK inside.
    volatile std::uint8_t sink = 0;
    const std::size_t start_new = g_new_count.load(std::memory_order_relaxed);
    {
        auto fid = view.frame_id();
        volatile auto pid = view.pid();
        volatile auto fd  = view.fd();
        volatile auto w   = view.width();
        volatile auto h   = view.height();
        sink ^= static_cast<std::uint8_t>(fid.size() & 0xFF);
        sink ^= static_cast<std::uint8_t>(pid & 0xFF);
        sink ^= static_cast<std::uint8_t>(static_cast<std::uint32_t>(fd) & 0xFF);
        sink ^= static_cast<std::uint8_t>(w & 0xFF);
        sink ^= static_cast<std::uint8_t>(h & 0xFF);
    }
    const std::size_t delta = g_new_count.load(std::memory_order_relaxed) - start_new;
    (void)sink;

    // Assertion — outside counted region.
    INFO("DmaBufferView field access alloc delta: " << delta);
    CHECK(delta == 0);
}

// ============================================================================
// Part B — Allocation counting: BorrowedBoxView field access via DetectView
// ============================================================================

TEST_CASE("BorrowedBoxView field access allocates zero heap memory", "[zero_copy][allocations]") {
    // Setup — outside counted region.
    auto det = ef::DetectView::from_cdr(
        ef::span<const std::uint8_t>{kGoldenDetectBytes, sizeof(kGoldenDetectBytes)});
    REQUIRE(det.has_value());
    REQUIRE(det->boxes_len() >= 1u);

    // Counted region — iterate boxes and access every field, no REQUIRE/CHECK inside.
    volatile std::uint8_t sink = 0;
    const std::size_t start_new = g_new_count.load(std::memory_order_relaxed);
    {
        for (auto box : det->boxes()) {
            volatile auto cx  = box.center_x();
            volatile auto cy  = box.center_y();
            volatile auto bw  = box.width();
            volatile auto bh  = box.height();
            auto lbl          = box.label();
            volatile auto sc  = box.score();
            auto tid          = box.track_id();
            volatile auto tlt = box.track_lifetime();
            sink ^= static_cast<std::uint8_t>(static_cast<int>(cx * 255) & 0xFF);
            sink ^= static_cast<std::uint8_t>(static_cast<int>(cy * 255) & 0xFF);
            sink ^= static_cast<std::uint8_t>(static_cast<int>(bw * 255) & 0xFF);
            sink ^= static_cast<std::uint8_t>(static_cast<int>(bh * 255) & 0xFF);
            sink ^= static_cast<std::uint8_t>(lbl.size() & 0xFF);
            sink ^= static_cast<std::uint8_t>(static_cast<int>(sc * 255) & 0xFF);
            sink ^= static_cast<std::uint8_t>(tid.size() & 0xFF);
            sink ^= static_cast<std::uint8_t>(static_cast<std::uint32_t>(tlt) & 0xFF);
        }
    }
    const std::size_t delta = g_new_count.load(std::memory_order_relaxed) - start_new;
    (void)sink;

    // Assertion — outside counted region.
    INFO("BorrowedBoxView field access alloc delta: " << delta);
    CHECK(delta == 0);
}

// ============================================================================
// Part B — Allocation counting: BorrowedMaskView field access via ModelView
// ============================================================================

TEST_CASE("BorrowedMaskView field access allocates zero heap memory", "[zero_copy][allocations]") {
    // Setup — outside counted region.
    auto model = ef::ModelView::from_cdr(
        ef::span<const std::uint8_t>{kGoldenModelBytes, sizeof(kGoldenModelBytes)});
    REQUIRE(model.has_value());
    REQUIRE(model->masks_len() >= 1u);

    // Counted region — iterate masks and access every field, no REQUIRE/CHECK inside.
    volatile std::uint8_t sink = 0;
    const std::size_t start_new = g_new_count.load(std::memory_order_relaxed);
    {
        for (auto mask : model->masks()) {
            volatile auto h  = mask.height();
            volatile auto w  = mask.width();
            auto enc         = mask.encoding();
            auto d           = mask.data();
            volatile auto bx = mask.boxed();
            sink ^= static_cast<std::uint8_t>(h & 0xFF);
            sink ^= static_cast<std::uint8_t>(w & 0xFF);
            sink ^= static_cast<std::uint8_t>(enc.size() & 0xFF);
            if (!d.empty()) sink ^= d[0];
            sink ^= static_cast<std::uint8_t>(bx ? 1u : 0u);
        }
    }
    const std::size_t delta = g_new_count.load(std::memory_order_relaxed) - start_new;
    (void)sink;

    // Assertion — outside counted region.
    INFO("BorrowedMaskView field access alloc delta: " << delta);
    CHECK(delta == 0);
}

#pragma GCC diagnostic pop
