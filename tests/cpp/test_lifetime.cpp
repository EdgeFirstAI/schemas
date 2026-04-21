/**
 * @file test_lifetime.cpp
 * @brief Compile-time and runtime move-semantics invariant tests for the C++ wrapper.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.
 *
 * Verifies:
 *  - CdrFixed value types are trivially copyable and nothrow default constructible.
 *  - View types and Owning types are move-only (no copy, nothrow move).
 *  - Runtime move construct / move assign leave the moved-from object safely
 *    destructible (no double-free under ASan).
 *  - Self-move does not corrupt state.
 */

// DmaBuffer / DmaBufferView are deprecated in 3.1.0; these tests stay
// in place through the deprecation window.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

#include <type_traits>
#include <vector>
#include <cstdint>

namespace ef = edgefirst::schemas;

// ============================================================================
// Compile-time: CdrFixed value types
// Must be trivially copyable and nothrow default constructible.
// ============================================================================

static_assert(std::is_trivially_copyable_v<ef::Time>);
static_assert(std::is_nothrow_default_constructible_v<ef::Time>);

static_assert(std::is_trivially_copyable_v<ef::Duration>);
static_assert(std::is_nothrow_default_constructible_v<ef::Duration>);

static_assert(std::is_trivially_copyable_v<ef::Vector3>);
static_assert(std::is_nothrow_default_constructible_v<ef::Vector3>);

static_assert(std::is_trivially_copyable_v<ef::Point>);
static_assert(std::is_nothrow_default_constructible_v<ef::Point>);

static_assert(std::is_trivially_copyable_v<ef::Quaternion>);
static_assert(std::is_nothrow_default_constructible_v<ef::Quaternion>);

static_assert(std::is_trivially_copyable_v<ef::Pose>);
static_assert(std::is_nothrow_default_constructible_v<ef::Pose>);

static_assert(std::is_trivially_copyable_v<ef::Transform>);
static_assert(std::is_nothrow_default_constructible_v<ef::Transform>);

static_assert(std::is_trivially_copyable_v<ef::Twist>);
static_assert(std::is_nothrow_default_constructible_v<ef::Twist>);

static_assert(std::is_trivially_copyable_v<ef::Accel>);
static_assert(std::is_nothrow_default_constructible_v<ef::Accel>);

static_assert(std::is_trivially_copyable_v<ef::NavSatStatus>);
static_assert(std::is_nothrow_default_constructible_v<ef::NavSatStatus>);

// ============================================================================
// Compile-time: View types — move-only
// ============================================================================

static_assert(!std::is_copy_constructible_v<ef::HeaderView>);
static_assert(!std::is_copy_assignable_v<ef::HeaderView>);
static_assert(std::is_nothrow_move_constructible_v<ef::HeaderView>);
static_assert(std::is_nothrow_move_assignable_v<ef::HeaderView>);

static_assert(!std::is_copy_constructible_v<ef::CompressedImageView>);
static_assert(!std::is_copy_assignable_v<ef::CompressedImageView>);
static_assert(std::is_nothrow_move_constructible_v<ef::CompressedImageView>);
static_assert(std::is_nothrow_move_assignable_v<ef::CompressedImageView>);

static_assert(!std::is_copy_constructible_v<ef::ImuView>);
static_assert(!std::is_copy_assignable_v<ef::ImuView>);
static_assert(std::is_nothrow_move_constructible_v<ef::ImuView>);
static_assert(std::is_nothrow_move_assignable_v<ef::ImuView>);

static_assert(!std::is_copy_constructible_v<ef::NavSatFixView>);
static_assert(!std::is_copy_assignable_v<ef::NavSatFixView>);
static_assert(std::is_nothrow_move_constructible_v<ef::NavSatFixView>);
static_assert(std::is_nothrow_move_assignable_v<ef::NavSatFixView>);

static_assert(!std::is_copy_constructible_v<ef::CameraInfoView>);
static_assert(!std::is_copy_assignable_v<ef::CameraInfoView>);
static_assert(std::is_nothrow_move_constructible_v<ef::CameraInfoView>);
static_assert(std::is_nothrow_move_assignable_v<ef::CameraInfoView>);

static_assert(!std::is_copy_constructible_v<ef::TransformStampedView>);
static_assert(!std::is_copy_assignable_v<ef::TransformStampedView>);
static_assert(std::is_nothrow_move_constructible_v<ef::TransformStampedView>);
static_assert(std::is_nothrow_move_assignable_v<ef::TransformStampedView>);

static_assert(!std::is_copy_constructible_v<ef::CompressedVideoView>);
static_assert(!std::is_copy_assignable_v<ef::CompressedVideoView>);
static_assert(std::is_nothrow_move_constructible_v<ef::CompressedVideoView>);
static_assert(std::is_nothrow_move_assignable_v<ef::CompressedVideoView>);

static_assert(!std::is_copy_constructible_v<ef::MaskView>);
static_assert(!std::is_copy_assignable_v<ef::MaskView>);
static_assert(std::is_nothrow_move_constructible_v<ef::MaskView>);
static_assert(std::is_nothrow_move_assignable_v<ef::MaskView>);

static_assert(!std::is_copy_constructible_v<ef::DmaBufferView>);
static_assert(!std::is_copy_assignable_v<ef::DmaBufferView>);
static_assert(std::is_nothrow_move_constructible_v<ef::DmaBufferView>);
static_assert(std::is_nothrow_move_assignable_v<ef::DmaBufferView>);

static_assert(!std::is_copy_constructible_v<ef::LocalTimeView>);
static_assert(!std::is_copy_assignable_v<ef::LocalTimeView>);
static_assert(std::is_nothrow_move_constructible_v<ef::LocalTimeView>);
static_assert(std::is_nothrow_move_assignable_v<ef::LocalTimeView>);

static_assert(!std::is_copy_constructible_v<ef::TrackView>);
static_assert(!std::is_copy_assignable_v<ef::TrackView>);
static_assert(std::is_nothrow_move_constructible_v<ef::TrackView>);
static_assert(std::is_nothrow_move_assignable_v<ef::TrackView>);

static_assert(!std::is_copy_constructible_v<ef::ImageView>);
static_assert(!std::is_copy_assignable_v<ef::ImageView>);
static_assert(std::is_nothrow_move_constructible_v<ef::ImageView>);
static_assert(std::is_nothrow_move_assignable_v<ef::ImageView>);

static_assert(!std::is_copy_constructible_v<ef::PointCloud2View>);
static_assert(!std::is_copy_assignable_v<ef::PointCloud2View>);
static_assert(std::is_nothrow_move_constructible_v<ef::PointCloud2View>);
static_assert(std::is_nothrow_move_assignable_v<ef::PointCloud2View>);

static_assert(!std::is_copy_constructible_v<ef::RadarCubeView>);
static_assert(!std::is_copy_assignable_v<ef::RadarCubeView>);
static_assert(std::is_nothrow_move_constructible_v<ef::RadarCubeView>);
static_assert(std::is_nothrow_move_assignable_v<ef::RadarCubeView>);

static_assert(!std::is_copy_constructible_v<ef::RadarInfoView>);
static_assert(!std::is_copy_assignable_v<ef::RadarInfoView>);
static_assert(std::is_nothrow_move_constructible_v<ef::RadarInfoView>);
static_assert(std::is_nothrow_move_assignable_v<ef::RadarInfoView>);

static_assert(!std::is_copy_constructible_v<ef::BoxView>);
static_assert(!std::is_copy_assignable_v<ef::BoxView>);
static_assert(std::is_nothrow_move_constructible_v<ef::BoxView>);
static_assert(std::is_nothrow_move_assignable_v<ef::BoxView>);

// BorrowedBoxView / BorrowedMaskView: yielded by ChildRange iteration.
// Non-default-constructible (require a parent-borrowed handle at construction),
// copy-constructible (range-based for loops copy-construct the loop variable),
// non-copy-assignable (prevents storing past parent lifetime),
// non-move-assignable (same reason).
static_assert(!std::is_default_constructible_v<ef::detail::BorrowedBoxView>);
static_assert( std::is_copy_constructible_v<ef::detail::BorrowedBoxView>);
static_assert(!std::is_copy_assignable_v<ef::detail::BorrowedBoxView>);
static_assert( std::is_move_constructible_v<ef::detail::BorrowedBoxView>);
static_assert(!std::is_move_assignable_v<ef::detail::BorrowedBoxView>);
static_assert( std::is_trivially_destructible_v<ef::detail::BorrowedBoxView>);

static_assert(!std::is_default_constructible_v<ef::detail::BorrowedMaskView>);
static_assert( std::is_copy_constructible_v<ef::detail::BorrowedMaskView>);
static_assert(!std::is_copy_assignable_v<ef::detail::BorrowedMaskView>);
static_assert( std::is_move_constructible_v<ef::detail::BorrowedMaskView>);
static_assert(!std::is_move_assignable_v<ef::detail::BorrowedMaskView>);
static_assert( std::is_trivially_destructible_v<ef::detail::BorrowedMaskView>);

static_assert(!std::is_copy_constructible_v<ef::DetectView>);
static_assert(!std::is_copy_assignable_v<ef::DetectView>);
static_assert(std::is_nothrow_move_constructible_v<ef::DetectView>);
static_assert(std::is_nothrow_move_assignable_v<ef::DetectView>);

static_assert(!std::is_copy_constructible_v<ef::ModelView>);
static_assert(!std::is_copy_assignable_v<ef::ModelView>);
static_assert(std::is_nothrow_move_constructible_v<ef::ModelView>);
static_assert(std::is_nothrow_move_assignable_v<ef::ModelView>);

static_assert(!std::is_copy_constructible_v<ef::ModelInfoView>);
static_assert(!std::is_copy_assignable_v<ef::ModelInfoView>);
static_assert(std::is_nothrow_move_constructible_v<ef::ModelInfoView>);
static_assert(std::is_nothrow_move_assignable_v<ef::ModelInfoView>);

// ============================================================================
// Compile-time: Owning types — move-only
// ============================================================================

static_assert(!std::is_copy_constructible_v<ef::Header>);
static_assert(!std::is_copy_assignable_v<ef::Header>);
static_assert(std::is_nothrow_move_constructible_v<ef::Header>);
static_assert(std::is_nothrow_move_assignable_v<ef::Header>);

static_assert(!std::is_copy_constructible_v<ef::CompressedImage>);
static_assert(!std::is_copy_assignable_v<ef::CompressedImage>);
static_assert(std::is_nothrow_move_constructible_v<ef::CompressedImage>);
static_assert(std::is_nothrow_move_assignable_v<ef::CompressedImage>);

static_assert(!std::is_copy_constructible_v<ef::CompressedVideo>);
static_assert(!std::is_copy_assignable_v<ef::CompressedVideo>);
static_assert(std::is_nothrow_move_constructible_v<ef::CompressedVideo>);
static_assert(std::is_nothrow_move_assignable_v<ef::CompressedVideo>);

static_assert(!std::is_copy_constructible_v<ef::DmaBuffer>);
static_assert(!std::is_copy_assignable_v<ef::DmaBuffer>);
static_assert(std::is_nothrow_move_constructible_v<ef::DmaBuffer>);
static_assert(std::is_nothrow_move_assignable_v<ef::DmaBuffer>);

static_assert(!std::is_copy_constructible_v<ef::Mask>);
static_assert(!std::is_copy_assignable_v<ef::Mask>);
static_assert(std::is_nothrow_move_constructible_v<ef::Mask>);
static_assert(std::is_nothrow_move_assignable_v<ef::Mask>);

static_assert(!std::is_copy_constructible_v<ef::Image>);
static_assert(!std::is_copy_assignable_v<ef::Image>);
static_assert(std::is_nothrow_move_constructible_v<ef::Image>);
static_assert(std::is_nothrow_move_assignable_v<ef::Image>);

// ============================================================================
// Runtime: HeaderView move semantics
// ============================================================================

TEST_CASE("HeaderView move construct leaves source safely destructible", "[lifetime]") {
    auto hdr = ef::Header::encode({1, 2}, "cam");
    REQUIRE(hdr.has_value());
    auto cdr = hdr->as_cdr();
    auto v1 = ef::HeaderView::from_cdr(cdr);
    REQUIRE(v1.has_value());

    // Move construct: v2 owns the handle, v1 is in a moved-from (null handle) state.
    auto v2 = std::move(*v1);
    CHECK(v2.frame_id() == "cam");
    // v1's destructor must not double-free — verified by running under ASan.
}

TEST_CASE("HeaderView move assign", "[lifetime]") {
    auto hdr1 = ef::Header::encode({1, 2}, "cam1");
    auto hdr2 = ef::Header::encode({3, 4}, "cam2");
    REQUIRE(hdr1.has_value());
    REQUIRE(hdr2.has_value());
    auto v1 = ef::HeaderView::from_cdr(hdr1->as_cdr());
    auto v2 = ef::HeaderView::from_cdr(hdr2->as_cdr());
    REQUIRE(v1.has_value());
    REQUIRE(v2.has_value());

    // Move assign: v1 should now see cam2, v2 is safely destructible.
    *v1 = std::move(*v2);
    CHECK(v1->frame_id() == "cam2");
}

TEST_CASE("HeaderView self-move is safe", "[lifetime]") {
    auto hdr = ef::Header::encode({1, 2}, "cam");
    REQUIRE(hdr.has_value());
    auto view = ef::HeaderView::from_cdr(hdr->as_cdr());
    REQUIRE(view.has_value());

    // Use an indirection through a pointer to suppress compiler self-move warnings
    // while still exercising the self-move code path.
    ef::HeaderView* p = &(*view);
    *p = std::move(*view);  // self-move via pointer alias
    CHECK(view->frame_id() == "cam");
}

// ============================================================================
// Runtime: Header (owning) move semantics
// ============================================================================

TEST_CASE("Header move construct leaves source safely destructible", "[lifetime]") {
    auto h1 = ef::Header::encode({10, 20}, "lidar");
    REQUIRE(h1.has_value());

    auto h2 = std::move(*h1);
    CHECK(h2.frame_id() == "lidar");
    CHECK(h2.stamp().sec == 10);
    CHECK(h2.stamp().nanosec == 20);
    // h1 destructor must be safe — verified by ASan.
}

TEST_CASE("Header move assign", "[lifetime]") {
    auto h1 = ef::Header::encode({1, 0}, "a");
    auto h2 = ef::Header::encode({2, 0}, "b");
    REQUIRE(h1.has_value());
    REQUIRE(h2.has_value());

    *h1 = std::move(*h2);
    CHECK(h1->frame_id() == "b");
}

TEST_CASE("Header self-move is safe", "[lifetime]") {
    auto h = ef::Header::encode({5, 6}, "self");
    REQUIRE(h.has_value());

    // Use pointer indirection to suppress compiler self-move warnings.
    ef::Header* p = &(*h);
    *p = std::move(*h);
    CHECK(h->frame_id() == "self");
}

// ============================================================================
// Runtime: Image (owning) move semantics
// ============================================================================

TEST_CASE("Image move construct leaves source safely destructible", "[lifetime]") {
    std::vector<std::uint8_t> pixels(640 * 480 * 3, 42);
    auto img = ef::Image::encode(
        {1, 0}, "cam", 480, 640, "rgb8", false, 640 * 3,
        {pixels.data(), pixels.size()});
    REQUIRE(img.has_value());

    auto img2 = std::move(*img);
    CHECK(img2.frame_id() == "cam");
    CHECK(img2.width() == 640);
    CHECK(img2.height() == 480);
    // img destructor must be safe — verified by ASan.
}

TEST_CASE("Image move assign", "[lifetime]") {
    std::vector<std::uint8_t> px1(100, 1);
    std::vector<std::uint8_t> px2(200, 2);
    auto img1 = ef::Image::encode({1, 0}, "a", 10, 10, "mono8", false, 10,
                                   {px1.data(), px1.size()});
    auto img2 = ef::Image::encode({2, 0}, "b", 20, 10, "mono8", false, 10,
                                   {px2.data(), px2.size()});
    REQUIRE(img1.has_value());
    REQUIRE(img2.has_value());

    *img1 = std::move(*img2);
    CHECK(img1->frame_id() == "b");
    CHECK(img1->height() == 20);
}

// ============================================================================
// Runtime: ImageView move semantics
// ============================================================================

TEST_CASE("ImageView move construct leaves source safely destructible", "[lifetime]") {
    std::vector<std::uint8_t> pixels(100, 7);
    auto img = ef::Image::encode({1, 0}, "cam", 10, 10, "mono8", false, 10,
                                  {pixels.data(), pixels.size()});
    REQUIRE(img.has_value());
    auto cdr = img->as_cdr();

    auto v1 = ef::ImageView::from_cdr(cdr);
    REQUIRE(v1.has_value());

    auto v2 = std::move(*v1);
    CHECK(v2.frame_id() == "cam");
    CHECK(v2.width() == 10);
    // v1 destructor must be safe — verified by ASan.
}

TEST_CASE("ImageView move assign", "[lifetime]") {
    std::vector<std::uint8_t> px(100, 5);
    auto img1 = ef::Image::encode({1, 0}, "c1", 10, 10, "mono8", false, 10,
                                   {px.data(), px.size()});
    auto img2 = ef::Image::encode({2, 0}, "c2", 10, 10, "mono8", false, 10,
                                   {px.data(), px.size()});
    REQUIRE(img1.has_value());
    REQUIRE(img2.has_value());

    auto v1 = ef::ImageView::from_cdr(img1->as_cdr());
    auto v2 = ef::ImageView::from_cdr(img2->as_cdr());
    REQUIRE(v1.has_value());
    REQUIRE(v2.has_value());

    *v1 = std::move(*v2);
    CHECK(v1->frame_id() == "c2");
}

// ============================================================================
// Runtime: Mask (NoCdr owning) move semantics
// ============================================================================

TEST_CASE("Mask move construct leaves source safely destructible", "[lifetime]") {
    std::vector<std::uint8_t> mdata(64, 0xAA);
    auto m1 = ef::Mask::encode(8, 8, 64, "rle", {mdata.data(), mdata.size()}, false);
    REQUIRE(m1.has_value());

    auto m2 = std::move(*m1);
    CHECK(m2.height() == 8);
    CHECK(m2.width() == 8);
    CHECK(m2.encoding() == "rle");
    // m1 destructor must be safe — verified by ASan.
}

// ============================================================================
// Golden CDR fixtures — inline copies to avoid cross-TU symbol collisions.
// Source: kGoldenDetectBytes from tests/cpp/test_detect.cpp
//   stamp.sec=1234567890, stamp.nanosec=123456789, frame_id="test_frame", 3 boxes
// Source: kGoldenModelBytes from tests/cpp/test_zero_copy.cpp
//   stamp.sec=1234567890, stamp.nanosec=123456789, frame_id="test_frame",
//   1 box (label="car"), 1 mask (encoding="raw", height=2, width=4)
// ============================================================================

static constexpr std::uint8_t kGoldenDetectBytesForMove[] = {
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

static constexpr std::uint8_t kGoldenModelBytesForMove[] = {
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
// Runtime: DetectView move semantics
// ============================================================================

TEST_CASE("DetectView error paths", "[lifetime][detect]") {
    auto bad = ef::DetectView::from_cdr({});
    CHECK_FALSE(bad.has_value());
}

TEST_CASE("DetectView move semantics", "[lifetime][detect]") {
    auto v1 = ef::DetectView::from_cdr(
        {kGoldenDetectBytesForMove, sizeof(kGoldenDetectBytesForMove)});
    REQUIRE(v1.has_value());
    const auto orig_frame = std::string(v1->frame_id());

    // Move construct
    auto v2 = std::move(*v1);
    CHECK(v2.frame_id() == orig_frame);
    CHECK(v2.boxes_len() == 3u);
    // v1 is moved-from; destructor must be safe (ASan would catch double-free).

    // Move assign from a fresh view
    auto v3 = ef::DetectView::from_cdr(
        {kGoldenDetectBytesForMove, sizeof(kGoldenDetectBytesForMove)});
    REQUIRE(v3.has_value());
    v2 = std::move(*v3);
    CHECK(v2.frame_id() == orig_frame);
    CHECK(v2.boxes_len() == 3u);
}

// ============================================================================
// Runtime: ModelView move semantics
// ============================================================================

TEST_CASE("ModelView error paths", "[lifetime][model]") {
    auto bad = ef::ModelView::from_cdr({});
    CHECK_FALSE(bad.has_value());
}

TEST_CASE("ModelView move semantics", "[lifetime][model]") {
    auto v1 = ef::ModelView::from_cdr(
        {kGoldenModelBytesForMove, sizeof(kGoldenModelBytesForMove)});
    REQUIRE(v1.has_value());
    const auto orig_frame = std::string(v1->frame_id());

    // Move construct
    auto v2 = std::move(*v1);
    CHECK(v2.frame_id() == orig_frame);
    CHECK(v2.masks_len() == 1u);
    // v1 is moved-from; destructor must be safe (ASan would catch double-free).

    // Move assign from a fresh view
    auto v3 = ef::ModelView::from_cdr(
        {kGoldenModelBytesForMove, sizeof(kGoldenModelBytesForMove)});
    REQUIRE(v3.has_value());
    v2 = std::move(*v3);
    CHECK(v2.frame_id() == orig_frame);
    CHECK(v2.masks_len() == 1u);
}

// ============================================================================
// Runtime: Time (CdrFixed) — trivially copyable, copy works as expected
// ============================================================================

TEST_CASE("Time is trivially copyable: copy and original are independent", "[lifetime]") {
    ef::Time t1{100, 200};
    ef::Time t2 = t1;  // trivial copy
    CHECK(t2.sec == 100);
    CHECK(t2.nanosec == 200);

    t2.sec = 999;
    CHECK(t1.sec == 100);  // t1 unaffected by modification of t2
}

#pragma GCC diagnostic pop
