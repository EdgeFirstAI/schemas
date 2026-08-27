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

#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

#include <type_traits>
#include <vector>
#include <cstdint>

namespace ef = edgefirst::schemas;
using ef::builtin_interfaces::Time;
using ef::builtin_interfaces::Duration;
using ef::geometry_msgs::Vector3;
using ef::geometry_msgs::Point;
using ef::geometry_msgs::Point32;
using ef::geometry_msgs::Quaternion;
using ef::geometry_msgs::Pose;
using ef::geometry_msgs::Transform;
using ef::geometry_msgs::Twist;
using ef::geometry_msgs::Accel;
using ef::geometry_msgs::Wrench;
using ef::geometry_msgs::PoseWithCovariance;
using ef::geometry_msgs::TwistWithCovariance;
using ef::geometry_msgs::AccelWithCovariance;
using ef::geometry_msgs::AccelStampedView;
using ef::geometry_msgs::AccelStampedBuilder;
using ef::geometry_msgs::TwistStampedView;
using ef::geometry_msgs::TwistStampedBuilder;
using ef::geometry_msgs::WrenchStampedView;
using ef::geometry_msgs::WrenchStampedBuilder;
using ef::geometry_msgs::PointStampedView;
using ef::geometry_msgs::PointStampedBuilder;
using ef::geometry_msgs::InertiaStampedView;
using ef::geometry_msgs::InertiaStampedBuilder;
using ef::geometry_msgs::Vector3StampedView;
using ef::geometry_msgs::Vector3StampedBuilder;
using ef::geometry_msgs::PoseStampedView;
using ef::geometry_msgs::PoseStampedBuilder;
using ef::geometry_msgs::QuaternionStampedView;
using ef::geometry_msgs::QuaternionStampedBuilder;
using ef::geometry_msgs::PoseWithCovarianceStampedView;
using ef::geometry_msgs::PoseWithCovarianceStampedBuilder;
using ef::geometry_msgs::TwistWithCovarianceStampedView;
using ef::geometry_msgs::TwistWithCovarianceStampedBuilder;
using ef::geometry_msgs::AccelWithCovarianceStampedView;
using ef::geometry_msgs::AccelWithCovarianceStampedBuilder;
using ef::geometry_msgs::PolygonView;
using ef::geometry_msgs::PolygonBuilder;
using ef::geometry_msgs::PolygonStampedView;
using ef::geometry_msgs::PolygonStampedBuilder;
using ef::geometry_msgs::PoseArrayView;
using ef::geometry_msgs::PoseArrayBuilder;
using ef::geometry_msgs::TransformStampedView;
using ef::geometry_msgs::TransformStampedBuilder;
using ef::std_msgs::Header;
using ef::std_msgs::HeaderView;
using ef::std_msgs::HeaderBuilder;
using ef::sensor_msgs::NavSatStatus;
using ef::sensor_msgs::CompressedImage;
using ef::sensor_msgs::CompressedImageView;
using ef::sensor_msgs::CompressedImageBuilder;
using ef::sensor_msgs::Image;
using ef::sensor_msgs::ImageView;
using ef::sensor_msgs::ImageBuilder;
using ef::sensor_msgs::ImuView;
using ef::sensor_msgs::ImuBuilder;
using ef::sensor_msgs::NavSatFixView;
using ef::sensor_msgs::NavSatFixBuilder;
using ef::sensor_msgs::CameraInfoView;
using ef::sensor_msgs::CameraInfoBuilder;
using ef::sensor_msgs::PointCloud2View;
using ef::sensor_msgs::PointCloud2Builder;
using ef::sensor_msgs::PointFieldBuilder;
using ef::sensor_msgs::MagneticFieldView;
using ef::sensor_msgs::MagneticFieldBuilder;
using ef::sensor_msgs::FluidPressureView;
using ef::sensor_msgs::FluidPressureBuilder;
using ef::sensor_msgs::TemperatureView;
using ef::sensor_msgs::TemperatureBuilder;
using ef::sensor_msgs::BatteryStateView;
using ef::sensor_msgs::BatteryStateBuilder;
using ef::sensor_msgs::RelativeHumidityView;
using ef::sensor_msgs::RelativeHumidityBuilder;
using ef::sensor_msgs::TimeReferenceView;
using ef::sensor_msgs::TimeReferenceBuilder;
using ef::nav_msgs::MapMetaData;
using ef::nav_msgs::OdometryView;
using ef::nav_msgs::OdometryBuilder;
using ef::nav_msgs::GridCellsView;
using ef::nav_msgs::GridCellsBuilder;
using ef::nav_msgs::OccupancyGridView;
using ef::nav_msgs::OccupancyGridBuilder;
using ef::nav_msgs::PathView;
using ef::nav_msgs::PathBuilder;
using ef::foxglove_msgs::CompressedVideo;
using ef::foxglove_msgs::CompressedVideoView;
using ef::foxglove_msgs::CompressedVideoBuilder;
using ef::foxglove_msgs::TextAnnotationBuilder;
using ef::foxglove_msgs::PointAnnotationBuilder;
using ef::foxglove_msgs::ImageAnnotationBuilder;
using ef::mavros_msgs::AltitudeView;
using ef::mavros_msgs::AltitudeBuilder;
using ef::mavros_msgs::VfrHudView;
using ef::mavros_msgs::VfrHudBuilder;
using ef::mavros_msgs::EstimatorStatusView;
using ef::mavros_msgs::EstimatorStatusBuilder;
using ef::mavros_msgs::ExtendedStateView;
using ef::mavros_msgs::ExtendedStateBuilder;
using ef::mavros_msgs::SysStatusView;
using ef::mavros_msgs::SysStatusBuilder;
using ef::mavros_msgs::StateView;
using ef::mavros_msgs::StateBuilder;
using ef::mavros_msgs::StatusTextView;
using ef::mavros_msgs::StatusTextBuilder;
using ef::mavros_msgs::GpsRawView;
using ef::mavros_msgs::GpsRawBuilder;
using ef::mavros_msgs::TimesyncStatusView;
using ef::mavros_msgs::TimesyncStatusBuilder;
using ef::edgefirst_msgs::Mask;
using ef::edgefirst_msgs::MaskView;
using ef::edgefirst_msgs::MaskBuilder;
using ef::edgefirst_msgs::LocalTimeView;
using ef::edgefirst_msgs::LocalTimeBuilder;
using ef::edgefirst_msgs::TrackView;
using ef::edgefirst_msgs::TrackBuilder;
using ef::edgefirst_msgs::BoxView;
using ef::edgefirst_msgs::DetectView;
using ef::edgefirst_msgs::DetectBuilder;
using ef::edgefirst_msgs::DetectBoxBuilder;
using ef::edgefirst_msgs::ModelView;
using ef::edgefirst_msgs::ModelBuilder;
using ef::edgefirst_msgs::ModelInfoView;
using ef::edgefirst_msgs::ModelInfoBuilder;
using ef::edgefirst_msgs::RadarCubeView;
using ef::edgefirst_msgs::RadarCubeBuilder;
using ef::edgefirst_msgs::RadarInfoView;
using ef::edgefirst_msgs::RadarInfoBuilder;
using ef::edgefirst_msgs::VibrationView;
using ef::edgefirst_msgs::VibrationBuilder;
using ef::edgefirst_msgs::TensorView;
using ef::edgefirst_msgs::TensorBuilder;
using ef::edgefirst_msgs::TensorStampedView;
using ef::edgefirst_msgs::TensorStampedBuilder;
using ef::edgefirst_msgs::CameraFrameView;
using ef::edgefirst_msgs::CameraFrameBuilder;
using FoxgloveCompressedImage = ef::foxglove_msgs::CompressedImage;
using FoxgloveCompressedImageView = ef::foxglove_msgs::CompressedImageView;
using FoxgloveCompressedImageBuilder = ef::foxglove_msgs::CompressedImageBuilder;


// ============================================================================
// Compile-time: CdrFixed value types
// Must be trivially copyable and nothrow default constructible.
// ============================================================================

static_assert(std::is_trivially_copyable_v<Time>);
static_assert(std::is_nothrow_default_constructible_v<Time>);

static_assert(std::is_trivially_copyable_v<Duration>);
static_assert(std::is_nothrow_default_constructible_v<Duration>);

static_assert(std::is_trivially_copyable_v<Vector3>);
static_assert(std::is_nothrow_default_constructible_v<Vector3>);

static_assert(std::is_trivially_copyable_v<Point>);
static_assert(std::is_nothrow_default_constructible_v<Point>);

static_assert(std::is_trivially_copyable_v<Quaternion>);
static_assert(std::is_nothrow_default_constructible_v<Quaternion>);

static_assert(std::is_trivially_copyable_v<Pose>);
static_assert(std::is_nothrow_default_constructible_v<Pose>);

static_assert(std::is_trivially_copyable_v<Transform>);
static_assert(std::is_nothrow_default_constructible_v<Transform>);

static_assert(std::is_trivially_copyable_v<Twist>);
static_assert(std::is_nothrow_default_constructible_v<Twist>);

static_assert(std::is_trivially_copyable_v<Accel>);
static_assert(std::is_nothrow_default_constructible_v<Accel>);

static_assert(std::is_trivially_copyable_v<NavSatStatus>);
static_assert(std::is_nothrow_default_constructible_v<NavSatStatus>);

// ============================================================================
// Compile-time: View types — move-only
// ============================================================================

static_assert(!std::is_copy_constructible_v<HeaderView>);
static_assert(!std::is_copy_assignable_v<HeaderView>);
static_assert(std::is_nothrow_move_constructible_v<HeaderView>);
static_assert(std::is_nothrow_move_assignable_v<HeaderView>);

static_assert(!std::is_copy_constructible_v<CompressedImageView>);
static_assert(!std::is_copy_assignable_v<CompressedImageView>);
static_assert(std::is_nothrow_move_constructible_v<CompressedImageView>);
static_assert(std::is_nothrow_move_assignable_v<CompressedImageView>);

static_assert(!std::is_copy_constructible_v<ImuView>);
static_assert(!std::is_copy_assignable_v<ImuView>);
static_assert(std::is_nothrow_move_constructible_v<ImuView>);
static_assert(std::is_nothrow_move_assignable_v<ImuView>);

static_assert(!std::is_copy_constructible_v<NavSatFixView>);
static_assert(!std::is_copy_assignable_v<NavSatFixView>);
static_assert(std::is_nothrow_move_constructible_v<NavSatFixView>);
static_assert(std::is_nothrow_move_assignable_v<NavSatFixView>);

static_assert(!std::is_copy_constructible_v<CameraInfoView>);
static_assert(!std::is_copy_assignable_v<CameraInfoView>);
static_assert(std::is_nothrow_move_constructible_v<CameraInfoView>);
static_assert(std::is_nothrow_move_assignable_v<CameraInfoView>);

static_assert(!std::is_copy_constructible_v<TransformStampedView>);
static_assert(!std::is_copy_assignable_v<TransformStampedView>);
static_assert(std::is_nothrow_move_constructible_v<TransformStampedView>);
static_assert(std::is_nothrow_move_assignable_v<TransformStampedView>);

static_assert(!std::is_copy_constructible_v<CompressedVideoView>);
static_assert(!std::is_copy_assignable_v<CompressedVideoView>);
static_assert(std::is_nothrow_move_constructible_v<CompressedVideoView>);
static_assert(std::is_nothrow_move_assignable_v<CompressedVideoView>);

static_assert(!std::is_copy_constructible_v<MaskView>);
static_assert(!std::is_copy_assignable_v<MaskView>);
static_assert(std::is_nothrow_move_constructible_v<MaskView>);
static_assert(std::is_nothrow_move_assignable_v<MaskView>);

static_assert(!std::is_copy_constructible_v<LocalTimeView>);
static_assert(!std::is_copy_assignable_v<LocalTimeView>);
static_assert(std::is_nothrow_move_constructible_v<LocalTimeView>);
static_assert(std::is_nothrow_move_assignable_v<LocalTimeView>);

static_assert(!std::is_copy_constructible_v<TrackView>);
static_assert(!std::is_copy_assignable_v<TrackView>);
static_assert(std::is_nothrow_move_constructible_v<TrackView>);
static_assert(std::is_nothrow_move_assignable_v<TrackView>);

static_assert(!std::is_copy_constructible_v<ImageView>);
static_assert(!std::is_copy_assignable_v<ImageView>);
static_assert(std::is_nothrow_move_constructible_v<ImageView>);
static_assert(std::is_nothrow_move_assignable_v<ImageView>);

static_assert(!std::is_copy_constructible_v<PointCloud2View>);
static_assert(!std::is_copy_assignable_v<PointCloud2View>);
static_assert(std::is_nothrow_move_constructible_v<PointCloud2View>);
static_assert(std::is_nothrow_move_assignable_v<PointCloud2View>);

static_assert(!std::is_copy_constructible_v<RadarCubeView>);
static_assert(!std::is_copy_assignable_v<RadarCubeView>);
static_assert(std::is_nothrow_move_constructible_v<RadarCubeView>);
static_assert(std::is_nothrow_move_assignable_v<RadarCubeView>);

static_assert(!std::is_copy_constructible_v<RadarInfoView>);
static_assert(!std::is_copy_assignable_v<RadarInfoView>);
static_assert(std::is_nothrow_move_constructible_v<RadarInfoView>);
static_assert(std::is_nothrow_move_assignable_v<RadarInfoView>);

static_assert(!std::is_copy_constructible_v<BoxView>);
static_assert(!std::is_copy_assignable_v<BoxView>);
static_assert(std::is_nothrow_move_constructible_v<BoxView>);
static_assert(std::is_nothrow_move_assignable_v<BoxView>);

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

static_assert(!std::is_copy_constructible_v<DetectView>);
static_assert(!std::is_copy_assignable_v<DetectView>);
static_assert(std::is_nothrow_move_constructible_v<DetectView>);
static_assert(std::is_nothrow_move_assignable_v<DetectView>);

static_assert(!std::is_copy_constructible_v<ModelView>);
static_assert(!std::is_copy_assignable_v<ModelView>);
static_assert(std::is_nothrow_move_constructible_v<ModelView>);
static_assert(std::is_nothrow_move_assignable_v<ModelView>);

static_assert(!std::is_copy_constructible_v<ModelInfoView>);
static_assert(!std::is_copy_assignable_v<ModelInfoView>);
static_assert(std::is_nothrow_move_constructible_v<ModelInfoView>);
static_assert(std::is_nothrow_move_assignable_v<ModelInfoView>);

// ============================================================================
// Compile-time: Owning types — move-only
// ============================================================================

static_assert(!std::is_copy_constructible_v<Header>);
static_assert(!std::is_copy_assignable_v<Header>);
static_assert(std::is_nothrow_move_constructible_v<Header>);
static_assert(std::is_nothrow_move_assignable_v<Header>);

static_assert(!std::is_copy_constructible_v<CompressedImage>);
static_assert(!std::is_copy_assignable_v<CompressedImage>);
static_assert(std::is_nothrow_move_constructible_v<CompressedImage>);
static_assert(std::is_nothrow_move_assignable_v<CompressedImage>);

static_assert(!std::is_copy_constructible_v<CompressedVideo>);
static_assert(!std::is_copy_assignable_v<CompressedVideo>);
static_assert(std::is_nothrow_move_constructible_v<CompressedVideo>);
static_assert(std::is_nothrow_move_assignable_v<CompressedVideo>);

static_assert(!std::is_copy_constructible_v<Mask>);
static_assert(!std::is_copy_assignable_v<Mask>);
static_assert(std::is_nothrow_move_constructible_v<Mask>);
static_assert(std::is_nothrow_move_assignable_v<Mask>);

static_assert(!std::is_copy_constructible_v<Image>);
static_assert(!std::is_copy_assignable_v<Image>);
static_assert(std::is_nothrow_move_constructible_v<Image>);
static_assert(std::is_nothrow_move_assignable_v<Image>);

// ============================================================================
// Runtime: HeaderView move semantics
// ============================================================================

TEST_CASE("HeaderView move construct leaves source safely destructible", "[lifetime]") {
    auto hdr = Header::encode({1, 2}, "cam");
    REQUIRE(hdr.has_value());
    auto cdr = hdr->as_cdr();
    auto v1 = HeaderView::from_cdr(cdr);
    REQUIRE(v1.has_value());

    // Move construct: v2 owns the handle, v1 is in a moved-from (null handle) state.
    auto v2 = std::move(*v1);
    CHECK(v2.frame_id() == "cam");
    // v1's destructor must not double-free — verified by running under ASan.
}

TEST_CASE("HeaderView move assign", "[lifetime]") {
    auto hdr1 = Header::encode({1, 2}, "cam1");
    auto hdr2 = Header::encode({3, 4}, "cam2");
    REQUIRE(hdr1.has_value());
    REQUIRE(hdr2.has_value());
    auto v1 = HeaderView::from_cdr(hdr1->as_cdr());
    auto v2 = HeaderView::from_cdr(hdr2->as_cdr());
    REQUIRE(v1.has_value());
    REQUIRE(v2.has_value());

    // Move assign: v1 should now see cam2, v2 is safely destructible.
    *v1 = std::move(*v2);
    CHECK(v1->frame_id() == "cam2");
}

TEST_CASE("HeaderView self-move is safe", "[lifetime]") {
    auto hdr = Header::encode({1, 2}, "cam");
    REQUIRE(hdr.has_value());
    auto view = HeaderView::from_cdr(hdr->as_cdr());
    REQUIRE(view.has_value());

    // Use an indirection through a pointer to suppress compiler self-move warnings
    // while still exercising the self-move code path.
    HeaderView* p = &(*view);
    *p = std::move(*view);  // self-move via pointer alias
    CHECK(view->frame_id() == "cam");
}

// ============================================================================
// Runtime: Header (owning) move semantics
// ============================================================================

TEST_CASE("Header move construct leaves source safely destructible", "[lifetime]") {
    auto h1 = Header::encode({10, 20}, "lidar");
    REQUIRE(h1.has_value());

    auto h2 = std::move(*h1);
    CHECK(h2.frame_id() == "lidar");
    CHECK(h2.stamp().sec == 10);
    CHECK(h2.stamp().nanosec == 20);
    // h1 destructor must be safe — verified by ASan.
}

TEST_CASE("Header move assign", "[lifetime]") {
    auto h1 = Header::encode({1, 0}, "a");
    auto h2 = Header::encode({2, 0}, "b");
    REQUIRE(h1.has_value());
    REQUIRE(h2.has_value());

    *h1 = std::move(*h2);
    CHECK(h1->frame_id() == "b");
}

TEST_CASE("Header self-move is safe", "[lifetime]") {
    auto h = Header::encode({5, 6}, "self");
    REQUIRE(h.has_value());

    // Use pointer indirection to suppress compiler self-move warnings.
    Header* p = &(*h);
    *p = std::move(*h);
    CHECK(h->frame_id() == "self");
}

// ============================================================================
// Runtime: Image (owning) move semantics
// ============================================================================

TEST_CASE("Image move construct leaves source safely destructible", "[lifetime]") {
    std::vector<std::uint8_t> pixels(640 * 480 * 3, 42);
    auto img = Image::encode(
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
    auto img1 = Image::encode({1, 0}, "a", 10, 10, "mono8", false, 10,
                                   {px1.data(), px1.size()});
    auto img2 = Image::encode({2, 0}, "b", 20, 10, "mono8", false, 10,
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
    auto img = Image::encode({1, 0}, "cam", 10, 10, "mono8", false, 10,
                                  {pixels.data(), pixels.size()});
    REQUIRE(img.has_value());
    auto cdr = img->as_cdr();

    auto v1 = ImageView::from_cdr(cdr);
    REQUIRE(v1.has_value());

    auto v2 = std::move(*v1);
    CHECK(v2.frame_id() == "cam");
    CHECK(v2.width() == 10);
    // v1 destructor must be safe — verified by ASan.
}

TEST_CASE("ImageView move assign", "[lifetime]") {
    std::vector<std::uint8_t> px(100, 5);
    auto img1 = Image::encode({1, 0}, "c1", 10, 10, "mono8", false, 10,
                                   {px.data(), px.size()});
    auto img2 = Image::encode({2, 0}, "c2", 10, 10, "mono8", false, 10,
                                   {px.data(), px.size()});
    REQUIRE(img1.has_value());
    REQUIRE(img2.has_value());

    auto v1 = ImageView::from_cdr(img1->as_cdr());
    auto v2 = ImageView::from_cdr(img2->as_cdr());
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
    auto m1 = Mask::encode(8, 8, 64, "rle", {mdata.data(), mdata.size()}, false);
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
    auto bad = DetectView::from_cdr({});
    CHECK_FALSE(bad.has_value());
}

TEST_CASE("DetectView move semantics", "[lifetime][detect]") {
    auto v1 = DetectView::from_cdr(
        {kGoldenDetectBytesForMove, sizeof(kGoldenDetectBytesForMove)});
    REQUIRE(v1.has_value());
    const auto orig_frame = std::string(v1->frame_id());

    // Move construct
    auto v2 = std::move(*v1);
    CHECK(v2.frame_id() == orig_frame);
    CHECK(v2.boxes_len() == 3u);
    // v1 is moved-from; destructor must be safe (ASan would catch double-free).

    // Move assign from a fresh view
    auto v3 = DetectView::from_cdr(
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
    auto bad = ModelView::from_cdr({});
    CHECK_FALSE(bad.has_value());
}

TEST_CASE("ModelView move semantics", "[lifetime][model]") {
    auto v1 = ModelView::from_cdr(
        {kGoldenModelBytesForMove, sizeof(kGoldenModelBytesForMove)});
    REQUIRE(v1.has_value());
    const auto orig_frame = std::string(v1->frame_id());

    // Move construct
    auto v2 = std::move(*v1);
    CHECK(v2.frame_id() == orig_frame);
    CHECK(v2.masks_len() == 1u);
    // v1 is moved-from; destructor must be safe (ASan would catch double-free).

    // Move assign from a fresh view
    auto v3 = ModelView::from_cdr(
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
    Time t1{100, 200};
    Time t2 = t1;  // trivial copy
    CHECK(t2.sec == 100);
    CHECK(t2.nanosec == 200);

    t2.sec = 999;
    CHECK(t1.sec == 100);  // t1 unaffected by modification of t2
}
