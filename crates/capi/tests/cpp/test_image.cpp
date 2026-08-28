/**
 * @file test_image.cpp
 * @brief Zero-copy pointer-identity tests for Image / ImageView
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.
 */

#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

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


TEST_CASE("Image encode+view roundtrip", "[buffer_backed][image]") {
    std::vector<std::uint8_t> pixels(640 * 480 * 3, 42);
    auto img = Image::encode(
        {1000, 500}, "cam",
        480, 640, "rgb8", false,
        640 * 3, {pixels.data(), pixels.size()});
    REQUIRE(img.has_value());

    CHECK(img->stamp().sec == 1000);
    CHECK(img->stamp().nanosec == 500);
    CHECK(img->frame_id() == "cam");
    CHECK(img->height() == 480);
    CHECK(img->width() == 640);
    CHECK(img->encoding() == "rgb8");
    CHECK(img->is_bigendian() == 0);
    CHECK(img->step() == 640 * 3);
    CHECK(img->data().size() == pixels.size());
    CHECK(!img->as_cdr().empty());
}

TEST_CASE("ImageView data is zero-copy", "[buffer_backed][image]") {
    std::vector<std::uint8_t> pixels(640 * 480 * 3, 42);
    auto img = Image::encode(
        {1000, 500}, "cam",
        480, 640, "rgb8", false,
        640 * 3, {pixels.data(), pixels.size()});
    REQUIRE(img.has_value());

    // Create a view over the encoded CDR
    auto cdr = img->as_cdr();
    auto view = ImageView::from_cdr(cdr);
    REQUIRE(view.has_value());

    // Compute the byte window of the CDR buffer as integers. Using uintptr_t
    // for pointer-identity checks avoids two problems: (a) relational
    // comparison of unrelated pointers is UB, so if a regression caused px
    // or enc to land outside the buffer the comparison itself would be UB;
    // (b) Catch2's StringMaker<char const*> specialisation would call
    // strlen on raw char pointers during JUnit reporter formatting, reading
    // past the end of the non-NUL-terminated CDR byte array under ASan.
    const auto buf_start = reinterpret_cast<std::uintptr_t>(cdr.data());
    const auto buf_end   = buf_start + cdr.size();

    // Zero-copy assertion: data() must point INTO the cdr buffer.
    auto px = view->data();
    const auto px_start = reinterpret_cast<std::uintptr_t>(px.data());
    const auto px_end   = px_start + px.size();
    CHECK(px_start >= buf_start);
    CHECK(px_end   <= buf_end);

    // String accessor also borrows from the CDR buffer — check BOTH bounds.
    auto enc = view->encoding();
    const auto enc_start = reinterpret_cast<std::uintptr_t>(enc.data());
    const auto enc_end   = enc_start + enc.size();
    CHECK(enc_start >= buf_start);
    CHECK(enc_end   <= buf_end);
    CHECK(enc == "rgb8");
}

TEST_CASE("ImageView from_cdr error on empty span", "[buffer_backed][image]") {
    auto v = ImageView::from_cdr({});
    REQUIRE_FALSE(v.has_value());
}

TEST_CASE("ImageView accessors match encoded values", "[buffer_backed][image]") {
    std::vector<std::uint8_t> pixels(320 * 240 * 1, 7);
    auto img = Image::encode(
        {9, 8}, "lidar_cam",
        240, 320, "mono8", false,
        320, {pixels.data(), pixels.size()});
    REQUIRE(img.has_value());

    auto cdr = img->as_cdr();
    auto view = ImageView::from_cdr(cdr);
    REQUIRE(view.has_value());

    CHECK(view->stamp().sec == 9);
    CHECK(view->stamp().nanosec == 8);
    CHECK(view->frame_id() == "lidar_cam");
    CHECK(view->height() == 240);
    CHECK(view->width() == 320);
    CHECK(view->encoding() == "mono8");
    CHECK(view->is_bigendian() == 0);
    CHECK(view->step() == 320);
    CHECK(view->data().size() == pixels.size());
}

TEST_CASE("Image move semantics", "[buffer_backed][image]") {
    std::vector<std::uint8_t> pixels(10, 1);
    auto img = Image::encode(
        {1, 2}, "f", 1, 10, "mono8", false, 10,
        {pixels.data(), pixels.size()});
    REQUIRE(img.has_value());
    auto img2 = std::move(*img);
    CHECK(img2.width() == 10);
    CHECK(!img2.as_cdr().empty());
}
