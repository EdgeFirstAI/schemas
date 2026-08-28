/**
 * @file test_geometry_builders.cpp
 * @brief Catch2 tests for geometry_msgs / Odometry C++ builders.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.
 */

#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

#include <array>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

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


static constexpr Time kStamp{1234567890, 123456789};
static constexpr const char* kFrame = "test_frame";

static std::vector<std::uint8_t> load_fixture(const std::string& relpath) {
    std::FILE* f = std::fopen(relpath.c_str(), "rb");
    REQUIRE(f != nullptr);
    REQUIRE(std::fseek(f, 0, SEEK_END) == 0);
    long sz = std::ftell(f);
    REQUIRE(sz >= 0);
    REQUIRE(std::fseek(f, 0, SEEK_SET) == 0);
    std::vector<std::uint8_t> buf(static_cast<std::size_t>(sz));
    std::size_t got = std::fread(buf.data(), 1, buf.size(), f);
    std::fclose(f);
    REQUIRE(got == buf.size());
    return buf;
}

template <typename BuildResult>
static std::vector<std::uint8_t> take_bytes(BuildResult&& r) {
    REQUIRE(r.has_value());
    std::vector<std::uint8_t> out(r->data, r->data + r->size);
    edgefirst_schemas_bytes_free(r->data, r->size);
    return out;
}

TEST_CASE("AccelStampedBuilder bytes match golden", "[geometry][builder]") {
    auto golden = load_fixture("testdata/cdr/geometry_msgs/AccelStamped.cdr");
    auto b = AccelStampedBuilder::create();
    REQUIRE(b.has_value());
    b->stamp(kStamp);
    REQUIRE(b->frame_id(kFrame).has_value());
    b->linear_acceleration(1.0, 2.0, 3.0);
    b->angular_acceleration(0.1, 0.2, 0.3);
    auto bytes = take_bytes(b->build());
    REQUIRE(bytes == golden);
}

TEST_CASE("TransformStampedBuilder bytes match golden", "[geometry][builder]") {
    auto golden = load_fixture("testdata/cdr/geometry_msgs/TransformStamped.cdr");
    auto b = TransformStampedBuilder::create();
    REQUIRE(b.has_value());
    b->stamp(kStamp);
    REQUIRE(b->frame_id(kFrame).has_value());
    REQUIRE(b->child_frame_id("child_frame").has_value());
    b->translation(1.5, -2.5, 3.0);
    b->rotation(0.0, 0.0, 0.0, 1.0);
    auto bytes = take_bytes(b->build());
    REQUIRE(bytes == golden);
}

TEST_CASE("PoseArrayBuilder bytes match golden", "[geometry][builder]") {
    auto golden = load_fixture("testdata/cdr/geometry_msgs/PoseArray.cdr");
    Pose poses[2] = {
        Pose{1.5, -2.5, 3.0, 0.0, 0.0, 0.0, 1.0},
        Pose{10.0, 20.0, 30.0, 0.0, 0.0, 0.0, 1.0},
    };
    auto b = PoseArrayBuilder::create();
    REQUIRE(b.has_value());
    b->stamp(kStamp);
    REQUIRE(b->frame_id(kFrame).has_value());
    REQUIRE(b->poses(ef::span<const Pose>{poses, 2}).has_value());
    auto bytes = take_bytes(b->build());
    REQUIRE(bytes == golden);
}

TEST_CASE("OdometryBuilder bytes match golden", "[nav][builder]") {
    auto golden = load_fixture("testdata/cdr/nav_msgs/Odometry.cdr");
    std::array<double, 36> pose_cov{};
    std::array<double, 36> twist_cov{};
    for (int i = 0; i < 6; i++) {
        pose_cov[static_cast<std::size_t>(i * 6 + i)] = 0.1 * (i + 1);
        twist_cov[static_cast<std::size_t>(i * 6 + i)] = 0.02 * (i + 1);
    }
    pose_cov[1] = 0.01;
    pose_cov[6] = 0.01;
    twist_cov[7] = 0.001;

    auto b = OdometryBuilder::create();
    REQUIRE(b.has_value());
    b->stamp(kStamp);
    REQUIRE(b->frame_id(kFrame).has_value());
    REQUIRE(b->child_frame_id("base_link").has_value());
    b->pose(Pose{1.5, -2.5, 3.0, 0.0, 0.0, 0.0, 1.0});
    REQUIRE(b->pose_covariance(ef::span<const double>{pose_cov.data(), pose_cov.size()}).has_value());
    b->twist(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    REQUIRE(b->twist_covariance(ef::span<const double>{twist_cov.data(), twist_cov.size()}).has_value());
    auto bytes = take_bytes(b->build());
    REQUIRE(bytes == golden);
}
