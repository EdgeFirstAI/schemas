/**
 * @file test_mavros_views.cpp
 * @brief Catch2 tests for mavros_msgs C++ View classes
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.
 */

#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

#include <cstdint>
#include <cstdio>
#include <string>
#include <string_view>
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


// ============================================================================
// AltitudeView
// ============================================================================

TEST_CASE("AltitudeView from_cdr empty returns error", "[mavros][altitude]") {
    auto result = AltitudeView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("AltitudeView from_cdr invalid returns error", "[mavros][altitude]") {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    auto result = AltitudeView::from_cdr({bad, sizeof(bad)});
    REQUIRE_FALSE(result.has_value());
}

// ============================================================================
// VfrHudView
// ============================================================================

TEST_CASE("VfrHudView from_cdr empty returns error", "[mavros][vfrhud]") {
    auto result = VfrHudView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("VfrHudView from_cdr invalid returns error", "[mavros][vfrhud]") {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    auto result = VfrHudView::from_cdr({bad, sizeof(bad)});
    REQUIRE_FALSE(result.has_value());
}

// ============================================================================
// EstimatorStatusView
// ============================================================================

TEST_CASE("EstimatorStatusView from_cdr empty returns error", "[mavros][estimator]") {
    auto result = EstimatorStatusView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("EstimatorStatusView from_cdr invalid returns error", "[mavros][estimator]") {
    uint8_t bad[] = {0x01, 0x02};
    auto result = EstimatorStatusView::from_cdr({bad, sizeof(bad)});
    REQUIRE_FALSE(result.has_value());
}

// ============================================================================
// ExtendedStateView
// ============================================================================

TEST_CASE("ExtendedStateView from_cdr empty returns error", "[mavros][extstate]") {
    auto result = ExtendedStateView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("ExtendedStateView constants", "[mavros][extstate]") {
    CHECK(ExtendedStateView::VTOL_STATE_UNDEFINED == 0);
    CHECK(ExtendedStateView::VTOL_STATE_FW == 4);
    CHECK(ExtendedStateView::LANDED_STATE_IN_AIR == 2);
}

// ============================================================================
// SysStatusView
// ============================================================================

TEST_CASE("SysStatusView from_cdr empty returns error", "[mavros][sysstatus]") {
    auto result = SysStatusView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("SysStatusView from_cdr invalid returns error", "[mavros][sysstatus]") {
    uint8_t bad[] = {0xFF, 0xFF, 0xFF, 0xFF};
    auto result = SysStatusView::from_cdr({bad, sizeof(bad)});
    REQUIRE_FALSE(result.has_value());
}

// ============================================================================
// StateView
// ============================================================================

TEST_CASE("StateView from_cdr empty returns error", "[mavros][state]") {
    auto result = StateView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("StateView constants", "[mavros][state]") {
    CHECK(StateView::MAV_STATE_UNINIT == 0);
    CHECK(StateView::MAV_STATE_ACTIVE == 4);
    CHECK(StateView::MAV_STATE_FLIGHT_TERMINATION == 8);
}

// ============================================================================
// StatusTextView
// ============================================================================

TEST_CASE("StatusTextView from_cdr empty returns error", "[mavros][statustext]") {
    auto result = StatusTextView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("StatusTextView severity constants", "[mavros][statustext]") {
    CHECK(StatusTextView::SEVERITY_EMERGENCY == 0);
    CHECK(StatusTextView::SEVERITY_DEBUG == 7);
}

// ============================================================================
// GpsRawView
// ============================================================================

TEST_CASE("GpsRawView from_cdr empty returns error", "[mavros][gpsraw]") {
    auto result = GpsRawView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("GpsRawView fix type constants", "[mavros][gpsraw]") {
    CHECK(GpsRawView::GPS_FIX_TYPE_NO_GPS == 0);
    CHECK(GpsRawView::GPS_FIX_TYPE_3D_FIX == 3);
    CHECK(GpsRawView::GPS_FIX_TYPE_PPP == 8);
}

// ============================================================================
// TimesyncStatusView
// ============================================================================

TEST_CASE("TimesyncStatusView from_cdr empty returns error", "[mavros][timesync]") {
    auto result = TimesyncStatusView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("TimesyncStatusView from_cdr invalid returns error", "[mavros][timesync]") {
    uint8_t bad[] = {0xBA, 0xDC, 0x0D, 0xE0};
    auto result = TimesyncStatusView::from_cdr({bad, sizeof(bad)});
    REQUIRE_FALSE(result.has_value());
}

static std::vector<std::uint8_t> load_fixture_mav(const std::string& relpath) {
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

TEST_CASE("AltitudeBuilder bytes match golden", "[mavros][builder]") {
    auto golden = load_fixture_mav("testdata/cdr/mavros_msgs/Altitude.cdr");
    auto b = AltitudeBuilder::create();
    REQUIRE(b.has_value());
    b->stamp(Time{1234567890, 123456789});
    REQUIRE(b->frame_id("test_frame").has_value());
    b->monotonic(100.0f).amsl(50.0f).local(10.0f)
        .relative(5.0f).terrain(2.0f).bottom_clearance(1.5f);
    auto r = b->build();
    REQUIRE(r.has_value());
    std::vector<std::uint8_t> out(r->data, r->data + r->size);
    edgefirst_schemas_bytes_free(r->data, r->size);
    REQUIRE(out == golden);
}
