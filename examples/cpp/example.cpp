/**
 * @file example.cpp
 * @brief Example usage of EdgeFirst Schemas C++ API
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.
 *
 * Demonstrates:
 * - CdrFixed encode/decode (Time, Vector3) into stack buffers
 * - Buffer-backed encode (Header, Image) returning owning types
 * - Buffer-backed decode via View types (move-only, RAII)
 * - expected<T, Error> for fallible operations (no exceptions)
 * - Zero-copy field access via std::string_view and span<const uint8_t>
 */

#include <edgefirst/schemas.hpp>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>
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


static int example_time() {
    std::cout << "\n=== Example: Time (CdrFixed) ===\n";

    Time t{1234567890, 123456789};

    std::array<std::uint8_t, 64> buf{};
    auto written = t.encode(ef::span<std::uint8_t>{buf.data(), buf.size()});
    if (!written) {
        std::cerr << "builtin_interfaces_time_encode failed: " << written.error().category() << "\n";
        return -1;
    }
    std::cout << "Encoded Time: " << *written << " CDR bytes\n";

    auto decoded = Time::decode(ef::span<const std::uint8_t>{buf.data(), *written});
    if (!decoded) {
        std::cerr << "builtin_interfaces_time_decode failed: " << decoded.error().category() << "\n";
        return -1;
    }
    std::cout << "Decoded: " << decoded->sec << "." << decoded->nanosec << "\n";

    if (decoded->sec != 1234567890 || decoded->nanosec != 123456789) {
        std::cerr << "Time roundtrip mismatch\n";
        return -1;
    }

    std::cout << "Time example completed successfully!\n";
    return 0;
}

static int example_vector3() {
    std::cout << "\n=== Example: Vector3 (CdrFixed) ===\n";

    Vector3 v{1.5, 2.5, 3.5};

    std::array<std::uint8_t, 64> buf{};
    auto written = v.encode(ef::span<std::uint8_t>{buf.data(), buf.size()});
    if (!written) {
        std::cerr << "geometry_msgs_vector3_encode failed: " << written.error().category() << "\n";
        return -1;
    }
    std::cout << "Encoded Vector3: " << *written << " CDR bytes\n";

    auto decoded = Vector3::decode(ef::span<const std::uint8_t>{buf.data(), *written});
    if (!decoded) {
        std::cerr << "geometry_msgs_vector3_decode failed: " << decoded.error().category() << "\n";
        return -1;
    }
    std::cout << "Decoded: (" << decoded->x << ", " << decoded->y << ", " << decoded->z << ")\n";

    if (decoded->x != 1.5 || decoded->y != 2.5 || decoded->z != 3.5) {
        std::cerr << "Vector3 roundtrip mismatch\n";
        return -1;
    }

    std::cout << "Vector3 example completed successfully!\n";
    return 0;
}

static int example_header() {
    std::cout << "\n=== Example: Header (buffer-backed) ===\n";

    // Encode a Header — returns owning Header
    auto encoded = Header::encode(
        Time{1234567890, 123456789},
        "camera_frame"
    );
    if (!encoded) {
        std::cerr << "Header::encode failed: " << encoded.error().category() << "\n";
        return -1;
    }
    std::cout << "Encoded Header: " << encoded->as_cdr().size() << " CDR bytes\n";

    // Access fields from owning Header
    auto stamp = encoded->stamp();
    auto frame_id = encoded->frame_id();

    std::cout << "Decoded: stamp=" << stamp.sec << "." << stamp.nanosec
              << " frame_id=\"" << frame_id << "\"\n";

    bool ok = (stamp.sec == 1234567890 && stamp.nanosec == 123456789
               && frame_id == "camera_frame");

    if (!ok) {
        std::cerr << "Header roundtrip mismatch\n";
        return -1;
    }

    std::cout << "Header example completed successfully!\n";
    return 0;
}

static int example_image() {
    std::cout << "\n=== Example: Image (buffer-backed) ===\n";

    // Create dummy pixel data
    std::size_t data_size = 640 * 480 * 3;  // VGA RGB8
    std::vector<std::uint8_t> pixel_data(data_size);
    for (std::size_t i = 0; i < data_size; i++) {
        pixel_data[i] = static_cast<std::uint8_t>(i % 256);
    }

    // Encode an Image — all fields in one call
    auto encoded = Image::encode(
        Time{1000, 500000},
        "camera",
        480, 640,                // height, width
        "rgb8",
        false,                   // is_bigendian
        640 * 3,                 // step
        ef::span<const std::uint8_t>{pixel_data.data(), pixel_data.size()}
    );
    if (!encoded) {
        std::cerr << "Image::encode failed: " << encoded.error().category() << "\n";
        return -1;
    }
    std::cout << "Encoded Image: " << encoded->as_cdr().size() << " CDR bytes\n";

    // Access fields from owning Image
    auto width = encoded->width();
    auto height = encoded->height();
    auto encoding = encoded->encoding();
    auto data = encoded->data();

    std::cout << "Decoded: " << width << "x" << height
              << " encoding=\"" << encoding << "\" data=" << data.size() << " bytes\n";

    bool ok = (width == 640 && height == 480
               && encoding == "rgb8"
               && data.size() == data_size);

    if (!ok) {
        std::cerr << "Image roundtrip mismatch\n";
        return -1;
    }

    // Forward the raw CDR bytes (zero re-serialization cost)
    auto cdr = encoded->as_cdr();
    std::cout << "CDR bytes available for forwarding: " << cdr.size() << " bytes\n";

    std::cout << "Image example completed successfully!\n";
    return 0;
}

int main() {
    std::cout << "EdgeFirst Schemas C++ API Examples\n";
    std::cout << "===================================\n";

    int failures = 0;

    if (example_time() != 0)       failures++;
    if (example_vector3() != 0)    failures++;
    if (example_header() != 0)     failures++;
    if (example_image() != 0)      failures++;

    std::cout << "\n===================================\n";
    if (failures > 0) {
        std::cerr << failures << " example(s) FAILED\n";
        return 1;
    }

    std::cout << "All C++ examples completed successfully!\n";
    return 0;
}
