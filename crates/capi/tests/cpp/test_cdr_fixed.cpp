#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

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


// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

TEST_CASE("Time roundtrip", "[cdr_fixed][time]") {
    Time t{1234567890, 123456789};

    auto sz = t.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 12);

    std::uint8_t buf[64]{};
    auto written = t.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 12);

    auto decoded = Time::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->sec == 1234567890);
    CHECK(decoded->nanosec == 123456789);
}

TEST_CASE("Time buffer too small", "[cdr_fixed][time]") {
    Time t{1, 2};
    std::uint8_t buf[1]{};
    auto result = t.encode(ef::span<std::uint8_t>{buf, 1});
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ENOBUFS);
}

TEST_CASE("Time decode error", "[cdr_fixed][time]") {
    ef::span<const std::uint8_t> empty{};
    auto result = Time::decode(empty);
    REQUIRE_FALSE(result.has_value());
}

// ---------------------------------------------------------------------------
// Duration
// ---------------------------------------------------------------------------

TEST_CASE("Duration roundtrip", "[cdr_fixed][duration]") {
    Duration d{-500, 999999999};

    auto sz = d.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 12);

    std::uint8_t buf[64]{};
    auto written = d.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 12);

    auto decoded = Duration::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->sec == -500);
    CHECK(decoded->nanosec == 999999999);
}

// ---------------------------------------------------------------------------
// Vector3
// ---------------------------------------------------------------------------

TEST_CASE("Vector3 roundtrip", "[cdr_fixed][vector3]") {
    Vector3 v{1.0, 2.0, 3.0};

    auto sz = v.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 28);

    std::uint8_t buf[64]{};
    auto written = v.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 28);

    auto decoded = Vector3::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->x == 1.0);
    CHECK(decoded->y == 2.0);
    CHECK(decoded->z == 3.0);
}

// ---------------------------------------------------------------------------
// Point
// ---------------------------------------------------------------------------

TEST_CASE("Point roundtrip", "[cdr_fixed][point]") {
    Point p{10.5, -3.14, 0.0};

    auto sz = p.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 28);

    std::uint8_t buf[64]{};
    auto written = p.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 28);

    auto decoded = Point::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->x == 10.5);
    CHECK(decoded->y == -3.14);
    CHECK(decoded->z == 0.0);
}

// ---------------------------------------------------------------------------
// Quaternion
// ---------------------------------------------------------------------------

TEST_CASE("Quaternion roundtrip", "[cdr_fixed][quaternion]") {
    Quaternion q{0.0, 0.0, 0.0, 1.0};

    auto sz = q.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 36);

    std::uint8_t buf[64]{};
    auto written = q.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 36);

    auto decoded = Quaternion::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->x == 0.0);
    CHECK(decoded->y == 0.0);
    CHECK(decoded->z == 0.0);
    CHECK(decoded->w == 1.0);
}

// ---------------------------------------------------------------------------
// Pose
// ---------------------------------------------------------------------------

TEST_CASE("Pose roundtrip", "[cdr_fixed][pose]") {
    Pose p{1.0, 2.0, 3.0, 0.0, 0.0, 0.707, 0.707};

    auto sz = p.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 60);

    std::uint8_t buf[128]{};
    auto written = p.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 60);

    auto decoded = Pose::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->px == 1.0);
    CHECK(decoded->py == 2.0);
    CHECK(decoded->pz == 3.0);
    CHECK(decoded->ox == 0.0);
    CHECK(decoded->oy == 0.0);
    CHECK(decoded->oz == 0.707);
    CHECK(decoded->ow == 0.707);
}

TEST_CASE("Pose buffer too small", "[cdr_fixed][pose]") {
    Pose p{1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0};
    std::uint8_t buf[1]{};
    auto result = p.encode(ef::span<std::uint8_t>{buf, 1});
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ENOBUFS);
}

TEST_CASE("Pose decode error", "[cdr_fixed][pose]") {
    ef::span<const std::uint8_t> empty{};
    auto result = Pose::decode(empty);
    REQUIRE_FALSE(result.has_value());
}

// ---------------------------------------------------------------------------
// Transform
// ---------------------------------------------------------------------------

TEST_CASE("Transform roundtrip", "[cdr_fixed][transform]") {
    Transform tf{1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0};

    auto sz = tf.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 60);

    std::uint8_t buf[128]{};
    auto written = tf.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 60);

    auto decoded = Transform::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->tx == 1.0);
    CHECK(decoded->ty == 2.0);
    CHECK(decoded->tz == 3.0);
    CHECK(decoded->rx == 0.0);
    CHECK(decoded->ry == 0.0);
    CHECK(decoded->rz == 0.0);
    CHECK(decoded->rw == 1.0);
}

// ---------------------------------------------------------------------------
// Twist
// ---------------------------------------------------------------------------

TEST_CASE("Twist roundtrip", "[cdr_fixed][twist]") {
    Twist tw{1.5, -0.5, 0.0, 0.1, 0.2, 0.3};

    auto sz = tw.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 52);

    std::uint8_t buf[128]{};
    auto written = tw.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 52);

    auto decoded = Twist::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->lx == 1.5);
    CHECK(decoded->ly == -0.5);
    CHECK(decoded->lz == 0.0);
    CHECK(decoded->ax == 0.1);
    CHECK(decoded->ay == 0.2);
    CHECK(decoded->az == 0.3);
}

// ---------------------------------------------------------------------------
// Accel
// ---------------------------------------------------------------------------

TEST_CASE("Accel roundtrip", "[cdr_fixed][accel]") {
    Accel a{9.81, 0.0, 0.0, 0.0, 0.0, 0.0};

    auto sz = a.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 52);

    std::uint8_t buf[128]{};
    auto written = a.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 52);

    auto decoded = Accel::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->lx == 9.81);
    CHECK(decoded->ly == 0.0);
    CHECK(decoded->lz == 0.0);
    CHECK(decoded->ax == 0.0);
    CHECK(decoded->ay == 0.0);
    CHECK(decoded->az == 0.0);
}

// ---------------------------------------------------------------------------
// NavSatStatus
// ---------------------------------------------------------------------------

TEST_CASE("NavSatStatus roundtrip", "[cdr_fixed][nav_sat_status]") {
    NavSatStatus n{2, 15};

    auto sz = n.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 8);

    std::uint8_t buf[64]{};
    auto written = n.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 8);

    auto decoded = NavSatStatus::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->status == 2);
    CHECK(decoded->service == 15);
}

TEST_CASE("NavSatStatus buffer too small", "[cdr_fixed][nav_sat_status]") {
    NavSatStatus n{1, 1};
    std::uint8_t buf[1]{};
    auto result = n.encode(ef::span<std::uint8_t>{buf, 1});
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ENOBUFS);
}

TEST_CASE("NavSatStatus decode error", "[cdr_fixed][nav_sat_status]") {
    ef::span<const std::uint8_t> empty{};
    auto result = NavSatStatus::decode(empty);
    REQUIRE_FALSE(result.has_value());
}
