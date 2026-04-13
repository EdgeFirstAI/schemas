#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

namespace ef = edgefirst::schemas;

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

TEST_CASE("Time roundtrip", "[cdr_fixed][time]") {
    ef::Time t{1234567890, 123456789};

    auto sz = t.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 12);

    std::uint8_t buf[64]{};
    auto written = t.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 12);

    auto decoded = ef::Time::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->sec == 1234567890);
    CHECK(decoded->nanosec == 123456789);
}

TEST_CASE("Time buffer too small", "[cdr_fixed][time]") {
    ef::Time t{1, 2};
    std::uint8_t buf[1]{};
    auto result = t.encode(ef::span<std::uint8_t>{buf, 1});
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ENOBUFS);
}

TEST_CASE("Time decode error", "[cdr_fixed][time]") {
    ef::span<const std::uint8_t> empty{};
    auto result = ef::Time::decode(empty);
    REQUIRE_FALSE(result.has_value());
}

// ---------------------------------------------------------------------------
// Duration
// ---------------------------------------------------------------------------

TEST_CASE("Duration roundtrip", "[cdr_fixed][duration]") {
    ef::Duration d{-500, 999999999};

    auto sz = d.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 12);

    std::uint8_t buf[64]{};
    auto written = d.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 12);

    auto decoded = ef::Duration::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->sec == -500);
    CHECK(decoded->nanosec == 999999999);
}

// ---------------------------------------------------------------------------
// Vector3
// ---------------------------------------------------------------------------

TEST_CASE("Vector3 roundtrip", "[cdr_fixed][vector3]") {
    ef::Vector3 v{1.0, 2.0, 3.0};

    auto sz = v.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 28);

    std::uint8_t buf[64]{};
    auto written = v.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 28);

    auto decoded = ef::Vector3::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->x == 1.0);
    CHECK(decoded->y == 2.0);
    CHECK(decoded->z == 3.0);
}

// ---------------------------------------------------------------------------
// Point
// ---------------------------------------------------------------------------

TEST_CASE("Point roundtrip", "[cdr_fixed][point]") {
    ef::Point p{10.5, -3.14, 0.0};

    auto sz = p.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 28);

    std::uint8_t buf[64]{};
    auto written = p.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 28);

    auto decoded = ef::Point::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->x == 10.5);
    CHECK(decoded->y == -3.14);
    CHECK(decoded->z == 0.0);
}

// ---------------------------------------------------------------------------
// Quaternion
// ---------------------------------------------------------------------------

TEST_CASE("Quaternion roundtrip", "[cdr_fixed][quaternion]") {
    ef::Quaternion q{0.0, 0.0, 0.0, 1.0};

    auto sz = q.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 36);

    std::uint8_t buf[64]{};
    auto written = q.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 36);

    auto decoded = ef::Quaternion::decode(ef::span<const std::uint8_t>{buf, *written});
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
    ef::Pose p{1.0, 2.0, 3.0, 0.0, 0.0, 0.707, 0.707};

    auto sz = p.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 60);

    std::uint8_t buf[128]{};
    auto written = p.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 60);

    auto decoded = ef::Pose::decode(ef::span<const std::uint8_t>{buf, *written});
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
    ef::Pose p{1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0};
    std::uint8_t buf[1]{};
    auto result = p.encode(ef::span<std::uint8_t>{buf, 1});
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ENOBUFS);
}

TEST_CASE("Pose decode error", "[cdr_fixed][pose]") {
    ef::span<const std::uint8_t> empty{};
    auto result = ef::Pose::decode(empty);
    REQUIRE_FALSE(result.has_value());
}

// ---------------------------------------------------------------------------
// Transform
// ---------------------------------------------------------------------------

TEST_CASE("Transform roundtrip", "[cdr_fixed][transform]") {
    ef::Transform tf{1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0};

    auto sz = tf.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 60);

    std::uint8_t buf[128]{};
    auto written = tf.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 60);

    auto decoded = ef::Transform::decode(ef::span<const std::uint8_t>{buf, *written});
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
    ef::Twist tw{1.5, -0.5, 0.0, 0.1, 0.2, 0.3};

    auto sz = tw.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 52);

    std::uint8_t buf[128]{};
    auto written = tw.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 52);

    auto decoded = ef::Twist::decode(ef::span<const std::uint8_t>{buf, *written});
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
    ef::Accel a{9.81, 0.0, 0.0, 0.0, 0.0, 0.0};

    auto sz = a.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 52);

    std::uint8_t buf[128]{};
    auto written = a.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 52);

    auto decoded = ef::Accel::decode(ef::span<const std::uint8_t>{buf, *written});
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
    ef::NavSatStatus n{2, 15};

    auto sz = n.encoded_size();
    REQUIRE(sz.has_value());
    REQUIRE(*sz == 8);

    std::uint8_t buf[64]{};
    auto written = n.encode(ef::span<std::uint8_t>{buf, sizeof(buf)});
    REQUIRE(written.has_value());
    REQUIRE(*written == 8);

    auto decoded = ef::NavSatStatus::decode(ef::span<const std::uint8_t>{buf, *written});
    REQUIRE(decoded.has_value());
    CHECK(decoded->status == 2);
    CHECK(decoded->service == 15);
}

TEST_CASE("NavSatStatus buffer too small", "[cdr_fixed][nav_sat_status]") {
    ef::NavSatStatus n{1, 1};
    std::uint8_t buf[1]{};
    auto result = n.encode(ef::span<std::uint8_t>{buf, 1});
    REQUIRE_FALSE(result.has_value());
    CHECK(result.error().code == ENOBUFS);
}

TEST_CASE("NavSatStatus decode error", "[cdr_fixed][nav_sat_status]") {
    ef::span<const std::uint8_t> empty{};
    auto result = ef::NavSatStatus::decode(empty);
    REQUIRE_FALSE(result.has_value());
}
