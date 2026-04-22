/**
 * @file test_top2_770_schemas.cpp
 * @brief C++ tests for TOP2-770 schema additions:
 *        PoseWithCovariance, TwistWithCovariance, MagneticField,
 *        FluidPressure, Temperature, BatteryState, Odometry, Vibration.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.
 */

#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace ef = edgefirst::schemas;

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

// ── CdrFixed ───────────────────────────────────────────────────────────

TEST_CASE("PoseWithCovariance encode/decode roundtrip", "[top2_770]") {
    ef::PoseWithCovariance p(ef::Pose{1.5, -2.5, 3.0, 0.0, 0.0, 0.0, 1.0}, {});
    for (int i = 0; i < 6; ++i) {
        p.covariance[i * 6 + i] = 0.1 * (i + 1);
    }
    p.covariance[1] = 0.01;
    p.covariance[6] = 0.01;
    auto sz = p.encoded_size();
    REQUIRE(sz.has_value());
    std::vector<std::uint8_t> buf(*sz);
    auto w = p.encode(ef::span<std::uint8_t>{buf.data(), buf.size()});
    REQUIRE(w.has_value());
    auto dec = ef::PoseWithCovariance::decode(
        ef::span<const std::uint8_t>{buf.data(), *w});
    REQUIRE(dec.has_value());
    CHECK(dec->pose.px == 1.5);
    CHECK(dec->pose.ow == 1.0);
    CHECK(dec->covariance[0] == 0.1);
    CHECK(dec->covariance[35] == Approx(0.6));
    CHECK(dec->covariance[1] == 0.01);
}

TEST_CASE("TwistWithCovariance encode/decode roundtrip", "[top2_770]") {
    ef::TwistWithCovariance t(ef::Twist{1.0, 2.0, 3.0, 0.1, 0.2, 0.3}, {});
    for (int i = 0; i < 6; ++i) {
        t.covariance[i * 6 + i] = 0.02 * (i + 1);
    }
    t.covariance[7] = 0.001;
    auto sz = t.encoded_size();
    REQUIRE(sz.has_value());
    std::vector<std::uint8_t> buf(*sz);
    auto w = t.encode(ef::span<std::uint8_t>{buf.data(), buf.size()});
    REQUIRE(w.has_value());
    auto dec = ef::TwistWithCovariance::decode(
        ef::span<const std::uint8_t>{buf.data(), *w});
    REQUIRE(dec.has_value());
    CHECK(dec->twist.lx == 1.0);
    CHECK(dec->twist.az == 0.3);
    CHECK(dec->covariance[0] == 0.02);
    CHECK(dec->covariance[7] == 0.001);
}

// ── Buffer-backed (view-only) ─────────────────────────────────────────

TEST_CASE("MagneticFieldView decodes golden fixture", "[top2_770]") {
    auto bytes = load_fixture("testdata/cdr/sensor_msgs/MagneticField.cdr");
    auto view = ef::MagneticFieldView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(view.has_value());
    CHECK(view->frame_id() == "test_frame");
    auto mf = view->magnetic_field();
    CHECK(mf.x == Approx(2.5e-5));
    CHECK(mf.y == Approx(-1.2e-5));
    CHECK(mf.z == Approx(4.1e-5));
    auto cov = view->magnetic_field_covariance();
    CHECK(cov[0] == Approx(1e-10));
    CHECK(cov[4] == Approx(1e-10));
}

TEST_CASE("FluidPressureView decodes golden fixture", "[top2_770]") {
    auto bytes = load_fixture("testdata/cdr/sensor_msgs/FluidPressure.cdr");
    auto view = ef::FluidPressureView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(view.has_value());
    CHECK(view->frame_id() == "test_frame");
    CHECK(view->fluid_pressure() == Approx(101325.0));
    CHECK(view->variance() == Approx(25.0));
}

TEST_CASE("TemperatureView decodes golden fixture", "[top2_770]") {
    auto bytes = load_fixture("testdata/cdr/sensor_msgs/Temperature.cdr");
    auto view = ef::TemperatureView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(view.has_value());
    CHECK(view->frame_id() == "test_frame");
    CHECK(view->temperature() == Approx(22.5));
    CHECK(view->variance() == Approx(0.01));
}

TEST_CASE("BatteryStateView decodes golden fixture", "[top2_770]") {
    auto bytes = load_fixture("testdata/cdr/sensor_msgs/BatteryState.cdr");
    auto view = ef::BatteryStateView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(view.has_value());
    CHECK(view->frame_id() == "test_frame");
    CHECK(view->voltage() == Approx(12.34f));
    CHECK(view->percentage() == Approx(0.84f));
    CHECK(view->power_supply_status()
          == ef::BatteryStateView::POWER_SUPPLY_STATUS_DISCHARGING);
    CHECK(view->power_supply_technology()
          == ef::BatteryStateView::POWER_SUPPLY_TECHNOLOGY_LIPO);
    CHECK(view->present());
    CHECK(view->cell_voltage_len() == 3u);
    std::array<float, 4> cells{};
    auto n = view->cell_voltage(
        ef::span<float>{cells.data(), cells.size()});
    CHECK(n == 3u);
    CHECK(cells[0] == Approx(4.11f));
    CHECK(cells[2] == Approx(4.10f));
    CHECK(view->location() == "battery0");
    CHECK(view->serial_number() == "SN0123456");
}

TEST_CASE("OdometryView decodes golden fixture", "[top2_770]") {
    auto bytes = load_fixture("testdata/cdr/nav_msgs/Odometry.cdr");
    auto view = ef::OdometryView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(view.has_value());
    CHECK(view->frame_id() == "test_frame");
    CHECK(view->child_frame_id() == "base_link");
    auto p = view->pose();
    CHECK(p.pose.px == Approx(1.5));
    CHECK(p.pose.ow == Approx(1.0));
    CHECK(p.covariance[0] == Approx(0.1));
    auto t = view->twist();
    CHECK(t.twist.lx == Approx(1.0));
    CHECK(t.twist.az == Approx(0.3));
    CHECK(t.covariance[7] == Approx(0.001));
}

TEST_CASE("VibrationView decodes golden fixture", "[top2_770]") {
    auto bytes = load_fixture("testdata/cdr/edgefirst_msgs/Vibration.cdr");
    auto view = ef::VibrationView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(view.has_value());
    CHECK(view->frame_id() == "test_frame");
    CHECK(view->measurement_type() == ef::VibrationView::MEASUREMENT_RMS);
    CHECK(view->unit() == ef::VibrationView::UNIT_ACCEL_M_PER_S2);
    CHECK(view->band_lower_hz() == Approx(10.0f));
    CHECK(view->band_upper_hz() == Approx(1000.0f));
    auto v = view->vibration();
    CHECK(v.x == Approx(0.42));
    CHECK(v.y == Approx(0.51));
    CHECK(v.z == Approx(0.37));
    CHECK(view->clipping_len() == 3u);
    std::array<std::uint32_t, 4> cl{};
    auto n = view->clipping(
        ef::span<std::uint32_t>{cl.data(), cl.size()});
    CHECK(n == 3u);
    CHECK(cl[0] == 3u);
    CHECK(cl[1] == 1u);
    CHECK(cl[2] == 0u);
}

TEST_CASE("new view types are move-only", "[top2_770][lifetime]") {
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<ef::MagneticFieldView>);
    STATIC_REQUIRE(std::is_move_constructible_v<ef::MagneticFieldView>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<ef::OdometryView>);
    STATIC_REQUIRE(std::is_move_constructible_v<ef::OdometryView>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<ef::VibrationView>);
    STATIC_REQUIRE(std::is_move_constructible_v<ef::VibrationView>);
}
