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

static constexpr ef::Time kStamp{1234567890, 123456789};
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
    ros_bytes_free(r->data, r->size);
    return out;
}

TEST_CASE("AccelStampedBuilder bytes match golden", "[geometry][builder]") {
    auto golden = load_fixture("testdata/cdr/geometry_msgs/AccelStamped.cdr");
    auto b = ef::AccelStampedBuilder::create();
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
    auto b = ef::TransformStampedBuilder::create();
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
    ef::Pose poses[2] = {
        ef::Pose{1.5, -2.5, 3.0, 0.0, 0.0, 0.0, 1.0},
        ef::Pose{10.0, 20.0, 30.0, 0.0, 0.0, 0.0, 1.0},
    };
    auto b = ef::PoseArrayBuilder::create();
    REQUIRE(b.has_value());
    b->stamp(kStamp);
    REQUIRE(b->frame_id(kFrame).has_value());
    REQUIRE(b->poses(ef::span<const ef::Pose>{poses, 2}).has_value());
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

    auto b = ef::OdometryBuilder::create();
    REQUIRE(b.has_value());
    b->stamp(kStamp);
    REQUIRE(b->frame_id(kFrame).has_value());
    REQUIRE(b->child_frame_id("base_link").has_value());
    b->pose(ef::Pose{1.5, -2.5, 3.0, 0.0, 0.0, 0.0, 1.0});
    REQUIRE(b->pose_covariance(ef::span<const double>{pose_cov.data(), pose_cov.size()}).has_value());
    b->twist(1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    REQUIRE(b->twist_covariance(ef::span<const double>{twist_cov.data(), twist_cov.size()}).has_value());
    auto bytes = take_bytes(b->build());
    REQUIRE(bytes == golden);
}
