/**
 * @file test_detect.cpp
 * @brief Range iteration tests for DetectView / ModelView / BoxView
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.
 */

#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <string_view>
#include <vector>

namespace ef = edgefirst::schemas;

// ============================================================================
// Golden CDR fixtures
//
// kGoldenDetectBytes: Detect with 3 boxes, mirroring the detect_multi_cdr
// fixture used in tests/c/test_edgefirst_msgs.c (Detect_multi.cdr).
//   stamp.sec  = 1234567890
//   stamp.nanosec = 123456789
//   frame_id   = "test_frame"
//   boxes[0]   label="a",      score≈0.95, track_id="t",             track_lifetime=1
//   boxes[1]   label="person", score≈0.87, track_id="track_long_id", track_lifetime=10
//   boxes[2]   label="ab",     score=0.50, track_id="abc",            track_lifetime=0
//
// kGoldenDetectEmptyBytes: Detect with 0 boxes.
//   stamp.sec  = 1234567890
//   stamp.nanosec = 123456789
//   frame_id   = "test_frame"
//   boxes_len  = 0
// ============================================================================

static constexpr std::uint8_t kGoldenDetectBytes[] = {
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

static constexpr std::uint8_t kGoldenDetectEmptyBytes[] = {
    0x00,0x01,0x00,0x00,0xd2,0x02,0x96,0x49,0x15,0xcd,0x5b,0x07,0x0b,0x00,0x00,0x00,
    0x74,0x65,0x73,0x74,0x5f,0x66,0x72,0x61,0x6d,0x65,0x00,0x00,0xd2,0x02,0x96,0x49,
    0x15,0xcd,0x5b,0x07,0x00,0x00,0x00,0x00,0x40,0x42,0x0f,0x00,0x00,0x00,0x00,0x00,
    0x80,0x84,0x1e,0x00,0x00,0x00,0x00,0x00,
};

// ============================================================================
// BoxView — standalone parse from CDR
// ============================================================================

TEST_CASE("BoxView from_cdr error on empty span", "[buffer_backed][box]") {
    auto v = ef::BoxView::from_cdr({});
    REQUIRE_FALSE(v.has_value());
}

// ============================================================================
// DetectView — from_cdr error path
// ============================================================================

TEST_CASE("DetectView from_cdr error on empty span", "[buffer_backed][detect]") {
    auto v = ef::DetectView::from_cdr({});
    REQUIRE_FALSE(v.has_value());
}

// ============================================================================
// ModelView — from_cdr error path
// ============================================================================

TEST_CASE("ModelView from_cdr error on empty span", "[buffer_backed][model]") {
    auto v = ef::ModelView::from_cdr({});
    REQUIRE_FALSE(v.has_value());
}

// ============================================================================
// ModelInfoView — from_cdr error path
// ============================================================================

TEST_CASE("ModelInfoView from_cdr error on empty span", "[buffer_backed][model_info]") {
    auto v = ef::ModelInfoView::from_cdr({});
    REQUIRE_FALSE(v.has_value());
}

// ============================================================================
// PointCloud2View — from_cdr error path
// ============================================================================

TEST_CASE("PointCloud2View from_cdr error on empty span", "[buffer_backed][pointcloud2]") {
    auto v = ef::PointCloud2View::from_cdr({});
    REQUIRE_FALSE(v.has_value());
}

// ============================================================================
// RadarCubeView — from_cdr error path
// ============================================================================

TEST_CASE("RadarCubeView from_cdr error on empty span", "[buffer_backed][radar_cube]") {
    auto v = ef::RadarCubeView::from_cdr({});
    REQUIRE_FALSE(v.has_value());
}

// ============================================================================
// RadarInfoView — from_cdr error path
// ============================================================================

TEST_CASE("RadarInfoView from_cdr error on empty span", "[buffer_backed][radar_info]") {
    auto v = ef::RadarInfoView::from_cdr({});
    REQUIRE_FALSE(v.has_value());
}

// ============================================================================
// DetectView boxes() iteration — runtime tests using golden CDR fixture
// ============================================================================

TEST_CASE("DetectView range iteration yields expected boxes", "[detect][iteration]") {
    auto det = ef::DetectView::from_cdr(
        ef::span<const std::uint8_t>{kGoldenDetectBytes, sizeof(kGoldenDetectBytes)});
    REQUIRE(det.has_value());

    // Parent header fields
    CHECK(det->stamp().sec == 1234567890u);
    CHECK(det->stamp().nanosec == 123456789u);
    CHECK(det->frame_id() == "test_frame");

    // Count check via boxes_len and ChildRange::size
    CHECK(det->boxes_len() == 3u);
    CHECK(det->boxes().size() == 3u);
    CHECK_FALSE(det->boxes().empty());

    // Range-based iteration: collect labels and scores
    std::vector<std::string_view> labels;
    std::vector<float>            scores;
    std::vector<std::string_view> track_ids;
    std::size_t count = 0;
    for (auto box : det->boxes()) {
        labels.push_back(box.label());
        scores.push_back(box.score());
        track_ids.push_back(box.track_id());
        ++count;
    }
    REQUIRE(count == 3u);

    CHECK(labels[0] == "a");
    CHECK(labels[1] == "person");
    CHECK(labels[2] == "ab");

    CHECK(scores[0] == Approx(0.95f).epsilon(0.001f));
    CHECK(scores[1] == Approx(0.87f).epsilon(0.001f));
    CHECK(scores[2] == Approx(0.50f).epsilon(0.001f));

    CHECK(track_ids[0] == "t");
    CHECK(track_ids[1] == "track_long_id");
    CHECK(track_ids[2] == "abc");

    // track_lifetime spot-checks
    auto it = det->boxes().begin();
    CHECK((*it).track_lifetime() == 1);
    ++it;
    CHECK((*it).track_lifetime() == 10);

    // Pointer-identity: label data must borrow into the CDR buffer (zero-copy)
    // Cast through uintptr_t to avoid Catch2 StringMaker<char const*> calling
    // strlen on non-NUL-terminated byte arrays (ASan global-buffer-overflow).
    auto box0 = *det->boxes().begin();
    const auto label_addr = reinterpret_cast<std::uintptr_t>(box0.label().data());
    const auto buf_begin  = reinterpret_cast<std::uintptr_t>(kGoldenDetectBytes);
    const auto buf_end    = buf_begin + sizeof(kGoldenDetectBytes);
    CHECK(label_addr >= buf_begin);
    CHECK(label_addr <  buf_end);
}

TEST_CASE("DetectView empty iteration", "[detect][iteration]") {
    auto det = ef::DetectView::from_cdr(
        ef::span<const std::uint8_t>{kGoldenDetectEmptyBytes, sizeof(kGoldenDetectEmptyBytes)});
    REQUIRE(det.has_value());

    CHECK(det->boxes_len() == 0u);
    CHECK(det->boxes().size() == 0u);
    CHECK(det->boxes().empty());
    CHECK(det->boxes().begin() == det->boxes().end());

    std::size_t count = 0;
    for ([[maybe_unused]] auto box : det->boxes()) {
        ++count;
    }
    CHECK(count == 0u);
}

TEST_CASE("DetectView boxes iterator STL interop", "[detect][iteration]") {
    auto det = ef::DetectView::from_cdr(
        ef::span<const std::uint8_t>{kGoldenDetectBytes, sizeof(kGoldenDetectBytes)});
    REQUIRE(det.has_value());

    // std::distance must compile and return the expected count
    auto n = std::distance(det->boxes().begin(), det->boxes().end());
    CHECK(n == 3);

    // std::count_if: count boxes with score > 0.6
    auto high_conf = std::count_if(
        det->boxes().begin(), det->boxes().end(),
        [](const ef::detail::BorrowedBoxView& b) { return b.score() > 0.6f; });
    CHECK(high_conf == 2);  // "a" (0.95) and "person" (0.87) qualify; "ab" (0.50) does not
}

TEST_CASE("DetectView from_cdr on garbage bytes", "[detect][error]") {
    std::vector<std::uint8_t> garbage(32, 0xFF);
    auto bad = ef::DetectView::from_cdr({garbage.data(), garbage.size()});
    // Garbage must either fail cleanly or parse without crashing
    (void)bad;
}

TEST_CASE("DetectView boxes expose track_created via BorrowedBoxView", "[detect][track_created]") {
    // kGoldenDetectBytes encodes 3 boxes with the following track_created values:
    //   boxes[0] label="a"      -> track_created={sec=1, nanosec=0}
    //   boxes[1] label="person" -> track_created={sec=2, nanosec=0}
    //   boxes[2] label="ab"     -> track_created={sec=0, nanosec=0}
    auto det = ef::DetectView::from_cdr(
        ef::span<const std::uint8_t>{kGoldenDetectBytes, sizeof(kGoldenDetectBytes)});
    REQUIRE(det.has_value());
    REQUIRE(det->boxes_len() == 3u);

    std::vector<ef::Time> track_createds;
    for (auto box : det->boxes()) {
        track_createds.push_back(box.track_created());
    }
    REQUIRE(track_createds.size() == 3u);

    CHECK(track_createds[0].sec     == 1);
    CHECK(track_createds[0].nanosec == 0u);
    CHECK(track_createds[1].sec     == 2);
    CHECK(track_createds[1].nanosec == 0u);
    CHECK(track_createds[2].sec     == 0);
    CHECK(track_createds[2].nanosec == 0u);
}

TEST_CASE("BorrowedMaskView API surface compiles", "[buffer_backed][mask]") {
    static_assert(
        sizeof(ef::detail::BorrowedMaskView) > 0,
        "BorrowedMaskView must be a complete type");
}

// ============================================================================
// ModelInfoView labels / shapes API
// ============================================================================

TEST_CASE("ModelInfoView labels_len and label accessor compile", "[buffer_backed][model_info]") {
    // Structural test: ensure the method signatures are correct.
    // We can't call them without a valid CDR, but we can verify the types.
    // Use a lambda to capture method pointer types without invoking them.
    auto check_labels_len = [](const ef::ModelInfoView& v) -> std::uint32_t {
        return v.labels_len();
    };
    auto check_label = [](const ef::ModelInfoView& v, std::uint32_t i) -> std::string_view {
        return v.label(i);
    };
    auto check_input_shape = [](const ef::ModelInfoView& v) -> ef::span<const std::uint32_t> {
        return v.input_shape();
    };
    auto check_output_shape = [](const ef::ModelInfoView& v) -> ef::span<const std::uint32_t> {
        return v.output_shape();
    };
    (void)check_labels_len;
    (void)check_label;
    (void)check_input_shape;
    (void)check_output_shape;
}
