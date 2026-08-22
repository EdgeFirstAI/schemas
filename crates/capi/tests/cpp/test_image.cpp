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

TEST_CASE("Image encode+view roundtrip", "[buffer_backed][image]") {
    std::vector<std::uint8_t> pixels(640 * 480 * 3, 42);
    auto img = ef::Image::encode(
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
    auto img = ef::Image::encode(
        {1000, 500}, "cam",
        480, 640, "rgb8", false,
        640 * 3, {pixels.data(), pixels.size()});
    REQUIRE(img.has_value());

    // Create a view over the encoded CDR
    auto cdr = img->as_cdr();
    auto view = ef::ImageView::from_cdr(cdr);
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
    auto v = ef::ImageView::from_cdr({});
    REQUIRE_FALSE(v.has_value());
}

TEST_CASE("ImageView accessors match encoded values", "[buffer_backed][image]") {
    std::vector<std::uint8_t> pixels(320 * 240 * 1, 7);
    auto img = ef::Image::encode(
        {9, 8}, "lidar_cam",
        240, 320, "mono8", false,
        320, {pixels.data(), pixels.size()});
    REQUIRE(img.has_value());

    auto cdr = img->as_cdr();
    auto view = ef::ImageView::from_cdr(cdr);
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
    auto img = ef::Image::encode(
        {1, 2}, "f", 1, 10, "mono8", false, 10,
        {pixels.data(), pixels.size()});
    REQUIRE(img.has_value());
    auto img2 = std::move(*img);
    CHECK(img2.width() == 10);
    CHECK(!img2.as_cdr().empty());
}
