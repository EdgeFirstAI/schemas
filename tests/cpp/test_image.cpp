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

    // Zero-copy assertion: data() pointer must point INTO the cdr buffer
    auto px = view->data();
    CHECK(px.data() >= cdr.data());
    CHECK(px.data() + px.size() <= cdr.data() + cdr.size());

    // String accessor also borrows from the CDR buffer
    auto enc = view->encoding();
    CHECK(enc.data() >= reinterpret_cast<const char*>(cdr.data()));
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
