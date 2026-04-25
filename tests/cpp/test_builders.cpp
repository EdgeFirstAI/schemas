/**
 * @file test_builders.cpp
 * @brief Tests for C++ builder wrappers in schemas.hpp
 *
 * Covers: create/build/encode_into round-trips, fluent chaining, move
 * semantics, error handling, and representative message types from each
 * namespace (std_msgs, sensor_msgs, edgefirst_msgs, foxglove_msgs).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.
 */

#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace ef = edgefirst::schemas;

// ============================================================================
// Helpers
// ============================================================================

/// Free a Released buffer (wraps ros_bytes_free).
static void free_released(ef::Released& r) {
    if (r.data) {
        ros_bytes_free(r.data, r.size);
        r.data = nullptr;
        r.size = 0;
    }
}

// ============================================================================
// std_msgs — HeaderBuilder
// ============================================================================

TEST_CASE("HeaderBuilder create + build round-trip", "[builder][header]") {
    auto b = ef::HeaderBuilder::create();
    REQUIRE(b.has_value());

    // Fluent chaining
    b->stamp({100, 200});
    auto fid = b->frame_id("world");
    REQUIRE(fid.has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    auto rel = *result;
    REQUIRE(rel.data != nullptr);
    REQUIRE(rel.size > 0);

    // Round-trip: parse the CDR and verify fields
    auto view = ef::HeaderView::from_cdr({rel.data, rel.size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 100);
    CHECK(view->stamp().nanosec == 200);
    CHECK(view->frame_id() == "world");

    free_released(rel);
}

TEST_CASE("HeaderBuilder encode_into", "[builder][header][encode_into]") {
    auto b = ef::HeaderBuilder::create();
    REQUIRE(b.has_value());
    b->stamp({42, 0});
    auto fid = b->frame_id("enc");
    REQUIRE(fid.has_value());

    // Encode into a pre-allocated buffer
    std::vector<std::uint8_t> buf(1024);
    auto len = b->encode_into({buf.data(), buf.size()});
    REQUIRE(len.has_value());
    REQUIRE(*len > 0);
    REQUIRE(*len <= buf.size());

    auto view = ef::HeaderView::from_cdr({buf.data(), *len});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 42);
    CHECK(view->frame_id() == "enc");
}

TEST_CASE("HeaderBuilder encode_into too-small buffer", "[builder][header][error]") {
    auto b = ef::HeaderBuilder::create();
    REQUIRE(b.has_value());
    b->stamp({1, 2});
    auto fid = b->frame_id("tiny");
    REQUIRE(fid.has_value());

    std::uint8_t buf[4] = {};
    auto len = b->encode_into({buf, sizeof(buf)});
    CHECK(!len.has_value());
}

// ============================================================================
// sensor_msgs — ImuBuilder
// ============================================================================

TEST_CASE("ImuBuilder full round-trip", "[builder][imu]") {
    auto b = ef::ImuBuilder::create();
    REQUIRE(b.has_value());

    b->stamp({10, 20})
      .orientation({1.0, 2.0, 3.0, 4.0})
      .angular_velocity({0.1, 0.2, 0.3})
      .linear_acceleration({9.8, 0.0, 0.0});

    auto fid = b->frame_id("imu_link");
    REQUIRE(fid.has_value());

    std::array<double, 9> cov = {1,0,0, 0,1,0, 0,0,1};
    REQUIRE(b->orientation_covariance(cov).has_value());
    REQUIRE(b->angular_velocity_covariance(cov).has_value());
    REQUIRE(b->linear_acceleration_covariance(cov).has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    auto rel = *result;

    auto view = ef::ImuView::from_cdr({rel.data, rel.size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 10);
    CHECK(view->stamp().nanosec == 20);
    CHECK(view->frame_id() == "imu_link");

    auto q = view->orientation();
    CHECK(q.x == Approx(1.0));
    CHECK(q.y == Approx(2.0));
    CHECK(q.z == Approx(3.0));
    CHECK(q.w == Approx(4.0));

    auto av = view->angular_velocity();
    CHECK(av.x == Approx(0.1));
    CHECK(av.y == Approx(0.2));
    CHECK(av.z == Approx(0.3));

    auto la = view->linear_acceleration();
    CHECK(la.x == Approx(9.8));
    CHECK(la.y == Approx(0.0));
    CHECK(la.z == Approx(0.0));

    auto oc = view->orientation_covariance();
    CHECK(oc[0] == Approx(1.0));
    CHECK(oc[4] == Approx(1.0));
    CHECK(oc[8] == Approx(1.0));
    CHECK(oc[1] == Approx(0.0));

    free_released(rel);
}

// ============================================================================
// sensor_msgs — TemperatureBuilder (simplest sensor type)
// ============================================================================

TEST_CASE("TemperatureBuilder round-trip", "[builder][temperature]") {
    auto b = ef::TemperatureBuilder::create();
    REQUIRE(b.has_value());

    b->stamp({300, 0}).temperature(25.5).variance(0.01);
    REQUIRE(b->frame_id("therm").has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    auto rel = *result;

    auto view = ef::TemperatureView::from_cdr({rel.data, rel.size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 300);
    CHECK(view->frame_id() == "therm");
    CHECK(view->temperature() == Approx(25.5));
    CHECK(view->variance() == Approx(0.01));

    free_released(rel);
}

// ============================================================================
// sensor_msgs — NavSatFixBuilder
// ============================================================================

TEST_CASE("NavSatFixBuilder round-trip", "[builder][navsatfix]") {
    auto b = ef::NavSatFixBuilder::create();
    REQUIRE(b.has_value());

    b->stamp({500, 100})
      .status(0, 1)
      .latitude(45.5)
      .longitude(-73.6)
      .altitude(50.0)
      .position_covariance_type(2);
    REQUIRE(b->frame_id("gps").has_value());

    std::array<double, 9> cov = {1,0,0, 0,1,0, 0,0,1};
    REQUIRE(b->position_covariance(cov).has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    auto rel = *result;

    auto view = ef::NavSatFixView::from_cdr({rel.data, rel.size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 500);
    CHECK(view->frame_id() == "gps");
    CHECK(view->latitude() == Approx(45.5));
    CHECK(view->longitude() == Approx(-73.6));
    CHECK(view->altitude() == Approx(50.0));

    free_released(rel);
}

// ============================================================================
// edgefirst_msgs — DetectBuilder (with borrowed box array)
// ============================================================================

TEST_CASE("DetectBuilder with boxes round-trip", "[builder][detect]") {
    auto b = ef::DetectBuilder::create();
    REQUIRE(b.has_value());

    b->stamp({700, 0})
      .input_timestamp({690, 0})
      .model_time({695, 0})
      .output_time({700, 0});
    REQUIRE(b->frame_id("cam0").has_value());

    // Build a box descriptor — the elem struct is borrowed by the builder.
    ros_detect_box_elem_t box{};
    box.center_x = 0.5f;
    box.center_y = 0.5f;
    box.width    = 0.2f;
    box.height   = 0.3f;
    box.label    = "person";
    box.score    = 0.95f;
    box.distance = 5.0f;
    box.speed    = 1.2f;
    box.track_id = "t1";
    box.track_lifetime = 10;
    box.track_created_sec = 600;
    box.track_created_nanosec = 0;

    ef::span<const ros_detect_box_elem_t> boxes_span(&box, 1);
    REQUIRE(b->boxes(boxes_span).has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    auto rel = *result;

    auto view = ef::DetectView::from_cdr({rel.data, rel.size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 700);
    CHECK(view->frame_id() == "cam0");
    CHECK(view->boxes_len() == 1);

    // Iterate boxes
    std::size_t count = 0;
    for (auto bv : view->boxes()) {
        CHECK(bv.label() == "person");
        CHECK(bv.score() == Approx(0.95f));
        CHECK(bv.center_x() == Approx(0.5f));
        CHECK(bv.width() == Approx(0.2f));
        CHECK(bv.height() == Approx(0.3f));
        ++count;
    }
    CHECK(count == 1);

    free_released(rel);
}

// ============================================================================
// edgefirst_msgs — VibrationBuilder
// ============================================================================

TEST_CASE("VibrationBuilder round-trip", "[builder][vibration]") {
    auto b = ef::VibrationBuilder::create();
    REQUIRE(b.has_value());

    b->stamp({800, 0})
      .vibration({1.0, 2.0, 3.0})
      .band_lower_hz(10.0f)
      .band_upper_hz(1000.0f)
      .measurement_type(ef::VibrationView::MEASUREMENT_RMS)
      .unit(ef::VibrationView::UNIT_ACCEL_G);
    REQUIRE(b->frame_id("accel0").has_value());

    std::array<std::uint32_t, 3> clip = {0, 0, 1};
    REQUIRE(b->clipping({clip.data(), clip.size()}).has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    auto rel = *result;

    auto view = ef::VibrationView::from_cdr({rel.data, rel.size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 800);
    CHECK(view->frame_id() == "accel0");
    CHECK(view->vibration().x == Approx(1.0));
    CHECK(view->vibration().y == Approx(2.0));
    CHECK(view->vibration().z == Approx(3.0));
    CHECK(view->band_lower_hz() == Approx(10.0f));
    CHECK(view->band_upper_hz() == Approx(1000.0f));
    CHECK(view->measurement_type() == ef::VibrationView::MEASUREMENT_RMS);
    CHECK(view->unit() == ef::VibrationView::UNIT_ACCEL_G);
    CHECK(view->clipping_len() == 3);

    std::array<std::uint32_t, 3> clip_out{};
    view->clipping({clip_out.data(), clip_out.size()});
    CHECK(clip_out[0] == 0);
    CHECK(clip_out[1] == 0);
    CHECK(clip_out[2] == 1);

    free_released(rel);
}

// ============================================================================
// edgefirst_msgs — ImageBuilder (builder vs. legacy encode)
// ============================================================================

TEST_CASE("ImageBuilder round-trip", "[builder][image]") {
    auto b = ef::ImageBuilder::create();
    REQUIRE(b.has_value());

    std::vector<std::uint8_t> pixels(320 * 240 * 3, 0xAB);

    b->stamp({1000, 500})
      .height(240)
      .width(320)
      .is_bigendian(0)
      .step(320 * 3);
    REQUIRE(b->frame_id("cam").has_value());
    REQUIRE(b->encoding("rgb8").has_value());
    REQUIRE(b->data({pixels.data(), pixels.size()}).has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    auto rel = *result;

    auto view = ef::ImageView::from_cdr({rel.data, rel.size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 1000);
    CHECK(view->stamp().nanosec == 500);
    CHECK(view->frame_id() == "cam");
    CHECK(view->height() == 240);
    CHECK(view->width() == 320);
    CHECK(view->encoding() == "rgb8");
    CHECK(view->step() == 320u * 3u);
    CHECK(view->data().size() == pixels.size());

    free_released(rel);
}

// ============================================================================
// edgefirst_msgs — CameraFrameBuilder
// ============================================================================

TEST_CASE("CameraFrameBuilder round-trip", "[builder][camera_frame]") {
    auto b = ef::CameraFrameBuilder::create();
    REQUIRE(b.has_value());

    b->stamp({900, 0})
      .seq(42)
      .pid(1)
      .width(1920)
      .height(1080)
      .fence_fd(-1);
    REQUIRE(b->frame_id("cam0").has_value());
    REQUIRE(b->format("NV12").has_value());
    REQUIRE(b->color_space("bt709").has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    auto rel = *result;

    auto view = ef::CameraFrameView::from_cdr({rel.data, rel.size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 900);
    CHECK(view->frame_id() == "cam0");
    CHECK(view->seq() == 42);
    CHECK(view->width() == 1920);
    CHECK(view->height() == 1080);
    CHECK(view->format() == "NV12");

    free_released(rel);
}

// ============================================================================
// foxglove_msgs — FoxgloveTextAnnotationBuilder (builder-only, no view)
// ============================================================================

TEST_CASE("FoxgloveTextAnnotationBuilder build succeeds", "[builder][foxglove]") {
    auto b = ef::FoxgloveTextAnnotationBuilder::create();
    REQUIRE(b.has_value());

    b->timestamp({1100, 0})
      .position(100.0, 200.0)
      .font_size(14.0)
      .text_color(1.0, 0.0, 0.0, 1.0)
      .background_color(0.0, 0.0, 0.0, 0.5);
    REQUIRE(b->text("Hello world").has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    CHECK(result->data != nullptr);
    CHECK(result->size > 0);

    // No view type available — just verify we can build
    ros_bytes_free(result->data, result->size);
}

TEST_CASE("FoxgloveTextAnnotationBuilder encode_into", "[builder][foxglove][encode_into]") {
    auto b = ef::FoxgloveTextAnnotationBuilder::create();
    REQUIRE(b.has_value());

    b->timestamp({0, 0}).position(0, 0).font_size(12.0);
    REQUIRE(b->text("test").has_value());

    std::vector<std::uint8_t> buf(4096);
    auto len = b->encode_into({buf.data(), buf.size()});
    REQUIRE(len.has_value());
    CHECK(*len > 0);
}

// ============================================================================
// foxglove_msgs — FoxglovePointAnnotationBuilder (builder-only)
// ============================================================================

TEST_CASE("FoxglovePointAnnotationBuilder build succeeds", "[builder][foxglove]") {
    auto b = ef::FoxglovePointAnnotationBuilder::create();
    REQUIRE(b.has_value());

    b->timestamp({1200, 0})
      .type(1)  // LINE_LIST
      .outline_color(0.0, 1.0, 0.0, 1.0)
      .fill_color(0.0, 0.5, 0.0, 0.5)
      .thickness(2.0);

    ros_foxglove_point2_elem_t pts[] = {{10.0, 20.0}, {30.0, 40.0}};
    REQUIRE(b->points({pts, 2}).has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    CHECK(result->data != nullptr);
    ros_bytes_free(result->data, result->size);
}

// ============================================================================
// foxglove_msgs — FoxgloveImageAnnotationBuilder (builder-only)
// ============================================================================

TEST_CASE("FoxgloveImageAnnotationBuilder build succeeds", "[builder][foxglove]") {
    auto b = ef::FoxgloveImageAnnotationBuilder::create();
    REQUIRE(b.has_value());

    // Build with empty arrays — valid message with no annotations
    auto result = b->build();
    REQUIRE(result.has_value());
    CHECK(result->data != nullptr);
    ros_bytes_free(result->data, result->size);
}

// ============================================================================
// Move semantics
// ============================================================================

TEST_CASE("Builder move constructor", "[builder][move]") {
    auto b1 = ef::HeaderBuilder::create();
    REQUIRE(b1.has_value());
    b1->stamp({1, 2});

    // Move construct
    ef::HeaderBuilder b2 = std::move(*b1);

    // The moved-to builder should still work
    REQUIRE(b2.frame_id("moved").has_value());
    auto result = b2.build();
    REQUIRE(result.has_value());

    auto view = ef::HeaderView::from_cdr({result->data, result->size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 1);
    CHECK(view->frame_id() == "moved");

    free_released(*result);
}

TEST_CASE("Builder move assignment", "[builder][move]") {
    auto b1 = ef::HeaderBuilder::create();
    auto b2 = ef::HeaderBuilder::create();
    REQUIRE(b1.has_value());
    REQUIRE(b2.has_value());

    b1->stamp({10, 20});
    b2->stamp({30, 40});

    // Move assign — b2's old handle should be freed, b1's transferred
    *b2 = std::move(*b1);
    REQUIRE(b2->frame_id("reassigned").has_value());

    auto result = b2->build();
    REQUIRE(result.has_value());

    auto view = ef::HeaderView::from_cdr({result->data, result->size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 10);
    CHECK(view->frame_id() == "reassigned");

    free_released(*result);
}

// ============================================================================
// ModelInfoBuilder — complex type with labels
// ============================================================================

TEST_CASE("ModelInfoBuilder round-trip", "[builder][model_info]") {
    auto b = ef::ModelInfoBuilder::create();
    REQUIRE(b.has_value());

    b->stamp({1300, 0}).input_type(1).output_type(1);
    REQUIRE(b->frame_id("model0").has_value());
    REQUIRE(b->model_type("detection").has_value());
    REQUIRE(b->model_format("tflite").has_value());
    REQUIRE(b->model_name("yolov8n").has_value());

    std::array<std::uint32_t, 4> in_shape = {1, 320, 320, 3};
    std::array<std::uint32_t, 3> out_shape = {1, 100, 6};
    REQUIRE(b->input_shape({in_shape.data(), in_shape.size()}).has_value());
    REQUIRE(b->output_shape({out_shape.data(), out_shape.size()}).has_value());

    const char* labels[] = {"person", "car", "bicycle"};
    REQUIRE(b->labels(labels, 3).has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    auto rel = *result;

    auto view = ef::ModelInfoView::from_cdr({rel.data, rel.size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 1300);
    CHECK(view->frame_id() == "model0");
    CHECK(view->model_type() == "detection");
    CHECK(view->model_format() == "tflite");
    CHECK(view->model_name() == "yolov8n");
    CHECK(view->labels_len() == 3);

    free_released(rel);
}

// ============================================================================
// CompressedImageBuilder
// ============================================================================

TEST_CASE("CompressedImageBuilder round-trip", "[builder][compressed_image]") {
    auto b = ef::CompressedImageBuilder::create();
    REQUIRE(b.has_value());

    std::vector<std::uint8_t> jpeg_data(1000, 0xFF);

    b->stamp({400, 0});
    REQUIRE(b->frame_id("cam1").has_value());
    REQUIRE(b->format("jpeg").has_value());
    REQUIRE(b->data({jpeg_data.data(), jpeg_data.size()}).has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    auto rel = *result;

    auto view = ef::CompressedImageView::from_cdr({rel.data, rel.size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 400);
    CHECK(view->frame_id() == "cam1");
    CHECK(view->format() == "jpeg");
    CHECK(view->data().size() == 1000);

    free_released(rel);
}

// ============================================================================
// PointCloud2Builder
// ============================================================================

TEST_CASE("PointCloud2Builder round-trip", "[builder][pointcloud2]") {
    auto b = ef::PointCloud2Builder::create();
    REQUIRE(b.has_value());

    b->stamp({600, 0})
      .height(1)
      .width(100)
      .is_bigendian(0)
      .point_step(16)
      .row_step(1600)
      .is_dense(true);
    REQUIRE(b->frame_id("lidar").has_value());

    // Point fields
    ros_point_field_elem_t fields[] = {
        {"x", 0, 7, 1},
        {"y", 4, 7, 1},
        {"z", 8, 7, 1},
    };
    REQUIRE(b->fields({fields, 3}).has_value());

    // Dummy point data
    std::vector<std::uint8_t> data(1600, 0);
    REQUIRE(b->data({data.data(), data.size()}).has_value());

    auto result = b->build();
    REQUIRE(result.has_value());
    auto rel = *result;

    auto view = ef::PointCloud2View::from_cdr({rel.data, rel.size});
    REQUIRE(view.has_value());
    CHECK(view->stamp().sec == 600);
    CHECK(view->frame_id() == "lidar");
    CHECK(view->height() == 1);
    CHECK(view->width() == 100);
    CHECK(view->point_step() == 16);

    free_released(rel);
}
