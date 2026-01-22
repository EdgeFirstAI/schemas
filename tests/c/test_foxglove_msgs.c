/**
 * @file test_foxglove_msgs.c
 * @brief Criterion tests for Foxglove messages (FoxgloveCompressedVideo)
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright 2025 Au-Zone Technologies. All Rights Reserved.
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include "edgefirst/schemas.h"

// ============================================================================
// FoxgloveCompressedVideo Tests
// ============================================================================

Test(foxglove_msgs, compressed_video_new_and_free) {
    FoxgloveCompressedVideo *video = foxglove_compressed_video_new();
    cr_assert_not_null(video, "foxglove_compressed_video_new() returned NULL");
    foxglove_compressed_video_free(video);
}

Test(foxglove_msgs, compressed_video_free_null) {
    // Should not crash when freeing NULL
    foxglove_compressed_video_free(NULL);
}

Test(foxglove_msgs, compressed_video_get_header) {
    FoxgloveCompressedVideo *video = foxglove_compressed_video_new();
    cr_assert_not_null(video);

    const RosHeader *header = foxglove_compressed_video_get_header(video);
    cr_assert_not_null(header, "Header should not be NULL");

    foxglove_compressed_video_free(video);
}

Test(foxglove_msgs, compressed_video_get_header_mut) {
    FoxgloveCompressedVideo *video = foxglove_compressed_video_new();
    cr_assert_not_null(video);

    RosHeader *header = foxglove_compressed_video_get_header_mut(video);
    cr_assert_not_null(header, "Mutable header should not be NULL");

    // Set frame_id through the mutable header
    int ret = ros_header_set_frame_id(header, "camera0");
    cr_assert_eq(ret, 0);

    // Verify via const getter
    const RosHeader *const_header = foxglove_compressed_video_get_header(video);
    char *frame_id = ros_header_get_frame_id(const_header);
    cr_assert_str_eq(frame_id, "camera0");
    free(frame_id);

    foxglove_compressed_video_free(video);
}

Test(foxglove_msgs, compressed_video_data_empty_default) {
    FoxgloveCompressedVideo *video = foxglove_compressed_video_new();
    cr_assert_not_null(video);

    size_t len = 0;
    const uint8_t *data = foxglove_compressed_video_get_data(video, &len);
    (void)data;  // Pointer may or may not be NULL for empty vec
    cr_assert_eq(len, 0, "Default data length should be 0");

    foxglove_compressed_video_free(video);
}

Test(foxglove_msgs, compressed_video_set_data) {
    FoxgloveCompressedVideo *video = foxglove_compressed_video_new();
    cr_assert_not_null(video);

    // Simulated H.264 NAL unit header (not real video data)
    uint8_t test_data[] = {0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0x00, 0x1e};

    int ret = foxglove_compressed_video_set_data(video, test_data, sizeof(test_data));
    cr_assert_eq(ret, 0, "set_data should return 0 on success");

    size_t len = 0;
    const uint8_t *data = foxglove_compressed_video_get_data(video, &len);
    cr_assert_eq(len, sizeof(test_data));

    for (size_t i = 0; i < len; i++) {
        cr_assert_eq(data[i], test_data[i], "Data mismatch at index %zu", i);
    }

    foxglove_compressed_video_free(video);
}

Test(foxglove_msgs, compressed_video_format_empty_default) {
    FoxgloveCompressedVideo *video = foxglove_compressed_video_new();
    cr_assert_not_null(video);

    char *format = foxglove_compressed_video_get_format(video);
    cr_assert_not_null(format);
    cr_assert_str_eq(format, "", "Default format should be empty");
    free(format);

    foxglove_compressed_video_free(video);
}

Test(foxglove_msgs, compressed_video_set_format_h264) {
    FoxgloveCompressedVideo *video = foxglove_compressed_video_new();
    cr_assert_not_null(video);

    int ret = foxglove_compressed_video_set_format(video, "h264");
    cr_assert_eq(ret, 0, "set_format should return 0 on success");

    char *format = foxglove_compressed_video_get_format(video);
    cr_assert_not_null(format);
    cr_assert_str_eq(format, "h264");
    free(format);

    foxglove_compressed_video_free(video);
}

Test(foxglove_msgs, compressed_video_set_format_h265) {
    FoxgloveCompressedVideo *video = foxglove_compressed_video_new();
    cr_assert_not_null(video);

    int ret = foxglove_compressed_video_set_format(video, "h265");
    cr_assert_eq(ret, 0);

    char *format = foxglove_compressed_video_get_format(video);
    cr_assert_str_eq(format, "h265");
    free(format);

    foxglove_compressed_video_free(video);
}

Test(foxglove_msgs, compressed_video_serialize_empty) {
    FoxgloveCompressedVideo *video = foxglove_compressed_video_new();
    cr_assert_not_null(video);

    // Query required size (Khronos pattern: buffer=NULL)
    size_t required_size = 0;
    int ret = foxglove_compressed_video_serialize(video, NULL, 0, &required_size);
    cr_assert_eq(ret, 0, "Size query should succeed");
    cr_assert_gt(required_size, 0, "Required size should be > 0");

    // Allocate and serialize
    uint8_t *buffer = (uint8_t *)malloc(required_size);
    cr_assert_not_null(buffer);
    ret = foxglove_compressed_video_serialize(video, buffer, required_size, NULL);
    cr_assert_eq(ret, 0, "Serialization should succeed");

    free(buffer);
    foxglove_compressed_video_free(video);
}

Test(foxglove_msgs, compressed_video_serialize_deserialize_roundtrip) {
    FoxgloveCompressedVideo *original = foxglove_compressed_video_new();
    cr_assert_not_null(original);

    // Set up test data
    RosHeader *header = foxglove_compressed_video_get_header_mut(original);
    RosTime *stamp = ros_header_get_stamp_mut(header);
    ros_time_set_sec(stamp, 1234567890);
    ros_time_set_nanosec(stamp, 123456789);
    ros_header_set_frame_id(header, "video_stream");

    uint8_t test_data[] = {0x00, 0x00, 0x00, 0x01, 0x65, 0x88, 0x84, 0x00};
    foxglove_compressed_video_set_data(original, test_data, sizeof(test_data));
    foxglove_compressed_video_set_format(original, "h264");

    // Serialize using Khronos pattern
    size_t required_size = 0;
    int ret = foxglove_compressed_video_serialize(original, NULL, 0, &required_size);
    cr_assert_eq(ret, 0);
    cr_assert_gt(required_size, 0);

    uint8_t *buffer = (uint8_t *)malloc(required_size);
    cr_assert_not_null(buffer);
    ret = foxglove_compressed_video_serialize(original, buffer, required_size, NULL);
    cr_assert_eq(ret, 0);

    // Deserialize
    FoxgloveCompressedVideo *deserialized = foxglove_compressed_video_deserialize(buffer, required_size);
    cr_assert_not_null(deserialized, "Deserialization should succeed");

    // Verify header
    const RosHeader *deser_header = foxglove_compressed_video_get_header(deserialized);
    const RosTime *deser_stamp = ros_header_get_stamp(deser_header);
    cr_assert_eq(ros_time_get_sec(deser_stamp), 1234567890);
    cr_assert_eq(ros_time_get_nanosec(deser_stamp), 123456789);

    char *frame_id = ros_header_get_frame_id(deser_header);
    cr_assert_str_eq(frame_id, "video_stream");
    free(frame_id);

    // Verify data
    size_t data_len = 0;
    const uint8_t *data = foxglove_compressed_video_get_data(deserialized, &data_len);
    cr_assert_eq(data_len, sizeof(test_data));
    for (size_t i = 0; i < data_len; i++) {
        cr_assert_eq(data[i], test_data[i]);
    }

    // Verify format
    char *format = foxglove_compressed_video_get_format(deserialized);
    cr_assert_str_eq(format, "h264");
    free(format);

    free(buffer);
    foxglove_compressed_video_free(original);
    foxglove_compressed_video_free(deserialized);
}

Test(foxglove_msgs, compressed_video_serialize_buffer_too_small) {
    FoxgloveCompressedVideo *video = foxglove_compressed_video_new();
    cr_assert_not_null(video);

    // Query required size
    size_t required_size = 0;
    int ret = foxglove_compressed_video_serialize(video, NULL, 0, &required_size);
    cr_assert_eq(ret, 0);
    cr_assert_gt(required_size, 0);

    // Try with buffer too small
    uint8_t small_buffer[4];
    size_t actual_size = 0;
    errno = 0;
    ret = foxglove_compressed_video_serialize(video, small_buffer, sizeof(small_buffer), &actual_size);
    cr_assert_eq(ret, -1, "Should fail with small buffer");
    cr_assert_eq(errno, ENOBUFS, "errno should be ENOBUFS");
    cr_assert_eq(actual_size, required_size, "Should still report required size");

    foxglove_compressed_video_free(video);
}

Test(foxglove_msgs, compressed_video_deserialize_null_bytes) {
    errno = 0;
    FoxgloveCompressedVideo *video = foxglove_compressed_video_deserialize(NULL, 100);
    cr_assert_null(video, "Should return NULL for NULL bytes");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(foxglove_msgs, compressed_video_deserialize_zero_length) {
    uint8_t dummy = 0;
    errno = 0;
    FoxgloveCompressedVideo *video = foxglove_compressed_video_deserialize(&dummy, 0);
    cr_assert_null(video, "Should return NULL for zero length");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(foxglove_msgs, compressed_video_deserialize_invalid_data) {
    uint8_t garbage[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    FoxgloveCompressedVideo *video = foxglove_compressed_video_deserialize(garbage, sizeof(garbage));
    cr_assert_null(video, "Should return NULL for invalid CDR data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(foxglove_msgs, compressed_video_large_data) {
    FoxgloveCompressedVideo *video = foxglove_compressed_video_new();
    cr_assert_not_null(video);

    // Simulate a larger video frame (1MB)
    size_t frame_size = 1024 * 1024;
    uint8_t *large_data = (uint8_t *)malloc(frame_size);
    cr_assert_not_null(large_data);

    // Fill with pattern
    for (size_t i = 0; i < frame_size; i++) {
        large_data[i] = (uint8_t)(i & 0xFF);
    }

    int ret = foxglove_compressed_video_set_data(video, large_data, frame_size);
    cr_assert_eq(ret, 0);

    size_t len = 0;
    const uint8_t *data = foxglove_compressed_video_get_data(video, &len);
    cr_assert_eq(len, frame_size);

    // Verify first and last bytes
    cr_assert_eq(data[0], 0);
    cr_assert_eq(data[frame_size - 1], (uint8_t)((frame_size - 1) & 0xFF));

    free(large_data);
    foxglove_compressed_video_free(video);
}

// ============================================================================
// Point2 Tests
// ============================================================================

Test(foxglove_msgs, point2_create_and_destroy) {
    FoxglovePoint2 *point = foxglove_point2_new();
    cr_assert_not_null(point);

    foxglove_point2_set_x(point, 100.5);
    foxglove_point2_set_y(point, 200.75);

    cr_assert_float_eq(foxglove_point2_get_x(point), 100.5, 0.0001);
    cr_assert_float_eq(foxglove_point2_get_y(point), 200.75, 0.0001);

    foxglove_point2_free(point);
}

Test(foxglove_msgs, point2_free_null) {
    foxglove_point2_free(NULL);
}

// ============================================================================
// Color Tests
// ============================================================================

Test(foxglove_msgs, color_create_and_destroy) {
    FoxgloveColor *color = foxglove_color_new();
    cr_assert_not_null(color);

    foxglove_color_set_r(color, 1.0);
    foxglove_color_set_g(color, 0.5);
    foxglove_color_set_b(color, 0.25);
    foxglove_color_set_a(color, 0.9);

    cr_assert_float_eq(foxglove_color_get_r(color), 1.0, 0.0001);
    cr_assert_float_eq(foxglove_color_get_g(color), 0.5, 0.0001);
    cr_assert_float_eq(foxglove_color_get_b(color), 0.25, 0.0001);
    cr_assert_float_eq(foxglove_color_get_a(color), 0.9, 0.0001);

    foxglove_color_free(color);
}

Test(foxglove_msgs, color_free_null) {
    foxglove_color_free(NULL);
}

// ============================================================================
// CircleAnnotations Tests
// ============================================================================

Test(foxglove_msgs, circle_annotations_create_and_destroy) {
    FoxgloveCircleAnnotations *circle = foxglove_circle_annotations_new();
    cr_assert_not_null(circle);

    // Set timestamp
    RosTime *timestamp = foxglove_circle_annotations_get_timestamp_mut(circle);
    cr_assert_not_null(timestamp);
    ros_time_set_sec(timestamp, 1000);
    ros_time_set_nanosec(timestamp, 500000000);

    // Set position
    FoxglovePoint2 *position = foxglove_circle_annotations_get_position_mut(circle);
    cr_assert_not_null(position);
    foxglove_point2_set_x(position, 320.0);
    foxglove_point2_set_y(position, 240.0);

    // Set diameter and thickness
    foxglove_circle_annotations_set_diameter(circle, 50.0);
    foxglove_circle_annotations_set_thickness(circle, 2.0);

    // Set colors
    FoxgloveColor *fill = foxglove_circle_annotations_get_fill_color_mut(circle);
    cr_assert_not_null(fill);
    foxglove_color_set_r(fill, 1.0);
    foxglove_color_set_a(fill, 0.5);

    FoxgloveColor *outline = foxglove_circle_annotations_get_outline_color_mut(circle);
    cr_assert_not_null(outline);
    foxglove_color_set_g(outline, 1.0);
    foxglove_color_set_a(outline, 1.0);

    // Verify
    cr_assert_float_eq(foxglove_circle_annotations_get_diameter(circle), 50.0, 0.0001);
    cr_assert_float_eq(foxglove_circle_annotations_get_thickness(circle), 2.0, 0.0001);

    const FoxglovePoint2 *pos = foxglove_circle_annotations_get_position(circle);
    cr_assert_float_eq(foxglove_point2_get_x(pos), 320.0, 0.0001);

    foxglove_circle_annotations_free(circle);
}

Test(foxglove_msgs, circle_annotations_free_null) {
    foxglove_circle_annotations_free(NULL);
}

// ============================================================================
// PointAnnotations Tests
// ============================================================================

Test(foxglove_msgs, point_annotations_create_and_destroy) {
    FoxglovePointAnnotations *ann = foxglove_point_annotations_new();
    cr_assert_not_null(ann);

    // Set timestamp
    RosTime *timestamp = foxglove_point_annotations_get_timestamp_mut(ann);
    cr_assert_not_null(timestamp);
    ros_time_set_sec(timestamp, 2000);

    // Set type
    foxglove_point_annotations_set_type(ann, FOXGLOVE_POINT_ANNOTATION_LINE_STRIP);

    // Set thickness
    foxglove_point_annotations_set_thickness(ann, 3.0);

    cr_assert_eq(foxglove_point_annotations_get_type(ann), FOXGLOVE_POINT_ANNOTATION_LINE_STRIP);
    cr_assert_float_eq(foxglove_point_annotations_get_thickness(ann), 3.0, 0.0001);

    foxglove_point_annotations_free(ann);
}

Test(foxglove_msgs, point_annotations_add_points) {
    FoxglovePointAnnotations *ann = foxglove_point_annotations_new();
    cr_assert_not_null(ann);

    // Add points
    FoxglovePoint2 *p1 = foxglove_point2_new();
    foxglove_point2_set_x(p1, 10.0);
    foxglove_point2_set_y(p1, 20.0);

    FoxglovePoint2 *p2 = foxglove_point2_new();
    foxglove_point2_set_x(p2, 30.0);
    foxglove_point2_set_y(p2, 40.0);

    int ret = foxglove_point_annotations_add_point(ann, p1);
    cr_assert_eq(ret, 0);

    ret = foxglove_point_annotations_add_point(ann, p2);
    cr_assert_eq(ret, 0);

    cr_assert_eq(foxglove_point_annotations_get_points_count(ann), 2);

    const FoxglovePoint2 *got = foxglove_point_annotations_get_point(ann, 0);
    cr_assert_not_null(got);
    cr_assert_float_eq(foxglove_point2_get_x(got), 10.0, 0.0001);

    const FoxglovePoint2 *got2 = foxglove_point_annotations_get_point(ann, 1);
    cr_assert_float_eq(foxglove_point2_get_x(got2), 30.0, 0.0001);

    // Clear points
    foxglove_point_annotations_clear_points(ann);
    cr_assert_eq(foxglove_point_annotations_get_points_count(ann), 0);

    foxglove_point2_free(p1);
    foxglove_point2_free(p2);
    foxglove_point_annotations_free(ann);
}

Test(foxglove_msgs, point_annotations_colors) {
    FoxglovePointAnnotations *ann = foxglove_point_annotations_new();
    cr_assert_not_null(ann);

    FoxgloveColor *outline = foxglove_point_annotations_get_outline_color_mut(ann);
    cr_assert_not_null(outline);
    foxglove_color_set_r(outline, 1.0);
    foxglove_color_set_g(outline, 0.0);
    foxglove_color_set_b(outline, 0.0);
    foxglove_color_set_a(outline, 1.0);

    FoxgloveColor *fill = foxglove_point_annotations_get_fill_color_mut(ann);
    cr_assert_not_null(fill);
    foxglove_color_set_r(fill, 0.0);
    foxglove_color_set_g(fill, 1.0);
    foxglove_color_set_b(fill, 0.0);
    foxglove_color_set_a(fill, 0.5);

    const FoxgloveColor *got_outline = foxglove_point_annotations_get_outline_color(ann);
    cr_assert_float_eq(foxglove_color_get_r(got_outline), 1.0, 0.0001);

    const FoxgloveColor *got_fill = foxglove_point_annotations_get_fill_color(ann);
    cr_assert_float_eq(foxglove_color_get_g(got_fill), 1.0, 0.0001);

    foxglove_point_annotations_free(ann);
}

Test(foxglove_msgs, point_annotations_free_null) {
    foxglove_point_annotations_free(NULL);
}

// ============================================================================
// TextAnnotations Tests
// ============================================================================

Test(foxglove_msgs, text_annotations_create_and_destroy) {
    FoxgloveTextAnnotations *ann = foxglove_text_annotations_new();
    cr_assert_not_null(ann);

    // Set timestamp
    RosTime *timestamp = foxglove_text_annotations_get_timestamp_mut(ann);
    cr_assert_not_null(timestamp);
    ros_time_set_sec(timestamp, 3000);

    // Set position
    FoxglovePoint2 *position = foxglove_text_annotations_get_position_mut(ann);
    cr_assert_not_null(position);
    foxglove_point2_set_x(position, 100.0);
    foxglove_point2_set_y(position, 50.0);

    // Set text
    int ret = foxglove_text_annotations_set_text(ann, "Detection: person");
    cr_assert_eq(ret, 0);

    // Set font size
    foxglove_text_annotations_set_font_size(ann, 14.0);

    // Verify
    char *text = foxglove_text_annotations_get_text(ann);
    cr_assert_str_eq(text, "Detection: person");
    free(text);

    cr_assert_float_eq(foxglove_text_annotations_get_font_size(ann), 14.0, 0.0001);

    const FoxglovePoint2 *pos = foxglove_text_annotations_get_position(ann);
    cr_assert_float_eq(foxglove_point2_get_x(pos), 100.0, 0.0001);

    foxglove_text_annotations_free(ann);
}

Test(foxglove_msgs, text_annotations_colors) {
    FoxgloveTextAnnotations *ann = foxglove_text_annotations_new();
    cr_assert_not_null(ann);

    FoxgloveColor *text_color = foxglove_text_annotations_get_text_color_mut(ann);
    cr_assert_not_null(text_color);
    foxglove_color_set_r(text_color, 1.0);
    foxglove_color_set_g(text_color, 1.0);
    foxglove_color_set_b(text_color, 1.0);
    foxglove_color_set_a(text_color, 1.0);

    FoxgloveColor *bg_color = foxglove_text_annotations_get_background_color_mut(ann);
    cr_assert_not_null(bg_color);
    foxglove_color_set_r(bg_color, 0.0);
    foxglove_color_set_g(bg_color, 0.0);
    foxglove_color_set_b(bg_color, 0.0);
    foxglove_color_set_a(bg_color, 0.7);

    const FoxgloveColor *got_text = foxglove_text_annotations_get_text_color(ann);
    cr_assert_float_eq(foxglove_color_get_r(got_text), 1.0, 0.0001);

    const FoxgloveColor *got_bg = foxglove_text_annotations_get_background_color(ann);
    cr_assert_float_eq(foxglove_color_get_a(got_bg), 0.7, 0.0001);

    foxglove_text_annotations_free(ann);
}

Test(foxglove_msgs, text_annotations_free_null) {
    foxglove_text_annotations_free(NULL);
}

// ============================================================================
// ImageAnnotations Tests
// ============================================================================

Test(foxglove_msgs, image_annotations_create_and_destroy) {
    FoxgloveImageAnnotations *ann = foxglove_image_annotations_new();
    cr_assert_not_null(ann);

    cr_assert_eq(foxglove_image_annotations_get_circles_count(ann), 0);
    cr_assert_eq(foxglove_image_annotations_get_points_count(ann), 0);
    cr_assert_eq(foxglove_image_annotations_get_texts_count(ann), 0);

    foxglove_image_annotations_free(ann);
}

Test(foxglove_msgs, image_annotations_add_circle) {
    FoxgloveImageAnnotations *ann = foxglove_image_annotations_new();
    cr_assert_not_null(ann);

    FoxgloveCircleAnnotations *circle = foxglove_circle_annotations_new();
    foxglove_circle_annotations_set_diameter(circle, 100.0);

    int ret = foxglove_image_annotations_add_circle(ann, circle);
    cr_assert_eq(ret, 0);

    cr_assert_eq(foxglove_image_annotations_get_circles_count(ann), 1);

    const FoxgloveCircleAnnotations *got = foxglove_image_annotations_get_circle(ann, 0);
    cr_assert_not_null(got);
    cr_assert_float_eq(foxglove_circle_annotations_get_diameter(got), 100.0, 0.0001);

    foxglove_image_annotations_clear_circles(ann);
    cr_assert_eq(foxglove_image_annotations_get_circles_count(ann), 0);

    foxglove_circle_annotations_free(circle);
    foxglove_image_annotations_free(ann);
}

Test(foxglove_msgs, image_annotations_add_point) {
    FoxgloveImageAnnotations *ann = foxglove_image_annotations_new();
    cr_assert_not_null(ann);

    FoxglovePointAnnotations *points = foxglove_point_annotations_new();
    foxglove_point_annotations_set_type(points, FOXGLOVE_POINT_ANNOTATION_POINTS);
    foxglove_point_annotations_set_thickness(points, 5.0);

    int ret = foxglove_image_annotations_add_point(ann, points);
    cr_assert_eq(ret, 0);

    cr_assert_eq(foxglove_image_annotations_get_points_count(ann), 1);

    const FoxglovePointAnnotations *got = foxglove_image_annotations_get_point(ann, 0);
    cr_assert_not_null(got);
    cr_assert_eq(foxglove_point_annotations_get_type(got), FOXGLOVE_POINT_ANNOTATION_POINTS);

    foxglove_image_annotations_clear_points(ann);
    cr_assert_eq(foxglove_image_annotations_get_points_count(ann), 0);

    foxglove_point_annotations_free(points);
    foxglove_image_annotations_free(ann);
}

Test(foxglove_msgs, image_annotations_add_text) {
    FoxgloveImageAnnotations *ann = foxglove_image_annotations_new();
    cr_assert_not_null(ann);

    FoxgloveTextAnnotations *text = foxglove_text_annotations_new();
    foxglove_text_annotations_set_text(text, "Label");
    foxglove_text_annotations_set_font_size(text, 12.0);

    int ret = foxglove_image_annotations_add_text(ann, text);
    cr_assert_eq(ret, 0);

    cr_assert_eq(foxglove_image_annotations_get_texts_count(ann), 1);

    const FoxgloveTextAnnotations *got = foxglove_image_annotations_get_text(ann, 0);
    cr_assert_not_null(got);
    cr_assert_float_eq(foxglove_text_annotations_get_font_size(got), 12.0, 0.0001);

    foxglove_image_annotations_clear_texts(ann);
    cr_assert_eq(foxglove_image_annotations_get_texts_count(ann), 0);

    foxglove_text_annotations_free(text);
    foxglove_image_annotations_free(ann);
}

Test(foxglove_msgs, image_annotations_serialize_deserialize) {
    FoxgloveImageAnnotations *original = foxglove_image_annotations_new();
    cr_assert_not_null(original);

    // Add a circle annotation
    FoxgloveCircleAnnotations *circle = foxglove_circle_annotations_new();
    foxglove_circle_annotations_set_diameter(circle, 75.0);
    foxglove_circle_annotations_set_thickness(circle, 3.0);
    FoxglovePoint2 *pos = foxglove_circle_annotations_get_position_mut(circle);
    foxglove_point2_set_x(pos, 150.0);
    foxglove_point2_set_y(pos, 200.0);
    foxglove_image_annotations_add_circle(original, circle);

    // Add a text annotation
    FoxgloveTextAnnotations *text = foxglove_text_annotations_new();
    foxglove_text_annotations_set_text(text, "Object");
    foxglove_text_annotations_set_font_size(text, 16.0);
    foxglove_image_annotations_add_text(original, text);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = foxglove_image_annotations_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);

    FoxgloveImageAnnotations *deserialized = foxglove_image_annotations_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_eq(foxglove_image_annotations_get_circles_count(deserialized), 1);
    cr_assert_eq(foxglove_image_annotations_get_texts_count(deserialized), 1);

    const FoxgloveCircleAnnotations *got_circle = foxglove_image_annotations_get_circle(deserialized, 0);
    cr_assert_float_eq(foxglove_circle_annotations_get_diameter(got_circle), 75.0, 0.0001);

    const FoxgloveTextAnnotations *got_text = foxglove_image_annotations_get_text(deserialized, 0);
    cr_assert_float_eq(foxglove_text_annotations_get_font_size(got_text), 16.0, 0.0001);

    foxglove_circle_annotations_free(circle);
    foxglove_text_annotations_free(text);
    foxglove_image_annotations_free(original);
    foxglove_image_annotations_free(deserialized);
    free(buffer);
}

Test(foxglove_msgs, image_annotations_free_null) {
    foxglove_image_annotations_free(NULL);
}
