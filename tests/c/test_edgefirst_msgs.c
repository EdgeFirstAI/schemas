/**
 * @file test_edgefirst_msgs.c
 * @brief Criterion tests for edgefirst_msgs types
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

// ============================================================================
// DetectTrack Tests
// ============================================================================

Test(edgefirst_msgs, detecttrack_create_and_destroy) {
    EdgeFirstDetectTrack *track = edgefirst_detecttrack_new();
    cr_assert_not_null(track);

    edgefirst_detecttrack_set_id(track, "42");
    edgefirst_detecttrack_set_lifetime(track, 10);

    char *id = edgefirst_detecttrack_get_id(track);
    cr_assert_str_eq(id, "42");
    free(id);

    cr_assert_eq(edgefirst_detecttrack_get_lifetime(track), 10);

    edgefirst_detecttrack_free(track);
}

Test(edgefirst_msgs, detecttrack_set_values) {
    EdgeFirstDetectTrack *track = edgefirst_detecttrack_new();
    cr_assert_not_null(track);

    edgefirst_detecttrack_set_id(track, "test-track-999");
    edgefirst_detecttrack_set_lifetime(track, 5);

    char *id = edgefirst_detecttrack_get_id(track);
    cr_assert_str_eq(id, "test-track-999");
    free(id);

    cr_assert_eq(edgefirst_detecttrack_get_lifetime(track), 5);

    edgefirst_detecttrack_free(track);
}

Test(edgefirst_msgs, detecttrack_serialize_deserialize) {
    EdgeFirstDetectTrack *original = edgefirst_detecttrack_new();
    cr_assert_not_null(original);

    edgefirst_detecttrack_set_id(original, "track-42");
    edgefirst_detecttrack_set_lifetime(original, 100);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = edgefirst_detecttrack_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);

    EdgeFirstDetectTrack *deserialized = edgefirst_detecttrack_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    char *id = edgefirst_detecttrack_get_id(deserialized);
    cr_assert_str_eq(id, "track-42");
    free(id);

    cr_assert_eq(edgefirst_detecttrack_get_lifetime(deserialized), 100);

    edgefirst_detecttrack_free(original);
    edgefirst_detecttrack_free(deserialized);
    free(buffer);
}

Test(edgefirst_msgs, detecttrack_free_null) {
    edgefirst_detecttrack_free(NULL);
}

// ============================================================================
// DetectBox2D Tests
// ============================================================================

Test(edgefirst_msgs, detectbox2d_create_and_destroy) {
    EdgeFirstDetectBox2D *box = edgefirst_detectbox2d_new();
    cr_assert_not_null(box);

    edgefirst_detectbox2d_set_center_x(box, 0.5f);
    edgefirst_detectbox2d_set_center_y(box, 0.5f);
    edgefirst_detectbox2d_set_width(box, 0.1f);
    edgefirst_detectbox2d_set_height(box, 0.2f);

    cr_assert_float_eq(edgefirst_detectbox2d_get_center_x(box), 0.5f, 0.0001f);
    cr_assert_float_eq(edgefirst_detectbox2d_get_center_y(box), 0.5f, 0.0001f);
    cr_assert_float_eq(edgefirst_detectbox2d_get_width(box), 0.1f, 0.0001f);
    cr_assert_float_eq(edgefirst_detectbox2d_get_height(box), 0.2f, 0.0001f);

    edgefirst_detectbox2d_free(box);
}

Test(edgefirst_msgs, detectbox2d_label_and_score) {
    EdgeFirstDetectBox2D *box = edgefirst_detectbox2d_new();
    cr_assert_not_null(box);

    edgefirst_detectbox2d_set_label(box, "person");
    edgefirst_detectbox2d_set_score(box, 0.95f);

    char *label = edgefirst_detectbox2d_get_label(box);
    cr_assert_str_eq(label, "person");
    free(label);

    cr_assert_float_eq(edgefirst_detectbox2d_get_score(box), 0.95f, 0.0001f);

    edgefirst_detectbox2d_free(box);
}

Test(edgefirst_msgs, detectbox2d_distance_and_speed) {
    EdgeFirstDetectBox2D *box = edgefirst_detectbox2d_new();
    cr_assert_not_null(box);

    edgefirst_detectbox2d_set_distance(box, 15.5f);
    edgefirst_detectbox2d_set_speed(box, 5.2f);

    cr_assert_float_eq(edgefirst_detectbox2d_get_distance(box), 15.5f, 0.0001f);
    cr_assert_float_eq(edgefirst_detectbox2d_get_speed(box), 5.2f, 0.0001f);

    edgefirst_detectbox2d_free(box);
}

Test(edgefirst_msgs, detectbox2d_serialize_deserialize) {
    EdgeFirstDetectBox2D *original = edgefirst_detectbox2d_new();
    cr_assert_not_null(original);

    edgefirst_detectbox2d_set_center_x(original, 0.25f);
    edgefirst_detectbox2d_set_center_y(original, 0.75f);
    edgefirst_detectbox2d_set_width(original, 0.15f);
    edgefirst_detectbox2d_set_height(original, 0.35f);
    edgefirst_detectbox2d_set_label(original, "car");
    edgefirst_detectbox2d_set_score(original, 0.87f);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = edgefirst_detectbox2d_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    EdgeFirstDetectBox2D *deserialized = edgefirst_detectbox2d_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_float_eq(edgefirst_detectbox2d_get_center_x(deserialized), 0.25f, 0.0001f);
    cr_assert_float_eq(edgefirst_detectbox2d_get_center_y(deserialized), 0.75f, 0.0001f);

    char *label = edgefirst_detectbox2d_get_label(deserialized);
    cr_assert_str_eq(label, "car");
    free(label);

    edgefirst_detectbox2d_free(original);
    edgefirst_detectbox2d_free(deserialized);
    free(buffer);
}

Test(edgefirst_msgs, detectbox2d_free_null) {
    edgefirst_detectbox2d_free(NULL);
}

// ============================================================================
// Detect Tests
// ============================================================================

Test(edgefirst_msgs, detect_create_and_destroy) {
    EdgeFirstDetect *detect = edgefirst_detect_new();
    cr_assert_not_null(detect);

    RosHeader *header = edgefirst_detect_get_header_mut(detect);
    cr_assert_not_null(header);

    edgefirst_detect_free(detect);
}

Test(edgefirst_msgs, detect_add_box) {
    EdgeFirstDetect *detect = edgefirst_detect_new();
    cr_assert_not_null(detect);

    EdgeFirstDetectBox2D *box = edgefirst_detectbox2d_new();
    edgefirst_detectbox2d_set_label(box, "pedestrian");
    edgefirst_detectbox2d_set_score(box, 0.9f);

    int ret = edgefirst_detect_add_box(detect, box);
    cr_assert_eq(ret, 0);

    size_t box_count = 0;
    const EdgeFirstDetectBox2D *boxes = edgefirst_detect_get_boxes(detect, &box_count);
    cr_assert_eq(box_count, 1);
    cr_assert_not_null(boxes);

    edgefirst_detectbox2d_free(box);
    edgefirst_detect_free(detect);
}

Test(edgefirst_msgs, detect_serialize_deserialize) {
    EdgeFirstDetect *original = edgefirst_detect_new();
    cr_assert_not_null(original);

    RosHeader *header = edgefirst_detect_get_header_mut(original);
    ros_header_set_frame_id(header, "camera_front");

    EdgeFirstDetectBox2D *box = edgefirst_detectbox2d_new();
    edgefirst_detectbox2d_set_label(box, "bicycle");
    edgefirst_detectbox2d_set_score(box, 0.75f);
    edgefirst_detect_add_box(original, box);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = edgefirst_detect_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    EdgeFirstDetect *deserialized = edgefirst_detect_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    size_t box_count = 0;
    edgefirst_detect_get_boxes(deserialized, &box_count);
    cr_assert_eq(box_count, 1);

    edgefirst_detectbox2d_free(box);
    edgefirst_detect_free(original);
    edgefirst_detect_free(deserialized);
    free(buffer);
}

Test(edgefirst_msgs, detect_free_null) {
    edgefirst_detect_free(NULL);
}

// ============================================================================
// Mask Tests
// ============================================================================

Test(edgefirst_msgs, mask_create_and_destroy) {
    EdgeFirstMask *mask = edgefirst_mask_new();
    cr_assert_not_null(mask);

    edgefirst_mask_set_height(mask, 480);
    edgefirst_mask_set_width(mask, 640);
    edgefirst_mask_set_boxed(mask, true);

    cr_assert_eq(edgefirst_mask_get_height(mask), 480);
    cr_assert_eq(edgefirst_mask_get_width(mask), 640);
    cr_assert_eq(edgefirst_mask_get_boxed(mask), true);

    edgefirst_mask_free(mask);
}

Test(edgefirst_msgs, mask_set_encoding) {
    EdgeFirstMask *mask = edgefirst_mask_new();
    cr_assert_not_null(mask);

    int ret = edgefirst_mask_set_encoding(mask, "rle");
    cr_assert_eq(ret, 0);

    char *encoding = edgefirst_mask_get_encoding(mask);
    cr_assert_str_eq(encoding, "rle");
    free(encoding);

    edgefirst_mask_free(mask);
}

Test(edgefirst_msgs, mask_set_data) {
    EdgeFirstMask *mask = edgefirst_mask_new();
    cr_assert_not_null(mask);

    uint8_t data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    int ret = edgefirst_mask_set_mask(mask, data, sizeof(data));
    cr_assert_eq(ret, 0);

    size_t len = 0;
    const uint8_t *mask_data = edgefirst_mask_get_mask(mask, &len);
    cr_assert_eq(len, sizeof(data));
    for (size_t i = 0; i < len; i++) {
        cr_assert_eq(mask_data[i], data[i]);
    }

    edgefirst_mask_free(mask);
}

Test(edgefirst_msgs, mask_serialize_deserialize) {
    EdgeFirstMask *original = edgefirst_mask_new();
    cr_assert_not_null(original);

    edgefirst_mask_set_height(original, 100);
    edgefirst_mask_set_width(original, 200);
    edgefirst_mask_set_encoding(original, "raw");
    edgefirst_mask_set_boxed(original, false);

    uint8_t data[] = {0xFF, 0x00, 0xFF, 0x00};
    edgefirst_mask_set_mask(original, data, sizeof(data));

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = edgefirst_mask_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    EdgeFirstMask *deserialized = edgefirst_mask_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_eq(edgefirst_mask_get_height(deserialized), 100);
    cr_assert_eq(edgefirst_mask_get_width(deserialized), 200);

    edgefirst_mask_free(original);
    edgefirst_mask_free(deserialized);
    free(buffer);
}

Test(edgefirst_msgs, mask_free_null) {
    edgefirst_mask_free(NULL);
}

// ============================================================================
// DmaBuf Tests
// ============================================================================

Test(edgefirst_msgs, dmabuf_create_and_destroy) {
    EdgeFirstDmaBuf *dmabuf = edgefirst_dmabuf_new();
    cr_assert_not_null(dmabuf);

    edgefirst_dmabuf_set_pid(dmabuf, 1234);
    edgefirst_dmabuf_set_fd(dmabuf, 5);
    edgefirst_dmabuf_set_width(dmabuf, 1920);
    edgefirst_dmabuf_set_height(dmabuf, 1080);

    cr_assert_eq(edgefirst_dmabuf_get_pid(dmabuf), 1234);
    cr_assert_eq(edgefirst_dmabuf_get_fd(dmabuf), 5);
    cr_assert_eq(edgefirst_dmabuf_get_width(dmabuf), 1920);
    cr_assert_eq(edgefirst_dmabuf_get_height(dmabuf), 1080);

    edgefirst_dmabuf_free(dmabuf);
}

Test(edgefirst_msgs, dmabuf_stride_and_fourcc) {
    EdgeFirstDmaBuf *dmabuf = edgefirst_dmabuf_new();
    cr_assert_not_null(dmabuf);

    edgefirst_dmabuf_set_stride(dmabuf, 7680);
    edgefirst_dmabuf_set_fourcc(dmabuf, 0x56595559); // YUYV

    cr_assert_eq(edgefirst_dmabuf_get_stride(dmabuf), 7680);
    cr_assert_eq(edgefirst_dmabuf_get_fourcc(dmabuf), 0x56595559);

    edgefirst_dmabuf_free(dmabuf);
}

Test(edgefirst_msgs, dmabuf_serialize_deserialize) {
    EdgeFirstDmaBuf *original = edgefirst_dmabuf_new();
    cr_assert_not_null(original);

    edgefirst_dmabuf_set_pid(original, 5678);
    edgefirst_dmabuf_set_fd(original, 10);
    edgefirst_dmabuf_set_width(original, 640);
    edgefirst_dmabuf_set_height(original, 480);
    edgefirst_dmabuf_set_stride(original, 1280);
    edgefirst_dmabuf_set_length(original, 614400);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = edgefirst_dmabuf_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    EdgeFirstDmaBuf *deserialized = edgefirst_dmabuf_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_eq(edgefirst_dmabuf_get_pid(deserialized), 5678);
    cr_assert_eq(edgefirst_dmabuf_get_fd(deserialized), 10);
    cr_assert_eq(edgefirst_dmabuf_get_width(deserialized), 640);
    cr_assert_eq(edgefirst_dmabuf_get_height(deserialized), 480);

    edgefirst_dmabuf_free(original);
    edgefirst_dmabuf_free(deserialized);
    free(buffer);
}

Test(edgefirst_msgs, dmabuf_free_null) {
    edgefirst_dmabuf_free(NULL);
}

// ============================================================================
// RadarCube Tests (with Khronos-style serialize)
// ============================================================================

Test(edgefirst_msgs, radarcube_create_and_destroy) {
    EdgeFirstRadarCube *cube = edgefirst_radarcube_new();
    cr_assert_not_null(cube);

    RosHeader *header = edgefirst_radarcube_get_header_mut(cube);
    cr_assert_not_null(header);

    edgefirst_radarcube_set_timestamp(cube, 123456789ULL);
    cr_assert_eq(edgefirst_radarcube_get_timestamp(cube), 123456789ULL);

    edgefirst_radarcube_free(cube);
}

Test(edgefirst_msgs, radarcube_layout_and_shape) {
    EdgeFirstRadarCube *cube = edgefirst_radarcube_new();
    cr_assert_not_null(cube);

    uint8_t layout[] = {
        EDGEFIRST_RADAR_CUBE_DIMENSION_RANGE,
        EDGEFIRST_RADAR_CUBE_DIMENSION_DOPPLER,
        EDGEFIRST_RADAR_CUBE_DIMENSION_AZIMUTH
    };
    edgefirst_radarcube_set_layout(cube, layout, 3);

    uint16_t shape[] = {128, 64, 32};
    edgefirst_radarcube_set_shape(cube, shape, 3);

    size_t layout_len = 0;
    const uint8_t *got_layout = edgefirst_radarcube_get_layout(cube, &layout_len);
    cr_assert_eq(layout_len, 3);
    cr_assert_eq(got_layout[0], EDGEFIRST_RADAR_CUBE_DIMENSION_RANGE);
    cr_assert_eq(got_layout[1], EDGEFIRST_RADAR_CUBE_DIMENSION_DOPPLER);
    cr_assert_eq(got_layout[2], EDGEFIRST_RADAR_CUBE_DIMENSION_AZIMUTH);

    size_t shape_len = 0;
    const uint16_t *got_shape = edgefirst_radarcube_get_shape(cube, &shape_len);
    cr_assert_eq(shape_len, 3);
    cr_assert_eq(got_shape[0], 128);
    cr_assert_eq(got_shape[1], 64);
    cr_assert_eq(got_shape[2], 32);

    edgefirst_radarcube_free(cube);
}

Test(edgefirst_msgs, radarcube_serialize_empty) {
    EdgeFirstRadarCube *cube = edgefirst_radarcube_new();
    cr_assert_not_null(cube);

    // Query required size (Khronos pattern: buffer=NULL)
    size_t required_size = 0;
    int ret = edgefirst_radarcube_serialize(cube, NULL, 0, &required_size);
    cr_assert_eq(ret, 0, "Size query should succeed");
    cr_assert_gt(required_size, 0, "Required size should be > 0");

    // Allocate and serialize
    uint8_t *buffer = (uint8_t *)malloc(required_size);
    cr_assert_not_null(buffer);
    ret = edgefirst_radarcube_serialize(cube, buffer, required_size, NULL);
    cr_assert_eq(ret, 0, "Serialization should succeed");

    free(buffer);
    edgefirst_radarcube_free(cube);
}

Test(edgefirst_msgs, radarcube_serialize_deserialize_roundtrip) {
    EdgeFirstRadarCube *original = edgefirst_radarcube_new();
    cr_assert_not_null(original);

    // Set up realistic radar cube
    RosHeader *header = edgefirst_radarcube_get_header_mut(original);
    RosTime *stamp = ros_header_get_stamp_mut(header);
    ros_time_set_sec(stamp, 1234567890);
    ros_time_set_nanosec(stamp, 123456789);
    ros_header_set_frame_id(header, "radar_front");

    edgefirst_radarcube_set_timestamp(original, 9876543210ULL);

    uint8_t layout[] = {
        EDGEFIRST_RADAR_CUBE_DIMENSION_RANGE,
        EDGEFIRST_RADAR_CUBE_DIMENSION_DOPPLER
    };
    edgefirst_radarcube_set_layout(original, layout, 2);

    uint16_t shape[] = {64, 32};
    edgefirst_radarcube_set_shape(original, shape, 2);

    float scales[] = {1.5f, 0.1f};
    edgefirst_radarcube_set_scales(original, scales, 2);

    // Small complex cube (real/imag pairs)
    int16_t cube_data[] = {100, 50, -100, -50, 200, 100, -200, -100};
    edgefirst_radarcube_set_cube(original, cube_data, 8);
    edgefirst_radarcube_set_is_complex(original, true);

    // Serialize using Khronos pattern
    size_t required_size = 0;
    int ret = edgefirst_radarcube_serialize(original, NULL, 0, &required_size);
    cr_assert_eq(ret, 0);
    cr_assert_gt(required_size, 0);

    uint8_t *buffer = (uint8_t *)malloc(required_size);
    cr_assert_not_null(buffer);
    ret = edgefirst_radarcube_serialize(original, buffer, required_size, NULL);
    cr_assert_eq(ret, 0);

    // Deserialize
    EdgeFirstRadarCube *deserialized = edgefirst_radarcube_deserialize(buffer, required_size);
    cr_assert_not_null(deserialized);

    // Verify header
    const RosHeader *deser_header = edgefirst_radarcube_get_header(deserialized);
    const RosTime *deser_stamp = ros_header_get_stamp(deser_header);
    cr_assert_eq(ros_time_get_sec(deser_stamp), 1234567890);
    cr_assert_eq(ros_time_get_nanosec(deser_stamp), 123456789);

    char *frame_id = ros_header_get_frame_id(deser_header);
    cr_assert_str_eq(frame_id, "radar_front");
    free(frame_id);

    // Verify timestamp
    cr_assert_eq(edgefirst_radarcube_get_timestamp(deserialized), 9876543210ULL);

    // Verify layout
    size_t layout_len = 0;
    const uint8_t *got_layout = edgefirst_radarcube_get_layout(deserialized, &layout_len);
    cr_assert_eq(layout_len, 2);
    cr_assert_eq(got_layout[0], EDGEFIRST_RADAR_CUBE_DIMENSION_RANGE);
    cr_assert_eq(got_layout[1], EDGEFIRST_RADAR_CUBE_DIMENSION_DOPPLER);

    // Verify is_complex
    cr_assert_eq(edgefirst_radarcube_get_is_complex(deserialized), true);

    free(buffer);
    edgefirst_radarcube_free(original);
    edgefirst_radarcube_free(deserialized);
}

Test(edgefirst_msgs, radarcube_serialize_buffer_too_small) {
    EdgeFirstRadarCube *cube = edgefirst_radarcube_new();
    cr_assert_not_null(cube);

    // Query required size
    size_t required_size = 0;
    int ret = edgefirst_radarcube_serialize(cube, NULL, 0, &required_size);
    cr_assert_eq(ret, 0);
    cr_assert_gt(required_size, 0);

    // Try with buffer too small
    uint8_t small_buffer[4];
    size_t actual_size = 0;
    errno = 0;
    ret = edgefirst_radarcube_serialize(cube, small_buffer, sizeof(small_buffer), &actual_size);
    cr_assert_eq(ret, -1, "Should fail with small buffer");
    cr_assert_eq(errno, ENOBUFS, "errno should be ENOBUFS");
    cr_assert_eq(actual_size, required_size, "Should still report required size");

    edgefirst_radarcube_free(cube);
}

Test(edgefirst_msgs, radarcube_free_null) {
    edgefirst_radarcube_free(NULL);
}

// ============================================================================
// Date Tests
// ============================================================================

Test(edgefirst_msgs, date_create_and_destroy) {
    EdgeFirstDate *date = edgefirst_date_new();
    cr_assert_not_null(date);

    edgefirst_date_set_year(date, 2025);
    edgefirst_date_set_month(date, 6);
    edgefirst_date_set_day(date, 15);

    cr_assert_eq(edgefirst_date_get_year(date), 2025);
    cr_assert_eq(edgefirst_date_get_month(date), 6);
    cr_assert_eq(edgefirst_date_get_day(date), 15);

    edgefirst_date_free(date);
}

Test(edgefirst_msgs, date_serialize_deserialize) {
    EdgeFirstDate *original = edgefirst_date_new();
    cr_assert_not_null(original);

    edgefirst_date_set_year(original, 2024);
    edgefirst_date_set_month(original, 12);
    edgefirst_date_set_day(original, 25);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = edgefirst_date_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    EdgeFirstDate *deserialized = edgefirst_date_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_eq(edgefirst_date_get_year(deserialized), 2024);
    cr_assert_eq(edgefirst_date_get_month(deserialized), 12);
    cr_assert_eq(edgefirst_date_get_day(deserialized), 25);

    edgefirst_date_free(original);
    edgefirst_date_free(deserialized);
    free(buffer);
}

Test(edgefirst_msgs, date_free_null) {
    edgefirst_date_free(NULL);
}

// ============================================================================
// LocalTime Tests
// ============================================================================

Test(edgefirst_msgs, local_time_create_and_destroy) {
    EdgeFirstLocalTime *lt = edgefirst_local_time_new();
    cr_assert_not_null(lt);

    RosHeader *header = edgefirst_local_time_get_header_mut(lt);
    cr_assert_not_null(header);
    ros_header_set_frame_id(header, "gps");

    EdgeFirstDate *date = edgefirst_local_time_get_date_mut(lt);
    cr_assert_not_null(date);
    edgefirst_date_set_year(date, 2025);
    edgefirst_date_set_month(date, 1);
    edgefirst_date_set_day(date, 1);

    RosTime *time = edgefirst_local_time_get_time_mut(lt);
    cr_assert_not_null(time);
    ros_time_set_sec(time, 43200);  // 12:00:00
    ros_time_set_nanosec(time, 0);

    edgefirst_local_time_set_timezone(lt, -300);  // UTC-5

    cr_assert_eq(edgefirst_local_time_get_timezone(lt), -300);

    const EdgeFirstDate *date_const = edgefirst_local_time_get_date(lt);
    cr_assert_eq(edgefirst_date_get_year(date_const), 2025);

    const RosTime *time_const = edgefirst_local_time_get_time(lt);
    cr_assert_eq(ros_time_get_sec(time_const), 43200);

    edgefirst_local_time_free(lt);
}

Test(edgefirst_msgs, local_time_serialize_deserialize) {
    EdgeFirstLocalTime *original = edgefirst_local_time_new();
    cr_assert_not_null(original);

    EdgeFirstDate *date = edgefirst_local_time_get_date_mut(original);
    edgefirst_date_set_year(date, 2025);
    edgefirst_date_set_month(date, 6);
    edgefirst_date_set_day(date, 15);

    RosTime *time = edgefirst_local_time_get_time_mut(original);
    ros_time_set_sec(time, 50400);  // 14:00:00
    ros_time_set_nanosec(time, 500000000);

    edgefirst_local_time_set_timezone(original, 60);  // UTC+1

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = edgefirst_local_time_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    EdgeFirstLocalTime *deserialized = edgefirst_local_time_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    const EdgeFirstDate *deser_date = edgefirst_local_time_get_date(deserialized);
    cr_assert_eq(edgefirst_date_get_year(deser_date), 2025);
    cr_assert_eq(edgefirst_date_get_month(deser_date), 6);

    cr_assert_eq(edgefirst_local_time_get_timezone(deserialized), 60);

    edgefirst_local_time_free(original);
    edgefirst_local_time_free(deserialized);
    free(buffer);
}

Test(edgefirst_msgs, local_time_free_null) {
    edgefirst_local_time_free(NULL);
}

// ============================================================================
// RadarInfo Tests
// ============================================================================

Test(edgefirst_msgs, radar_info_create_and_destroy) {
    EdgeFirstRadarInfo *info = edgefirst_radar_info_new();
    cr_assert_not_null(info);

    RosHeader *header = edgefirst_radar_info_get_header_mut(info);
    cr_assert_not_null(header);
    ros_header_set_frame_id(header, "radar_front");

    edgefirst_radar_info_set_center_frequency(info, "77GHz");
    edgefirst_radar_info_set_frequency_sweep(info, "FMCW");
    edgefirst_radar_info_set_range_toggle(info, "long");
    edgefirst_radar_info_set_detection_sensitivity(info, "high");
    edgefirst_radar_info_set_cube(info, true);

    char *center_freq = edgefirst_radar_info_get_center_frequency(info);
    cr_assert_str_eq(center_freq, "77GHz");
    free(center_freq);

    cr_assert_eq(edgefirst_radar_info_get_cube(info), true);

    edgefirst_radar_info_free(info);
}

Test(edgefirst_msgs, radar_info_serialize_deserialize) {
    EdgeFirstRadarInfo *original = edgefirst_radar_info_new();
    cr_assert_not_null(original);

    RosHeader *header = edgefirst_radar_info_get_header_mut(original);
    ros_header_set_frame_id(header, "radar0");

    edgefirst_radar_info_set_center_frequency(original, "79GHz");
    edgefirst_radar_info_set_frequency_sweep(original, "linear");
    edgefirst_radar_info_set_range_toggle(original, "short");
    edgefirst_radar_info_set_detection_sensitivity(original, "medium");
    edgefirst_radar_info_set_cube(original, false);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = edgefirst_radar_info_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    EdgeFirstRadarInfo *deserialized = edgefirst_radar_info_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    char *center_freq = edgefirst_radar_info_get_center_frequency(deserialized);
    cr_assert_str_eq(center_freq, "79GHz");
    free(center_freq);

    cr_assert_eq(edgefirst_radar_info_get_cube(deserialized), false);

    edgefirst_radar_info_free(original);
    edgefirst_radar_info_free(deserialized);
    free(buffer);
}

Test(edgefirst_msgs, radar_info_free_null) {
    edgefirst_radar_info_free(NULL);
}

// ============================================================================
// Model Tests
// ============================================================================

Test(edgefirst_msgs, model_create_and_destroy) {
    EdgeFirstModel *model = edgefirst_model_new();
    cr_assert_not_null(model);

    RosHeader *header = edgefirst_model_get_header_mut(model);
    cr_assert_not_null(header);
    ros_header_set_frame_id(header, "model_output");

    // Access timing fields
    RosDuration *input_time = edgefirst_model_get_input_time_mut(model);
    cr_assert_not_null(input_time);
    ros_duration_set_sec(input_time, 0);
    ros_duration_set_nanosec(input_time, 5000000);  // 5ms

    RosDuration *model_time = edgefirst_model_get_model_time_mut(model);
    cr_assert_not_null(model_time);
    ros_duration_set_sec(model_time, 0);
    ros_duration_set_nanosec(model_time, 10000000);  // 10ms

    cr_assert_eq(edgefirst_model_get_boxes_count(model), 0);
    cr_assert_eq(edgefirst_model_get_masks_count(model), 0);

    edgefirst_model_free(model);
}

Test(edgefirst_msgs, model_add_boxes) {
    EdgeFirstModel *model = edgefirst_model_new();
    cr_assert_not_null(model);

    // Add a detection box
    EdgeFirstDetectBox2D *box = edgefirst_detectbox2d_new();
    edgefirst_detectbox2d_set_label(box, "person");
    edgefirst_detectbox2d_set_score(box, 0.95f);
    edgefirst_detectbox2d_set_center_x(box, 0.5f);
    edgefirst_detectbox2d_set_center_y(box, 0.5f);

    int ret = edgefirst_model_add_box(model, box);
    cr_assert_eq(ret, 0);

    cr_assert_eq(edgefirst_model_get_boxes_count(model), 1);

    const EdgeFirstDetectBox2D *got_box = edgefirst_model_get_box(model, 0);
    cr_assert_not_null(got_box);
    cr_assert_float_eq(edgefirst_detectbox2d_get_score(got_box), 0.95f, 0.0001f);

    // Clear boxes
    edgefirst_model_clear_boxes(model);
    cr_assert_eq(edgefirst_model_get_boxes_count(model), 0);

    edgefirst_detectbox2d_free(box);
    edgefirst_model_free(model);
}

Test(edgefirst_msgs, model_add_masks) {
    EdgeFirstModel *model = edgefirst_model_new();
    cr_assert_not_null(model);

    // Add a mask
    EdgeFirstMask *mask = edgefirst_mask_new();
    edgefirst_mask_set_height(mask, 480);
    edgefirst_mask_set_width(mask, 640);
    edgefirst_mask_set_encoding(mask, "rle");

    int ret = edgefirst_model_add_mask(model, mask);
    cr_assert_eq(ret, 0);

    cr_assert_eq(edgefirst_model_get_masks_count(model), 1);

    const EdgeFirstMask *got_mask = edgefirst_model_get_mask(model, 0);
    cr_assert_not_null(got_mask);
    cr_assert_eq(edgefirst_mask_get_height(got_mask), 480);

    // Clear masks
    edgefirst_model_clear_masks(model);
    cr_assert_eq(edgefirst_model_get_masks_count(model), 0);

    edgefirst_mask_free(mask);
    edgefirst_model_free(model);
}

Test(edgefirst_msgs, model_serialize_deserialize) {
    EdgeFirstModel *original = edgefirst_model_new();
    cr_assert_not_null(original);

    RosHeader *header = edgefirst_model_get_header_mut(original);
    ros_header_set_frame_id(header, "yolov8");

    // Add a box
    EdgeFirstDetectBox2D *box = edgefirst_detectbox2d_new();
    edgefirst_detectbox2d_set_label(box, "car");
    edgefirst_detectbox2d_set_score(box, 0.87f);
    edgefirst_model_add_box(original, box);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = edgefirst_model_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    EdgeFirstModel *deserialized = edgefirst_model_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_eq(edgefirst_model_get_boxes_count(deserialized), 1);

    const EdgeFirstDetectBox2D *deser_box = edgefirst_model_get_box(deserialized, 0);
    cr_assert_float_eq(edgefirst_detectbox2d_get_score(deser_box), 0.87f, 0.0001f);

    edgefirst_detectbox2d_free(box);
    edgefirst_model_free(original);
    edgefirst_model_free(deserialized);
    free(buffer);
}

Test(edgefirst_msgs, model_free_null) {
    edgefirst_model_free(NULL);
}

// ============================================================================
// ModelInfo Tests
// ============================================================================

Test(edgefirst_msgs, model_info_create_and_destroy) {
    EdgeFirstModelInfo *info = edgefirst_model_info_new();
    cr_assert_not_null(info);

    RosHeader *header = edgefirst_model_info_get_header_mut(info);
    cr_assert_not_null(header);
    ros_header_set_frame_id(header, "model_info");

    edgefirst_model_info_set_model_name(info, "yolov8n");
    edgefirst_model_info_set_model_type(info, "detection");
    edgefirst_model_info_set_model_format(info, "tflite");

    char *name = edgefirst_model_info_get_model_name(info);
    cr_assert_str_eq(name, "yolov8n");
    free(name);

    char *type = edgefirst_model_info_get_model_type(info);
    cr_assert_str_eq(type, "detection");
    free(type);

    char *format = edgefirst_model_info_get_model_format(info);
    cr_assert_str_eq(format, "tflite");
    free(format);

    edgefirst_model_info_free(info);
}

Test(edgefirst_msgs, model_info_set_shapes) {
    EdgeFirstModelInfo *info = edgefirst_model_info_new();
    cr_assert_not_null(info);

    uint32_t input_shape[] = {1, 640, 640, 3};
    int ret = edgefirst_model_info_set_input_shape(info, input_shape, 4);
    cr_assert_eq(ret, 0);

    edgefirst_model_info_set_input_type(info, EDGEFIRST_MODEL_INFO_UINT8);

    uint32_t output_shape[] = {1, 25200, 85};
    ret = edgefirst_model_info_set_output_shape(info, output_shape, 3);
    cr_assert_eq(ret, 0);

    edgefirst_model_info_set_output_type(info, EDGEFIRST_MODEL_INFO_FLOAT32);

    size_t shape_len = 0;
    const uint32_t *got_input = edgefirst_model_info_get_input_shape(info, &shape_len);
    cr_assert_eq(shape_len, 4);
    cr_assert_eq(got_input[1], 640);

    cr_assert_eq(edgefirst_model_info_get_input_type(info), EDGEFIRST_MODEL_INFO_UINT8);
    cr_assert_eq(edgefirst_model_info_get_output_type(info), EDGEFIRST_MODEL_INFO_FLOAT32);

    edgefirst_model_info_free(info);
}

Test(edgefirst_msgs, model_info_labels) {
    EdgeFirstModelInfo *info = edgefirst_model_info_new();
    cr_assert_not_null(info);

    int ret = edgefirst_model_info_add_label(info, "person");
    cr_assert_eq(ret, 0);

    ret = edgefirst_model_info_add_label(info, "car");
    cr_assert_eq(ret, 0);

    ret = edgefirst_model_info_add_label(info, "bicycle");
    cr_assert_eq(ret, 0);

    cr_assert_eq(edgefirst_model_info_get_labels_count(info), 3);

    char *label0 = edgefirst_model_info_get_label(info, 0);
    cr_assert_str_eq(label0, "person");
    free(label0);

    char *label2 = edgefirst_model_info_get_label(info, 2);
    cr_assert_str_eq(label2, "bicycle");
    free(label2);

    edgefirst_model_info_clear_labels(info);
    cr_assert_eq(edgefirst_model_info_get_labels_count(info), 0);

    edgefirst_model_info_free(info);
}

Test(edgefirst_msgs, model_info_serialize_deserialize) {
    EdgeFirstModelInfo *original = edgefirst_model_info_new();
    cr_assert_not_null(original);

    edgefirst_model_info_set_model_name(original, "ssd_mobilenet");
    edgefirst_model_info_set_model_type(original, "object_detection");
    edgefirst_model_info_set_model_format(original, "onnx");

    uint32_t input_shape[] = {1, 300, 300, 3};
    edgefirst_model_info_set_input_shape(original, input_shape, 4);
    edgefirst_model_info_set_input_type(original, EDGEFIRST_MODEL_INFO_FLOAT32);

    edgefirst_model_info_add_label(original, "background");
    edgefirst_model_info_add_label(original, "person");

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = edgefirst_model_info_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    EdgeFirstModelInfo *deserialized = edgefirst_model_info_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    char *name = edgefirst_model_info_get_model_name(deserialized);
    cr_assert_str_eq(name, "ssd_mobilenet");
    free(name);

    cr_assert_eq(edgefirst_model_info_get_labels_count(deserialized), 2);

    char *label1 = edgefirst_model_info_get_label(deserialized, 1);
    cr_assert_str_eq(label1, "person");
    free(label1);

    edgefirst_model_info_free(original);
    edgefirst_model_info_free(deserialized);
    free(buffer);
}

Test(edgefirst_msgs, model_info_free_null) {
    edgefirst_model_info_free(NULL);
}
