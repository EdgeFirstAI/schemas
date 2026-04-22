/**
 * @file test_edgefirst_msgs.c
 * @brief Criterion tests for edgefirst_msgs types
 *
 * Buffer-backed types: Track, Box (DetectBox), Detect, Mask, DmaBuffer,
 *   RadarCube, RadarInfo, Model, ModelInfo, LocalTime
 *
 * Types with encode: Mask, DmaBuffer
 * Types without encode: Track, Box, Detect, RadarCube, RadarInfo,
 *   Model, ModelInfo, LocalTime
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

// Load a golden CDR fixture from testdata/ at runtime. Returns a malloc'd
// buffer (caller frees) and writes length via *out_len. Prefer this to
// hex-embedding additional CameraFrame variants — keeps the C suite in
// lock-step with the Python generator without bloating the test source.
static uint8_t *load_fixture(const char *relpath, size_t *out_len) {
    FILE *f = fopen(relpath, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return NULL; }
    uint8_t *buf = (uint8_t *) malloc((size_t) sz);
    if (!buf) { fclose(f); return NULL; }
    size_t got = fread(buf, 1, (size_t) sz, f);
    fclose(f);
    if (got != (size_t) sz) { free(buf); return NULL; }
    *out_len = got;
    return buf;
}

// ============================================================================
// Track Tests (buffer-backed — no encode)
// ============================================================================

Test(edgefirst_msgs, track_from_cdr_null) {
    errno = 0;
    ros_track_t *handle = ros_track_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, track_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_track_t *handle = ros_track_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(edgefirst_msgs, track_free_null) {
    ros_track_free(NULL);
}

Test(edgefirst_msgs, track_getters_null) {
    cr_assert_null(ros_track_get_id(NULL));
    cr_assert_eq(ros_track_get_lifetime(NULL), 0);
}

// ============================================================================
// Box (DetectBox) Tests (buffer-backed — no encode)
// ============================================================================

Test(edgefirst_msgs, box_from_cdr_null) {
    errno = 0;
    ros_box_t *handle = ros_box_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, box_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_box_t *handle = ros_box_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(edgefirst_msgs, box_free_null) {
    ros_box_free(NULL);
}

Test(edgefirst_msgs, box_getters_null) {
    cr_assert_float_eq(ros_box_get_center_x(NULL), 0.0f, 0.0001f);
    cr_assert_float_eq(ros_box_get_center_y(NULL), 0.0f, 0.0001f);
    cr_assert_float_eq(ros_box_get_width(NULL), 0.0f, 0.0001f);
    cr_assert_float_eq(ros_box_get_height(NULL), 0.0f, 0.0001f);
    cr_assert_null(ros_box_get_label(NULL));
    cr_assert_float_eq(ros_box_get_score(NULL), 0.0f, 0.0001f);
    cr_assert_float_eq(ros_box_get_distance(NULL), 0.0f, 0.0001f);
    cr_assert_float_eq(ros_box_get_speed(NULL), 0.0f, 0.0001f);
    cr_assert_null(ros_box_get_track_id(NULL));
    cr_assert_eq(ros_box_get_track_lifetime(NULL), 0);
    cr_assert_eq(ros_box_get_track_created_sec(NULL), 0);
    cr_assert_eq(ros_box_get_track_created_nanosec(NULL), 0u);
}

/* Box.cdr golden: center_x=0.5 center_y=0.5 width=0.1 height=0.2 label="car"
 * score=0.98 distance=10.0 speed=5.0 track_id="t1" track_lifetime=5
 * track_created={sec=95, nanosec=0}
 */
static const uint8_t box_cdr[] = {
    0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x3f,0x00,0x00,0x00,0x3f,0xcd,0xcc,0xcc,0x3d,
    0xcd,0xcc,0x4c,0x3e,0x04,0x00,0x00,0x00,0x63,0x61,0x72,0x00,0x48,0xe1,0x7a,0x3f,
    0x00,0x00,0x20,0x41,0x00,0x00,0xa0,0x40,0x03,0x00,0x00,0x00,0x74,0x31,0x00,0x00,
    0x05,0x00,0x00,0x00,0x5f,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

Test(edgefirst_msgs, box_track_created_roundtrip) {
    ros_box_t *handle = ros_box_from_cdr(box_cdr, sizeof(box_cdr));
    cr_assert_not_null(handle);
    cr_assert_eq(ros_box_get_track_created_sec(handle), 95);
    cr_assert_eq(ros_box_get_track_created_nanosec(handle), 0u);
    ros_box_free(handle);
}

// ============================================================================
// Detect Tests (buffer-backed — no encode)
// ============================================================================

Test(edgefirst_msgs, detect_from_cdr_null) {
    errno = 0;
    ros_detect_t *handle = ros_detect_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, detect_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_detect_t *handle = ros_detect_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(edgefirst_msgs, detect_free_null) {
    ros_detect_free(NULL);
}

Test(edgefirst_msgs, detect_getters_null) {
    cr_assert_eq(ros_detect_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_detect_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_detect_get_frame_id(NULL));
    cr_assert_eq(ros_detect_get_boxes_len(NULL), 0);
}

// ============================================================================
// Mask Tests (buffer-backed — has encode)
// ============================================================================

Test(edgefirst_msgs, mask_encode_from_cdr_roundtrip) {
    uint8_t mask_data[] = {0xFF, 0x00, 0xFF, 0x00};
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = ros_mask_encode(&bytes, &len,
                              100,             // height
                              200,             // width
                              4,               // length
                              "raw",           // encoding
                              mask_data, sizeof(mask_data), // data
                              false);          // boxed
    cr_assert_eq(ret, 0);
    cr_assert_not_null(bytes);

    ros_mask_t *handle = ros_mask_from_cdr(bytes, len);
    cr_assert_not_null(handle);

    cr_assert_eq(ros_mask_get_height(handle), 100);
    cr_assert_eq(ros_mask_get_width(handle), 200);
    cr_assert_eq(ros_mask_get_length(handle), 4);
    cr_assert_str_eq(ros_mask_get_encoding(handle), "raw");
    cr_assert_eq(ros_mask_get_boxed(handle), false);

    size_t data_len = 0;
    const uint8_t *data = ros_mask_get_data(handle, &data_len);
    cr_assert_eq(data_len, sizeof(mask_data));
    for (size_t i = 0; i < data_len; i++) {
        cr_assert_eq(data[i], mask_data[i]);
    }

    ros_mask_free(handle);
    ros_bytes_free(bytes, len);
}

Test(edgefirst_msgs, mask_encode_boxed) {
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = ros_mask_encode(&bytes, &len,
                              480, 640, 0,
                              "rle",
                              NULL, 0,
                              true);
    cr_assert_eq(ret, 0);

    ros_mask_t *handle = ros_mask_from_cdr(bytes, len);
    cr_assert_not_null(handle);
    cr_assert_eq(ros_mask_get_boxed(handle), true);

    ros_mask_free(handle);
    ros_bytes_free(bytes, len);
}

Test(edgefirst_msgs, mask_from_cdr_null) {
    errno = 0;
    ros_mask_t *handle = ros_mask_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, mask_free_null) {
    ros_mask_free(NULL);
}

Test(edgefirst_msgs, mask_getters_null) {
    cr_assert_eq(ros_mask_get_height(NULL), 0);
    cr_assert_eq(ros_mask_get_width(NULL), 0);
    cr_assert_eq(ros_mask_get_length(NULL), 0);
    cr_assert_null(ros_mask_get_encoding(NULL));
    cr_assert_null(ros_mask_get_data(NULL, NULL));
    cr_assert_eq(ros_mask_get_boxed(NULL), false);
}

// ============================================================================
// DmaBuffer Tests (buffer-backed — has encode)
//
// DmaBuffer is deprecated in 3.1.0 and removed in 4.0.0. The test suite keeps
// coverage alive through the 3.x line to catch regressions while the API
// still ships; the deprecation warnings that would otherwise break -Werror
// are suppressed locally so that consumer code remains exposed to them.
// ============================================================================

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

Test(edgefirst_msgs, dmabuffer_encode_from_cdr_roundtrip) {
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = ros_dmabuffer_encode(&bytes, &len,
                                   100, 200,       // stamp
                                   "camera0",      // frame_id
                                   5678,           // pid
                                   10,             // fd
                                   640,            // width
                                   480,            // height
                                   1280,           // stride
                                   0x56595559,     // fourcc (YUYV)
                                   614400);        // length
    cr_assert_eq(ret, 0);
    cr_assert_not_null(bytes);

    ros_dmabuffer_t *handle = ros_dmabuffer_from_cdr(bytes, len);
    cr_assert_not_null(handle);

    cr_assert_eq(ros_dmabuffer_get_stamp_sec(handle), 100);
    cr_assert_eq(ros_dmabuffer_get_stamp_nanosec(handle), 200);
    cr_assert_str_eq(ros_dmabuffer_get_frame_id(handle), "camera0");
    cr_assert_eq(ros_dmabuffer_get_pid(handle), 5678);
    cr_assert_eq(ros_dmabuffer_get_fd(handle), 10);
    cr_assert_eq(ros_dmabuffer_get_width(handle), 640);
    cr_assert_eq(ros_dmabuffer_get_height(handle), 480);
    cr_assert_eq(ros_dmabuffer_get_stride(handle), 1280);
    cr_assert_eq(ros_dmabuffer_get_fourcc(handle), 0x56595559);
    cr_assert_eq(ros_dmabuffer_get_length(handle), 614400);

    ros_dmabuffer_free(handle);
    ros_bytes_free(bytes, len);
}

Test(edgefirst_msgs, dmabuffer_from_cdr_null) {
    errno = 0;
    ros_dmabuffer_t *handle = ros_dmabuffer_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, dmabuffer_free_null) {
    ros_dmabuffer_free(NULL);
}

Test(edgefirst_msgs, dmabuffer_getters_null) {
    cr_assert_eq(ros_dmabuffer_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_dmabuffer_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_dmabuffer_get_frame_id(NULL));
    cr_assert_eq(ros_dmabuffer_get_pid(NULL), 0);
    cr_assert_eq(ros_dmabuffer_get_fd(NULL), 0);
    cr_assert_eq(ros_dmabuffer_get_width(NULL), 0);
    cr_assert_eq(ros_dmabuffer_get_height(NULL), 0);
    cr_assert_eq(ros_dmabuffer_get_stride(NULL), 0);
    cr_assert_eq(ros_dmabuffer_get_fourcc(NULL), 0);
    cr_assert_eq(ros_dmabuffer_get_length(NULL), 0);
}

#pragma GCC diagnostic pop

// ============================================================================
// RadarCube Tests (buffer-backed — no encode)
// ============================================================================

Test(edgefirst_msgs, radar_cube_from_cdr_null) {
    errno = 0;
    ros_radar_cube_t *handle = ros_radar_cube_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, radar_cube_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_radar_cube_t *handle = ros_radar_cube_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(edgefirst_msgs, radar_cube_free_null) {
    ros_radar_cube_free(NULL);
}

Test(edgefirst_msgs, radar_cube_getters_null) {
    cr_assert_eq(ros_radar_cube_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_radar_cube_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_radar_cube_get_frame_id(NULL));
    cr_assert_eq(ros_radar_cube_get_timestamp(NULL), 0);
    cr_assert_null(ros_radar_cube_get_layout(NULL, NULL));
    cr_assert_null(ros_radar_cube_get_cube_raw(NULL, NULL));
    cr_assert_eq(ros_radar_cube_get_cube_len(NULL), 0);
    cr_assert_eq(ros_radar_cube_get_is_complex(NULL), false);
}

// ============================================================================
// RadarInfo Tests (buffer-backed — no encode)
// ============================================================================

Test(edgefirst_msgs, radar_info_from_cdr_null) {
    errno = 0;
    ros_radar_info_t *handle = ros_radar_info_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, radar_info_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_radar_info_t *handle = ros_radar_info_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(edgefirst_msgs, radar_info_free_null) {
    ros_radar_info_free(NULL);
}

Test(edgefirst_msgs, radar_info_getters_null) {
    cr_assert_eq(ros_radar_info_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_radar_info_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_radar_info_get_frame_id(NULL));
    cr_assert_null(ros_radar_info_get_center_frequency(NULL));
    cr_assert_null(ros_radar_info_get_frequency_sweep(NULL));
    cr_assert_null(ros_radar_info_get_range_toggle(NULL));
    cr_assert_null(ros_radar_info_get_detection_sensitivity(NULL));
    cr_assert_eq(ros_radar_info_get_cube(NULL), false);
}

// ============================================================================
// Model Tests (buffer-backed — no encode)
// ============================================================================

Test(edgefirst_msgs, model_from_cdr_null) {
    errno = 0;
    ros_model_t *handle = ros_model_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, model_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_model_t *handle = ros_model_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(edgefirst_msgs, model_free_null) {
    ros_model_free(NULL);
}

Test(edgefirst_msgs, model_getters_null) {
    cr_assert_eq(ros_model_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_model_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_model_get_frame_id(NULL));
    cr_assert_eq(ros_model_get_boxes_len(NULL), 0);
    cr_assert_eq(ros_model_get_masks_len(NULL), 0);
}

// ============================================================================
// ModelInfo Tests (buffer-backed — no encode)
// ============================================================================

Test(edgefirst_msgs, model_info_from_cdr_null) {
    errno = 0;
    ros_model_info_t *handle = ros_model_info_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, model_info_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_model_info_t *handle = ros_model_info_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(edgefirst_msgs, model_info_free_null) {
    ros_model_info_free(NULL);
}

Test(edgefirst_msgs, model_info_getters_null) {
    cr_assert_eq(ros_model_info_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_model_info_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_model_info_get_frame_id(NULL));
    cr_assert_null(ros_model_info_get_model_name(NULL));
    cr_assert_null(ros_model_info_get_model_type(NULL));
    cr_assert_null(ros_model_info_get_model_format(NULL));
    cr_assert_eq(ros_model_info_get_input_type(NULL), 0);
    cr_assert_eq(ros_model_info_get_output_type(NULL), 0);
    cr_assert_null(ros_model_info_get_input_shape(NULL, NULL));
    cr_assert_null(ros_model_info_get_output_shape(NULL, NULL));
    cr_assert_eq(ros_model_info_get_labels_len(NULL), 0);
    cr_assert_null(ros_model_info_get_label(NULL, 0));
}

// ============================================================================
// LocalTime Tests (buffer-backed — no encode)
// ============================================================================

Test(edgefirst_msgs, local_time_from_cdr_null) {
    errno = 0;
    ros_local_time_t *handle = ros_local_time_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, local_time_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_local_time_t *handle = ros_local_time_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(edgefirst_msgs, local_time_free_null) {
    ros_local_time_free(NULL);
}

Test(edgefirst_msgs, local_time_getters_null) {
    cr_assert_eq(ros_local_time_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_local_time_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_local_time_get_frame_id(NULL));
    cr_assert_eq(ros_local_time_get_timezone(NULL), 0);
}

// ============================================================================
// Indexed child accessor tests — ros_detect_get_box
//
// Uses golden CDR from testdata/cdr/edgefirst_msgs/Detect_multi.cdr
// which contains 3 boxes: label="a" score=0.95, label="person" score=0.87,
// label="ab" score=0.50.
// ============================================================================

// Detect_multi.cdr embedded as a literal byte array
static const uint8_t detect_multi_cdr[] = {
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

Test(edgefirst_msgs, detect_get_box_roundtrip) {
    ros_detect_t *handle = ros_detect_from_cdr(detect_multi_cdr, sizeof(detect_multi_cdr));
    cr_assert_not_null(handle);

    uint32_t len = ros_detect_get_boxes_len(handle);
    cr_assert_eq(len, 3u);

    // Box 0: label="a"
    const ros_box_t *box0 = ros_detect_get_box(handle, 0);
    cr_assert_not_null(box0);
    cr_assert_str_eq(ros_box_get_label(box0), "a");
    cr_assert_str_eq(ros_box_get_track_id(box0), "t");
    cr_assert_float_eq(ros_box_get_score(box0), 0.95f, 0.001f);

    // Box 1: label="person"
    const ros_box_t *box1 = ros_detect_get_box(handle, 1);
    cr_assert_not_null(box1);
    cr_assert_str_eq(ros_box_get_label(box1), "person");
    cr_assert_str_eq(ros_box_get_track_id(box1), "track_long_id");
    cr_assert_eq(ros_box_get_track_lifetime(box1), 10);

    // Box 2: label="ab"
    const ros_box_t *box2 = ros_detect_get_box(handle, 2);
    cr_assert_not_null(box2);
    cr_assert_str_eq(ros_box_get_label(box2), "ab");
    cr_assert_str_eq(ros_box_get_track_id(box2), "abc");

    ros_detect_free(handle);
}

Test(edgefirst_msgs, detect_get_box_null_view) {
    errno = 0;
    const ros_box_t *box = ros_detect_get_box(NULL, 0);
    cr_assert_null(box);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, detect_get_box_out_of_bounds) {
    ros_detect_t *handle = ros_detect_from_cdr(detect_multi_cdr, sizeof(detect_multi_cdr));
    cr_assert_not_null(handle);

    uint32_t len = ros_detect_get_boxes_len(handle);
    errno = 0;
    const ros_box_t *box = ros_detect_get_box(handle, len); // one past the end
    cr_assert_null(box);
    cr_assert_eq(errno, EINVAL);

    ros_detect_free(handle);
}

// ============================================================================
// Indexed child accessor tests — ros_model_get_box / ros_model_get_mask
//
// Uses golden CDR from testdata/cdr/edgefirst_msgs/Model.cdr
// which contains 1 box (label="car") and 1 mask (encoding="raw", 8 bytes).
// ============================================================================

// Model.cdr embedded as a literal byte array
static const uint8_t model_cdr[] = {
    0x00,0x01,0x00,0x00,0xd2,0x02,0x96,0x49,0x15,0xcd,0x5b,0x07,0x0b,0x00,0x00,0x00,
    0x74,0x65,0x73,0x74,0x5f,0x66,0x72,0x61,0x6d,0x65,0x00,0x00,0x00,0x00,0x00,0x00,
    0x40,0x42,0x0f,0x00,0x00,0x00,0x00,0x00,0x40,0x4b,0x4c,0x00,0x00,0x00,0x00,0x00,
    0x20,0xa1,0x07,0x00,0x00,0x00,0x00,0x00,0x40,0x0d,0x03,0x00,0x01,0x00,0x00,0x00,
    0x00,0x00,0x00,0x3f,0x00,0x00,0x00,0x3f,0xcd,0xcc,0xcc,0x3d,0xcd,0xcc,0x4c,0x3e,
    0x04,0x00,0x00,0x00,0x63,0x61,0x72,0x00,0x48,0xe1,0x7a,0x3f,0x00,0x00,0x20,0x41,
    0x00,0x00,0xa0,0x40,0x03,0x00,0x00,0x00,0x74,0x31,0x00,0x00,0x05,0x00,0x00,0x00,
    0x5f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x02,0x00,0x00,0x00,
    0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x08,0x00,0x00,0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x01
};

Test(edgefirst_msgs, model_get_box_roundtrip) {
    ros_model_t *handle = ros_model_from_cdr(model_cdr, sizeof(model_cdr));
    cr_assert_not_null(handle);

    uint32_t boxes_len = ros_model_get_boxes_len(handle);
    cr_assert_eq(boxes_len, 1u);

    const ros_box_t *box0 = ros_model_get_box(handle, 0);
    cr_assert_not_null(box0);
    cr_assert_str_eq(ros_box_get_label(box0), "car");
    cr_assert_float_eq(ros_box_get_score(box0), 0.98f, 0.001f);

    ros_model_free(handle);
}

Test(edgefirst_msgs, model_get_box_null_view) {
    errno = 0;
    const ros_box_t *box = ros_model_get_box(NULL, 0);
    cr_assert_null(box);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, model_get_box_out_of_bounds) {
    ros_model_t *handle = ros_model_from_cdr(model_cdr, sizeof(model_cdr));
    cr_assert_not_null(handle);

    uint32_t len = ros_model_get_boxes_len(handle);
    errno = 0;
    const ros_box_t *box = ros_model_get_box(handle, len);
    cr_assert_null(box);
    cr_assert_eq(errno, EINVAL);

    ros_model_free(handle);
}

Test(edgefirst_msgs, model_get_mask_roundtrip) {
    ros_model_t *handle = ros_model_from_cdr(model_cdr, sizeof(model_cdr));
    cr_assert_not_null(handle);

    uint32_t masks_len = ros_model_get_masks_len(handle);
    cr_assert_eq(masks_len, 1u);

    const ros_mask_t *mask0 = ros_model_get_mask(handle, 0);
    cr_assert_not_null(mask0);
    cr_assert_eq(ros_mask_get_height(mask0), 2u);
    cr_assert_eq(ros_mask_get_width(mask0), 4u);
    cr_assert_eq(ros_mask_get_boxed(mask0), true);
    size_t data_len = 0;
    const uint8_t *data = ros_mask_get_data(mask0, &data_len);
    cr_assert_not_null(data);
    cr_assert_eq(data_len, 8u);

    ros_model_free(handle);
}

Test(edgefirst_msgs, model_get_mask_null_view) {
    errno = 0;
    const ros_mask_t *mask = ros_model_get_mask(NULL, 0);
    cr_assert_null(mask);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, model_get_mask_out_of_bounds) {
    ros_model_t *handle = ros_model_from_cdr(model_cdr, sizeof(model_cdr));
    cr_assert_not_null(handle);

    uint32_t len = ros_model_get_masks_len(handle);
    errno = 0;
    const ros_mask_t *mask = ros_model_get_mask(handle, len);
    cr_assert_null(mask);
    cr_assert_eq(errno, EINVAL);

    ros_model_free(handle);
}

// ============================================================================
// Defensive ownership tag: ros_box_free / ros_mask_free on parent-borrowed
// handles must safely no-op and set errno=EINVAL instead of corrupting the
// parent's internal child vector. Hardening added in response to PR #15
// review comments 7 and 8 (Copilot code review).
// ============================================================================

Test(memory_safety, ros_box_free_rejects_parent_borrowed) {
    // Decode a Detect and obtain a parent-borrowed child box pointer.
    ros_detect_t *parent = ros_detect_from_cdr(detect_multi_cdr,
                                                sizeof(detect_multi_cdr));
    cr_assert_not_null(parent);

    const ros_box_t *borrowed = ros_detect_get_box(parent, 0);
    cr_assert_not_null(borrowed);
    cr_assert_str_eq(ros_box_get_label(borrowed), "a");

    // Mistakenly cast away const and call ros_box_free. The defensive
    // ownership tag must detect this, set errno=EINVAL, and NOT free
    // the pointer (which would corrupt the parent's child_boxes Vec).
    errno = 0;
    ros_box_free((ros_box_t *) borrowed);
    cr_assert_eq(errno, EINVAL,
                 "ros_box_free on a parent-borrowed handle must set errno=EINVAL");

    // The parent must still be intact — re-fetching the same child returns
    // the same valid pointer with intact fields.
    const ros_box_t *again = ros_detect_get_box(parent, 0);
    cr_assert_not_null(again);
    cr_assert_eq(again, borrowed, "parent-borrowed pointer must be stable");
    cr_assert_str_eq(ros_box_get_label(again), "a");

    // Parent free still works and releases all child storage cleanly.
    ros_detect_free(parent);
}

Test(memory_safety, ros_box_free_still_frees_standalone) {
    // Positive control: ros_box_free on a ros_box_from_cdr result still
    // works exactly as before (the owned flag is true for standalone
    // handles). This ensures the hardening did not break the happy path.
    ros_box_t *owned = ros_box_from_cdr(box_cdr, sizeof(box_cdr));
    cr_assert_not_null(owned);

    errno = 0;
    ros_box_free(owned);
    // No errno expected on success — we can't read `owned` here (freed),
    // but the test passing under ASan proves there was no double-free.
    cr_assert_eq(errno, 0,
                 "ros_box_free on a standalone handle must not set errno");
}

Test(memory_safety, ros_mask_free_rejects_parent_borrowed) {
    // Decode a Model and obtain a parent-borrowed child mask pointer.
    ros_model_t *parent = ros_model_from_cdr(model_cdr, sizeof(model_cdr));
    cr_assert_not_null(parent);

    const ros_mask_t *borrowed = ros_model_get_mask(parent, 0);
    cr_assert_not_null(borrowed);
    cr_assert_eq(ros_mask_get_height(borrowed), 2u);

    // Mistakenly cast away const and call ros_mask_free. Must no-op and
    // set errno=EINVAL.
    errno = 0;
    ros_mask_free((ros_mask_t *) borrowed);
    cr_assert_eq(errno, EINVAL,
                 "ros_mask_free on a parent-borrowed handle must set errno=EINVAL");

    // Parent is still intact.
    const ros_mask_t *again = ros_model_get_mask(parent, 0);
    cr_assert_not_null(again);
    cr_assert_eq(again, borrowed);
    cr_assert_eq(ros_mask_get_height(again), 2u);

    ros_model_free(parent);
}

Test(memory_safety, ros_mask_free_still_frees_standalone) {
    // Positive control: verify a standalone mask decoded from a
    // freshly-encoded CDR buffer is still correctly freed.
    uint8_t *bytes = NULL;
    size_t   len = 0;
    uint8_t  mdata[] = {1, 2, 3, 4, 5, 6, 7, 8};
    int rc = ros_mask_encode(&bytes, &len,
                             2u, 4u, (uint32_t)sizeof(mdata),
                             "mono8", mdata, sizeof(mdata), true);
    cr_assert_eq(rc, 0);
    cr_assert_not_null(bytes);

    ros_mask_t *owned = ros_mask_from_cdr(bytes, len);
    cr_assert_not_null(owned);

    errno = 0;
    ros_mask_free(owned);
    cr_assert_eq(errno, 0);

    ros_bytes_free(bytes, len);
}

// ============================================================================
// Regression test for NULL-view out_len initialization (commits 3a14cbe, 991903f)
//
// Task #43: Verify that blob getters and as_cdr functions explicitly write 0 to
// *out_len when the view is NULL, preventing uninitialized memory reads.
// This test uses sentinel values (0xDEADBEEF) pre-assigned to out_len to catch
// any code path that leaves out_len untouched.
// ============================================================================

Test(memory_safety, blob_getters_null_view_initializes_out_len) {
    size_t out_len;
    const uint8_t *data_ptr;

    // ========================================================================
    // Test representative blob getters (commit 3a14cbe fixes)
    // ========================================================================

    // ros_image_get_data: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_image_get_data(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_image_get_data should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_image_get_data must initialize out_len to 0 on NULL view");

    // ros_compressed_image_get_data: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_compressed_image_get_data(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_compressed_image_get_data should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_compressed_image_get_data must initialize out_len to 0 on NULL view");

    // ros_compressed_video_get_data: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_compressed_video_get_data(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_compressed_video_get_data should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_compressed_video_get_data must initialize out_len to 0 on NULL view");

    // ros_mask_get_data: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_mask_get_data(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_mask_get_data should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_mask_get_data must initialize out_len to 0 on NULL view");

    // ros_radar_cube_get_cube_raw: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_radar_cube_get_cube_raw(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_radar_cube_get_cube_raw should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_radar_cube_get_cube_raw must initialize out_len to 0 on NULL view");

    // ros_point_cloud2_get_data: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_point_cloud2_get_data(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_point_cloud2_get_data should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_point_cloud2_get_data must initialize out_len to 0 on NULL view");

    // ros_model_info_get_input_shape: NULL view must initialize out_len to 0
    // Note: returns *const u32, but uses same out_len mechanism
    out_len = 0xDEADBEEF;
    const uint32_t *shape_ptr = ros_model_info_get_input_shape(NULL, &out_len);
    cr_assert_null(shape_ptr, "ros_model_info_get_input_shape should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_model_info_get_input_shape must initialize out_len to 0 on NULL view");

    // ros_model_info_get_output_shape: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    shape_ptr = ros_model_info_get_output_shape(NULL, &out_len);
    cr_assert_null(shape_ptr, "ros_model_info_get_output_shape should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_model_info_get_output_shape must initialize out_len to 0 on NULL view");

    // ========================================================================
    // Test representative macro-generated as_cdr functions (commit 991903f fixes)
    // The impl_as_cdr! macro expands 15 times; we test a representative subset
    // ========================================================================

    // ros_header_as_cdr: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_header_as_cdr(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_header_as_cdr should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_header_as_cdr must initialize out_len to 0 on NULL view");

    // ros_image_as_cdr: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_image_as_cdr(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_image_as_cdr should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_image_as_cdr must initialize out_len to 0 on NULL view");

    // ros_compressed_image_as_cdr: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_compressed_image_as_cdr(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_compressed_image_as_cdr should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_compressed_image_as_cdr must initialize out_len to 0 on NULL view");

    // ros_point_cloud2_as_cdr: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_point_cloud2_as_cdr(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_point_cloud2_as_cdr should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_point_cloud2_as_cdr must initialize out_len to 0 on NULL view");

    // ros_dmabuffer_as_cdr: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    data_ptr = ros_dmabuffer_as_cdr(NULL, &out_len);
#pragma GCC diagnostic pop
    cr_assert_null(data_ptr, "ros_dmabuffer_as_cdr should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_dmabuffer_as_cdr must initialize out_len to 0 on NULL view");

    // ros_imu_as_cdr: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_imu_as_cdr(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_imu_as_cdr should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_imu_as_cdr must initialize out_len to 0 on NULL view");

    // ros_radar_cube_as_cdr: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_radar_cube_as_cdr(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_radar_cube_as_cdr should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_radar_cube_as_cdr must initialize out_len to 0 on NULL view");

    // ros_track_as_cdr: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_track_as_cdr(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_track_as_cdr should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_track_as_cdr must initialize out_len to 0 on NULL view");

    // ros_local_time_as_cdr: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_local_time_as_cdr(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_local_time_as_cdr should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_local_time_as_cdr must initialize out_len to 0 on NULL view");

    // ========================================================================
    // Test manually-written as_cdr functions (also fixed in commit 3a14cbe)
    // ========================================================================

    // ros_detect_as_cdr: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_detect_as_cdr(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_detect_as_cdr should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_detect_as_cdr must initialize out_len to 0 on NULL view");

    // ros_model_as_cdr: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    data_ptr = ros_model_as_cdr(NULL, &out_len);
    cr_assert_null(data_ptr, "ros_model_as_cdr should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_model_as_cdr must initialize out_len to 0 on NULL view");

    // ========================================================================
    // Test radar layout getter (commit 991903f fix)
    // ========================================================================

    // ros_radar_cube_get_layout: NULL view must initialize out_len to 0
    out_len = 0xDEADBEEF;
    const uint8_t *layout_ptr = ros_radar_cube_get_layout(NULL, &out_len);
    cr_assert_null(layout_ptr, "ros_radar_cube_get_layout should return NULL for NULL view");
    cr_assert_eq(out_len, 0, "ros_radar_cube_get_layout must initialize out_len to 0 on NULL view");
}

// ============================================================================
// CameraFrame / CameraPlane Tests (buffer-backed, multi-plane, no encode)
// ============================================================================
//
// Uses inlined bytes from testdata/cdr/edgefirst_msgs/CameraFrame_nv12.cdr.
// Encoding a full NV12 frame descriptor: seq=2, pid=1234, 1920x1080, NV12,
// BT.709 limited range, no fence, two shared-fd planes (Y then UV).

static const uint8_t camera_frame_nv12_cdr[] = {
    0x00,0x01,0x00,0x00,0xd2,0x02,0x96,0x49,0x15,0xcd,0x5b,0x07,0x0b,0x00,0x00,0x00,
    0x74,0x65,0x73,0x74,0x5f,0x66,0x72,0x61,0x6d,0x65,0x00,0x00,0x02,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0xd2,0x04,0x00,0x00,0x80,0x07,0x00,0x00,0x38,0x04,0x00,0x00,
    0x05,0x00,0x00,0x00,0x4e,0x56,0x31,0x32,0x00,0x00,0x00,0x00,0x06,0x00,0x00,0x00,
    0x62,0x74,0x37,0x30,0x39,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x62,0x74,0x37,0x30,
    0x39,0x00,0x00,0x00,0x06,0x00,0x00,0x00,0x62,0x74,0x37,0x30,0x39,0x00,0x00,0x00,
    0x08,0x00,0x00,0x00,0x6c,0x69,0x6d,0x69,0x74,0x65,0x64,0x00,0xff,0xff,0xff,0xff,
    0x02,0x00,0x00,0x00,0x2a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x07,0x00,0x00,
    0x00,0xa4,0x1f,0x00,0x00,0xa4,0x1f,0x00,0x00,0x00,0x00,0x00,0x2a,0x00,0x00,0x00,
    0x00,0xa4,0x1f,0x00,0x80,0x07,0x00,0x00,0x00,0xd2,0x0f,0x00,0x00,0xd2,0x0f,0x00,
    0x00,0x00,0x00,0x00,
};

Test(edgefirst_msgs, camera_frame_from_cdr_null) {
    errno = 0;
    ros_camera_frame_t *h = ros_camera_frame_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, camera_frame_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_camera_frame_t *h = ros_camera_frame_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(edgefirst_msgs, camera_frame_free_null) {
    ros_camera_frame_free(NULL);
}

Test(edgefirst_msgs, camera_frame_getters_null) {
    cr_assert_eq(ros_camera_frame_get_seq(NULL), 0u);
    cr_assert_eq(ros_camera_frame_get_pid(NULL), 0u);
    cr_assert_eq(ros_camera_frame_get_width(NULL), 0u);
    cr_assert_eq(ros_camera_frame_get_height(NULL), 0u);
    cr_assert_eq(ros_camera_frame_get_fence_fd(NULL), -1);
    cr_assert_null(ros_camera_frame_get_format(NULL));
    cr_assert_null(ros_camera_frame_get_frame_id(NULL));
    cr_assert_eq(ros_camera_frame_get_planes_len(NULL), 0u);
}

Test(edgefirst_msgs, camera_frame_nv12_decode) {
    ros_camera_frame_t *h =
        ros_camera_frame_from_cdr(camera_frame_nv12_cdr,
                                         sizeof(camera_frame_nv12_cdr));
    cr_assert_not_null(h);

    cr_assert_eq(ros_camera_frame_get_seq(h), 2u);
    cr_assert_eq(ros_camera_frame_get_pid(h), 1234u);
    cr_assert_eq(ros_camera_frame_get_width(h), 1920u);
    cr_assert_eq(ros_camera_frame_get_height(h), 1080u);
    cr_assert_eq(ros_camera_frame_get_fence_fd(h), -1);
    cr_assert_str_eq(ros_camera_frame_get_format(h), "NV12");
    cr_assert_str_eq(ros_camera_frame_get_color_space(h), "bt709");
    cr_assert_str_eq(ros_camera_frame_get_color_transfer(h), "bt709");
    cr_assert_str_eq(ros_camera_frame_get_color_encoding(h), "bt709");
    cr_assert_str_eq(ros_camera_frame_get_color_range(h), "limited");
    cr_assert_eq(ros_camera_frame_get_planes_len(h), 2u);

    const ros_camera_plane_t *y = ros_camera_frame_get_plane(h, 0);
    cr_assert_not_null(y);
    cr_assert_eq(ros_camera_plane_get_fd(y), 42);
    cr_assert_eq(ros_camera_plane_get_offset(y), 0u);
    cr_assert_eq(ros_camera_plane_get_stride(y), 1920u);
    cr_assert_eq(ros_camera_plane_get_size(y), 2073600u);
    cr_assert_eq(ros_camera_plane_get_used(y), 2073600u);

    const ros_camera_plane_t *uv = ros_camera_frame_get_plane(h, 1);
    cr_assert_not_null(uv);
    cr_assert_eq(ros_camera_plane_get_offset(uv), 2073600u);
    cr_assert_eq(ros_camera_plane_get_size(uv), 1036800u);

    // Shared-fd NV12: both planes reference fd 42.
    cr_assert_eq(ros_camera_plane_get_fd(uv),
                 ros_camera_plane_get_fd(y));

    // For fd-backed planes data[] is empty.
    size_t data_len = 0xDEADBEEF;
    const uint8_t *data = ros_camera_plane_get_data(y, &data_len);
    cr_assert_null(data);
    cr_assert_eq(data_len, 0u);

    ros_camera_frame_free(h);
}

Test(edgefirst_msgs, camera_frame_get_plane_out_of_bounds) {
    ros_camera_frame_t *h =
        ros_camera_frame_from_cdr(camera_frame_nv12_cdr,
                                         sizeof(camera_frame_nv12_cdr));
    cr_assert_not_null(h);
    errno = 0;
    const ros_camera_plane_t *p = ros_camera_frame_get_plane(h, 99);
    cr_assert_null(p);
    cr_assert_eq(errno, EINVAL);
    ros_camera_frame_free(h);
}

Test(memory_safety, ros_camera_plane_free_rejects_parent_borrowed) {
    ros_camera_frame_t *parent =
        ros_camera_frame_from_cdr(camera_frame_nv12_cdr,
                                         sizeof(camera_frame_nv12_cdr));
    cr_assert_not_null(parent);
    const ros_camera_plane_t *borrowed =
        ros_camera_frame_get_plane(parent, 0);
    cr_assert_not_null(borrowed);

    errno = 0;
    ros_camera_plane_free((ros_camera_plane_t *) borrowed);
    cr_assert_eq(errno, EINVAL,
                 "freeing a parent-borrowed plane must no-op with errno=EINVAL");

    // Parent still intact.
    const ros_camera_plane_t *again =
        ros_camera_frame_get_plane(parent, 0);
    cr_assert_eq(again, borrowed);
    cr_assert_eq(ros_camera_plane_get_fd(again), 42);

    ros_camera_frame_free(parent);
}

// -- Additional CameraFrame fixture coverage ---------------------------------
// The NV12 case above lives as an inlined byte array for self-containment;
// i420 / h264 / split_fd / inlined / empty / planar_nchw are loaded at
// runtime from testdata/ to keep the C suite in sync with the Python
// generator without copy/pasting five more hex blobs.

Test(edgefirst_msgs, camera_frame_i420_three_planes) {
    size_t len = 0;
    uint8_t *b = load_fixture("testdata/cdr/edgefirst_msgs/CameraFrame_i420.cdr",
                              &len);
    cr_assert_not_null(b, "fixture missing — run scripts/generate_cdr_testdata.py");
    ros_camera_frame_t *h = ros_camera_frame_from_cdr(b, len);
    cr_assert_not_null(h);
    cr_assert_str_eq(ros_camera_frame_get_format(h), "I420");
    cr_assert_eq(ros_camera_frame_get_planes_len(h), 3u);
    // Y plane full resolution, U/V planes quarter-size (chroma subsampled).
    cr_assert_eq(ros_camera_plane_get_size(ros_camera_frame_get_plane(h, 0)),
                 1920u * 1080u);
    cr_assert_eq(ros_camera_plane_get_size(ros_camera_frame_get_plane(h, 1)),
                 (1920u / 2u) * (1080u / 2u));
    cr_assert_eq(ros_camera_plane_get_size(ros_camera_frame_get_plane(h, 2)),
                 (1920u / 2u) * (1080u / 2u));
    ros_camera_frame_free(h);
    free(b);
}

Test(edgefirst_msgs, camera_frame_h264_used_lt_size) {
    size_t len = 0;
    uint8_t *b = load_fixture("testdata/cdr/edgefirst_msgs/CameraFrame_h264.cdr",
                              &len);
    cr_assert_not_null(b);
    ros_camera_frame_t *h = ros_camera_frame_from_cdr(b, len);
    cr_assert_not_null(h);
    cr_assert_str_eq(ros_camera_frame_get_format(h), "h264");
    cr_assert_eq(ros_camera_frame_get_planes_len(h), 1u);
    const ros_camera_plane_t *p = ros_camera_frame_get_plane(h, 0);
    // The defining property of a codec bitstream: valid payload is a
    // subset of the allocated DMA buffer.
    cr_assert_lt(ros_camera_plane_get_used(p),
                 ros_camera_plane_get_size(p),
                 "h264 payload must be strictly smaller than its buffer");
    ros_camera_frame_free(h);
    free(b);
}

Test(edgefirst_msgs, camera_frame_split_fd_distinct_fds_and_fence) {
    size_t len = 0;
    uint8_t *b = load_fixture(
        "testdata/cdr/edgefirst_msgs/CameraFrame_split_fd.cdr", &len);
    cr_assert_not_null(b);
    ros_camera_frame_t *h = ros_camera_frame_from_cdr(b, len);
    cr_assert_not_null(h);
    cr_assert_eq(ros_camera_frame_get_fence_fd(h), 77);
    cr_assert_eq(ros_camera_frame_get_planes_len(h), 2u);
    int y_fd = ros_camera_plane_get_fd(ros_camera_frame_get_plane(h, 0));
    int uv_fd = ros_camera_plane_get_fd(ros_camera_frame_get_plane(h, 1));
    cr_assert_neq(y_fd, uv_fd, "V4L2 MPLANE delivers distinct fds per plane");
    ros_camera_frame_free(h);
    free(b);
}

Test(edgefirst_msgs, camera_frame_inlined_bridge) {
    size_t len = 0;
    uint8_t *b = load_fixture(
        "testdata/cdr/edgefirst_msgs/CameraFrame_inlined.cdr", &len);
    cr_assert_not_null(b);
    ros_camera_frame_t *h = ros_camera_frame_from_cdr(b, len);
    cr_assert_not_null(h);
    cr_assert_eq(ros_camera_frame_get_pid(h), 0u,
                 "off-device bridges signal pid=0");
    const ros_camera_plane_t *p = ros_camera_frame_get_plane(h, 0);
    cr_assert_eq(ros_camera_plane_get_fd(p), -1,
                 "fd=-1 selects inlined payload path");
    size_t data_len = 0;
    const uint8_t *data = ros_camera_plane_get_data(p, &data_len);
    cr_assert_not_null(data);
    cr_assert_eq(data_len, 1024u);
    cr_assert_eq(data[0], 0x00);
    cr_assert_eq(data[255], 0xFF);
    cr_assert_eq(data[256], 0x00);  // i & 0xFF wraps
    ros_camera_frame_free(h);
    free(b);
}

Test(edgefirst_msgs, camera_frame_planar_nchw) {
    size_t len = 0;
    uint8_t *b = load_fixture(
        "testdata/cdr/edgefirst_msgs/CameraFrame_planar_nchw.cdr", &len);
    cr_assert_not_null(b);
    ros_camera_frame_t *h = ros_camera_frame_from_cdr(b, len);
    cr_assert_not_null(h);
    cr_assert_str_eq(ros_camera_frame_get_format(h), "rgb8_planar_nchw");
    cr_assert_eq(ros_camera_frame_get_planes_len(h), 3u);
    ros_camera_frame_free(h);
    free(b);
}

Test(edgefirst_msgs, camera_frame_empty_metadata_only) {
    // Zero-plane frame: num_planes == 0 is valid (metadata-only frame).
    size_t len = 0;
    uint8_t *b = load_fixture(
        "testdata/cdr/edgefirst_msgs/CameraFrame_empty.cdr", &len);
    cr_assert_not_null(b);
    ros_camera_frame_t *h = ros_camera_frame_from_cdr(b, len);
    cr_assert_not_null(h);
    cr_assert_eq(ros_camera_frame_get_planes_len(h), 0u);
    errno = 0;
    cr_assert_null(ros_camera_frame_get_plane(h, 0));
    cr_assert_eq(errno, EINVAL);
    ros_camera_frame_free(h);
    free(b);
}

// ============================================================================
// Vibration Tests (TOP2-770)
// ============================================================================

Test(edgefirst_msgs, vibration_from_golden_fixture) {
    size_t len = 0;
    uint8_t *b = load_fixture("testdata/cdr/edgefirst_msgs/Vibration.cdr", &len);
    cr_assert_not_null(b, "failed to load Vibration fixture");
    ros_vibration_t *v = ros_vibration_from_cdr(b, len);
    cr_assert_not_null(v);
    cr_assert_str_eq(ros_vibration_get_frame_id(v), "test_frame");
    cr_assert_eq(ros_vibration_get_measurement_type(v),
                 ROS_VIBRATION_MEASUREMENT_RMS);
    cr_assert_eq(ros_vibration_get_unit(v), ROS_VIBRATION_UNIT_ACCEL_M_PER_S2);
    cr_assert_float_eq(ros_vibration_get_band_lower_hz(v), 10.0f, 1e-5);
    cr_assert_float_eq(ros_vibration_get_band_upper_hz(v), 1000.0f, 1e-5);
    double x = 0, y = 0, z = 0;
    ros_vibration_get_vibration(v, &x, &y, &z);
    cr_assert_float_eq(x, 0.42, 1e-12);
    cr_assert_float_eq(y, 0.51, 1e-12);
    cr_assert_float_eq(z, 0.37, 1e-12);
    cr_assert_eq(ros_vibration_get_clipping_len(v), 3);
    uint32_t cl[4] = {0};
    uint32_t n = ros_vibration_get_clipping(v, cl, 4);
    cr_assert_eq(n, 3);
    cr_assert_eq(cl[0], 3);
    cr_assert_eq(cl[1], 1);
    cr_assert_eq(cl[2], 0);
    ros_vibration_free(v);
    free(b);
}

Test(edgefirst_msgs, vibration_from_cdr_null) {
    errno = 0;
    cr_assert_null(ros_vibration_from_cdr(NULL, 100));
    cr_assert_eq(errno, EINVAL);
}

Test(edgefirst_msgs, vibration_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    cr_assert_null(ros_vibration_from_cdr(bad, sizeof(bad)));
    cr_assert_eq(errno, EBADMSG);
}

Test(edgefirst_msgs, vibration_free_null) {
    ros_vibration_free(NULL);
}

Test(edgefirst_msgs, vibration_getters_null) {
    cr_assert_eq(ros_vibration_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_vibration_get_unit(NULL), 0);
    cr_assert_eq(ros_vibration_get_clipping_len(NULL), 0);
    cr_assert_eq(ros_vibration_get_clipping(NULL, NULL, 0), 0);
    cr_assert_null(ros_vibration_get_frame_id(NULL));
    ros_vibration_get_vibration(NULL, NULL, NULL, NULL);
}
