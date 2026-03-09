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
#include <string.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

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
// ============================================================================

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
