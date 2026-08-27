/**
 * @file test_foxglove_msgs.c
 * @brief Criterion tests for Foxglove messages (FoxgloveCompressedVideo)
 *
 * Buffer-backed type: FoxgloveCompressedVideo
 *   ros_foxglove_compressed_video_builder_* / ros_compressed_video_from_cdr
 *   ros_compressed_video_get_* / ros_compressed_video_free
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright 2025 Au-Zone Technologies. All Rights Reserved.
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "edgefirst/schemas.h"

static int build_compressed_video(uint8_t **bytes, size_t *len,
                                  int32_t sec, uint32_t nsec,
                                  const char *frame_id,
                                  const uint8_t *data, size_t data_len,
                                  const char *format) {
    ros_foxglove_compressed_video_builder_t *b =
        ros_foxglove_compressed_video_builder_new();
    if (!b) return -1;
    ros_foxglove_compressed_video_builder_set_stamp(b, sec, nsec);
    if (ros_foxglove_compressed_video_builder_set_frame_id(b, frame_id) != 0 ||
        ros_foxglove_compressed_video_builder_set_data(b, data, data_len) != 0 ||
        ros_foxglove_compressed_video_builder_set_format(b, format) != 0) {
        ros_foxglove_compressed_video_builder_free(b);
        return -1;
    }
    int ret = ros_foxglove_compressed_video_builder_build(b, bytes, len);
    ros_foxglove_compressed_video_builder_free(b);
    return ret;
}

static int build_foxglove_compressed_image(uint8_t **bytes, size_t *len,
                                           int32_t sec, uint32_t nsec,
                                           const char *frame_id,
                                           const uint8_t *data, size_t data_len,
                                           const char *format) {
    ros_foxglove_compressed_image_builder_t *b =
        ros_foxglove_compressed_image_builder_new();
    if (!b) return -1;
    ros_foxglove_compressed_image_builder_set_stamp(b, sec, nsec);
    if (ros_foxglove_compressed_image_builder_set_frame_id(b, frame_id) != 0 ||
        ros_foxglove_compressed_image_builder_set_data(b, data, data_len) != 0 ||
        ros_foxglove_compressed_image_builder_set_format(b, format) != 0) {
        ros_foxglove_compressed_image_builder_free(b);
        return -1;
    }
    int ret = ros_foxglove_compressed_image_builder_build(b, bytes, len);
    ros_foxglove_compressed_image_builder_free(b);
    return ret;
}

// ============================================================================
// FoxgloveCompressedVideo Tests
// ============================================================================

Test(foxglove_msgs, compressed_video_encode_from_cdr_roundtrip) {
    uint8_t test_data[] = {0x00, 0x00, 0x00, 0x01, 0x65, 0x88, 0x84, 0x00};
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = build_compressed_video(&bytes, &len,
                                     1234567890, 123456789,
                                     "video_stream",
                                     test_data, sizeof(test_data),
                                     "h264");
    cr_assert_eq(ret, 0, "Encode should succeed");
    cr_assert_not_null(bytes);
    cr_assert_gt(len, 0);

    ros_compressed_video_t *handle = ros_compressed_video_from_cdr(bytes, len);
    cr_assert_not_null(handle, "from_cdr should succeed");

    // Verify header
    cr_assert_eq(ros_compressed_video_get_stamp_sec(handle), 1234567890);
    cr_assert_eq(ros_compressed_video_get_stamp_nanosec(handle), 123456789);
    cr_assert_str_eq(ros_compressed_video_get_frame_id(handle), "video_stream");

    // Verify format
    cr_assert_str_eq(ros_compressed_video_get_format(handle), "h264");

    // Verify data
    size_t data_len = 0;
    const uint8_t *data = ros_compressed_video_get_data(handle, &data_len);
    cr_assert_eq(data_len, sizeof(test_data));
    for (size_t i = 0; i < data_len; i++) {
        cr_assert_eq(data[i], test_data[i]);
    }

    ros_compressed_video_free(handle);
    ros_bytes_free(bytes, len);
}

Test(foxglove_msgs, compressed_video_encode_h265) {
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = build_compressed_video(&bytes, &len, 0, 0, "cam0", NULL, 0, "h265");
    cr_assert_eq(ret, 0);

    ros_compressed_video_t *handle = ros_compressed_video_from_cdr(bytes, len);
    cr_assert_not_null(handle);
    cr_assert_str_eq(ros_compressed_video_get_format(handle), "h265");

    ros_compressed_video_free(handle);
    ros_bytes_free(bytes, len);
}

Test(foxglove_msgs, compressed_video_encode_empty_data) {
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = build_compressed_video(&bytes, &len, 0, 0, "", NULL, 0, "");
    cr_assert_eq(ret, 0);

    ros_compressed_video_t *handle = ros_compressed_video_from_cdr(bytes, len);
    cr_assert_not_null(handle);

    size_t data_len = 0;
    const uint8_t *data = ros_compressed_video_get_data(handle, &data_len);
    (void)data;
    cr_assert_eq(data_len, 0, "Default data length should be 0");

    ros_compressed_video_free(handle);
    ros_bytes_free(bytes, len);
}

Test(foxglove_msgs, compressed_video_large_data) {
    // Simulate a larger video frame (1MB)
    size_t frame_size = 1024 * 1024;
    uint8_t *large_data = (uint8_t *)malloc(frame_size);
    cr_assert_not_null(large_data);

    for (size_t i = 0; i < frame_size; i++) {
        large_data[i] = (uint8_t)(i & 0xFF);
    }

    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = build_compressed_video(&bytes, &len, 0, 0, "cam",
                                     large_data, frame_size, "h264");
    cr_assert_eq(ret, 0);

    ros_compressed_video_t *handle = ros_compressed_video_from_cdr(bytes, len);
    cr_assert_not_null(handle);

    size_t data_len = 0;
    const uint8_t *data = ros_compressed_video_get_data(handle, &data_len);
    cr_assert_eq(data_len, frame_size);
    cr_assert_eq(data[0], 0);
    cr_assert_eq(data[frame_size - 1], (uint8_t)((frame_size - 1) & 0xFF));

    free(large_data);
    ros_compressed_video_free(handle);
    ros_bytes_free(bytes, len);
}

Test(foxglove_msgs, compressed_video_from_cdr_null) {
    errno = 0;
    ros_compressed_video_t *handle = ros_compressed_video_from_cdr(NULL, 100);
    cr_assert_null(handle, "Should return NULL for NULL data");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(foxglove_msgs, compressed_video_from_cdr_invalid) {
    uint8_t garbage[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_compressed_video_t *handle = ros_compressed_video_from_cdr(garbage, sizeof(garbage));
    cr_assert_null(handle, "Should return NULL for invalid CDR data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(foxglove_msgs, compressed_video_free_null) {
    // Should not crash
    ros_compressed_video_free(NULL);
}

Test(foxglove_msgs, compressed_video_getters_null) {
    cr_assert_eq(ros_compressed_video_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_compressed_video_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_compressed_video_get_frame_id(NULL));
    cr_assert_null(ros_compressed_video_get_format(NULL));
    cr_assert_null(ros_compressed_video_get_data(NULL, NULL));
}

// ============================================================================
// FoxgloveCompressedImage Tests
//
// Wire-identical to CompressedVideo; uses the ros_foxglove_compressed_image_
// prefix (the short ros_compressed_image_ belongs to sensor_msgs).
// ============================================================================

Test(foxglove_msgs, compressed_image_encode_from_cdr_roundtrip) {
    uint8_t test_data[] = {0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46};
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = build_foxglove_compressed_image(&bytes, &len,
                                              1234567890, 123456789,
                                              "image_stream",
                                              test_data, sizeof(test_data),
                                              "jpeg");
    cr_assert_eq(ret, 0, "Encode should succeed");
    cr_assert_not_null(bytes);
    cr_assert_gt(len, 0);

    ros_foxglove_compressed_image_t *handle =
        ros_foxglove_compressed_image_from_cdr(bytes, len);
    cr_assert_not_null(handle, "from_cdr should succeed");

    cr_assert_eq(ros_foxglove_compressed_image_get_stamp_sec(handle), 1234567890);
    cr_assert_eq(ros_foxglove_compressed_image_get_stamp_nanosec(handle), 123456789);
    // timestamp alias getters must agree with stamp getters.
    cr_assert_eq(ros_foxglove_compressed_image_get_timestamp_sec(handle), 1234567890);
    cr_assert_eq(ros_foxglove_compressed_image_get_timestamp_nanosec(handle), 123456789);
    cr_assert_str_eq(ros_foxglove_compressed_image_get_frame_id(handle), "image_stream");
    cr_assert_str_eq(ros_foxglove_compressed_image_get_format(handle), "jpeg");

    size_t data_len = 0;
    const uint8_t *data = ros_foxglove_compressed_image_get_data(handle, &data_len);
    cr_assert_eq(data_len, sizeof(test_data));
    for (size_t i = 0; i < data_len; i++) {
        cr_assert_eq(data[i], test_data[i]);
    }

    ros_foxglove_compressed_image_free(handle);
    ros_bytes_free(bytes, len);
}

Test(foxglove_msgs, compressed_image_builder_roundtrip) {
    uint8_t test_data[] = {0x89, 0x50, 0x4E, 0x47}; // PNG signature prefix
    ros_foxglove_compressed_image_builder_t *b =
        ros_foxglove_compressed_image_builder_new();
    cr_assert_not_null(b);

    ros_foxglove_compressed_image_builder_set_timestamp(b, 7, 9);
    cr_assert_eq(ros_foxglove_compressed_image_builder_set_frame_id(b, "cam0"), 0);
    cr_assert_eq(ros_foxglove_compressed_image_builder_set_data(b, test_data, sizeof(test_data)), 0);
    cr_assert_eq(ros_foxglove_compressed_image_builder_set_format(b, "png"), 0);

    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_foxglove_compressed_image_builder_build(b, &bytes, &len), 0);
    ros_foxglove_compressed_image_builder_free(b);

    ros_foxglove_compressed_image_t *handle =
        ros_foxglove_compressed_image_from_cdr(bytes, len);
    cr_assert_not_null(handle);
    cr_assert_eq(ros_foxglove_compressed_image_get_stamp_sec(handle), 7);
    cr_assert_str_eq(ros_foxglove_compressed_image_get_format(handle), "png");

    // In-place stamp setter rewrites the buffer.
    cr_assert_eq(ros_foxglove_compressed_image_set_stamp(bytes, len, 100, 200), 0);
    ros_foxglove_compressed_image_free(handle);
    handle = ros_foxglove_compressed_image_from_cdr(bytes, len);
    cr_assert_eq(ros_foxglove_compressed_image_get_stamp_sec(handle), 100);
    cr_assert_eq(ros_foxglove_compressed_image_get_stamp_nanosec(handle), 200);

    ros_foxglove_compressed_image_free(handle);
    ros_bytes_free(bytes, len);
}

Test(foxglove_msgs, compressed_image_wire_identical_to_video) {
    uint8_t d[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t *img = NULL, *vid = NULL;
    size_t img_len = 0, vid_len = 0;

    cr_assert_eq(build_foxglove_compressed_image(
                     &img, &img_len, 42, 7, "cam0", d, sizeof(d), "h264"),
                 0);
    cr_assert_eq(build_compressed_video(
                     &vid, &vid_len, 42, 7, "cam0", d, sizeof(d), "h264"),
                 0);
    cr_assert_eq(img_len, vid_len, "wire sizes must match");
    cr_assert_eq(memcmp(img, vid, img_len), 0, "CDR bytes must be identical");

    ros_bytes_free(img, img_len);
    ros_bytes_free(vid, vid_len);
}

Test(foxglove_msgs, compressed_image_from_cdr_null) {
    errno = 0;
    ros_foxglove_compressed_image_t *handle =
        ros_foxglove_compressed_image_from_cdr(NULL, 100);
    cr_assert_null(handle, "Should return NULL for NULL data");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(foxglove_msgs, compressed_image_from_cdr_invalid) {
    uint8_t garbage[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_foxglove_compressed_image_t *handle =
        ros_foxglove_compressed_image_from_cdr(garbage, sizeof(garbage));
    cr_assert_null(handle, "Should return NULL for invalid CDR data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(foxglove_msgs, compressed_image_getters_null) {
    cr_assert_eq(ros_foxglove_compressed_image_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_foxglove_compressed_image_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_foxglove_compressed_image_get_frame_id(NULL));
    cr_assert_null(ros_foxglove_compressed_image_get_format(NULL));
    cr_assert_null(ros_foxglove_compressed_image_get_data(NULL, NULL));
}
