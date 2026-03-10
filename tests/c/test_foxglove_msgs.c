/**
 * @file test_foxglove_msgs.c
 * @brief Criterion tests for Foxglove messages (FoxgloveCompressedVideo)
 *
 * Buffer-backed type: FoxgloveCompressedVideo
 *   ros_compressed_video_encode(&out_bytes, &out_len, stamp_sec, stamp_nanosec,
 *                                frame_id, data, data_len, format) -> int
 *   ros_compressed_video_from_cdr(data, len) -> ros_compressed_video_t*
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

// ============================================================================
// FoxgloveCompressedVideo Tests
// ============================================================================

Test(foxglove_msgs, compressed_video_encode_from_cdr_roundtrip) {
    uint8_t test_data[] = {0x00, 0x00, 0x00, 0x01, 0x65, 0x88, 0x84, 0x00};
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = ros_compressed_video_encode(&bytes, &len,
                                          1234567890, 123456789,  // stamp
                                          "video_stream",         // frame_id
                                          test_data, sizeof(test_data), // data
                                          "h264");                // format
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

    int ret = ros_compressed_video_encode(&bytes, &len,
                                          0, 0,
                                          "cam0",
                                          NULL, 0,
                                          "h265");
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

    int ret = ros_compressed_video_encode(&bytes, &len,
                                          0, 0,
                                          "",
                                          NULL, 0,
                                          "");
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

    int ret = ros_compressed_video_encode(&bytes, &len,
                                          0, 0,
                                          "cam",
                                          large_data, frame_size,
                                          "h264");
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
