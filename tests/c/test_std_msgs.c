/**
 * @file test_std_msgs.c
 * @brief Criterion tests for std_msgs (Header — buffer-backed)
 *
 * Header API:
 *   ros_header_encode(&out_bytes, &out_len, stamp_sec, stamp_nanosec, frame_id) -> int
 *   ros_header_from_cdr(data, len) -> ros_header_t*
 *   ros_header_get_stamp_sec(handle) -> i32
 *   ros_header_get_stamp_nanosec(handle) -> u32
 *   ros_header_get_frame_id(handle) -> const char*
 *   ros_header_free(handle)
 *   ros_bytes_free(bytes, len)
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Header Tests
// ============================================================================

Test(std_msgs, header_encode_from_cdr_roundtrip) {
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = ros_header_encode(&bytes, &len, 42, 999, "test_frame");
    cr_assert_eq(ret, 0, "Encode should succeed");
    cr_assert_not_null(bytes, "Output bytes should not be NULL");
    cr_assert_gt(len, 0, "Output length should be > 0");

    ros_header_t *handle = ros_header_from_cdr(bytes, len);
    cr_assert_not_null(handle, "from_cdr should succeed");

    cr_assert_eq(ros_header_get_stamp_sec(handle), 42);
    cr_assert_eq(ros_header_get_stamp_nanosec(handle), 999);

    const char *frame_id = ros_header_get_frame_id(handle);
    cr_assert_not_null(frame_id);
    cr_assert_str_eq(frame_id, "test_frame");

    ros_header_free(handle);
    ros_bytes_free(bytes, len);
}

Test(std_msgs, header_encode_empty_frame_id) {
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = ros_header_encode(&bytes, &len, 0, 0, "");
    cr_assert_eq(ret, 0);

    ros_header_t *handle = ros_header_from_cdr(bytes, len);
    cr_assert_not_null(handle);

    cr_assert_eq(ros_header_get_stamp_sec(handle), 0);
    cr_assert_eq(ros_header_get_stamp_nanosec(handle), 0);

    const char *frame_id = ros_header_get_frame_id(handle);
    cr_assert_not_null(frame_id);
    cr_assert_str_eq(frame_id, "");

    ros_header_free(handle);
    ros_bytes_free(bytes, len);
}

Test(std_msgs, header_encode_null_frame_id) {
    uint8_t *bytes = NULL;
    size_t len = 0;

    // NULL frame_id should be treated as empty string
    int ret = ros_header_encode(&bytes, &len, 100, 200, NULL);
    cr_assert_eq(ret, 0);

    ros_header_t *handle = ros_header_from_cdr(bytes, len);
    cr_assert_not_null(handle);

    cr_assert_eq(ros_header_get_stamp_sec(handle), 100);
    cr_assert_eq(ros_header_get_stamp_nanosec(handle), 200);

    ros_header_free(handle);
    ros_bytes_free(bytes, len);
}

Test(std_msgs, header_encode_long_frame_id) {
    char long_string[1001];
    memset(long_string, 'A', 1000);
    long_string[1000] = '\0';

    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = ros_header_encode(&bytes, &len, 1, 2, long_string);
    cr_assert_eq(ret, 0);

    ros_header_t *handle = ros_header_from_cdr(bytes, len);
    cr_assert_not_null(handle);

    const char *frame_id = ros_header_get_frame_id(handle);
    cr_assert_not_null(frame_id);
    cr_assert_str_eq(frame_id, long_string);

    ros_header_free(handle);
    ros_bytes_free(bytes, len);
}

Test(std_msgs, header_encode_special_chars) {
    const char *special = "frame/with-special_chars.123";
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = ros_header_encode(&bytes, &len, 0, 0, special);
    cr_assert_eq(ret, 0);

    ros_header_t *handle = ros_header_from_cdr(bytes, len);
    cr_assert_not_null(handle);

    const char *frame_id = ros_header_get_frame_id(handle);
    cr_assert_str_eq(frame_id, special);

    ros_header_free(handle);
    ros_bytes_free(bytes, len);
}

Test(std_msgs, header_from_cdr_null) {
    errno = 0;
    ros_header_t *handle = ros_header_from_cdr(NULL, 100);
    cr_assert_null(handle, "Should return NULL for NULL data");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(std_msgs, header_from_cdr_invalid) {
    uint8_t bad_data[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    errno = 0;
    ros_header_t *handle = ros_header_from_cdr(bad_data, 4);
    cr_assert_null(handle, "Should return NULL for invalid data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(std_msgs, header_free_null) {
    // Should not crash when freeing NULL
    ros_header_free(NULL);
}

Test(std_msgs, header_getters_null_handle) {
    cr_assert_eq(ros_header_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_header_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_header_get_frame_id(NULL));
}
