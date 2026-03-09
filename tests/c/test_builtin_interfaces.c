/**
 * @file test_builtin_interfaces.c
 * @brief Criterion tests for builtin_interfaces (Time, Duration)
 *
 * Tests the CdrFixed encode/decode API:
 *   ros_time_encode(buf, cap, &written, sec, nanosec) -> int
 *   ros_time_decode(data, len, &sec, &nanosec) -> int
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Time Tests
// ============================================================================

Test(builtin_interfaces, time_encode_decode_roundtrip) {
    uint8_t buf[64];
    size_t written = 0;

    int ret = ros_time_encode(buf, sizeof(buf), &written, 12345, 67890);
    cr_assert_eq(ret, 0, "Encode should succeed");
    cr_assert_gt(written, 0, "Written bytes should be > 0");

    int32_t sec = 0;
    uint32_t nanosec = 0;
    ret = ros_time_decode(buf, written, &sec, &nanosec);
    cr_assert_eq(ret, 0, "Decode should succeed");
    cr_assert_eq(sec, 12345, "Seconds should be 12345");
    cr_assert_eq(nanosec, 67890, "Nanoseconds should be 67890");
}

Test(builtin_interfaces, time_encode_decode_zero) {
    uint8_t buf[64];
    size_t written = 0;

    int ret = ros_time_encode(buf, sizeof(buf), &written, 0, 0);
    cr_assert_eq(ret, 0);

    int32_t sec = -1;
    uint32_t nanosec = 1;
    ret = ros_time_decode(buf, written, &sec, &nanosec);
    cr_assert_eq(ret, 0);
    cr_assert_eq(sec, 0);
    cr_assert_eq(nanosec, 0);
}

Test(builtin_interfaces, time_encode_decode_negative) {
    uint8_t buf[64];
    size_t written = 0;

    int ret = ros_time_encode(buf, sizeof(buf), &written, -100, 500);
    cr_assert_eq(ret, 0);

    int32_t sec = 0;
    uint32_t nanosec = 0;
    ret = ros_time_decode(buf, written, &sec, &nanosec);
    cr_assert_eq(ret, 0);
    cr_assert_eq(sec, -100);
    cr_assert_eq(nanosec, 500);
}

Test(builtin_interfaces, time_encode_decode_max_nanosec) {
    uint8_t buf[64];
    size_t written = 0;

    int ret = ros_time_encode(buf, sizeof(buf), &written, 42, 999999999);
    cr_assert_eq(ret, 0);

    int32_t sec = 0;
    uint32_t nanosec = 0;
    ret = ros_time_decode(buf, written, &sec, &nanosec);
    cr_assert_eq(ret, 0);
    cr_assert_eq(sec, 42);
    cr_assert_eq(nanosec, 999999999);
}

Test(builtin_interfaces, time_encode_size_query) {
    // Pass NULL buffer to query required size
    size_t written = 0;
    int ret = ros_time_encode(NULL, 0, &written, 1, 2);
    cr_assert_eq(ret, 0, "Size query should succeed");
    cr_assert_gt(written, 0, "Required size should be > 0");
}

Test(builtin_interfaces, time_encode_buffer_too_small) {
    uint8_t buf[2]; // Way too small
    size_t written = 0;

    errno = 0;
    int ret = ros_time_encode(buf, sizeof(buf), &written, 1, 2);
    cr_assert_eq(ret, -1, "Should fail with small buffer");
    cr_assert_eq(errno, ENOBUFS, "errno should be ENOBUFS");
}

Test(builtin_interfaces, time_decode_null_data) {
    int32_t sec = 0;
    uint32_t nanosec = 0;

    errno = 0;
    int ret = ros_time_decode(NULL, 100, &sec, &nanosec);
    cr_assert_eq(ret, -1, "Should fail with NULL data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(builtin_interfaces, time_decode_zero_length) {
    uint8_t buf[10] = {0};

    errno = 0;
    int32_t sec = 0;
    uint32_t nanosec = 0;
    int ret = ros_time_decode(buf, 0, &sec, &nanosec);
    cr_assert_eq(ret, -1, "Should fail with zero length");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(builtin_interfaces, time_decode_null_out_pointers) {
    // Encode valid data first
    uint8_t buf[64];
    size_t written = 0;
    int ret = ros_time_encode(buf, sizeof(buf), &written, 10, 20);
    cr_assert_eq(ret, 0);

    // Decode with NULL output pointers -- should not crash
    ret = ros_time_decode(buf, written, NULL, NULL);
    cr_assert_eq(ret, 0, "Decode with NULL out pointers should succeed");
}

// ============================================================================
// Duration Tests
// ============================================================================

Test(builtin_interfaces, duration_encode_decode_roundtrip) {
    uint8_t buf[64];
    size_t written = 0;

    int ret = ros_duration_encode(buf, sizeof(buf), &written, 123, 456789);
    cr_assert_eq(ret, 0);
    cr_assert_gt(written, 0);

    int32_t sec = 0;
    uint32_t nanosec = 0;
    ret = ros_duration_decode(buf, written, &sec, &nanosec);
    cr_assert_eq(ret, 0);
    cr_assert_eq(sec, 123);
    cr_assert_eq(nanosec, 456789);
}

Test(builtin_interfaces, duration_encode_decode_negative) {
    uint8_t buf[64];
    size_t written = 0;

    int ret = ros_duration_encode(buf, sizeof(buf), &written, -50, 100);
    cr_assert_eq(ret, 0);

    int32_t sec = 0;
    uint32_t nanosec = 0;
    ret = ros_duration_decode(buf, written, &sec, &nanosec);
    cr_assert_eq(ret, 0);
    cr_assert_eq(sec, -50);
    cr_assert_eq(nanosec, 100);
}

Test(builtin_interfaces, duration_encode_decode_large) {
    uint8_t buf[64];
    size_t written = 0;

    int ret = ros_duration_encode(buf, sizeof(buf), &written, 300, 500000000);
    cr_assert_eq(ret, 0);

    int32_t sec = 0;
    uint32_t nanosec = 0;
    ret = ros_duration_decode(buf, written, &sec, &nanosec);
    cr_assert_eq(ret, 0);
    cr_assert_eq(sec, 300);
    cr_assert_eq(nanosec, 500000000);
}

Test(builtin_interfaces, duration_decode_invalid_data) {
    uint8_t bad_data[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    errno = 0;
    int32_t sec = 0;
    uint32_t nanosec = 0;
    int ret = ros_duration_decode(bad_data, 2, &sec, &nanosec);
    cr_assert_eq(ret, -1, "Should fail with invalid data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(builtin_interfaces, duration_decode_null_data) {
    errno = 0;
    int32_t sec = 0;
    uint32_t nanosec = 0;
    int ret = ros_duration_decode(NULL, 100, &sec, &nanosec);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EBADMSG);
}

Test(builtin_interfaces, duration_encode_size_query) {
    size_t written = 0;
    int ret = ros_duration_encode(NULL, 0, &written, 7777, 8888);
    cr_assert_eq(ret, 0, "Size query should succeed");
    cr_assert_gt(written, 0);
}
