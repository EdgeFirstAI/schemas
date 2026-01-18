/**
 * @file test_builtin_interfaces.c
 * @brief Criterion tests for builtin_interfaces (Time, Duration)
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Time Tests
// ============================================================================

Test(builtin_interfaces, time_create_and_destroy) {
    Time *time = edgefirst_time_create(12345, 67890);
    cr_assert_not_null(time, "Time creation should succeed");
    
    int32_t sec = edgefirst_time_get_sec(time);
    uint32_t nanosec = edgefirst_time_get_nanosec(time);
    
    cr_assert_eq(sec, 12345, "Seconds should be 12345");
    cr_assert_eq(nanosec, 67890, "Nanoseconds should be 67890");
    
    edgefirst_time_destroy(time);
}

Test(builtin_interfaces, time_create_zero) {
    Time *time = edgefirst_time_create(0, 0);
    cr_assert_not_null(time, "Time creation with zeros should succeed");
    
    cr_assert_eq(edgefirst_time_get_sec(time), 0);
    cr_assert_eq(edgefirst_time_get_nanosec(time), 0);
    
    edgefirst_time_destroy(time);
}

Test(builtin_interfaces, time_create_negative) {
    Time *time = edgefirst_time_create(-100, 500);
    cr_assert_not_null(time, "Time creation with negative sec should succeed");
    
    cr_assert_eq(edgefirst_time_get_sec(time), -100);
    cr_assert_eq(edgefirst_time_get_nanosec(time), 500);
    
    edgefirst_time_destroy(time);
}

Test(builtin_interfaces, time_set_values) {
    Time *time = edgefirst_time_create(0, 0);
    cr_assert_not_null(time);
    
    edgefirst_time_set_sec(time, 99999);
    edgefirst_time_set_nanosec(time, 88888);
    
    cr_assert_eq(edgefirst_time_get_sec(time), 99999);
    cr_assert_eq(edgefirst_time_get_nanosec(time), 88888);
    
    edgefirst_time_destroy(time);
}

Test(builtin_interfaces, time_serialize_deserialize) {
    Time *original = edgefirst_time_create(42, 999999999);
    cr_assert_not_null(original);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_time_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0, "Serialization should succeed");
    cr_assert_not_null(buffer, "Buffer should not be NULL");
    cr_assert_gt(len, 0, "Length should be > 0");
    
    Time *deserialized = edgefirst_time_deserialize(buffer, len);
    cr_assert_not_null(deserialized, "Deserialization should succeed");
    
    cr_assert_eq(edgefirst_time_get_sec(deserialized), 42);
    cr_assert_eq(edgefirst_time_get_nanosec(deserialized), 999999999);
    
    edgefirst_time_destroy(original);
    edgefirst_time_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

Test(builtin_interfaces, time_serialize_null_ptr) {
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    errno = 0;
    int ret = edgefirst_time_serialize(NULL, &buffer, &len);
    cr_assert_eq(ret, -1, "Should return -1 for NULL pointer");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(builtin_interfaces, time_deserialize_null_buffer) {
    errno = 0;
    Time *time = edgefirst_time_deserialize(NULL, 100);
    cr_assert_null(time, "Should return NULL for NULL buffer");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(builtin_interfaces, time_deserialize_zero_length) {
    uint8_t buffer[10] = {0};
    errno = 0;
    Time *time = edgefirst_time_deserialize(buffer, 0);
    cr_assert_null(time, "Should return NULL for zero length");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

// ============================================================================
// Duration Tests
// ============================================================================

Test(builtin_interfaces, duration_create_and_destroy) {
    Duration *duration = edgefirst_duration_create(123, 456789);
    cr_assert_not_null(duration, "Duration creation should succeed");
    
    int32_t sec = edgefirst_duration_get_sec(duration);
    uint32_t nanosec = edgefirst_duration_get_nanosec(duration);
    
    cr_assert_eq(sec, 123, "Seconds should be 123");
    cr_assert_eq(nanosec, 456789, "Nanoseconds should be 456789");
    
    edgefirst_duration_destroy(duration);
}

Test(builtin_interfaces, duration_create_zero) {
    Duration *duration = edgefirst_duration_create(0, 0);
    cr_assert_not_null(duration);
    
    cr_assert_eq(edgefirst_duration_get_sec(duration), 0);
    cr_assert_eq(edgefirst_duration_get_nanosec(duration), 0);
    
    edgefirst_duration_destroy(duration);
}

Test(builtin_interfaces, duration_create_negative) {
    Duration *duration = edgefirst_duration_create(-50, 100);
    cr_assert_not_null(duration);
    
    cr_assert_eq(edgefirst_duration_get_sec(duration), -50);
    cr_assert_eq(edgefirst_duration_get_nanosec(duration), 100);
    
    edgefirst_duration_destroy(duration);
}

Test(builtin_interfaces, duration_set_values) {
    Duration *duration = edgefirst_duration_create(0, 0);
    cr_assert_not_null(duration);
    
    edgefirst_duration_set_sec(duration, 7777);
    edgefirst_duration_set_nanosec(duration, 8888);
    
    cr_assert_eq(edgefirst_duration_get_sec(duration), 7777);
    cr_assert_eq(edgefirst_duration_get_nanosec(duration), 8888);
    
    edgefirst_duration_destroy(duration);
}

Test(builtin_interfaces, duration_serialize_deserialize) {
    Duration *original = edgefirst_duration_create(300, 500000000);
    cr_assert_not_null(original);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_duration_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0, "Serialization should succeed");
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);
    
    Duration *deserialized = edgefirst_duration_deserialize(buffer, len);
    cr_assert_not_null(deserialized, "Deserialization should succeed");
    
    cr_assert_eq(edgefirst_duration_get_sec(deserialized), 300);
    cr_assert_eq(edgefirst_duration_get_nanosec(deserialized), 500000000);
    
    edgefirst_duration_destroy(original);
    edgefirst_duration_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

Test(builtin_interfaces, duration_serialize_null_ptr) {
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    errno = 0;
    int ret = edgefirst_duration_serialize(NULL, &buffer, &len);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

Test(builtin_interfaces, duration_deserialize_invalid) {
    uint8_t bad_data[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    errno = 0;
    Duration *duration = edgefirst_duration_deserialize(bad_data, 4);
    cr_assert_null(duration, "Should return NULL for invalid data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}
