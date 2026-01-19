/**
 * @file test_builtin_interfaces.c
 * @brief Criterion tests for builtin_interfaces (Time, Duration)
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

Test(builtin_interfaces, time_create_and_destroy) {
    RosTime *t = ros_time_new();
    cr_assert_not_null(t, "Time creation should succeed");

    ros_time_set_sec(t, 12345);
    ros_time_set_nanosec(t, 67890);

    int32_t sec = ros_time_get_sec(t);
    uint32_t nanosec = ros_time_get_nanosec(t);

    cr_assert_eq(sec, 12345, "Seconds should be 12345");
    cr_assert_eq(nanosec, 67890, "Nanoseconds should be 67890");

    ros_time_free(t);
}

Test(builtin_interfaces, time_create_zero) {
    RosTime *t = ros_time_new();
    cr_assert_not_null(t, "Time creation should succeed");

    // Default values should be zero
    cr_assert_eq(ros_time_get_sec(t), 0);
    cr_assert_eq(ros_time_get_nanosec(t), 0);

    ros_time_free(t);
}

Test(builtin_interfaces, time_create_negative) {
    RosTime *t = ros_time_new();
    cr_assert_not_null(t, "Time creation should succeed");

    ros_time_set_sec(t, -100);
    ros_time_set_nanosec(t, 500);

    cr_assert_eq(ros_time_get_sec(t), -100);
    cr_assert_eq(ros_time_get_nanosec(t), 500);

    ros_time_free(t);
}

Test(builtin_interfaces, time_set_values) {
    RosTime *t = ros_time_new();
    cr_assert_not_null(t);

    ros_time_set_sec(t, 99999);
    ros_time_set_nanosec(t, 88888);

    cr_assert_eq(ros_time_get_sec(t), 99999);
    cr_assert_eq(ros_time_get_nanosec(t), 88888);

    ros_time_free(t);
}

Test(builtin_interfaces, time_serialize_deserialize) {
    RosTime *original = ros_time_new();
    cr_assert_not_null(original);

    ros_time_set_sec(original, 42);
    ros_time_set_nanosec(original, 999999999);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_time_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0, "Serialization should succeed");
    cr_assert_not_null(buffer, "Buffer should not be NULL");
    cr_assert_gt(len, 0, "Length should be > 0");

    RosTime *deserialized = ros_time_deserialize(buffer, len);
    cr_assert_not_null(deserialized, "Deserialization should succeed");

    cr_assert_eq(ros_time_get_sec(deserialized), 42);
    cr_assert_eq(ros_time_get_nanosec(deserialized), 999999999);

    ros_time_free(original);
    ros_time_free(deserialized);
    free(buffer);
}

Test(builtin_interfaces, time_serialize_null_ptr) {
    uint8_t *buffer = NULL;
    size_t len = 0;

    errno = 0;
    int ret = ros_time_serialize(NULL, &buffer, &len);
    cr_assert_eq(ret, -1, "Should return -1 for NULL pointer");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(builtin_interfaces, time_deserialize_null_buffer) {
    errno = 0;
    RosTime *t = ros_time_deserialize(NULL, 100);
    cr_assert_null(t, "Should return NULL for NULL buffer");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(builtin_interfaces, time_deserialize_zero_length) {
    uint8_t buffer[10] = {0};
    errno = 0;
    RosTime *t = ros_time_deserialize(buffer, 0);
    cr_assert_null(t, "Should return NULL for zero length");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(builtin_interfaces, time_free_null) {
    // Should not crash when freeing NULL
    ros_time_free(NULL);
}

// ============================================================================
// Duration Tests
// ============================================================================

Test(builtin_interfaces, duration_create_and_destroy) {
    RosDuration *duration = ros_duration_new();
    cr_assert_not_null(duration, "Duration creation should succeed");

    ros_duration_set_sec(duration, 123);
    ros_duration_set_nanosec(duration, 456789);

    int32_t sec = ros_duration_get_sec(duration);
    uint32_t nanosec = ros_duration_get_nanosec(duration);

    cr_assert_eq(sec, 123, "Seconds should be 123");
    cr_assert_eq(nanosec, 456789, "Nanoseconds should be 456789");

    ros_duration_free(duration);
}

Test(builtin_interfaces, duration_create_zero) {
    RosDuration *duration = ros_duration_new();
    cr_assert_not_null(duration);

    // Default values should be zero
    cr_assert_eq(ros_duration_get_sec(duration), 0);
    cr_assert_eq(ros_duration_get_nanosec(duration), 0);

    ros_duration_free(duration);
}

Test(builtin_interfaces, duration_create_negative) {
    RosDuration *duration = ros_duration_new();
    cr_assert_not_null(duration);

    ros_duration_set_sec(duration, -50);
    ros_duration_set_nanosec(duration, 100);

    cr_assert_eq(ros_duration_get_sec(duration), -50);
    cr_assert_eq(ros_duration_get_nanosec(duration), 100);

    ros_duration_free(duration);
}

Test(builtin_interfaces, duration_set_values) {
    RosDuration *duration = ros_duration_new();
    cr_assert_not_null(duration);

    ros_duration_set_sec(duration, 7777);
    ros_duration_set_nanosec(duration, 8888);

    cr_assert_eq(ros_duration_get_sec(duration), 7777);
    cr_assert_eq(ros_duration_get_nanosec(duration), 8888);

    ros_duration_free(duration);
}

Test(builtin_interfaces, duration_serialize_deserialize) {
    RosDuration *original = ros_duration_new();
    cr_assert_not_null(original);

    ros_duration_set_sec(original, 300);
    ros_duration_set_nanosec(original, 500000000);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_duration_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0, "Serialization should succeed");
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);

    RosDuration *deserialized = ros_duration_deserialize(buffer, len);
    cr_assert_not_null(deserialized, "Deserialization should succeed");

    cr_assert_eq(ros_duration_get_sec(deserialized), 300);
    cr_assert_eq(ros_duration_get_nanosec(deserialized), 500000000);

    ros_duration_free(original);
    ros_duration_free(deserialized);
    free(buffer);
}

Test(builtin_interfaces, duration_serialize_null_ptr) {
    uint8_t *buffer = NULL;
    size_t len = 0;

    errno = 0;
    int ret = ros_duration_serialize(NULL, &buffer, &len);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

Test(builtin_interfaces, duration_deserialize_invalid) {
    uint8_t bad_data[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    errno = 0;
    RosDuration *duration = ros_duration_deserialize(bad_data, 4);
    cr_assert_null(duration, "Should return NULL for invalid data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(builtin_interfaces, duration_free_null) {
    // Should not crash when freeing NULL
    ros_duration_free(NULL);
}
