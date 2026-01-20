/**
 * @file test_rosgraph_msgs.c
 * @brief Criterion tests for rosgraph_msgs types
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <stdlib.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Clock Tests
// ============================================================================

Test(rosgraph_msgs, clock_create_and_destroy) {
    RosClock *clock = ros_clock_new();
    cr_assert_not_null(clock);

    RosTime *time = ros_clock_get_clock_mut(clock);
    cr_assert_not_null(time);
    ros_time_set_sec(time, 12345);
    ros_time_set_nanosec(time, 67890);

    const RosTime *time_const = ros_clock_get_clock(clock);
    cr_assert_eq(ros_time_get_sec(time_const), 12345);
    cr_assert_eq(ros_time_get_nanosec(time_const), 67890);

    ros_clock_free(clock);
}

Test(rosgraph_msgs, clock_default_zero) {
    RosClock *clock = ros_clock_new();
    cr_assert_not_null(clock);

    const RosTime *time = ros_clock_get_clock(clock);
    cr_assert_eq(ros_time_get_sec(time), 0);
    cr_assert_eq(ros_time_get_nanosec(time), 0);

    ros_clock_free(clock);
}

Test(rosgraph_msgs, clock_serialize_deserialize) {
    RosClock *original = ros_clock_new();
    cr_assert_not_null(original);

    RosTime *time = ros_clock_get_clock_mut(original);
    ros_time_set_sec(time, 1000000);
    ros_time_set_nanosec(time, 123456789);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_clock_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);

    RosClock *deserialized = ros_clock_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    const RosTime *deser_time = ros_clock_get_clock(deserialized);
    cr_assert_eq(ros_time_get_sec(deser_time), 1000000);
    cr_assert_eq(ros_time_get_nanosec(deser_time), 123456789);

    ros_clock_free(original);
    ros_clock_free(deserialized);
    free(buffer);
}

Test(rosgraph_msgs, clock_free_null) {
    ros_clock_free(NULL);
}

Test(rosgraph_msgs, clock_deserialize_null) {
    errno = 0;
    RosClock *clock = ros_clock_deserialize(NULL, 10);
    cr_assert_null(clock);
    cr_assert_eq(errno, EINVAL);
}

Test(rosgraph_msgs, clock_deserialize_empty) {
    uint8_t buffer[1] = {0};
    errno = 0;
    RosClock *clock = ros_clock_deserialize(buffer, 0);
    cr_assert_null(clock);
    cr_assert_eq(errno, EINVAL);
}

Test(rosgraph_msgs, clock_serialize_null) {
    uint8_t *buffer = NULL;
    size_t len = 0;

    errno = 0;
    int ret = ros_clock_serialize(NULL, &buffer, &len);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}
