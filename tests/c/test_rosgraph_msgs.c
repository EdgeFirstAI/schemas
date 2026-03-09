/**
 * @file test_rosgraph_msgs.c
 * @brief Criterion tests for rosgraph_msgs types
 *
 * Note: Clock type has no FFI bindings in the current v2 buffer-view API.
 * The Clock type uses builtin_interfaces::Time which is available via
 * ros_time_encode/ros_time_decode. This file is a placeholder.
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Clock is represented as Time — tested via builtin_interfaces
// ============================================================================

Test(rosgraph_msgs, clock_via_time_encode_decode) {
    // Clock contains a single Time field. We exercise Time encode/decode
    // as a proxy for Clock functionality.
    uint8_t buf[64];
    size_t written = 0;

    int ret = ros_time_encode(buf, sizeof(buf), &written, 1000000, 123456789);
    cr_assert_eq(ret, 0);

    int32_t sec = 0;
    uint32_t nanosec = 0;
    ret = ros_time_decode(buf, written, &sec, &nanosec);
    cr_assert_eq(ret, 0);
    cr_assert_eq(sec, 1000000);
    cr_assert_eq(nanosec, 123456789);
}
