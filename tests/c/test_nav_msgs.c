/**
 * @file test_nav_msgs.c
 * @brief Criterion tests for nav_msgs types (TOP2-770)
 *
 * Buffer-backed: Odometry
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

static uint8_t *load_fixture_nm(const char *relpath, size_t *out_len) {
    FILE *f = fopen(relpath, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return NULL; }
    uint8_t *buf = (uint8_t *) malloc((size_t) sz);
    size_t got = fread(buf, 1, (size_t) sz, f);
    fclose(f);
    if (got != (size_t) sz) { free(buf); return NULL; }
    *out_len = got;
    return buf;
}

Test(nav_msgs, odometry_from_golden_fixture) {
    size_t len = 0;
    uint8_t *b = load_fixture_nm("testdata/cdr/nav_msgs/Odometry.cdr", &len);
    cr_assert_not_null(b, "failed to load Odometry fixture");
    ros_odometry_t *v = ros_odometry_from_cdr(b, len);
    cr_assert_not_null(v);

    cr_assert_str_eq(ros_odometry_get_frame_id(v), "test_frame");
    cr_assert_str_eq(ros_odometry_get_child_frame_id(v), "base_link");

    double px = 0, py = 0, pz = 0, ox = 0, oy = 0, oz = 0, ow = 0;
    ros_odometry_get_pose(v, &px, &py, &pz, &ox, &oy, &oz, &ow);
    cr_assert_float_eq(px, 1.5, 1e-12);
    cr_assert_float_eq(pz, 3.0, 1e-12);
    cr_assert_float_eq(ow, 1.0, 1e-12);

    double pose_cov[36] = {0};
    ros_odometry_get_pose_covariance(v, pose_cov);
    cr_assert_float_eq(pose_cov[0], 0.1, 1e-12);
    cr_assert_float_eq(pose_cov[1], 0.01, 1e-12);

    double lx = 0, ly = 0, lz = 0, ax = 0, ay = 0, az = 0;
    ros_odometry_get_twist(v, &lx, &ly, &lz, &ax, &ay, &az);
    cr_assert_float_eq(lx, 1.0, 1e-12);
    cr_assert_float_eq(az, 0.3, 1e-12);

    double twist_cov[36] = {0};
    ros_odometry_get_twist_covariance(v, twist_cov);
    cr_assert_float_eq(twist_cov[0], 0.02, 1e-12);
    cr_assert_float_eq(twist_cov[7], 0.001, 1e-12);

    ros_odometry_free(v);
    free(b);
}

Test(nav_msgs, odometry_from_cdr_null) {
    errno = 0;
    cr_assert_null(ros_odometry_from_cdr(NULL, 100));
    cr_assert_eq(errno, EINVAL);
}

Test(nav_msgs, odometry_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    cr_assert_null(ros_odometry_from_cdr(bad, sizeof(bad)));
    cr_assert_eq(errno, EBADMSG);
}

Test(nav_msgs, odometry_free_null) {
    ros_odometry_free(NULL);
}

Test(nav_msgs, odometry_getters_null) {
    cr_assert_eq(ros_odometry_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_odometry_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_odometry_get_frame_id(NULL));
    cr_assert_null(ros_odometry_get_child_frame_id(NULL));
    /* Pointer-out getters must be safe on NULL view. */
    ros_odometry_get_pose(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ros_odometry_get_pose_covariance(NULL, NULL);
    ros_odometry_get_twist(NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    ros_odometry_get_twist_covariance(NULL, NULL);
}
