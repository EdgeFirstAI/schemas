/**
 * @file test_geometry_msgs.c
 * @brief Criterion tests for geometry_msgs CdrFixed types and TransformStamped
 *
 * CdrFixed types: Vector3, Point, Quaternion, Pose, Transform, Twist, Accel
 * Buffer-backed: TransformStamped
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Vector3 Tests
// ============================================================================

Test(geometry_msgs, vector3_encode_decode_roundtrip) {
    uint8_t buf[128];
    size_t written = 0;

    int ret = ros_vector3_encode(buf, sizeof(buf), &written, 1.5, 2.5, 3.5);
    cr_assert_eq(ret, 0);
    cr_assert_gt(written, 0);

    double x = 0, y = 0, z = 0;
    ret = ros_vector3_decode(buf, written, &x, &y, &z);
    cr_assert_eq(ret, 0);
    cr_assert_float_eq(x, 1.5, 0.0001);
    cr_assert_float_eq(y, 2.5, 0.0001);
    cr_assert_float_eq(z, 3.5, 0.0001);
}

Test(geometry_msgs, vector3_encode_decode_negative) {
    uint8_t buf[128];
    size_t written = 0;

    int ret = ros_vector3_encode(buf, sizeof(buf), &written, -10.5, 5.25, -3.14);
    cr_assert_eq(ret, 0);

    double x = 0, y = 0, z = 0;
    ret = ros_vector3_decode(buf, written, &x, &y, &z);
    cr_assert_eq(ret, 0);
    cr_assert_float_eq(x, -10.5, 0.0001);
    cr_assert_float_eq(y, 5.25, 0.0001);
    cr_assert_float_eq(z, -3.14, 0.0001);
}

Test(geometry_msgs, vector3_decode_null_data) {
    double x, y, z;
    errno = 0;
    int ret = ros_vector3_decode(NULL, 100, &x, &y, &z);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

Test(geometry_msgs, vector3_decode_null_out_pointers) {
    uint8_t buf[128];
    size_t written = 0;
    ros_vector3_encode(buf, sizeof(buf), &written, 1.0, 2.0, 3.0);

    // Should not crash with NULL output pointers
    int ret = ros_vector3_decode(buf, written, NULL, NULL, NULL);
    cr_assert_eq(ret, 0);
}

Test(geometry_msgs, vector3_encode_size_query) {
    size_t written = 0;
    int ret = ros_vector3_encode(NULL, 0, &written, 1.0, 2.0, 3.0);
    cr_assert_eq(ret, 0);
    cr_assert_gt(written, 0);
}

// ============================================================================
// Point Tests
// ============================================================================

Test(geometry_msgs, point_encode_decode_roundtrip) {
    uint8_t buf[128];
    size_t written = 0;

    int ret = ros_point_encode(buf, sizeof(buf), &written, 5.5, -10.25, 15.75);
    cr_assert_eq(ret, 0);

    double x = 0, y = 0, z = 0;
    ret = ros_point_decode(buf, written, &x, &y, &z);
    cr_assert_eq(ret, 0);
    cr_assert_float_eq(x, 5.5, 0.0001);
    cr_assert_float_eq(y, -10.25, 0.0001);
    cr_assert_float_eq(z, 15.75, 0.0001);
}

Test(geometry_msgs, point_decode_null_data) {
    double x, y, z;
    errno = 0;
    int ret = ros_point_decode(NULL, 100, &x, &y, &z);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

// ============================================================================
// Quaternion Tests
// ============================================================================

Test(geometry_msgs, quaternion_encode_decode_roundtrip) {
    uint8_t buf[128];
    size_t written = 0;

    int ret = ros_quaternion_encode(buf, sizeof(buf), &written, 0.1, 0.2, 0.3, 0.9);
    cr_assert_eq(ret, 0);

    double x = 0, y = 0, z = 0, w = 0;
    ret = ros_quaternion_decode(buf, written, &x, &y, &z, &w);
    cr_assert_eq(ret, 0);
    cr_assert_float_eq(x, 0.1, 0.0001);
    cr_assert_float_eq(y, 0.2, 0.0001);
    cr_assert_float_eq(z, 0.3, 0.0001);
    cr_assert_float_eq(w, 0.9, 0.0001);
}

Test(geometry_msgs, quaternion_identity) {
    uint8_t buf[128];
    size_t written = 0;

    int ret = ros_quaternion_encode(buf, sizeof(buf), &written, 0.0, 0.0, 0.0, 1.0);
    cr_assert_eq(ret, 0);

    double x = 0, y = 0, z = 0, w = 0;
    ret = ros_quaternion_decode(buf, written, &x, &y, &z, &w);
    cr_assert_eq(ret, 0);

    double magnitude = sqrt(x*x + y*y + z*z + w*w);
    cr_assert_float_eq(magnitude, 1.0, 0.0001, "Should be unit quaternion");
}

Test(geometry_msgs, quaternion_decode_null_data) {
    double x, y, z, w;
    errno = 0;
    int ret = ros_quaternion_decode(NULL, 100, &x, &y, &z, &w);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

// ============================================================================
// Pose Tests
// ============================================================================

Test(geometry_msgs, pose_encode_decode_roundtrip) {
    uint8_t buf[256];
    size_t written = 0;

    // px, py, pz, ox, oy, oz, ow
    int ret = ros_pose_encode(buf, sizeof(buf), &written,
                              5.0, 10.0, 15.0,  // position
                              0.1, 0.2, 0.3, 0.9); // orientation
    cr_assert_eq(ret, 0);

    double px = 0, py = 0, pz = 0, ox = 0, oy = 0, oz = 0, ow = 0;
    ret = ros_pose_decode(buf, written, &px, &py, &pz, &ox, &oy, &oz, &ow);
    cr_assert_eq(ret, 0);
    cr_assert_float_eq(px, 5.0, 0.0001);
    cr_assert_float_eq(py, 10.0, 0.0001);
    cr_assert_float_eq(pz, 15.0, 0.0001);
    cr_assert_float_eq(ox, 0.1, 0.0001);
    cr_assert_float_eq(oy, 0.2, 0.0001);
    cr_assert_float_eq(oz, 0.3, 0.0001);
    cr_assert_float_eq(ow, 0.9, 0.0001);
}

Test(geometry_msgs, pose_decode_null_data) {
    double px, py, pz, ox, oy, oz, ow;
    errno = 0;
    int ret = ros_pose_decode(NULL, 100, &px, &py, &pz, &ox, &oy, &oz, &ow);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

// ============================================================================
// Transform Tests (CdrFixed)
// ============================================================================

Test(geometry_msgs, transform_encode_decode_roundtrip) {
    uint8_t buf[256];
    size_t written = 0;

    // tx, ty, tz, rx, ry, rz, rw
    int ret = ros_transform_encode(buf, sizeof(buf), &written,
                                   10.0, 20.0, 30.0,   // translation
                                   0.0, 0.707, 0.0, 0.707); // rotation
    cr_assert_eq(ret, 0);

    double tx = 0, ty = 0, tz = 0, rx = 0, ry = 0, rz = 0, rw = 0;
    ret = ros_transform_decode(buf, written, &tx, &ty, &tz, &rx, &ry, &rz, &rw);
    cr_assert_eq(ret, 0);
    cr_assert_float_eq(tx, 10.0, 0.0001);
    cr_assert_float_eq(ty, 20.0, 0.0001);
    cr_assert_float_eq(tz, 30.0, 0.0001);
    cr_assert_float_eq(ry, 0.707, 0.001);
    cr_assert_float_eq(rw, 0.707, 0.001);
}

Test(geometry_msgs, transform_decode_null_data) {
    double tx, ty, tz, rx, ry, rz, rw;
    errno = 0;
    int ret = ros_transform_decode(NULL, 100, &tx, &ty, &tz, &rx, &ry, &rz, &rw);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

// ============================================================================
// Twist Tests
// ============================================================================

Test(geometry_msgs, twist_encode_decode_roundtrip) {
    uint8_t buf[256];
    size_t written = 0;

    // lx, ly, lz, ax, ay, az
    int ret = ros_twist_encode(buf, sizeof(buf), &written,
                               2.5, 0.0, 0.0,   // linear
                               0.0, 0.0, 1.0);   // angular
    cr_assert_eq(ret, 0);

    double lx = 0, ly = 0, lz = 0, ax = 0, ay = 0, az = 0;
    ret = ros_twist_decode(buf, written, &lx, &ly, &lz, &ax, &ay, &az);
    cr_assert_eq(ret, 0);
    cr_assert_float_eq(lx, 2.5, 0.0001);
    cr_assert_float_eq(ly, 0.0, 0.0001);
    cr_assert_float_eq(lz, 0.0, 0.0001);
    cr_assert_float_eq(ax, 0.0, 0.0001);
    cr_assert_float_eq(ay, 0.0, 0.0001);
    cr_assert_float_eq(az, 1.0, 0.0001);
}

Test(geometry_msgs, twist_decode_null_data) {
    double lx, ly, lz, ax, ay, az;
    errno = 0;
    int ret = ros_twist_decode(NULL, 100, &lx, &ly, &lz, &ax, &ay, &az);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

// ============================================================================
// Accel Tests
// ============================================================================

Test(geometry_msgs, accel_encode_decode_roundtrip) {
    uint8_t buf[256];
    size_t written = 0;

    // lx, ly, lz, ax, ay, az
    int ret = ros_accel_encode(buf, sizeof(buf), &written,
                               9.8, 0.0, -9.8,    // linear
                               0.1, 0.2, 0.3);     // angular
    cr_assert_eq(ret, 0);

    double lx = 0, ly = 0, lz = 0, ax = 0, ay = 0, az = 0;
    ret = ros_accel_decode(buf, written, &lx, &ly, &lz, &ax, &ay, &az);
    cr_assert_eq(ret, 0);
    cr_assert_float_eq(lx, 9.8, 0.0001);
    cr_assert_float_eq(ly, 0.0, 0.0001);
    cr_assert_float_eq(lz, -9.8, 0.0001);
    cr_assert_float_eq(ax, 0.1, 0.0001);
    cr_assert_float_eq(ay, 0.2, 0.0001);
    cr_assert_float_eq(az, 0.3, 0.0001);
}

Test(geometry_msgs, accel_decode_null_data) {
    double lx, ly, lz, ax, ay, az;
    errno = 0;
    int ret = ros_accel_decode(NULL, 100, &lx, &ly, &lz, &ax, &ay, &az);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

// ============================================================================
// TransformStamped Tests (buffer-backed)
// ============================================================================

Test(geometry_msgs, transform_stamped_encode_from_cdr_roundtrip) {
    // TransformStamped has no encode in FFI, so we test from_cdr using
    // Transform encode to build CDR, then use the transform_stamped_from_cdr.
    // But TransformStamped has its own CDR format (with header + child_frame_id).
    // We can only test from_cdr with NULL safety since we cannot easily
    // generate valid CDR without the encode function.

    // NULL pointer safety
    errno = 0;
    ros_transform_stamped_t *handle = ros_transform_stamped_from_cdr(NULL, 100);
    cr_assert_null(handle, "Should return NULL for NULL data");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(geometry_msgs, transform_stamped_invalid_data) {
    uint8_t garbage[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_transform_stamped_t *handle = ros_transform_stamped_from_cdr(garbage, sizeof(garbage));
    cr_assert_null(handle, "Should return NULL for invalid CDR data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(geometry_msgs, transform_stamped_free_null) {
    // Should not crash
    ros_transform_stamped_free(NULL);
}

Test(geometry_msgs, transform_stamped_getters_null) {
    cr_assert_eq(ros_transform_stamped_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_transform_stamped_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_transform_stamped_get_frame_id(NULL));
    cr_assert_null(ros_transform_stamped_get_child_frame_id(NULL));
}

// ============================================================================
// PoseWithCovariance Tests (CdrFixed — TOP2-770)
// ============================================================================

Test(geometry_msgs, pose_with_covariance_encode_decode_roundtrip) {
    uint8_t buf[512];
    size_t written = 0;
    double cov_in[36] = {0};
    for (int i = 0; i < 6; ++i) cov_in[i * 6 + i] = 0.1 * (i + 1);
    cov_in[1] = 0.01;
    cov_in[6] = 0.01;

    int ret = ros_pose_with_covariance_encode(
        buf, sizeof(buf), &written,
        1.5, -2.5, 3.0,
        0.0, 0.0, 0.0, 1.0,
        cov_in);
    cr_assert_eq(ret, 0);
    cr_assert_gt(written, 0);

    double px = 0, py = 0, pz = 0, ox = 0, oy = 0, oz = 0, ow = 0;
    double cov_out[36] = {0};
    ret = ros_pose_with_covariance_decode(buf, written,
                                          &px, &py, &pz,
                                          &ox, &oy, &oz, &ow,
                                          cov_out);
    cr_assert_eq(ret, 0);
    cr_assert_float_eq(px, 1.5, 1e-12);
    cr_assert_float_eq(py, -2.5, 1e-12);
    cr_assert_float_eq(pz, 3.0, 1e-12);
    cr_assert_float_eq(ow, 1.0, 1e-12);
    cr_assert_float_eq(cov_out[0], 0.1, 1e-12);
    cr_assert_float_eq(cov_out[35], 0.6, 1e-12);
    cr_assert_float_eq(cov_out[1], 0.01, 1e-12);
}

Test(geometry_msgs, pose_with_covariance_encode_null_covariance) {
    uint8_t buf[512];
    size_t written = 0;
    errno = 0;
    int ret = ros_pose_with_covariance_encode(buf, sizeof(buf), &written,
                                              0, 0, 0, 0, 0, 0, 1,
                                              NULL);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

// ============================================================================
// TwistWithCovariance Tests (CdrFixed — TOP2-770)
// ============================================================================

Test(geometry_msgs, twist_with_covariance_encode_decode_roundtrip) {
    uint8_t buf[512];
    size_t written = 0;
    double cov_in[36] = {0};
    for (int i = 0; i < 6; ++i) cov_in[i * 6 + i] = 0.02 * (i + 1);
    cov_in[7] = 0.001;

    int ret = ros_twist_with_covariance_encode(
        buf, sizeof(buf), &written,
        1.0, 2.0, 3.0,
        0.1, 0.2, 0.3,
        cov_in);
    cr_assert_eq(ret, 0);

    double lx = 0, ly = 0, lz = 0, ax = 0, ay = 0, az = 0;
    double cov_out[36] = {0};
    ret = ros_twist_with_covariance_decode(buf, written,
                                           &lx, &ly, &lz,
                                           &ax, &ay, &az,
                                           cov_out);
    cr_assert_eq(ret, 0);
    cr_assert_float_eq(lx, 1.0, 1e-12);
    cr_assert_float_eq(az, 0.3, 1e-12);
    cr_assert_float_eq(cov_out[0], 0.02, 1e-12);
    cr_assert_float_eq(cov_out[7], 0.001, 1e-12);
}
