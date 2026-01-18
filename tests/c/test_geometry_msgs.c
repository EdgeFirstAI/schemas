/**
 * @file test_geometry_msgs.c
 * @brief Criterion tests for geometry_msgs types
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <math.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Vector3 Tests
// ============================================================================

Test(geometry_msgs, vector3_create_and_destroy) {
    Vector3 *vec = edgefirst_vector3_create(1.0, 2.0, 3.0);
    cr_assert_not_null(vec);
    
    cr_assert_float_eq(edgefirst_vector3_get_x(vec), 1.0, 0.0001);
    cr_assert_float_eq(edgefirst_vector3_get_y(vec), 2.0, 0.0001);
    cr_assert_float_eq(edgefirst_vector3_get_z(vec), 3.0, 0.0001);
    
    edgefirst_vector3_destroy(vec);
}

Test(geometry_msgs, vector3_set_values) {
    Vector3 *vec = edgefirst_vector3_create(0.0, 0.0, 0.0);
    cr_assert_not_null(vec);
    
    edgefirst_vector3_set_x(vec, 10.5);
    edgefirst_vector3_set_y(vec, -5.25);
    edgefirst_vector3_set_z(vec, 3.14);
    
    cr_assert_float_eq(edgefirst_vector3_get_x(vec), 10.5, 0.0001);
    cr_assert_float_eq(edgefirst_vector3_get_y(vec), -5.25, 0.0001);
    cr_assert_float_eq(edgefirst_vector3_get_z(vec), 3.14, 0.0001);
    
    edgefirst_vector3_destroy(vec);
}

Test(geometry_msgs, vector3_serialize_deserialize) {
    Vector3 *original = edgefirst_vector3_create(1.5, 2.5, 3.5);
    cr_assert_not_null(original);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_vector3_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);
    
    Vector3 *deserialized = edgefirst_vector3_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    cr_assert_float_eq(edgefirst_vector3_get_x(deserialized), 1.5, 0.0001);
    cr_assert_float_eq(edgefirst_vector3_get_y(deserialized), 2.5, 0.0001);
    cr_assert_float_eq(edgefirst_vector3_get_z(deserialized), 3.5, 0.0001);
    
    edgefirst_vector3_destroy(original);
    edgefirst_vector3_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

// ============================================================================
// Point Tests
// ============================================================================

Test(geometry_msgs, point_create_and_destroy) {
    Point *point = edgefirst_point_create(10.0, 20.0, 30.0);
    cr_assert_not_null(point);
    
    cr_assert_float_eq(edgefirst_point_get_x(point), 10.0, 0.0001);
    cr_assert_float_eq(edgefirst_point_get_y(point), 20.0, 0.0001);
    cr_assert_float_eq(edgefirst_point_get_z(point), 30.0, 0.0001);
    
    edgefirst_point_destroy(point);
}

Test(geometry_msgs, point_serialize_deserialize) {
    Point *original = edgefirst_point_create(5.5, -10.25, 15.75);
    cr_assert_not_null(original);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_point_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    Point *deserialized = edgefirst_point_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    cr_assert_float_eq(edgefirst_point_get_x(deserialized), 5.5, 0.0001);
    cr_assert_float_eq(edgefirst_point_get_y(deserialized), -10.25, 0.0001);
    cr_assert_float_eq(edgefirst_point_get_z(deserialized), 15.75, 0.0001);
    
    edgefirst_point_destroy(original);
    edgefirst_point_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

// ============================================================================
// Quaternion Tests
// ============================================================================

Test(geometry_msgs, quaternion_create_and_destroy) {
    Quaternion *quat = edgefirst_quaternion_create(0.0, 0.0, 0.0, 1.0);
    cr_assert_not_null(quat);
    
    cr_assert_float_eq(edgefirst_quaternion_get_x(quat), 0.0, 0.0001);
    cr_assert_float_eq(edgefirst_quaternion_get_y(quat), 0.0, 0.0001);
    cr_assert_float_eq(edgefirst_quaternion_get_z(quat), 0.0, 0.0001);
    cr_assert_float_eq(edgefirst_quaternion_get_w(quat), 1.0, 0.0001);
    
    edgefirst_quaternion_destroy(quat);
}

Test(geometry_msgs, quaternion_identity) {
    Quaternion *quat = edgefirst_quaternion_create(0.0, 0.0, 0.0, 1.0);
    cr_assert_not_null(quat);
    
    // Identity quaternion
    double x = edgefirst_quaternion_get_x(quat);
    double y = edgefirst_quaternion_get_y(quat);
    double z = edgefirst_quaternion_get_z(quat);
    double w = edgefirst_quaternion_get_w(quat);
    
    double magnitude = sqrt(x*x + y*y + z*z + w*w);
    cr_assert_float_eq(magnitude, 1.0, 0.0001, "Should be unit quaternion");
    
    edgefirst_quaternion_destroy(quat);
}

Test(geometry_msgs, quaternion_serialize_deserialize) {
    Quaternion *original = edgefirst_quaternion_create(0.1, 0.2, 0.3, 0.9);
    cr_assert_not_null(original);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_quaternion_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    Quaternion *deserialized = edgefirst_quaternion_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    cr_assert_float_eq(edgefirst_quaternion_get_x(deserialized), 0.1, 0.0001);
    cr_assert_float_eq(edgefirst_quaternion_get_y(deserialized), 0.2, 0.0001);
    cr_assert_float_eq(edgefirst_quaternion_get_z(deserialized), 0.3, 0.0001);
    cr_assert_float_eq(edgefirst_quaternion_get_w(deserialized), 0.9, 0.0001);
    
    edgefirst_quaternion_destroy(original);
    edgefirst_quaternion_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

// ============================================================================
// Pose Tests
// ============================================================================

Test(geometry_msgs, pose_create_and_destroy) {
    Pose *pose = edgefirst_pose_create();
    cr_assert_not_null(pose);
    
    Point *position = edgefirst_pose_get_position(pose);
    cr_assert_not_null(position);
    
    Quaternion *orientation = edgefirst_pose_get_orientation(pose);
    cr_assert_not_null(orientation);
    
    edgefirst_pose_destroy(pose);
}

Test(geometry_msgs, pose_set_position_orientation) {
    Pose *pose = edgefirst_pose_create();
    cr_assert_not_null(pose);
    
    Point *pos = edgefirst_point_create(1.0, 2.0, 3.0);
    Quaternion *orient = edgefirst_quaternion_create(0.0, 0.0, 0.0, 1.0);
    
    edgefirst_pose_set_position(pose, pos);
    edgefirst_pose_set_orientation(pose, orient);
    
    Point *got_pos = edgefirst_pose_get_position(pose);
    cr_assert_float_eq(edgefirst_point_get_x(got_pos), 1.0, 0.0001);
    
    Quaternion *got_orient = edgefirst_pose_get_orientation(pose);
    cr_assert_float_eq(edgefirst_quaternion_get_w(got_orient), 1.0, 0.0001);
    
    edgefirst_point_destroy(pos);
    edgefirst_quaternion_destroy(orient);
    edgefirst_pose_destroy(pose);
}

Test(geometry_msgs, pose_serialize_deserialize) {
    Pose *original = edgefirst_pose_create();
    cr_assert_not_null(original);
    
    Point *pos = edgefirst_point_create(5.0, 10.0, 15.0);
    Quaternion *orient = edgefirst_quaternion_create(0.5, 0.5, 0.5, 0.5);
    
    edgefirst_pose_set_position(original, pos);
    edgefirst_pose_set_orientation(original, orient);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_pose_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    Pose *deserialized = edgefirst_pose_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    Point *deser_pos = edgefirst_pose_get_position(deserialized);
    cr_assert_float_eq(edgefirst_point_get_x(deser_pos), 5.0, 0.0001);
    cr_assert_float_eq(edgefirst_point_get_y(deser_pos), 10.0, 0.0001);
    
    Quaternion *deser_orient = edgefirst_pose_get_orientation(deserialized);
    cr_assert_float_eq(edgefirst_quaternion_get_x(deser_orient), 0.5, 0.0001);
    
    edgefirst_point_destroy(pos);
    edgefirst_quaternion_destroy(orient);
    edgefirst_pose_destroy(original);
    edgefirst_pose_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

// ============================================================================
// Transform Tests
// ============================================================================

Test(geometry_msgs, transform_create_and_destroy) {
    Transform *transform = edgefirst_transform_create();
    cr_assert_not_null(transform);
    
    Vector3 *translation = edgefirst_transform_get_translation(transform);
    cr_assert_not_null(translation);
    
    Quaternion *rotation = edgefirst_transform_get_rotation(transform);
    cr_assert_not_null(rotation);
    
    edgefirst_transform_destroy(transform);
}

Test(geometry_msgs, transform_serialize_deserialize) {
    Transform *original = edgefirst_transform_create();
    cr_assert_not_null(original);
    
    Vector3 *trans = edgefirst_vector3_create(1.0, 2.0, 3.0);
    Quaternion *rot = edgefirst_quaternion_create(0.0, 0.0, 0.707, 0.707);
    
    edgefirst_transform_set_translation(original, trans);
    edgefirst_transform_set_rotation(original, rot);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_transform_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    Transform *deserialized = edgefirst_transform_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    Vector3 *deser_trans = edgefirst_transform_get_translation(deserialized);
    cr_assert_float_eq(edgefirst_vector3_get_x(deser_trans), 1.0, 0.0001);
    
    edgefirst_vector3_destroy(trans);
    edgefirst_quaternion_destroy(rot);
    edgefirst_transform_destroy(original);
    edgefirst_transform_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}
