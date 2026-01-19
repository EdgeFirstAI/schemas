/**
 * @file test_geometry_msgs.c
 * @brief Criterion tests for geometry_msgs types
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Vector3 Tests
// ============================================================================

Test(geometry_msgs, vector3_create_and_destroy) {
    RosVector3 *vec = ros_vector3_new();
    cr_assert_not_null(vec);

    ros_vector3_set_x(vec, 1.0);
    ros_vector3_set_y(vec, 2.0);
    ros_vector3_set_z(vec, 3.0);

    cr_assert_float_eq(ros_vector3_get_x(vec), 1.0, 0.0001);
    cr_assert_float_eq(ros_vector3_get_y(vec), 2.0, 0.0001);
    cr_assert_float_eq(ros_vector3_get_z(vec), 3.0, 0.0001);

    ros_vector3_free(vec);
}

Test(geometry_msgs, vector3_default_zero) {
    RosVector3 *vec = ros_vector3_new();
    cr_assert_not_null(vec);

    cr_assert_float_eq(ros_vector3_get_x(vec), 0.0, 0.0001);
    cr_assert_float_eq(ros_vector3_get_y(vec), 0.0, 0.0001);
    cr_assert_float_eq(ros_vector3_get_z(vec), 0.0, 0.0001);

    ros_vector3_free(vec);
}

Test(geometry_msgs, vector3_set_values) {
    RosVector3 *vec = ros_vector3_new();
    cr_assert_not_null(vec);

    ros_vector3_set_x(vec, 10.5);
    ros_vector3_set_y(vec, -5.25);
    ros_vector3_set_z(vec, 3.14);

    cr_assert_float_eq(ros_vector3_get_x(vec), 10.5, 0.0001);
    cr_assert_float_eq(ros_vector3_get_y(vec), -5.25, 0.0001);
    cr_assert_float_eq(ros_vector3_get_z(vec), 3.14, 0.0001);

    ros_vector3_free(vec);
}

Test(geometry_msgs, vector3_serialize_deserialize) {
    RosVector3 *original = ros_vector3_new();
    cr_assert_not_null(original);

    ros_vector3_set_x(original, 1.5);
    ros_vector3_set_y(original, 2.5);
    ros_vector3_set_z(original, 3.5);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_vector3_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);

    RosVector3 *deserialized = ros_vector3_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_float_eq(ros_vector3_get_x(deserialized), 1.5, 0.0001);
    cr_assert_float_eq(ros_vector3_get_y(deserialized), 2.5, 0.0001);
    cr_assert_float_eq(ros_vector3_get_z(deserialized), 3.5, 0.0001);

    ros_vector3_free(original);
    ros_vector3_free(deserialized);
    free(buffer);
}

Test(geometry_msgs, vector3_free_null) {
    // Should not crash when freeing NULL
    ros_vector3_free(NULL);
}

// ============================================================================
// Point Tests
// ============================================================================

Test(geometry_msgs, point_create_and_destroy) {
    RosPoint *point = ros_point_new();
    cr_assert_not_null(point);

    ros_point_set_x(point, 10.0);
    ros_point_set_y(point, 20.0);
    ros_point_set_z(point, 30.0);

    cr_assert_float_eq(ros_point_get_x(point), 10.0, 0.0001);
    cr_assert_float_eq(ros_point_get_y(point), 20.0, 0.0001);
    cr_assert_float_eq(ros_point_get_z(point), 30.0, 0.0001);

    ros_point_free(point);
}

Test(geometry_msgs, point_serialize_deserialize) {
    RosPoint *original = ros_point_new();
    cr_assert_not_null(original);

    ros_point_set_x(original, 5.5);
    ros_point_set_y(original, -10.25);
    ros_point_set_z(original, 15.75);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_point_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosPoint *deserialized = ros_point_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_float_eq(ros_point_get_x(deserialized), 5.5, 0.0001);
    cr_assert_float_eq(ros_point_get_y(deserialized), -10.25, 0.0001);
    cr_assert_float_eq(ros_point_get_z(deserialized), 15.75, 0.0001);

    ros_point_free(original);
    ros_point_free(deserialized);
    free(buffer);
}

Test(geometry_msgs, point_free_null) {
    // Should not crash when freeing NULL
    ros_point_free(NULL);
}

// ============================================================================
// Quaternion Tests
// ============================================================================

Test(geometry_msgs, quaternion_create_and_destroy) {
    RosQuaternion *quat = ros_quaternion_new();
    cr_assert_not_null(quat);

    // Set to identity quaternion
    ros_quaternion_set_x(quat, 0.0);
    ros_quaternion_set_y(quat, 0.0);
    ros_quaternion_set_z(quat, 0.0);
    ros_quaternion_set_w(quat, 1.0);

    cr_assert_float_eq(ros_quaternion_get_x(quat), 0.0, 0.0001);
    cr_assert_float_eq(ros_quaternion_get_y(quat), 0.0, 0.0001);
    cr_assert_float_eq(ros_quaternion_get_z(quat), 0.0, 0.0001);
    cr_assert_float_eq(ros_quaternion_get_w(quat), 1.0, 0.0001);

    ros_quaternion_free(quat);
}

Test(geometry_msgs, quaternion_identity) {
    RosQuaternion *quat = ros_quaternion_new();
    cr_assert_not_null(quat);

    ros_quaternion_set_x(quat, 0.0);
    ros_quaternion_set_y(quat, 0.0);
    ros_quaternion_set_z(quat, 0.0);
    ros_quaternion_set_w(quat, 1.0);

    // Identity quaternion should be a unit quaternion
    double x = ros_quaternion_get_x(quat);
    double y = ros_quaternion_get_y(quat);
    double z = ros_quaternion_get_z(quat);
    double w = ros_quaternion_get_w(quat);

    double magnitude = sqrt(x*x + y*y + z*z + w*w);
    cr_assert_float_eq(magnitude, 1.0, 0.0001, "Should be unit quaternion");

    ros_quaternion_free(quat);
}

Test(geometry_msgs, quaternion_serialize_deserialize) {
    RosQuaternion *original = ros_quaternion_new();
    cr_assert_not_null(original);

    ros_quaternion_set_x(original, 0.1);
    ros_quaternion_set_y(original, 0.2);
    ros_quaternion_set_z(original, 0.3);
    ros_quaternion_set_w(original, 0.9);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_quaternion_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosQuaternion *deserialized = ros_quaternion_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_float_eq(ros_quaternion_get_x(deserialized), 0.1, 0.0001);
    cr_assert_float_eq(ros_quaternion_get_y(deserialized), 0.2, 0.0001);
    cr_assert_float_eq(ros_quaternion_get_z(deserialized), 0.3, 0.0001);
    cr_assert_float_eq(ros_quaternion_get_w(deserialized), 0.9, 0.0001);

    ros_quaternion_free(original);
    ros_quaternion_free(deserialized);
    free(buffer);
}

Test(geometry_msgs, quaternion_free_null) {
    // Should not crash when freeing NULL
    ros_quaternion_free(NULL);
}

// ============================================================================
// Point32 Tests
// ============================================================================

Test(geometry_msgs, point32_create_and_destroy) {
    RosPoint32 *point = ros_point32_new();
    cr_assert_not_null(point);

    ros_point32_set_x(point, 1.5f);
    ros_point32_set_y(point, 2.5f);
    ros_point32_set_z(point, 3.5f);

    cr_assert_float_eq(ros_point32_get_x(point), 1.5f, 0.0001f);
    cr_assert_float_eq(ros_point32_get_y(point), 2.5f, 0.0001f);
    cr_assert_float_eq(ros_point32_get_z(point), 3.5f, 0.0001f);

    ros_point32_free(point);
}

Test(geometry_msgs, point32_serialize_deserialize) {
    RosPoint32 *original = ros_point32_new();
    cr_assert_not_null(original);

    ros_point32_set_x(original, 10.5f);
    ros_point32_set_y(original, -20.25f);
    ros_point32_set_z(original, 30.75f);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_point32_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosPoint32 *deserialized = ros_point32_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_float_eq(ros_point32_get_x(deserialized), 10.5f, 0.0001f);
    cr_assert_float_eq(ros_point32_get_y(deserialized), -20.25f, 0.0001f);
    cr_assert_float_eq(ros_point32_get_z(deserialized), 30.75f, 0.0001f);

    ros_point32_free(original);
    ros_point32_free(deserialized);
    free(buffer);
}

Test(geometry_msgs, point32_free_null) {
    ros_point32_free(NULL);
}

// ============================================================================
// Pose Tests
// ============================================================================

Test(geometry_msgs, pose_create_and_destroy) {
    RosPose *pose = ros_pose_new();
    cr_assert_not_null(pose);

    // Get child types (borrowed pointers - don't free!)
    RosPoint *position = ros_pose_get_position_mut(pose);
    cr_assert_not_null(position);
    ros_point_set_x(position, 1.0);
    ros_point_set_y(position, 2.0);
    ros_point_set_z(position, 3.0);

    RosQuaternion *orientation = ros_pose_get_orientation_mut(pose);
    cr_assert_not_null(orientation);
    ros_quaternion_set_x(orientation, 0.0);
    ros_quaternion_set_y(orientation, 0.0);
    ros_quaternion_set_z(orientation, 0.0);
    ros_quaternion_set_w(orientation, 1.0);

    // Verify via const getters
    const RosPoint *pos = ros_pose_get_position(pose);
    cr_assert_float_eq(ros_point_get_x(pos), 1.0, 0.0001);

    const RosQuaternion *ori = ros_pose_get_orientation(pose);
    cr_assert_float_eq(ros_quaternion_get_w(ori), 1.0, 0.0001);

    ros_pose_free(pose);
}

Test(geometry_msgs, pose_serialize_deserialize) {
    RosPose *original = ros_pose_new();
    cr_assert_not_null(original);

    RosPoint *pos = ros_pose_get_position_mut(original);
    ros_point_set_x(pos, 5.0);
    ros_point_set_y(pos, 10.0);
    ros_point_set_z(pos, 15.0);

    RosQuaternion *ori = ros_pose_get_orientation_mut(original);
    ros_quaternion_set_x(ori, 0.1);
    ros_quaternion_set_y(ori, 0.2);
    ros_quaternion_set_z(ori, 0.3);
    ros_quaternion_set_w(ori, 0.9);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_pose_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosPose *deserialized = ros_pose_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    const RosPoint *deser_pos = ros_pose_get_position(deserialized);
    cr_assert_float_eq(ros_point_get_x(deser_pos), 5.0, 0.0001);

    const RosQuaternion *deser_ori = ros_pose_get_orientation(deserialized);
    cr_assert_float_eq(ros_quaternion_get_w(deser_ori), 0.9, 0.0001);

    ros_pose_free(original);
    ros_pose_free(deserialized);
    free(buffer);
}

Test(geometry_msgs, pose_free_null) {
    ros_pose_free(NULL);
}

// ============================================================================
// Pose2D Tests
// ============================================================================

Test(geometry_msgs, pose2d_create_and_destroy) {
    RosPose2D *pose = ros_pose2d_new();
    cr_assert_not_null(pose);

    ros_pose2d_set_x(pose, 10.0);
    ros_pose2d_set_y(pose, 20.0);
    ros_pose2d_set_theta(pose, 3.14159);

    cr_assert_float_eq(ros_pose2d_get_x(pose), 10.0, 0.0001);
    cr_assert_float_eq(ros_pose2d_get_y(pose), 20.0, 0.0001);
    cr_assert_float_eq(ros_pose2d_get_theta(pose), 3.14159, 0.0001);

    ros_pose2d_free(pose);
}

Test(geometry_msgs, pose2d_serialize_deserialize) {
    RosPose2D *original = ros_pose2d_new();
    cr_assert_not_null(original);

    ros_pose2d_set_x(original, -5.5);
    ros_pose2d_set_y(original, 7.25);
    ros_pose2d_set_theta(original, 1.5708);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_pose2d_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosPose2D *deserialized = ros_pose2d_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_float_eq(ros_pose2d_get_x(deserialized), -5.5, 0.0001);
    cr_assert_float_eq(ros_pose2d_get_y(deserialized), 7.25, 0.0001);
    cr_assert_float_eq(ros_pose2d_get_theta(deserialized), 1.5708, 0.0001);

    ros_pose2d_free(original);
    ros_pose2d_free(deserialized);
    free(buffer);
}

Test(geometry_msgs, pose2d_free_null) {
    ros_pose2d_free(NULL);
}

// ============================================================================
// Transform Tests
// ============================================================================

Test(geometry_msgs, transform_create_and_destroy) {
    RosTransform *tf = ros_transform_new();
    cr_assert_not_null(tf);

    RosVector3 *translation = ros_transform_get_translation_mut(tf);
    cr_assert_not_null(translation);
    ros_vector3_set_x(translation, 1.0);
    ros_vector3_set_y(translation, 2.0);
    ros_vector3_set_z(translation, 3.0);

    RosQuaternion *rotation = ros_transform_get_rotation_mut(tf);
    cr_assert_not_null(rotation);
    ros_quaternion_set_w(rotation, 1.0);

    const RosVector3 *trans = ros_transform_get_translation(tf);
    cr_assert_float_eq(ros_vector3_get_x(trans), 1.0, 0.0001);

    ros_transform_free(tf);
}

Test(geometry_msgs, transform_serialize_deserialize) {
    RosTransform *original = ros_transform_new();
    cr_assert_not_null(original);

    RosVector3 *translation = ros_transform_get_translation_mut(original);
    ros_vector3_set_x(translation, 10.0);
    ros_vector3_set_y(translation, 20.0);
    ros_vector3_set_z(translation, 30.0);

    RosQuaternion *rotation = ros_transform_get_rotation_mut(original);
    ros_quaternion_set_x(rotation, 0.0);
    ros_quaternion_set_y(rotation, 0.707);
    ros_quaternion_set_z(rotation, 0.0);
    ros_quaternion_set_w(rotation, 0.707);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_transform_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosTransform *deserialized = ros_transform_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    const RosVector3 *deser_trans = ros_transform_get_translation(deserialized);
    cr_assert_float_eq(ros_vector3_get_x(deser_trans), 10.0, 0.0001);

    const RosQuaternion *deser_rot = ros_transform_get_rotation(deserialized);
    cr_assert_float_eq(ros_quaternion_get_y(deser_rot), 0.707, 0.001);

    ros_transform_free(original);
    ros_transform_free(deserialized);
    free(buffer);
}

Test(geometry_msgs, transform_free_null) {
    ros_transform_free(NULL);
}

// ============================================================================
// Twist Tests
// ============================================================================

Test(geometry_msgs, twist_create_and_destroy) {
    RosTwist *twist = ros_twist_new();
    cr_assert_not_null(twist);

    RosVector3 *linear = ros_twist_get_linear_mut(twist);
    cr_assert_not_null(linear);
    ros_vector3_set_x(linear, 1.0);
    ros_vector3_set_y(linear, 0.0);
    ros_vector3_set_z(linear, 0.0);

    RosVector3 *angular = ros_twist_get_angular_mut(twist);
    cr_assert_not_null(angular);
    ros_vector3_set_z(angular, 0.5);

    const RosVector3 *lin = ros_twist_get_linear(twist);
    cr_assert_float_eq(ros_vector3_get_x(lin), 1.0, 0.0001);

    ros_twist_free(twist);
}

Test(geometry_msgs, twist_serialize_deserialize) {
    RosTwist *original = ros_twist_new();
    cr_assert_not_null(original);

    RosVector3 *linear = ros_twist_get_linear_mut(original);
    ros_vector3_set_x(linear, 2.5);
    ros_vector3_set_y(linear, 0.0);
    ros_vector3_set_z(linear, 0.0);

    RosVector3 *angular = ros_twist_get_angular_mut(original);
    ros_vector3_set_x(angular, 0.0);
    ros_vector3_set_y(angular, 0.0);
    ros_vector3_set_z(angular, 1.0);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_twist_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosTwist *deserialized = ros_twist_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    const RosVector3 *deser_lin = ros_twist_get_linear(deserialized);
    cr_assert_float_eq(ros_vector3_get_x(deser_lin), 2.5, 0.0001);

    const RosVector3 *deser_ang = ros_twist_get_angular(deserialized);
    cr_assert_float_eq(ros_vector3_get_z(deser_ang), 1.0, 0.0001);

    ros_twist_free(original);
    ros_twist_free(deserialized);
    free(buffer);
}

Test(geometry_msgs, twist_free_null) {
    ros_twist_free(NULL);
}

// ============================================================================
// Inertia Tests
// ============================================================================

Test(geometry_msgs, inertia_create_and_destroy) {
    RosInertia *inertia = ros_inertia_new();
    cr_assert_not_null(inertia);

    ros_inertia_set_m(inertia, 10.0);
    ros_inertia_set_ixx(inertia, 1.0);
    ros_inertia_set_iyy(inertia, 2.0);
    ros_inertia_set_izz(inertia, 3.0);
    ros_inertia_set_ixy(inertia, 0.1);
    ros_inertia_set_ixz(inertia, 0.2);
    ros_inertia_set_iyz(inertia, 0.3);

    RosVector3 *com = ros_inertia_get_com_mut(inertia);
    ros_vector3_set_x(com, 0.5);
    ros_vector3_set_y(com, 0.5);
    ros_vector3_set_z(com, 0.5);

    cr_assert_float_eq(ros_inertia_get_m(inertia), 10.0, 0.0001);
    cr_assert_float_eq(ros_inertia_get_ixx(inertia), 1.0, 0.0001);
    cr_assert_float_eq(ros_inertia_get_iyy(inertia), 2.0, 0.0001);
    cr_assert_float_eq(ros_inertia_get_izz(inertia), 3.0, 0.0001);

    const RosVector3 *com_const = ros_inertia_get_com(inertia);
    cr_assert_float_eq(ros_vector3_get_x(com_const), 0.5, 0.0001);

    ros_inertia_free(inertia);
}

Test(geometry_msgs, inertia_serialize_deserialize) {
    RosInertia *original = ros_inertia_new();
    cr_assert_not_null(original);

    ros_inertia_set_m(original, 5.0);
    ros_inertia_set_ixx(original, 0.5);
    ros_inertia_set_iyy(original, 0.5);
    ros_inertia_set_izz(original, 0.5);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_inertia_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosInertia *deserialized = ros_inertia_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_float_eq(ros_inertia_get_m(deserialized), 5.0, 0.0001);
    cr_assert_float_eq(ros_inertia_get_ixx(deserialized), 0.5, 0.0001);

    ros_inertia_free(original);
    ros_inertia_free(deserialized);
    free(buffer);
}

Test(geometry_msgs, inertia_free_null) {
    ros_inertia_free(NULL);
}

// ============================================================================
// InertiaStamped Tests
// ============================================================================

Test(geometry_msgs, inertia_stamped_create_and_destroy) {
    RosInertiaStamped *stamped = ros_inertia_stamped_new();
    cr_assert_not_null(stamped);

    RosHeader *header = ros_inertia_stamped_get_header_mut(stamped);
    cr_assert_not_null(header);
    ros_header_set_frame_id(header, "base_link");

    RosInertia *inertia = ros_inertia_stamped_get_inertia_mut(stamped);
    cr_assert_not_null(inertia);
    ros_inertia_set_m(inertia, 15.0);

    const RosHeader *header_const = ros_inertia_stamped_get_header(stamped);
    char *frame_id = ros_header_get_frame_id(header_const);
    cr_assert_str_eq(frame_id, "base_link");
    free(frame_id);

    const RosInertia *inertia_const = ros_inertia_stamped_get_inertia(stamped);
    cr_assert_float_eq(ros_inertia_get_m(inertia_const), 15.0, 0.0001);

    ros_inertia_stamped_free(stamped);
}

Test(geometry_msgs, inertia_stamped_serialize_deserialize) {
    RosInertiaStamped *original = ros_inertia_stamped_new();
    cr_assert_not_null(original);

    RosHeader *header = ros_inertia_stamped_get_header_mut(original);
    ros_header_set_frame_id(header, "link1");

    RosInertia *inertia = ros_inertia_stamped_get_inertia_mut(original);
    ros_inertia_set_m(inertia, 7.5);
    ros_inertia_set_ixx(inertia, 1.0);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_inertia_stamped_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosInertiaStamped *deserialized = ros_inertia_stamped_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    const RosHeader *deser_header = ros_inertia_stamped_get_header(deserialized);
    char *frame_id = ros_header_get_frame_id(deser_header);
    cr_assert_str_eq(frame_id, "link1");
    free(frame_id);

    const RosInertia *deser_inertia = ros_inertia_stamped_get_inertia(deserialized);
    cr_assert_float_eq(ros_inertia_get_m(deser_inertia), 7.5, 0.0001);

    ros_inertia_stamped_free(original);
    ros_inertia_stamped_free(deserialized);
    free(buffer);
}

Test(geometry_msgs, inertia_stamped_free_null) {
    ros_inertia_stamped_free(NULL);
}
