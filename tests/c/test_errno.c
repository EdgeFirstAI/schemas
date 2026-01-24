/**
 * @file test_errno.c
 * @brief Criterion tests for errno error handling across all message types
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Serialization Errno Tests
// ============================================================================

Test(errno_handling, serialize_null_pointer) {
    uint8_t *buffer = NULL;
    size_t len = 0;

    errno = 0;
    int ret = ros_header_serialize(NULL, &buffer, &len);
    cr_assert_eq(ret, -1, "Should return -1 for NULL pointer");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, serialize_null_output_buffer) {
    RosHeader *header = ros_header_new();
    cr_assert_not_null(header);
    size_t len = 0;

    errno = 0;
    int ret = ros_header_serialize(header, NULL, &len);
    cr_assert_eq(ret, -1, "Should return -1 for NULL output buffer");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");

    ros_header_free(header);
}

Test(errno_handling, serialize_null_output_len) {
    RosHeader *header = ros_header_new();
    cr_assert_not_null(header);
    uint8_t *buffer = NULL;

    errno = 0;
    int ret = ros_header_serialize(header, &buffer, NULL);
    cr_assert_eq(ret, -1, "Should return -1 for NULL output length");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");

    ros_header_free(header);
}

// ============================================================================
// Deserialization Errno Tests
// ============================================================================

Test(errno_handling, deserialize_null_buffer) {
    errno = 0;
    RosHeader *header = ros_header_deserialize(NULL, 100);
    cr_assert_null(header, "Should return NULL for NULL buffer");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, deserialize_zero_length) {
    uint8_t buffer[10] = {0};

    errno = 0;
    RosHeader *header = ros_header_deserialize(buffer, 0);
    cr_assert_null(header, "Should return NULL for zero length");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, deserialize_invalid_data) {
    uint8_t bad_data[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    errno = 0;
    RosHeader *header = ros_header_deserialize(bad_data, 4);
    cr_assert_null(header, "Should return NULL for invalid data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(errno_handling, deserialize_truncated_data) {
    // Serialize a valid message
    RosVector3 *original = ros_vector3_new();
    cr_assert_not_null(original);
    ros_vector3_set_x(original, 1.0);
    ros_vector3_set_y(original, 2.0);
    ros_vector3_set_z(original, 3.0);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_vector3_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_gt(len, 4);

    // Try to deserialize with truncated buffer
    errno = 0;
    RosVector3 *deserialized = ros_vector3_deserialize(buffer, 4);
    cr_assert_null(deserialized, "Should return NULL for truncated data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");

    ros_vector3_free(original);
    free(buffer);
}

// ============================================================================
// Setter Errno Tests
// ============================================================================

Test(errno_handling, set_frame_id_null_header) {
    errno = 0;
    int ret = ros_header_set_frame_id(NULL, "test");
    cr_assert_eq(ret, -1, "Should return -1 for NULL header");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, set_frame_id_null_string) {
    RosHeader *header = ros_header_new();
    cr_assert_not_null(header);

    errno = 0;
    int ret = ros_header_set_frame_id(header, NULL);
    cr_assert_eq(ret, -1, "Should return -1 for NULL string");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");

    ros_header_free(header);
}

Test(errno_handling, set_label_null_detectbox2d) {
    errno = 0;
    int ret = edgefirst_box_set_label(NULL, "person");
    cr_assert_eq(ret, -1, "Should return -1 for NULL detectbox2d");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, set_encoding_null_mask) {
    errno = 0;
    int ret = edgefirst_mask_set_encoding(NULL, "zstd");
    cr_assert_eq(ret, -1, "Should return -1 for NULL mask");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, set_data_null_mask) {
    uint8_t data[10] = {0};

    errno = 0;
    int ret = edgefirst_mask_set_mask(NULL, data, 10);
    cr_assert_eq(ret, -1, "Should return -1 for NULL mask");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, set_data_null_buffer) {
    EdgeFirstMask *mask = edgefirst_mask_new();
    cr_assert_not_null(mask);

    errno = 0;
    int ret = edgefirst_mask_set_mask(mask, NULL, 10);
    cr_assert_eq(ret, -1, "Should return -1 for NULL data buffer");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");

    edgefirst_mask_free(mask);
}

Test(errno_handling, set_covariance_null_navsatfix) {
    double covariance[9] = {0};

    errno = 0;
    int ret = ros_nav_sat_fix_set_position_covariance(NULL, covariance);
    cr_assert_eq(ret, -1, "Should return -1 for NULL NavSatFix");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, set_covariance_null_array) {
    RosNavSatFix *fix = ros_nav_sat_fix_new();
    cr_assert_not_null(fix);

    errno = 0;
    int ret = ros_nav_sat_fix_set_position_covariance(fix, NULL);
    cr_assert_eq(ret, -1, "Should return -1 for NULL covariance array");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");

    ros_nav_sat_fix_free(fix);
}

// ============================================================================
// Getter Errno Tests (NULL pointer safety)
// ============================================================================

Test(errno_handling, get_primitive_null_pointer) {
    // Primitive getters should handle NULL gracefully (return 0 or default)
    // Note: These don't set errno, they just return safe defaults
    int32_t sec = ros_time_get_sec(NULL);
    cr_assert_eq(sec, 0, "Should return 0 for NULL pointer");

    uint32_t nanosec = ros_time_get_nanosec(NULL);
    cr_assert_eq(nanosec, 0, "Should return 0 for NULL pointer");
}

Test(errno_handling, get_array_null_pointer) {
    size_t len = 999;
    const uint8_t *data = edgefirst_mask_get_mask(NULL, &len);
    cr_assert_null(data, "Should return NULL for NULL pointer");
    cr_assert_eq(len, 0, "Length should be set to 0");
}

Test(errno_handling, get_array_null_length_output) {
    EdgeFirstMask *mask = edgefirst_mask_new();
    cr_assert_not_null(mask);

    const uint8_t *data = edgefirst_mask_get_mask(mask, NULL);
    cr_assert_null(data, "Should return NULL when length output is NULL");

    edgefirst_mask_free(mask);
}

// ============================================================================
// Multiple Error Scenarios
// ============================================================================

Test(errno_handling, sequential_errors) {
    // Test that errno is properly set for each error

    // First error
    errno = 0;
    RosHeader *header1 = ros_header_deserialize(NULL, 10);
    cr_assert_null(header1);
    cr_assert_eq(errno, EINVAL);

    // Second error (different type)
    errno = 0;
    RosVector3 *vec = ros_vector3_deserialize(NULL, 20);
    cr_assert_null(vec);
    cr_assert_eq(errno, EINVAL);

    // Third error (setter)
    errno = 0;
    int ret = ros_header_set_frame_id(NULL, "test");
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, error_then_success) {
    // Test that successful operations don't set errno

    // Cause an error
    errno = 0;
    RosHeader *header = ros_header_deserialize(NULL, 10);
    cr_assert_null(header);
    cr_assert_eq(errno, EINVAL);

    // Clear errno manually (libraries should not clear errno)
    errno = 0;

    // Successful operation (should not touch errno)
    RosTime *valid = ros_time_new();
    cr_assert_not_null(valid);
    cr_assert_eq(errno, 0, "Successful operation should not set errno");

    ros_time_free(valid);
}

// ============================================================================
// Destructor NULL Safety Tests
// ============================================================================

Test(errno_handling, destroy_null_pointer) {
    // All destroy/free functions should safely handle NULL
    // These should not crash or set errno
    ros_time_free(NULL);
    ros_duration_free(NULL);
    ros_header_free(NULL);
    ros_vector3_free(NULL);
    ros_point_free(NULL);
    ros_quaternion_free(NULL);
    ros_color_rgba_free(NULL);
    edgefirst_track_free(NULL);
    edgefirst_box_free(NULL);
    edgefirst_detect_free(NULL);
    edgefirst_mask_free(NULL);
    edgefirst_dmabuf_free(NULL);
    edgefirst_radarcube_free(NULL);
    ros_point_field_free(NULL);
    ros_point_cloud2_free(NULL);
    ros_nav_sat_status_free(NULL);
    ros_nav_sat_fix_free(NULL);
    foxglove_compressed_video_free(NULL);

    // If we reach here, all free functions handled NULL safely
    cr_assert(1, "All free functions safely handled NULL");
}
