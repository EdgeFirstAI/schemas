/**
 * @file test_errno.c
 * @brief Criterion tests for errno error handling across all message types
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Serialization Errno Tests
// ============================================================================

Test(errno_handling, serialize_null_pointer) {
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    errno = 0;
    int ret = edgefirst_time_serialize(NULL, &buffer, &len);
    cr_assert_eq(ret, -1, "Should return -1 for NULL pointer");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, serialize_null_output_buffer) {
    Time *time = edgefirst_time_create(1, 2);
    size_t len = 0;
    
    errno = 0;
    int ret = edgefirst_time_serialize(time, NULL, &len);
    cr_assert_eq(ret, -1, "Should return -1 for NULL output buffer");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
    
    edgefirst_time_destroy(time);
}

Test(errno_handling, serialize_null_output_len) {
    Time *time = edgefirst_time_create(1, 2);
    uint8_t *buffer = NULL;
    
    errno = 0;
    int ret = edgefirst_time_serialize(time, &buffer, NULL);
    cr_assert_eq(ret, -1, "Should return -1 for NULL output length");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
    
    edgefirst_time_destroy(time);
}

// ============================================================================
// Deserialization Errno Tests
// ============================================================================

Test(errno_handling, deserialize_null_buffer) {
    errno = 0;
    Time *time = edgefirst_time_deserialize(NULL, 100);
    cr_assert_null(time, "Should return NULL for NULL buffer");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, deserialize_zero_length) {
    uint8_t buffer[10] = {0};
    
    errno = 0;
    Duration *duration = edgefirst_duration_deserialize(buffer, 0);
    cr_assert_null(duration, "Should return NULL for zero length");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, deserialize_invalid_data) {
    uint8_t bad_data[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    
    errno = 0;
    Header *header = edgefirst_header_deserialize(bad_data, 4);
    cr_assert_null(header, "Should return NULL for invalid data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(errno_handling, deserialize_truncated_data) {
    // Serialize a valid message
    Vector3 *original = edgefirst_vector3_create(1.0, 2.0, 3.0);
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_vector3_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_gt(len, 4);
    
    // Try to deserialize with truncated buffer
    errno = 0;
    Vector3 *deserialized = edgefirst_vector3_deserialize(buffer, 4);
    cr_assert_null(deserialized, "Should return NULL for truncated data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
    
    edgefirst_vector3_destroy(original);
    edgefirst_buffer_destroy(buffer);
}

// ============================================================================
// Setter Errno Tests
// ============================================================================

Test(errno_handling, set_frame_id_null_header) {
    errno = 0;
    int ret = edgefirst_header_set_frame_id(NULL, "test");
    cr_assert_eq(ret, -1, "Should return -1 for NULL header");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, set_frame_id_null_string) {
    Header *header = edgefirst_header_create();
    
    errno = 0;
    int ret = edgefirst_header_set_frame_id(header, NULL);
    cr_assert_eq(ret, -1, "Should return -1 for NULL string");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
    
    edgefirst_header_destroy(header);
}

Test(errno_handling, set_label_null_detect) {
    errno = 0;
    int ret = edgefirst_detect_set_label(NULL, "person");
    cr_assert_eq(ret, -1, "Should return -1 for NULL detect");
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
    int ret = edgefirst_mask_set_data(NULL, data, 10);
    cr_assert_eq(ret, -1, "Should return -1 for NULL mask");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, set_data_null_buffer) {
    Mask *mask = edgefirst_mask_create();
    
    errno = 0;
    int ret = edgefirst_mask_set_data(mask, NULL, 10);
    cr_assert_eq(ret, -1, "Should return -1 for NULL data buffer");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
    
    edgefirst_mask_destroy(mask);
}

Test(errno_handling, set_covariance_null_navsatfix) {
    double covariance[9] = {0};
    
    errno = 0;
    int ret = edgefirst_navsatfix_set_position_covariance(NULL, covariance);
    cr_assert_eq(ret, -1, "Should return -1 for NULL NavSatFix");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, set_covariance_null_array) {
    NavSatFix *fix = edgefirst_navsatfix_create();
    
    errno = 0;
    int ret = edgefirst_navsatfix_set_position_covariance(fix, NULL);
    cr_assert_eq(ret, -1, "Should return -1 for NULL covariance array");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
    
    edgefirst_navsatfix_destroy(fix);
}

// ============================================================================
// Getter Errno Tests (NULL pointer safety)
// ============================================================================

Test(errno_handling, get_primitive_null_pointer) {
    // Primitive getters should handle NULL gracefully (return 0 or default)
    // Note: These don't set errno, they just return safe defaults
    int32_t sec = edgefirst_time_get_sec(NULL);
    cr_assert_eq(sec, 0, "Should return 0 for NULL pointer");
    
    uint32_t nanosec = edgefirst_time_get_nanosec(NULL);
    cr_assert_eq(nanosec, 0, "Should return 0 for NULL pointer");
}

Test(errno_handling, get_string_null_pointer) {
    // String getters should return empty string for NULL
    const char *frame_id = edgefirst_header_get_frame_id(NULL);
    cr_assert_str_eq(frame_id, "", "Should return empty string for NULL pointer");
}

Test(errno_handling, get_array_null_pointer) {
    size_t len = 999;
    const uint8_t *data = edgefirst_mask_get_data(NULL, &len);
    cr_assert_null(data, "Should return NULL for NULL pointer");
    cr_assert_eq(len, 0, "Length should be set to 0");
}

Test(errno_handling, get_array_null_length_output) {
    Mask *mask = edgefirst_mask_create();
    
    const uint8_t *data = edgefirst_mask_get_data(mask, NULL);
    cr_assert_null(data, "Should return NULL when length output is NULL");
    
    edgefirst_mask_destroy(mask);
}

// ============================================================================
// Multiple Error Scenarios
// ============================================================================

Test(errno_handling, sequential_errors) {
    // Test that errno is properly set for each error
    
    // First error
    errno = 0;
    Time *time1 = edgefirst_time_deserialize(NULL, 10);
    cr_assert_null(time1);
    cr_assert_eq(errno, EINVAL);
    
    // Second error (different type)
    errno = 0;
    Header *header = edgefirst_header_deserialize(NULL, 20);
    cr_assert_null(header);
    cr_assert_eq(errno, EINVAL);
    
    // Third error (setter)
    errno = 0;
    int ret = edgefirst_header_set_frame_id(NULL, "test");
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, error_then_success) {
    // Test that successful operations don't set errno
    
    // Cause an error
    errno = 0;
    Time *time = edgefirst_time_deserialize(NULL, 10);
    cr_assert_null(time);
    cr_assert_eq(errno, EINVAL);
    
    // Clear errno manually (libraries should not clear errno)
    errno = 0;
    
    // Successful operation (should not touch errno)
    Time *valid = edgefirst_time_create(1, 2);
    cr_assert_not_null(valid);
    cr_assert_eq(errno, 0, "Successful operation should not set errno");
    
    edgefirst_time_destroy(valid);
}

// ============================================================================
// Destructor NULL Safety Tests
// ============================================================================

Test(errno_handling, destroy_null_pointer) {
    // All destroy functions should safely handle NULL
    // These should not crash or set errno
    edgefirst_time_destroy(NULL);
    edgefirst_duration_destroy(NULL);
    edgefirst_header_destroy(NULL);
    edgefirst_vector3_destroy(NULL);
    edgefirst_point_destroy(NULL);
    edgefirst_quaternion_destroy(NULL);
    edgefirst_pose_destroy(NULL);
    edgefirst_transform_destroy(NULL);
    edgefirst_colorrgba_destroy(NULL);
    edgefirst_detect_track_destroy(NULL);
    edgefirst_detect_box2d_destroy(NULL);
    edgefirst_detect_destroy(NULL);
    edgefirst_mask_destroy(NULL);
    edgefirst_dmabuf_destroy(NULL);
    edgefirst_pointfield_destroy(NULL);
    edgefirst_pointcloud2_destroy(NULL);
    edgefirst_navsatstatus_destroy(NULL);
    edgefirst_navsatfix_destroy(NULL);
    edgefirst_buffer_destroy(NULL);
    
    // If we reach here, all destroy functions handled NULL safely
    cr_assert(1, "All destroy functions safely handled NULL");
}
