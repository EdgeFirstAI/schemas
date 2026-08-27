/**
 * @file test_errno.c
 * @brief Criterion tests for errno error handling across all message types
 *
 * Tests error behaviour for the v2 buffer-view API:
 *   - CdrFixed encode/decode: EBADMSG on bad data, ENOBUFS on small buffer
 *   - Buffer-backed from_cdr: EINVAL for NULL, EBADMSG for invalid CDR
 *   - NULL pointer safety on all getters
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

// ============================================================================
// CdrFixed encode errno tests
// ============================================================================

Test(errno_handling, encode_buffer_too_small) {
    uint8_t buf[2]; // Way too small for any type
    size_t written = 0;

    errno = 0;
    int ret = builtin_interfaces_time_encode(buf, sizeof(buf), &written, 1, 2);
    cr_assert_eq(ret, -1, "Should return -1 for small buffer");
    cr_assert_eq(errno, ENOBUFS, "errno should be ENOBUFS");
}

Test(errno_handling, encode_size_query_succeeds) {
    size_t written = 0;

    // NULL buffer should succeed as a size query
    errno = 0;
    int ret = builtin_interfaces_time_encode(NULL, 0, &written, 1, 2);
    cr_assert_eq(ret, 0, "Size query should succeed");
    cr_assert_gt(written, 0);
}

// ============================================================================
// CdrFixed decode errno tests
// ============================================================================

Test(errno_handling, decode_null_data) {
    int32_t sec;
    uint32_t nanosec;

    errno = 0;
    int ret = builtin_interfaces_time_decode(NULL, 100, &sec, &nanosec);
    cr_assert_eq(ret, -1, "Should return -1 for NULL data");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL for NULL data");
}

Test(errno_handling, decode_too_short) {
    uint8_t buf[2] = {0x00, 0x01};

    errno = 0;
    int32_t sec;
    uint32_t nanosec;
    int ret = builtin_interfaces_time_decode(buf, sizeof(buf), &sec, &nanosec);
    cr_assert_eq(ret, -1, "Should return -1 for truncated data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(errno_handling, decode_truncated_vector3) {
    // Encode valid data, then try to decode with truncated buffer
    uint8_t buf[128];
    size_t written = 0;

    int ret = geometry_msgs_vector3_encode(buf, sizeof(buf), &written, 1.0, 2.0, 3.0);
    cr_assert_eq(ret, 0);
    cr_assert_gt(written, 4);

    double x, y, z;
    errno = 0;
    ret = geometry_msgs_vector3_decode(buf, 4, &x, &y, &z);
    cr_assert_eq(ret, -1, "Should return -1 for truncated data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

// ============================================================================
// Buffer-backed from_cdr errno tests
// ============================================================================

Test(errno_handling, from_cdr_null_header) {
    errno = 0;
    std_msgs_header_t *handle = std_msgs_header_from_cdr(NULL, 100);
    cr_assert_null(handle, "Should return NULL for NULL data");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, from_cdr_invalid_header) {
    uint8_t bad_data[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    errno = 0;
    std_msgs_header_t *handle = std_msgs_header_from_cdr(bad_data, sizeof(bad_data));
    cr_assert_null(handle, "Should return NULL for invalid CDR");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(errno_handling, from_cdr_null_compressed_image) {
    errno = 0;
    sensor_msgs_compressed_image_t *handle = sensor_msgs_compressed_image_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_compressed_video) {
    errno = 0;
    foxglove_msgs_compressed_video_t *handle = foxglove_msgs_compressed_video_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_mask) {
    errno = 0;
    edgefirst_msgs_mask_t *handle = edgefirst_msgs_mask_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_imu) {
    errno = 0;
    sensor_msgs_imu_t *handle = sensor_msgs_imu_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_nav_sat_fix) {
    errno = 0;
    sensor_msgs_nav_sat_fix_t *handle = sensor_msgs_nav_sat_fix_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_transform_stamped) {
    errno = 0;
    geometry_msgs_transform_stamped_t *handle = geometry_msgs_transform_stamped_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_detect) {
    errno = 0;
    edgefirst_msgs_detect_t *handle = edgefirst_msgs_detect_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_model) {
    errno = 0;
    edgefirst_msgs_model_t *handle = edgefirst_msgs_model_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_model_info) {
    errno = 0;
    edgefirst_msgs_model_info_t *handle = edgefirst_msgs_model_info_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_point_cloud2) {
    errno = 0;
    sensor_msgs_point_cloud2_t *handle = sensor_msgs_point_cloud2_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_camera_info) {
    errno = 0;
    sensor_msgs_camera_info_t *handle = sensor_msgs_camera_info_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_track) {
    errno = 0;
    edgefirst_msgs_track_t *handle = edgefirst_msgs_track_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_box) {
    errno = 0;
    edgefirst_msgs_box_t *handle = edgefirst_msgs_box_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_local_time) {
    errno = 0;
    edgefirst_msgs_local_time_t *handle = edgefirst_msgs_local_time_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_radar_cube) {
    errno = 0;
    edgefirst_msgs_radar_cube_t *handle = edgefirst_msgs_radar_cube_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_radar_info) {
    errno = 0;
    edgefirst_msgs_radar_info_t *handle = edgefirst_msgs_radar_info_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

// ============================================================================
// NULL pointer safety on getters
// ============================================================================

Test(errno_handling, getter_null_time) {
    // CdrFixed decode with NULL should return -1/EINVAL
    int32_t sec;
    uint32_t nanosec;
    errno = 0;
    int ret = builtin_interfaces_time_decode(NULL, 8, &sec, &nanosec);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, getter_null_header) {
    cr_assert_eq(std_msgs_header_get_stamp_sec(NULL), 0);
    cr_assert_eq(std_msgs_header_get_stamp_nanosec(NULL), 0);
    cr_assert_null(std_msgs_header_get_frame_id(NULL));
}

Test(errno_handling, getter_null_image) {
    cr_assert_eq(sensor_msgs_image_get_height(NULL), 0);
    cr_assert_eq(sensor_msgs_image_get_width(NULL), 0);
    cr_assert_null(sensor_msgs_image_get_encoding(NULL));
    cr_assert_null(sensor_msgs_image_get_data(NULL, NULL));
}

Test(errno_handling, getter_null_mask) {
    cr_assert_eq(edgefirst_msgs_mask_get_height(NULL), 0);
    cr_assert_eq(edgefirst_msgs_mask_get_width(NULL), 0);
    cr_assert_null(edgefirst_msgs_mask_get_encoding(NULL));
    cr_assert_null(edgefirst_msgs_mask_get_data(NULL, NULL));
    cr_assert_eq(edgefirst_msgs_mask_get_boxed(NULL), false);
}

// ============================================================================
// Sequential error recovery
// ============================================================================

Test(errno_handling, sequential_errors) {
    // First error: NULL decode
    errno = 0;
    int32_t sec;
    uint32_t nanosec;
    int ret = builtin_interfaces_time_decode(NULL, 8, &sec, &nanosec);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);

    // Second error: NULL from_cdr
    errno = 0;
    std_msgs_header_t *handle = std_msgs_header_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);

    // Third error: buffer too small
    uint8_t buf[2];
    size_t written;
    errno = 0;
    ret = geometry_msgs_vector3_encode(buf, sizeof(buf), &written, 1.0, 2.0, 3.0);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, ENOBUFS);
}

// ============================================================================
// Destructor NULL safety
// ============================================================================

Test(errno_handling, free_null_all_types) {
    // All free functions should safely handle NULL
    std_msgs_header_free(NULL);
    sensor_msgs_image_free(NULL);
    sensor_msgs_compressed_image_free(NULL);
    foxglove_msgs_compressed_video_free(NULL);
    edgefirst_msgs_mask_free(NULL);
    sensor_msgs_imu_free(NULL);
    sensor_msgs_nav_sat_fix_free(NULL);
    geometry_msgs_transform_stamped_free(NULL);
    edgefirst_msgs_radar_cube_free(NULL);
    edgefirst_msgs_radar_info_free(NULL);
    edgefirst_msgs_detect_free(NULL);
    edgefirst_msgs_model_free(NULL);
    edgefirst_msgs_model_info_free(NULL);
    sensor_msgs_point_cloud2_free(NULL);
    sensor_msgs_camera_info_free(NULL);
    edgefirst_msgs_track_free(NULL);
    edgefirst_msgs_box_free(NULL);
    edgefirst_msgs_local_time_free(NULL);

    cr_assert(1, "All free functions safely handled NULL");
}

// ============================================================================
// edgefirst_schemas_bytes_free NULL safety
// ============================================================================

Test(errno_handling, bytes_free_null) {
    // Should not crash
    edgefirst_schemas_bytes_free(NULL, 0);
    edgefirst_schemas_bytes_free(NULL, 100);
}
