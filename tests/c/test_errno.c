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
    int ret = ros_time_encode(buf, sizeof(buf), &written, 1, 2);
    cr_assert_eq(ret, -1, "Should return -1 for small buffer");
    cr_assert_eq(errno, ENOBUFS, "errno should be ENOBUFS");
}

Test(errno_handling, encode_size_query_succeeds) {
    size_t written = 0;

    // NULL buffer should succeed as a size query
    errno = 0;
    int ret = ros_time_encode(NULL, 0, &written, 1, 2);
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
    int ret = ros_time_decode(NULL, 100, &sec, &nanosec);
    cr_assert_eq(ret, -1, "Should return -1 for NULL data");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL for NULL data");
}

Test(errno_handling, decode_too_short) {
    uint8_t buf[2] = {0x00, 0x01};

    errno = 0;
    int32_t sec;
    uint32_t nanosec;
    int ret = ros_time_decode(buf, sizeof(buf), &sec, &nanosec);
    cr_assert_eq(ret, -1, "Should return -1 for truncated data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(errno_handling, decode_truncated_vector3) {
    // Encode valid data, then try to decode with truncated buffer
    uint8_t buf[128];
    size_t written = 0;

    int ret = ros_vector3_encode(buf, sizeof(buf), &written, 1.0, 2.0, 3.0);
    cr_assert_eq(ret, 0);
    cr_assert_gt(written, 4);

    double x, y, z;
    errno = 0;
    ret = ros_vector3_decode(buf, 4, &x, &y, &z);
    cr_assert_eq(ret, -1, "Should return -1 for truncated data");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

// ============================================================================
// Buffer-backed from_cdr errno tests
// ============================================================================

Test(errno_handling, from_cdr_null_header) {
    errno = 0;
    ros_header_t *handle = ros_header_from_cdr(NULL, 100);
    cr_assert_null(handle, "Should return NULL for NULL data");
    cr_assert_eq(errno, EINVAL, "errno should be EINVAL");
}

Test(errno_handling, from_cdr_invalid_header) {
    uint8_t bad_data[4] = {0xFF, 0xFF, 0xFF, 0xFF};

    errno = 0;
    ros_header_t *handle = ros_header_from_cdr(bad_data, sizeof(bad_data));
    cr_assert_null(handle, "Should return NULL for invalid CDR");
    cr_assert_eq(errno, EBADMSG, "errno should be EBADMSG");
}

Test(errno_handling, from_cdr_null_compressed_image) {
    errno = 0;
    ros_compressed_image_t *handle = ros_compressed_image_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_compressed_video) {
    errno = 0;
    ros_compressed_video_t *handle = ros_compressed_video_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_mask) {
    errno = 0;
    ros_mask_t *handle = ros_mask_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_dmabuffer) {
    errno = 0;
    ros_dmabuffer_t *handle = ros_dmabuffer_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_imu) {
    errno = 0;
    ros_imu_t *handle = ros_imu_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_nav_sat_fix) {
    errno = 0;
    ros_nav_sat_fix_t *handle = ros_nav_sat_fix_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_transform_stamped) {
    errno = 0;
    ros_transform_stamped_t *handle = ros_transform_stamped_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_detect) {
    errno = 0;
    ros_detect_t *handle = ros_detect_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_model) {
    errno = 0;
    ros_model_t *handle = ros_model_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_model_info) {
    errno = 0;
    ros_model_info_t *handle = ros_model_info_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_point_cloud2) {
    errno = 0;
    ros_point_cloud2_t *handle = ros_point_cloud2_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_camera_info) {
    errno = 0;
    ros_camera_info_t *handle = ros_camera_info_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_track) {
    errno = 0;
    ros_track_t *handle = ros_track_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_box) {
    errno = 0;
    ros_box_t *handle = ros_box_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_local_time) {
    errno = 0;
    ros_local_time_t *handle = ros_local_time_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_radar_cube) {
    errno = 0;
    ros_radar_cube_t *handle = ros_radar_cube_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, from_cdr_null_radar_info) {
    errno = 0;
    ros_radar_info_t *handle = ros_radar_info_from_cdr(NULL, 100);
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
    int ret = ros_time_decode(NULL, 8, &sec, &nanosec);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

Test(errno_handling, getter_null_header) {
    cr_assert_eq(ros_header_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_header_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_header_get_frame_id(NULL));
}

Test(errno_handling, getter_null_image) {
    cr_assert_eq(ros_image_get_height(NULL), 0);
    cr_assert_eq(ros_image_get_width(NULL), 0);
    cr_assert_null(ros_image_get_encoding(NULL));
    cr_assert_null(ros_image_get_data(NULL, NULL));
}

Test(errno_handling, getter_null_mask) {
    cr_assert_eq(ros_mask_get_height(NULL), 0);
    cr_assert_eq(ros_mask_get_width(NULL), 0);
    cr_assert_null(ros_mask_get_encoding(NULL));
    cr_assert_null(ros_mask_get_data(NULL, NULL));
    cr_assert_eq(ros_mask_get_boxed(NULL), false);
}

// ============================================================================
// Sequential error recovery
// ============================================================================

Test(errno_handling, sequential_errors) {
    // First error: NULL decode
    errno = 0;
    int32_t sec;
    uint32_t nanosec;
    int ret = ros_time_decode(NULL, 8, &sec, &nanosec);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);

    // Second error: NULL from_cdr
    errno = 0;
    ros_header_t *handle = ros_header_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);

    // Third error: buffer too small
    uint8_t buf[2];
    size_t written;
    errno = 0;
    ret = ros_vector3_encode(buf, sizeof(buf), &written, 1.0, 2.0, 3.0);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, ENOBUFS);
}

// ============================================================================
// Destructor NULL safety
// ============================================================================

Test(errno_handling, free_null_all_types) {
    // All free functions should safely handle NULL
    ros_header_free(NULL);
    ros_image_free(NULL);
    ros_compressed_image_free(NULL);
    ros_compressed_video_free(NULL);
    ros_mask_free(NULL);
    ros_dmabuffer_free(NULL);
    ros_imu_free(NULL);
    ros_nav_sat_fix_free(NULL);
    ros_transform_stamped_free(NULL);
    ros_radar_cube_free(NULL);
    ros_radar_info_free(NULL);
    ros_detect_free(NULL);
    ros_model_free(NULL);
    ros_model_info_free(NULL);
    ros_point_cloud2_free(NULL);
    ros_camera_info_free(NULL);
    ros_track_free(NULL);
    ros_box_free(NULL);
    ros_local_time_free(NULL);

    cr_assert(1, "All free functions safely handled NULL");
}

// ============================================================================
// ros_bytes_free NULL safety
// ============================================================================

Test(errno_handling, bytes_free_null) {
    // Should not crash
    ros_bytes_free(NULL, 0);
    ros_bytes_free(NULL, 100);
}
