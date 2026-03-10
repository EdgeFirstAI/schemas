/**
 * @file test_sensor_msgs.c
 * @brief Criterion tests for sensor_msgs types
 *
 * CdrFixed: NavSatStatus
 * Buffer-backed: Image, CompressedImage, Imu, NavSatFix, PointCloud2, CameraInfo
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

// ============================================================================
// NavSatStatus Tests (CdrFixed)
// ============================================================================

Test(sensor_msgs, nav_sat_status_encode_decode_roundtrip) {
    uint8_t buf[64];
    size_t written = 0;

    int ret = ros_nav_sat_status_encode(buf, sizeof(buf), &written, 0, 9);
    cr_assert_eq(ret, 0);
    cr_assert_gt(written, 0);

    int8_t status = -1;
    uint16_t service = 0;
    ret = ros_nav_sat_status_decode(buf, written, &status, &service);
    cr_assert_eq(ret, 0);
    cr_assert_eq(status, 0);  // STATUS_FIX
    cr_assert_eq(service, 9); // GPS | GALILEO
}

Test(sensor_msgs, nav_sat_status_encode_decode_no_fix) {
    uint8_t buf[64];
    size_t written = 0;

    int ret = ros_nav_sat_status_encode(buf, sizeof(buf), &written, -1, 0);
    cr_assert_eq(ret, 0);

    int8_t status = 0;
    uint16_t service = 0;
    ret = ros_nav_sat_status_decode(buf, written, &status, &service);
    cr_assert_eq(ret, 0);
    cr_assert_eq(status, -1); // STATUS_NO_FIX
    cr_assert_eq(service, 0);
}

Test(sensor_msgs, nav_sat_status_decode_null_data) {
    int8_t status;
    uint16_t service;
    errno = 0;
    int ret = ros_nav_sat_status_decode(NULL, 100, &status, &service);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

Test(sensor_msgs, nav_sat_status_encode_size_query) {
    size_t written = 0;
    int ret = ros_nav_sat_status_encode(NULL, 0, &written, 0, 1);
    cr_assert_eq(ret, 0);
    cr_assert_gt(written, 0);
}

// ============================================================================
// Image Tests (buffer-backed)
// ============================================================================

Test(sensor_msgs, image_encode_from_cdr_roundtrip) {
    uint8_t pixel_data[] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00};
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = ros_image_encode(&bytes, &len,
                               100, 200,        // stamp
                               "camera0",       // frame_id
                               480, 640,        // height, width
                               "rgb8",          // encoding
                               0,               // is_bigendian
                               1920,            // step
                               pixel_data, sizeof(pixel_data)); // data
    cr_assert_eq(ret, 0);
    cr_assert_not_null(bytes);

    ros_image_t *handle = ros_image_from_cdr(bytes, len);
    cr_assert_not_null(handle);

    cr_assert_eq(ros_image_get_stamp_sec(handle), 100);
    cr_assert_eq(ros_image_get_stamp_nanosec(handle), 200);
    cr_assert_str_eq(ros_image_get_frame_id(handle), "camera0");
    cr_assert_eq(ros_image_get_height(handle), 480);
    cr_assert_eq(ros_image_get_width(handle), 640);
    cr_assert_str_eq(ros_image_get_encoding(handle), "rgb8");
    cr_assert_eq(ros_image_get_is_bigendian(handle), 0);
    cr_assert_eq(ros_image_get_step(handle), 1920);

    size_t data_len = 0;
    const uint8_t *data = ros_image_get_data(handle, &data_len);
    cr_assert_eq(data_len, sizeof(pixel_data));
    for (size_t i = 0; i < data_len; i++) {
        cr_assert_eq(data[i], pixel_data[i]);
    }

    ros_image_free(handle);
    ros_bytes_free(bytes, len);
}

Test(sensor_msgs, image_from_cdr_null) {
    errno = 0;
    ros_image_t *handle = ros_image_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(sensor_msgs, image_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD};
    errno = 0;
    ros_image_t *handle = ros_image_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(sensor_msgs, image_free_null) {
    ros_image_free(NULL);
}

Test(sensor_msgs, image_getters_null) {
    cr_assert_eq(ros_image_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_image_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_image_get_frame_id(NULL));
    cr_assert_eq(ros_image_get_height(NULL), 0);
    cr_assert_eq(ros_image_get_width(NULL), 0);
    cr_assert_null(ros_image_get_encoding(NULL));
    cr_assert_eq(ros_image_get_step(NULL), 0);
    cr_assert_null(ros_image_get_data(NULL, NULL));
}

// ============================================================================
// CompressedImage Tests (buffer-backed)
// ============================================================================

Test(sensor_msgs, compressed_image_encode_from_cdr_roundtrip) {
    uint8_t jpeg_data[] = {0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46};
    uint8_t *bytes = NULL;
    size_t len = 0;

    int ret = ros_compressed_image_encode(&bytes, &len,
                                          42, 123,           // stamp
                                          "camera_optical",  // frame_id
                                          "jpeg",            // format
                                          jpeg_data, sizeof(jpeg_data)); // data
    cr_assert_eq(ret, 0);
    cr_assert_not_null(bytes);

    ros_compressed_image_t *handle = ros_compressed_image_from_cdr(bytes, len);
    cr_assert_not_null(handle);

    cr_assert_eq(ros_compressed_image_get_stamp_sec(handle), 42);
    cr_assert_eq(ros_compressed_image_get_stamp_nanosec(handle), 123);
    cr_assert_str_eq(ros_compressed_image_get_frame_id(handle), "camera_optical");
    cr_assert_str_eq(ros_compressed_image_get_format(handle), "jpeg");

    size_t data_len = 0;
    const uint8_t *data = ros_compressed_image_get_data(handle, &data_len);
    cr_assert_eq(data_len, sizeof(jpeg_data));
    for (size_t i = 0; i < data_len; i++) {
        cr_assert_eq(data[i], jpeg_data[i]);
    }

    ros_compressed_image_free(handle);
    ros_bytes_free(bytes, len);
}

Test(sensor_msgs, compressed_image_from_cdr_null) {
    errno = 0;
    ros_compressed_image_t *handle = ros_compressed_image_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(sensor_msgs, compressed_image_free_null) {
    ros_compressed_image_free(NULL);
}

Test(sensor_msgs, compressed_image_getters_null) {
    cr_assert_eq(ros_compressed_image_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_compressed_image_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_compressed_image_get_frame_id(NULL));
    cr_assert_null(ros_compressed_image_get_format(NULL));
    cr_assert_null(ros_compressed_image_get_data(NULL, NULL));
}

// ============================================================================
// IMU Tests (buffer-backed — no encode, test from_cdr + getters)
// ============================================================================

Test(sensor_msgs, imu_from_cdr_null) {
    errno = 0;
    ros_imu_t *handle = ros_imu_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(sensor_msgs, imu_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_imu_t *handle = ros_imu_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(sensor_msgs, imu_free_null) {
    ros_imu_free(NULL);
}

Test(sensor_msgs, imu_getters_null) {
    cr_assert_eq(ros_imu_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_imu_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_imu_get_frame_id(NULL));

    // Orientation with NULL handle should not crash
    double x, y, z, w;
    ros_imu_get_orientation(NULL, &x, &y, &z, &w);

    // Angular velocity with NULL handle should not crash
    ros_imu_get_angular_velocity(NULL, &x, &y, &z);

    // Linear acceleration with NULL handle should not crash
    ros_imu_get_linear_acceleration(NULL, &x, &y, &z);
}

// ============================================================================
// NavSatFix Tests (buffer-backed — no encode, test from_cdr + getters)
// ============================================================================

Test(sensor_msgs, nav_sat_fix_from_cdr_null) {
    errno = 0;
    ros_nav_sat_fix_t *handle = ros_nav_sat_fix_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(sensor_msgs, nav_sat_fix_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_nav_sat_fix_t *handle = ros_nav_sat_fix_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(sensor_msgs, nav_sat_fix_free_null) {
    ros_nav_sat_fix_free(NULL);
}

Test(sensor_msgs, nav_sat_fix_getters_null) {
    cr_assert_eq(ros_nav_sat_fix_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_nav_sat_fix_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_nav_sat_fix_get_frame_id(NULL));
    cr_assert_float_eq(ros_nav_sat_fix_get_latitude(NULL), 0.0, 0.0001);
    cr_assert_float_eq(ros_nav_sat_fix_get_longitude(NULL), 0.0, 0.0001);
    cr_assert_float_eq(ros_nav_sat_fix_get_altitude(NULL), 0.0, 0.0001);
}

// ============================================================================
// PointCloud2 Tests (buffer-backed — no encode, test from_cdr + getters)
// ============================================================================

Test(sensor_msgs, point_cloud2_from_cdr_null) {
    errno = 0;
    ros_point_cloud2_t *handle = ros_point_cloud2_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(sensor_msgs, point_cloud2_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_point_cloud2_t *handle = ros_point_cloud2_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(sensor_msgs, point_cloud2_free_null) {
    ros_point_cloud2_free(NULL);
}

Test(sensor_msgs, point_cloud2_getters_null) {
    cr_assert_eq(ros_point_cloud2_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_point_cloud2_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_point_cloud2_get_frame_id(NULL));
    cr_assert_eq(ros_point_cloud2_get_height(NULL), 0);
    cr_assert_eq(ros_point_cloud2_get_width(NULL), 0);
    cr_assert_eq(ros_point_cloud2_get_point_step(NULL), 0);
    cr_assert_eq(ros_point_cloud2_get_row_step(NULL), 0);
    cr_assert_null(ros_point_cloud2_get_data(NULL, NULL));
    cr_assert_eq(ros_point_cloud2_get_is_dense(NULL), false);
    cr_assert_eq(ros_point_cloud2_get_is_bigendian(NULL), false);
    cr_assert_eq(ros_point_cloud2_get_fields_len(NULL), 0);
}

// ============================================================================
// CameraInfo Tests (buffer-backed — no encode, test from_cdr + getters)
// ============================================================================

Test(sensor_msgs, camera_info_from_cdr_null) {
    errno = 0;
    ros_camera_info_t *handle = ros_camera_info_from_cdr(NULL, 100);
    cr_assert_null(handle);
    cr_assert_eq(errno, EINVAL);
}

Test(sensor_msgs, camera_info_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_camera_info_t *handle = ros_camera_info_from_cdr(bad, sizeof(bad));
    cr_assert_null(handle);
    cr_assert_eq(errno, EBADMSG);
}

Test(sensor_msgs, camera_info_free_null) {
    ros_camera_info_free(NULL);
}

Test(sensor_msgs, camera_info_getters_null) {
    cr_assert_eq(ros_camera_info_get_stamp_sec(NULL), 0);
    cr_assert_eq(ros_camera_info_get_stamp_nanosec(NULL), 0);
    cr_assert_null(ros_camera_info_get_frame_id(NULL));
    cr_assert_eq(ros_camera_info_get_height(NULL), 0);
    cr_assert_eq(ros_camera_info_get_width(NULL), 0);
    cr_assert_null(ros_camera_info_get_distortion_model(NULL));
    cr_assert_eq(ros_camera_info_get_binning_x(NULL), 0);
    cr_assert_eq(ros_camera_info_get_binning_y(NULL), 0);
}
