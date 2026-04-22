/**
 * @file test_sensor_msgs.c
 * @brief Criterion tests for sensor_msgs types
 *
 * CdrFixed: NavSatStatus
 * Buffer-backed: Image, CompressedImage, Imu, NavSatFix, PointCloud2, CameraInfo
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
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

// ============================================================================
// MagneticField / FluidPressure / Temperature / BatteryState (TOP2-770)
// ============================================================================

static uint8_t *_load_fixture_sm(const char *relpath, size_t *out_len) {
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

Test(sensor_msgs, magnetic_field_from_golden_fixture) {
    size_t len = 0;
    uint8_t *b = _load_fixture_sm("testdata/cdr/sensor_msgs/MagneticField.cdr", &len);
    cr_assert_not_null(b, "failed to load MagneticField fixture");
    ros_magnetic_field_t *v = ros_magnetic_field_from_cdr(b, len);
    cr_assert_not_null(v);
    cr_assert_str_eq(ros_magnetic_field_get_frame_id(v), "test_frame");
    double x = 0, y = 0, z = 0;
    ros_magnetic_field_get_magnetic_field(v, &x, &y, &z);
    cr_assert_float_eq(x, 2.5e-5, 1e-12);
    cr_assert_float_eq(y, -1.2e-5, 1e-12);
    cr_assert_float_eq(z, 4.1e-5, 1e-12);
    double cov[9] = {0};
    ros_magnetic_field_get_magnetic_field_covariance(v, cov);
    cr_assert_float_eq(cov[0], 1e-10, 1e-20);
    ros_magnetic_field_free(v);
    free(b);
}

Test(sensor_msgs, magnetic_field_from_cdr_null) {
    errno = 0;
    cr_assert_null(ros_magnetic_field_from_cdr(NULL, 100));
    cr_assert_eq(errno, EINVAL);
}

Test(sensor_msgs, fluid_pressure_from_golden_fixture) {
    size_t len = 0;
    uint8_t *b = _load_fixture_sm("testdata/cdr/sensor_msgs/FluidPressure.cdr", &len);
    cr_assert_not_null(b);
    ros_fluid_pressure_t *v = ros_fluid_pressure_from_cdr(b, len);
    cr_assert_not_null(v);
    cr_assert_str_eq(ros_fluid_pressure_get_frame_id(v), "test_frame");
    cr_assert_float_eq(ros_fluid_pressure_get_fluid_pressure(v), 101325.0, 1e-9);
    cr_assert_float_eq(ros_fluid_pressure_get_variance(v), 25.0, 1e-9);
    ros_fluid_pressure_free(v);
    free(b);
}

Test(sensor_msgs, temperature_from_golden_fixture) {
    size_t len = 0;
    uint8_t *b = _load_fixture_sm("testdata/cdr/sensor_msgs/Temperature.cdr", &len);
    cr_assert_not_null(b);
    ros_temperature_t *v = ros_temperature_from_cdr(b, len);
    cr_assert_not_null(v);
    cr_assert_str_eq(ros_temperature_get_frame_id(v), "test_frame");
    cr_assert_float_eq(ros_temperature_get_temperature(v), 22.5, 1e-12);
    cr_assert_float_eq(ros_temperature_get_variance(v), 0.01, 1e-12);
    ros_temperature_free(v);
    free(b);
}

Test(sensor_msgs, battery_state_from_golden_fixture) {
    size_t len = 0;
    uint8_t *b = _load_fixture_sm("testdata/cdr/sensor_msgs/BatteryState.cdr", &len);
    cr_assert_not_null(b);
    ros_battery_state_t *v = ros_battery_state_from_cdr(b, len);
    cr_assert_not_null(v);
    cr_assert_str_eq(ros_battery_state_get_frame_id(v), "test_frame");
    cr_assert_float_eq(ros_battery_state_get_voltage(v), 12.34f, 1e-5);
    cr_assert_float_eq(ros_battery_state_get_percentage(v), 0.84f, 1e-5);
    cr_assert_eq(ros_battery_state_get_power_supply_status(v),
                 ROS_BATTERY_STATE_POWER_SUPPLY_STATUS_DISCHARGING);
    cr_assert_eq(ros_battery_state_get_power_supply_technology(v),
                 ROS_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LIPO);
    cr_assert(ros_battery_state_get_present(v));

    cr_assert_eq(ros_battery_state_get_cell_voltage_len(v), 3);
    float cells[4] = {0};
    uint32_t n = ros_battery_state_get_cell_voltage(v, cells, 4);
    cr_assert_eq(n, 3);
    cr_assert_float_eq(cells[0], 4.11f, 1e-5);
    cr_assert_float_eq(cells[2], 4.10f, 1e-5);

    cr_assert_str_eq(ros_battery_state_get_location(v), "battery0");
    cr_assert_str_eq(ros_battery_state_get_serial_number(v), "SN0123456");

    ros_battery_state_free(v);
    free(b);
}

Test(sensor_msgs, new_types_free_null_ok) {
    ros_magnetic_field_free(NULL);
    ros_fluid_pressure_free(NULL);
    ros_temperature_free(NULL);
    ros_battery_state_free(NULL);
}
