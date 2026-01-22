/**
 * @file test_sensor_msgs.c
 * @brief Criterion tests for sensor_msgs (PointCloud2, NavSatFix, etc.)
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include "edgefirst/schemas.h"

// ============================================================================
// PointField Tests
// ============================================================================

Test(sensor_msgs, pointfield_create_and_destroy) {
    RosPointField *field = ros_point_field_new();
    cr_assert_not_null(field);

    char *name = ros_point_field_get_name(field);
    cr_assert_not_null(name);
    cr_assert_str_eq(name, "", "Default name should be empty");
    free(name);

    cr_assert_eq(ros_point_field_get_offset(field), 0);
    cr_assert_eq(ros_point_field_get_datatype(field), 0);
    cr_assert_eq(ros_point_field_get_count(field), 1);  // ROS2 default: 1 element per field

    ros_point_field_free(field);
}

Test(sensor_msgs, pointfield_set_name) {
    RosPointField *field = ros_point_field_new();
    cr_assert_not_null(field);

    int ret = ros_point_field_set_name(field, "x");
    cr_assert_eq(ret, 0);

    char *name = ros_point_field_get_name(field);
    cr_assert_str_eq(name, "x");
    free(name);

    ros_point_field_free(field);
}

Test(sensor_msgs, pointfield_set_values) {
    RosPointField *field = ros_point_field_new();
    cr_assert_not_null(field);

    ros_point_field_set_name(field, "intensity");
    ros_point_field_set_offset(field, 12);
    ros_point_field_set_datatype(field, ROS_POINT_FIELD_FLOAT32);
    ros_point_field_set_count(field, 1);

    cr_assert_eq(ros_point_field_get_offset(field), 12);
    cr_assert_eq(ros_point_field_get_datatype(field), ROS_POINT_FIELD_FLOAT32);
    cr_assert_eq(ros_point_field_get_count(field), 1);

    ros_point_field_free(field);
}

Test(sensor_msgs, pointfield_datatype_constants) {
    // Verify ROS2 datatype constants match expected values
    cr_assert_eq(ROS_POINT_FIELD_INT8, 1);
    cr_assert_eq(ROS_POINT_FIELD_UINT8, 2);
    cr_assert_eq(ROS_POINT_FIELD_INT16, 3);
    cr_assert_eq(ROS_POINT_FIELD_UINT16, 4);
    cr_assert_eq(ROS_POINT_FIELD_INT32, 5);
    cr_assert_eq(ROS_POINT_FIELD_UINT32, 6);
    cr_assert_eq(ROS_POINT_FIELD_FLOAT32, 7);
    cr_assert_eq(ROS_POINT_FIELD_FLOAT64, 8);
}

// Note: ros_point_field_serialize/deserialize FFI not implemented yet

Test(sensor_msgs, pointfield_free_null) {
    // Should not crash when freeing NULL
    ros_point_field_free(NULL);
}

// ============================================================================
// PointCloud2 Tests
// ============================================================================

Test(sensor_msgs, pointcloud2_create_and_destroy) {
    RosPointCloud2 *cloud = ros_point_cloud2_new();
    cr_assert_not_null(cloud);

    RosHeader *header = ros_point_cloud2_get_header_mut(cloud);
    cr_assert_not_null(header);

    cr_assert_eq(ros_point_cloud2_get_height(cloud), 0);
    cr_assert_eq(ros_point_cloud2_get_width(cloud), 0);
    cr_assert_eq(ros_point_cloud2_get_point_step(cloud), 0);
    cr_assert_eq(ros_point_cloud2_get_row_step(cloud), 0);
    cr_assert_eq(ros_point_cloud2_get_is_dense(cloud), false);

    ros_point_cloud2_free(cloud);
}

Test(sensor_msgs, pointcloud2_set_dimensions) {
    RosPointCloud2 *cloud = ros_point_cloud2_new();
    cr_assert_not_null(cloud);

    ros_point_cloud2_set_height(cloud, 1);
    ros_point_cloud2_set_width(cloud, 1000);
    ros_point_cloud2_set_point_step(cloud, 16);
    ros_point_cloud2_set_row_step(cloud, 16000);
    ros_point_cloud2_set_is_dense(cloud, true);

    cr_assert_eq(ros_point_cloud2_get_height(cloud), 1);
    cr_assert_eq(ros_point_cloud2_get_width(cloud), 1000);
    cr_assert_eq(ros_point_cloud2_get_point_step(cloud), 16);
    cr_assert_eq(ros_point_cloud2_get_row_step(cloud), 16000);
    cr_assert_eq(ros_point_cloud2_get_is_dense(cloud), true);

    ros_point_cloud2_free(cloud);
}

Test(sensor_msgs, pointcloud2_set_is_bigendian) {
    RosPointCloud2 *cloud = ros_point_cloud2_new();
    cr_assert_not_null(cloud);

    ros_point_cloud2_set_is_bigendian(cloud, false);
    cr_assert_eq(ros_point_cloud2_get_is_bigendian(cloud), false);

    ros_point_cloud2_set_is_bigendian(cloud, true);
    cr_assert_eq(ros_point_cloud2_get_is_bigendian(cloud), true);

    ros_point_cloud2_free(cloud);
}

Test(sensor_msgs, pointcloud2_set_data) {
    RosPointCloud2 *cloud = ros_point_cloud2_new();
    cr_assert_not_null(cloud);

    uint8_t test_data[48]; // 3 points * 16 bytes
    for (int i = 0; i < 48; i++) {
        test_data[i] = (uint8_t)i;
    }

    int ret = ros_point_cloud2_set_data(cloud, test_data, 48);
    cr_assert_eq(ret, 0);

    size_t data_len = 0;
    const uint8_t *data = ros_point_cloud2_get_data(cloud, &data_len);
    cr_assert_eq(data_len, 48);

    for (size_t i = 0; i < 48; i++) {
        cr_assert_eq(data[i], test_data[i]);
    }

    ros_point_cloud2_free(cloud);
}

Test(sensor_msgs, pointcloud2_serialize_deserialize) {
    RosPointCloud2 *original = ros_point_cloud2_new();
    cr_assert_not_null(original);

    ros_point_cloud2_set_height(original, 1);
    ros_point_cloud2_set_width(original, 100);
    ros_point_cloud2_set_point_step(original, 12);
    ros_point_cloud2_set_row_step(original, 1200);
    ros_point_cloud2_set_is_bigendian(original, false);
    ros_point_cloud2_set_is_dense(original, true);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_point_cloud2_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosPointCloud2 *deserialized = ros_point_cloud2_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_eq(ros_point_cloud2_get_height(deserialized), 1);
    cr_assert_eq(ros_point_cloud2_get_width(deserialized), 100);
    cr_assert_eq(ros_point_cloud2_get_point_step(deserialized), 12);
    cr_assert_eq(ros_point_cloud2_get_is_dense(deserialized), true);

    ros_point_cloud2_free(original);
    ros_point_cloud2_free(deserialized);
    free(buffer);
}

Test(sensor_msgs, pointcloud2_free_null) {
    // Should not crash when freeing NULL
    ros_point_cloud2_free(NULL);
}

// ============================================================================
// NavSatStatus Tests
// ============================================================================

Test(sensor_msgs, navsatstatus_create_and_destroy) {
    RosNavSatStatus *status = ros_nav_sat_status_new();
    cr_assert_not_null(status);

    // ROS2 default: STATUS_NO_FIX (-1) indicates no GPS fix
    cr_assert_eq(ros_nav_sat_status_get_status(status), ROS_NAV_SAT_STATUS_STATUS_NO_FIX);
    cr_assert_eq(ros_nav_sat_status_get_service(status), 0);

    ros_nav_sat_status_free(status);
}

Test(sensor_msgs, navsatstatus_service_constants) {
    // Verify ROS2 service constants
    cr_assert_eq(ROS_NAV_SAT_STATUS_SERVICE_GPS, 1);
    cr_assert_eq(ROS_NAV_SAT_STATUS_SERVICE_GLONASS, 2);
    cr_assert_eq(ROS_NAV_SAT_STATUS_SERVICE_COMPASS, 4);
    cr_assert_eq(ROS_NAV_SAT_STATUS_SERVICE_GALILEO, 8);
}

Test(sensor_msgs, navsatstatus_status_constants) {
    // Verify ROS2 status constants
    cr_assert_eq(ROS_NAV_SAT_STATUS_STATUS_NO_FIX, -1);
    cr_assert_eq(ROS_NAV_SAT_STATUS_STATUS_FIX, 0);
    cr_assert_eq(ROS_NAV_SAT_STATUS_STATUS_SBAS_FIX, 1);
    cr_assert_eq(ROS_NAV_SAT_STATUS_STATUS_GBAS_FIX, 2);
}

Test(sensor_msgs, navsatstatus_set_values) {
    RosNavSatStatus *status = ros_nav_sat_status_new();
    cr_assert_not_null(status);

    ros_nav_sat_status_set_status(status, ROS_NAV_SAT_STATUS_STATUS_FIX);
    ros_nav_sat_status_set_service(status, ROS_NAV_SAT_STATUS_SERVICE_GPS | ROS_NAV_SAT_STATUS_SERVICE_GALILEO);

    cr_assert_eq(ros_nav_sat_status_get_status(status), ROS_NAV_SAT_STATUS_STATUS_FIX);
    cr_assert_eq(ros_nav_sat_status_get_service(status), ROS_NAV_SAT_STATUS_SERVICE_GPS | ROS_NAV_SAT_STATUS_SERVICE_GALILEO);

    ros_nav_sat_status_free(status);
}

// Note: ros_nav_sat_status_serialize/deserialize FFI not implemented yet

Test(sensor_msgs, navsatstatus_free_null) {
    // Should not crash when freeing NULL
    ros_nav_sat_status_free(NULL);
}

// ============================================================================
// NavSatFix Tests
// ============================================================================

Test(sensor_msgs, navsatfix_create_and_destroy) {
    RosNavSatFix *fix = ros_nav_sat_fix_new();
    cr_assert_not_null(fix);

    RosHeader *header = ros_nav_sat_fix_get_header_mut(fix);
    cr_assert_not_null(header);

    RosNavSatStatus *status = ros_nav_sat_fix_get_status_mut(fix);
    cr_assert_not_null(status);

    cr_assert_float_eq(ros_nav_sat_fix_get_latitude(fix), 0.0, 0.0001);
    cr_assert_float_eq(ros_nav_sat_fix_get_longitude(fix), 0.0, 0.0001);
    cr_assert_float_eq(ros_nav_sat_fix_get_altitude(fix), 0.0, 0.0001);

    ros_nav_sat_fix_free(fix);
}

Test(sensor_msgs, navsatfix_covariance_type_constants) {
    // Verify covariance type constants
    cr_assert_eq(ROS_NAV_SAT_FIX_COVARIANCE_TYPE_UNKNOWN, 0);
    cr_assert_eq(ROS_NAV_SAT_FIX_COVARIANCE_TYPE_APPROXIMATED, 1);
    cr_assert_eq(ROS_NAV_SAT_FIX_COVARIANCE_TYPE_DIAGONAL_KNOWN, 2);
    cr_assert_eq(ROS_NAV_SAT_FIX_COVARIANCE_TYPE_KNOWN, 3);
}

Test(sensor_msgs, navsatfix_set_position) {
    RosNavSatFix *fix = ros_nav_sat_fix_new();
    cr_assert_not_null(fix);

    ros_nav_sat_fix_set_latitude(fix, 45.5);
    ros_nav_sat_fix_set_longitude(fix, -73.6);
    ros_nav_sat_fix_set_altitude(fix, 100.5);

    cr_assert_float_eq(ros_nav_sat_fix_get_latitude(fix), 45.5, 0.0001);
    cr_assert_float_eq(ros_nav_sat_fix_get_longitude(fix), -73.6, 0.0001);
    cr_assert_float_eq(ros_nav_sat_fix_get_altitude(fix), 100.5, 0.0001);

    ros_nav_sat_fix_free(fix);
}

Test(sensor_msgs, navsatfix_set_covariance) {
    RosNavSatFix *fix = ros_nav_sat_fix_new();
    cr_assert_not_null(fix);

    double covariance[9] = {
        1.0, 0.0, 0.0,
        0.0, 2.0, 0.0,
        0.0, 0.0, 3.0
    };

    int ret = ros_nav_sat_fix_set_position_covariance(fix, covariance);
    cr_assert_eq(ret, 0);

    const double *got_cov = ros_nav_sat_fix_get_position_covariance(fix);
    cr_assert_not_null(got_cov);

    for (int i = 0; i < 9; i++) {
        cr_assert_float_eq(got_cov[i], covariance[i], 0.0001);
    }

    ros_nav_sat_fix_free(fix);
}

Test(sensor_msgs, navsatfix_set_covariance_type) {
    RosNavSatFix *fix = ros_nav_sat_fix_new();
    cr_assert_not_null(fix);

    ros_nav_sat_fix_set_position_covariance_type(fix, ROS_NAV_SAT_FIX_COVARIANCE_TYPE_KNOWN);

    cr_assert_eq(ros_nav_sat_fix_get_position_covariance_type(fix), ROS_NAV_SAT_FIX_COVARIANCE_TYPE_KNOWN);

    ros_nav_sat_fix_free(fix);
}

Test(sensor_msgs, navsatfix_serialize_deserialize) {
    RosNavSatFix *original = ros_nav_sat_fix_new();
    cr_assert_not_null(original);

    ros_nav_sat_fix_set_latitude(original, 40.7128);
    ros_nav_sat_fix_set_longitude(original, -74.0060);
    ros_nav_sat_fix_set_altitude(original, 10.0);
    ros_nav_sat_fix_set_position_covariance_type(original, ROS_NAV_SAT_FIX_COVARIANCE_TYPE_APPROXIMATED);

    RosNavSatStatus *status = ros_nav_sat_fix_get_status_mut(original);
    ros_nav_sat_status_set_status(status, ROS_NAV_SAT_STATUS_STATUS_FIX);
    ros_nav_sat_status_set_service(status, ROS_NAV_SAT_STATUS_SERVICE_GPS);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_nav_sat_fix_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosNavSatFix *deserialized = ros_nav_sat_fix_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_float_eq(ros_nav_sat_fix_get_latitude(deserialized), 40.7128, 0.0001);
    cr_assert_float_eq(ros_nav_sat_fix_get_longitude(deserialized), -74.0060, 0.0001);
    cr_assert_float_eq(ros_nav_sat_fix_get_altitude(deserialized), 10.0, 0.0001);

    RosNavSatStatus *deser_status = ros_nav_sat_fix_get_status_mut(deserialized);
    cr_assert_eq(ros_nav_sat_status_get_status(deser_status), ROS_NAV_SAT_STATUS_STATUS_FIX);

    ros_nav_sat_fix_free(original);
    ros_nav_sat_fix_free(deserialized);
    free(buffer);
}

Test(sensor_msgs, navsatfix_free_null) {
    // Should not crash when freeing NULL
    ros_nav_sat_fix_free(NULL);
}

// ============================================================================
// RegionOfInterest Tests
// ============================================================================

Test(sensor_msgs, roi_create_and_destroy) {
    RosRegionOfInterest *roi = ros_region_of_interest_new();
    cr_assert_not_null(roi);

    ros_region_of_interest_set_x_offset(roi, 100);
    ros_region_of_interest_set_y_offset(roi, 200);
    ros_region_of_interest_set_width(roi, 640);
    ros_region_of_interest_set_height(roi, 480);
    ros_region_of_interest_set_do_rectify(roi, true);

    cr_assert_eq(ros_region_of_interest_get_x_offset(roi), 100);
    cr_assert_eq(ros_region_of_interest_get_y_offset(roi), 200);
    cr_assert_eq(ros_region_of_interest_get_width(roi), 640);
    cr_assert_eq(ros_region_of_interest_get_height(roi), 480);
    cr_assert_eq(ros_region_of_interest_get_do_rectify(roi), true);

    ros_region_of_interest_free(roi);
}

Test(sensor_msgs, roi_serialize_deserialize) {
    RosRegionOfInterest *original = ros_region_of_interest_new();
    cr_assert_not_null(original);

    ros_region_of_interest_set_x_offset(original, 50);
    ros_region_of_interest_set_y_offset(original, 75);
    ros_region_of_interest_set_width(original, 320);
    ros_region_of_interest_set_height(original, 240);
    ros_region_of_interest_set_do_rectify(original, false);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_region_of_interest_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosRegionOfInterest *deserialized = ros_region_of_interest_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_eq(ros_region_of_interest_get_x_offset(deserialized), 50);
    cr_assert_eq(ros_region_of_interest_get_y_offset(deserialized), 75);
    cr_assert_eq(ros_region_of_interest_get_width(deserialized), 320);
    cr_assert_eq(ros_region_of_interest_get_height(deserialized), 240);
    cr_assert_eq(ros_region_of_interest_get_do_rectify(deserialized), false);

    ros_region_of_interest_free(original);
    ros_region_of_interest_free(deserialized);
    free(buffer);
}

Test(sensor_msgs, roi_free_null) {
    ros_region_of_interest_free(NULL);
}

// ============================================================================
// CompressedImage Tests
// ============================================================================

Test(sensor_msgs, compressed_image_create_and_destroy) {
    RosCompressedImage *image = ros_compressed_image_new();
    cr_assert_not_null(image);

    RosHeader *header = ros_compressed_image_get_header_mut(image);
    cr_assert_not_null(header);
    ros_header_set_frame_id(header, "camera_front");

    int ret = ros_compressed_image_set_format(image, "jpeg");
    cr_assert_eq(ret, 0);

    char *format = ros_compressed_image_get_format(image);
    cr_assert_str_eq(format, "jpeg");
    free(format);

    ros_compressed_image_free(image);
}

Test(sensor_msgs, compressed_image_set_data) {
    RosCompressedImage *image = ros_compressed_image_new();
    cr_assert_not_null(image);

    // Simulated JPEG header
    uint8_t jpeg_data[] = {0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46};

    int ret = ros_compressed_image_set_data(image, jpeg_data, sizeof(jpeg_data));
    cr_assert_eq(ret, 0);

    size_t len = 0;
    const uint8_t *data = ros_compressed_image_get_data(image, &len);
    cr_assert_eq(len, sizeof(jpeg_data));

    for (size_t i = 0; i < len; i++) {
        cr_assert_eq(data[i], jpeg_data[i]);
    }

    ros_compressed_image_free(image);
}

Test(sensor_msgs, compressed_image_serialize_deserialize) {
    RosCompressedImage *original = ros_compressed_image_new();
    cr_assert_not_null(original);

    RosHeader *header = ros_compressed_image_get_header_mut(original);
    ros_header_set_frame_id(header, "camera_optical");

    ros_compressed_image_set_format(original, "png");

    uint8_t png_data[] = {0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A};
    ros_compressed_image_set_data(original, png_data, sizeof(png_data));

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_compressed_image_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosCompressedImage *deserialized = ros_compressed_image_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    char *format = ros_compressed_image_get_format(deserialized);
    cr_assert_str_eq(format, "png");
    free(format);

    const RosHeader *deser_header = ros_compressed_image_get_header(deserialized);
    char *frame_id = ros_header_get_frame_id(deser_header);
    cr_assert_str_eq(frame_id, "camera_optical");
    free(frame_id);

    size_t data_len = 0;
    const uint8_t *data = ros_compressed_image_get_data(deserialized, &data_len);
    cr_assert_eq(data_len, sizeof(png_data));
    cr_assert_not_null(data);

    ros_compressed_image_free(original);
    ros_compressed_image_free(deserialized);
    free(buffer);
}

Test(sensor_msgs, compressed_image_free_null) {
    ros_compressed_image_free(NULL);
}

// ============================================================================
// IMU Tests
// ============================================================================

Test(sensor_msgs, imu_create_and_destroy) {
    RosImu *imu = ros_imu_new();
    cr_assert_not_null(imu);

    RosHeader *header = ros_imu_get_header_mut(imu);
    cr_assert_not_null(header);
    ros_header_set_frame_id(header, "imu_link");

    // Set orientation
    RosQuaternion *orientation = ros_imu_get_orientation_mut(imu);
    cr_assert_not_null(orientation);
    ros_quaternion_set_w(orientation, 1.0);

    // Set angular velocity
    RosVector3 *angular = ros_imu_get_angular_velocity_mut(imu);
    cr_assert_not_null(angular);
    ros_vector3_set_z(angular, 0.1);

    // Set linear acceleration
    RosVector3 *linear = ros_imu_get_linear_acceleration_mut(imu);
    cr_assert_not_null(linear);
    ros_vector3_set_z(linear, 9.81);

    // Verify
    const RosQuaternion *ori = ros_imu_get_orientation(imu);
    cr_assert_float_eq(ros_quaternion_get_w(ori), 1.0, 0.0001);

    const RosVector3 *ang = ros_imu_get_angular_velocity(imu);
    cr_assert_float_eq(ros_vector3_get_z(ang), 0.1, 0.0001);

    const RosVector3 *lin = ros_imu_get_linear_acceleration(imu);
    cr_assert_float_eq(ros_vector3_get_z(lin), 9.81, 0.0001);

    ros_imu_free(imu);
}

Test(sensor_msgs, imu_set_covariances) {
    RosImu *imu = ros_imu_new();
    cr_assert_not_null(imu);

    double covariance[9] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };

    int ret = ros_imu_set_orientation_covariance(imu, covariance);
    cr_assert_eq(ret, 0);

    ret = ros_imu_set_angular_velocity_covariance(imu, covariance);
    cr_assert_eq(ret, 0);

    ret = ros_imu_set_linear_acceleration_covariance(imu, covariance);
    cr_assert_eq(ret, 0);

    const double *ori_cov = ros_imu_get_orientation_covariance(imu);
    cr_assert_not_null(ori_cov);
    cr_assert_float_eq(ori_cov[0], 1.0, 0.0001);
    cr_assert_float_eq(ori_cov[4], 1.0, 0.0001);

    ros_imu_free(imu);
}

Test(sensor_msgs, imu_serialize_deserialize) {
    RosImu *original = ros_imu_new();
    cr_assert_not_null(original);

    RosHeader *header = ros_imu_get_header_mut(original);
    ros_header_set_frame_id(header, "imu_frame");

    RosQuaternion *ori = ros_imu_get_orientation_mut(original);
    ros_quaternion_set_w(ori, 1.0);

    RosVector3 *ang = ros_imu_get_angular_velocity_mut(original);
    ros_vector3_set_x(ang, 0.1);
    ros_vector3_set_y(ang, 0.2);
    ros_vector3_set_z(ang, 0.3);

    RosVector3 *lin = ros_imu_get_linear_acceleration_mut(original);
    ros_vector3_set_x(lin, 0.0);
    ros_vector3_set_y(lin, 0.0);
    ros_vector3_set_z(lin, 9.81);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_imu_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosImu *deserialized = ros_imu_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    const RosHeader *deser_header = ros_imu_get_header(deserialized);
    char *frame_id = ros_header_get_frame_id(deser_header);
    cr_assert_str_eq(frame_id, "imu_frame");
    free(frame_id);

    const RosVector3 *deser_lin = ros_imu_get_linear_acceleration(deserialized);
    cr_assert_float_eq(ros_vector3_get_z(deser_lin), 9.81, 0.0001);

    ros_imu_free(original);
    ros_imu_free(deserialized);
    free(buffer);
}

Test(sensor_msgs, imu_free_null) {
    ros_imu_free(NULL);
}

// ============================================================================
// CameraInfo Tests
// ============================================================================

Test(sensor_msgs, camera_info_create_and_destroy) {
    RosCameraInfo *info = ros_camera_info_new();
    cr_assert_not_null(info);

    RosHeader *header = ros_camera_info_get_header_mut(info);
    cr_assert_not_null(header);
    ros_header_set_frame_id(header, "camera_optical_frame");

    ros_camera_info_set_height(info, 480);
    ros_camera_info_set_width(info, 640);
    ros_camera_info_set_distortion_model(info, "plumb_bob");
    ros_camera_info_set_binning_x(info, 1);
    ros_camera_info_set_binning_y(info, 1);

    cr_assert_eq(ros_camera_info_get_height(info), 480);
    cr_assert_eq(ros_camera_info_get_width(info), 640);
    cr_assert_eq(ros_camera_info_get_binning_x(info), 1);

    char *model = ros_camera_info_get_distortion_model(info);
    cr_assert_str_eq(model, "plumb_bob");
    free(model);

    ros_camera_info_free(info);
}

Test(sensor_msgs, camera_info_set_matrices) {
    RosCameraInfo *info = ros_camera_info_new();
    cr_assert_not_null(info);

    // Intrinsic matrix K (3x3)
    double k[9] = {
        500.0, 0.0, 320.0,
        0.0, 500.0, 240.0,
        0.0, 0.0, 1.0
    };
    int ret = ros_camera_info_set_k(info, k);
    cr_assert_eq(ret, 0);

    // Rectification matrix R (3x3)
    double r[9] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    ret = ros_camera_info_set_r(info, r);
    cr_assert_eq(ret, 0);

    // Projection matrix P (3x4)
    double p[12] = {
        500.0, 0.0, 320.0, 0.0,
        0.0, 500.0, 240.0, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
    ret = ros_camera_info_set_p(info, p);
    cr_assert_eq(ret, 0);

    // Distortion coefficients D (variable length)
    double d[5] = {0.1, -0.2, 0.001, 0.002, 0.0};
    ret = ros_camera_info_set_d(info, d, 5);
    cr_assert_eq(ret, 0);

    // Verify K
    const double *k_got = ros_camera_info_get_k(info);
    cr_assert_not_null(k_got);
    cr_assert_float_eq(k_got[0], 500.0, 0.0001);
    cr_assert_float_eq(k_got[2], 320.0, 0.0001);

    // Verify D
    size_t d_len = 0;
    const double *d_got = ros_camera_info_get_d(info, &d_len);
    cr_assert_eq(d_len, 5);
    cr_assert_float_eq(d_got[0], 0.1, 0.0001);

    ros_camera_info_free(info);
}

Test(sensor_msgs, camera_info_roi) {
    RosCameraInfo *info = ros_camera_info_new();
    cr_assert_not_null(info);

    RosRegionOfInterest *roi = ros_camera_info_get_roi_mut(info);
    cr_assert_not_null(roi);

    ros_region_of_interest_set_x_offset(roi, 10);
    ros_region_of_interest_set_y_offset(roi, 20);
    ros_region_of_interest_set_width(roi, 620);
    ros_region_of_interest_set_height(roi, 460);

    const RosRegionOfInterest *roi_const = ros_camera_info_get_roi(info);
    cr_assert_eq(ros_region_of_interest_get_x_offset(roi_const), 10);
    cr_assert_eq(ros_region_of_interest_get_width(roi_const), 620);

    ros_camera_info_free(info);
}

Test(sensor_msgs, camera_info_serialize_deserialize) {
    RosCameraInfo *original = ros_camera_info_new();
    cr_assert_not_null(original);

    RosHeader *header = ros_camera_info_get_header_mut(original);
    ros_header_set_frame_id(header, "camera0");

    ros_camera_info_set_height(original, 720);
    ros_camera_info_set_width(original, 1280);
    ros_camera_info_set_distortion_model(original, "rational_polynomial");

    double k[9] = {
        800.0, 0.0, 640.0,
        0.0, 800.0, 360.0,
        0.0, 0.0, 1.0
    };
    ros_camera_info_set_k(original, k);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_camera_info_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);

    RosCameraInfo *deserialized = ros_camera_info_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_eq(ros_camera_info_get_height(deserialized), 720);
    cr_assert_eq(ros_camera_info_get_width(deserialized), 1280);

    char *model = ros_camera_info_get_distortion_model(deserialized);
    cr_assert_str_eq(model, "rational_polynomial");
    free(model);

    const double *k_got = ros_camera_info_get_k(deserialized);
    cr_assert_float_eq(k_got[0], 800.0, 0.0001);

    ros_camera_info_free(original);
    ros_camera_info_free(deserialized);
    free(buffer);
}

Test(sensor_msgs, camera_info_free_null) {
    ros_camera_info_free(NULL);
}
