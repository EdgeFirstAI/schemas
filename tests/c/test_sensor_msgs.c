/**
 * @file test_sensor_msgs.c
 * @brief Criterion tests for sensor_msgs (PointCloud2, NavSatFix, etc.)
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include "edgefirst/schemas.h"

// ============================================================================
// PointField Tests
// ============================================================================

Test(sensor_msgs, pointfield_create_and_destroy) {
    PointField *field = edgefirst_pointfield_create();
    cr_assert_not_null(field);
    
    const char *name = edgefirst_pointfield_get_name(field);
    cr_assert_str_eq(name, "", "Default name should be empty");
    
    cr_assert_eq(edgefirst_pointfield_get_offset(field), 0);
    cr_assert_eq(edgefirst_pointfield_get_datatype(field), 0);
    cr_assert_eq(edgefirst_pointfield_get_count(field), 0);
    
    edgefirst_pointfield_destroy(field);
}

Test(sensor_msgs, pointfield_set_name) {
    PointField *field = edgefirst_pointfield_create();
    cr_assert_not_null(field);
    
    int ret = edgefirst_pointfield_set_name(field, "x");
    cr_assert_eq(ret, 0);
    
    const char *name = edgefirst_pointfield_get_name(field);
    cr_assert_str_eq(name, "x");
    
    edgefirst_pointfield_destroy(field);
}

Test(sensor_msgs, pointfield_set_values) {
    PointField *field = edgefirst_pointfield_create();
    cr_assert_not_null(field);
    
    edgefirst_pointfield_set_name(field, "intensity");
    edgefirst_pointfield_set_offset(field, 12);
    edgefirst_pointfield_set_datatype(field, POINTFIELD_FLOAT32);
    edgefirst_pointfield_set_count(field, 1);
    
    cr_assert_eq(edgefirst_pointfield_get_offset(field), 12);
    cr_assert_eq(edgefirst_pointfield_get_datatype(field), POINTFIELD_FLOAT32);
    cr_assert_eq(edgefirst_pointfield_get_count(field), 1);
    
    edgefirst_pointfield_destroy(field);
}

Test(sensor_msgs, pointfield_datatype_constants) {
    // Verify ROS2 datatype constants match expected values
    cr_assert_eq(POINTFIELD_INT8, 1);
    cr_assert_eq(POINTFIELD_UINT8, 2);
    cr_assert_eq(POINTFIELD_INT16, 3);
    cr_assert_eq(POINTFIELD_UINT16, 4);
    cr_assert_eq(POINTFIELD_INT32, 5);
    cr_assert_eq(POINTFIELD_UINT32, 6);
    cr_assert_eq(POINTFIELD_FLOAT32, 7);
    cr_assert_eq(POINTFIELD_FLOAT64, 8);
}

Test(sensor_msgs, pointfield_serialize_deserialize) {
    PointField *original = edgefirst_pointfield_create();
    cr_assert_not_null(original);
    
    edgefirst_pointfield_set_name(original, "z");
    edgefirst_pointfield_set_offset(original, 8);
    edgefirst_pointfield_set_datatype(original, POINTFIELD_FLOAT32);
    edgefirst_pointfield_set_count(original, 1);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_pointfield_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    PointField *deserialized = edgefirst_pointfield_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    const char *name = edgefirst_pointfield_get_name(deserialized);
    cr_assert_str_eq(name, "z");
    cr_assert_eq(edgefirst_pointfield_get_offset(deserialized), 8);
    cr_assert_eq(edgefirst_pointfield_get_datatype(deserialized), POINTFIELD_FLOAT32);
    
    edgefirst_pointfield_destroy(original);
    edgefirst_pointfield_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

// ============================================================================
// PointCloud2 Tests
// ============================================================================

Test(sensor_msgs, pointcloud2_create_and_destroy) {
    PointCloud2 *cloud = edgefirst_pointcloud2_create();
    cr_assert_not_null(cloud);
    
    Header *header = edgefirst_pointcloud2_get_header(cloud);
    cr_assert_not_null(header);
    
    cr_assert_eq(edgefirst_pointcloud2_get_height(cloud), 0);
    cr_assert_eq(edgefirst_pointcloud2_get_width(cloud), 0);
    cr_assert_eq(edgefirst_pointcloud2_get_point_step(cloud), 0);
    cr_assert_eq(edgefirst_pointcloud2_get_row_step(cloud), 0);
    cr_assert_eq(edgefirst_pointcloud2_is_dense(cloud), 0);
    
    edgefirst_pointcloud2_destroy(cloud);
}

Test(sensor_msgs, pointcloud2_set_dimensions) {
    PointCloud2 *cloud = edgefirst_pointcloud2_create();
    cr_assert_not_null(cloud);
    
    edgefirst_pointcloud2_set_height(cloud, 1);
    edgefirst_pointcloud2_set_width(cloud, 1000);
    edgefirst_pointcloud2_set_point_step(cloud, 16);
    edgefirst_pointcloud2_set_row_step(cloud, 16000);
    edgefirst_pointcloud2_set_is_dense(cloud, 1);
    
    cr_assert_eq(edgefirst_pointcloud2_get_height(cloud), 1);
    cr_assert_eq(edgefirst_pointcloud2_get_width(cloud), 1000);
    cr_assert_eq(edgefirst_pointcloud2_get_point_step(cloud), 16);
    cr_assert_eq(edgefirst_pointcloud2_get_row_step(cloud), 16000);
    cr_assert_eq(edgefirst_pointcloud2_is_dense(cloud), 1);
    
    edgefirst_pointcloud2_destroy(cloud);
}

Test(sensor_msgs, pointcloud2_set_is_bigendian) {
    PointCloud2 *cloud = edgefirst_pointcloud2_create();
    cr_assert_not_null(cloud);
    
    edgefirst_pointcloud2_set_is_bigendian(cloud, 0);
    cr_assert_eq(edgefirst_pointcloud2_is_bigendian(cloud), 0);
    
    edgefirst_pointcloud2_set_is_bigendian(cloud, 1);
    cr_assert_eq(edgefirst_pointcloud2_is_bigendian(cloud), 1);
    
    edgefirst_pointcloud2_destroy(cloud);
}

Test(sensor_msgs, pointcloud2_set_data) {
    PointCloud2 *cloud = edgefirst_pointcloud2_create();
    cr_assert_not_null(cloud);
    
    uint8_t test_data[48]; // 3 points * 16 bytes
    for (int i = 0; i < 48; i++) {
        test_data[i] = (uint8_t)i;
    }
    
    int ret = edgefirst_pointcloud2_set_data(cloud, test_data, 48);
    cr_assert_eq(ret, 0);
    
    size_t data_len = 0;
    const uint8_t *data = edgefirst_pointcloud2_get_data(cloud, &data_len);
    cr_assert_eq(data_len, 48);
    
    for (size_t i = 0; i < 48; i++) {
        cr_assert_eq(data[i], test_data[i]);
    }
    
    edgefirst_pointcloud2_destroy(cloud);
}

Test(sensor_msgs, pointcloud2_serialize_deserialize) {
    PointCloud2 *original = edgefirst_pointcloud2_create();
    cr_assert_not_null(original);
    
    edgefirst_pointcloud2_set_height(original, 1);
    edgefirst_pointcloud2_set_width(original, 100);
    edgefirst_pointcloud2_set_point_step(original, 12);
    edgefirst_pointcloud2_set_row_step(original, 1200);
    edgefirst_pointcloud2_set_is_bigendian(original, 0);
    edgefirst_pointcloud2_set_is_dense(original, 1);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_pointcloud2_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    PointCloud2 *deserialized = edgefirst_pointcloud2_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    cr_assert_eq(edgefirst_pointcloud2_get_height(deserialized), 1);
    cr_assert_eq(edgefirst_pointcloud2_get_width(deserialized), 100);
    cr_assert_eq(edgefirst_pointcloud2_get_point_step(deserialized), 12);
    cr_assert_eq(edgefirst_pointcloud2_is_dense(deserialized), 1);
    
    edgefirst_pointcloud2_destroy(original);
    edgefirst_pointcloud2_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

// ============================================================================
// NavSatStatus Tests
// ============================================================================

Test(sensor_msgs, navsatstatus_create_and_destroy) {
    NavSatStatus *status = edgefirst_navsatstatus_create();
    cr_assert_not_null(status);
    
    cr_assert_eq(edgefirst_navsatstatus_get_status(status), 0);
    cr_assert_eq(edgefirst_navsatstatus_get_service(status), 0);
    
    edgefirst_navsatstatus_destroy(status);
}

Test(sensor_msgs, navsatstatus_service_constants) {
    // Verify ROS2 service constants
    cr_assert_eq(NAVSATSTATUS_SERVICE_GPS, 1);
    cr_assert_eq(NAVSATSTATUS_SERVICE_GLONASS, 2);
    cr_assert_eq(NAVSATSTATUS_SERVICE_COMPASS, 4);
    cr_assert_eq(NAVSATSTATUS_SERVICE_GALILEO, 8);
}

Test(sensor_msgs, navsatstatus_status_constants) {
    // Verify ROS2 status constants
    cr_assert_eq(NAVSATSTATUS_STATUS_NO_FIX, -1);
    cr_assert_eq(NAVSATSTATUS_STATUS_FIX, 0);
    cr_assert_eq(NAVSATSTATUS_STATUS_SBAS_FIX, 1);
    cr_assert_eq(NAVSATSTATUS_STATUS_GBAS_FIX, 2);
}

Test(sensor_msgs, navsatstatus_set_values) {
    NavSatStatus *status = edgefirst_navsatstatus_create();
    cr_assert_not_null(status);
    
    edgefirst_navsatstatus_set_status(status, NAVSATSTATUS_STATUS_FIX);
    edgefirst_navsatstatus_set_service(status, NAVSATSTATUS_SERVICE_GPS | NAVSATSTATUS_SERVICE_GALILEO);
    
    cr_assert_eq(edgefirst_navsatstatus_get_status(status), NAVSATSTATUS_STATUS_FIX);
    cr_assert_eq(edgefirst_navsatstatus_get_service(status), NAVSATSTATUS_SERVICE_GPS | NAVSATSTATUS_SERVICE_GALILEO);
    
    edgefirst_navsatstatus_destroy(status);
}

Test(sensor_msgs, navsatstatus_serialize_deserialize) {
    NavSatStatus *original = edgefirst_navsatstatus_create();
    cr_assert_not_null(original);
    
    edgefirst_navsatstatus_set_status(original, NAVSATSTATUS_STATUS_SBAS_FIX);
    edgefirst_navsatstatus_set_service(original, NAVSATSTATUS_SERVICE_GPS);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_navsatstatus_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    NavSatStatus *deserialized = edgefirst_navsatstatus_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    cr_assert_eq(edgefirst_navsatstatus_get_status(deserialized), NAVSATSTATUS_STATUS_SBAS_FIX);
    cr_assert_eq(edgefirst_navsatstatus_get_service(deserialized), NAVSATSTATUS_SERVICE_GPS);
    
    edgefirst_navsatstatus_destroy(original);
    edgefirst_navsatstatus_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

// ============================================================================
// NavSatFix Tests
// ============================================================================

Test(sensor_msgs, navsatfix_create_and_destroy) {
    NavSatFix *fix = edgefirst_navsatfix_create();
    cr_assert_not_null(fix);
    
    Header *header = edgefirst_navsatfix_get_header(fix);
    cr_assert_not_null(header);
    
    NavSatStatus *status = edgefirst_navsatfix_get_status(fix);
    cr_assert_not_null(status);
    
    cr_assert_float_eq(edgefirst_navsatfix_get_latitude(fix), 0.0, 0.0001);
    cr_assert_float_eq(edgefirst_navsatfix_get_longitude(fix), 0.0, 0.0001);
    cr_assert_float_eq(edgefirst_navsatfix_get_altitude(fix), 0.0, 0.0001);
    
    edgefirst_navsatfix_destroy(fix);
}

Test(sensor_msgs, navsatfix_covariance_type_constants) {
    // Verify covariance type constants
    cr_assert_eq(NAVSATFIX_COVARIANCE_TYPE_UNKNOWN, 0);
    cr_assert_eq(NAVSATFIX_COVARIANCE_TYPE_APPROXIMATED, 1);
    cr_assert_eq(NAVSATFIX_COVARIANCE_TYPE_DIAGONAL_KNOWN, 2);
    cr_assert_eq(NAVSATFIX_COVARIANCE_TYPE_KNOWN, 3);
}

Test(sensor_msgs, navsatfix_set_position) {
    NavSatFix *fix = edgefirst_navsatfix_create();
    cr_assert_not_null(fix);
    
    edgefirst_navsatfix_set_latitude(fix, 45.5);
    edgefirst_navsatfix_set_longitude(fix, -73.6);
    edgefirst_navsatfix_set_altitude(fix, 100.5);
    
    cr_assert_float_eq(edgefirst_navsatfix_get_latitude(fix), 45.5, 0.0001);
    cr_assert_float_eq(edgefirst_navsatfix_get_longitude(fix), -73.6, 0.0001);
    cr_assert_float_eq(edgefirst_navsatfix_get_altitude(fix), 100.5, 0.0001);
    
    edgefirst_navsatfix_destroy(fix);
}

Test(sensor_msgs, navsatfix_set_covariance) {
    NavSatFix *fix = edgefirst_navsatfix_create();
    cr_assert_not_null(fix);
    
    double covariance[9] = {
        1.0, 0.0, 0.0,
        0.0, 2.0, 0.0,
        0.0, 0.0, 3.0
    };
    
    int ret = edgefirst_navsatfix_set_position_covariance(fix, covariance);
    cr_assert_eq(ret, 0);
    
    const double *got_cov = edgefirst_navsatfix_get_position_covariance(fix);
    cr_assert_not_null(got_cov);
    
    for (int i = 0; i < 9; i++) {
        cr_assert_float_eq(got_cov[i], covariance[i], 0.0001);
    }
    
    edgefirst_navsatfix_destroy(fix);
}

Test(sensor_msgs, navsatfix_set_covariance_type) {
    NavSatFix *fix = edgefirst_navsatfix_create();
    cr_assert_not_null(fix);
    
    edgefirst_navsatfix_set_position_covariance_type(fix, NAVSATFIX_COVARIANCE_TYPE_KNOWN);
    
    cr_assert_eq(edgefirst_navsatfix_get_position_covariance_type(fix), NAVSATFIX_COVARIANCE_TYPE_KNOWN);
    
    edgefirst_navsatfix_destroy(fix);
}

Test(sensor_msgs, navsatfix_serialize_deserialize) {
    NavSatFix *original = edgefirst_navsatfix_create();
    cr_assert_not_null(original);
    
    edgefirst_navsatfix_set_latitude(original, 40.7128);
    edgefirst_navsatfix_set_longitude(original, -74.0060);
    edgefirst_navsatfix_set_altitude(original, 10.0);
    edgefirst_navsatfix_set_position_covariance_type(original, NAVSATFIX_COVARIANCE_TYPE_APPROXIMATED);
    
    NavSatStatus *status = edgefirst_navsatstatus_create();
    edgefirst_navsatstatus_set_status(status, NAVSATSTATUS_STATUS_FIX);
    edgefirst_navsatstatus_set_service(status, NAVSATSTATUS_SERVICE_GPS);
    edgefirst_navsatfix_set_status(original, status);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_navsatfix_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    NavSatFix *deserialized = edgefirst_navsatfix_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    cr_assert_float_eq(edgefirst_navsatfix_get_latitude(deserialized), 40.7128, 0.0001);
    cr_assert_float_eq(edgefirst_navsatfix_get_longitude(deserialized), -74.0060, 0.0001);
    cr_assert_float_eq(edgefirst_navsatfix_get_altitude(deserialized), 10.0, 0.0001);
    
    NavSatStatus *deser_status = edgefirst_navsatfix_get_status(deserialized);
    cr_assert_eq(edgefirst_navsatstatus_get_status(deser_status), NAVSATSTATUS_STATUS_FIX);
    
    edgefirst_navsatstatus_destroy(status);
    edgefirst_navsatfix_destroy(original);
    edgefirst_navsatfix_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}
