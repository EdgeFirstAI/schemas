/**
 * @file test_mavros_msgs.c
 * @brief Criterion tests for mavros_msgs types
 *
 * Buffer-backed types: Altitude, VfrHud, EstimatorStatus, ExtendedState,
 *   SysStatus, State, StatusText, GpsRaw, TimesyncStatus
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Altitude
// ============================================================================

Test(mavros_msgs, altitude_from_cdr_null) {
    errno = 0;
    ros_mavros_altitude_t *h = ros_mavros_altitude_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, altitude_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_mavros_altitude_t *h = ros_mavros_altitude_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, altitude_free_null) {
    ros_mavros_altitude_free(NULL); /* must not crash */
}

// ============================================================================
// VfrHud
// ============================================================================

Test(mavros_msgs, vfrhud_from_cdr_null) {
    errno = 0;
    ros_mavros_vfrhud_t *h = ros_mavros_vfrhud_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, vfrhud_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    ros_mavros_vfrhud_t *h = ros_mavros_vfrhud_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, vfrhud_free_null) {
    ros_mavros_vfrhud_free(NULL);
}

// ============================================================================
// EstimatorStatus
// ============================================================================

Test(mavros_msgs, estimator_status_from_cdr_null) {
    errno = 0;
    ros_mavros_estimator_status_t *h = ros_mavros_estimator_status_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, estimator_status_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD};
    errno = 0;
    ros_mavros_estimator_status_t *h = ros_mavros_estimator_status_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, estimator_status_free_null) {
    ros_mavros_estimator_status_free(NULL);
}

// ============================================================================
// ExtendedState
// ============================================================================

Test(mavros_msgs, extended_state_from_cdr_null) {
    errno = 0;
    ros_mavros_extended_state_t *h = ros_mavros_extended_state_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, extended_state_from_cdr_invalid) {
    uint8_t bad[] = {0x01};
    errno = 0;
    ros_mavros_extended_state_t *h = ros_mavros_extended_state_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, extended_state_free_null) {
    ros_mavros_extended_state_free(NULL);
}

Test(mavros_msgs, extended_state_constants) {
    cr_assert_eq(ROS_MAVROS_VTOL_STATE_UNDEFINED, 0);
    cr_assert_eq(ROS_MAVROS_VTOL_STATE_FW, 4);
    cr_assert_eq(ROS_MAVROS_LANDED_STATE_IN_AIR, 2);
}

// ============================================================================
// SysStatus
// ============================================================================

Test(mavros_msgs, sys_status_from_cdr_null) {
    errno = 0;
    ros_mavros_sys_status_t *h = ros_mavros_sys_status_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, sys_status_from_cdr_invalid) {
    uint8_t bad[] = {0xFF, 0xFF, 0xFF, 0xFF};
    errno = 0;
    ros_mavros_sys_status_t *h = ros_mavros_sys_status_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, sys_status_free_null) {
    ros_mavros_sys_status_free(NULL);
}

// ============================================================================
// State
// ============================================================================

Test(mavros_msgs, state_from_cdr_null) {
    errno = 0;
    ros_mavros_state_t *h = ros_mavros_state_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, state_from_cdr_invalid) {
    uint8_t bad[] = {0x00, 0x01};
    errno = 0;
    ros_mavros_state_t *h = ros_mavros_state_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, state_free_null) {
    ros_mavros_state_free(NULL);
}

Test(mavros_msgs, state_constants) {
    cr_assert_eq(ROS_MAVROS_MAV_STATE_UNINIT, 0);
    cr_assert_eq(ROS_MAVROS_MAV_STATE_ACTIVE, 4);
    cr_assert_eq(ROS_MAVROS_MAV_STATE_FLIGHT_TERMINATION, 8);
}

// ============================================================================
// StatusText
// ============================================================================

Test(mavros_msgs, status_text_from_cdr_null) {
    errno = 0;
    ros_mavros_status_text_t *h = ros_mavros_status_text_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, status_text_from_cdr_invalid) {
    uint8_t bad[] = {0xCA, 0xFE};
    errno = 0;
    ros_mavros_status_text_t *h = ros_mavros_status_text_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, status_text_free_null) {
    ros_mavros_status_text_free(NULL);
}

Test(mavros_msgs, status_text_severity_constants) {
    cr_assert_eq(ROS_MAVROS_SEVERITY_EMERGENCY, 0);
    cr_assert_eq(ROS_MAVROS_SEVERITY_DEBUG, 7);
}

// ============================================================================
// GpsRaw
// ============================================================================

Test(mavros_msgs, gps_raw_from_cdr_null) {
    errno = 0;
    ros_mavros_gps_raw_t *h = ros_mavros_gps_raw_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, gps_raw_from_cdr_invalid) {
    uint8_t bad[] = {0x00, 0x01, 0x02, 0x03};
    errno = 0;
    ros_mavros_gps_raw_t *h = ros_mavros_gps_raw_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, gps_raw_free_null) {
    ros_mavros_gps_raw_free(NULL);
}

Test(mavros_msgs, gps_raw_fix_type_constants) {
    cr_assert_eq(ROS_MAVROS_GPS_FIX_TYPE_NO_GPS, 0);
    cr_assert_eq(ROS_MAVROS_GPS_FIX_TYPE_3D_FIX, 3);
    cr_assert_eq(ROS_MAVROS_GPS_FIX_TYPE_PPP, 8);
}

// ============================================================================
// TimesyncStatus
// ============================================================================

Test(mavros_msgs, timesync_status_from_cdr_null) {
    errno = 0;
    ros_mavros_timesync_status_t *h = ros_mavros_timesync_status_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, timesync_status_from_cdr_invalid) {
    uint8_t bad[] = {0xBA, 0xDC, 0x0D, 0xE0};
    errno = 0;
    ros_mavros_timesync_status_t *h = ros_mavros_timesync_status_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, timesync_status_free_null) {
    ros_mavros_timesync_status_free(NULL);
}
