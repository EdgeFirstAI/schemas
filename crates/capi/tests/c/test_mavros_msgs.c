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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

static uint8_t *_load_fixture_mav(const char *relpath, size_t *out_len) {
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

// ============================================================================
// Altitude
// ============================================================================

Test(mavros_msgs, altitude_from_cdr_null) {
    errno = 0;
    mavros_msgs_altitude_t *h = mavros_msgs_altitude_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, altitude_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    mavros_msgs_altitude_t *h = mavros_msgs_altitude_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, altitude_free_null) {
    mavros_msgs_altitude_free(NULL); /* must not crash */
}

// ============================================================================
// VfrHud
// ============================================================================

Test(mavros_msgs, vfrhud_from_cdr_null) {
    errno = 0;
    mavros_msgs_vfrhud_t *h = mavros_msgs_vfrhud_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, vfrhud_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    mavros_msgs_vfrhud_t *h = mavros_msgs_vfrhud_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, vfrhud_free_null) {
    mavros_msgs_vfrhud_free(NULL);
}

// ============================================================================
// EstimatorStatus
// ============================================================================

Test(mavros_msgs, estimator_status_from_cdr_null) {
    errno = 0;
    mavros_msgs_estimator_status_t *h = mavros_msgs_estimator_status_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, estimator_status_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD};
    errno = 0;
    mavros_msgs_estimator_status_t *h = mavros_msgs_estimator_status_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, estimator_status_free_null) {
    mavros_msgs_estimator_status_free(NULL);
}

// ============================================================================
// ExtendedState
// ============================================================================

Test(mavros_msgs, extended_state_from_cdr_null) {
    errno = 0;
    mavros_msgs_extended_state_t *h = mavros_msgs_extended_state_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, extended_state_from_cdr_invalid) {
    uint8_t bad[] = {0x01};
    errno = 0;
    mavros_msgs_extended_state_t *h = mavros_msgs_extended_state_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, extended_state_free_null) {
    mavros_msgs_extended_state_free(NULL);
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
    mavros_msgs_sys_status_t *h = mavros_msgs_sys_status_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, sys_status_from_cdr_invalid) {
    uint8_t bad[] = {0xFF, 0xFF, 0xFF, 0xFF};
    errno = 0;
    mavros_msgs_sys_status_t *h = mavros_msgs_sys_status_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, sys_status_free_null) {
    mavros_msgs_sys_status_free(NULL);
}

// ============================================================================
// State
// ============================================================================

Test(mavros_msgs, state_from_cdr_null) {
    errno = 0;
    mavros_msgs_state_t *h = mavros_msgs_state_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, state_from_cdr_invalid) {
    uint8_t bad[] = {0x00, 0x01};
    errno = 0;
    mavros_msgs_state_t *h = mavros_msgs_state_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, state_free_null) {
    mavros_msgs_state_free(NULL);
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
    mavros_msgs_status_text_t *h = mavros_msgs_status_text_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, status_text_from_cdr_invalid) {
    uint8_t bad[] = {0xCA, 0xFE};
    errno = 0;
    mavros_msgs_status_text_t *h = mavros_msgs_status_text_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, status_text_free_null) {
    mavros_msgs_status_text_free(NULL);
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
    mavros_msgs_gps_raw_t *h = mavros_msgs_gps_raw_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, gps_raw_from_cdr_invalid) {
    uint8_t bad[] = {0x00, 0x01, 0x02, 0x03};
    errno = 0;
    mavros_msgs_gps_raw_t *h = mavros_msgs_gps_raw_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, gps_raw_free_null) {
    mavros_msgs_gps_raw_free(NULL);
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
    mavros_msgs_timesync_status_t *h = mavros_msgs_timesync_status_from_cdr(NULL, 100);
    cr_assert_null(h);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, timesync_status_from_cdr_invalid) {
    uint8_t bad[] = {0xBA, 0xDC, 0x0D, 0xE0};
    errno = 0;
    mavros_msgs_timesync_status_t *h = mavros_msgs_timesync_status_from_cdr(bad, sizeof(bad));
    cr_assert_null(h);
    cr_assert_eq(errno, EBADMSG);
}

Test(mavros_msgs, timesync_status_free_null) {
    mavros_msgs_timesync_status_free(NULL);
}

Test(mavros_msgs, altitude_builder_null) {
    errno = 0;
    cr_assert_eq(mavros_msgs_altitude_builder_set_frame_id(NULL, "x"), -1);
    cr_assert_eq(errno, EINVAL);
}

Test(mavros_msgs, altitude_builder_matches_golden) {
    size_t golden_len = 0;
    uint8_t *golden = _load_fixture_mav("testdata/cdr/mavros_msgs/Altitude.cdr",
                                       &golden_len);
    cr_assert_not_null(golden, "failed to load Altitude fixture");

    mavros_msgs_altitude_builder_t *b = mavros_msgs_altitude_builder_new();
    cr_assert_not_null(b);
    mavros_msgs_altitude_builder_set_stamp(b, 1234567890, 123456789u);
    cr_assert_eq(mavros_msgs_altitude_builder_set_frame_id(b, "test_frame"), 0);
    mavros_msgs_altitude_builder_set_monotonic(b, 100.0f);
    mavros_msgs_altitude_builder_set_amsl(b, 50.0f);
    mavros_msgs_altitude_builder_set_local(b, 10.0f);
    mavros_msgs_altitude_builder_set_relative(b, 5.0f);
    mavros_msgs_altitude_builder_set_terrain(b, 2.0f);
    mavros_msgs_altitude_builder_set_bottom_clearance(b, 1.5f);

    uint8_t *out = NULL;
    size_t out_len = 0;
    cr_assert_eq(mavros_msgs_altitude_builder_build(b, &out, &out_len), 0);
    cr_assert_eq(out_len, golden_len);
    cr_assert_eq(memcmp(out, golden, golden_len), 0);

    edgefirst_schemas_bytes_free(out, out_len);
    mavros_msgs_altitude_builder_free(b);
    free(golden);
}
