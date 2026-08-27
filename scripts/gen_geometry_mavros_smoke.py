#!/usr/bin/env python3
"""Generate crates/capi/tests/geometry_mavros_builder_smoke.rs"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "crates/capi/tests/geometry_mavros_builder_smoke.rs"

HEADER = r'''// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Builder FFI smoke tests for geometry_msgs, nav_msgs/Odometry, and mavros_msgs
//! types added in PR 1. Each test verifies C FFI encode_into output matches the
//! native Rust builder for the same golden field values.

#![allow(non_camel_case_types, clippy::too_many_lines)]

use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::geometry_msgs::{
    self, Accel, AccelWithCovariance, Inertia, Point, Point32, Pose, PoseWithCovariance,
    Quaternion, Transform, Twist, TwistWithCovariance, Vector3, Wrench,
};
use edgefirst_schemas::mavros_msgs;
use edgefirst_schemas::nav_msgs;
use std::ffi::CString;
use std::os::raw::c_char;

const STAMP_SEC: i32 = 1234567890;
const STAMP_NSEC: u32 = 123456789;

fn stamp() -> Time {
    Time::new(STAMP_SEC, STAMP_NSEC)
}

fn frame_id_cstr() -> CString {
    CString::new("test_frame").unwrap()
}

fn pose_cov() -> [f64; 36] {
    let mut cov = [0.0_f64; 36];
    for i in 0..6 {
        cov[i * 6 + i] = 0.1 * (i as f64 + 1.0);
    }
    cov[1] = 0.01;
    cov[6] = 0.01;
    cov
}

fn twist_cov() -> [f64; 36] {
    let mut cov = [0.0_f64; 36];
    for i in 0..6 {
        cov[i * 6 + i] = 0.02 * (i as f64 + 1.0);
    }
    cov[7] = 0.001;
    cov
}

fn accel_cov() -> [f64; 36] {
    let mut cov = [0.0_f64; 36];
    for i in 0..6 {
        cov[i * 6 + i] = 0.1 * (i as f64 + 1.0);
    }
    cov[1] = 0.01;
    cov[6] = 0.01;
    cov
}

macro_rules! stamped_setup {
    ($b:expr, $set_stamp:path, $set_frame:path) => {{
        $set_stamp($b, STAMP_SEC, STAMP_NSEC);
        let frame = frame_id_cstr();
        assert_eq!($set_frame($b, frame.as_ptr()), 0);
    }};
}

'''

OPAQUES = """
enum ros_point_stamped_builder_t {}
enum ros_vector3_stamped_builder_t {}
enum ros_pose_stamped_builder_t {}
enum ros_quaternion_stamped_builder_t {}
enum ros_accel_stamped_builder_t {}
enum ros_twist_stamped_builder_t {}
enum ros_wrench_stamped_builder_t {}
enum ros_inertia_stamped_builder_t {}
enum ros_transform_stamped_builder_t {}
enum ros_pose_with_covariance_stamped_builder_t {}
enum ros_twist_with_covariance_stamped_builder_t {}
enum ros_accel_with_covariance_stamped_builder_t {}
enum ros_polygon_builder_t {}
enum ros_polygon_stamped_builder_t {}
enum ros_pose_array_builder_t {}
enum ros_odometry_builder_t {}
enum ros_mavros_altitude_builder_t {}
enum ros_mavros_vfrhud_builder_t {}
enum ros_mavros_estimator_status_builder_t {}
enum ros_mavros_extended_state_builder_t {}
enum ros_mavros_sys_status_builder_t {}
enum ros_mavros_state_builder_t {}
enum ros_mavros_status_text_builder_t {}
enum ros_mavros_gps_raw_builder_t {}
enum ros_mavros_timesync_status_builder_t {}
"""

EXTERN = r'''
extern "C" {
    fn ros_point_stamped_builder_new() -> *mut ros_point_stamped_builder_t;
    fn ros_point_stamped_builder_free(b: *mut ros_point_stamped_builder_t);
    fn ros_point_stamped_builder_set_stamp(b: *mut ros_point_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_point_stamped_builder_set_frame_id(b: *mut ros_point_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_point_stamped_builder_set_point(b: *mut ros_point_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_point_stamped_builder_encode_into(b: *mut ros_point_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_vector3_stamped_builder_new() -> *mut ros_vector3_stamped_builder_t;
    fn ros_vector3_stamped_builder_free(b: *mut ros_vector3_stamped_builder_t);
    fn ros_vector3_stamped_builder_set_stamp(b: *mut ros_vector3_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_vector3_stamped_builder_set_frame_id(b: *mut ros_vector3_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_vector3_stamped_builder_set_vector(b: *mut ros_vector3_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_vector3_stamped_builder_encode_into(b: *mut ros_vector3_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_pose_stamped_builder_new() -> *mut ros_pose_stamped_builder_t;
    fn ros_pose_stamped_builder_free(b: *mut ros_pose_stamped_builder_t);
    fn ros_pose_stamped_builder_set_stamp(b: *mut ros_pose_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_pose_stamped_builder_set_frame_id(b: *mut ros_pose_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_pose_stamped_builder_set_position(b: *mut ros_pose_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_pose_stamped_builder_set_orientation(b: *mut ros_pose_stamped_builder_t, x: f64, y: f64, z: f64, w: f64);
    fn ros_pose_stamped_builder_encode_into(b: *mut ros_pose_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_quaternion_stamped_builder_new() -> *mut ros_quaternion_stamped_builder_t;
    fn ros_quaternion_stamped_builder_free(b: *mut ros_quaternion_stamped_builder_t);
    fn ros_quaternion_stamped_builder_set_stamp(b: *mut ros_quaternion_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_quaternion_stamped_builder_set_frame_id(b: *mut ros_quaternion_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_quaternion_stamped_builder_set_quaternion(b: *mut ros_quaternion_stamped_builder_t, x: f64, y: f64, z: f64, w: f64);
    fn ros_quaternion_stamped_builder_encode_into(b: *mut ros_quaternion_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_accel_stamped_builder_new() -> *mut ros_accel_stamped_builder_t;
    fn ros_accel_stamped_builder_free(b: *mut ros_accel_stamped_builder_t);
    fn ros_accel_stamped_builder_set_stamp(b: *mut ros_accel_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_accel_stamped_builder_set_frame_id(b: *mut ros_accel_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_accel_stamped_builder_set_linear_acceleration(b: *mut ros_accel_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_accel_stamped_builder_set_angular_acceleration(b: *mut ros_accel_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_accel_stamped_builder_encode_into(b: *mut ros_accel_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_twist_stamped_builder_new() -> *mut ros_twist_stamped_builder_t;
    fn ros_twist_stamped_builder_free(b: *mut ros_twist_stamped_builder_t);
    fn ros_twist_stamped_builder_set_stamp(b: *mut ros_twist_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_twist_stamped_builder_set_frame_id(b: *mut ros_twist_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_twist_stamped_builder_set_linear(b: *mut ros_twist_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_twist_stamped_builder_set_angular(b: *mut ros_twist_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_twist_stamped_builder_encode_into(b: *mut ros_twist_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_wrench_stamped_builder_new() -> *mut ros_wrench_stamped_builder_t;
    fn ros_wrench_stamped_builder_free(b: *mut ros_wrench_stamped_builder_t);
    fn ros_wrench_stamped_builder_set_stamp(b: *mut ros_wrench_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_wrench_stamped_builder_set_frame_id(b: *mut ros_wrench_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_wrench_stamped_builder_set_force(b: *mut ros_wrench_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_wrench_stamped_builder_set_torque(b: *mut ros_wrench_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_wrench_stamped_builder_encode_into(b: *mut ros_wrench_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_inertia_stamped_builder_new() -> *mut ros_inertia_stamped_builder_t;
    fn ros_inertia_stamped_builder_free(b: *mut ros_inertia_stamped_builder_t);
    fn ros_inertia_stamped_builder_set_stamp(b: *mut ros_inertia_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_inertia_stamped_builder_set_frame_id(b: *mut ros_inertia_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_inertia_stamped_builder_set_mass(b: *mut ros_inertia_stamped_builder_t, m: f64);
    fn ros_inertia_stamped_builder_set_com(b: *mut ros_inertia_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_inertia_stamped_builder_set_inertia_tensor(b: *mut ros_inertia_stamped_builder_t, ixx: f64, ixy: f64, ixz: f64, iyy: f64, iyz: f64, izz: f64);
    fn ros_inertia_stamped_builder_encode_into(b: *mut ros_inertia_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_transform_stamped_builder_new() -> *mut ros_transform_stamped_builder_t;
    fn ros_transform_stamped_builder_free(b: *mut ros_transform_stamped_builder_t);
    fn ros_transform_stamped_builder_set_stamp(b: *mut ros_transform_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_transform_stamped_builder_set_frame_id(b: *mut ros_transform_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_transform_stamped_builder_set_child_frame_id(b: *mut ros_transform_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_transform_stamped_builder_set_translation(b: *mut ros_transform_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_transform_stamped_builder_set_rotation(b: *mut ros_transform_stamped_builder_t, x: f64, y: f64, z: f64, w: f64);
    fn ros_transform_stamped_builder_encode_into(b: *mut ros_transform_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_pose_with_covariance_stamped_builder_new() -> *mut ros_pose_with_covariance_stamped_builder_t;
    fn ros_pose_with_covariance_stamped_builder_free(b: *mut ros_pose_with_covariance_stamped_builder_t);
    fn ros_pose_with_covariance_stamped_builder_set_stamp(b: *mut ros_pose_with_covariance_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_pose_with_covariance_stamped_builder_set_frame_id(b: *mut ros_pose_with_covariance_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_pose_with_covariance_stamped_builder_set_position(b: *mut ros_pose_with_covariance_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_pose_with_covariance_stamped_builder_set_orientation(b: *mut ros_pose_with_covariance_stamped_builder_t, x: f64, y: f64, z: f64, w: f64);
    fn ros_pose_with_covariance_stamped_builder_set_covariance(b: *mut ros_pose_with_covariance_stamped_builder_t, cov: *const f64) -> i32;
    fn ros_pose_with_covariance_stamped_builder_encode_into(b: *mut ros_pose_with_covariance_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_twist_with_covariance_stamped_builder_new() -> *mut ros_twist_with_covariance_stamped_builder_t;
    fn ros_twist_with_covariance_stamped_builder_free(b: *mut ros_twist_with_covariance_stamped_builder_t);
    fn ros_twist_with_covariance_stamped_builder_set_stamp(b: *mut ros_twist_with_covariance_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_twist_with_covariance_stamped_builder_set_frame_id(b: *mut ros_twist_with_covariance_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_twist_with_covariance_stamped_builder_set_linear(b: *mut ros_twist_with_covariance_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_twist_with_covariance_stamped_builder_set_angular(b: *mut ros_twist_with_covariance_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_twist_with_covariance_stamped_builder_set_covariance(b: *mut ros_twist_with_covariance_stamped_builder_t, cov: *const f64) -> i32;
    fn ros_twist_with_covariance_stamped_builder_encode_into(b: *mut ros_twist_with_covariance_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_accel_with_covariance_stamped_builder_new() -> *mut ros_accel_with_covariance_stamped_builder_t;
    fn ros_accel_with_covariance_stamped_builder_free(b: *mut ros_accel_with_covariance_stamped_builder_t);
    fn ros_accel_with_covariance_stamped_builder_set_stamp(b: *mut ros_accel_with_covariance_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_accel_with_covariance_stamped_builder_set_frame_id(b: *mut ros_accel_with_covariance_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_accel_with_covariance_stamped_builder_set_linear_acceleration(b: *mut ros_accel_with_covariance_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_accel_with_covariance_stamped_builder_set_angular_acceleration(b: *mut ros_accel_with_covariance_stamped_builder_t, x: f64, y: f64, z: f64);
    fn ros_accel_with_covariance_stamped_builder_set_covariance(b: *mut ros_accel_with_covariance_stamped_builder_t, cov: *const f64) -> i32;
    fn ros_accel_with_covariance_stamped_builder_encode_into(b: *mut ros_accel_with_covariance_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_polygon_builder_new() -> *mut ros_polygon_builder_t;
    fn ros_polygon_builder_free(b: *mut ros_polygon_builder_t);
    fn ros_polygon_builder_set_points(b: *mut ros_polygon_builder_t, xyz: *const f32, count: usize) -> i32;
    fn ros_polygon_builder_encode_into(b: *mut ros_polygon_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_polygon_stamped_builder_new() -> *mut ros_polygon_stamped_builder_t;
    fn ros_polygon_stamped_builder_free(b: *mut ros_polygon_stamped_builder_t);
    fn ros_polygon_stamped_builder_set_stamp(b: *mut ros_polygon_stamped_builder_t, sec: i32, nsec: u32);
    fn ros_polygon_stamped_builder_set_frame_id(b: *mut ros_polygon_stamped_builder_t, s: *const c_char) -> i32;
    fn ros_polygon_stamped_builder_set_points(b: *mut ros_polygon_stamped_builder_t, xyz: *const f32, count: usize) -> i32;
    fn ros_polygon_stamped_builder_encode_into(b: *mut ros_polygon_stamped_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_pose_array_builder_new() -> *mut ros_pose_array_builder_t;
    fn ros_pose_array_builder_free(b: *mut ros_pose_array_builder_t);
    fn ros_pose_array_builder_set_stamp(b: *mut ros_pose_array_builder_t, sec: i32, nsec: u32);
    fn ros_pose_array_builder_set_frame_id(b: *mut ros_pose_array_builder_t, s: *const c_char) -> i32;
    fn ros_pose_array_builder_set_poses(b: *mut ros_pose_array_builder_t, poses: *const f64, count: usize) -> i32;
    fn ros_pose_array_builder_encode_into(b: *mut ros_pose_array_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_odometry_builder_new() -> *mut ros_odometry_builder_t;
    fn ros_odometry_builder_free(b: *mut ros_odometry_builder_t);
    fn ros_odometry_builder_set_stamp(b: *mut ros_odometry_builder_t, sec: i32, nsec: u32);
    fn ros_odometry_builder_set_frame_id(b: *mut ros_odometry_builder_t, s: *const c_char) -> i32;
    fn ros_odometry_builder_set_child_frame_id(b: *mut ros_odometry_builder_t, s: *const c_char) -> i32;
    fn ros_odometry_builder_set_pose(b: *mut ros_odometry_builder_t, px: f64, py: f64, pz: f64, ox: f64, oy: f64, oz: f64, ow: f64);
    fn ros_odometry_builder_set_pose_covariance(b: *mut ros_odometry_builder_t, cov: *const f64) -> i32;
    fn ros_odometry_builder_set_twist(b: *mut ros_odometry_builder_t, lx: f64, ly: f64, lz: f64, ax: f64, ay: f64, az: f64);
    fn ros_odometry_builder_set_twist_covariance(b: *mut ros_odometry_builder_t, cov: *const f64) -> i32;
    fn ros_odometry_builder_encode_into(b: *mut ros_odometry_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;
    fn ros_odometry_builder_build(b: *mut ros_odometry_builder_t, out: *mut *mut u8, out_len: *mut usize) -> i32;

    fn ros_mavros_altitude_builder_new() -> *mut ros_mavros_altitude_builder_t;
    fn ros_mavros_altitude_builder_free(b: *mut ros_mavros_altitude_builder_t);
    fn ros_mavros_altitude_builder_set_stamp(b: *mut ros_mavros_altitude_builder_t, sec: i32, nsec: u32);
    fn ros_mavros_altitude_builder_set_frame_id(b: *mut ros_mavros_altitude_builder_t, s: *const c_char) -> i32;
    fn ros_mavros_altitude_builder_set_monotonic(b: *mut ros_mavros_altitude_builder_t, v: f32);
    fn ros_mavros_altitude_builder_set_amsl(b: *mut ros_mavros_altitude_builder_t, v: f32);
    fn ros_mavros_altitude_builder_set_local(b: *mut ros_mavros_altitude_builder_t, v: f32);
    fn ros_mavros_altitude_builder_set_relative(b: *mut ros_mavros_altitude_builder_t, v: f32);
    fn ros_mavros_altitude_builder_set_terrain(b: *mut ros_mavros_altitude_builder_t, v: f32);
    fn ros_mavros_altitude_builder_set_bottom_clearance(b: *mut ros_mavros_altitude_builder_t, v: f32);
    fn ros_mavros_altitude_builder_encode_into(b: *mut ros_mavros_altitude_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_mavros_vfrhud_builder_new() -> *mut ros_mavros_vfrhud_builder_t;
    fn ros_mavros_vfrhud_builder_free(b: *mut ros_mavros_vfrhud_builder_t);
    fn ros_mavros_vfrhud_builder_set_stamp(b: *mut ros_mavros_vfrhud_builder_t, sec: i32, nsec: u32);
    fn ros_mavros_vfrhud_builder_set_frame_id(b: *mut ros_mavros_vfrhud_builder_t, s: *const c_char) -> i32;
    fn ros_mavros_vfrhud_builder_set_airspeed(b: *mut ros_mavros_vfrhud_builder_t, v: f32);
    fn ros_mavros_vfrhud_builder_set_groundspeed(b: *mut ros_mavros_vfrhud_builder_t, v: f32);
    fn ros_mavros_vfrhud_builder_set_heading(b: *mut ros_mavros_vfrhud_builder_t, v: i16);
    fn ros_mavros_vfrhud_builder_set_throttle(b: *mut ros_mavros_vfrhud_builder_t, v: f32);
    fn ros_mavros_vfrhud_builder_set_altitude(b: *mut ros_mavros_vfrhud_builder_t, v: f32);
    fn ros_mavros_vfrhud_builder_set_climb(b: *mut ros_mavros_vfrhud_builder_t, v: f32);
    fn ros_mavros_vfrhud_builder_encode_into(b: *mut ros_mavros_vfrhud_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_mavros_estimator_status_builder_new() -> *mut ros_mavros_estimator_status_builder_t;
    fn ros_mavros_estimator_status_builder_free(b: *mut ros_mavros_estimator_status_builder_t);
    fn ros_mavros_estimator_status_builder_set_stamp(b: *mut ros_mavros_estimator_status_builder_t, sec: i32, nsec: u32);
    fn ros_mavros_estimator_status_builder_set_frame_id(b: *mut ros_mavros_estimator_status_builder_t, s: *const c_char) -> i32;
    fn ros_mavros_estimator_status_builder_set_attitude_status_flag(b: *mut ros_mavros_estimator_status_builder_t, v: bool);
    fn ros_mavros_estimator_status_builder_set_velocity_horiz_status_flag(b: *mut ros_mavros_estimator_status_builder_t, v: bool);
    fn ros_mavros_estimator_status_builder_set_velocity_vert_status_flag(b: *mut ros_mavros_estimator_status_builder_t, v: bool);
    fn ros_mavros_estimator_status_builder_set_pos_horiz_rel_status_flag(b: *mut ros_mavros_estimator_status_builder_t, v: bool);
    fn ros_mavros_estimator_status_builder_set_pos_horiz_abs_status_flag(b: *mut ros_mavros_estimator_status_builder_t, v: bool);
    fn ros_mavros_estimator_status_builder_set_pos_vert_abs_status_flag(b: *mut ros_mavros_estimator_status_builder_t, v: bool);
    fn ros_mavros_estimator_status_builder_set_pos_vert_agl_status_flag(b: *mut ros_mavros_estimator_status_builder_t, v: bool);
    fn ros_mavros_estimator_status_builder_set_const_pos_mode_status_flag(b: *mut ros_mavros_estimator_status_builder_t, v: bool) -> i32;
    fn ros_mavros_estimator_status_builder_set_pred_pos_horiz_rel_status_flag(b: *mut ros_mavros_estimator_status_builder_t, v: bool);
    fn ros_mavros_estimator_status_builder_set_pred_pos_horiz_abs_status_flag(b: *mut ros_mavros_estimator_status_builder_t, v: bool);
    fn ros_mavros_estimator_status_builder_set_gps_glitch_status_flag(b: *mut ros_mavros_estimator_status_builder_t, v: bool);
    fn ros_mavros_estimator_status_builder_set_accel_error_status_flag(b: *mut ros_mavros_estimator_status_builder_t, v: bool);
    fn ros_mavros_estimator_status_builder_encode_into(b: *mut ros_mavros_estimator_status_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_mavros_extended_state_builder_new() -> *mut ros_mavros_extended_state_builder_t;
    fn ros_mavros_extended_state_builder_free(b: *mut ros_mavros_extended_state_builder_t);
    fn ros_mavros_extended_state_builder_set_stamp(b: *mut ros_mavros_extended_state_builder_t, sec: i32, nsec: u32);
    fn ros_mavros_extended_state_builder_set_frame_id(b: *mut ros_mavros_extended_state_builder_t, s: *const c_char) -> i32;
    fn ros_mavros_extended_state_builder_set_vtol_state(b: *mut ros_mavros_extended_state_builder_t, v: u8);
    fn ros_mavros_extended_state_builder_set_landed_state(b: *mut ros_mavros_extended_state_builder_t, v: u8);
    fn ros_mavros_extended_state_builder_encode_into(b: *mut ros_mavros_extended_state_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_mavros_sys_status_builder_new() -> *mut ros_mavros_sys_status_builder_t;
    fn ros_mavros_sys_status_builder_free(b: *mut ros_mavros_sys_status_builder_t);
    fn ros_mavros_sys_status_builder_set_stamp(b: *mut ros_mavros_sys_status_builder_t, sec: i32, nsec: u32);
    fn ros_mavros_sys_status_builder_set_frame_id(b: *mut ros_mavros_sys_status_builder_t, s: *const c_char) -> i32;
    fn ros_mavros_sys_status_builder_set_sensors_present(b: *mut ros_mavros_sys_status_builder_t, v: u32);
    fn ros_mavros_sys_status_builder_set_sensors_enabled(b: *mut ros_mavros_sys_status_builder_t, v: u32);
    fn ros_mavros_sys_status_builder_set_sensors_health(b: *mut ros_mavros_sys_status_builder_t, v: u32);
    fn ros_mavros_sys_status_builder_set_load(b: *mut ros_mavros_sys_status_builder_t, v: u16);
    fn ros_mavros_sys_status_builder_set_voltage_battery(b: *mut ros_mavros_sys_status_builder_t, v: u16);
    fn ros_mavros_sys_status_builder_set_current_battery(b: *mut ros_mavros_sys_status_builder_t, v: i16);
    fn ros_mavros_sys_status_builder_set_battery_remaining(b: *mut ros_mavros_sys_status_builder_t, v: i8);
    fn ros_mavros_sys_status_builder_set_drop_rate_comm(b: *mut ros_mavros_sys_status_builder_t, v: u16);
    fn ros_mavros_sys_status_builder_set_errors_comm(b: *mut ros_mavros_sys_status_builder_t, v: u16);
    fn ros_mavros_sys_status_builder_set_errors_count1(b: *mut ros_mavros_sys_status_builder_t, v: u16);
    fn ros_mavros_sys_status_builder_set_errors_count2(b: *mut ros_mavros_sys_status_builder_t, v: u16);
    fn ros_mavros_sys_status_builder_set_errors_count3(b: *mut ros_mavros_sys_status_builder_t, v: u16);
    fn ros_mavros_sys_status_builder_set_errors_count4(b: *mut ros_mavros_sys_status_builder_t, v: u16);
    fn ros_mavros_sys_status_builder_encode_into(b: *mut ros_mavros_sys_status_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_mavros_state_builder_new() -> *mut ros_mavros_state_builder_t;
    fn ros_mavros_state_builder_free(b: *mut ros_mavros_state_builder_t);
    fn ros_mavros_state_builder_set_stamp(b: *mut ros_mavros_state_builder_t, sec: i32, nsec: u32);
    fn ros_mavros_state_builder_set_frame_id(b: *mut ros_mavros_state_builder_t, s: *const c_char) -> i32;
    fn ros_mavros_state_builder_set_connected(b: *mut ros_mavros_state_builder_t, v: bool);
    fn ros_mavros_state_builder_set_armed(b: *mut ros_mavros_state_builder_t, v: bool);
    fn ros_mavros_state_builder_set_guided(b: *mut ros_mavros_state_builder_t, v: bool);
    fn ros_mavros_state_builder_set_manual_input(b: *mut ros_mavros_state_builder_t, v: bool);
    fn ros_mavros_state_builder_set_mode(b: *mut ros_mavros_state_builder_t, s: *const c_char) -> i32;
    fn ros_mavros_state_builder_set_system_status(b: *mut ros_mavros_state_builder_t, v: u8);
    fn ros_mavros_state_builder_encode_into(b: *mut ros_mavros_state_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_mavros_status_text_builder_new() -> *mut ros_mavros_status_text_builder_t;
    fn ros_mavros_status_text_builder_free(b: *mut ros_mavros_status_text_builder_t);
    fn ros_mavros_status_text_builder_set_stamp(b: *mut ros_mavros_status_text_builder_t, sec: i32, nsec: u32);
    fn ros_mavros_status_text_builder_set_frame_id(b: *mut ros_mavros_status_text_builder_t, s: *const c_char) -> i32;
    fn ros_mavros_status_text_builder_set_severity(b: *mut ros_mavros_status_text_builder_t, v: u8);
    fn ros_mavros_status_text_builder_set_text(b: *mut ros_mavros_status_text_builder_t, s: *const c_char) -> i32;
    fn ros_mavros_status_text_builder_encode_into(b: *mut ros_mavros_status_text_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_mavros_gps_raw_builder_new() -> *mut ros_mavros_gps_raw_builder_t;
    fn ros_mavros_gps_raw_builder_free(b: *mut ros_mavros_gps_raw_builder_t);
    fn ros_mavros_gps_raw_builder_set_stamp(b: *mut ros_mavros_gps_raw_builder_t, sec: i32, nsec: u32);
    fn ros_mavros_gps_raw_builder_set_frame_id(b: *mut ros_mavros_gps_raw_builder_t, s: *const c_char) -> i32;
    fn ros_mavros_gps_raw_builder_set_fix_type(b: *mut ros_mavros_gps_raw_builder_t, v: u8);
    fn ros_mavros_gps_raw_builder_set_lat(b: *mut ros_mavros_gps_raw_builder_t, v: i32);
    fn ros_mavros_gps_raw_builder_set_lon(b: *mut ros_mavros_gps_raw_builder_t, v: i32);
    fn ros_mavros_gps_raw_builder_set_alt(b: *mut ros_mavros_gps_raw_builder_t, v: i32);
    fn ros_mavros_gps_raw_builder_set_eph(b: *mut ros_mavros_gps_raw_builder_t, v: u16);
    fn ros_mavros_gps_raw_builder_set_epv(b: *mut ros_mavros_gps_raw_builder_t, v: u16);
    fn ros_mavros_gps_raw_builder_set_vel(b: *mut ros_mavros_gps_raw_builder_t, v: u16);
    fn ros_mavros_gps_raw_builder_set_cog(b: *mut ros_mavros_gps_raw_builder_t, v: u16);
    fn ros_mavros_gps_raw_builder_set_satellites_visible(b: *mut ros_mavros_gps_raw_builder_t, v: u8);
    fn ros_mavros_gps_raw_builder_set_alt_ellipsoid(b: *mut ros_mavros_gps_raw_builder_t, v: i32);
    fn ros_mavros_gps_raw_builder_set_h_acc(b: *mut ros_mavros_gps_raw_builder_t, v: u32);
    fn ros_mavros_gps_raw_builder_set_v_acc(b: *mut ros_mavros_gps_raw_builder_t, v: u32);
    fn ros_mavros_gps_raw_builder_set_vel_acc(b: *mut ros_mavros_gps_raw_builder_t, v: u32);
    fn ros_mavros_gps_raw_builder_set_hdg_acc(b: *mut ros_mavros_gps_raw_builder_t, v: i32);
    fn ros_mavros_gps_raw_builder_set_yaw(b: *mut ros_mavros_gps_raw_builder_t, v: u16);
    fn ros_mavros_gps_raw_builder_set_dgps_numch(b: *mut ros_mavros_gps_raw_builder_t, v: u8);
    fn ros_mavros_gps_raw_builder_set_dgps_age(b: *mut ros_mavros_gps_raw_builder_t, v: u32);
    fn ros_mavros_gps_raw_builder_encode_into(b: *mut ros_mavros_gps_raw_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_mavros_timesync_status_builder_new() -> *mut ros_mavros_timesync_status_builder_t;
    fn ros_mavros_timesync_status_builder_free(b: *mut ros_mavros_timesync_status_builder_t);
    fn ros_mavros_timesync_status_builder_set_stamp(b: *mut ros_mavros_timesync_status_builder_t, sec: i32, nsec: u32);
    fn ros_mavros_timesync_status_builder_set_frame_id(b: *mut ros_mavros_timesync_status_builder_t, s: *const c_char) -> i32;
    fn ros_mavros_timesync_status_builder_set_remote_timestamp_ns(b: *mut ros_mavros_timesync_status_builder_t, v: u64);
    fn ros_mavros_timesync_status_builder_set_observed_offset_ns(b: *mut ros_mavros_timesync_status_builder_t, v: i64);
    fn ros_mavros_timesync_status_builder_set_estimated_offset_ns(b: *mut ros_mavros_timesync_status_builder_t, v: i64);
    fn ros_mavros_timesync_status_builder_set_round_trip_time_ms(b: *mut ros_mavros_timesync_status_builder_t, v: f32);
    fn ros_mavros_timesync_status_builder_encode_into(b: *mut ros_mavros_timesync_status_builder_t, buf: *mut u8, cap: usize, out_len: *mut usize) -> i32;

    fn ros_bytes_free(ptr: *mut u8, len: usize);
}
'''

TESTS = r'''
#[test]
fn ros_point_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_point_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_point_stamped_builder_set_stamp, ros_point_stamped_builder_set_frame_id);
        ros_point_stamped_builder_set_point(b, 1.5, -2.5, 3.0);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_point_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::PointStamped::builder()
            .stamp(stamp()).frame_id("test_frame")
            .point(Point { x: 1.5, y: -2.5, z: 3.0 }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_point_stamped_builder_free(b);
    }
}

#[test]
fn ros_vector3_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_vector3_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_vector3_stamped_builder_set_stamp, ros_vector3_stamped_builder_set_frame_id);
        ros_vector3_stamped_builder_set_vector(b, 1.0, 2.0, 3.0);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_vector3_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::Vector3Stamped::builder()
            .stamp(stamp()).frame_id("test_frame")
            .vector(Vector3 { x: 1.0, y: 2.0, z: 3.0 }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_vector3_stamped_builder_free(b);
    }
}

#[test]
fn ros_pose_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_pose_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_pose_stamped_builder_set_stamp, ros_pose_stamped_builder_set_frame_id);
        ros_pose_stamped_builder_set_position(b, 1.5, -2.5, 3.0);
        ros_pose_stamped_builder_set_orientation(b, 0.0, 0.0, 0.0, 1.0);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_pose_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::PoseStamped::builder()
            .stamp(stamp()).frame_id("test_frame")
            .pose(Pose {
                position: Point { x: 1.5, y: -2.5, z: 3.0 },
                orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_pose_stamped_builder_free(b);
    }
}

#[test]
fn ros_quaternion_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_quaternion_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_quaternion_stamped_builder_set_stamp, ros_quaternion_stamped_builder_set_frame_id);
        ros_quaternion_stamped_builder_set_quaternion(b, 0.0, 0.0, 0.0, 1.0);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_quaternion_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::QuaternionStamped::builder()
            .stamp(stamp()).frame_id("test_frame")
            .quaternion(Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_quaternion_stamped_builder_free(b);
    }
}

#[test]
fn ros_accel_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_accel_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_accel_stamped_builder_set_stamp, ros_accel_stamped_builder_set_frame_id);
        ros_accel_stamped_builder_set_linear_acceleration(b, 1.0, 2.0, 3.0);
        ros_accel_stamped_builder_set_angular_acceleration(b, 0.1, 0.2, 0.3);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_accel_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::AccelStamped::builder()
            .stamp(stamp()).frame_id("test_frame")
            .accel(Accel {
                linear: Vector3 { x: 1.0, y: 2.0, z: 3.0 },
                angular: Vector3 { x: 0.1, y: 0.2, z: 0.3 },
            }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_accel_stamped_builder_free(b);
    }
}

#[test]
fn ros_twist_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_twist_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_twist_stamped_builder_set_stamp, ros_twist_stamped_builder_set_frame_id);
        ros_twist_stamped_builder_set_linear(b, 1.0, 2.0, 3.0);
        ros_twist_stamped_builder_set_angular(b, 0.1, 0.2, 0.3);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_twist_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::TwistStamped::builder()
            .stamp(stamp()).frame_id("test_frame")
            .twist(Twist {
                linear: Vector3 { x: 1.0, y: 2.0, z: 3.0 },
                angular: Vector3 { x: 0.1, y: 0.2, z: 0.3 },
            }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_twist_stamped_builder_free(b);
    }
}

#[test]
fn ros_wrench_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_wrench_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_wrench_stamped_builder_set_stamp, ros_wrench_stamped_builder_set_frame_id);
        ros_wrench_stamped_builder_set_force(b, 1.0, 2.0, 3.0);
        ros_wrench_stamped_builder_set_torque(b, 0.1, 0.2, 0.3);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_wrench_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::WrenchStamped::builder()
            .stamp(stamp()).frame_id("test_frame")
            .wrench(Wrench {
                force: Vector3 { x: 1.0, y: 2.0, z: 3.0 },
                torque: Vector3 { x: 0.1, y: 0.2, z: 0.3 },
            }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_wrench_stamped_builder_free(b);
    }
}

#[test]
fn ros_inertia_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_inertia_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_inertia_stamped_builder_set_stamp, ros_inertia_stamped_builder_set_frame_id);
        ros_inertia_stamped_builder_set_mass(b, 10.0);
        ros_inertia_stamped_builder_set_com(b, 0.1, 0.2, 0.3);
        ros_inertia_stamped_builder_set_inertia_tensor(b, 1.0, 0.1, 0.2, 2.0, 0.3, 3.0);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_inertia_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::InertiaStamped::builder()
            .stamp(stamp()).frame_id("test_frame")
            .inertia(Inertia {
                m: 10.0,
                com: Vector3 { x: 0.1, y: 0.2, z: 0.3 },
                ixx: 1.0, ixy: 0.1, ixz: 0.2, iyy: 2.0, iyz: 0.3, izz: 3.0,
            }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_inertia_stamped_builder_free(b);
    }
}

#[test]
fn ros_transform_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_transform_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_transform_stamped_builder_set_stamp, ros_transform_stamped_builder_set_frame_id);
        let child = CString::new("child_frame").unwrap();
        assert_eq!(ros_transform_stamped_builder_set_child_frame_id(b, child.as_ptr()), 0);
        ros_transform_stamped_builder_set_translation(b, 1.5, -2.5, 3.0);
        ros_transform_stamped_builder_set_rotation(b, 0.0, 0.0, 0.0, 1.0);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_transform_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::TransformStamped::builder()
            .stamp(stamp()).frame_id("test_frame").child_frame_id("child_frame")
            .transform(Transform {
                translation: Vector3 { x: 1.5, y: -2.5, z: 3.0 },
                rotation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_transform_stamped_builder_free(b);
    }
}

#[test]
fn ros_pose_with_covariance_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_pose_with_covariance_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_pose_with_covariance_stamped_builder_set_stamp, ros_pose_with_covariance_stamped_builder_set_frame_id);
        ros_pose_with_covariance_stamped_builder_set_position(b, 1.5, -2.5, 3.0);
        ros_pose_with_covariance_stamped_builder_set_orientation(b, 0.0, 0.0, 0.0, 1.0);
        let cov = pose_cov();
        assert_eq!(ros_pose_with_covariance_stamped_builder_set_covariance(b, cov.as_ptr()), 0);
        let mut buf = vec![0u8; 1024];
        let mut out_len = 0usize;
        assert_eq!(ros_pose_with_covariance_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::PoseWithCovarianceStamped::builder()
            .stamp(stamp()).frame_id("test_frame")
            .pose_with_covariance(PoseWithCovariance {
                pose: Pose {
                    position: Point { x: 1.5, y: -2.5, z: 3.0 },
                    orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                },
                covariance: pose_cov(),
            }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_pose_with_covariance_stamped_builder_free(b);
    }
}

#[test]
fn ros_twist_with_covariance_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_twist_with_covariance_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_twist_with_covariance_stamped_builder_set_stamp, ros_twist_with_covariance_stamped_builder_set_frame_id);
        ros_twist_with_covariance_stamped_builder_set_linear(b, 1.0, 2.0, 3.0);
        ros_twist_with_covariance_stamped_builder_set_angular(b, 0.1, 0.2, 0.3);
        let cov = twist_cov();
        assert_eq!(ros_twist_with_covariance_stamped_builder_set_covariance(b, cov.as_ptr()), 0);
        let mut buf = vec![0u8; 1024];
        let mut out_len = 0usize;
        assert_eq!(ros_twist_with_covariance_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::TwistWithCovarianceStamped::builder()
            .stamp(stamp()).frame_id("test_frame")
            .twist_with_covariance(TwistWithCovariance {
                twist: Twist {
                    linear: Vector3 { x: 1.0, y: 2.0, z: 3.0 },
                    angular: Vector3 { x: 0.1, y: 0.2, z: 0.3 },
                },
                covariance: twist_cov(),
            }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_twist_with_covariance_stamped_builder_free(b);
    }
}

#[test]
fn ros_accel_with_covariance_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_accel_with_covariance_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_accel_with_covariance_stamped_builder_set_stamp, ros_accel_with_covariance_stamped_builder_set_frame_id);
        ros_accel_with_covariance_stamped_builder_set_linear_acceleration(b, 1.0, 2.0, 3.0);
        ros_accel_with_covariance_stamped_builder_set_angular_acceleration(b, 0.1, 0.2, 0.3);
        let cov = accel_cov();
        assert_eq!(ros_accel_with_covariance_stamped_builder_set_covariance(b, cov.as_ptr()), 0);
        let mut buf = vec![0u8; 1024];
        let mut out_len = 0usize;
        assert_eq!(ros_accel_with_covariance_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::AccelWithCovarianceStamped::builder()
            .stamp(stamp()).frame_id("test_frame")
            .accel_with_covariance(AccelWithCovariance {
                accel: Accel {
                    linear: Vector3 { x: 1.0, y: 2.0, z: 3.0 },
                    angular: Vector3 { x: 0.1, y: 0.2, z: 0.3 },
                },
                covariance: accel_cov(),
            }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_accel_with_covariance_stamped_builder_free(b);
    }
}

#[test]
fn ros_polygon_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_polygon_builder_new();
        assert!(!b.is_null());
        let xyz: [f32; 9] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        assert_eq!(ros_polygon_builder_set_points(b, xyz.as_ptr(), 3), 0);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_polygon_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::Polygon::builder()
            .points(&[
                Point32 { x: 1.0, y: 2.0, z: 3.0 },
                Point32 { x: 4.0, y: 5.0, z: 6.0 },
                Point32 { x: 7.0, y: 8.0, z: 9.0 },
            ]).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_polygon_builder_free(b);
    }
}

#[test]
fn ros_polygon_stamped_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_polygon_stamped_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_polygon_stamped_builder_set_stamp, ros_polygon_stamped_builder_set_frame_id);
        let xyz: [f32; 9] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        assert_eq!(ros_polygon_stamped_builder_set_points(b, xyz.as_ptr(), 3), 0);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_polygon_stamped_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::PolygonStamped::builder()
            .stamp(stamp()).frame_id("test_frame")
            .points(&[
                Point32 { x: 1.0, y: 2.0, z: 3.0 },
                Point32 { x: 4.0, y: 5.0, z: 6.0 },
                Point32 { x: 7.0, y: 8.0, z: 9.0 },
            ]).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_polygon_stamped_builder_free(b);
    }
}

#[test]
fn ros_pose_array_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_pose_array_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_pose_array_builder_set_stamp, ros_pose_array_builder_set_frame_id);
        let poses: [f64; 14] = [
            1.5, -2.5, 3.0, 0.0, 0.0, 0.0, 1.0,
            10.0, 20.0, 30.0, 0.0, 0.0, 0.0, 1.0,
        ];
        assert_eq!(ros_pose_array_builder_set_poses(b, poses.as_ptr(), 2), 0);
        let mut buf = vec![0u8; 1024];
        let mut out_len = 0usize;
        assert_eq!(ros_pose_array_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = geometry_msgs::PoseArray::builder()
            .stamp(stamp()).frame_id("test_frame")
            .poses(&[
                Pose {
                    position: Point { x: 1.5, y: -2.5, z: 3.0 },
                    orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                },
                Pose {
                    position: Point { x: 10.0, y: 20.0, z: 30.0 },
                    orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                },
            ]).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_pose_array_builder_free(b);
    }
}

#[test]
fn ros_odometry_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_odometry_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_odometry_builder_set_stamp, ros_odometry_builder_set_frame_id);
        let child = CString::new("base_link").unwrap();
        assert_eq!(ros_odometry_builder_set_child_frame_id(b, child.as_ptr()), 0);
        ros_odometry_builder_set_pose(b, 1.5, -2.5, 3.0, 0.0, 0.0, 0.0, 1.0);
        let pcov = pose_cov();
        let tcov = twist_cov();
        assert_eq!(ros_odometry_builder_set_pose_covariance(b, pcov.as_ptr()), 0);
        ros_odometry_builder_set_twist(b, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        assert_eq!(ros_odometry_builder_set_twist_covariance(b, tcov.as_ptr()), 0);
        let mut buf = vec![0u8; 2048];
        let mut out_len = 0usize;
        assert_eq!(ros_odometry_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = nav_msgs::Odometry::builder()
            .stamp(stamp()).frame_id("test_frame").child_frame_id("base_link")
            .pose(PoseWithCovariance {
                pose: Pose {
                    position: Point { x: 1.5, y: -2.5, z: 3.0 },
                    orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                },
                covariance: pose_cov(),
            })
            .twist(TwistWithCovariance {
                twist: Twist {
                    linear: Vector3 { x: 1.0, y: 2.0, z: 3.0 },
                    angular: Vector3 { x: 0.1, y: 0.2, z: 0.3 },
                },
                covariance: twist_cov(),
            }).build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_odometry_builder_free(b);
    }
}

#[test]
fn ros_odometry_builder_build_matches_rust_builder() {
    unsafe {
        let b = ros_odometry_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_odometry_builder_set_stamp, ros_odometry_builder_set_frame_id);
        let child = CString::new("base_link").unwrap();
        assert_eq!(ros_odometry_builder_set_child_frame_id(b, child.as_ptr()), 0);
        ros_odometry_builder_set_pose(b, 1.5, -2.5, 3.0, 0.0, 0.0, 0.0, 1.0);
        let pcov = pose_cov();
        let tcov = twist_cov();
        assert_eq!(ros_odometry_builder_set_pose_covariance(b, pcov.as_ptr()), 0);
        ros_odometry_builder_set_twist(b, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
        assert_eq!(ros_odometry_builder_set_twist_covariance(b, tcov.as_ptr()), 0);
        let mut out_ptr: *mut u8 = std::ptr::null_mut();
        let mut out_len = 0usize;
        assert_eq!(ros_odometry_builder_build(b, &mut out_ptr, &mut out_len), 0);
        assert!(!out_ptr.is_null());
        let via_rust = nav_msgs::Odometry::builder()
            .stamp(stamp()).frame_id("test_frame").child_frame_id("base_link")
            .pose(PoseWithCovariance {
                pose: Pose {
                    position: Point { x: 1.5, y: -2.5, z: 3.0 },
                    orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
                },
                covariance: pose_cov(),
            })
            .twist(TwistWithCovariance {
                twist: Twist {
                    linear: Vector3 { x: 1.0, y: 2.0, z: 3.0 },
                    angular: Vector3 { x: 0.1, y: 0.2, z: 0.3 },
                },
                covariance: twist_cov(),
            }).build().unwrap();
        assert_eq!(std::slice::from_raw_parts(out_ptr, out_len), via_rust.as_cdr());
        ros_bytes_free(out_ptr, out_len);
        ros_odometry_builder_free(b);
    }
}

#[test]
fn ros_mavros_altitude_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_mavros_altitude_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_mavros_altitude_builder_set_stamp, ros_mavros_altitude_builder_set_frame_id);
        ros_mavros_altitude_builder_set_monotonic(b, 100.0);
        ros_mavros_altitude_builder_set_amsl(b, 50.0);
        ros_mavros_altitude_builder_set_local(b, 10.0);
        ros_mavros_altitude_builder_set_relative(b, 5.0);
        ros_mavros_altitude_builder_set_terrain(b, 2.0);
        ros_mavros_altitude_builder_set_bottom_clearance(b, 1.5);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_mavros_altitude_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = mavros_msgs::Altitude::builder()
            .stamp(stamp()).frame_id("test_frame")
            .monotonic(100.0).amsl(50.0).local(10.0).relative(5.0).terrain(2.0).bottom_clearance(1.5)
            .build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_mavros_altitude_builder_free(b);
    }
}

#[test]
fn ros_mavros_vfrhud_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_mavros_vfrhud_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_mavros_vfrhud_builder_set_stamp, ros_mavros_vfrhud_builder_set_frame_id);
        ros_mavros_vfrhud_builder_set_airspeed(b, 12.5);
        ros_mavros_vfrhud_builder_set_groundspeed(b, 11.0);
        ros_mavros_vfrhud_builder_set_heading(b, 90);
        ros_mavros_vfrhud_builder_set_throttle(b, 0.75);
        ros_mavros_vfrhud_builder_set_altitude(b, 120.0);
        ros_mavros_vfrhud_builder_set_climb(b, 0.5);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_mavros_vfrhud_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = mavros_msgs::VfrHud::builder()
            .stamp(stamp()).frame_id("test_frame")
            .airspeed(12.5).groundspeed(11.0).heading(90).throttle(0.75).altitude(120.0).climb(0.5)
            .build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_mavros_vfrhud_builder_free(b);
    }
}

#[test]
fn ros_mavros_estimator_status_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_mavros_estimator_status_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_mavros_estimator_status_builder_set_stamp, ros_mavros_estimator_status_builder_set_frame_id);
        ros_mavros_estimator_status_builder_set_attitude_status_flag(b, true);
        ros_mavros_estimator_status_builder_set_velocity_horiz_status_flag(b, true);
        ros_mavros_estimator_status_builder_set_velocity_vert_status_flag(b, true);
        ros_mavros_estimator_status_builder_set_pos_horiz_rel_status_flag(b, true);
        ros_mavros_estimator_status_builder_set_pos_horiz_abs_status_flag(b, false);
        ros_mavros_estimator_status_builder_set_pos_vert_abs_status_flag(b, true);
        ros_mavros_estimator_status_builder_set_pos_vert_agl_status_flag(b, false);
        assert_eq!(ros_mavros_estimator_status_builder_set_const_pos_mode_status_flag(b, false), 0);
        ros_mavros_estimator_status_builder_set_pred_pos_horiz_rel_status_flag(b, true);
        ros_mavros_estimator_status_builder_set_pred_pos_horiz_abs_status_flag(b, false);
        ros_mavros_estimator_status_builder_set_gps_glitch_status_flag(b, false);
        ros_mavros_estimator_status_builder_set_accel_error_status_flag(b, false);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_mavros_estimator_status_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = mavros_msgs::EstimatorStatus::builder()
            .stamp(stamp()).frame_id("test_frame")
            .attitude_status_flag(true).velocity_horiz_status_flag(true).velocity_vert_status_flag(true)
            .pos_horiz_rel_status_flag(true).pos_horiz_abs_status_flag(false).pos_vert_abs_status_flag(true)
            .pos_vert_agl_status_flag(false).const_pos_mode_status_flag(false)
            .pred_pos_horiz_rel_status_flag(true).pred_pos_horiz_abs_status_flag(false)
            .gps_glitch_status_flag(false).accel_error_status_flag(false)
            .build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_mavros_estimator_status_builder_free(b);
    }
}

#[test]
fn ros_mavros_extended_state_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_mavros_extended_state_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_mavros_extended_state_builder_set_stamp, ros_mavros_extended_state_builder_set_frame_id);
        ros_mavros_extended_state_builder_set_vtol_state(b, 3);
        ros_mavros_extended_state_builder_set_landed_state(b, 2);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_mavros_extended_state_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = mavros_msgs::ExtendedState::builder()
            .stamp(stamp()).frame_id("test_frame").vtol_state(3).landed_state(2)
            .build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_mavros_extended_state_builder_free(b);
    }
}

#[test]
fn ros_mavros_sys_status_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_mavros_sys_status_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_mavros_sys_status_builder_set_stamp, ros_mavros_sys_status_builder_set_frame_id);
        ros_mavros_sys_status_builder_set_sensors_present(b, 0xFFFF);
        ros_mavros_sys_status_builder_set_sensors_enabled(b, 0x00FF);
        ros_mavros_sys_status_builder_set_sensors_health(b, 0x00FE);
        ros_mavros_sys_status_builder_set_load(b, 500);
        ros_mavros_sys_status_builder_set_voltage_battery(b, 12600);
        ros_mavros_sys_status_builder_set_current_battery(b, -150);
        ros_mavros_sys_status_builder_set_battery_remaining(b, 85);
        ros_mavros_sys_status_builder_set_drop_rate_comm(b, 12);
        ros_mavros_sys_status_builder_set_errors_comm(b, 3);
        ros_mavros_sys_status_builder_set_errors_count1(b, 1);
        ros_mavros_sys_status_builder_set_errors_count2(b, 2);
        ros_mavros_sys_status_builder_set_errors_count3(b, 3);
        ros_mavros_sys_status_builder_set_errors_count4(b, 4);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_mavros_sys_status_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = mavros_msgs::SysStatus::builder()
            .stamp(stamp()).frame_id("test_frame")
            .sensors_present(0xFFFF).sensors_enabled(0x00FF).sensors_health(0x00FE)
            .load(500).voltage_battery(12600).current_battery(-150).battery_remaining(85)
            .drop_rate_comm(12).errors_comm(3).errors_count1(1).errors_count2(2)
            .errors_count3(3).errors_count4(4)
            .build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_mavros_sys_status_builder_free(b);
    }
}

#[test]
fn ros_mavros_state_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_mavros_state_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_mavros_state_builder_set_stamp, ros_mavros_state_builder_set_frame_id);
        ros_mavros_state_builder_set_connected(b, true);
        ros_mavros_state_builder_set_armed(b, true);
        ros_mavros_state_builder_set_guided(b, true);
        ros_mavros_state_builder_set_manual_input(b, false);
        let mode = CString::new("OFFBOARD").unwrap();
        assert_eq!(ros_mavros_state_builder_set_mode(b, mode.as_ptr()), 0);
        ros_mavros_state_builder_set_system_status(b, 4);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_mavros_state_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = mavros_msgs::State::builder()
            .stamp(stamp()).frame_id("test_frame")
            .connected(true).armed(true).guided(true).manual_input(false)
            .mode("OFFBOARD").system_status(4)
            .build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_mavros_state_builder_free(b);
    }
}

#[test]
fn ros_mavros_status_text_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_mavros_status_text_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_mavros_status_text_builder_set_stamp, ros_mavros_status_text_builder_set_frame_id);
        ros_mavros_status_text_builder_set_severity(b, 4);
        let text = CString::new("prearm: compass not calibrated").unwrap();
        assert_eq!(ros_mavros_status_text_builder_set_text(b, text.as_ptr()), 0);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_mavros_status_text_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = mavros_msgs::StatusText::builder()
            .stamp(stamp()).frame_id("test_frame").severity(4)
            .text("prearm: compass not calibrated")
            .build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_mavros_status_text_builder_free(b);
    }
}

#[test]
fn ros_mavros_gps_raw_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_mavros_gps_raw_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_mavros_gps_raw_builder_set_stamp, ros_mavros_gps_raw_builder_set_frame_id);
        ros_mavros_gps_raw_builder_set_fix_type(b, 3);
        ros_mavros_gps_raw_builder_set_lat(b, 473762200);
        ros_mavros_gps_raw_builder_set_lon(b, 85453900);
        ros_mavros_gps_raw_builder_set_alt(b, 488000);
        ros_mavros_gps_raw_builder_set_eph(b, 120);
        ros_mavros_gps_raw_builder_set_epv(b, 180);
        ros_mavros_gps_raw_builder_set_vel(b, 250);
        ros_mavros_gps_raw_builder_set_cog(b, 9000);
        ros_mavros_gps_raw_builder_set_satellites_visible(b, 12);
        ros_mavros_gps_raw_builder_set_alt_ellipsoid(b, 500000);
        ros_mavros_gps_raw_builder_set_h_acc(b, 150);
        ros_mavros_gps_raw_builder_set_v_acc(b, 200);
        ros_mavros_gps_raw_builder_set_vel_acc(b, 50);
        ros_mavros_gps_raw_builder_set_hdg_acc(b, 100);
        ros_mavros_gps_raw_builder_set_yaw(b, 9000);
        ros_mavros_gps_raw_builder_set_dgps_numch(b, 8);
        ros_mavros_gps_raw_builder_set_dgps_age(b, 120);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_mavros_gps_raw_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = mavros_msgs::GpsRaw::builder()
            .stamp(stamp()).frame_id("test_frame")
            .fix_type(3).lat(473762200).lon(85453900).alt(488000)
            .eph(120).epv(180).vel(250).cog(9000).satellites_visible(12)
            .alt_ellipsoid(500000).h_acc(150).v_acc(200).vel_acc(50).hdg_acc(100)
            .yaw(9000).dgps_numch(8).dgps_age(120)
            .build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_mavros_gps_raw_builder_free(b);
    }
}

#[test]
fn ros_mavros_timesync_status_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_mavros_timesync_status_builder_new();
        assert!(!b.is_null());
        stamped_setup!(b, ros_mavros_timesync_status_builder_set_stamp, ros_mavros_timesync_status_builder_set_frame_id);
        ros_mavros_timesync_status_builder_set_remote_timestamp_ns(b, 1_234_567_890_123);
        ros_mavros_timesync_status_builder_set_observed_offset_ns(b, -1500);
        ros_mavros_timesync_status_builder_set_estimated_offset_ns(b, -1200);
        ros_mavros_timesync_status_builder_set_round_trip_time_ms(b, 2.5);
        let mut buf = vec![0u8; 512];
        let mut out_len = 0usize;
        assert_eq!(ros_mavros_timesync_status_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len), 0);
        let via_rust = mavros_msgs::TimesyncStatus::builder()
            .stamp(stamp()).frame_id("test_frame")
            .remote_timestamp_ns(1_234_567_890_123).observed_offset_ns(-1500)
            .estimated_offset_ns(-1200).round_trip_time_ms(2.5)
            .build().unwrap();
        assert_eq!(&buf[..out_len], via_rust.as_cdr());
        ros_mavros_timesync_status_builder_free(b);
    }
}
'''

OUT.write_text(HEADER + OPAQUES + EXTERN + TESTS)
print(f"Wrote {OUT} ({OUT.stat().st_size} bytes)")
