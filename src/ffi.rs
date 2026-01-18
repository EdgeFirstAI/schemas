// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! C FFI (Foreign Function Interface) module
//!
//! This module provides C-compatible bindings for all schema types with
//! CDR serialization support.

#![allow(non_camel_case_types)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]

use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::ptr;
use std::slice;

use crate::serde_cdr;
use crate::*;

// =============================================================================
// errno Support
// =============================================================================

/// errno constants matching POSIX
const EINVAL: i32 = libc::EINVAL;
const ENOMEM: i32 = libc::ENOMEM;
const EBADMSG: i32 = libc::EBADMSG;

/// Platform-specific errno setter
#[cfg(target_os = "macos")]
fn set_errno(code: i32) {
    unsafe {
        *libc::__error() = code;
    }
}

#[cfg(target_os = "linux")]
fn set_errno(code: i32) {
    unsafe {
        *libc::__errno_location() = code;
    }
}

#[cfg(target_os = "windows")]
fn set_errno(code: i32) {
    unsafe {
        *libc::_errno() = code;
    }
}

#[cfg(not(any(target_os = "macos", target_os = "linux", target_os = "windows")))]
fn set_errno(_code: i32) {
    // Fallback for unsupported platforms - no-op
    // errno may not be properly set on these platforms
}

/// Helper to convert Rust string to C string
fn string_to_c_char(s: &str) -> *mut c_char {
    match CString::new(s) {
        Ok(c_str) => c_str.into_raw(),
        Err(_) => {
            set_errno(ENOMEM);
            ptr::null_mut()
        }
    }
}

/// Helper to convert C string to Rust String
unsafe fn c_char_to_string(s: *const c_char) -> Option<String> {
    if s.is_null() {
        return None;
    }
    match CStr::from_ptr(s).to_str() {
        Ok(rust_str) => Some(rust_str.to_owned()),
        Err(_) => None,
    }
}

/// Helper macro for null pointer checks (returns -1 for int functions)
macro_rules! check_null {
    ($ptr:expr) => {
        if $ptr.is_null() {
            set_errno(EINVAL);
            return -1;
        }
    };
}

/// Helper macro for null pointer checks (returns NULL for pointer functions)
macro_rules! check_null_ret_null {
    ($ptr:expr) => {
        if $ptr.is_null() {
            set_errno(EINVAL);
            return ptr::null_mut();
        }
    };
}

// =============================================================================
// builtin_interfaces::Time
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_time_new() -> *mut builtin_interfaces::Time {
    let boxed = Box::new(builtin_interfaces::Time { sec: 0, nanosec: 0 });
    Box::into_raw(boxed)
}

#[no_mangle]
pub extern "C" fn ros_time_free(time: *mut builtin_interfaces::Time) {
    if !time.is_null() {
        unsafe {
            drop(Box::from_raw(time));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_time_get_sec(time: *const builtin_interfaces::Time) -> i32 {
    unsafe {
        assert!(!time.is_null());
        (*time).sec
    }
}

#[no_mangle]
pub extern "C" fn ros_time_get_nanosec(time: *const builtin_interfaces::Time) -> u32 {
    unsafe {
        assert!(!time.is_null());
        (*time).nanosec
    }
}

#[no_mangle]
pub extern "C" fn ros_time_set_sec(time: *mut builtin_interfaces::Time, sec: i32) {
    unsafe {
        assert!(!time.is_null());
        (*time).sec = sec;
    }
}

#[no_mangle]
pub extern "C" fn ros_time_set_nanosec(time: *mut builtin_interfaces::Time, nanosec: u32) {
    unsafe {
        assert!(!time.is_null());
        (*time).nanosec = nanosec;
    }
}

#[no_mangle]
pub extern "C" fn ros_time_serialize(
    time: *const builtin_interfaces::Time,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(time);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*time) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_time_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut builtin_interfaces::Time {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<builtin_interfaces::Time>(slice) {
            Ok(time) => Box::into_raw(Box::new(time)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// builtin_interfaces::Duration
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_duration_new() -> *mut builtin_interfaces::Duration {
    Box::into_raw(Box::new(builtin_interfaces::Duration {
        sec: 0,
        nanosec: 0,
    }))
}

#[no_mangle]
pub extern "C" fn ros_duration_free(duration: *mut builtin_interfaces::Duration) {
    if !duration.is_null() {
        unsafe {
            drop(Box::from_raw(duration));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_duration_get_sec(duration: *const builtin_interfaces::Duration) -> i32 {
    unsafe {
        assert!(!duration.is_null());
        (*duration).sec
    }
}

#[no_mangle]
pub extern "C" fn ros_duration_get_nanosec(duration: *const builtin_interfaces::Duration) -> u32 {
    unsafe {
        assert!(!duration.is_null());
        (*duration).nanosec
    }
}

#[no_mangle]
pub extern "C" fn ros_duration_set_sec(duration: *mut builtin_interfaces::Duration, sec: i32) {
    unsafe {
        assert!(!duration.is_null());
        (*duration).sec = sec;
    }
}

#[no_mangle]
pub extern "C" fn ros_duration_set_nanosec(
    duration: *mut builtin_interfaces::Duration,
    nanosec: u32,
) {
    unsafe {
        assert!(!duration.is_null());
        (*duration).nanosec = nanosec;
    }
}

#[no_mangle]
pub extern "C" fn ros_duration_serialize(
    duration: *const builtin_interfaces::Duration,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(duration);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*duration) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_duration_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut builtin_interfaces::Duration {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<builtin_interfaces::Duration>(slice) {
            Ok(duration) => Box::into_raw(Box::new(duration)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// std_msgs::Header
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_header_new() -> *mut std_msgs::Header {
    Box::into_raw(Box::new(std_msgs::Header {
        stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
        frame_id: String::new(),
    }))
}

#[no_mangle]
pub extern "C" fn ros_header_free(header: *mut std_msgs::Header) {
    if !header.is_null() {
        unsafe {
            drop(Box::from_raw(header));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_header_get_stamp(
    header: *const std_msgs::Header,
) -> *const builtin_interfaces::Time {
    unsafe {
        assert!(!header.is_null());
        &(*header).stamp
    }
}

#[no_mangle]
pub extern "C" fn ros_header_get_stamp_mut(
    header: *mut std_msgs::Header,
) -> *mut builtin_interfaces::Time {
    unsafe {
        assert!(!header.is_null());
        &mut (*header).stamp
    }
}

#[no_mangle]
pub extern "C" fn ros_header_get_frame_id(header: *const std_msgs::Header) -> *mut c_char {
    unsafe {
        assert!(!header.is_null());
        string_to_c_char(&(*header).frame_id)
    }
}

#[no_mangle]
pub extern "C" fn ros_header_set_frame_id(
    header: *mut std_msgs::Header,
    frame_id: *const c_char,
) -> i32 {
    check_null!(header);
    check_null!(frame_id);

    unsafe {
        match c_char_to_string(frame_id) {
            Some(s) => {
                (*header).frame_id = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_header_serialize(
    header: *const std_msgs::Header,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(header);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*header) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_header_deserialize(bytes: *const u8, len: usize) -> *mut std_msgs::Header {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<std_msgs::Header>(slice) {
            Ok(header) => Box::into_raw(Box::new(header)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// std_msgs::ColorRGBA
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_color_rgba_new() -> *mut std_msgs::ColorRGBA {
    Box::into_raw(Box::new(std_msgs::ColorRGBA {
        r: 0.0,
        g: 0.0,
        b: 0.0,
        a: 1.0,
    }))
}

#[no_mangle]
pub extern "C" fn ros_color_rgba_free(color: *mut std_msgs::ColorRGBA) {
    if !color.is_null() {
        unsafe {
            drop(Box::from_raw(color));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_color_rgba_get_r(color: *const std_msgs::ColorRGBA) -> f32 {
    unsafe {
        assert!(!color.is_null());
        (*color).r
    }
}

#[no_mangle]
pub extern "C" fn ros_color_rgba_get_g(color: *const std_msgs::ColorRGBA) -> f32 {
    unsafe {
        assert!(!color.is_null());
        (*color).g
    }
}

#[no_mangle]
pub extern "C" fn ros_color_rgba_get_b(color: *const std_msgs::ColorRGBA) -> f32 {
    unsafe {
        assert!(!color.is_null());
        (*color).b
    }
}

#[no_mangle]
pub extern "C" fn ros_color_rgba_get_a(color: *const std_msgs::ColorRGBA) -> f32 {
    unsafe {
        assert!(!color.is_null());
        (*color).a
    }
}

#[no_mangle]
pub extern "C" fn ros_color_rgba_set_r(color: *mut std_msgs::ColorRGBA, r: f32) {
    unsafe {
        assert!(!color.is_null());
        (*color).r = r;
    }
}

#[no_mangle]
pub extern "C" fn ros_color_rgba_set_g(color: *mut std_msgs::ColorRGBA, g: f32) {
    unsafe {
        assert!(!color.is_null());
        (*color).g = g;
    }
}

#[no_mangle]
pub extern "C" fn ros_color_rgba_set_b(color: *mut std_msgs::ColorRGBA, b: f32) {
    unsafe {
        assert!(!color.is_null());
        (*color).b = b;
    }
}

#[no_mangle]
pub extern "C" fn ros_color_rgba_set_a(color: *mut std_msgs::ColorRGBA, a: f32) {
    unsafe {
        assert!(!color.is_null());
        (*color).a = a;
    }
}

#[no_mangle]
pub extern "C" fn ros_color_rgba_serialize(
    color: *const std_msgs::ColorRGBA,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(color);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*color) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_color_rgba_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut std_msgs::ColorRGBA {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<std_msgs::ColorRGBA>(slice) {
            Ok(color) => Box::into_raw(Box::new(color)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::Vector3
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_vector3_new() -> *mut geometry_msgs::Vector3 {
    Box::into_raw(Box::new(geometry_msgs::Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }))
}

#[no_mangle]
pub extern "C" fn ros_vector3_free(vec: *mut geometry_msgs::Vector3) {
    if !vec.is_null() {
        unsafe {
            drop(Box::from_raw(vec));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_vector3_get_x(vec: *const geometry_msgs::Vector3) -> f64 {
    unsafe {
        assert!(!vec.is_null());
        (*vec).x
    }
}

#[no_mangle]
pub extern "C" fn ros_vector3_get_y(vec: *const geometry_msgs::Vector3) -> f64 {
    unsafe {
        assert!(!vec.is_null());
        (*vec).y
    }
}

#[no_mangle]
pub extern "C" fn ros_vector3_get_z(vec: *const geometry_msgs::Vector3) -> f64 {
    unsafe {
        assert!(!vec.is_null());
        (*vec).z
    }
}

#[no_mangle]
pub extern "C" fn ros_vector3_set_x(vec: *mut geometry_msgs::Vector3, x: f64) {
    unsafe {
        assert!(!vec.is_null());
        (*vec).x = x;
    }
}

#[no_mangle]
pub extern "C" fn ros_vector3_set_y(vec: *mut geometry_msgs::Vector3, y: f64) {
    unsafe {
        assert!(!vec.is_null());
        (*vec).y = y;
    }
}

#[no_mangle]
pub extern "C" fn ros_vector3_set_z(vec: *mut geometry_msgs::Vector3, z: f64) {
    unsafe {
        assert!(!vec.is_null());
        (*vec).z = z;
    }
}

#[no_mangle]
pub extern "C" fn ros_vector3_serialize(
    vec: *const geometry_msgs::Vector3,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(vec);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*vec) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_vector3_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut geometry_msgs::Vector3 {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::Vector3>(slice) {
            Ok(vec) => Box::into_raw(Box::new(vec)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::Point
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_point_new() -> *mut geometry_msgs::Point {
    Box::into_raw(Box::new(geometry_msgs::Point {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }))
}

#[no_mangle]
pub extern "C" fn ros_point_free(point: *mut geometry_msgs::Point) {
    if !point.is_null() {
        unsafe {
            drop(Box::from_raw(point));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_point_get_x(point: *const geometry_msgs::Point) -> f64 {
    unsafe {
        assert!(!point.is_null());
        (*point).x
    }
}

#[no_mangle]
pub extern "C" fn ros_point_get_y(point: *const geometry_msgs::Point) -> f64 {
    unsafe {
        assert!(!point.is_null());
        (*point).y
    }
}

#[no_mangle]
pub extern "C" fn ros_point_get_z(point: *const geometry_msgs::Point) -> f64 {
    unsafe {
        assert!(!point.is_null());
        (*point).z
    }
}

#[no_mangle]
pub extern "C" fn ros_point_set_x(point: *mut geometry_msgs::Point, x: f64) {
    unsafe {
        assert!(!point.is_null());
        (*point).x = x;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_set_y(point: *mut geometry_msgs::Point, y: f64) {
    unsafe {
        assert!(!point.is_null());
        (*point).y = y;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_set_z(point: *mut geometry_msgs::Point, z: f64) {
    unsafe {
        assert!(!point.is_null());
        (*point).z = z;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_serialize(
    point: *const geometry_msgs::Point,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(point);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*point) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_point_deserialize(bytes: *const u8, len: usize) -> *mut geometry_msgs::Point {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::Point>(slice) {
            Ok(point) => Box::into_raw(Box::new(point)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::Quaternion
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_quaternion_new() -> *mut geometry_msgs::Quaternion {
    Box::into_raw(Box::new(geometry_msgs::Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    }))
}

#[no_mangle]
pub extern "C" fn ros_quaternion_free(quat: *mut geometry_msgs::Quaternion) {
    if !quat.is_null() {
        unsafe {
            drop(Box::from_raw(quat));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_quaternion_get_x(quat: *const geometry_msgs::Quaternion) -> f64 {
    unsafe {
        assert!(!quat.is_null());
        (*quat).x
    }
}

#[no_mangle]
pub extern "C" fn ros_quaternion_get_y(quat: *const geometry_msgs::Quaternion) -> f64 {
    unsafe {
        assert!(!quat.is_null());
        (*quat).y
    }
}

#[no_mangle]
pub extern "C" fn ros_quaternion_get_z(quat: *const geometry_msgs::Quaternion) -> f64 {
    unsafe {
        assert!(!quat.is_null());
        (*quat).z
    }
}

#[no_mangle]
pub extern "C" fn ros_quaternion_get_w(quat: *const geometry_msgs::Quaternion) -> f64 {
    unsafe {
        assert!(!quat.is_null());
        (*quat).w
    }
}

#[no_mangle]
pub extern "C" fn ros_quaternion_set_x(quat: *mut geometry_msgs::Quaternion, x: f64) {
    unsafe {
        assert!(!quat.is_null());
        (*quat).x = x;
    }
}

#[no_mangle]
pub extern "C" fn ros_quaternion_set_y(quat: *mut geometry_msgs::Quaternion, y: f64) {
    unsafe {
        assert!(!quat.is_null());
        (*quat).y = y;
    }
}

#[no_mangle]
pub extern "C" fn ros_quaternion_set_z(quat: *mut geometry_msgs::Quaternion, z: f64) {
    unsafe {
        assert!(!quat.is_null());
        (*quat).z = z;
    }
}

#[no_mangle]
pub extern "C" fn ros_quaternion_set_w(quat: *mut geometry_msgs::Quaternion, w: f64) {
    unsafe {
        assert!(!quat.is_null());
        (*quat).w = w;
    }
}

#[no_mangle]
pub extern "C" fn ros_quaternion_serialize(
    quat: *const geometry_msgs::Quaternion,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(quat);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*quat) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_quaternion_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut geometry_msgs::Quaternion {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::Quaternion>(slice) {
            Ok(quat) => Box::into_raw(Box::new(quat)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// sensor_msgs::Image
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_image_new() -> *mut sensor_msgs::Image {
    Box::into_raw(Box::new(sensor_msgs::Image {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        height: 0,
        width: 0,
        encoding: String::new(),
        is_bigendian: 0,
        step: 0,
        data: Vec::new(),
    }))
}

#[no_mangle]
pub extern "C" fn ros_image_free(image: *mut sensor_msgs::Image) {
    if !image.is_null() {
        unsafe {
            drop(Box::from_raw(image));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_image_get_header(
    image: *const sensor_msgs::Image,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!image.is_null());
        &(*image).header
    }
}

#[no_mangle]
pub extern "C" fn ros_image_get_header_mut(
    image: *mut sensor_msgs::Image,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!image.is_null());
        &mut (*image).header
    }
}

#[no_mangle]
pub extern "C" fn ros_image_get_height(image: *const sensor_msgs::Image) -> u32 {
    unsafe {
        assert!(!image.is_null());
        (*image).height
    }
}

#[no_mangle]
pub extern "C" fn ros_image_get_width(image: *const sensor_msgs::Image) -> u32 {
    unsafe {
        assert!(!image.is_null());
        (*image).width
    }
}

#[no_mangle]
pub extern "C" fn ros_image_get_encoding(image: *const sensor_msgs::Image) -> *mut c_char {
    unsafe {
        assert!(!image.is_null());
        string_to_c_char(&(*image).encoding)
    }
}

#[no_mangle]
pub extern "C" fn ros_image_get_is_bigendian(image: *const sensor_msgs::Image) -> u8 {
    unsafe {
        assert!(!image.is_null());
        (*image).is_bigendian
    }
}

#[no_mangle]
pub extern "C" fn ros_image_get_step(image: *const sensor_msgs::Image) -> u32 {
    unsafe {
        assert!(!image.is_null());
        (*image).step
    }
}

#[no_mangle]
pub extern "C" fn ros_image_get_data(
    image: *const sensor_msgs::Image,
    out_len: *mut usize,
) -> *const u8 {
    unsafe {
        assert!(!image.is_null());
        assert!(!out_len.is_null());
        let data = &(*image).data;
        *out_len = data.len();
        data.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn ros_image_set_height(image: *mut sensor_msgs::Image, height: u32) {
    unsafe {
        assert!(!image.is_null());
        (*image).height = height;
    }
}

#[no_mangle]
pub extern "C" fn ros_image_set_width(image: *mut sensor_msgs::Image, width: u32) {
    unsafe {
        assert!(!image.is_null());
        (*image).width = width;
    }
}

#[no_mangle]
pub extern "C" fn ros_image_set_encoding(
    image: *mut sensor_msgs::Image,
    encoding: *const c_char,
) -> i32 {
    check_null!(image);
    check_null!(encoding);

    unsafe {
        match c_char_to_string(encoding) {
            Some(s) => {
                (*image).encoding = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_image_set_is_bigendian(image: *mut sensor_msgs::Image, is_bigendian: u8) {
    unsafe {
        assert!(!image.is_null());
        (*image).is_bigendian = is_bigendian;
    }
}

#[no_mangle]
pub extern "C" fn ros_image_set_step(image: *mut sensor_msgs::Image, step: u32) {
    unsafe {
        assert!(!image.is_null());
        (*image).step = step;
    }
}

#[no_mangle]
pub extern "C" fn ros_image_set_data(
    image: *mut sensor_msgs::Image,
    data: *const u8,
    len: usize,
) -> i32 {
    check_null!(image);
    check_null!(data);

    unsafe {
        let slice = slice::from_raw_parts(data, len);
        (*image).data = slice.to_vec();
        0
    }
}

#[no_mangle]
pub extern "C" fn ros_image_serialize(
    image: *const sensor_msgs::Image,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(image);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*image) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_image_deserialize(bytes: *const u8, len: usize) -> *mut sensor_msgs::Image {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<sensor_msgs::Image>(slice) {
            Ok(image) => Box::into_raw(Box::new(image)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// edgefirst_msgs::DmaBuf
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_new() -> *mut edgefirst_msgs::DmaBuf {
    Box::into_raw(Box::new(edgefirst_msgs::DmaBuf {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        pid: 0,
        fd: -1,
        width: 0,
        height: 0,
        stride: 0,
        fourcc: 0,
        length: 0,
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_free(dmabuf: *mut edgefirst_msgs::DmaBuf) {
    if !dmabuf.is_null() {
        unsafe {
            drop(Box::from_raw(dmabuf));
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_header(
    dmabuf: *const edgefirst_msgs::DmaBuf,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!dmabuf.is_null());
        &(*dmabuf).header
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_header_mut(
    dmabuf: *mut edgefirst_msgs::DmaBuf,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!dmabuf.is_null());
        &mut (*dmabuf).header
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_pid(dmabuf: *const edgefirst_msgs::DmaBuf) -> u32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).pid
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_fd(dmabuf: *const edgefirst_msgs::DmaBuf) -> i32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).fd
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_width(dmabuf: *const edgefirst_msgs::DmaBuf) -> u32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).width
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_height(dmabuf: *const edgefirst_msgs::DmaBuf) -> u32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).height
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_stride(dmabuf: *const edgefirst_msgs::DmaBuf) -> u32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).stride
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_fourcc(dmabuf: *const edgefirst_msgs::DmaBuf) -> u32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).fourcc
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_length(dmabuf: *const edgefirst_msgs::DmaBuf) -> u32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).length
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_pid(dmabuf: *mut edgefirst_msgs::DmaBuf, pid: u32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).pid = pid;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_fd(dmabuf: *mut edgefirst_msgs::DmaBuf, fd: i32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).fd = fd;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_width(dmabuf: *mut edgefirst_msgs::DmaBuf, width: u32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).width = width;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_height(dmabuf: *mut edgefirst_msgs::DmaBuf, height: u32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).height = height;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_stride(dmabuf: *mut edgefirst_msgs::DmaBuf, stride: u32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).stride = stride;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_fourcc(dmabuf: *mut edgefirst_msgs::DmaBuf, fourcc: u32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).fourcc = fourcc;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_length(dmabuf: *mut edgefirst_msgs::DmaBuf, length: u32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).length = length;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_serialize(
    dmabuf: *const edgefirst_msgs::DmaBuf,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(dmabuf);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*dmabuf) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::DmaBuf {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::DmaBuf>(slice) {
            Ok(dmabuf) => Box::into_raw(Box::new(dmabuf)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// Note: Additional types (CameraInfo, IMU, PointCloud2, RadarCube, Detect, Model, etc.)
// follow the same pattern. For a production implementation, these would be generated
// using macros or build scripts to reduce code duplication.

// =============================================================================
// Tier 1 C API Implementation
// =============================================================================
// DetectTrack, DetectBox2D, Detect, Mask, PointField, PointCloud2,
// NavSatStatus, NavSatFix

// =============================================================================
// edgefirst_msgs::DetectTrack
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_detecttrack_new() -> *mut edgefirst_msgs::DetectTrack {
    Box::into_raw(Box::new(edgefirst_msgs::DetectTrack {
        id: String::new(),
        lifetime: 0,
        created: builtin_interfaces::Time { sec: 0, nanosec: 0 },
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_detecttrack_free(track: *mut edgefirst_msgs::DetectTrack) {
    if !track.is_null() {
        unsafe {
            drop(Box::from_raw(track));
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detecttrack_get_id(
    track: *const edgefirst_msgs::DetectTrack,
) -> *mut c_char {
    unsafe {
        assert!(!track.is_null());
        string_to_c_char(&(*track).id)
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detecttrack_get_lifetime(
    track: *const edgefirst_msgs::DetectTrack,
) -> i32 {
    unsafe {
        assert!(!track.is_null());
        (*track).lifetime
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detecttrack_get_created_mut(
    track: *mut edgefirst_msgs::DetectTrack,
) -> *mut builtin_interfaces::Time {
    unsafe {
        assert!(!track.is_null());
        &mut (*track).created
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detecttrack_set_id(
    track: *mut edgefirst_msgs::DetectTrack,
    id: *const c_char,
) -> i32 {
    check_null!(track);
    check_null!(id);

    unsafe {
        match c_char_to_string(id) {
            Some(s) => {
                (*track).id = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detecttrack_set_lifetime(
    track: *mut edgefirst_msgs::DetectTrack,
    lifetime: i32,
) {
    unsafe {
        assert!(!track.is_null());
        (*track).lifetime = lifetime;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detecttrack_serialize(
    track: *const edgefirst_msgs::DetectTrack,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(track);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*track) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detecttrack_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::DetectTrack {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::DetectTrack>(slice) {
            Ok(track) => Box::into_raw(Box::new(track)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// edgefirst_msgs::DetectBox2D
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_new() -> *mut edgefirst_msgs::DetectBox2D {
    Box::into_raw(Box::new(edgefirst_msgs::DetectBox2D {
        center_x: 0.0,
        center_y: 0.0,
        width: 0.0,
        height: 0.0,
        label: String::new(),
        score: 0.0,
        distance: 0.0,
        speed: 0.0,
        track: edgefirst_msgs::DetectTrack {
            id: String::new(),
            lifetime: 0,
            created: builtin_interfaces::Time { sec: 0, nanosec: 0 },
        },
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_free(box2d: *mut edgefirst_msgs::DetectBox2D) {
    if !box2d.is_null() {
        unsafe {
            drop(Box::from_raw(box2d));
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_get_center_x(
    box2d: *const edgefirst_msgs::DetectBox2D,
) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).center_x
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_get_center_y(
    box2d: *const edgefirst_msgs::DetectBox2D,
) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).center_y
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_get_width(
    box2d: *const edgefirst_msgs::DetectBox2D,
) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).width
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_get_height(
    box2d: *const edgefirst_msgs::DetectBox2D,
) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).height
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_get_label(
    box2d: *const edgefirst_msgs::DetectBox2D,
) -> *mut c_char {
    unsafe {
        assert!(!box2d.is_null());
        string_to_c_char(&(*box2d).label)
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_get_score(
    box2d: *const edgefirst_msgs::DetectBox2D,
) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).score
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_get_distance(
    box2d: *const edgefirst_msgs::DetectBox2D,
) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).distance
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_get_speed(
    box2d: *const edgefirst_msgs::DetectBox2D,
) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).speed
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_get_track_mut(
    box2d: *mut edgefirst_msgs::DetectBox2D,
) -> *mut edgefirst_msgs::DetectTrack {
    unsafe {
        assert!(!box2d.is_null());
        &mut (*box2d).track
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_set_center_x(
    box2d: *mut edgefirst_msgs::DetectBox2D,
    center_x: f32,
) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).center_x = center_x;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_set_center_y(
    box2d: *mut edgefirst_msgs::DetectBox2D,
    center_y: f32,
) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).center_y = center_y;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_set_width(
    box2d: *mut edgefirst_msgs::DetectBox2D,
    width: f32,
) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).width = width;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_set_height(
    box2d: *mut edgefirst_msgs::DetectBox2D,
    height: f32,
) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).height = height;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_set_label(
    box2d: *mut edgefirst_msgs::DetectBox2D,
    label: *const c_char,
) -> i32 {
    check_null!(box2d);
    check_null!(label);

    unsafe {
        match c_char_to_string(label) {
            Some(s) => {
                (*box2d).label = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_set_score(
    box2d: *mut edgefirst_msgs::DetectBox2D,
    score: f32,
) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).score = score;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_set_distance(
    box2d: *mut edgefirst_msgs::DetectBox2D,
    distance: f32,
) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).distance = distance;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_set_speed(
    box2d: *mut edgefirst_msgs::DetectBox2D,
    speed: f32,
) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).speed = speed;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_serialize(
    box2d: *const edgefirst_msgs::DetectBox2D,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(box2d);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*box2d) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detectbox2d_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::DetectBox2D {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::DetectBox2D>(slice) {
            Ok(box2d) => Box::into_raw(Box::new(box2d)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// edgefirst_msgs::Detect
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_detect_new() -> *mut edgefirst_msgs::Detect {
    Box::into_raw(Box::new(edgefirst_msgs::Detect {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        input_timestamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
        model_time: builtin_interfaces::Time { sec: 0, nanosec: 0 },
        output_time: builtin_interfaces::Time { sec: 0, nanosec: 0 },
        boxes: Vec::new(),
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_detect_free(detect: *mut edgefirst_msgs::Detect) {
    if !detect.is_null() {
        unsafe {
            drop(Box::from_raw(detect));
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detect_get_header_mut(
    detect: *mut edgefirst_msgs::Detect,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!detect.is_null());
        &mut (*detect).header
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detect_get_input_timestamp_mut(
    detect: *mut edgefirst_msgs::Detect,
) -> *mut builtin_interfaces::Time {
    unsafe {
        assert!(!detect.is_null());
        &mut (*detect).input_timestamp
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detect_get_model_time_mut(
    detect: *mut edgefirst_msgs::Detect,
) -> *mut builtin_interfaces::Time {
    unsafe {
        assert!(!detect.is_null());
        &mut (*detect).model_time
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detect_get_output_time_mut(
    detect: *mut edgefirst_msgs::Detect,
) -> *mut builtin_interfaces::Time {
    unsafe {
        assert!(!detect.is_null());
        &mut (*detect).output_time
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detect_get_boxes(
    detect: *const edgefirst_msgs::Detect,
    out_len: *mut usize,
) -> *const edgefirst_msgs::DetectBox2D {
    unsafe {
        assert!(!detect.is_null());
        assert!(!out_len.is_null());
        *out_len = (*detect).boxes.len();
        (*detect).boxes.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detect_add_box(
    detect: *mut edgefirst_msgs::Detect,
    box2d: *const edgefirst_msgs::DetectBox2D,
) -> i32 {
    check_null!(detect);
    check_null!(box2d);

    unsafe {
        (*detect).boxes.push((*box2d).clone());
        0
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detect_clear_boxes(detect: *mut edgefirst_msgs::Detect) {
    unsafe {
        assert!(!detect.is_null());
        (*detect).boxes.clear();
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detect_serialize(
    detect: *const edgefirst_msgs::Detect,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(detect);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*detect) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_detect_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::Detect {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::Detect>(slice) {
            Ok(detect) => Box::into_raw(Box::new(detect)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// edgefirst_msgs::Mask
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_mask_new() -> *mut edgefirst_msgs::Mask {
    Box::into_raw(Box::new(edgefirst_msgs::Mask {
        height: 0,
        width: 0,
        length: 0,
        encoding: String::new(),
        mask: Vec::new(),
        boxed: false,
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_free(mask: *mut edgefirst_msgs::Mask) {
    if !mask.is_null() {
        unsafe {
            drop(Box::from_raw(mask));
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_get_height(mask: *const edgefirst_msgs::Mask) -> u32 {
    unsafe {
        assert!(!mask.is_null());
        (*mask).height
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_get_width(mask: *const edgefirst_msgs::Mask) -> u32 {
    unsafe {
        assert!(!mask.is_null());
        (*mask).width
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_get_length(mask: *const edgefirst_msgs::Mask) -> u32 {
    unsafe {
        assert!(!mask.is_null());
        (*mask).length
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_get_encoding(mask: *const edgefirst_msgs::Mask) -> *mut c_char {
    unsafe {
        assert!(!mask.is_null());
        string_to_c_char(&(*mask).encoding)
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_get_mask(
    mask: *const edgefirst_msgs::Mask,
    out_len: *mut usize,
) -> *const u8 {
    unsafe {
        assert!(!mask.is_null());
        assert!(!out_len.is_null());
        *out_len = (*mask).mask.len();
        (*mask).mask.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_get_boxed(mask: *const edgefirst_msgs::Mask) -> bool {
    unsafe {
        assert!(!mask.is_null());
        (*mask).boxed
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_set_height(mask: *mut edgefirst_msgs::Mask, height: u32) {
    unsafe {
        assert!(!mask.is_null());
        (*mask).height = height;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_set_width(mask: *mut edgefirst_msgs::Mask, width: u32) {
    unsafe {
        assert!(!mask.is_null());
        (*mask).width = width;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_set_length(mask: *mut edgefirst_msgs::Mask, length: u32) {
    unsafe {
        assert!(!mask.is_null());
        (*mask).length = length;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_set_encoding(
    mask: *mut edgefirst_msgs::Mask,
    encoding: *const c_char,
) -> i32 {
    check_null!(mask);
    check_null!(encoding);

    unsafe {
        match c_char_to_string(encoding) {
            Some(s) => {
                (*mask).encoding = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_set_mask(
    mask: *mut edgefirst_msgs::Mask,
    data: *const u8,
    len: usize,
) -> i32 {
    check_null!(mask);
    check_null!(data);

    unsafe {
        let slice = slice::from_raw_parts(data, len);
        (*mask).mask = slice.to_vec();
        0
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_set_boxed(mask: *mut edgefirst_msgs::Mask, boxed: bool) {
    unsafe {
        assert!(!mask.is_null());
        (*mask).boxed = boxed;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_serialize(
    mask: *const edgefirst_msgs::Mask,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(mask);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*mask) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_mask_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::Mask {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::Mask>(slice) {
            Ok(mask) => Box::into_raw(Box::new(mask)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// sensor_msgs::PointField
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_point_field_new() -> *mut sensor_msgs::PointField {
    Box::into_raw(Box::new(sensor_msgs::PointField {
        name: String::new(),
        offset: 0,
        datatype: 0,
        count: 1,
    }))
}

#[no_mangle]
pub extern "C" fn ros_point_field_free(field: *mut sensor_msgs::PointField) {
    if !field.is_null() {
        unsafe {
            drop(Box::from_raw(field));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_get_name(field: *const sensor_msgs::PointField) -> *mut c_char {
    unsafe {
        assert!(!field.is_null());
        string_to_c_char(&(*field).name)
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_get_offset(field: *const sensor_msgs::PointField) -> u32 {
    unsafe {
        assert!(!field.is_null());
        (*field).offset
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_get_datatype(field: *const sensor_msgs::PointField) -> u8 {
    unsafe {
        assert!(!field.is_null());
        (*field).datatype
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_get_count(field: *const sensor_msgs::PointField) -> u32 {
    unsafe {
        assert!(!field.is_null());
        (*field).count
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_set_name(
    field: *mut sensor_msgs::PointField,
    name: *const c_char,
) -> i32 {
    check_null!(field);
    check_null!(name);

    unsafe {
        match c_char_to_string(name) {
            Some(s) => {
                (*field).name = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_set_offset(field: *mut sensor_msgs::PointField, offset: u32) {
    unsafe {
        assert!(!field.is_null());
        (*field).offset = offset;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_set_datatype(field: *mut sensor_msgs::PointField, datatype: u8) {
    unsafe {
        assert!(!field.is_null());
        (*field).datatype = datatype;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_set_count(field: *mut sensor_msgs::PointField, count: u32) {
    unsafe {
        assert!(!field.is_null());
        (*field).count = count;
    }
}

// =============================================================================
// sensor_msgs::PointCloud2
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_point_cloud2_new() -> *mut sensor_msgs::PointCloud2 {
    Box::into_raw(Box::new(sensor_msgs::PointCloud2 {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        height: 0,
        width: 0,
        fields: Vec::new(),
        is_bigendian: false,
        point_step: 0,
        row_step: 0,
        data: Vec::new(),
        is_dense: false,
    }))
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_free(cloud: *mut sensor_msgs::PointCloud2) {
    if !cloud.is_null() {
        unsafe {
            drop(Box::from_raw(cloud));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_header_mut(
    cloud: *mut sensor_msgs::PointCloud2,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!cloud.is_null());
        &mut (*cloud).header
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_height(cloud: *const sensor_msgs::PointCloud2) -> u32 {
    unsafe {
        assert!(!cloud.is_null());
        (*cloud).height
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_width(cloud: *const sensor_msgs::PointCloud2) -> u32 {
    unsafe {
        assert!(!cloud.is_null());
        (*cloud).width
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_fields(
    cloud: *const sensor_msgs::PointCloud2,
    out_len: *mut usize,
) -> *const sensor_msgs::PointField {
    unsafe {
        assert!(!cloud.is_null());
        assert!(!out_len.is_null());
        *out_len = (*cloud).fields.len();
        (*cloud).fields.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_is_bigendian(
    cloud: *const sensor_msgs::PointCloud2,
) -> bool {
    unsafe {
        assert!(!cloud.is_null());
        (*cloud).is_bigendian
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_point_step(cloud: *const sensor_msgs::PointCloud2) -> u32 {
    unsafe {
        assert!(!cloud.is_null());
        (*cloud).point_step
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_row_step(cloud: *const sensor_msgs::PointCloud2) -> u32 {
    unsafe {
        assert!(!cloud.is_null());
        (*cloud).row_step
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_data(
    cloud: *const sensor_msgs::PointCloud2,
    out_len: *mut usize,
) -> *const u8 {
    unsafe {
        assert!(!cloud.is_null());
        assert!(!out_len.is_null());
        *out_len = (*cloud).data.len();
        (*cloud).data.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_is_dense(cloud: *const sensor_msgs::PointCloud2) -> bool {
    unsafe {
        assert!(!cloud.is_null());
        (*cloud).is_dense
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_height(cloud: *mut sensor_msgs::PointCloud2, height: u32) {
    unsafe {
        assert!(!cloud.is_null());
        (*cloud).height = height;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_width(cloud: *mut sensor_msgs::PointCloud2, width: u32) {
    unsafe {
        assert!(!cloud.is_null());
        (*cloud).width = width;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_is_bigendian(
    cloud: *mut sensor_msgs::PointCloud2,
    is_bigendian: bool,
) {
    unsafe {
        assert!(!cloud.is_null());
        (*cloud).is_bigendian = is_bigendian;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_point_step(
    cloud: *mut sensor_msgs::PointCloud2,
    point_step: u32,
) {
    unsafe {
        assert!(!cloud.is_null());
        (*cloud).point_step = point_step;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_row_step(
    cloud: *mut sensor_msgs::PointCloud2,
    row_step: u32,
) {
    unsafe {
        assert!(!cloud.is_null());
        (*cloud).row_step = row_step;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_is_dense(
    cloud: *mut sensor_msgs::PointCloud2,
    is_dense: bool,
) {
    unsafe {
        assert!(!cloud.is_null());
        (*cloud).is_dense = is_dense;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_add_field(
    cloud: *mut sensor_msgs::PointCloud2,
    field: *const sensor_msgs::PointField,
) -> i32 {
    check_null!(cloud);
    check_null!(field);

    unsafe {
        (*cloud).fields.push((*field).clone());
        0
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_data(
    cloud: *mut sensor_msgs::PointCloud2,
    data: *const u8,
    len: usize,
) -> i32 {
    check_null!(cloud);
    check_null!(data);

    unsafe {
        let slice = slice::from_raw_parts(data, len);
        (*cloud).data = slice.to_vec();
        0
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_serialize(
    cloud: *const sensor_msgs::PointCloud2,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(cloud);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*cloud) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut sensor_msgs::PointCloud2 {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<sensor_msgs::PointCloud2>(slice) {
            Ok(cloud) => Box::into_raw(Box::new(cloud)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// sensor_msgs::NavSatStatus
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_nav_sat_status_new() -> *mut sensor_msgs::NavSatStatus {
    Box::into_raw(Box::new(sensor_msgs::NavSatStatus {
        status: -1,
        service: 0,
    }))
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_status_free(status: *mut sensor_msgs::NavSatStatus) {
    if !status.is_null() {
        unsafe {
            drop(Box::from_raw(status));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_status_get_status(status: *const sensor_msgs::NavSatStatus) -> i8 {
    unsafe {
        assert!(!status.is_null());
        (*status).status
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_status_get_service(status: *const sensor_msgs::NavSatStatus) -> u16 {
    unsafe {
        assert!(!status.is_null());
        (*status).service
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_status_set_status(status: *mut sensor_msgs::NavSatStatus, value: i8) {
    unsafe {
        assert!(!status.is_null());
        (*status).status = value;
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_status_set_service(
    status: *mut sensor_msgs::NavSatStatus,
    value: u16,
) {
    unsafe {
        assert!(!status.is_null());
        (*status).service = value;
    }
}

// =============================================================================
// sensor_msgs::NavSatFix
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_new() -> *mut sensor_msgs::NavSatFix {
    Box::into_raw(Box::new(sensor_msgs::NavSatFix {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        status: sensor_msgs::NavSatStatus {
            status: -1,
            service: 0,
        },
        latitude: 0.0,
        longitude: 0.0,
        altitude: 0.0,
        position_covariance: [0.0; 9],
        position_covariance_type: 0,
    }))
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_free(fix: *mut sensor_msgs::NavSatFix) {
    if !fix.is_null() {
        unsafe {
            drop(Box::from_raw(fix));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_header_mut(
    fix: *mut sensor_msgs::NavSatFix,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!fix.is_null());
        &mut (*fix).header
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_status_mut(
    fix: *mut sensor_msgs::NavSatFix,
) -> *mut sensor_msgs::NavSatStatus {
    unsafe {
        assert!(!fix.is_null());
        &mut (*fix).status
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_latitude(fix: *const sensor_msgs::NavSatFix) -> f64 {
    unsafe {
        assert!(!fix.is_null());
        (*fix).latitude
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_longitude(fix: *const sensor_msgs::NavSatFix) -> f64 {
    unsafe {
        assert!(!fix.is_null());
        (*fix).longitude
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_altitude(fix: *const sensor_msgs::NavSatFix) -> f64 {
    unsafe {
        assert!(!fix.is_null());
        (*fix).altitude
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_position_covariance(
    fix: *const sensor_msgs::NavSatFix,
) -> *const f64 {
    unsafe {
        assert!(!fix.is_null());
        (*fix).position_covariance.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_position_covariance_type(
    fix: *const sensor_msgs::NavSatFix,
) -> u8 {
    unsafe {
        assert!(!fix.is_null());
        (*fix).position_covariance_type
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_set_latitude(fix: *mut sensor_msgs::NavSatFix, latitude: f64) {
    unsafe {
        assert!(!fix.is_null());
        (*fix).latitude = latitude;
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_set_longitude(fix: *mut sensor_msgs::NavSatFix, longitude: f64) {
    unsafe {
        assert!(!fix.is_null());
        (*fix).longitude = longitude;
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_set_altitude(fix: *mut sensor_msgs::NavSatFix, altitude: f64) {
    unsafe {
        assert!(!fix.is_null());
        (*fix).altitude = altitude;
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_set_position_covariance(
    fix: *mut sensor_msgs::NavSatFix,
    covariance: *const f64,
) {
    unsafe {
        assert!(!fix.is_null());
        assert!(!covariance.is_null());
        let slice = slice::from_raw_parts(covariance, 9);
        (*fix).position_covariance.copy_from_slice(slice);
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_set_position_covariance_type(
    fix: *mut sensor_msgs::NavSatFix,
    cov_type: u8,
) {
    unsafe {
        assert!(!fix.is_null());
        (*fix).position_covariance_type = cov_type;
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_serialize(
    fix: *const sensor_msgs::NavSatFix,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(fix);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*fix) {
            Ok(bytes) => {
                let len = bytes.len();
                let ptr = Box::into_raw(bytes.into_boxed_slice()) as *mut u8;
                *out_bytes = ptr;
                *out_len = len;
                0
            }
            Err(_) => {
                set_errno(ENOMEM);
                -1
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut sensor_msgs::NavSatFix {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<sensor_msgs::NavSatFix>(slice) {
            Ok(fix) => Box::into_raw(Box::new(fix)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}
