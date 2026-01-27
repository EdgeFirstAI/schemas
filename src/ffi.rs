// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! C FFI (Foreign Function Interface) module
//!
//! This module provides C-compatible bindings for all schema types with
//! CDR serialization support.

#![allow(non_camel_case_types)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]
// Explicit borrows on raw pointer dereferences (e.g., `(&(*ptr).field).get()`) are intentional
// in FFI code to make reference creation visible when working with unsafe raw pointers.
// Rust 1.93+ flags these as needless_borrow but removing them triggers implicit autoref errors.
#![allow(clippy::needless_borrow)]

use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::ptr;
use std::slice;

use crate::serde_cdr;
use crate::*;

// =============================================================================
// errno Support
// =============================================================================

/// errno constants - libc crate provides these for all platforms including Windows
const EINVAL: i32 = libc::EINVAL;
const ENOMEM: i32 = libc::ENOMEM;
const EBADMSG: i32 = libc::EBADMSG;
const ENOBUFS: i32 = libc::ENOBUFS;

/// Set errno portably across all platforms (Linux, macOS, Windows, etc.)
fn set_errno(code: i32) {
    errno::set_errno(errno::Errno(code));
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
    if time.is_null() {
        return 0;
    }
    unsafe { (*time).sec }
}

#[no_mangle]
pub extern "C" fn ros_time_get_nanosec(time: *const builtin_interfaces::Time) -> u32 {
    if time.is_null() {
        return 0;
    }
    unsafe { (*time).nanosec }
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
// edgefirst_msgs::DmaBuffer
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_new() -> *mut edgefirst_msgs::DmaBuffer {
    Box::into_raw(Box::new(edgefirst_msgs::DmaBuffer {
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
pub extern "C" fn edgefirst_dmabuf_free(dmabuf: *mut edgefirst_msgs::DmaBuffer) {
    if !dmabuf.is_null() {
        unsafe {
            drop(Box::from_raw(dmabuf));
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_header(
    dmabuf: *const edgefirst_msgs::DmaBuffer,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!dmabuf.is_null());
        &(*dmabuf).header
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_header_mut(
    dmabuf: *mut edgefirst_msgs::DmaBuffer,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!dmabuf.is_null());
        &mut (*dmabuf).header
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_pid(dmabuf: *const edgefirst_msgs::DmaBuffer) -> u32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).pid
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_fd(dmabuf: *const edgefirst_msgs::DmaBuffer) -> i32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).fd
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_width(dmabuf: *const edgefirst_msgs::DmaBuffer) -> u32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).width
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_height(dmabuf: *const edgefirst_msgs::DmaBuffer) -> u32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).height
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_stride(dmabuf: *const edgefirst_msgs::DmaBuffer) -> u32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).stride
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_fourcc(dmabuf: *const edgefirst_msgs::DmaBuffer) -> u32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).fourcc
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_get_length(dmabuf: *const edgefirst_msgs::DmaBuffer) -> u32 {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).length
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_pid(dmabuf: *mut edgefirst_msgs::DmaBuffer, pid: u32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).pid = pid;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_fd(dmabuf: *mut edgefirst_msgs::DmaBuffer, fd: i32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).fd = fd;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_width(dmabuf: *mut edgefirst_msgs::DmaBuffer, width: u32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).width = width;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_height(dmabuf: *mut edgefirst_msgs::DmaBuffer, height: u32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).height = height;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_stride(dmabuf: *mut edgefirst_msgs::DmaBuffer, stride: u32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).stride = stride;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_fourcc(dmabuf: *mut edgefirst_msgs::DmaBuffer, fourcc: u32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).fourcc = fourcc;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_set_length(dmabuf: *mut edgefirst_msgs::DmaBuffer, length: u32) {
    unsafe {
        assert!(!dmabuf.is_null());
        (*dmabuf).length = length;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_dmabuf_serialize(
    dmabuf: *const edgefirst_msgs::DmaBuffer,
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
) -> *mut edgefirst_msgs::DmaBuffer {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::DmaBuffer>(slice) {
            Ok(dmabuf) => Box::into_raw(Box::new(dmabuf)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// foxglove_msgs::FoxgloveCompressedVideo
// =============================================================================

#[no_mangle]
pub extern "C" fn foxglove_compressed_video_new() -> *mut foxglove_msgs::FoxgloveCompressedVideo {
    Box::into_raw(Box::new(foxglove_msgs::FoxgloveCompressedVideo {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        data: Vec::new(),
        format: String::new(),
    }))
}

#[no_mangle]
pub extern "C" fn foxglove_compressed_video_free(
    video: *mut foxglove_msgs::FoxgloveCompressedVideo,
) {
    if !video.is_null() {
        unsafe {
            drop(Box::from_raw(video));
        }
    }
}

#[no_mangle]
pub extern "C" fn foxglove_compressed_video_get_header(
    video: *const foxglove_msgs::FoxgloveCompressedVideo,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!video.is_null());
        &(*video).header
    }
}

#[no_mangle]
pub extern "C" fn foxglove_compressed_video_get_header_mut(
    video: *mut foxglove_msgs::FoxgloveCompressedVideo,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!video.is_null());
        &mut (*video).header
    }
}

#[no_mangle]
pub extern "C" fn foxglove_compressed_video_get_data(
    video: *const foxglove_msgs::FoxgloveCompressedVideo,
    out_len: *mut usize,
) -> *const u8 {
    unsafe {
        assert!(!video.is_null());
        assert!(!out_len.is_null());
        let data = &(*video).data;
        *out_len = data.len();
        data.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn foxglove_compressed_video_get_format(
    video: *const foxglove_msgs::FoxgloveCompressedVideo,
) -> *mut c_char {
    unsafe {
        assert!(!video.is_null());
        string_to_c_char(&(*video).format)
    }
}

#[no_mangle]
pub extern "C" fn foxglove_compressed_video_set_data(
    video: *mut foxglove_msgs::FoxgloveCompressedVideo,
    data: *const u8,
    len: usize,
) -> i32 {
    check_null!(video);
    check_null!(data);

    unsafe {
        let slice = slice::from_raw_parts(data, len);
        (*video).data = slice.to_vec();
        0
    }
}

#[no_mangle]
pub extern "C" fn foxglove_compressed_video_set_format(
    video: *mut foxglove_msgs::FoxgloveCompressedVideo,
    format: *const c_char,
) -> i32 {
    check_null!(video);
    check_null!(format);

    unsafe {
        match c_char_to_string(format) {
            Some(s) => {
                (*video).format = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

/// Serializes FoxgloveCompressedVideo to CDR format using Khronos-style buffer pattern.
///
/// # Arguments
/// * `video` - Video to serialize (must not be NULL)
/// * `buffer` - Output buffer (may be NULL to query size)
/// * `capacity` - Buffer size in bytes (ignored if buffer is NULL)
/// * `size` - Receives bytes written/required (may be NULL)
///
/// # Returns
/// 0 on success, -1 on error with errno set:
/// - EINVAL: video is NULL
/// - ENOBUFS: buffer too small (size always written with required capacity)
/// - EBADMSG: serialization failed
#[no_mangle]
pub extern "C" fn foxglove_compressed_video_serialize(
    video: *const foxglove_msgs::FoxgloveCompressedVideo,
    buffer: *mut u8,
    capacity: usize,
    size: *mut usize,
) -> i32 {
    if video.is_null() {
        set_errno(EINVAL);
        return -1;
    }

    let bytes = match unsafe { serde_cdr::serialize(&*video) } {
        Ok(b) => b,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };

    // Write size if requested (always, even on ENOBUFS)
    if !size.is_null() {
        unsafe {
            *size = bytes.len();
        }
    }

    // If buffer is NULL, caller is querying size only
    if buffer.is_null() {
        return 0;
    }

    // Check capacity
    if capacity < bytes.len() {
        set_errno(ENOBUFS);
        return -1;
    }

    // Copy to caller's buffer
    unsafe {
        ptr::copy_nonoverlapping(bytes.as_ptr(), buffer, bytes.len());
    }
    0
}

#[no_mangle]
pub extern "C" fn foxglove_compressed_video_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut foxglove_msgs::FoxgloveCompressedVideo {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<foxglove_msgs::FoxgloveCompressedVideo>(slice) {
            Ok(video) => Box::into_raw(Box::new(video)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// edgefirst_msgs::RadarCube
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_new() -> *mut edgefirst_msgs::RadarCube {
    Box::into_raw(Box::new(edgefirst_msgs::RadarCube {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        timestamp: 0,
        layout: Vec::new(),
        shape: Vec::new(),
        scales: Vec::new(),
        cube: Vec::new(),
        is_complex: false,
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_free(cube: *mut edgefirst_msgs::RadarCube) {
    if !cube.is_null() {
        unsafe {
            drop(Box::from_raw(cube));
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_get_header(
    cube: *const edgefirst_msgs::RadarCube,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!cube.is_null());
        &(*cube).header
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_get_header_mut(
    cube: *mut edgefirst_msgs::RadarCube,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!cube.is_null());
        &mut (*cube).header
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_get_timestamp(cube: *const edgefirst_msgs::RadarCube) -> u64 {
    unsafe {
        assert!(!cube.is_null());
        (*cube).timestamp
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_set_timestamp(
    cube: *mut edgefirst_msgs::RadarCube,
    timestamp: u64,
) {
    unsafe {
        assert!(!cube.is_null());
        (*cube).timestamp = timestamp;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_get_layout(
    cube: *const edgefirst_msgs::RadarCube,
    out_len: *mut usize,
) -> *const u8 {
    unsafe {
        assert!(!cube.is_null());
        assert!(!out_len.is_null());
        let layout = &(*cube).layout;
        *out_len = layout.len();
        layout.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_set_layout(
    cube: *mut edgefirst_msgs::RadarCube,
    layout: *const u8,
    len: usize,
) -> i32 {
    check_null!(cube);
    check_null!(layout);

    unsafe {
        let slice = slice::from_raw_parts(layout, len);
        (*cube).layout = slice.to_vec();
        0
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_get_shape(
    cube: *const edgefirst_msgs::RadarCube,
    out_len: *mut usize,
) -> *const u16 {
    unsafe {
        assert!(!cube.is_null());
        assert!(!out_len.is_null());
        let shape = &(*cube).shape;
        *out_len = shape.len();
        shape.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_set_shape(
    cube: *mut edgefirst_msgs::RadarCube,
    shape: *const u16,
    len: usize,
) -> i32 {
    check_null!(cube);
    check_null!(shape);

    unsafe {
        let slice = slice::from_raw_parts(shape, len);
        (*cube).shape = slice.to_vec();
        0
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_get_scales(
    cube: *const edgefirst_msgs::RadarCube,
    out_len: *mut usize,
) -> *const f32 {
    unsafe {
        assert!(!cube.is_null());
        assert!(!out_len.is_null());
        let scales = &(*cube).scales;
        *out_len = scales.len();
        scales.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_set_scales(
    cube: *mut edgefirst_msgs::RadarCube,
    scales: *const f32,
    len: usize,
) -> i32 {
    check_null!(cube);
    check_null!(scales);

    unsafe {
        let slice = slice::from_raw_parts(scales, len);
        (*cube).scales = slice.to_vec();
        0
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_get_cube(
    cube: *const edgefirst_msgs::RadarCube,
    out_len: *mut usize,
) -> *const i16 {
    unsafe {
        assert!(!cube.is_null());
        assert!(!out_len.is_null());
        let data = &(*cube).cube;
        *out_len = data.len();
        data.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_set_cube(
    cube: *mut edgefirst_msgs::RadarCube,
    data: *const i16,
    len: usize,
) -> i32 {
    check_null!(cube);
    check_null!(data);

    unsafe {
        let slice = slice::from_raw_parts(data, len);
        (*cube).cube = slice.to_vec();
        0
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_get_is_complex(
    cube: *const edgefirst_msgs::RadarCube,
) -> bool {
    unsafe {
        assert!(!cube.is_null());
        (*cube).is_complex
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_set_is_complex(
    cube: *mut edgefirst_msgs::RadarCube,
    is_complex: bool,
) {
    unsafe {
        assert!(!cube.is_null());
        (*cube).is_complex = is_complex;
    }
}

/// Serializes RadarCube to CDR format using Khronos-style buffer pattern.
///
/// # Arguments
/// * `cube` - RadarCube to serialize (must not be NULL)
/// * `buffer` - Output buffer (may be NULL to query size)
/// * `capacity` - Buffer size in bytes (ignored if buffer is NULL)
/// * `size` - Receives bytes written/required (may be NULL)
///
/// # Returns
/// 0 on success, -1 on error with errno set:
/// - EINVAL: cube is NULL
/// - ENOBUFS: buffer too small (size always written with required capacity)
/// - EBADMSG: serialization failed
#[no_mangle]
pub extern "C" fn edgefirst_radarcube_serialize(
    cube: *const edgefirst_msgs::RadarCube,
    buffer: *mut u8,
    capacity: usize,
    size: *mut usize,
) -> i32 {
    if cube.is_null() {
        set_errno(EINVAL);
        return -1;
    }

    let bytes = match unsafe { serde_cdr::serialize(&*cube) } {
        Ok(b) => b,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };

    // Write size if requested (always, even on ENOBUFS)
    if !size.is_null() {
        unsafe {
            *size = bytes.len();
        }
    }

    // If buffer is NULL, caller is querying size only
    if buffer.is_null() {
        return 0;
    }

    // Check capacity
    if capacity < bytes.len() {
        set_errno(ENOBUFS);
        return -1;
    }

    // Copy to caller's buffer
    unsafe {
        ptr::copy_nonoverlapping(bytes.as_ptr(), buffer, bytes.len());
    }
    0
}

#[no_mangle]
pub extern "C" fn edgefirst_radarcube_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::RadarCube {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::RadarCube>(slice) {
            Ok(cube) => Box::into_raw(Box::new(cube)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// Tier 1 C API Implementation
// =============================================================================
// Track, Box, Detect, Mask, PointField, PointCloud2,
// NavSatStatus, NavSatFix

// =============================================================================
// edgefirst_msgs::Track
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_track_new() -> *mut edgefirst_msgs::Track {
    Box::into_raw(Box::new(edgefirst_msgs::Track {
        id: String::new(),
        lifetime: 0,
        created: builtin_interfaces::Time { sec: 0, nanosec: 0 },
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_track_free(track: *mut edgefirst_msgs::Track) {
    if !track.is_null() {
        unsafe {
            drop(Box::from_raw(track));
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_track_get_id(track: *const edgefirst_msgs::Track) -> *mut c_char {
    unsafe {
        assert!(!track.is_null());
        string_to_c_char(&(*track).id)
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_track_get_lifetime(track: *const edgefirst_msgs::Track) -> i32 {
    unsafe {
        assert!(!track.is_null());
        (*track).lifetime
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_track_get_created_mut(
    track: *mut edgefirst_msgs::Track,
) -> *mut builtin_interfaces::Time {
    unsafe {
        assert!(!track.is_null());
        &mut (*track).created
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_track_set_id(
    track: *mut edgefirst_msgs::Track,
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
pub extern "C" fn edgefirst_track_set_lifetime(track: *mut edgefirst_msgs::Track, lifetime: i32) {
    unsafe {
        assert!(!track.is_null());
        (*track).lifetime = lifetime;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_track_serialize(
    track: *const edgefirst_msgs::Track,
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
pub extern "C" fn edgefirst_track_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::Track {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::Track>(slice) {
            Ok(track) => Box::into_raw(Box::new(track)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// edgefirst_msgs::Box
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_box_new() -> *mut edgefirst_msgs::Box {
    Box::into_raw(Box::new(edgefirst_msgs::Box {
        center_x: 0.0,
        center_y: 0.0,
        width: 0.0,
        height: 0.0,
        label: String::new(),
        score: 0.0,
        distance: 0.0,
        speed: 0.0,
        track: edgefirst_msgs::Track {
            id: String::new(),
            lifetime: 0,
            created: builtin_interfaces::Time { sec: 0, nanosec: 0 },
        },
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_box_free(box2d: *mut edgefirst_msgs::Box) {
    if !box2d.is_null() {
        unsafe {
            drop(Box::from_raw(box2d));
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_get_center_x(box2d: *const edgefirst_msgs::Box) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).center_x
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_get_center_y(box2d: *const edgefirst_msgs::Box) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).center_y
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_get_width(box2d: *const edgefirst_msgs::Box) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).width
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_get_height(box2d: *const edgefirst_msgs::Box) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).height
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_get_label(box2d: *const edgefirst_msgs::Box) -> *mut c_char {
    unsafe {
        assert!(!box2d.is_null());
        string_to_c_char(&(*box2d).label)
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_get_score(box2d: *const edgefirst_msgs::Box) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).score
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_get_distance(box2d: *const edgefirst_msgs::Box) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).distance
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_get_speed(box2d: *const edgefirst_msgs::Box) -> f32 {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).speed
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_get_track_mut(
    box2d: *mut edgefirst_msgs::Box,
) -> *mut edgefirst_msgs::Track {
    unsafe {
        assert!(!box2d.is_null());
        &mut (*box2d).track
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_set_center_x(box2d: *mut edgefirst_msgs::Box, center_x: f32) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).center_x = center_x;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_set_center_y(box2d: *mut edgefirst_msgs::Box, center_y: f32) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).center_y = center_y;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_set_width(box2d: *mut edgefirst_msgs::Box, width: f32) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).width = width;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_set_height(box2d: *mut edgefirst_msgs::Box, height: f32) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).height = height;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_set_label(
    box2d: *mut edgefirst_msgs::Box,
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
pub extern "C" fn edgefirst_box_set_score(box2d: *mut edgefirst_msgs::Box, score: f32) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).score = score;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_set_distance(box2d: *mut edgefirst_msgs::Box, distance: f32) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).distance = distance;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_set_speed(box2d: *mut edgefirst_msgs::Box, speed: f32) {
    unsafe {
        assert!(!box2d.is_null());
        (*box2d).speed = speed;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_box_serialize(
    box2d: *const edgefirst_msgs::Box,
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
pub extern "C" fn edgefirst_box_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::Box {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::Box>(slice) {
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
) -> *const edgefirst_msgs::Box {
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
    box2d: *const edgefirst_msgs::Box,
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
    if mask.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    if out_len.is_null() {
        return ptr::null();
    }
    unsafe {
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
pub extern "C" fn ros_nav_sat_status_get_status(status: *const sensor_msgs::NavSatStatus) -> i16 {
    unsafe {
        assert!(!status.is_null());
        (*status).status as i16
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
pub extern "C" fn ros_nav_sat_status_set_status(
    status: *mut sensor_msgs::NavSatStatus,
    value: i16,
) {
    unsafe {
        assert!(!status.is_null());
        (*status).status = value as i8;
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
) -> i32 {
    if fix.is_null() || covariance.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        let slice = slice::from_raw_parts(covariance, 9);
        (*fix).position_covariance.copy_from_slice(slice);
    }
    0
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

// =============================================================================
// geometry_msgs::Point32
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_point32_new() -> *mut geometry_msgs::Point32 {
    Box::into_raw(Box::new(geometry_msgs::Point32 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }))
}

#[no_mangle]
pub extern "C" fn ros_point32_free(point: *mut geometry_msgs::Point32) {
    if !point.is_null() {
        unsafe {
            drop(Box::from_raw(point));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_point32_get_x(point: *const geometry_msgs::Point32) -> f32 {
    unsafe {
        assert!(!point.is_null());
        (*point).x
    }
}

#[no_mangle]
pub extern "C" fn ros_point32_get_y(point: *const geometry_msgs::Point32) -> f32 {
    unsafe {
        assert!(!point.is_null());
        (*point).y
    }
}

#[no_mangle]
pub extern "C" fn ros_point32_get_z(point: *const geometry_msgs::Point32) -> f32 {
    unsafe {
        assert!(!point.is_null());
        (*point).z
    }
}

#[no_mangle]
pub extern "C" fn ros_point32_set_x(point: *mut geometry_msgs::Point32, x: f32) {
    unsafe {
        assert!(!point.is_null());
        (*point).x = x;
    }
}

#[no_mangle]
pub extern "C" fn ros_point32_set_y(point: *mut geometry_msgs::Point32, y: f32) {
    unsafe {
        assert!(!point.is_null());
        (*point).y = y;
    }
}

#[no_mangle]
pub extern "C" fn ros_point32_set_z(point: *mut geometry_msgs::Point32, z: f32) {
    unsafe {
        assert!(!point.is_null());
        (*point).z = z;
    }
}

#[no_mangle]
pub extern "C" fn ros_point32_serialize(
    point: *const geometry_msgs::Point32,
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
pub extern "C" fn ros_point32_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut geometry_msgs::Point32 {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::Point32>(slice) {
            Ok(point) => Box::into_raw(Box::new(point)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::Pose
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_pose_new() -> *mut geometry_msgs::Pose {
    Box::into_raw(Box::new(geometry_msgs::Pose {
        position: geometry_msgs::Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        orientation: geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
    }))
}

#[no_mangle]
pub extern "C" fn ros_pose_free(pose: *mut geometry_msgs::Pose) {
    if !pose.is_null() {
        unsafe {
            drop(Box::from_raw(pose));
        }
    }
}

/// Returns a pointer to the position field. The returned pointer is owned by
/// the parent Pose and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_pose_get_position(
    pose: *const geometry_msgs::Pose,
) -> *const geometry_msgs::Point {
    unsafe {
        assert!(!pose.is_null());
        &(*pose).position
    }
}

/// Returns a mutable pointer to the position field for modification.
/// The returned pointer is owned by the parent Pose and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_pose_get_position_mut(
    pose: *mut geometry_msgs::Pose,
) -> *mut geometry_msgs::Point {
    unsafe {
        assert!(!pose.is_null());
        &mut (*pose).position
    }
}

/// Returns a pointer to the orientation field. The returned pointer is owned by
/// the parent Pose and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_pose_get_orientation(
    pose: *const geometry_msgs::Pose,
) -> *const geometry_msgs::Quaternion {
    unsafe {
        assert!(!pose.is_null());
        &(*pose).orientation
    }
}

/// Returns a mutable pointer to the orientation field for modification.
/// The returned pointer is owned by the parent Pose and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_pose_get_orientation_mut(
    pose: *mut geometry_msgs::Pose,
) -> *mut geometry_msgs::Quaternion {
    unsafe {
        assert!(!pose.is_null());
        &mut (*pose).orientation
    }
}

#[no_mangle]
pub extern "C" fn ros_pose_serialize(
    pose: *const geometry_msgs::Pose,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(pose);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*pose) {
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
pub extern "C" fn ros_pose_deserialize(bytes: *const u8, len: usize) -> *mut geometry_msgs::Pose {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::Pose>(slice) {
            Ok(pose) => Box::into_raw(Box::new(pose)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::Pose2D
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_pose2d_new() -> *mut geometry_msgs::Pose2D {
    Box::into_raw(Box::new(geometry_msgs::Pose2D {
        x: 0.0,
        y: 0.0,
        theta: 0.0,
    }))
}

#[no_mangle]
pub extern "C" fn ros_pose2d_free(pose: *mut geometry_msgs::Pose2D) {
    if !pose.is_null() {
        unsafe {
            drop(Box::from_raw(pose));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_pose2d_get_x(pose: *const geometry_msgs::Pose2D) -> f64 {
    unsafe {
        assert!(!pose.is_null());
        (*pose).x
    }
}

#[no_mangle]
pub extern "C" fn ros_pose2d_get_y(pose: *const geometry_msgs::Pose2D) -> f64 {
    unsafe {
        assert!(!pose.is_null());
        (*pose).y
    }
}

#[no_mangle]
pub extern "C" fn ros_pose2d_get_theta(pose: *const geometry_msgs::Pose2D) -> f64 {
    unsafe {
        assert!(!pose.is_null());
        (*pose).theta
    }
}

#[no_mangle]
pub extern "C" fn ros_pose2d_set_x(pose: *mut geometry_msgs::Pose2D, x: f64) {
    unsafe {
        assert!(!pose.is_null());
        (*pose).x = x;
    }
}

#[no_mangle]
pub extern "C" fn ros_pose2d_set_y(pose: *mut geometry_msgs::Pose2D, y: f64) {
    unsafe {
        assert!(!pose.is_null());
        (*pose).y = y;
    }
}

#[no_mangle]
pub extern "C" fn ros_pose2d_set_theta(pose: *mut geometry_msgs::Pose2D, theta: f64) {
    unsafe {
        assert!(!pose.is_null());
        (*pose).theta = theta;
    }
}

#[no_mangle]
pub extern "C" fn ros_pose2d_serialize(
    pose: *const geometry_msgs::Pose2D,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(pose);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*pose) {
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
pub extern "C" fn ros_pose2d_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut geometry_msgs::Pose2D {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::Pose2D>(slice) {
            Ok(pose) => Box::into_raw(Box::new(pose)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::Transform
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_transform_new() -> *mut geometry_msgs::Transform {
    Box::into_raw(Box::new(geometry_msgs::Transform {
        translation: geometry_msgs::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        rotation: geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
    }))
}

#[no_mangle]
pub extern "C" fn ros_transform_free(transform: *mut geometry_msgs::Transform) {
    if !transform.is_null() {
        unsafe {
            drop(Box::from_raw(transform));
        }
    }
}

/// Returns a pointer to the translation field. The returned pointer is owned by
/// the parent Transform and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_transform_get_translation(
    transform: *const geometry_msgs::Transform,
) -> *const geometry_msgs::Vector3 {
    unsafe {
        assert!(!transform.is_null());
        &(*transform).translation
    }
}

/// Returns a mutable pointer to the translation field for modification.
/// The returned pointer is owned by the parent Transform and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_transform_get_translation_mut(
    transform: *mut geometry_msgs::Transform,
) -> *mut geometry_msgs::Vector3 {
    unsafe {
        assert!(!transform.is_null());
        &mut (*transform).translation
    }
}

/// Returns a pointer to the rotation field. The returned pointer is owned by
/// the parent Transform and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_transform_get_rotation(
    transform: *const geometry_msgs::Transform,
) -> *const geometry_msgs::Quaternion {
    unsafe {
        assert!(!transform.is_null());
        &(*transform).rotation
    }
}

/// Returns a mutable pointer to the rotation field for modification.
/// The returned pointer is owned by the parent Transform and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_transform_get_rotation_mut(
    transform: *mut geometry_msgs::Transform,
) -> *mut geometry_msgs::Quaternion {
    unsafe {
        assert!(!transform.is_null());
        &mut (*transform).rotation
    }
}

#[no_mangle]
pub extern "C" fn ros_transform_serialize(
    transform: *const geometry_msgs::Transform,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(transform);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*transform) {
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
pub extern "C" fn ros_transform_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut geometry_msgs::Transform {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::Transform>(slice) {
            Ok(transform) => Box::into_raw(Box::new(transform)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::Twist
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_twist_new() -> *mut geometry_msgs::Twist {
    Box::into_raw(Box::new(geometry_msgs::Twist {
        linear: geometry_msgs::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        angular: geometry_msgs::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
    }))
}

#[no_mangle]
pub extern "C" fn ros_twist_free(twist: *mut geometry_msgs::Twist) {
    if !twist.is_null() {
        unsafe {
            drop(Box::from_raw(twist));
        }
    }
}

/// Returns a pointer to the linear velocity field. The returned pointer is owned by
/// the parent Twist and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_twist_get_linear(
    twist: *const geometry_msgs::Twist,
) -> *const geometry_msgs::Vector3 {
    unsafe {
        assert!(!twist.is_null());
        &(*twist).linear
    }
}

/// Returns a mutable pointer to the linear velocity field for modification.
/// The returned pointer is owned by the parent Twist and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_twist_get_linear_mut(
    twist: *mut geometry_msgs::Twist,
) -> *mut geometry_msgs::Vector3 {
    unsafe {
        assert!(!twist.is_null());
        &mut (*twist).linear
    }
}

/// Returns a pointer to the angular velocity field. The returned pointer is owned by
/// the parent Twist and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_twist_get_angular(
    twist: *const geometry_msgs::Twist,
) -> *const geometry_msgs::Vector3 {
    unsafe {
        assert!(!twist.is_null());
        &(*twist).angular
    }
}

/// Returns a mutable pointer to the angular velocity field for modification.
/// The returned pointer is owned by the parent Twist and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_twist_get_angular_mut(
    twist: *mut geometry_msgs::Twist,
) -> *mut geometry_msgs::Vector3 {
    unsafe {
        assert!(!twist.is_null());
        &mut (*twist).angular
    }
}

#[no_mangle]
pub extern "C" fn ros_twist_serialize(
    twist: *const geometry_msgs::Twist,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(twist);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*twist) {
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
pub extern "C" fn ros_twist_deserialize(bytes: *const u8, len: usize) -> *mut geometry_msgs::Twist {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::Twist>(slice) {
            Ok(twist) => Box::into_raw(Box::new(twist)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::Inertia
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_inertia_new() -> *mut geometry_msgs::Inertia {
    Box::into_raw(Box::new(geometry_msgs::Inertia {
        m: 0.0,
        com: geometry_msgs::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        ixx: 0.0,
        ixy: 0.0,
        ixz: 0.0,
        iyy: 0.0,
        iyz: 0.0,
        izz: 0.0,
    }))
}

#[no_mangle]
pub extern "C" fn ros_inertia_free(inertia: *mut geometry_msgs::Inertia) {
    if !inertia.is_null() {
        unsafe {
            drop(Box::from_raw(inertia));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_get_m(inertia: *const geometry_msgs::Inertia) -> f64 {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).m
    }
}

/// Returns a pointer to the center of mass field. The returned pointer is owned by
/// the parent Inertia and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_inertia_get_com(
    inertia: *const geometry_msgs::Inertia,
) -> *const geometry_msgs::Vector3 {
    unsafe {
        assert!(!inertia.is_null());
        &(*inertia).com
    }
}

/// Returns a mutable pointer to the center of mass field for modification.
/// The returned pointer is owned by the parent Inertia and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_inertia_get_com_mut(
    inertia: *mut geometry_msgs::Inertia,
) -> *mut geometry_msgs::Vector3 {
    unsafe {
        assert!(!inertia.is_null());
        &mut (*inertia).com
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_get_ixx(inertia: *const geometry_msgs::Inertia) -> f64 {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).ixx
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_get_ixy(inertia: *const geometry_msgs::Inertia) -> f64 {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).ixy
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_get_ixz(inertia: *const geometry_msgs::Inertia) -> f64 {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).ixz
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_get_iyy(inertia: *const geometry_msgs::Inertia) -> f64 {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).iyy
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_get_iyz(inertia: *const geometry_msgs::Inertia) -> f64 {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).iyz
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_get_izz(inertia: *const geometry_msgs::Inertia) -> f64 {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).izz
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_set_m(inertia: *mut geometry_msgs::Inertia, m: f64) {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).m = m;
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_set_ixx(inertia: *mut geometry_msgs::Inertia, ixx: f64) {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).ixx = ixx;
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_set_ixy(inertia: *mut geometry_msgs::Inertia, ixy: f64) {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).ixy = ixy;
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_set_ixz(inertia: *mut geometry_msgs::Inertia, ixz: f64) {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).ixz = ixz;
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_set_iyy(inertia: *mut geometry_msgs::Inertia, iyy: f64) {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).iyy = iyy;
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_set_iyz(inertia: *mut geometry_msgs::Inertia, iyz: f64) {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).iyz = iyz;
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_set_izz(inertia: *mut geometry_msgs::Inertia, izz: f64) {
    unsafe {
        assert!(!inertia.is_null());
        (*inertia).izz = izz;
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_serialize(
    inertia: *const geometry_msgs::Inertia,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(inertia);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*inertia) {
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
pub extern "C" fn ros_inertia_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut geometry_msgs::Inertia {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::Inertia>(slice) {
            Ok(inertia) => Box::into_raw(Box::new(inertia)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::InertiaStamped
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_inertia_stamped_new() -> *mut geometry_msgs::InertiaStamped {
    Box::into_raw(Box::new(geometry_msgs::InertiaStamped {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        inertia: geometry_msgs::Inertia {
            m: 0.0,
            com: geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            ixx: 0.0,
            ixy: 0.0,
            ixz: 0.0,
            iyy: 0.0,
            iyz: 0.0,
            izz: 0.0,
        },
    }))
}

#[no_mangle]
pub extern "C" fn ros_inertia_stamped_free(inertia: *mut geometry_msgs::InertiaStamped) {
    if !inertia.is_null() {
        unsafe {
            drop(Box::from_raw(inertia));
        }
    }
}

/// Returns a pointer to the header field. The returned pointer is owned by
/// the parent InertiaStamped and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_inertia_stamped_get_header(
    inertia: *const geometry_msgs::InertiaStamped,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!inertia.is_null());
        &(*inertia).header
    }
}

/// Returns a mutable pointer to the header field for modification.
/// The returned pointer is owned by the parent InertiaStamped and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_inertia_stamped_get_header_mut(
    inertia: *mut geometry_msgs::InertiaStamped,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!inertia.is_null());
        &mut (*inertia).header
    }
}

/// Returns a pointer to the inertia field. The returned pointer is owned by
/// the parent InertiaStamped and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_inertia_stamped_get_inertia(
    stamped: *const geometry_msgs::InertiaStamped,
) -> *const geometry_msgs::Inertia {
    unsafe {
        assert!(!stamped.is_null());
        &(*stamped).inertia
    }
}

/// Returns a mutable pointer to the inertia field for modification.
/// The returned pointer is owned by the parent InertiaStamped and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_inertia_stamped_get_inertia_mut(
    stamped: *mut geometry_msgs::InertiaStamped,
) -> *mut geometry_msgs::Inertia {
    unsafe {
        assert!(!stamped.is_null());
        &mut (*stamped).inertia
    }
}

#[no_mangle]
pub extern "C" fn ros_inertia_stamped_serialize(
    inertia: *const geometry_msgs::InertiaStamped,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(inertia);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*inertia) {
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
pub extern "C" fn ros_inertia_stamped_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut geometry_msgs::InertiaStamped {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::InertiaStamped>(slice) {
            Ok(inertia) => Box::into_raw(Box::new(inertia)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// sensor_msgs::RegionOfInterest
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_region_of_interest_new() -> *mut sensor_msgs::RegionOfInterest {
    Box::into_raw(Box::new(sensor_msgs::RegionOfInterest {
        x_offset: 0,
        y_offset: 0,
        height: 0,
        width: 0,
        do_rectify: false,
    }))
}

#[no_mangle]
pub extern "C" fn ros_region_of_interest_free(roi: *mut sensor_msgs::RegionOfInterest) {
    if !roi.is_null() {
        unsafe {
            drop(Box::from_raw(roi));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_region_of_interest_get_x_offset(
    roi: *const sensor_msgs::RegionOfInterest,
) -> u32 {
    unsafe {
        assert!(!roi.is_null());
        (*roi).x_offset
    }
}

#[no_mangle]
pub extern "C" fn ros_region_of_interest_get_y_offset(
    roi: *const sensor_msgs::RegionOfInterest,
) -> u32 {
    unsafe {
        assert!(!roi.is_null());
        (*roi).y_offset
    }
}

#[no_mangle]
pub extern "C" fn ros_region_of_interest_get_height(
    roi: *const sensor_msgs::RegionOfInterest,
) -> u32 {
    unsafe {
        assert!(!roi.is_null());
        (*roi).height
    }
}

#[no_mangle]
pub extern "C" fn ros_region_of_interest_get_width(
    roi: *const sensor_msgs::RegionOfInterest,
) -> u32 {
    unsafe {
        assert!(!roi.is_null());
        (*roi).width
    }
}

#[no_mangle]
pub extern "C" fn ros_region_of_interest_get_do_rectify(
    roi: *const sensor_msgs::RegionOfInterest,
) -> bool {
    unsafe {
        assert!(!roi.is_null());
        (*roi).do_rectify
    }
}

#[no_mangle]
pub extern "C" fn ros_region_of_interest_set_x_offset(
    roi: *mut sensor_msgs::RegionOfInterest,
    x_offset: u32,
) {
    unsafe {
        assert!(!roi.is_null());
        (*roi).x_offset = x_offset;
    }
}

#[no_mangle]
pub extern "C" fn ros_region_of_interest_set_y_offset(
    roi: *mut sensor_msgs::RegionOfInterest,
    y_offset: u32,
) {
    unsafe {
        assert!(!roi.is_null());
        (*roi).y_offset = y_offset;
    }
}

#[no_mangle]
pub extern "C" fn ros_region_of_interest_set_height(
    roi: *mut sensor_msgs::RegionOfInterest,
    height: u32,
) {
    unsafe {
        assert!(!roi.is_null());
        (*roi).height = height;
    }
}

#[no_mangle]
pub extern "C" fn ros_region_of_interest_set_width(
    roi: *mut sensor_msgs::RegionOfInterest,
    width: u32,
) {
    unsafe {
        assert!(!roi.is_null());
        (*roi).width = width;
    }
}

#[no_mangle]
pub extern "C" fn ros_region_of_interest_set_do_rectify(
    roi: *mut sensor_msgs::RegionOfInterest,
    do_rectify: bool,
) {
    unsafe {
        assert!(!roi.is_null());
        (*roi).do_rectify = do_rectify;
    }
}

#[no_mangle]
pub extern "C" fn ros_region_of_interest_serialize(
    roi: *const sensor_msgs::RegionOfInterest,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(roi);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*roi) {
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
pub extern "C" fn ros_region_of_interest_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut sensor_msgs::RegionOfInterest {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<sensor_msgs::RegionOfInterest>(slice) {
            Ok(roi) => Box::into_raw(Box::new(roi)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// sensor_msgs::CompressedImage
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_compressed_image_new() -> *mut sensor_msgs::CompressedImage {
    Box::into_raw(Box::new(sensor_msgs::CompressedImage {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        format: String::new(),
        data: Vec::new(),
    }))
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_free(image: *mut sensor_msgs::CompressedImage) {
    if !image.is_null() {
        unsafe {
            drop(Box::from_raw(image));
        }
    }
}

/// Returns a pointer to the header field. The returned pointer is owned by
/// the parent CompressedImage and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_compressed_image_get_header(
    image: *const sensor_msgs::CompressedImage,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!image.is_null());
        &(*image).header
    }
}

/// Returns a mutable pointer to the header field for modification.
/// The returned pointer is owned by the parent CompressedImage and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_compressed_image_get_header_mut(
    image: *mut sensor_msgs::CompressedImage,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!image.is_null());
        &mut (*image).header
    }
}

/// Returns the format string. Caller owns the returned string and must free it.
#[no_mangle]
pub extern "C" fn ros_compressed_image_get_format(
    image: *const sensor_msgs::CompressedImage,
) -> *mut c_char {
    unsafe {
        assert!(!image.is_null());
        string_to_c_char(&(*image).format)
    }
}

/// Returns a pointer to the image data and sets the length.
/// The returned pointer is owned by the parent CompressedImage and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_compressed_image_get_data(
    image: *const sensor_msgs::CompressedImage,
    out_len: *mut usize,
) -> *const u8 {
    if image.is_null() {
        if !out_len.is_null() {
            unsafe { *out_len = 0 };
        }
        return ptr::null();
    }
    unsafe {
        if !out_len.is_null() {
            *out_len = (*image).data.len();
        }
        (*image).data.as_ptr()
    }
}

/// Sets the format string. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn ros_compressed_image_set_format(
    image: *mut sensor_msgs::CompressedImage,
    format: *const c_char,
) -> i32 {
    check_null!(image);
    check_null!(format);

    unsafe {
        match c_char_to_string(format) {
            Some(s) => {
                (*image).format = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

/// Sets the image data. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn ros_compressed_image_set_data(
    image: *mut sensor_msgs::CompressedImage,
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
pub extern "C" fn ros_compressed_image_serialize(
    image: *const sensor_msgs::CompressedImage,
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
pub extern "C" fn ros_compressed_image_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut sensor_msgs::CompressedImage {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<sensor_msgs::CompressedImage>(slice) {
            Ok(image) => Box::into_raw(Box::new(image)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// sensor_msgs::IMU
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_imu_new() -> *mut sensor_msgs::IMU {
    Box::into_raw(Box::new(sensor_msgs::IMU {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        orientation: geometry_msgs::Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
        orientation_covariance: [0.0; 9],
        angular_velocity: geometry_msgs::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: geometry_msgs::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        linear_acceleration_covariance: [0.0; 9],
    }))
}

#[no_mangle]
pub extern "C" fn ros_imu_free(imu: *mut sensor_msgs::IMU) {
    if !imu.is_null() {
        unsafe {
            drop(Box::from_raw(imu));
        }
    }
}

/// Returns a pointer to the header field. The returned pointer is owned by
/// the parent IMU and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_imu_get_header(imu: *const sensor_msgs::IMU) -> *const std_msgs::Header {
    unsafe {
        assert!(!imu.is_null());
        &(*imu).header
    }
}

/// Returns a mutable pointer to the header field for modification.
/// The returned pointer is owned by the parent IMU and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_imu_get_header_mut(imu: *mut sensor_msgs::IMU) -> *mut std_msgs::Header {
    unsafe {
        assert!(!imu.is_null());
        &mut (*imu).header
    }
}

/// Returns a pointer to the orientation quaternion. The returned pointer is owned by
/// the parent IMU and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_imu_get_orientation(
    imu: *const sensor_msgs::IMU,
) -> *const geometry_msgs::Quaternion {
    unsafe {
        assert!(!imu.is_null());
        &(*imu).orientation
    }
}

/// Returns a mutable pointer to the orientation quaternion for modification.
/// The returned pointer is owned by the parent IMU and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_imu_get_orientation_mut(
    imu: *mut sensor_msgs::IMU,
) -> *mut geometry_msgs::Quaternion {
    unsafe {
        assert!(!imu.is_null());
        &mut (*imu).orientation
    }
}

/// Returns a pointer to the orientation covariance array (9 elements).
/// The returned pointer is owned by the parent IMU and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_imu_get_orientation_covariance(imu: *const sensor_msgs::IMU) -> *const f64 {
    unsafe {
        assert!(!imu.is_null());
        (*imu).orientation_covariance.as_ptr()
    }
}

/// Sets the orientation covariance array (must point to 9 f64 values).
/// Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn ros_imu_set_orientation_covariance(
    imu: *mut sensor_msgs::IMU,
    covariance: *const f64,
) -> i32 {
    check_null!(imu);
    check_null!(covariance);

    unsafe {
        let slice = slice::from_raw_parts(covariance, 9);
        (*imu).orientation_covariance.copy_from_slice(slice);
        0
    }
}

/// Returns a pointer to the angular velocity vector. The returned pointer is owned by
/// the parent IMU and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_imu_get_angular_velocity(
    imu: *const sensor_msgs::IMU,
) -> *const geometry_msgs::Vector3 {
    unsafe {
        assert!(!imu.is_null());
        &(*imu).angular_velocity
    }
}

/// Returns a mutable pointer to the angular velocity vector for modification.
/// The returned pointer is owned by the parent IMU and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_imu_get_angular_velocity_mut(
    imu: *mut sensor_msgs::IMU,
) -> *mut geometry_msgs::Vector3 {
    unsafe {
        assert!(!imu.is_null());
        &mut (*imu).angular_velocity
    }
}

/// Returns a pointer to the angular velocity covariance array (9 elements).
/// The returned pointer is owned by the parent IMU and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_imu_get_angular_velocity_covariance(
    imu: *const sensor_msgs::IMU,
) -> *const f64 {
    unsafe {
        assert!(!imu.is_null());
        (*imu).angular_velocity_covariance.as_ptr()
    }
}

/// Sets the angular velocity covariance array (must point to 9 f64 values).
/// Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn ros_imu_set_angular_velocity_covariance(
    imu: *mut sensor_msgs::IMU,
    covariance: *const f64,
) -> i32 {
    check_null!(imu);
    check_null!(covariance);

    unsafe {
        let slice = slice::from_raw_parts(covariance, 9);
        (*imu).angular_velocity_covariance.copy_from_slice(slice);
        0
    }
}

/// Returns a pointer to the linear acceleration vector. The returned pointer is owned by
/// the parent IMU and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_imu_get_linear_acceleration(
    imu: *const sensor_msgs::IMU,
) -> *const geometry_msgs::Vector3 {
    unsafe {
        assert!(!imu.is_null());
        &(*imu).linear_acceleration
    }
}

/// Returns a mutable pointer to the linear acceleration vector for modification.
/// The returned pointer is owned by the parent IMU and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_imu_get_linear_acceleration_mut(
    imu: *mut sensor_msgs::IMU,
) -> *mut geometry_msgs::Vector3 {
    unsafe {
        assert!(!imu.is_null());
        &mut (*imu).linear_acceleration
    }
}

/// Returns a pointer to the linear acceleration covariance array (9 elements).
/// The returned pointer is owned by the parent IMU and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_imu_get_linear_acceleration_covariance(
    imu: *const sensor_msgs::IMU,
) -> *const f64 {
    unsafe {
        assert!(!imu.is_null());
        (*imu).linear_acceleration_covariance.as_ptr()
    }
}

/// Sets the linear acceleration covariance array (must point to 9 f64 values).
/// Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn ros_imu_set_linear_acceleration_covariance(
    imu: *mut sensor_msgs::IMU,
    covariance: *const f64,
) -> i32 {
    check_null!(imu);
    check_null!(covariance);

    unsafe {
        let slice = slice::from_raw_parts(covariance, 9);
        (*imu).linear_acceleration_covariance.copy_from_slice(slice);
        0
    }
}

#[no_mangle]
pub extern "C" fn ros_imu_serialize(
    imu: *const sensor_msgs::IMU,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(imu);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*imu) {
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
pub extern "C" fn ros_imu_deserialize(bytes: *const u8, len: usize) -> *mut sensor_msgs::IMU {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<sensor_msgs::IMU>(slice) {
            Ok(imu) => Box::into_raw(Box::new(imu)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// sensor_msgs::CameraInfo
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_camera_info_new() -> *mut sensor_msgs::CameraInfo {
    Box::into_raw(Box::new(sensor_msgs::CameraInfo {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        height: 0,
        width: 0,
        distortion_model: String::new(),
        d: Vec::new(),
        k: [0.0; 9],
        r: [0.0; 9],
        p: [0.0; 12],
        binning_x: 0,
        binning_y: 0,
        roi: sensor_msgs::RegionOfInterest {
            x_offset: 0,
            y_offset: 0,
            height: 0,
            width: 0,
            do_rectify: false,
        },
    }))
}

#[no_mangle]
pub extern "C" fn ros_camera_info_free(info: *mut sensor_msgs::CameraInfo) {
    if !info.is_null() {
        unsafe {
            drop(Box::from_raw(info));
        }
    }
}

/// Returns a pointer to the header field. The returned pointer is owned by
/// the parent CameraInfo and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_camera_info_get_header(
    info: *const sensor_msgs::CameraInfo,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!info.is_null());
        &(*info).header
    }
}

/// Returns a mutable pointer to the header field for modification.
/// The returned pointer is owned by the parent CameraInfo and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_camera_info_get_header_mut(
    info: *mut sensor_msgs::CameraInfo,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!info.is_null());
        &mut (*info).header
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_get_height(info: *const sensor_msgs::CameraInfo) -> u32 {
    unsafe {
        assert!(!info.is_null());
        (*info).height
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_get_width(info: *const sensor_msgs::CameraInfo) -> u32 {
    unsafe {
        assert!(!info.is_null());
        (*info).width
    }
}

/// Returns the distortion model string. Caller owns the returned string and must free it.
#[no_mangle]
pub extern "C" fn ros_camera_info_get_distortion_model(
    info: *const sensor_msgs::CameraInfo,
) -> *mut c_char {
    unsafe {
        assert!(!info.is_null());
        string_to_c_char(&(*info).distortion_model)
    }
}

/// Returns a pointer to the distortion coefficients array and sets the length.
/// The returned pointer is owned by the parent CameraInfo and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_camera_info_get_d(
    info: *const sensor_msgs::CameraInfo,
    out_len: *mut usize,
) -> *const f64 {
    if info.is_null() {
        if !out_len.is_null() {
            unsafe { *out_len = 0 };
        }
        return ptr::null();
    }
    unsafe {
        if !out_len.is_null() {
            *out_len = (*info).d.len();
        }
        (*info).d.as_ptr()
    }
}

/// Returns a pointer to the intrinsic camera matrix K (9 elements, row-major).
/// The returned pointer is owned by the parent CameraInfo and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_camera_info_get_k(info: *const sensor_msgs::CameraInfo) -> *const f64 {
    unsafe {
        assert!(!info.is_null());
        (*info).k.as_ptr()
    }
}

/// Returns a pointer to the rectification matrix R (9 elements, row-major).
/// The returned pointer is owned by the parent CameraInfo and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_camera_info_get_r(info: *const sensor_msgs::CameraInfo) -> *const f64 {
    unsafe {
        assert!(!info.is_null());
        (*info).r.as_ptr()
    }
}

/// Returns a pointer to the projection matrix P (12 elements, row-major).
/// The returned pointer is owned by the parent CameraInfo and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_camera_info_get_p(info: *const sensor_msgs::CameraInfo) -> *const f64 {
    unsafe {
        assert!(!info.is_null());
        (*info).p.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_get_binning_x(info: *const sensor_msgs::CameraInfo) -> u32 {
    unsafe {
        assert!(!info.is_null());
        (*info).binning_x
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_get_binning_y(info: *const sensor_msgs::CameraInfo) -> u32 {
    unsafe {
        assert!(!info.is_null());
        (*info).binning_y
    }
}

/// Returns a pointer to the region of interest. The returned pointer is owned by
/// the parent CameraInfo and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_camera_info_get_roi(
    info: *const sensor_msgs::CameraInfo,
) -> *const sensor_msgs::RegionOfInterest {
    unsafe {
        assert!(!info.is_null());
        &(*info).roi
    }
}

/// Returns a mutable pointer to the region of interest for modification.
/// The returned pointer is owned by the parent CameraInfo and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_camera_info_get_roi_mut(
    info: *mut sensor_msgs::CameraInfo,
) -> *mut sensor_msgs::RegionOfInterest {
    unsafe {
        assert!(!info.is_null());
        &mut (*info).roi
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_set_height(info: *mut sensor_msgs::CameraInfo, height: u32) {
    unsafe {
        assert!(!info.is_null());
        (*info).height = height;
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_set_width(info: *mut sensor_msgs::CameraInfo, width: u32) {
    unsafe {
        assert!(!info.is_null());
        (*info).width = width;
    }
}

/// Sets the distortion model string. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn ros_camera_info_set_distortion_model(
    info: *mut sensor_msgs::CameraInfo,
    model: *const c_char,
) -> i32 {
    check_null!(info);
    check_null!(model);

    unsafe {
        match c_char_to_string(model) {
            Some(s) => {
                (*info).distortion_model = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

/// Sets the distortion coefficients array. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn ros_camera_info_set_d(
    info: *mut sensor_msgs::CameraInfo,
    d: *const f64,
    len: usize,
) -> i32 {
    check_null!(info);
    check_null!(d);

    unsafe {
        let slice = slice::from_raw_parts(d, len);
        (*info).d = slice.to_vec();
        0
    }
}

/// Sets the intrinsic camera matrix K (must point to 9 f64 values).
/// Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn ros_camera_info_set_k(info: *mut sensor_msgs::CameraInfo, k: *const f64) -> i32 {
    check_null!(info);
    check_null!(k);

    unsafe {
        let slice = slice::from_raw_parts(k, 9);
        (*info).k.copy_from_slice(slice);
        0
    }
}

/// Sets the rectification matrix R (must point to 9 f64 values).
/// Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn ros_camera_info_set_r(info: *mut sensor_msgs::CameraInfo, r: *const f64) -> i32 {
    check_null!(info);
    check_null!(r);

    unsafe {
        let slice = slice::from_raw_parts(r, 9);
        (*info).r.copy_from_slice(slice);
        0
    }
}

/// Sets the projection matrix P (must point to 12 f64 values).
/// Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn ros_camera_info_set_p(info: *mut sensor_msgs::CameraInfo, p: *const f64) -> i32 {
    check_null!(info);
    check_null!(p);

    unsafe {
        let slice = slice::from_raw_parts(p, 12);
        (*info).p.copy_from_slice(slice);
        0
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_set_binning_x(
    info: *mut sensor_msgs::CameraInfo,
    binning_x: u32,
) {
    unsafe {
        assert!(!info.is_null());
        (*info).binning_x = binning_x;
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_set_binning_y(
    info: *mut sensor_msgs::CameraInfo,
    binning_y: u32,
) {
    unsafe {
        assert!(!info.is_null());
        (*info).binning_y = binning_y;
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_serialize(
    info: *const sensor_msgs::CameraInfo,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(info);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*info) {
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
pub extern "C" fn ros_camera_info_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut sensor_msgs::CameraInfo {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<sensor_msgs::CameraInfo>(slice) {
            Ok(info) => Box::into_raw(Box::new(info)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// edgefirst_msgs::Date
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_date_new() -> *mut edgefirst_msgs::Date {
    Box::into_raw(Box::new(edgefirst_msgs::Date {
        year: 0,
        month: 0,
        day: 0,
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_date_free(date: *mut edgefirst_msgs::Date) {
    if !date.is_null() {
        unsafe {
            drop(Box::from_raw(date));
        }
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_date_get_year(date: *const edgefirst_msgs::Date) -> u16 {
    unsafe {
        assert!(!date.is_null());
        (*date).year
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_date_get_month(date: *const edgefirst_msgs::Date) -> u8 {
    unsafe {
        assert!(!date.is_null());
        (*date).month
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_date_get_day(date: *const edgefirst_msgs::Date) -> u8 {
    unsafe {
        assert!(!date.is_null());
        (*date).day
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_date_set_year(date: *mut edgefirst_msgs::Date, year: u16) {
    unsafe {
        assert!(!date.is_null());
        (*date).year = year;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_date_set_month(date: *mut edgefirst_msgs::Date, month: u8) {
    unsafe {
        assert!(!date.is_null());
        (*date).month = month;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_date_set_day(date: *mut edgefirst_msgs::Date, day: u8) {
    unsafe {
        assert!(!date.is_null());
        (*date).day = day;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_date_serialize(
    date: *const edgefirst_msgs::Date,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(date);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*date) {
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
pub extern "C" fn edgefirst_date_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::Date {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::Date>(slice) {
            Ok(date) => Box::into_raw(Box::new(date)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// edgefirst_msgs::LocalTime
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_local_time_new() -> *mut edgefirst_msgs::LocalTime {
    Box::into_raw(Box::new(edgefirst_msgs::LocalTime {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        date: edgefirst_msgs::Date {
            year: 0,
            month: 0,
            day: 0,
        },
        time: builtin_interfaces::Time { sec: 0, nanosec: 0 },
        timezone: 0,
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_local_time_free(local_time: *mut edgefirst_msgs::LocalTime) {
    if !local_time.is_null() {
        unsafe {
            drop(Box::from_raw(local_time));
        }
    }
}

/// Returns a pointer to the header field. The returned pointer is owned by
/// the parent LocalTime and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn edgefirst_local_time_get_header(
    local_time: *const edgefirst_msgs::LocalTime,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!local_time.is_null());
        &(*local_time).header
    }
}

/// Returns a mutable pointer to the header field for modification.
/// The returned pointer is owned by the parent LocalTime and must NOT be freed.
#[no_mangle]
pub extern "C" fn edgefirst_local_time_get_header_mut(
    local_time: *mut edgefirst_msgs::LocalTime,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!local_time.is_null());
        &mut (*local_time).header
    }
}

/// Returns a pointer to the date field. The returned pointer is owned by
/// the parent LocalTime and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn edgefirst_local_time_get_date(
    local_time: *const edgefirst_msgs::LocalTime,
) -> *const edgefirst_msgs::Date {
    unsafe {
        assert!(!local_time.is_null());
        &(*local_time).date
    }
}

/// Returns a mutable pointer to the date field for modification.
/// The returned pointer is owned by the parent LocalTime and must NOT be freed.
#[no_mangle]
pub extern "C" fn edgefirst_local_time_get_date_mut(
    local_time: *mut edgefirst_msgs::LocalTime,
) -> *mut edgefirst_msgs::Date {
    unsafe {
        assert!(!local_time.is_null());
        &mut (*local_time).date
    }
}

/// Returns a pointer to the time field. The returned pointer is owned by
/// the parent LocalTime and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn edgefirst_local_time_get_time(
    local_time: *const edgefirst_msgs::LocalTime,
) -> *const builtin_interfaces::Time {
    unsafe {
        assert!(!local_time.is_null());
        &(*local_time).time
    }
}

/// Returns a mutable pointer to the time field for modification.
/// The returned pointer is owned by the parent LocalTime and must NOT be freed.
#[no_mangle]
pub extern "C" fn edgefirst_local_time_get_time_mut(
    local_time: *mut edgefirst_msgs::LocalTime,
) -> *mut builtin_interfaces::Time {
    unsafe {
        assert!(!local_time.is_null());
        &mut (*local_time).time
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_local_time_get_timezone(
    local_time: *const edgefirst_msgs::LocalTime,
) -> i16 {
    unsafe {
        assert!(!local_time.is_null());
        (*local_time).timezone
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_local_time_set_timezone(
    local_time: *mut edgefirst_msgs::LocalTime,
    timezone: i16,
) {
    unsafe {
        assert!(!local_time.is_null());
        (*local_time).timezone = timezone;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_local_time_serialize(
    local_time: *const edgefirst_msgs::LocalTime,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(local_time);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*local_time) {
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
pub extern "C" fn edgefirst_local_time_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::LocalTime {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::LocalTime>(slice) {
            Ok(local_time) => Box::into_raw(Box::new(local_time)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// edgefirst_msgs::RadarInfo
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_radar_info_new() -> *mut edgefirst_msgs::RadarInfo {
    Box::into_raw(Box::new(edgefirst_msgs::RadarInfo {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        center_frequency: String::new(),
        frequency_sweep: String::new(),
        range_toggle: String::new(),
        detection_sensitivity: String::new(),
        cube: false,
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_radar_info_free(info: *mut edgefirst_msgs::RadarInfo) {
    if !info.is_null() {
        unsafe {
            drop(Box::from_raw(info));
        }
    }
}

/// Returns a pointer to the header field. The returned pointer is owned by
/// the parent RadarInfo and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn edgefirst_radar_info_get_header(
    info: *const edgefirst_msgs::RadarInfo,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!info.is_null());
        &(*info).header
    }
}

/// Returns a mutable pointer to the header field for modification.
/// The returned pointer is owned by the parent RadarInfo and must NOT be freed.
#[no_mangle]
pub extern "C" fn edgefirst_radar_info_get_header_mut(
    info: *mut edgefirst_msgs::RadarInfo,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!info.is_null());
        &mut (*info).header
    }
}

/// Returns the center frequency string. Caller owns the returned string and must free it.
#[no_mangle]
pub extern "C" fn edgefirst_radar_info_get_center_frequency(
    info: *const edgefirst_msgs::RadarInfo,
) -> *mut c_char {
    unsafe {
        assert!(!info.is_null());
        string_to_c_char(&(*info).center_frequency)
    }
}

/// Returns the frequency sweep string. Caller owns the returned string and must free it.
#[no_mangle]
pub extern "C" fn edgefirst_radar_info_get_frequency_sweep(
    info: *const edgefirst_msgs::RadarInfo,
) -> *mut c_char {
    unsafe {
        assert!(!info.is_null());
        string_to_c_char(&(*info).frequency_sweep)
    }
}

/// Returns the range toggle string. Caller owns the returned string and must free it.
#[no_mangle]
pub extern "C" fn edgefirst_radar_info_get_range_toggle(
    info: *const edgefirst_msgs::RadarInfo,
) -> *mut c_char {
    unsafe {
        assert!(!info.is_null());
        string_to_c_char(&(*info).range_toggle)
    }
}

/// Returns the detection sensitivity string. Caller owns the returned string and must free it.
#[no_mangle]
pub extern "C" fn edgefirst_radar_info_get_detection_sensitivity(
    info: *const edgefirst_msgs::RadarInfo,
) -> *mut c_char {
    unsafe {
        assert!(!info.is_null());
        string_to_c_char(&(*info).detection_sensitivity)
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radar_info_get_cube(info: *const edgefirst_msgs::RadarInfo) -> bool {
    unsafe {
        assert!(!info.is_null());
        (*info).cube
    }
}

/// Sets the center frequency string. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn edgefirst_radar_info_set_center_frequency(
    info: *mut edgefirst_msgs::RadarInfo,
    center_frequency: *const c_char,
) -> i32 {
    check_null!(info);
    check_null!(center_frequency);

    unsafe {
        match c_char_to_string(center_frequency) {
            Some(s) => {
                (*info).center_frequency = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

/// Sets the frequency sweep string. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn edgefirst_radar_info_set_frequency_sweep(
    info: *mut edgefirst_msgs::RadarInfo,
    frequency_sweep: *const c_char,
) -> i32 {
    check_null!(info);
    check_null!(frequency_sweep);

    unsafe {
        match c_char_to_string(frequency_sweep) {
            Some(s) => {
                (*info).frequency_sweep = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

/// Sets the range toggle string. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn edgefirst_radar_info_set_range_toggle(
    info: *mut edgefirst_msgs::RadarInfo,
    range_toggle: *const c_char,
) -> i32 {
    check_null!(info);
    check_null!(range_toggle);

    unsafe {
        match c_char_to_string(range_toggle) {
            Some(s) => {
                (*info).range_toggle = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

/// Sets the detection sensitivity string. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn edgefirst_radar_info_set_detection_sensitivity(
    info: *mut edgefirst_msgs::RadarInfo,
    detection_sensitivity: *const c_char,
) -> i32 {
    check_null!(info);
    check_null!(detection_sensitivity);

    unsafe {
        match c_char_to_string(detection_sensitivity) {
            Some(s) => {
                (*info).detection_sensitivity = s;
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
pub extern "C" fn edgefirst_radar_info_set_cube(info: *mut edgefirst_msgs::RadarInfo, cube: bool) {
    unsafe {
        assert!(!info.is_null());
        (*info).cube = cube;
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_radar_info_serialize(
    info: *const edgefirst_msgs::RadarInfo,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(info);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*info) {
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
pub extern "C" fn edgefirst_radar_info_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::RadarInfo {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::RadarInfo>(slice) {
            Ok(info) => Box::into_raw(Box::new(info)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// edgefirst_msgs::Model
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_model_new() -> *mut edgefirst_msgs::Model {
    Box::into_raw(Box::new(edgefirst_msgs::Model {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        input_time: builtin_interfaces::Duration { sec: 0, nanosec: 0 },
        model_time: builtin_interfaces::Duration { sec: 0, nanosec: 0 },
        output_time: builtin_interfaces::Duration { sec: 0, nanosec: 0 },
        decode_time: builtin_interfaces::Duration { sec: 0, nanosec: 0 },
        boxes: Vec::new(),
        masks: Vec::new(),
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_model_free(model: *mut edgefirst_msgs::Model) {
    if !model.is_null() {
        unsafe {
            drop(Box::from_raw(model));
        }
    }
}

/// Returns a pointer to the header field. The returned pointer is owned by
/// the parent Model and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_header(
    model: *const edgefirst_msgs::Model,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!model.is_null());
        &(*model).header
    }
}

/// Returns a mutable pointer to the header field for modification.
/// The returned pointer is owned by the parent Model and must NOT be freed.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_header_mut(
    model: *mut edgefirst_msgs::Model,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!model.is_null());
        &mut (*model).header
    }
}

/// Returns a pointer to the input_time duration. The returned pointer is owned by
/// the parent Model and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_input_time(
    model: *const edgefirst_msgs::Model,
) -> *const builtin_interfaces::Duration {
    unsafe {
        assert!(!model.is_null());
        &(*model).input_time
    }
}

/// Returns a mutable pointer to the input_time duration for modification.
/// The returned pointer is owned by the parent Model and must NOT be freed.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_input_time_mut(
    model: *mut edgefirst_msgs::Model,
) -> *mut builtin_interfaces::Duration {
    unsafe {
        assert!(!model.is_null());
        &mut (*model).input_time
    }
}

/// Returns a pointer to the model_time duration. The returned pointer is owned by
/// the parent Model and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_model_time(
    model: *const edgefirst_msgs::Model,
) -> *const builtin_interfaces::Duration {
    unsafe {
        assert!(!model.is_null());
        &(*model).model_time
    }
}

/// Returns a mutable pointer to the model_time duration for modification.
/// The returned pointer is owned by the parent Model and must NOT be freed.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_model_time_mut(
    model: *mut edgefirst_msgs::Model,
) -> *mut builtin_interfaces::Duration {
    unsafe {
        assert!(!model.is_null());
        &mut (*model).model_time
    }
}

/// Returns a pointer to the output_time duration. The returned pointer is owned by
/// the parent Model and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_output_time(
    model: *const edgefirst_msgs::Model,
) -> *const builtin_interfaces::Duration {
    unsafe {
        assert!(!model.is_null());
        &(*model).output_time
    }
}

/// Returns a mutable pointer to the output_time duration for modification.
/// The returned pointer is owned by the parent Model and must NOT be freed.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_output_time_mut(
    model: *mut edgefirst_msgs::Model,
) -> *mut builtin_interfaces::Duration {
    unsafe {
        assert!(!model.is_null());
        &mut (*model).output_time
    }
}

/// Returns a pointer to the decode_time duration. The returned pointer is owned by
/// the parent Model and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_decode_time(
    model: *const edgefirst_msgs::Model,
) -> *const builtin_interfaces::Duration {
    unsafe {
        assert!(!model.is_null());
        &(*model).decode_time
    }
}

/// Returns a mutable pointer to the decode_time duration for modification.
/// The returned pointer is owned by the parent Model and must NOT be freed.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_decode_time_mut(
    model: *mut edgefirst_msgs::Model,
) -> *mut builtin_interfaces::Duration {
    unsafe {
        assert!(!model.is_null());
        &mut (*model).decode_time
    }
}

/// Returns a pointer to the box at the given index. The returned pointer is owned by
/// the parent Model and must NOT be freed by the caller.
/// Returns NULL if index is out of bounds.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_box(
    model: *const edgefirst_msgs::Model,
    index: usize,
) -> *const edgefirst_msgs::Box {
    unsafe {
        assert!(!model.is_null());
        match (&(*model).boxes).get(index) {
            Some(box2d) => box2d,
            None => ptr::null(),
        }
    }
}

/// Returns the number of detection boxes.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_boxes_count(model: *const edgefirst_msgs::Model) -> usize {
    unsafe {
        assert!(!model.is_null());
        (*model).boxes.len()
    }
}

/// Adds a copy of the given box to the boxes vector. Returns 0 on success.
#[no_mangle]
pub extern "C" fn edgefirst_model_add_box(
    model: *mut edgefirst_msgs::Model,
    box2d: *const edgefirst_msgs::Box,
) -> i32 {
    check_null!(model);
    check_null!(box2d);

    unsafe {
        (*model).boxes.push((*box2d).clone());
        0
    }
}

/// Clears all detection boxes.
#[no_mangle]
pub extern "C" fn edgefirst_model_clear_boxes(model: *mut edgefirst_msgs::Model) {
    unsafe {
        assert!(!model.is_null());
        (*model).boxes.clear();
    }
}

/// Returns a pointer to the mask at the given index. The returned pointer is owned by
/// the parent Model and must NOT be freed by the caller.
/// Returns NULL if index is out of bounds.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_mask(
    model: *const edgefirst_msgs::Model,
    index: usize,
) -> *const edgefirst_msgs::Mask {
    unsafe {
        assert!(!model.is_null());
        match (&(*model).masks).get(index) {
            Some(mask) => mask,
            None => ptr::null(),
        }
    }
}

/// Returns the number of masks.
#[no_mangle]
pub extern "C" fn edgefirst_model_get_masks_count(model: *const edgefirst_msgs::Model) -> usize {
    unsafe {
        assert!(!model.is_null());
        (*model).masks.len()
    }
}

/// Adds a copy of the given mask to the masks vector. Returns 0 on success.
#[no_mangle]
pub extern "C" fn edgefirst_model_add_mask(
    model: *mut edgefirst_msgs::Model,
    mask: *const edgefirst_msgs::Mask,
) -> i32 {
    check_null!(model);
    check_null!(mask);

    unsafe {
        (*model).masks.push((*mask).clone());
        0
    }
}

/// Clears all masks.
#[no_mangle]
pub extern "C" fn edgefirst_model_clear_masks(model: *mut edgefirst_msgs::Model) {
    unsafe {
        assert!(!model.is_null());
        (*model).masks.clear();
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_model_serialize(
    model: *const edgefirst_msgs::Model,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(model);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*model) {
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
pub extern "C" fn edgefirst_model_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::Model {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::Model>(slice) {
            Ok(model) => Box::into_raw(Box::new(model)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// edgefirst_msgs::ModelInfo
// =============================================================================

#[no_mangle]
pub extern "C" fn edgefirst_model_info_new() -> *mut edgefirst_msgs::ModelInfo {
    Box::into_raw(Box::new(edgefirst_msgs::ModelInfo {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        input_shape: Vec::new(),
        input_type: 0,
        output_shape: Vec::new(),
        output_type: 0,
        labels: Vec::new(),
        model_type: String::new(),
        model_format: String::new(),
        model_name: String::new(),
    }))
}

#[no_mangle]
pub extern "C" fn edgefirst_model_info_free(info: *mut edgefirst_msgs::ModelInfo) {
    if !info.is_null() {
        unsafe {
            drop(Box::from_raw(info));
        }
    }
}

/// Returns a pointer to the header field. The returned pointer is owned by
/// the parent ModelInfo and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_get_header(
    info: *const edgefirst_msgs::ModelInfo,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!info.is_null());
        &(*info).header
    }
}

/// Returns a mutable pointer to the header field for modification.
/// The returned pointer is owned by the parent ModelInfo and must NOT be freed.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_get_header_mut(
    info: *mut edgefirst_msgs::ModelInfo,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!info.is_null());
        &mut (*info).header
    }
}

/// Returns a pointer to the input shape array and sets the length.
/// The returned pointer is owned by the parent ModelInfo and must NOT be freed.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_get_input_shape(
    info: *const edgefirst_msgs::ModelInfo,
    out_len: *mut usize,
) -> *const u32 {
    if info.is_null() {
        if !out_len.is_null() {
            unsafe { *out_len = 0 };
        }
        return ptr::null();
    }
    unsafe {
        if !out_len.is_null() {
            *out_len = (*info).input_shape.len();
        }
        (*info).input_shape.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_model_info_get_input_type(
    info: *const edgefirst_msgs::ModelInfo,
) -> u8 {
    unsafe {
        assert!(!info.is_null());
        (*info).input_type
    }
}

/// Returns a pointer to the output shape array and sets the length.
/// The returned pointer is owned by the parent ModelInfo and must NOT be freed.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_get_output_shape(
    info: *const edgefirst_msgs::ModelInfo,
    out_len: *mut usize,
) -> *const u32 {
    if info.is_null() {
        if !out_len.is_null() {
            unsafe { *out_len = 0 };
        }
        return ptr::null();
    }
    unsafe {
        if !out_len.is_null() {
            *out_len = (*info).output_shape.len();
        }
        (*info).output_shape.as_ptr()
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_model_info_get_output_type(
    info: *const edgefirst_msgs::ModelInfo,
) -> u8 {
    unsafe {
        assert!(!info.is_null());
        (*info).output_type
    }
}

/// Returns the number of labels.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_get_labels_count(
    info: *const edgefirst_msgs::ModelInfo,
) -> usize {
    unsafe {
        assert!(!info.is_null());
        (*info).labels.len()
    }
}

/// Returns the label at the given index. Caller owns the returned string and must free it.
/// Returns NULL if index is out of bounds.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_get_label(
    info: *const edgefirst_msgs::ModelInfo,
    index: usize,
) -> *mut c_char {
    unsafe {
        assert!(!info.is_null());
        match (&(*info).labels).get(index) {
            Some(label) => string_to_c_char(label),
            None => ptr::null_mut(),
        }
    }
}

/// Returns the model type string. Caller owns the returned string and must free it.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_get_model_type(
    info: *const edgefirst_msgs::ModelInfo,
) -> *mut c_char {
    unsafe {
        assert!(!info.is_null());
        string_to_c_char(&(*info).model_type)
    }
}

/// Returns the model format string. Caller owns the returned string and must free it.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_get_model_format(
    info: *const edgefirst_msgs::ModelInfo,
) -> *mut c_char {
    unsafe {
        assert!(!info.is_null());
        string_to_c_char(&(*info).model_format)
    }
}

/// Returns the model name string. Caller owns the returned string and must free it.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_get_model_name(
    info: *const edgefirst_msgs::ModelInfo,
) -> *mut c_char {
    unsafe {
        assert!(!info.is_null());
        string_to_c_char(&(*info).model_name)
    }
}

/// Sets the input shape array. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_set_input_shape(
    info: *mut edgefirst_msgs::ModelInfo,
    shape: *const u32,
    len: usize,
) -> i32 {
    check_null!(info);
    check_null!(shape);

    unsafe {
        let slice = slice::from_raw_parts(shape, len);
        (*info).input_shape = slice.to_vec();
        0
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_model_info_set_input_type(
    info: *mut edgefirst_msgs::ModelInfo,
    input_type: u8,
) {
    unsafe {
        assert!(!info.is_null());
        (*info).input_type = input_type;
    }
}

/// Sets the output shape array. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_set_output_shape(
    info: *mut edgefirst_msgs::ModelInfo,
    shape: *const u32,
    len: usize,
) -> i32 {
    check_null!(info);
    check_null!(shape);

    unsafe {
        let slice = slice::from_raw_parts(shape, len);
        (*info).output_shape = slice.to_vec();
        0
    }
}

#[no_mangle]
pub extern "C" fn edgefirst_model_info_set_output_type(
    info: *mut edgefirst_msgs::ModelInfo,
    output_type: u8,
) {
    unsafe {
        assert!(!info.is_null());
        (*info).output_type = output_type;
    }
}

/// Adds a label to the labels vector. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_add_label(
    info: *mut edgefirst_msgs::ModelInfo,
    label: *const c_char,
) -> i32 {
    check_null!(info);
    check_null!(label);

    unsafe {
        match c_char_to_string(label) {
            Some(s) => {
                (*info).labels.push(s);
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

/// Clears all labels.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_clear_labels(info: *mut edgefirst_msgs::ModelInfo) {
    unsafe {
        assert!(!info.is_null());
        (*info).labels.clear();
    }
}

/// Sets the model type string. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_set_model_type(
    info: *mut edgefirst_msgs::ModelInfo,
    model_type: *const c_char,
) -> i32 {
    check_null!(info);
    check_null!(model_type);

    unsafe {
        match c_char_to_string(model_type) {
            Some(s) => {
                (*info).model_type = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

/// Sets the model format string. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_set_model_format(
    info: *mut edgefirst_msgs::ModelInfo,
    model_format: *const c_char,
) -> i32 {
    check_null!(info);
    check_null!(model_format);

    unsafe {
        match c_char_to_string(model_format) {
            Some(s) => {
                (*info).model_format = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

/// Sets the model name string. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn edgefirst_model_info_set_model_name(
    info: *mut edgefirst_msgs::ModelInfo,
    model_name: *const c_char,
) -> i32 {
    check_null!(info);
    check_null!(model_name);

    unsafe {
        match c_char_to_string(model_name) {
            Some(s) => {
                (*info).model_name = s;
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
pub extern "C" fn edgefirst_model_info_serialize(
    info: *const edgefirst_msgs::ModelInfo,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(info);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*info) {
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
pub extern "C" fn edgefirst_model_info_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut edgefirst_msgs::ModelInfo {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<edgefirst_msgs::ModelInfo>(slice) {
            Ok(info) => Box::into_raw(Box::new(info)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// foxglove_msgs::FoxglovePoint2
// =============================================================================

#[no_mangle]
pub extern "C" fn foxglove_point2_new() -> *mut foxglove_msgs::FoxglovePoint2 {
    Box::into_raw(Box::new(foxglove_msgs::FoxglovePoint2 { x: 0.0, y: 0.0 }))
}

#[no_mangle]
pub extern "C" fn foxglove_point2_free(point: *mut foxglove_msgs::FoxglovePoint2) {
    if !point.is_null() {
        unsafe {
            drop(Box::from_raw(point));
        }
    }
}

#[no_mangle]
pub extern "C" fn foxglove_point2_get_x(point: *const foxglove_msgs::FoxglovePoint2) -> f64 {
    unsafe {
        assert!(!point.is_null());
        (*point).x
    }
}

#[no_mangle]
pub extern "C" fn foxglove_point2_get_y(point: *const foxglove_msgs::FoxglovePoint2) -> f64 {
    unsafe {
        assert!(!point.is_null());
        (*point).y
    }
}

#[no_mangle]
pub extern "C" fn foxglove_point2_set_x(point: *mut foxglove_msgs::FoxglovePoint2, x: f64) {
    unsafe {
        assert!(!point.is_null());
        (*point).x = x;
    }
}

#[no_mangle]
pub extern "C" fn foxglove_point2_set_y(point: *mut foxglove_msgs::FoxglovePoint2, y: f64) {
    unsafe {
        assert!(!point.is_null());
        (*point).y = y;
    }
}

// =============================================================================
// foxglove_msgs::FoxgloveColor
// =============================================================================

#[no_mangle]
pub extern "C" fn foxglove_color_new() -> *mut foxglove_msgs::FoxgloveColor {
    Box::into_raw(Box::new(foxglove_msgs::FoxgloveColor {
        r: 0.0,
        g: 0.0,
        b: 0.0,
        a: 1.0,
    }))
}

#[no_mangle]
pub extern "C" fn foxglove_color_free(color: *mut foxglove_msgs::FoxgloveColor) {
    if !color.is_null() {
        unsafe {
            drop(Box::from_raw(color));
        }
    }
}

#[no_mangle]
pub extern "C" fn foxglove_color_get_r(color: *const foxglove_msgs::FoxgloveColor) -> f64 {
    unsafe {
        assert!(!color.is_null());
        (*color).r
    }
}

#[no_mangle]
pub extern "C" fn foxglove_color_get_g(color: *const foxglove_msgs::FoxgloveColor) -> f64 {
    unsafe {
        assert!(!color.is_null());
        (*color).g
    }
}

#[no_mangle]
pub extern "C" fn foxglove_color_get_b(color: *const foxglove_msgs::FoxgloveColor) -> f64 {
    unsafe {
        assert!(!color.is_null());
        (*color).b
    }
}

#[no_mangle]
pub extern "C" fn foxglove_color_get_a(color: *const foxglove_msgs::FoxgloveColor) -> f64 {
    unsafe {
        assert!(!color.is_null());
        (*color).a
    }
}

#[no_mangle]
pub extern "C" fn foxglove_color_set_r(color: *mut foxglove_msgs::FoxgloveColor, r: f64) {
    unsafe {
        assert!(!color.is_null());
        (*color).r = r;
    }
}

#[no_mangle]
pub extern "C" fn foxglove_color_set_g(color: *mut foxglove_msgs::FoxgloveColor, g: f64) {
    unsafe {
        assert!(!color.is_null());
        (*color).g = g;
    }
}

#[no_mangle]
pub extern "C" fn foxglove_color_set_b(color: *mut foxglove_msgs::FoxgloveColor, b: f64) {
    unsafe {
        assert!(!color.is_null());
        (*color).b = b;
    }
}

#[no_mangle]
pub extern "C" fn foxglove_color_set_a(color: *mut foxglove_msgs::FoxgloveColor, a: f64) {
    unsafe {
        assert!(!color.is_null());
        (*color).a = a;
    }
}

// =============================================================================
// foxglove_msgs::FoxgloveCircleAnnotations
// =============================================================================

#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_new() -> *mut foxglove_msgs::FoxgloveCircleAnnotations
{
    Box::into_raw(Box::new(foxglove_msgs::FoxgloveCircleAnnotations {
        timestamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
        position: foxglove_msgs::FoxglovePoint2 { x: 0.0, y: 0.0 },
        diameter: 0.0,
        thickness: 1.0,
        fill_color: foxglove_msgs::FoxgloveColor {
            r: 0.0,
            g: 0.0,
            b: 0.0,
            a: 0.0,
        },
        outline_color: foxglove_msgs::FoxgloveColor {
            r: 1.0,
            g: 1.0,
            b: 1.0,
            a: 1.0,
        },
    }))
}

#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_free(
    circle: *mut foxglove_msgs::FoxgloveCircleAnnotations,
) {
    if !circle.is_null() {
        unsafe {
            drop(Box::from_raw(circle));
        }
    }
}

/// Returns a pointer to the timestamp field. The returned pointer is owned by
/// the parent FoxgloveCircleAnnotations and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_get_timestamp(
    circle: *const foxglove_msgs::FoxgloveCircleAnnotations,
) -> *const builtin_interfaces::Time {
    unsafe {
        assert!(!circle.is_null());
        &(*circle).timestamp
    }
}

/// Returns a mutable pointer to the timestamp field for modification.
/// The returned pointer is owned by the parent and must NOT be freed.
#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_get_timestamp_mut(
    circle: *mut foxglove_msgs::FoxgloveCircleAnnotations,
) -> *mut builtin_interfaces::Time {
    unsafe {
        assert!(!circle.is_null());
        &mut (*circle).timestamp
    }
}

/// Returns a pointer to the position field. The returned pointer is owned by
/// the parent FoxgloveCircleAnnotations and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_get_position(
    circle: *const foxglove_msgs::FoxgloveCircleAnnotations,
) -> *const foxglove_msgs::FoxglovePoint2 {
    unsafe {
        assert!(!circle.is_null());
        &(*circle).position
    }
}

/// Returns a mutable pointer to the position field for modification.
/// The returned pointer is owned by the parent and must NOT be freed.
#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_get_position_mut(
    circle: *mut foxglove_msgs::FoxgloveCircleAnnotations,
) -> *mut foxglove_msgs::FoxglovePoint2 {
    unsafe {
        assert!(!circle.is_null());
        &mut (*circle).position
    }
}

#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_get_diameter(
    circle: *const foxglove_msgs::FoxgloveCircleAnnotations,
) -> f64 {
    unsafe {
        assert!(!circle.is_null());
        (*circle).diameter
    }
}

#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_get_thickness(
    circle: *const foxglove_msgs::FoxgloveCircleAnnotations,
) -> f64 {
    unsafe {
        assert!(!circle.is_null());
        (*circle).thickness
    }
}

/// Returns a pointer to the fill_color field. The returned pointer is owned by
/// the parent FoxgloveCircleAnnotations and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_get_fill_color(
    circle: *const foxglove_msgs::FoxgloveCircleAnnotations,
) -> *const foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!circle.is_null());
        &(*circle).fill_color
    }
}

/// Returns a mutable pointer to the fill_color field for modification.
/// The returned pointer is owned by the parent and must NOT be freed.
#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_get_fill_color_mut(
    circle: *mut foxglove_msgs::FoxgloveCircleAnnotations,
) -> *mut foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!circle.is_null());
        &mut (*circle).fill_color
    }
}

/// Returns a pointer to the outline_color field. The returned pointer is owned by
/// the parent FoxgloveCircleAnnotations and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_get_outline_color(
    circle: *const foxglove_msgs::FoxgloveCircleAnnotations,
) -> *const foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!circle.is_null());
        &(*circle).outline_color
    }
}

/// Returns a mutable pointer to the outline_color field for modification.
/// The returned pointer is owned by the parent and must NOT be freed.
#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_get_outline_color_mut(
    circle: *mut foxglove_msgs::FoxgloveCircleAnnotations,
) -> *mut foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!circle.is_null());
        &mut (*circle).outline_color
    }
}

#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_set_diameter(
    circle: *mut foxglove_msgs::FoxgloveCircleAnnotations,
    diameter: f64,
) {
    unsafe {
        assert!(!circle.is_null());
        (*circle).diameter = diameter;
    }
}

#[no_mangle]
pub extern "C" fn foxglove_circle_annotations_set_thickness(
    circle: *mut foxglove_msgs::FoxgloveCircleAnnotations,
    thickness: f64,
) {
    unsafe {
        assert!(!circle.is_null());
        (*circle).thickness = thickness;
    }
}

// =============================================================================
// foxglove_msgs::FoxglovePointAnnotations
// =============================================================================

#[no_mangle]
pub extern "C" fn foxglove_point_annotations_new() -> *mut foxglove_msgs::FoxglovePointAnnotations {
    Box::into_raw(Box::new(foxglove_msgs::FoxglovePointAnnotations {
        timestamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
        type_: 0,
        points: Vec::new(),
        outline_color: foxglove_msgs::FoxgloveColor {
            r: 1.0,
            g: 1.0,
            b: 1.0,
            a: 1.0,
        },
        outline_colors: Vec::new(),
        fill_color: foxglove_msgs::FoxgloveColor {
            r: 0.0,
            g: 0.0,
            b: 0.0,
            a: 0.0,
        },
        thickness: 1.0,
    }))
}

#[no_mangle]
pub extern "C" fn foxglove_point_annotations_free(
    points: *mut foxglove_msgs::FoxglovePointAnnotations,
) {
    if !points.is_null() {
        unsafe {
            drop(Box::from_raw(points));
        }
    }
}

/// Returns a pointer to the timestamp field. The returned pointer is owned by
/// the parent FoxglovePointAnnotations and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_get_timestamp(
    ann: *const foxglove_msgs::FoxglovePointAnnotations,
) -> *const builtin_interfaces::Time {
    unsafe {
        assert!(!ann.is_null());
        &(*ann).timestamp
    }
}

/// Returns a mutable pointer to the timestamp field for modification.
/// The returned pointer is owned by the parent and must NOT be freed.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_get_timestamp_mut(
    ann: *mut foxglove_msgs::FoxglovePointAnnotations,
) -> *mut builtin_interfaces::Time {
    unsafe {
        assert!(!ann.is_null());
        &mut (*ann).timestamp
    }
}

#[no_mangle]
pub extern "C" fn foxglove_point_annotations_get_type(
    ann: *const foxglove_msgs::FoxglovePointAnnotations,
) -> u8 {
    unsafe {
        assert!(!ann.is_null());
        (*ann).type_
    }
}

#[no_mangle]
pub extern "C" fn foxglove_point_annotations_set_type(
    ann: *mut foxglove_msgs::FoxglovePointAnnotations,
    type_: u8,
) {
    unsafe {
        assert!(!ann.is_null());
        (*ann).type_ = type_;
    }
}

/// Returns a pointer to the point at the given index. The returned pointer is owned by
/// the parent FoxglovePointAnnotations and must NOT be freed by the caller.
/// Returns NULL if index is out of bounds.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_get_point(
    ann: *const foxglove_msgs::FoxglovePointAnnotations,
    index: usize,
) -> *const foxglove_msgs::FoxglovePoint2 {
    unsafe {
        assert!(!ann.is_null());
        match (&(*ann).points).get(index) {
            Some(point) => point,
            None => ptr::null(),
        }
    }
}

/// Returns the number of points.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_get_points_count(
    ann: *const foxglove_msgs::FoxglovePointAnnotations,
) -> usize {
    unsafe {
        assert!(!ann.is_null());
        (*ann).points.len()
    }
}

/// Adds a copy of the given point to the points vector. Returns 0 on success.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_add_point(
    ann: *mut foxglove_msgs::FoxglovePointAnnotations,
    point: *const foxglove_msgs::FoxglovePoint2,
) -> i32 {
    check_null!(ann);
    check_null!(point);

    unsafe {
        (*ann).points.push((*point).clone());
        0
    }
}

/// Clears all points.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_clear_points(
    ann: *mut foxglove_msgs::FoxglovePointAnnotations,
) {
    unsafe {
        assert!(!ann.is_null());
        (*ann).points.clear();
    }
}

/// Returns a pointer to the outline_color field. The returned pointer is owned by
/// the parent FoxglovePointAnnotations and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_get_outline_color(
    ann: *const foxglove_msgs::FoxglovePointAnnotations,
) -> *const foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!ann.is_null());
        &(*ann).outline_color
    }
}

/// Returns a mutable pointer to the outline_color field for modification.
/// The returned pointer is owned by the parent and must NOT be freed.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_get_outline_color_mut(
    ann: *mut foxglove_msgs::FoxglovePointAnnotations,
) -> *mut foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!ann.is_null());
        &mut (*ann).outline_color
    }
}

/// Returns a pointer to the outline_color at the given index. The returned pointer is owned by
/// the parent FoxglovePointAnnotations and must NOT be freed by the caller.
/// Returns NULL if index is out of bounds.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_get_outline_color_at(
    ann: *const foxglove_msgs::FoxglovePointAnnotations,
    index: usize,
) -> *const foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!ann.is_null());
        match (&(*ann).outline_colors).get(index) {
            Some(color) => color,
            None => ptr::null(),
        }
    }
}

/// Returns the number of outline colors.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_get_outline_colors_count(
    ann: *const foxglove_msgs::FoxglovePointAnnotations,
) -> usize {
    unsafe {
        assert!(!ann.is_null());
        (*ann).outline_colors.len()
    }
}

/// Adds a copy of the given color to the outline_colors vector. Returns 0 on success.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_add_outline_color(
    ann: *mut foxglove_msgs::FoxglovePointAnnotations,
    color: *const foxglove_msgs::FoxgloveColor,
) -> i32 {
    check_null!(ann);
    check_null!(color);

    unsafe {
        (*ann).outline_colors.push((*color).clone());
        0
    }
}

/// Clears all outline colors.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_clear_outline_colors(
    ann: *mut foxglove_msgs::FoxglovePointAnnotations,
) {
    unsafe {
        assert!(!ann.is_null());
        (*ann).outline_colors.clear();
    }
}

/// Returns a pointer to the fill_color field. The returned pointer is owned by
/// the parent FoxglovePointAnnotations and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_get_fill_color(
    ann: *const foxglove_msgs::FoxglovePointAnnotations,
) -> *const foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!ann.is_null());
        &(*ann).fill_color
    }
}

/// Returns a mutable pointer to the fill_color field for modification.
/// The returned pointer is owned by the parent and must NOT be freed.
#[no_mangle]
pub extern "C" fn foxglove_point_annotations_get_fill_color_mut(
    ann: *mut foxglove_msgs::FoxglovePointAnnotations,
) -> *mut foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!ann.is_null());
        &mut (*ann).fill_color
    }
}

#[no_mangle]
pub extern "C" fn foxglove_point_annotations_get_thickness(
    ann: *const foxglove_msgs::FoxglovePointAnnotations,
) -> f64 {
    unsafe {
        assert!(!ann.is_null());
        (*ann).thickness
    }
}

#[no_mangle]
pub extern "C" fn foxglove_point_annotations_set_thickness(
    ann: *mut foxglove_msgs::FoxglovePointAnnotations,
    thickness: f64,
) {
    unsafe {
        assert!(!ann.is_null());
        (*ann).thickness = thickness;
    }
}

// =============================================================================
// foxglove_msgs::FoxgloveTextAnnotations
// =============================================================================

#[no_mangle]
pub extern "C" fn foxglove_text_annotations_new() -> *mut foxglove_msgs::FoxgloveTextAnnotations {
    Box::into_raw(Box::new(foxglove_msgs::FoxgloveTextAnnotations {
        timestamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
        position: foxglove_msgs::FoxglovePoint2 { x: 0.0, y: 0.0 },
        text: String::new(),
        font_size: 12.0,
        text_color: foxglove_msgs::FoxgloveColor {
            r: 1.0,
            g: 1.0,
            b: 1.0,
            a: 1.0,
        },
        background_color: foxglove_msgs::FoxgloveColor {
            r: 0.0,
            g: 0.0,
            b: 0.0,
            a: 0.0,
        },
    }))
}

#[no_mangle]
pub extern "C" fn foxglove_text_annotations_free(
    text: *mut foxglove_msgs::FoxgloveTextAnnotations,
) {
    if !text.is_null() {
        unsafe {
            drop(Box::from_raw(text));
        }
    }
}

/// Returns a pointer to the timestamp field. The returned pointer is owned by
/// the parent FoxgloveTextAnnotations and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn foxglove_text_annotations_get_timestamp(
    text: *const foxglove_msgs::FoxgloveTextAnnotations,
) -> *const builtin_interfaces::Time {
    unsafe {
        assert!(!text.is_null());
        &(*text).timestamp
    }
}

/// Returns a mutable pointer to the timestamp field for modification.
/// The returned pointer is owned by the parent and must NOT be freed.
#[no_mangle]
pub extern "C" fn foxglove_text_annotations_get_timestamp_mut(
    text: *mut foxglove_msgs::FoxgloveTextAnnotations,
) -> *mut builtin_interfaces::Time {
    unsafe {
        assert!(!text.is_null());
        &mut (*text).timestamp
    }
}

/// Returns a pointer to the position field. The returned pointer is owned by
/// the parent FoxgloveTextAnnotations and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn foxglove_text_annotations_get_position(
    ann: *const foxglove_msgs::FoxgloveTextAnnotations,
) -> *const foxglove_msgs::FoxglovePoint2 {
    unsafe {
        assert!(!ann.is_null());
        &(*ann).position
    }
}

/// Returns a mutable pointer to the position field for modification.
/// The returned pointer is owned by the parent and must NOT be freed.
#[no_mangle]
pub extern "C" fn foxglove_text_annotations_get_position_mut(
    ann: *mut foxglove_msgs::FoxgloveTextAnnotations,
) -> *mut foxglove_msgs::FoxglovePoint2 {
    unsafe {
        assert!(!ann.is_null());
        &mut (*ann).position
    }
}

/// Returns the text string. Caller owns the returned string and must free it.
#[no_mangle]
pub extern "C" fn foxglove_text_annotations_get_text(
    ann: *const foxglove_msgs::FoxgloveTextAnnotations,
) -> *mut c_char {
    unsafe {
        assert!(!ann.is_null());
        string_to_c_char(&(*ann).text)
    }
}

/// Sets the text string. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn foxglove_text_annotations_set_text(
    ann: *mut foxglove_msgs::FoxgloveTextAnnotations,
    text: *const c_char,
) -> i32 {
    check_null!(ann);
    check_null!(text);

    unsafe {
        match c_char_to_string(text) {
            Some(s) => {
                (*ann).text = s;
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
pub extern "C" fn foxglove_text_annotations_get_font_size(
    ann: *const foxglove_msgs::FoxgloveTextAnnotations,
) -> f64 {
    unsafe {
        assert!(!ann.is_null());
        (*ann).font_size
    }
}

#[no_mangle]
pub extern "C" fn foxglove_text_annotations_set_font_size(
    ann: *mut foxglove_msgs::FoxgloveTextAnnotations,
    font_size: f64,
) {
    unsafe {
        assert!(!ann.is_null());
        (*ann).font_size = font_size;
    }
}

/// Returns a pointer to the text_color field. The returned pointer is owned by
/// the parent FoxgloveTextAnnotations and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn foxglove_text_annotations_get_text_color(
    ann: *const foxglove_msgs::FoxgloveTextAnnotations,
) -> *const foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!ann.is_null());
        &(*ann).text_color
    }
}

/// Returns a mutable pointer to the text_color field for modification.
/// The returned pointer is owned by the parent and must NOT be freed.
#[no_mangle]
pub extern "C" fn foxglove_text_annotations_get_text_color_mut(
    ann: *mut foxglove_msgs::FoxgloveTextAnnotations,
) -> *mut foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!ann.is_null());
        &mut (*ann).text_color
    }
}

/// Returns a pointer to the background_color field. The returned pointer is owned by
/// the parent FoxgloveTextAnnotations and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn foxglove_text_annotations_get_background_color(
    ann: *const foxglove_msgs::FoxgloveTextAnnotations,
) -> *const foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!ann.is_null());
        &(*ann).background_color
    }
}

/// Returns a mutable pointer to the background_color field for modification.
/// The returned pointer is owned by the parent and must NOT be freed.
#[no_mangle]
pub extern "C" fn foxglove_text_annotations_get_background_color_mut(
    ann: *mut foxglove_msgs::FoxgloveTextAnnotations,
) -> *mut foxglove_msgs::FoxgloveColor {
    unsafe {
        assert!(!ann.is_null());
        &mut (*ann).background_color
    }
}

// =============================================================================
// foxglove_msgs::FoxgloveImageAnnotations
// =============================================================================

#[no_mangle]
pub extern "C" fn foxglove_image_annotations_new() -> *mut foxglove_msgs::FoxgloveImageAnnotations {
    Box::into_raw(Box::new(foxglove_msgs::FoxgloveImageAnnotations {
        circles: Vec::new(),
        points: Vec::new(),
        texts: Vec::new(),
    }))
}

#[no_mangle]
pub extern "C" fn foxglove_image_annotations_free(
    ann: *mut foxglove_msgs::FoxgloveImageAnnotations,
) {
    if !ann.is_null() {
        unsafe {
            drop(Box::from_raw(ann));
        }
    }
}

/// Returns a pointer to the circle annotation at the given index. The returned pointer is owned by
/// the parent FoxgloveImageAnnotations and must NOT be freed by the caller.
/// Returns NULL if index is out of bounds.
#[no_mangle]
pub extern "C" fn foxglove_image_annotations_get_circle(
    ann: *const foxglove_msgs::FoxgloveImageAnnotations,
    index: usize,
) -> *const foxglove_msgs::FoxgloveCircleAnnotations {
    unsafe {
        assert!(!ann.is_null());
        match (&(*ann).circles).get(index) {
            Some(circle) => circle,
            None => ptr::null(),
        }
    }
}

/// Returns the number of circle annotations.
#[no_mangle]
pub extern "C" fn foxglove_image_annotations_get_circles_count(
    ann: *const foxglove_msgs::FoxgloveImageAnnotations,
) -> usize {
    unsafe {
        assert!(!ann.is_null());
        (*ann).circles.len()
    }
}

/// Adds a copy of the given circle annotation. Returns 0 on success.
#[no_mangle]
pub extern "C" fn foxglove_image_annotations_add_circle(
    ann: *mut foxglove_msgs::FoxgloveImageAnnotations,
    circle: *const foxglove_msgs::FoxgloveCircleAnnotations,
) -> i32 {
    check_null!(ann);
    check_null!(circle);

    unsafe {
        (*ann).circles.push((*circle).clone());
        0
    }
}

/// Clears all circle annotations.
#[no_mangle]
pub extern "C" fn foxglove_image_annotations_clear_circles(
    ann: *mut foxglove_msgs::FoxgloveImageAnnotations,
) {
    unsafe {
        assert!(!ann.is_null());
        (*ann).circles.clear();
    }
}

/// Returns a pointer to the point annotation at the given index. The returned pointer is owned by
/// the parent FoxgloveImageAnnotations and must NOT be freed by the caller.
/// Returns NULL if index is out of bounds.
#[no_mangle]
pub extern "C" fn foxglove_image_annotations_get_point(
    ann: *const foxglove_msgs::FoxgloveImageAnnotations,
    index: usize,
) -> *const foxglove_msgs::FoxglovePointAnnotations {
    unsafe {
        assert!(!ann.is_null());
        match (&(*ann).points).get(index) {
            Some(point) => point,
            None => ptr::null(),
        }
    }
}

/// Returns the number of point annotations.
#[no_mangle]
pub extern "C" fn foxglove_image_annotations_get_points_count(
    ann: *const foxglove_msgs::FoxgloveImageAnnotations,
) -> usize {
    unsafe {
        assert!(!ann.is_null());
        (*ann).points.len()
    }
}

/// Adds a copy of the given point annotation. Returns 0 on success.
#[no_mangle]
pub extern "C" fn foxglove_image_annotations_add_point(
    ann: *mut foxglove_msgs::FoxgloveImageAnnotations,
    point: *const foxglove_msgs::FoxglovePointAnnotations,
) -> i32 {
    check_null!(ann);
    check_null!(point);

    unsafe {
        (*ann).points.push((*point).clone());
        0
    }
}

/// Clears all point annotations.
#[no_mangle]
pub extern "C" fn foxglove_image_annotations_clear_points(
    ann: *mut foxglove_msgs::FoxgloveImageAnnotations,
) {
    unsafe {
        assert!(!ann.is_null());
        (*ann).points.clear();
    }
}

/// Returns a pointer to the text annotation at the given index. The returned pointer is owned by
/// the parent FoxgloveImageAnnotations and must NOT be freed by the caller.
/// Returns NULL if index is out of bounds.
#[no_mangle]
pub extern "C" fn foxglove_image_annotations_get_text(
    ann: *const foxglove_msgs::FoxgloveImageAnnotations,
    index: usize,
) -> *const foxglove_msgs::FoxgloveTextAnnotations {
    unsafe {
        assert!(!ann.is_null());
        match (&(*ann).texts).get(index) {
            Some(text) => text,
            None => ptr::null(),
        }
    }
}

/// Returns the number of text annotations.
#[no_mangle]
pub extern "C" fn foxglove_image_annotations_get_texts_count(
    ann: *const foxglove_msgs::FoxgloveImageAnnotations,
) -> usize {
    unsafe {
        assert!(!ann.is_null());
        (*ann).texts.len()
    }
}

/// Adds a copy of the given text annotation. Returns 0 on success.
#[no_mangle]
pub extern "C" fn foxglove_image_annotations_add_text(
    ann: *mut foxglove_msgs::FoxgloveImageAnnotations,
    text: *const foxglove_msgs::FoxgloveTextAnnotations,
) -> i32 {
    check_null!(ann);
    check_null!(text);

    unsafe {
        (*ann).texts.push((*text).clone());
        0
    }
}

/// Clears all text annotations.
#[no_mangle]
pub extern "C" fn foxglove_image_annotations_clear_texts(
    ann: *mut foxglove_msgs::FoxgloveImageAnnotations,
) {
    unsafe {
        assert!(!ann.is_null());
        (*ann).texts.clear();
    }
}

#[no_mangle]
pub extern "C" fn foxglove_image_annotations_serialize(
    ann: *const foxglove_msgs::FoxgloveImageAnnotations,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(ann);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*ann) {
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
pub extern "C" fn foxglove_image_annotations_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut foxglove_msgs::FoxgloveImageAnnotations {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<foxglove_msgs::FoxgloveImageAnnotations>(slice) {
            Ok(ann) => Box::into_raw(Box::new(ann)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::Accel
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_accel_new() -> *mut geometry_msgs::Accel {
    Box::into_raw(Box::new(geometry_msgs::Accel {
        linear: geometry_msgs::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        angular: geometry_msgs::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
    }))
}

#[no_mangle]
pub extern "C" fn ros_accel_free(accel: *mut geometry_msgs::Accel) {
    if !accel.is_null() {
        unsafe {
            drop(Box::from_raw(accel));
        }
    }
}

/// Returns a pointer to the linear acceleration field. The returned pointer is owned by
/// the parent Accel and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_accel_get_linear(
    accel: *const geometry_msgs::Accel,
) -> *const geometry_msgs::Vector3 {
    unsafe {
        assert!(!accel.is_null());
        &(*accel).linear
    }
}

/// Returns a mutable pointer to the linear acceleration field for modification.
/// The returned pointer is owned by the parent Accel and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_accel_get_linear_mut(
    accel: *mut geometry_msgs::Accel,
) -> *mut geometry_msgs::Vector3 {
    unsafe {
        assert!(!accel.is_null());
        &mut (*accel).linear
    }
}

/// Returns a pointer to the angular acceleration field. The returned pointer is owned by
/// the parent Accel and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_accel_get_angular(
    accel: *const geometry_msgs::Accel,
) -> *const geometry_msgs::Vector3 {
    unsafe {
        assert!(!accel.is_null());
        &(*accel).angular
    }
}

/// Returns a mutable pointer to the angular acceleration field for modification.
/// The returned pointer is owned by the parent Accel and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_accel_get_angular_mut(
    accel: *mut geometry_msgs::Accel,
) -> *mut geometry_msgs::Vector3 {
    unsafe {
        assert!(!accel.is_null());
        &mut (*accel).angular
    }
}

#[no_mangle]
pub extern "C" fn ros_accel_serialize(
    accel: *const geometry_msgs::Accel,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(accel);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*accel) {
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
pub extern "C" fn ros_accel_deserialize(bytes: *const u8, len: usize) -> *mut geometry_msgs::Accel {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::Accel>(slice) {
            Ok(accel) => Box::into_raw(Box::new(accel)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::AccelStamped
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_accel_stamped_new() -> *mut geometry_msgs::AccelStamped {
    Box::into_raw(Box::new(geometry_msgs::AccelStamped {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        accel: geometry_msgs::Accel {
            linear: geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular: geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        },
    }))
}

#[no_mangle]
pub extern "C" fn ros_accel_stamped_free(accel: *mut geometry_msgs::AccelStamped) {
    if !accel.is_null() {
        unsafe {
            drop(Box::from_raw(accel));
        }
    }
}

/// Returns a pointer to the header field. The returned pointer is owned by
/// the parent AccelStamped and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_accel_stamped_get_header(
    accel: *const geometry_msgs::AccelStamped,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!accel.is_null());
        &(*accel).header
    }
}

/// Returns a mutable pointer to the header field for modification.
/// The returned pointer is owned by the parent AccelStamped and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_accel_stamped_get_header_mut(
    accel: *mut geometry_msgs::AccelStamped,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!accel.is_null());
        &mut (*accel).header
    }
}

/// Returns a pointer to the accel field. The returned pointer is owned by
/// the parent AccelStamped and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_accel_stamped_get_accel(
    stamped: *const geometry_msgs::AccelStamped,
) -> *const geometry_msgs::Accel {
    unsafe {
        assert!(!stamped.is_null());
        &(*stamped).accel
    }
}

/// Returns a mutable pointer to the accel field for modification.
/// The returned pointer is owned by the parent AccelStamped and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_accel_stamped_get_accel_mut(
    stamped: *mut geometry_msgs::AccelStamped,
) -> *mut geometry_msgs::Accel {
    unsafe {
        assert!(!stamped.is_null());
        &mut (*stamped).accel
    }
}

#[no_mangle]
pub extern "C" fn ros_accel_stamped_serialize(
    accel: *const geometry_msgs::AccelStamped,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(accel);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*accel) {
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
pub extern "C" fn ros_accel_stamped_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut geometry_msgs::AccelStamped {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::AccelStamped>(slice) {
            Ok(accel) => Box::into_raw(Box::new(accel)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::PointStamped
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_point_stamped_new() -> *mut geometry_msgs::PointStamped {
    Box::into_raw(Box::new(geometry_msgs::PointStamped {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        point: geometry_msgs::Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
    }))
}

#[no_mangle]
pub extern "C" fn ros_point_stamped_free(point: *mut geometry_msgs::PointStamped) {
    if !point.is_null() {
        unsafe {
            drop(Box::from_raw(point));
        }
    }
}

/// Returns a pointer to the header field. The returned pointer is owned by
/// the parent PointStamped and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_point_stamped_get_header(
    point: *const geometry_msgs::PointStamped,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!point.is_null());
        &(*point).header
    }
}

/// Returns a mutable pointer to the header field for modification.
/// The returned pointer is owned by the parent PointStamped and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_point_stamped_get_header_mut(
    point: *mut geometry_msgs::PointStamped,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!point.is_null());
        &mut (*point).header
    }
}

/// Returns a pointer to the point field. The returned pointer is owned by
/// the parent PointStamped and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_point_stamped_get_point(
    stamped: *const geometry_msgs::PointStamped,
) -> *const geometry_msgs::Point {
    unsafe {
        assert!(!stamped.is_null());
        &(*stamped).point
    }
}

/// Returns a mutable pointer to the point field for modification.
/// The returned pointer is owned by the parent PointStamped and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_point_stamped_get_point_mut(
    stamped: *mut geometry_msgs::PointStamped,
) -> *mut geometry_msgs::Point {
    unsafe {
        assert!(!stamped.is_null());
        &mut (*stamped).point
    }
}

#[no_mangle]
pub extern "C" fn ros_point_stamped_serialize(
    point: *const geometry_msgs::PointStamped,
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
pub extern "C" fn ros_point_stamped_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut geometry_msgs::PointStamped {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::PointStamped>(slice) {
            Ok(point) => Box::into_raw(Box::new(point)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::TransformStamped
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_transform_stamped_new() -> *mut geometry_msgs::TransformStamped {
    Box::into_raw(Box::new(geometry_msgs::TransformStamped {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        child_frame_id: String::new(),
        transform: geometry_msgs::Transform {
            translation: geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: geometry_msgs::Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        },
    }))
}

#[no_mangle]
pub extern "C" fn ros_transform_stamped_free(transform: *mut geometry_msgs::TransformStamped) {
    if !transform.is_null() {
        unsafe {
            drop(Box::from_raw(transform));
        }
    }
}

/// Returns a pointer to the header field. The returned pointer is owned by
/// the parent TransformStamped and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_transform_stamped_get_header(
    transform: *const geometry_msgs::TransformStamped,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!transform.is_null());
        &(*transform).header
    }
}

/// Returns a mutable pointer to the header field for modification.
/// The returned pointer is owned by the parent TransformStamped and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_transform_stamped_get_header_mut(
    transform: *mut geometry_msgs::TransformStamped,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!transform.is_null());
        &mut (*transform).header
    }
}

/// Returns the child_frame_id field. Caller must free the returned string.
#[no_mangle]
pub extern "C" fn ros_transform_stamped_get_child_frame_id(
    transform: *const geometry_msgs::TransformStamped,
) -> *mut c_char {
    unsafe {
        assert!(!transform.is_null());
        string_to_c_char(&(*transform).child_frame_id)
    }
}

/// Sets the child_frame_id field. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn ros_transform_stamped_set_child_frame_id(
    transform: *mut geometry_msgs::TransformStamped,
    child_frame_id: *const c_char,
) -> i32 {
    check_null!(transform);
    check_null!(child_frame_id);

    unsafe {
        match c_char_to_string(child_frame_id) {
            Some(s) => {
                (*transform).child_frame_id = s;
                0
            }
            None => {
                set_errno(EINVAL);
                -1
            }
        }
    }
}

/// Returns a pointer to the transform field. The returned pointer is owned by
/// the parent TransformStamped and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_transform_stamped_get_transform(
    stamped: *const geometry_msgs::TransformStamped,
) -> *const geometry_msgs::Transform {
    unsafe {
        assert!(!stamped.is_null());
        &(*stamped).transform
    }
}

/// Returns a mutable pointer to the transform field for modification.
/// The returned pointer is owned by the parent TransformStamped and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_transform_stamped_get_transform_mut(
    stamped: *mut geometry_msgs::TransformStamped,
) -> *mut geometry_msgs::Transform {
    unsafe {
        assert!(!stamped.is_null());
        &mut (*stamped).transform
    }
}

#[no_mangle]
pub extern "C" fn ros_transform_stamped_serialize(
    transform: *const geometry_msgs::TransformStamped,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(transform);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*transform) {
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
pub extern "C" fn ros_transform_stamped_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut geometry_msgs::TransformStamped {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::TransformStamped>(slice) {
            Ok(transform) => Box::into_raw(Box::new(transform)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// geometry_msgs::TwistStamped
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_twist_stamped_new() -> *mut geometry_msgs::TwistStamped {
    Box::into_raw(Box::new(geometry_msgs::TwistStamped {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        },
        twist: geometry_msgs::Twist {
            linear: geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular: geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        },
    }))
}

#[no_mangle]
pub extern "C" fn ros_twist_stamped_free(twist: *mut geometry_msgs::TwistStamped) {
    if !twist.is_null() {
        unsafe {
            drop(Box::from_raw(twist));
        }
    }
}

/// Returns a pointer to the header field. The returned pointer is owned by
/// the parent TwistStamped and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_twist_stamped_get_header(
    twist: *const geometry_msgs::TwistStamped,
) -> *const std_msgs::Header {
    unsafe {
        assert!(!twist.is_null());
        &(*twist).header
    }
}

/// Returns a mutable pointer to the header field for modification.
/// The returned pointer is owned by the parent TwistStamped and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_twist_stamped_get_header_mut(
    twist: *mut geometry_msgs::TwistStamped,
) -> *mut std_msgs::Header {
    unsafe {
        assert!(!twist.is_null());
        &mut (*twist).header
    }
}

/// Returns a pointer to the twist field. The returned pointer is owned by
/// the parent TwistStamped and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_twist_stamped_get_twist(
    stamped: *const geometry_msgs::TwistStamped,
) -> *const geometry_msgs::Twist {
    unsafe {
        assert!(!stamped.is_null());
        &(*stamped).twist
    }
}

/// Returns a mutable pointer to the twist field for modification.
/// The returned pointer is owned by the parent TwistStamped and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_twist_stamped_get_twist_mut(
    stamped: *mut geometry_msgs::TwistStamped,
) -> *mut geometry_msgs::Twist {
    unsafe {
        assert!(!stamped.is_null());
        &mut (*stamped).twist
    }
}

#[no_mangle]
pub extern "C" fn ros_twist_stamped_serialize(
    twist: *const geometry_msgs::TwistStamped,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(twist);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*twist) {
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
pub extern "C" fn ros_twist_stamped_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut geometry_msgs::TwistStamped {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<geometry_msgs::TwistStamped>(slice) {
            Ok(twist) => Box::into_raw(Box::new(twist)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// rosgraph_msgs::Clock
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_clock_new() -> *mut rosgraph_msgs::Clock {
    Box::into_raw(Box::new(rosgraph_msgs::Clock {
        clock: builtin_interfaces::Time { sec: 0, nanosec: 0 },
    }))
}

#[no_mangle]
pub extern "C" fn ros_clock_free(clock: *mut rosgraph_msgs::Clock) {
    if !clock.is_null() {
        unsafe {
            drop(Box::from_raw(clock));
        }
    }
}

/// Returns a pointer to the clock field. The returned pointer is owned by
/// the parent Clock and must NOT be freed by the caller.
#[no_mangle]
pub extern "C" fn ros_clock_get_clock(
    clock: *const rosgraph_msgs::Clock,
) -> *const builtin_interfaces::Time {
    unsafe {
        assert!(!clock.is_null());
        &(*clock).clock
    }
}

/// Returns a mutable pointer to the clock field for modification.
/// The returned pointer is owned by the parent Clock and must NOT be freed.
#[no_mangle]
pub extern "C" fn ros_clock_get_clock_mut(
    clock: *mut rosgraph_msgs::Clock,
) -> *mut builtin_interfaces::Time {
    unsafe {
        assert!(!clock.is_null());
        &mut (*clock).clock
    }
}

#[no_mangle]
pub extern "C" fn ros_clock_serialize(
    clock: *const rosgraph_msgs::Clock,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    check_null!(clock);
    check_null!(out_bytes);
    check_null!(out_len);

    unsafe {
        match serde_cdr::serialize(&*clock) {
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
pub extern "C" fn ros_clock_deserialize(bytes: *const u8, len: usize) -> *mut rosgraph_msgs::Clock {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<rosgraph_msgs::Clock>(slice) {
            Ok(clock) => Box::into_raw(Box::new(clock)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// service::ServiceHeader
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_service_header_new() -> *mut service::ServiceHeader {
    Box::into_raw(Box::new(service::ServiceHeader { guid: 0, seq: 0 }))
}

#[no_mangle]
pub extern "C" fn ros_service_header_free(header: *mut service::ServiceHeader) {
    if !header.is_null() {
        unsafe {
            drop(Box::from_raw(header));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_service_header_get_guid(header: *const service::ServiceHeader) -> i64 {
    if header.is_null() {
        return 0;
    }
    unsafe { (*header).guid }
}

#[no_mangle]
pub extern "C" fn ros_service_header_get_seq(header: *const service::ServiceHeader) -> u64 {
    if header.is_null() {
        return 0;
    }
    unsafe { (*header).seq }
}

#[no_mangle]
pub extern "C" fn ros_service_header_set_guid(header: *mut service::ServiceHeader, guid: i64) {
    unsafe {
        assert!(!header.is_null());
        (*header).guid = guid;
    }
}

#[no_mangle]
pub extern "C" fn ros_service_header_set_seq(header: *mut service::ServiceHeader, seq: u64) {
    unsafe {
        assert!(!header.is_null());
        (*header).seq = seq;
    }
}

#[no_mangle]
pub extern "C" fn ros_service_header_serialize(
    header: *const service::ServiceHeader,
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
pub extern "C" fn ros_service_header_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut service::ServiceHeader {
    check_null_ret_null!(bytes);

    if len == 0 {
        set_errno(EINVAL);
        return ptr::null_mut();
    }

    unsafe {
        let slice = slice::from_raw_parts(bytes, len);
        match serde_cdr::deserialize::<service::ServiceHeader>(slice) {
            Ok(header) => Box::into_raw(Box::new(header)),
            Err(_) => {
                set_errno(EBADMSG);
                ptr::null_mut()
            }
        }
    }
}

// =============================================================================
// Schema Registry
// =============================================================================

use crate::schema_registry;

/// Check if a schema name is supported by this library.
///
/// # Arguments
/// * `schema` - The schema name to check (e.g., "sensor_msgs/msg/Image")
///
/// # Returns
/// * 1 if the schema is supported
/// * 0 if the schema is not supported or the input is NULL
///
/// # Example
/// ```c
/// if (edgefirst_schema_is_supported("sensor_msgs/msg/Image")) {
///     // Schema is supported
/// }
/// ```
#[no_mangle]
pub extern "C" fn edgefirst_schema_is_supported(schema: *const c_char) -> i32 {
    if schema.is_null() {
        return 0;
    }

    unsafe {
        match CStr::from_ptr(schema).to_str() {
            Ok(s) => {
                if schema_registry::is_supported(s) {
                    1
                } else {
                    0
                }
            }
            Err(_) => 0,
        }
    }
}

/// Get the number of supported schemas.
///
/// # Returns
/// The total number of supported schema types.
#[no_mangle]
pub extern "C" fn edgefirst_schema_count() -> usize {
    schema_registry::list_schemas().len()
}

/// Get a schema name by index.
///
/// # Arguments
/// * `index` - The index of the schema (0 to count-1)
///
/// # Returns
/// * Pointer to the schema name string (static lifetime, do not free)
/// * NULL if index is out of bounds
///
/// # Example
/// ```c
/// size_t count = edgefirst_schema_count();
/// for (size_t i = 0; i < count; i++) {
///     const char* name = edgefirst_schema_get(i);
///     printf("Schema %zu: %s\n", i, name);
/// }
/// ```
#[no_mangle]
pub extern "C" fn edgefirst_schema_get(index: usize) -> *const c_char {
    let schemas = schema_registry::list_schemas();
    if index >= schemas.len() {
        return ptr::null();
    }
    // Schema names are &'static str so we can return them directly
    schemas[index].as_ptr() as *const c_char
}
