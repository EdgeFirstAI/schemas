// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! C FFI builder for nav_msgs/Odometry.
#![allow(non_camel_case_types)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]
#![allow(clippy::too_many_arguments)]

use super::{c_to_str_checked, return_cdr_bytes, set_errno, EBADMSG, EINVAL, ENOBUFS};
use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::geometry_msgs::{
    Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3,
};
use edgefirst_schemas::nav_msgs::Odometry;
use std::os::raw::c_char;
use std::slice;

fn map_build<T>(
    r: Result<T, edgefirst_schemas::cdr::CdrError>,
    into: impl FnOnce(T) -> Vec<u8>,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    match r {
        Ok(v) => return_cdr_bytes(into(v), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

fn map_encode_into(r: Result<usize, edgefirst_schemas::cdr::CdrError>, out_len: *mut usize) -> i32 {
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(edgefirst_schemas::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

struct OdometryBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    child_frame_id: String,
    px: f64,
    py: f64,
    pz: f64,
    ox: f64,
    oy: f64,
    oz: f64,
    ow: f64,
    pose_cov: [f64; 36],
    lx: f64,
    ly: f64,
    lz: f64,
    ax: f64,
    ay: f64,
    az: f64,
    twist_cov: [f64; 36],
}

pub struct nav_msgs_odometry_builder_t(OdometryBuilderOwned);

#[no_mangle]
pub extern "C" fn nav_msgs_odometry_builder_new() -> *mut nav_msgs_odometry_builder_t {
    Box::into_raw(Box::new(nav_msgs_odometry_builder_t(
        OdometryBuilderOwned {
            stamp_sec: 0,
            stamp_nanosec: 0,
            frame_id: String::new(),
            child_frame_id: String::new(),
            px: 0.0,
            py: 0.0,
            pz: 0.0,
            ox: 0.0,
            oy: 0.0,
            oz: 0.0,
            ow: 1.0,
            pose_cov: [0.0; 36],
            lx: 0.0,
            ly: 0.0,
            lz: 0.0,
            ax: 0.0,
            ay: 0.0,
            az: 0.0,
            twist_cov: [0.0; 36],
        },
    )))
}

#[no_mangle]
pub extern "C" fn nav_msgs_odometry_builder_free(b: *mut nav_msgs_odometry_builder_t) {
    if !b.is_null() {
        unsafe {
            drop(Box::from_raw(b));
        }
    }
}

#[no_mangle]
pub extern "C" fn nav_msgs_odometry_builder_set_stamp(
    b: *mut nav_msgs_odometry_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.stamp_sec = sec;
    inner.stamp_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn nav_msgs_odometry_builder_set_frame_id(
    b: *mut nav_msgs_odometry_builder_t,
    s: *const c_char,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let s_str = match unsafe { c_to_str_checked(s) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    unsafe {
        (*b).0.frame_id = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn nav_msgs_odometry_builder_set_child_frame_id(
    b: *mut nav_msgs_odometry_builder_t,
    s: *const c_char,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let s_str = match unsafe { c_to_str_checked(s) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    unsafe {
        (*b).0.child_frame_id = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn nav_msgs_odometry_builder_set_pose(
    b: *mut nav_msgs_odometry_builder_t,
    px: f64,
    py: f64,
    pz: f64,
    ox: f64,
    oy: f64,
    oz: f64,
    ow: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.px = px;
    inner.py = py;
    inner.pz = pz;
    inner.ox = ox;
    inner.oy = oy;
    inner.oz = oz;
    inner.ow = ow;
}

#[no_mangle]
pub extern "C" fn nav_msgs_odometry_builder_set_pose_covariance(
    b: *mut nav_msgs_odometry_builder_t,
    cov: *const f64,
) -> i32 {
    if b.is_null() || cov.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0
            .pose_cov
            .copy_from_slice(slice::from_raw_parts(cov, 36));
    }
    0
}

#[no_mangle]
pub extern "C" fn nav_msgs_odometry_builder_set_twist(
    b: *mut nav_msgs_odometry_builder_t,
    lx: f64,
    ly: f64,
    lz: f64,
    ax: f64,
    ay: f64,
    az: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.lx = lx;
    inner.ly = ly;
    inner.lz = lz;
    inner.ax = ax;
    inner.ay = ay;
    inner.az = az;
}

#[no_mangle]
pub extern "C" fn nav_msgs_odometry_builder_set_twist_covariance(
    b: *mut nav_msgs_odometry_builder_t,
    cov: *const f64,
) -> i32 {
    if b.is_null() || cov.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0
            .twist_cov
            .copy_from_slice(slice::from_raw_parts(cov, 36));
    }
    0
}

fn odometry_builder(
    inner: &OdometryBuilderOwned,
) -> edgefirst_schemas::nav_msgs::OdometryBuilder<'_> {
    let mut b = Odometry::builder();
    b.stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .child_frame_id(inner.child_frame_id.as_str())
        .pose(PoseWithCovariance {
            pose: Pose {
                position: Point {
                    x: inner.px,
                    y: inner.py,
                    z: inner.pz,
                },
                orientation: Quaternion {
                    x: inner.ox,
                    y: inner.oy,
                    z: inner.oz,
                    w: inner.ow,
                },
            },
            covariance: inner.pose_cov,
        })
        .twist(TwistWithCovariance {
            twist: Twist {
                linear: Vector3 {
                    x: inner.lx,
                    y: inner.ly,
                    z: inner.lz,
                },
                angular: Vector3 {
                    x: inner.ax,
                    y: inner.ay,
                    z: inner.az,
                },
            },
            covariance: inner.twist_cov,
        });
    b
}

#[no_mangle]
pub extern "C" fn nav_msgs_odometry_builder_build(
    b: *mut nav_msgs_odometry_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    map_build(
        odometry_builder(inner).build(),
        |v| v.into_cdr(),
        out_bytes,
        out_len,
    )
}

#[no_mangle]
pub extern "C" fn nav_msgs_odometry_builder_encode_into(
    b: *mut nav_msgs_odometry_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    map_encode_into(odometry_builder(inner).encode_into_slice(dst), out_len)
}
