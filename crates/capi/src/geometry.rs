// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! C FFI builders for geometry_msgs buffer-backed types.
#![allow(non_camel_case_types)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]
#![allow(clippy::too_many_arguments)]

use super::{c_to_str_checked, return_cdr_bytes, set_errno, EBADMSG, EINVAL, ENOBUFS};
use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::geometry_msgs::*;
use std::os::raw::c_char;
use std::slice;

/// Maximum CDR sequence length encoded as `u32` on the wire.
const MAX_CDR_SEQ_LEN: usize = u32::MAX as usize;

fn validate_elem_sequence(count: usize, elems_per_item: usize) -> Result<(), i32> {
    if count > MAX_CDR_SEQ_LEN {
        return Err(EINVAL);
    }
    let total = count.checked_mul(elems_per_item).ok_or(EINVAL)?;
    if total > isize::MAX as usize {
        return Err(EINVAL);
    }
    Ok(())
}

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

macro_rules! impl_builder_new_free {
    ($handle:ident, $owned:ty, $new:ident, $free:ident, $init:expr) => {
        pub struct $handle($owned);

        #[no_mangle]
        pub extern "C" fn $new() -> *mut $handle {
            Box::into_raw(Box::new($handle($init)))
        }

        #[no_mangle]
        pub extern "C" fn $free(b: *mut $handle) {
            if !b.is_null() {
                unsafe {
                    drop(Box::from_raw(b));
                }
            }
        }
    };
}

macro_rules! impl_builder_stamp_frame {
    ($handle:ident, $set_stamp:ident, $set_frame:ident) => {
        #[no_mangle]
        pub extern "C" fn $set_stamp(b: *mut $handle, sec: i32, nanosec: u32) {
            if b.is_null() {
                return;
            }
            let inner = unsafe { &mut (*b).0 };
            inner.stamp_sec = sec;
            inner.stamp_nanosec = nanosec;
        }

        #[no_mangle]
        pub extern "C" fn $set_frame(b: *mut $handle, s: *const c_char) -> i32 {
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
    };
}

struct AccelStampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    lx: f64,
    ly: f64,
    lz: f64,
    ax: f64,
    ay: f64,
    az: f64,
}

impl_builder_new_free!(
    geometry_msgs_accel_stamped_builder_t,
    AccelStampedBuilderOwned,
    geometry_msgs_accel_stamped_builder_new,
    geometry_msgs_accel_stamped_builder_free,
    AccelStampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        lx: 0.0,
        ly: 0.0,
        lz: 0.0,
        ax: 0.0,
        ay: 0.0,
        az: 0.0,
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_accel_stamped_builder_t,
    geometry_msgs_accel_stamped_builder_set_stamp,
    geometry_msgs_accel_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_accel_stamped_builder_set_linear_acceleration(
    b: *mut geometry_msgs_accel_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.lx = x;
    inner.ly = y;
    inner.lz = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_accel_stamped_builder_set_angular_acceleration(
    b: *mut geometry_msgs_accel_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.ax = x;
    inner.ay = y;
    inner.az = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_accel_stamped_builder_build(
    b: *mut geometry_msgs_accel_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = AccelStamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .accel(Accel {
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
        })
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_accel_stamped_builder_encode_into(
    b: *mut geometry_msgs_accel_stamped_builder_t,
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
    map_encode_into(
        AccelStamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .accel(Accel {
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
            })
            .encode_into_slice(dst),
        out_len,
    )
}

struct TwistStampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    lx: f64,
    ly: f64,
    lz: f64,
    ax: f64,
    ay: f64,
    az: f64,
}

impl_builder_new_free!(
    geometry_msgs_twist_stamped_builder_t,
    TwistStampedBuilderOwned,
    geometry_msgs_twist_stamped_builder_new,
    geometry_msgs_twist_stamped_builder_free,
    TwistStampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        lx: 0.0,
        ly: 0.0,
        lz: 0.0,
        ax: 0.0,
        ay: 0.0,
        az: 0.0,
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_twist_stamped_builder_t,
    geometry_msgs_twist_stamped_builder_set_stamp,
    geometry_msgs_twist_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_twist_stamped_builder_set_linear(
    b: *mut geometry_msgs_twist_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.lx = x;
    inner.ly = y;
    inner.lz = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_twist_stamped_builder_set_angular(
    b: *mut geometry_msgs_twist_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.ax = x;
    inner.ay = y;
    inner.az = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_twist_stamped_builder_build(
    b: *mut geometry_msgs_twist_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = TwistStamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .twist(Twist {
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
        })
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_twist_stamped_builder_encode_into(
    b: *mut geometry_msgs_twist_stamped_builder_t,
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
    map_encode_into(
        TwistStamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .twist(Twist {
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
            })
            .encode_into_slice(dst),
        out_len,
    )
}

struct WrenchStampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    fx: f64,
    fy: f64,
    fz: f64,
    tx: f64,
    ty: f64,
    tz: f64,
}

impl_builder_new_free!(
    geometry_msgs_wrench_stamped_builder_t,
    WrenchStampedBuilderOwned,
    geometry_msgs_wrench_stamped_builder_new,
    geometry_msgs_wrench_stamped_builder_free,
    WrenchStampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        fx: 0.0,
        fy: 0.0,
        fz: 0.0,
        tx: 0.0,
        ty: 0.0,
        tz: 0.0,
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_wrench_stamped_builder_t,
    geometry_msgs_wrench_stamped_builder_set_stamp,
    geometry_msgs_wrench_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_wrench_stamped_builder_set_force(
    b: *mut geometry_msgs_wrench_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.fx = x;
    inner.fy = y;
    inner.fz = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_wrench_stamped_builder_set_torque(
    b: *mut geometry_msgs_wrench_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.tx = x;
    inner.ty = y;
    inner.tz = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_wrench_stamped_builder_build(
    b: *mut geometry_msgs_wrench_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = WrenchStamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .wrench(Wrench {
            force: Vector3 {
                x: inner.fx,
                y: inner.fy,
                z: inner.fz,
            },
            torque: Vector3 {
                x: inner.tx,
                y: inner.ty,
                z: inner.tz,
            },
        })
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_wrench_stamped_builder_encode_into(
    b: *mut geometry_msgs_wrench_stamped_builder_t,
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
    map_encode_into(
        WrenchStamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .wrench(Wrench {
                force: Vector3 {
                    x: inner.fx,
                    y: inner.fy,
                    z: inner.fz,
                },
                torque: Vector3 {
                    x: inner.tx,
                    y: inner.ty,
                    z: inner.tz,
                },
            })
            .encode_into_slice(dst),
        out_len,
    )
}

struct PointStampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    x: f64,
    y: f64,
    z: f64,
}

impl_builder_new_free!(
    geometry_msgs_point_stamped_builder_t,
    PointStampedBuilderOwned,
    geometry_msgs_point_stamped_builder_new,
    geometry_msgs_point_stamped_builder_free,
    PointStampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_point_stamped_builder_t,
    geometry_msgs_point_stamped_builder_set_stamp,
    geometry_msgs_point_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_point_stamped_builder_set_point(
    b: *mut geometry_msgs_point_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.x = x;
    inner.y = y;
    inner.z = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_point_stamped_builder_build(
    b: *mut geometry_msgs_point_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = PointStamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .point(Point {
            x: inner.x,
            y: inner.y,
            z: inner.z,
        })
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_point_stamped_builder_encode_into(
    b: *mut geometry_msgs_point_stamped_builder_t,
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
    map_encode_into(
        PointStamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .point(Point {
                x: inner.x,
                y: inner.y,
                z: inner.z,
            })
            .encode_into_slice(dst),
        out_len,
    )
}

struct Vector3StampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    x: f64,
    y: f64,
    z: f64,
}

impl_builder_new_free!(
    geometry_msgs_vector3_stamped_builder_t,
    Vector3StampedBuilderOwned,
    geometry_msgs_vector3_stamped_builder_new,
    geometry_msgs_vector3_stamped_builder_free,
    Vector3StampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        x: 0.0,
        y: 0.0,
        z: 0.0,
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_vector3_stamped_builder_t,
    geometry_msgs_vector3_stamped_builder_set_stamp,
    geometry_msgs_vector3_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_vector3_stamped_builder_set_vector(
    b: *mut geometry_msgs_vector3_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.x = x;
    inner.y = y;
    inner.z = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_vector3_stamped_builder_build(
    b: *mut geometry_msgs_vector3_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = Vector3Stamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .vector(Vector3 {
            x: inner.x,
            y: inner.y,
            z: inner.z,
        })
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_vector3_stamped_builder_encode_into(
    b: *mut geometry_msgs_vector3_stamped_builder_t,
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
    map_encode_into(
        Vector3Stamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .vector(Vector3 {
                x: inner.x,
                y: inner.y,
                z: inner.z,
            })
            .encode_into_slice(dst),
        out_len,
    )
}

struct QuaternionStampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

impl_builder_new_free!(
    geometry_msgs_quaternion_stamped_builder_t,
    QuaternionStampedBuilderOwned,
    geometry_msgs_quaternion_stamped_builder_new,
    geometry_msgs_quaternion_stamped_builder_free,
    QuaternionStampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_quaternion_stamped_builder_t,
    geometry_msgs_quaternion_stamped_builder_set_stamp,
    geometry_msgs_quaternion_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_quaternion_stamped_builder_set_quaternion(
    b: *mut geometry_msgs_quaternion_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
    w: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.x = x;
    inner.y = y;
    inner.z = z;
    inner.w = w;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_quaternion_stamped_builder_build(
    b: *mut geometry_msgs_quaternion_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = QuaternionStamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .quaternion(Quaternion {
            x: inner.x,
            y: inner.y,
            z: inner.z,
            w: inner.w,
        })
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_quaternion_stamped_builder_encode_into(
    b: *mut geometry_msgs_quaternion_stamped_builder_t,
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
    map_encode_into(
        QuaternionStamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .quaternion(Quaternion {
                x: inner.x,
                y: inner.y,
                z: inner.z,
                w: inner.w,
            })
            .encode_into_slice(dst),
        out_len,
    )
}

struct PoseStampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    px: f64,
    py: f64,
    pz: f64,
    ox: f64,
    oy: f64,
    oz: f64,
    ow: f64,
}

impl_builder_new_free!(
    geometry_msgs_pose_stamped_builder_t,
    PoseStampedBuilderOwned,
    geometry_msgs_pose_stamped_builder_new,
    geometry_msgs_pose_stamped_builder_free,
    PoseStampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        px: 0.0,
        py: 0.0,
        pz: 0.0,
        ox: 0.0,
        oy: 0.0,
        oz: 0.0,
        ow: 1.0,
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_pose_stamped_builder_t,
    geometry_msgs_pose_stamped_builder_set_stamp,
    geometry_msgs_pose_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_pose_stamped_builder_set_position(
    b: *mut geometry_msgs_pose_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.px = x;
    inner.py = y;
    inner.pz = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_pose_stamped_builder_set_orientation(
    b: *mut geometry_msgs_pose_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
    w: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.ox = x;
    inner.oy = y;
    inner.oz = z;
    inner.ow = w;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_pose_stamped_builder_build(
    b: *mut geometry_msgs_pose_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = PoseStamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .pose(Pose {
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
        })
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_pose_stamped_builder_encode_into(
    b: *mut geometry_msgs_pose_stamped_builder_t,
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
    map_encode_into(
        PoseStamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .pose(Pose {
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
            })
            .encode_into_slice(dst),
        out_len,
    )
}

struct InertiaStampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    m: f64,
    cx: f64,
    cy: f64,
    cz: f64,
    ixx: f64,
    ixy: f64,
    ixz: f64,
    iyy: f64,
    iyz: f64,
    izz: f64,
}

impl_builder_new_free!(
    geometry_msgs_inertia_stamped_builder_t,
    InertiaStampedBuilderOwned,
    geometry_msgs_inertia_stamped_builder_new,
    geometry_msgs_inertia_stamped_builder_free,
    InertiaStampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        m: 0.0,
        cx: 0.0,
        cy: 0.0,
        cz: 0.0,
        ixx: 0.0,
        ixy: 0.0,
        ixz: 0.0,
        iyy: 0.0,
        iyz: 0.0,
        izz: 0.0,
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_inertia_stamped_builder_t,
    geometry_msgs_inertia_stamped_builder_set_stamp,
    geometry_msgs_inertia_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_inertia_stamped_builder_set_mass(
    b: *mut geometry_msgs_inertia_stamped_builder_t,
    m: f64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.m = m;
    }
}

#[no_mangle]
pub extern "C" fn geometry_msgs_inertia_stamped_builder_set_com(
    b: *mut geometry_msgs_inertia_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.cx = x;
    inner.cy = y;
    inner.cz = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_inertia_stamped_builder_set_inertia_tensor(
    b: *mut geometry_msgs_inertia_stamped_builder_t,
    ixx: f64,
    ixy: f64,
    ixz: f64,
    iyy: f64,
    iyz: f64,
    izz: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.ixx = ixx;
    inner.ixy = ixy;
    inner.ixz = ixz;
    inner.iyy = iyy;
    inner.iyz = iyz;
    inner.izz = izz;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_inertia_stamped_builder_build(
    b: *mut geometry_msgs_inertia_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = InertiaStamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .inertia(Inertia {
            m: inner.m,
            com: Vector3 {
                x: inner.cx,
                y: inner.cy,
                z: inner.cz,
            },
            ixx: inner.ixx,
            ixy: inner.ixy,
            ixz: inner.ixz,
            iyy: inner.iyy,
            iyz: inner.iyz,
            izz: inner.izz,
        })
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_inertia_stamped_builder_encode_into(
    b: *mut geometry_msgs_inertia_stamped_builder_t,
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
    map_encode_into(
        InertiaStamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .inertia(Inertia {
                m: inner.m,
                com: Vector3 {
                    x: inner.cx,
                    y: inner.cy,
                    z: inner.cz,
                },
                ixx: inner.ixx,
                ixy: inner.ixy,
                ixz: inner.ixz,
                iyy: inner.iyy,
                iyz: inner.iyz,
                izz: inner.izz,
            })
            .encode_into_slice(dst),
        out_len,
    )
}

struct TransformStampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    child_frame_id: String,
    tx: f64,
    ty: f64,
    tz: f64,
    rx: f64,
    ry: f64,
    rz: f64,
    rw: f64,
}

impl_builder_new_free!(
    geometry_msgs_transform_stamped_builder_t,
    TransformStampedBuilderOwned,
    geometry_msgs_transform_stamped_builder_new,
    geometry_msgs_transform_stamped_builder_free,
    TransformStampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        child_frame_id: String::new(),
        tx: 0.0,
        ty: 0.0,
        tz: 0.0,
        rx: 0.0,
        ry: 0.0,
        rz: 0.0,
        rw: 1.0,
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_transform_stamped_builder_t,
    geometry_msgs_transform_stamped_builder_set_stamp,
    geometry_msgs_transform_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_transform_stamped_builder_set_child_frame_id(
    b: *mut geometry_msgs_transform_stamped_builder_t,
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
pub extern "C" fn geometry_msgs_transform_stamped_builder_set_translation(
    b: *mut geometry_msgs_transform_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.tx = x;
    inner.ty = y;
    inner.tz = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_transform_stamped_builder_set_rotation(
    b: *mut geometry_msgs_transform_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
    w: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.rx = x;
    inner.ry = y;
    inner.rz = z;
    inner.rw = w;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_transform_stamped_builder_build(
    b: *mut geometry_msgs_transform_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = TransformStamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .child_frame_id(inner.child_frame_id.as_str())
        .transform(Transform {
            translation: Vector3 {
                x: inner.tx,
                y: inner.ty,
                z: inner.tz,
            },
            rotation: Quaternion {
                x: inner.rx,
                y: inner.ry,
                z: inner.rz,
                w: inner.rw,
            },
        })
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_transform_stamped_builder_encode_into(
    b: *mut geometry_msgs_transform_stamped_builder_t,
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
    map_encode_into(
        TransformStamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .child_frame_id(inner.child_frame_id.as_str())
            .transform(Transform {
                translation: Vector3 {
                    x: inner.tx,
                    y: inner.ty,
                    z: inner.tz,
                },
                rotation: Quaternion {
                    x: inner.rx,
                    y: inner.ry,
                    z: inner.rz,
                    w: inner.rw,
                },
            })
            .encode_into_slice(dst),
        out_len,
    )
}

fn copy_cov36(dst: &mut [f64; 36], cov: *const f64) -> i32 {
    if cov.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        dst.copy_from_slice(slice::from_raw_parts(cov, 36));
    }
    0
}

struct PoseWithCovarianceStampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    px: f64,
    py: f64,
    pz: f64,
    ox: f64,
    oy: f64,
    oz: f64,
    ow: f64,
    covariance: [f64; 36],
}

impl_builder_new_free!(
    geometry_msgs_pose_with_covariance_stamped_builder_t,
    PoseWithCovarianceStampedBuilderOwned,
    geometry_msgs_pose_with_covariance_stamped_builder_new,
    geometry_msgs_pose_with_covariance_stamped_builder_free,
    PoseWithCovarianceStampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        px: 0.0,
        py: 0.0,
        pz: 0.0,
        ox: 0.0,
        oy: 0.0,
        oz: 0.0,
        ow: 1.0,
        covariance: [0.0; 36],
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_pose_with_covariance_stamped_builder_t,
    geometry_msgs_pose_with_covariance_stamped_builder_set_stamp,
    geometry_msgs_pose_with_covariance_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_pose_with_covariance_stamped_builder_set_position(
    b: *mut geometry_msgs_pose_with_covariance_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.px = x;
    inner.py = y;
    inner.pz = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_pose_with_covariance_stamped_builder_set_orientation(
    b: *mut geometry_msgs_pose_with_covariance_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
    w: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.ox = x;
    inner.oy = y;
    inner.oz = z;
    inner.ow = w;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_pose_with_covariance_stamped_builder_set_covariance(
    b: *mut geometry_msgs_pose_with_covariance_stamped_builder_t,
    cov: *const f64,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    copy_cov36(&mut unsafe { &mut (*b).0 }.covariance, cov)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_pose_with_covariance_stamped_builder_build(
    b: *mut geometry_msgs_pose_with_covariance_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = PoseWithCovarianceStamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .pose_with_covariance(PoseWithCovariance {
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
            covariance: inner.covariance,
        })
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_pose_with_covariance_stamped_builder_encode_into(
    b: *mut geometry_msgs_pose_with_covariance_stamped_builder_t,
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
    map_encode_into(
        PoseWithCovarianceStamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .pose_with_covariance(PoseWithCovariance {
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
                covariance: inner.covariance,
            })
            .encode_into_slice(dst),
        out_len,
    )
}

struct TwistWithCovarianceStampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    lx: f64,
    ly: f64,
    lz: f64,
    ax: f64,
    ay: f64,
    az: f64,
    covariance: [f64; 36],
}

impl_builder_new_free!(
    geometry_msgs_twist_with_covariance_stamped_builder_t,
    TwistWithCovarianceStampedBuilderOwned,
    geometry_msgs_twist_with_covariance_stamped_builder_new,
    geometry_msgs_twist_with_covariance_stamped_builder_free,
    TwistWithCovarianceStampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        lx: 0.0,
        ly: 0.0,
        lz: 0.0,
        ax: 0.0,
        ay: 0.0,
        az: 0.0,
        covariance: [0.0; 36],
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_twist_with_covariance_stamped_builder_t,
    geometry_msgs_twist_with_covariance_stamped_builder_set_stamp,
    geometry_msgs_twist_with_covariance_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_twist_with_covariance_stamped_builder_set_linear(
    b: *mut geometry_msgs_twist_with_covariance_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.lx = x;
    inner.ly = y;
    inner.lz = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_twist_with_covariance_stamped_builder_set_angular(
    b: *mut geometry_msgs_twist_with_covariance_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.ax = x;
    inner.ay = y;
    inner.az = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_twist_with_covariance_stamped_builder_set_covariance(
    b: *mut geometry_msgs_twist_with_covariance_stamped_builder_t,
    cov: *const f64,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    copy_cov36(&mut unsafe { &mut (*b).0 }.covariance, cov)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_twist_with_covariance_stamped_builder_build(
    b: *mut geometry_msgs_twist_with_covariance_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = TwistWithCovarianceStamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .twist_with_covariance(TwistWithCovariance {
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
            covariance: inner.covariance,
        })
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_twist_with_covariance_stamped_builder_encode_into(
    b: *mut geometry_msgs_twist_with_covariance_stamped_builder_t,
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
    map_encode_into(
        TwistWithCovarianceStamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .twist_with_covariance(TwistWithCovariance {
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
                covariance: inner.covariance,
            })
            .encode_into_slice(dst),
        out_len,
    )
}

struct AccelWithCovarianceStampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    lx: f64,
    ly: f64,
    lz: f64,
    ax: f64,
    ay: f64,
    az: f64,
    covariance: [f64; 36],
}

impl_builder_new_free!(
    geometry_msgs_accel_with_covariance_stamped_builder_t,
    AccelWithCovarianceStampedBuilderOwned,
    geometry_msgs_accel_with_covariance_stamped_builder_new,
    geometry_msgs_accel_with_covariance_stamped_builder_free,
    AccelWithCovarianceStampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        lx: 0.0,
        ly: 0.0,
        lz: 0.0,
        ax: 0.0,
        ay: 0.0,
        az: 0.0,
        covariance: [0.0; 36],
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_accel_with_covariance_stamped_builder_t,
    geometry_msgs_accel_with_covariance_stamped_builder_set_stamp,
    geometry_msgs_accel_with_covariance_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_accel_with_covariance_stamped_builder_set_linear_acceleration(
    b: *mut geometry_msgs_accel_with_covariance_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.lx = x;
    inner.ly = y;
    inner.lz = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_accel_with_covariance_stamped_builder_set_angular_acceleration(
    b: *mut geometry_msgs_accel_with_covariance_stamped_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.ax = x;
    inner.ay = y;
    inner.az = z;
}

#[no_mangle]
pub extern "C" fn geometry_msgs_accel_with_covariance_stamped_builder_set_covariance(
    b: *mut geometry_msgs_accel_with_covariance_stamped_builder_t,
    cov: *const f64,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    copy_cov36(&mut unsafe { &mut (*b).0 }.covariance, cov)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_accel_with_covariance_stamped_builder_build(
    b: *mut geometry_msgs_accel_with_covariance_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = AccelWithCovarianceStamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .accel_with_covariance(AccelWithCovariance {
            accel: Accel {
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
            covariance: inner.covariance,
        })
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_accel_with_covariance_stamped_builder_encode_into(
    b: *mut geometry_msgs_accel_with_covariance_stamped_builder_t,
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
    map_encode_into(
        AccelWithCovarianceStamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .accel_with_covariance(AccelWithCovariance {
                accel: Accel {
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
                covariance: inner.covariance,
            })
            .encode_into_slice(dst),
        out_len,
    )
}

struct PolygonBuilderOwned {
    xyz: *const f32,
    count: usize,
}

impl_builder_new_free!(
    geometry_msgs_polygon_builder_t,
    PolygonBuilderOwned,
    geometry_msgs_polygon_builder_new,
    geometry_msgs_polygon_builder_free,
    PolygonBuilderOwned {
        xyz: std::ptr::null(),
        count: 0,
    }
);

#[no_mangle]
pub extern "C" fn geometry_msgs_polygon_builder_set_points(
    b: *mut geometry_msgs_polygon_builder_t,
    xyz: *const f32,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if xyz.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    if count > 0 {
        if let Err(e) = validate_elem_sequence(count, 3) {
            set_errno(e);
            return -1;
        }
    }
    unsafe {
        (*b).0.xyz = xyz;
        (*b).0.count = count;
    }
    0
}

fn polygon_points(xyz: *const f32, count: usize) -> Vec<Point32> {
    if xyz.is_null() || count == 0 {
        return Vec::new();
    }
    if validate_elem_sequence(count, 3).is_err() {
        return Vec::new();
    }
    let src = unsafe { slice::from_raw_parts(xyz, count * 3) };
    src.as_chunks::<3>()
        .0
        .iter()
        .map(|c| Point32 {
            x: c[0],
            y: c[1],
            z: c[2],
        })
        .collect()
}

#[no_mangle]
pub extern "C" fn geometry_msgs_polygon_builder_build(
    b: *mut geometry_msgs_polygon_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = Polygon::builder()
        .points(&polygon_points(inner.xyz, inner.count))
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_polygon_builder_encode_into(
    b: *mut geometry_msgs_polygon_builder_t,
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
    map_encode_into(
        Polygon::builder()
            .points(&polygon_points(inner.xyz, inner.count))
            .encode_into_slice(dst),
        out_len,
    )
}

struct PolygonStampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    xyz: *const f32,
    count: usize,
}

impl_builder_new_free!(
    geometry_msgs_polygon_stamped_builder_t,
    PolygonStampedBuilderOwned,
    geometry_msgs_polygon_stamped_builder_new,
    geometry_msgs_polygon_stamped_builder_free,
    PolygonStampedBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        xyz: std::ptr::null(),
        count: 0,
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_polygon_stamped_builder_t,
    geometry_msgs_polygon_stamped_builder_set_stamp,
    geometry_msgs_polygon_stamped_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_polygon_stamped_builder_set_points(
    b: *mut geometry_msgs_polygon_stamped_builder_t,
    xyz: *const f32,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if xyz.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.xyz = xyz;
        (*b).0.count = count;
    }
    0
}

#[no_mangle]
pub extern "C" fn geometry_msgs_polygon_stamped_builder_build(
    b: *mut geometry_msgs_polygon_stamped_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = PolygonStamped::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .points(&polygon_points(inner.xyz, inner.count))
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_polygon_stamped_builder_encode_into(
    b: *mut geometry_msgs_polygon_stamped_builder_t,
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
    map_encode_into(
        PolygonStamped::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .points(&polygon_points(inner.xyz, inner.count))
            .encode_into_slice(dst),
        out_len,
    )
}

struct PoseArrayBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    poses: *const f64,
    count: usize,
}

impl_builder_new_free!(
    geometry_msgs_pose_array_builder_t,
    PoseArrayBuilderOwned,
    geometry_msgs_pose_array_builder_new,
    geometry_msgs_pose_array_builder_free,
    PoseArrayBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        poses: std::ptr::null(),
        count: 0,
    }
);
impl_builder_stamp_frame!(
    geometry_msgs_pose_array_builder_t,
    geometry_msgs_pose_array_builder_set_stamp,
    geometry_msgs_pose_array_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn geometry_msgs_pose_array_builder_set_poses(
    b: *mut geometry_msgs_pose_array_builder_t,
    poses: *const f64,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if poses.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    if count > 0 {
        if let Err(e) = validate_elem_sequence(count, 7) {
            set_errno(e);
            return -1;
        }
    }
    unsafe {
        (*b).0.poses = poses;
        (*b).0.count = count;
    }
    0
}

fn pose_array_poses(poses: *const f64, count: usize) -> Vec<Pose> {
    if poses.is_null() || count == 0 {
        return Vec::new();
    }
    if validate_elem_sequence(count, 7).is_err() {
        return Vec::new();
    }
    let src = unsafe { slice::from_raw_parts(poses, count * 7) };
    src.as_chunks::<7>()
        .0
        .iter()
        .map(|c| Pose {
            position: Point {
                x: c[0],
                y: c[1],
                z: c[2],
            },
            orientation: Quaternion {
                x: c[3],
                y: c[4],
                z: c[5],
                w: c[6],
            },
        })
        .collect()
}

#[no_mangle]
pub extern "C" fn geometry_msgs_pose_array_builder_build(
    b: *mut geometry_msgs_pose_array_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = PoseArray::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .poses(&pose_array_poses(inner.poses, inner.count))
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn geometry_msgs_pose_array_builder_encode_into(
    b: *mut geometry_msgs_pose_array_builder_t,
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
    map_encode_into(
        PoseArray::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .poses(&pose_array_poses(inner.poses, inner.count))
            .encode_into_slice(dst),
        out_len,
    )
}
