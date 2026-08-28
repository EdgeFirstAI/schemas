// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! C FFI builders for mavros_msgs buffer-backed types.
#![allow(non_camel_case_types)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]
#![allow(clippy::too_many_arguments)]

use super::{c_to_str_checked, return_cdr_bytes, set_errno, EBADMSG, EINVAL, ENOBUFS};
use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::mavros_msgs::*;
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
struct AltitudeBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    monotonic: f32,
    amsl: f32,
    local: f32,
    relative: f32,
    terrain: f32,
    bottom_clearance: f32,
}
impl_builder_new_free!(
    mavros_msgs_altitude_builder_t,
    AltitudeBuilderOwned,
    mavros_msgs_altitude_builder_new,
    mavros_msgs_altitude_builder_free,
    AltitudeBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        monotonic: 0.0,
        amsl: 0.0,
        local: 0.0,
        relative: 0.0,
        terrain: 0.0,
        bottom_clearance: 0.0,
    }
);
impl_builder_stamp_frame!(
    mavros_msgs_altitude_builder_t,
    mavros_msgs_altitude_builder_set_stamp,
    mavros_msgs_altitude_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn mavros_msgs_altitude_builder_set_monotonic(
    b: *mut mavros_msgs_altitude_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.monotonic = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_altitude_builder_set_amsl(
    b: *mut mavros_msgs_altitude_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.amsl = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_altitude_builder_set_local(
    b: *mut mavros_msgs_altitude_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.local = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_altitude_builder_set_relative(
    b: *mut mavros_msgs_altitude_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.relative = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_altitude_builder_set_terrain(
    b: *mut mavros_msgs_altitude_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.terrain = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_altitude_builder_set_bottom_clearance(
    b: *mut mavros_msgs_altitude_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.bottom_clearance = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_altitude_builder_build(
    b: *mut mavros_msgs_altitude_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = Altitude::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .monotonic(inner.monotonic)
        .amsl(inner.amsl)
        .local(inner.local)
        .relative(inner.relative)
        .terrain(inner.terrain)
        .bottom_clearance(inner.bottom_clearance)
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn mavros_msgs_altitude_builder_encode_into(
    b: *mut mavros_msgs_altitude_builder_t,
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
        Altitude::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .monotonic(inner.monotonic)
            .amsl(inner.amsl)
            .local(inner.local)
            .relative(inner.relative)
            .terrain(inner.terrain)
            .bottom_clearance(inner.bottom_clearance)
            .encode_into_slice(dst),
        out_len,
    )
}

struct VfrHudBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    airspeed: f32,
    groundspeed: f32,
    heading: i16,
    throttle: f32,
    altitude: f32,
    climb: f32,
}
impl_builder_new_free!(
    mavros_msgs_vfrhud_builder_t,
    VfrHudBuilderOwned,
    mavros_msgs_vfrhud_builder_new,
    mavros_msgs_vfrhud_builder_free,
    VfrHudBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        airspeed: 0.0,
        groundspeed: 0.0,
        heading: 0,
        throttle: 0.0,
        altitude: 0.0,
        climb: 0.0,
    }
);
impl_builder_stamp_frame!(
    mavros_msgs_vfrhud_builder_t,
    mavros_msgs_vfrhud_builder_set_stamp,
    mavros_msgs_vfrhud_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn mavros_msgs_vfrhud_builder_set_airspeed(
    b: *mut mavros_msgs_vfrhud_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.airspeed = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_vfrhud_builder_set_groundspeed(
    b: *mut mavros_msgs_vfrhud_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.groundspeed = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_vfrhud_builder_set_heading(
    b: *mut mavros_msgs_vfrhud_builder_t,
    v: i16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.heading = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_vfrhud_builder_set_throttle(
    b: *mut mavros_msgs_vfrhud_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.throttle = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_vfrhud_builder_set_altitude(
    b: *mut mavros_msgs_vfrhud_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.altitude = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_vfrhud_builder_set_climb(
    b: *mut mavros_msgs_vfrhud_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.climb = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_vfrhud_builder_build(
    b: *mut mavros_msgs_vfrhud_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = VfrHud::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .airspeed(inner.airspeed)
        .groundspeed(inner.groundspeed)
        .heading(inner.heading)
        .throttle(inner.throttle)
        .altitude(inner.altitude)
        .climb(inner.climb)
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn mavros_msgs_vfrhud_builder_encode_into(
    b: *mut mavros_msgs_vfrhud_builder_t,
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
        VfrHud::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .airspeed(inner.airspeed)
            .groundspeed(inner.groundspeed)
            .heading(inner.heading)
            .throttle(inner.throttle)
            .altitude(inner.altitude)
            .climb(inner.climb)
            .encode_into_slice(dst),
        out_len,
    )
}

struct EstimatorStatusBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    attitude_status_flag: bool,
    velocity_horiz_status_flag: bool,
    velocity_vert_status_flag: bool,
    pos_horiz_rel_status_flag: bool,
    pos_horiz_abs_status_flag: bool,
    pos_vert_abs_status_flag: bool,
    pos_vert_agl_status_flag: bool,
    const_pos_mode_status_flag: bool,
    pred_pos_horiz_rel_status_flag: bool,
    pred_pos_horiz_abs_status_flag: bool,
    gps_glitch_status_flag: bool,
    accel_error_status_flag: bool,
}
impl_builder_new_free!(
    mavros_msgs_estimator_status_builder_t,
    EstimatorStatusBuilderOwned,
    mavros_msgs_estimator_status_builder_new,
    mavros_msgs_estimator_status_builder_free,
    EstimatorStatusBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        attitude_status_flag: false,
        velocity_horiz_status_flag: false,
        velocity_vert_status_flag: false,
        pos_horiz_rel_status_flag: false,
        pos_horiz_abs_status_flag: false,
        pos_vert_abs_status_flag: false,
        pos_vert_agl_status_flag: false,
        const_pos_mode_status_flag: false,
        pred_pos_horiz_rel_status_flag: false,
        pred_pos_horiz_abs_status_flag: false,
        gps_glitch_status_flag: false,
        accel_error_status_flag: false,
    }
);
impl_builder_stamp_frame!(
    mavros_msgs_estimator_status_builder_t,
    mavros_msgs_estimator_status_builder_set_stamp,
    mavros_msgs_estimator_status_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_set_attitude_status_flag(
    b: *mut mavros_msgs_estimator_status_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.attitude_status_flag = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_set_velocity_horiz_status_flag(
    b: *mut mavros_msgs_estimator_status_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.velocity_horiz_status_flag = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_set_velocity_vert_status_flag(
    b: *mut mavros_msgs_estimator_status_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.velocity_vert_status_flag = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_set_pos_horiz_rel_status_flag(
    b: *mut mavros_msgs_estimator_status_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.pos_horiz_rel_status_flag = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_set_pos_horiz_abs_status_flag(
    b: *mut mavros_msgs_estimator_status_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.pos_horiz_abs_status_flag = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_set_pos_vert_abs_status_flag(
    b: *mut mavros_msgs_estimator_status_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.pos_vert_abs_status_flag = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_set_pos_vert_agl_status_flag(
    b: *mut mavros_msgs_estimator_status_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.pos_vert_agl_status_flag = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_set_const_pos_mode_status_flag(
    b: *mut mavros_msgs_estimator_status_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.const_pos_mode_status_flag = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_set_pred_pos_horiz_rel_status_flag(
    b: *mut mavros_msgs_estimator_status_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.pred_pos_horiz_rel_status_flag = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_set_pred_pos_horiz_abs_status_flag(
    b: *mut mavros_msgs_estimator_status_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.pred_pos_horiz_abs_status_flag = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_set_gps_glitch_status_flag(
    b: *mut mavros_msgs_estimator_status_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.gps_glitch_status_flag = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_set_accel_error_status_flag(
    b: *mut mavros_msgs_estimator_status_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.accel_error_status_flag = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_build(
    b: *mut mavros_msgs_estimator_status_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = EstimatorStatus::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .attitude_status_flag(inner.attitude_status_flag)
        .velocity_horiz_status_flag(inner.velocity_horiz_status_flag)
        .velocity_vert_status_flag(inner.velocity_vert_status_flag)
        .pos_horiz_rel_status_flag(inner.pos_horiz_rel_status_flag)
        .pos_horiz_abs_status_flag(inner.pos_horiz_abs_status_flag)
        .pos_vert_abs_status_flag(inner.pos_vert_abs_status_flag)
        .pos_vert_agl_status_flag(inner.pos_vert_agl_status_flag)
        .const_pos_mode_status_flag(inner.const_pos_mode_status_flag)
        .pred_pos_horiz_rel_status_flag(inner.pred_pos_horiz_rel_status_flag)
        .pred_pos_horiz_abs_status_flag(inner.pred_pos_horiz_abs_status_flag)
        .gps_glitch_status_flag(inner.gps_glitch_status_flag)
        .accel_error_status_flag(inner.accel_error_status_flag)
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn mavros_msgs_estimator_status_builder_encode_into(
    b: *mut mavros_msgs_estimator_status_builder_t,
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
        EstimatorStatus::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .attitude_status_flag(inner.attitude_status_flag)
            .velocity_horiz_status_flag(inner.velocity_horiz_status_flag)
            .velocity_vert_status_flag(inner.velocity_vert_status_flag)
            .pos_horiz_rel_status_flag(inner.pos_horiz_rel_status_flag)
            .pos_horiz_abs_status_flag(inner.pos_horiz_abs_status_flag)
            .pos_vert_abs_status_flag(inner.pos_vert_abs_status_flag)
            .pos_vert_agl_status_flag(inner.pos_vert_agl_status_flag)
            .const_pos_mode_status_flag(inner.const_pos_mode_status_flag)
            .pred_pos_horiz_rel_status_flag(inner.pred_pos_horiz_rel_status_flag)
            .pred_pos_horiz_abs_status_flag(inner.pred_pos_horiz_abs_status_flag)
            .gps_glitch_status_flag(inner.gps_glitch_status_flag)
            .accel_error_status_flag(inner.accel_error_status_flag)
            .encode_into_slice(dst),
        out_len,
    )
}

struct ExtendedStateBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    vtol_state: u8,
    landed_state: u8,
}
impl_builder_new_free!(
    mavros_msgs_extended_state_builder_t,
    ExtendedStateBuilderOwned,
    mavros_msgs_extended_state_builder_new,
    mavros_msgs_extended_state_builder_free,
    ExtendedStateBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        vtol_state: 0,
        landed_state: 0,
    }
);
impl_builder_stamp_frame!(
    mavros_msgs_extended_state_builder_t,
    mavros_msgs_extended_state_builder_set_stamp,
    mavros_msgs_extended_state_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn mavros_msgs_extended_state_builder_set_vtol_state(
    b: *mut mavros_msgs_extended_state_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.vtol_state = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_extended_state_builder_set_landed_state(
    b: *mut mavros_msgs_extended_state_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.landed_state = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_extended_state_builder_build(
    b: *mut mavros_msgs_extended_state_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = ExtendedState::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .vtol_state(inner.vtol_state)
        .landed_state(inner.landed_state)
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn mavros_msgs_extended_state_builder_encode_into(
    b: *mut mavros_msgs_extended_state_builder_t,
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
        ExtendedState::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .vtol_state(inner.vtol_state)
            .landed_state(inner.landed_state)
            .encode_into_slice(dst),
        out_len,
    )
}

struct SysStatusBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    sensors_present: u32,
    sensors_enabled: u32,
    sensors_health: u32,
    load: u16,
    voltage_battery: u16,
    current_battery: i16,
    battery_remaining: i8,
    drop_rate_comm: u16,
    errors_comm: u16,
    errors_count1: u16,
    errors_count2: u16,
    errors_count3: u16,
    errors_count4: u16,
}
impl_builder_new_free!(
    mavros_msgs_sys_status_builder_t,
    SysStatusBuilderOwned,
    mavros_msgs_sys_status_builder_new,
    mavros_msgs_sys_status_builder_free,
    SysStatusBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        sensors_present: 0,
        sensors_enabled: 0,
        sensors_health: 0,
        load: 0,
        voltage_battery: 0,
        current_battery: 0,
        battery_remaining: 0,
        drop_rate_comm: 0,
        errors_comm: 0,
        errors_count1: 0,
        errors_count2: 0,
        errors_count3: 0,
        errors_count4: 0,
    }
);
impl_builder_stamp_frame!(
    mavros_msgs_sys_status_builder_t,
    mavros_msgs_sys_status_builder_set_stamp,
    mavros_msgs_sys_status_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_sensors_present(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: u32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.sensors_present = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_sensors_enabled(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: u32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.sensors_enabled = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_sensors_health(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: u32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.sensors_health = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_load(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.load = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_voltage_battery(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.voltage_battery = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_current_battery(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: i16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.current_battery = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_battery_remaining(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: i8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.battery_remaining = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_drop_rate_comm(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.drop_rate_comm = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_errors_comm(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.errors_comm = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_errors_count1(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.errors_count1 = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_errors_count2(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.errors_count2 = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_errors_count3(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.errors_count3 = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_set_errors_count4(
    b: *mut mavros_msgs_sys_status_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.errors_count4 = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_build(
    b: *mut mavros_msgs_sys_status_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = SysStatus::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .sensors_present(inner.sensors_present)
        .sensors_enabled(inner.sensors_enabled)
        .sensors_health(inner.sensors_health)
        .load(inner.load)
        .voltage_battery(inner.voltage_battery)
        .current_battery(inner.current_battery)
        .battery_remaining(inner.battery_remaining)
        .drop_rate_comm(inner.drop_rate_comm)
        .errors_comm(inner.errors_comm)
        .errors_count1(inner.errors_count1)
        .errors_count2(inner.errors_count2)
        .errors_count3(inner.errors_count3)
        .errors_count4(inner.errors_count4)
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn mavros_msgs_sys_status_builder_encode_into(
    b: *mut mavros_msgs_sys_status_builder_t,
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
        SysStatus::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .sensors_present(inner.sensors_present)
            .sensors_enabled(inner.sensors_enabled)
            .sensors_health(inner.sensors_health)
            .load(inner.load)
            .voltage_battery(inner.voltage_battery)
            .current_battery(inner.current_battery)
            .battery_remaining(inner.battery_remaining)
            .drop_rate_comm(inner.drop_rate_comm)
            .errors_comm(inner.errors_comm)
            .errors_count1(inner.errors_count1)
            .errors_count2(inner.errors_count2)
            .errors_count3(inner.errors_count3)
            .errors_count4(inner.errors_count4)
            .encode_into_slice(dst),
        out_len,
    )
}

struct StateBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    connected: bool,
    armed: bool,
    guided: bool,
    manual_input: bool,
    mode: String,
    system_status: u8,
}
impl_builder_new_free!(
    mavros_msgs_state_builder_t,
    StateBuilderOwned,
    mavros_msgs_state_builder_new,
    mavros_msgs_state_builder_free,
    StateBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        connected: false,
        armed: false,
        guided: false,
        manual_input: false,
        mode: String::new(),
        system_status: 0,
    }
);
impl_builder_stamp_frame!(
    mavros_msgs_state_builder_t,
    mavros_msgs_state_builder_set_stamp,
    mavros_msgs_state_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn mavros_msgs_state_builder_set_connected(
    b: *mut mavros_msgs_state_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.connected = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_state_builder_set_armed(
    b: *mut mavros_msgs_state_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.armed = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_state_builder_set_guided(
    b: *mut mavros_msgs_state_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.guided = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_state_builder_set_manual_input(
    b: *mut mavros_msgs_state_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.manual_input = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_state_builder_set_mode(
    b: *mut mavros_msgs_state_builder_t,
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
        (*b).0.mode = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn mavros_msgs_state_builder_set_system_status(
    b: *mut mavros_msgs_state_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.system_status = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_state_builder_build(
    b: *mut mavros_msgs_state_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = State::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .connected(inner.connected)
        .armed(inner.armed)
        .guided(inner.guided)
        .manual_input(inner.manual_input)
        .mode(inner.mode.as_str())
        .system_status(inner.system_status)
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn mavros_msgs_state_builder_encode_into(
    b: *mut mavros_msgs_state_builder_t,
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
        State::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .connected(inner.connected)
            .armed(inner.armed)
            .guided(inner.guided)
            .manual_input(inner.manual_input)
            .mode(inner.mode.as_str())
            .system_status(inner.system_status)
            .encode_into_slice(dst),
        out_len,
    )
}

struct StatusTextBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    severity: u8,
    text: String,
}
impl_builder_new_free!(
    mavros_msgs_status_text_builder_t,
    StatusTextBuilderOwned,
    mavros_msgs_status_text_builder_new,
    mavros_msgs_status_text_builder_free,
    StatusTextBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        severity: 0,
        text: String::new(),
    }
);
impl_builder_stamp_frame!(
    mavros_msgs_status_text_builder_t,
    mavros_msgs_status_text_builder_set_stamp,
    mavros_msgs_status_text_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn mavros_msgs_status_text_builder_set_severity(
    b: *mut mavros_msgs_status_text_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.severity = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_status_text_builder_set_text(
    b: *mut mavros_msgs_status_text_builder_t,
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
        (*b).0.text = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn mavros_msgs_status_text_builder_build(
    b: *mut mavros_msgs_status_text_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = StatusText::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .severity(inner.severity)
        .text(inner.text.as_str())
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn mavros_msgs_status_text_builder_encode_into(
    b: *mut mavros_msgs_status_text_builder_t,
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
        StatusText::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .severity(inner.severity)
            .text(inner.text.as_str())
            .encode_into_slice(dst),
        out_len,
    )
}

struct GpsRawBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    fix_type: u8,
    lat: i32,
    lon: i32,
    alt: i32,
    eph: u16,
    epv: u16,
    vel: u16,
    cog: u16,
    satellites_visible: u8,
    alt_ellipsoid: i32,
    h_acc: u32,
    v_acc: u32,
    vel_acc: u32,
    hdg_acc: i32,
    yaw: u16,
    dgps_numch: u8,
    dgps_age: u32,
}
impl_builder_new_free!(
    mavros_msgs_gps_raw_builder_t,
    GpsRawBuilderOwned,
    mavros_msgs_gps_raw_builder_new,
    mavros_msgs_gps_raw_builder_free,
    GpsRawBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        fix_type: 0,
        lat: 0,
        lon: 0,
        alt: 0,
        eph: 0,
        epv: 0,
        vel: 0,
        cog: 0,
        satellites_visible: 0,
        alt_ellipsoid: 0,
        h_acc: 0,
        v_acc: 0,
        vel_acc: 0,
        hdg_acc: 0,
        yaw: 0,
        dgps_numch: 0,
        dgps_age: 0,
    }
);
impl_builder_stamp_frame!(
    mavros_msgs_gps_raw_builder_t,
    mavros_msgs_gps_raw_builder_set_stamp,
    mavros_msgs_gps_raw_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_fix_type(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.fix_type = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_lat(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: i32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.lat = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_lon(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: i32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.lon = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_alt(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: i32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.alt = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_eph(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.eph = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_epv(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.epv = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_vel(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.vel = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_cog(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.cog = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_satellites_visible(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.satellites_visible = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_alt_ellipsoid(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: i32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.alt_ellipsoid = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_h_acc(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: u32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.h_acc = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_v_acc(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: u32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.v_acc = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_vel_acc(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: u32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.vel_acc = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_hdg_acc(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: i32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.hdg_acc = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_yaw(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.yaw = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_dgps_numch(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.dgps_numch = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_set_dgps_age(
    b: *mut mavros_msgs_gps_raw_builder_t,
    v: u32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.dgps_age = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_build(
    b: *mut mavros_msgs_gps_raw_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = GpsRaw::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .fix_type(inner.fix_type)
        .lat(inner.lat)
        .lon(inner.lon)
        .alt(inner.alt)
        .eph(inner.eph)
        .epv(inner.epv)
        .vel(inner.vel)
        .cog(inner.cog)
        .satellites_visible(inner.satellites_visible)
        .alt_ellipsoid(inner.alt_ellipsoid)
        .h_acc(inner.h_acc)
        .v_acc(inner.v_acc)
        .vel_acc(inner.vel_acc)
        .hdg_acc(inner.hdg_acc)
        .yaw(inner.yaw)
        .dgps_numch(inner.dgps_numch)
        .dgps_age(inner.dgps_age)
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn mavros_msgs_gps_raw_builder_encode_into(
    b: *mut mavros_msgs_gps_raw_builder_t,
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
        GpsRaw::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .fix_type(inner.fix_type)
            .lat(inner.lat)
            .lon(inner.lon)
            .alt(inner.alt)
            .eph(inner.eph)
            .epv(inner.epv)
            .vel(inner.vel)
            .cog(inner.cog)
            .satellites_visible(inner.satellites_visible)
            .alt_ellipsoid(inner.alt_ellipsoid)
            .h_acc(inner.h_acc)
            .v_acc(inner.v_acc)
            .vel_acc(inner.vel_acc)
            .hdg_acc(inner.hdg_acc)
            .yaw(inner.yaw)
            .dgps_numch(inner.dgps_numch)
            .dgps_age(inner.dgps_age)
            .encode_into_slice(dst),
        out_len,
    )
}

struct TimesyncStatusBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    remote_timestamp_ns: u64,
    observed_offset_ns: i64,
    estimated_offset_ns: i64,
    round_trip_time_ms: f32,
}
impl_builder_new_free!(
    mavros_msgs_timesync_status_builder_t,
    TimesyncStatusBuilderOwned,
    mavros_msgs_timesync_status_builder_new,
    mavros_msgs_timesync_status_builder_free,
    TimesyncStatusBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        remote_timestamp_ns: 0,
        observed_offset_ns: 0,
        estimated_offset_ns: 0,
        round_trip_time_ms: 0.0,
    }
);
impl_builder_stamp_frame!(
    mavros_msgs_timesync_status_builder_t,
    mavros_msgs_timesync_status_builder_set_stamp,
    mavros_msgs_timesync_status_builder_set_frame_id
);

#[no_mangle]
pub extern "C" fn mavros_msgs_timesync_status_builder_set_remote_timestamp_ns(
    b: *mut mavros_msgs_timesync_status_builder_t,
    v: u64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.remote_timestamp_ns = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_timesync_status_builder_set_observed_offset_ns(
    b: *mut mavros_msgs_timesync_status_builder_t,
    v: i64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.observed_offset_ns = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_timesync_status_builder_set_estimated_offset_ns(
    b: *mut mavros_msgs_timesync_status_builder_t,
    v: i64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.estimated_offset_ns = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_timesync_status_builder_set_round_trip_time_ms(
    b: *mut mavros_msgs_timesync_status_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.round_trip_time_ms = v;
    }
}

#[no_mangle]
pub extern "C" fn mavros_msgs_timesync_status_builder_build(
    b: *mut mavros_msgs_timesync_status_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = TimesyncStatus::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .remote_timestamp_ns(inner.remote_timestamp_ns)
        .observed_offset_ns(inner.observed_offset_ns)
        .estimated_offset_ns(inner.estimated_offset_ns)
        .round_trip_time_ms(inner.round_trip_time_ms)
        .build();
    map_build(r, |v| v.into_cdr(), out_bytes, out_len)
}

#[no_mangle]
pub extern "C" fn mavros_msgs_timesync_status_builder_encode_into(
    b: *mut mavros_msgs_timesync_status_builder_t,
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
        TimesyncStatus::builder()
            .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
            .frame_id(inner.frame_id.as_str())
            .remote_timestamp_ns(inner.remote_timestamp_ns)
            .observed_offset_ns(inner.observed_offset_ns)
            .estimated_offset_ns(inner.estimated_offset_ns)
            .round_trip_time_ms(inner.round_trip_time_ms)
            .encode_into_slice(dst),
        out_len,
    )
}
