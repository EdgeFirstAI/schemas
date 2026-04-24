// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! C FFI (Foreign Function Interface) module — v2 buffer-view API
//!
//! This module provides zero-copy C-compatible bindings using the
//! buffer-backed view types. Wire format is unchanged (CDR1 LE).
//!
//! ## API Pattern
//!
//! **CdrFixed types** (Time, Duration, Vector3, etc.):
//!   - `ros_<type>_encode(buf, cap, &written, ...fields)` → write CDR to caller buffer
//!   - `ros_<type>_decode(data, len, ...out_fields)` → read fields from CDR
//!
//! **Buffer-backed types** (Image, CompressedImage, etc.):
//!   - `ros_<type>_from_cdr(data, len)` → opaque handle (zero-copy borrow of `data`)
//!   - `ros_<type>_get_<field>(handle)` → O(1) field access
//!   - `ros_<type>_free(handle)` → release handle
//!   - `ros_<type>_encode(&out_bytes, &out_len, ...fields)` → allocate + write CDR
//!
//! `from_cdr` borrows the caller's buffer — the returned handle stores a pointer
//! into `data`, not a copy. The caller must keep `data` alive until `_free()`.
//! String and blob getters return `const` pointers into the original `data` buffer.

#![allow(non_camel_case_types)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]
#![allow(clippy::needless_borrow)]

use std::os::raw::c_char;
use std::ptr;
use std::slice;

use crate::builtin_interfaces::{Duration, Time};
use crate::cdr::{self, CdrFixed};
use crate::edgefirst_msgs;
use crate::foxglove_msgs;
use crate::geometry_msgs::{self, *};
use crate::nav_msgs;
use crate::sensor_msgs::{self, NavSatStatus};
use crate::std_msgs;

// =============================================================================
// Helpers
// =============================================================================

const EINVAL: i32 = libc::EINVAL;
const EBADMSG: i32 = libc::EBADMSG;
const ENOBUFS: i32 = libc::ENOBUFS;

fn set_errno(code: i32) {
    errno::set_errno(errno::Errno(code));
}

/// Return a C string pointer. For non-empty strings from CDR buffers,
/// the byte after the &str content is the CDR NUL terminator, so
/// as_ptr() yields a valid C string.
fn str_as_c(s: &str) -> *const c_char {
    if s.is_empty() {
        c"".as_ptr()
    } else {
        // SAFETY: CDR strings are NUL-terminated in the buffer. rd_string()
        // returns a &str that excludes the trailing NUL, but the NUL byte
        // is present immediately after in the buffer.
        s.as_ptr() as *const c_char
    }
}

/// Convert a C string to &str. Returns "" for null pointers.
unsafe fn c_to_str<'a>(s: *const c_char) -> &'a str {
    if s.is_null() {
        ""
    } else {
        std::ffi::CStr::from_ptr(s).to_str().unwrap_or("")
    }
}

/// Erase the borrow lifetime on a `&[u8]` slice for FFI.
///
/// # Safety
/// The C caller guarantees `data` outlives the returned handle.
/// This is documented in CAPI.md — the handle borrows the caller's buffer.
unsafe fn erase_lifetime(s: &[u8]) -> &'static [u8] {
    unsafe { slice::from_raw_parts(s.as_ptr(), s.len()) }
}

// Note: there are no `erase_box_view_lifetime` / `erase_mask_view_lifetime`
// helpers any more. Every construction path for `DetectBoxView<'static>` and
// `MaskView<'static>` uses a parse helper that yields the `'static` view
// directly from a `&'static [u8]` input slice, so no unsafe `mem::transmute`
// is required to widen method-returned references. The standalone paths
// call `DetectBox::from_cdr_as_view` / `Mask::from_cdr_as_view`; the
// parent-child paths call `Detect::from_cdr_collect_boxes` /
// `Model::from_cdr_collect_children`. Both produce views whose `&str` and
// `&[u8]` fields are naturally tied to the buffer's lifetime via the
// `scan_*_element` primitives, not to a temporary `&self`.

macro_rules! check_null_ret_null {
    ($ptr:expr) => {
        if $ptr.is_null() {
            set_errno(EINVAL);
            return ptr::null_mut();
        }
    };
}

// =============================================================================
// Memory management
// =============================================================================

/// Free a byte buffer returned by any `ros_*_encode()` function.
///
/// # Safety
/// `bytes` must have been returned by a prior encode call. Passing any other
/// pointer is undefined behaviour. Passing NULL is safe (no-op).
///
/// Capacity invariant: `return_cdr_bytes()` converts `Vec<u8>` →
/// `Box<[u8]>` via `into_boxed_slice()`, which calls `shrink_to_fit()`
/// first, guaranteeing `capacity == len`. We reconstruct the Vec with
/// `capacity = len` here, matching the original allocation.
#[no_mangle]
pub extern "C" fn ros_bytes_free(bytes: *mut u8, len: usize) {
    if !bytes.is_null() && len > 0 {
        unsafe {
            drop(Vec::from_raw_parts(bytes, len, len));
        }
    }
}

// =============================================================================
// CdrFixed encode/decode helpers
// =============================================================================

/// Generic encode for CdrFixed types into a caller-provided buffer.
///
/// Pass `buf = NULL` to query the required size (returned via `written`).
/// Returns 0 on success, -1 on error (sets errno to `ENOBUFS`).
///
/// Note: we encode first, then use the actual buffer length rather than
/// `CDR_HEADER_SIZE + T::CDR_SIZE`, because CDR_SIZE represents the
/// packed content size at optimal alignment and does not account for
/// leading alignment padding after the 4-byte CDR header (e.g. f64 fields
/// need 4 bytes of padding from offset 4 → 8).
fn encode_fixed_to_buf<T: CdrFixed>(val: &T, buf: *mut u8, cap: usize, written: *mut usize) -> i32 {
    let bytes = match cdr::encode_fixed(val) {
        Ok(b) => b,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    let size = bytes.len();
    unsafe {
        if !written.is_null() {
            *written = size;
        }
    }
    if buf.is_null() {
        return 0; // Size query only
    }
    if cap < size {
        set_errno(ENOBUFS);
        return -1;
    }
    unsafe {
        ptr::copy_nonoverlapping(bytes.as_ptr(), buf, size);
    }
    0
}

/// Generic decode for CdrFixed types. Sets errno and returns Err on failure.
/// EINVAL for NULL data pointer, EBADMSG for decode failures.
fn decode_fixed_from_buf<T: CdrFixed>(data: *const u8, len: usize) -> Result<T, ()> {
    if data.is_null() {
        set_errno(EINVAL);
        return Err(());
    }
    if len < cdr::CDR_HEADER_SIZE {
        set_errno(EBADMSG);
        return Err(());
    }
    let slice = unsafe { slice::from_raw_parts(data, len) };
    cdr::decode_fixed(slice).map_err(|_| set_errno(EBADMSG))
}

// =============================================================================
// builtin_interfaces::Time
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_time_encode(
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
    sec: i32,
    nanosec: u32,
) -> i32 {
    let t = Time { sec, nanosec };
    encode_fixed_to_buf(&t, buf, cap, written)
}

#[no_mangle]
pub extern "C" fn ros_time_decode(
    data: *const u8,
    len: usize,
    sec: *mut i32,
    nanosec: *mut u32,
) -> i32 {
    match decode_fixed_from_buf::<Time>(data, len) {
        Ok(t) => unsafe {
            if !sec.is_null() {
                *sec = t.sec;
            }
            if !nanosec.is_null() {
                *nanosec = t.nanosec;
            }
            0
        },
        Err(()) => -1,
    }
}

// =============================================================================
// builtin_interfaces::Duration
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_duration_encode(
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
    sec: i32,
    nanosec: u32,
) -> i32 {
    let d = Duration { sec, nanosec };
    encode_fixed_to_buf(&d, buf, cap, written)
}

#[no_mangle]
pub extern "C" fn ros_duration_decode(
    data: *const u8,
    len: usize,
    sec: *mut i32,
    nanosec: *mut u32,
) -> i32 {
    match decode_fixed_from_buf::<Duration>(data, len) {
        Ok(d) => unsafe {
            if !sec.is_null() {
                *sec = d.sec;
            }
            if !nanosec.is_null() {
                *nanosec = d.nanosec;
            }
            0
        },
        Err(()) => -1,
    }
}

// =============================================================================
// geometry_msgs CdrFixed types
// =============================================================================

// Vector3
#[no_mangle]
pub extern "C" fn ros_vector3_encode(
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
    x: f64,
    y: f64,
    z: f64,
) -> i32 {
    encode_fixed_to_buf(&Vector3 { x, y, z }, buf, cap, written)
}

#[no_mangle]
pub extern "C" fn ros_vector3_decode(
    data: *const u8,
    len: usize,
    x: *mut f64,
    y: *mut f64,
    z: *mut f64,
) -> i32 {
    match decode_fixed_from_buf::<Vector3>(data, len) {
        Ok(v) => unsafe {
            if !x.is_null() {
                *x = v.x;
            }
            if !y.is_null() {
                *y = v.y;
            }
            if !z.is_null() {
                *z = v.z;
            }
            0
        },
        Err(()) => -1,
    }
}

// Point
#[no_mangle]
pub extern "C" fn ros_point_encode(
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
    x: f64,
    y: f64,
    z: f64,
) -> i32 {
    encode_fixed_to_buf(&Point { x, y, z }, buf, cap, written)
}

#[no_mangle]
pub extern "C" fn ros_point_decode(
    data: *const u8,
    len: usize,
    x: *mut f64,
    y: *mut f64,
    z: *mut f64,
) -> i32 {
    match decode_fixed_from_buf::<Point>(data, len) {
        Ok(v) => unsafe {
            if !x.is_null() {
                *x = v.x;
            }
            if !y.is_null() {
                *y = v.y;
            }
            if !z.is_null() {
                *z = v.z;
            }
            0
        },
        Err(()) => -1,
    }
}

// Quaternion
#[no_mangle]
pub extern "C" fn ros_quaternion_encode(
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
    x: f64,
    y: f64,
    z: f64,
    w: f64,
) -> i32 {
    encode_fixed_to_buf(&Quaternion { x, y, z, w }, buf, cap, written)
}

#[no_mangle]
pub extern "C" fn ros_quaternion_decode(
    data: *const u8,
    len: usize,
    x: *mut f64,
    y: *mut f64,
    z: *mut f64,
    w: *mut f64,
) -> i32 {
    match decode_fixed_from_buf::<Quaternion>(data, len) {
        Ok(v) => unsafe {
            if !x.is_null() {
                *x = v.x;
            }
            if !y.is_null() {
                *y = v.y;
            }
            if !z.is_null() {
                *z = v.z;
            }
            if !w.is_null() {
                *w = v.w;
            }
            0
        },
        Err(()) => -1,
    }
}

// Pose
#[no_mangle]
pub extern "C" fn ros_pose_encode(
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
    px: f64,
    py: f64,
    pz: f64,
    ox: f64,
    oy: f64,
    oz: f64,
    ow: f64,
) -> i32 {
    let val = Pose {
        position: Point {
            x: px,
            y: py,
            z: pz,
        },
        orientation: Quaternion {
            x: ox,
            y: oy,
            z: oz,
            w: ow,
        },
    };
    encode_fixed_to_buf(&val, buf, cap, written)
}

#[no_mangle]
pub extern "C" fn ros_pose_decode(
    data: *const u8,
    len: usize,
    px: *mut f64,
    py: *mut f64,
    pz: *mut f64,
    ox: *mut f64,
    oy: *mut f64,
    oz: *mut f64,
    ow: *mut f64,
) -> i32 {
    match decode_fixed_from_buf::<Pose>(data, len) {
        Ok(v) => unsafe {
            if !px.is_null() {
                *px = v.position.x;
            }
            if !py.is_null() {
                *py = v.position.y;
            }
            if !pz.is_null() {
                *pz = v.position.z;
            }
            if !ox.is_null() {
                *ox = v.orientation.x;
            }
            if !oy.is_null() {
                *oy = v.orientation.y;
            }
            if !oz.is_null() {
                *oz = v.orientation.z;
            }
            if !ow.is_null() {
                *ow = v.orientation.w;
            }
            0
        },
        Err(()) => -1,
    }
}

// Transform
#[no_mangle]
pub extern "C" fn ros_transform_encode(
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
    tx: f64,
    ty: f64,
    tz: f64,
    rx: f64,
    ry: f64,
    rz: f64,
    rw: f64,
) -> i32 {
    let val = Transform {
        translation: Vector3 {
            x: tx,
            y: ty,
            z: tz,
        },
        rotation: Quaternion {
            x: rx,
            y: ry,
            z: rz,
            w: rw,
        },
    };
    encode_fixed_to_buf(&val, buf, cap, written)
}

#[no_mangle]
pub extern "C" fn ros_transform_decode(
    data: *const u8,
    len: usize,
    tx: *mut f64,
    ty: *mut f64,
    tz: *mut f64,
    rx: *mut f64,
    ry: *mut f64,
    rz: *mut f64,
    rw: *mut f64,
) -> i32 {
    match decode_fixed_from_buf::<Transform>(data, len) {
        Ok(v) => unsafe {
            if !tx.is_null() {
                *tx = v.translation.x;
            }
            if !ty.is_null() {
                *ty = v.translation.y;
            }
            if !tz.is_null() {
                *tz = v.translation.z;
            }
            if !rx.is_null() {
                *rx = v.rotation.x;
            }
            if !ry.is_null() {
                *ry = v.rotation.y;
            }
            if !rz.is_null() {
                *rz = v.rotation.z;
            }
            if !rw.is_null() {
                *rw = v.rotation.w;
            }
            0
        },
        Err(()) => -1,
    }
}

// Twist
#[no_mangle]
pub extern "C" fn ros_twist_encode(
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
    lx: f64,
    ly: f64,
    lz: f64,
    ax: f64,
    ay: f64,
    az: f64,
) -> i32 {
    let val = Twist {
        linear: Vector3 {
            x: lx,
            y: ly,
            z: lz,
        },
        angular: Vector3 {
            x: ax,
            y: ay,
            z: az,
        },
    };
    encode_fixed_to_buf(&val, buf, cap, written)
}

#[no_mangle]
pub extern "C" fn ros_twist_decode(
    data: *const u8,
    len: usize,
    lx: *mut f64,
    ly: *mut f64,
    lz: *mut f64,
    ax: *mut f64,
    ay: *mut f64,
    az: *mut f64,
) -> i32 {
    match decode_fixed_from_buf::<Twist>(data, len) {
        Ok(v) => unsafe {
            if !lx.is_null() {
                *lx = v.linear.x;
            }
            if !ly.is_null() {
                *ly = v.linear.y;
            }
            if !lz.is_null() {
                *lz = v.linear.z;
            }
            if !ax.is_null() {
                *ax = v.angular.x;
            }
            if !ay.is_null() {
                *ay = v.angular.y;
            }
            if !az.is_null() {
                *az = v.angular.z;
            }
            0
        },
        Err(()) => -1,
    }
}

// Accel
#[no_mangle]
pub extern "C" fn ros_accel_encode(
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
    lx: f64,
    ly: f64,
    lz: f64,
    ax: f64,
    ay: f64,
    az: f64,
) -> i32 {
    let val = Accel {
        linear: Vector3 {
            x: lx,
            y: ly,
            z: lz,
        },
        angular: Vector3 {
            x: ax,
            y: ay,
            z: az,
        },
    };
    encode_fixed_to_buf(&val, buf, cap, written)
}

#[no_mangle]
pub extern "C" fn ros_accel_decode(
    data: *const u8,
    len: usize,
    lx: *mut f64,
    ly: *mut f64,
    lz: *mut f64,
    ax: *mut f64,
    ay: *mut f64,
    az: *mut f64,
) -> i32 {
    match decode_fixed_from_buf::<Accel>(data, len) {
        Ok(v) => unsafe {
            if !lx.is_null() {
                *lx = v.linear.x;
            }
            if !ly.is_null() {
                *ly = v.linear.y;
            }
            if !lz.is_null() {
                *lz = v.linear.z;
            }
            if !ax.is_null() {
                *ax = v.angular.x;
            }
            if !ay.is_null() {
                *ay = v.angular.y;
            }
            if !az.is_null() {
                *az = v.angular.z;
            }
            0
        },
        Err(()) => -1,
    }
}

// NavSatStatus
#[no_mangle]
pub extern "C" fn ros_nav_sat_status_encode(
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
    status: i8,
    service: u16,
) -> i32 {
    encode_fixed_to_buf(&NavSatStatus { status, service }, buf, cap, written)
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_status_decode(
    data: *const u8,
    len: usize,
    status: *mut i8,
    service: *mut u16,
) -> i32 {
    match decode_fixed_from_buf::<NavSatStatus>(data, len) {
        Ok(v) => unsafe {
            if !status.is_null() {
                *status = v.status;
            }
            if !service.is_null() {
                *service = v.service;
            }
            0
        },
        Err(()) => -1,
    }
}

// =============================================================================
// Buffer-backed view types — macro for common boilerplate
// =============================================================================

/// Decode a little-endian CDR sequence of `f32` in-place into a caller buffer.
///
/// `seq_off` points at the 4-byte element count; the elements follow
/// immediately (no alignment padding: f32 aligns to 4, matching the count).
/// Returns the element count regardless of `out`/`cap`. Allocation-free.
fn copy_le_f32_seq(data: &[u8], seq_off: usize, out: *mut f32, cap: usize) -> u32 {
    let Some(len_end) = seq_off.checked_add(4) else {
        return 0;
    };
    if len_end > data.len() {
        return 0;
    }
    let n = u32::from_le_bytes(data[seq_off..len_end].try_into().unwrap()) as usize;
    if !out.is_null() && cap > 0 {
        let copy_n = n.min(cap);
        let elems = match len_end.checked_add(copy_n.saturating_mul(4)) {
            Some(e) if e <= data.len() => &data[len_end..e],
            _ => return n as u32,
        };
        for i in 0..copy_n {
            let b = i * 4;
            let val = f32::from_le_bytes([elems[b], elems[b + 1], elems[b + 2], elems[b + 3]]);
            unsafe {
                *out.add(i) = val;
            }
        }
    }
    n as u32
}

/// Decode a little-endian CDR sequence of `u32` in-place into a caller buffer.
/// Same layout assumptions as [`copy_le_f32_seq`]. Allocation-free.
fn copy_le_u32_seq(data: &[u8], seq_off: usize, out: *mut u32, cap: usize) -> u32 {
    let Some(len_end) = seq_off.checked_add(4) else {
        return 0;
    };
    if len_end > data.len() {
        return 0;
    }
    let n = u32::from_le_bytes(data[seq_off..len_end].try_into().unwrap()) as usize;
    if !out.is_null() && cap > 0 {
        let copy_n = n.min(cap);
        let elems = match len_end.checked_add(copy_n.saturating_mul(4)) {
            Some(e) if e <= data.len() => &data[len_end..e],
            _ => return n as u32,
        };
        for i in 0..copy_n {
            let b = i * 4;
            let val = u32::from_le_bytes([elems[b], elems[b + 1], elems[b + 2], elems[b + 3]]);
            unsafe {
                *out.add(i) = val;
            }
        }
    }
    n as u32
}

/// Helper to return CDR bytes from an owned view (encode result).
/// Leaks the Vec as a raw pointer; caller must use ros_bytes_free().
fn return_cdr_bytes(cdr: Vec<u8>, out_bytes: *mut *mut u8, out_len: *mut usize) -> i32 {
    let len = cdr.len();
    let ptr = Box::into_raw(cdr.into_boxed_slice()) as *mut u8;
    unsafe {
        if !out_bytes.is_null() {
            *out_bytes = ptr;
        }
        if !out_len.is_null() {
            *out_len = len;
        }
    }
    0
}

// =============================================================================
// Header (buffer-backed)
// =============================================================================

pub struct ros_header_t(std_msgs::Header<&'static [u8]>);

/// @brief Create a Header view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_header_from_cdr(data: *const u8, len: usize) -> *mut ros_header_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match std_msgs::Header::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_header_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_header_free(view: *mut ros_header_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_header_get_stamp_sec(view: *const ros_header_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_header_get_stamp_nanosec(view: *const ros_header_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_header_get_frame_id(view: *const ros_header_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_header_encode(
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: *const c_char,
) -> i32 {
    let fid = unsafe { c_to_str(frame_id) };
    let v = match std_msgs::Header::builder()
        .stamp(Time::new(stamp_sec, stamp_nanosec))
        .frame_id(fid)
        .build()
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    return_cdr_bytes(v.into_cdr(), out_bytes, out_len)
}

// =============================================================================
// Image (buffer-backed)
// =============================================================================

pub struct ros_image_t(sensor_msgs::Image<&'static [u8]>);

/// @brief Create an Image view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_image_from_cdr(data: *const u8, len: usize) -> *mut ros_image_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match sensor_msgs::Image::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_image_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_image_free(view: *mut ros_image_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_image_get_stamp_sec(view: *const ros_image_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_image_get_stamp_nanosec(view: *const ros_image_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_image_get_frame_id(view: *const ros_image_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_image_get_height(view: *const ros_image_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.height() }
}

#[no_mangle]
pub extern "C" fn ros_image_get_width(view: *const ros_image_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.width() }
}

#[no_mangle]
pub extern "C" fn ros_image_get_encoding(view: *const ros_image_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.encoding() })
}

#[no_mangle]
pub extern "C" fn ros_image_get_is_bigendian(view: *const ros_image_t) -> u8 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.is_bigendian() }
}

#[no_mangle]
pub extern "C" fn ros_image_get_step(view: *const ros_image_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.step() }
}

#[no_mangle]
pub extern "C" fn ros_image_get_data(view: *const ros_image_t, out_len: *mut usize) -> *const u8 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    let data = unsafe { (*view).0.data() };
    unsafe {
        if !out_len.is_null() {
            *out_len = data.len();
        }
    }
    data.as_ptr()
}

#[no_mangle]
pub extern "C" fn ros_image_encode(
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: *const c_char,
    height: u32,
    width: u32,
    encoding: *const c_char,
    is_bigendian: u8,
    step: u32,
    data: *const u8,
    data_len: usize,
) -> i32 {
    let fid = unsafe { c_to_str(frame_id) };
    let enc = unsafe { c_to_str(encoding) };
    let d = if data.is_null() {
        &[]
    } else {
        unsafe { slice::from_raw_parts(data, data_len) }
    };
    let v = match sensor_msgs::Image::builder()
        .stamp(Time::new(stamp_sec, stamp_nanosec))
        .frame_id(fid)
        .height(height)
        .width(width)
        .encoding(enc)
        .is_bigendian(is_bigendian)
        .step(step)
        .data(d)
        .build()
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    return_cdr_bytes(v.into_cdr(), out_bytes, out_len)
}

// =============================================================================
// CompressedImage (buffer-backed)
// =============================================================================

pub struct ros_compressed_image_t(sensor_msgs::CompressedImage<&'static [u8]>);

/// @brief Create a CompressedImage view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_compressed_image_from_cdr(
    data: *const u8,
    len: usize,
) -> *mut ros_compressed_image_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match sensor_msgs::CompressedImage::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_compressed_image_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_free(view: *mut ros_compressed_image_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_get_stamp_sec(view: *const ros_compressed_image_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_get_stamp_nanosec(
    view: *const ros_compressed_image_t,
) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_get_frame_id(
    view: *const ros_compressed_image_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_get_format(
    view: *const ros_compressed_image_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.format() })
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_get_data(
    view: *const ros_compressed_image_t,
    out_len: *mut usize,
) -> *const u8 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    let data = unsafe { (*view).0.data() };
    unsafe {
        if !out_len.is_null() {
            *out_len = data.len();
        }
    }
    data.as_ptr()
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_encode(
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: *const c_char,
    format: *const c_char,
    data: *const u8,
    data_len: usize,
) -> i32 {
    let fid = unsafe { c_to_str(frame_id) };
    let fmt = unsafe { c_to_str(format) };
    let d = if data.is_null() {
        &[]
    } else {
        unsafe { slice::from_raw_parts(data, data_len) }
    };
    let v =
        match sensor_msgs::CompressedImage::new(Time::new(stamp_sec, stamp_nanosec), fid, fmt, d) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    return_cdr_bytes(v.into_cdr(), out_bytes, out_len)
}

// =============================================================================
// FoxgloveCompressedVideo (buffer-backed)
// =============================================================================

pub struct ros_compressed_video_t(foxglove_msgs::FoxgloveCompressedVideo<&'static [u8]>);

/// @brief Create a CompressedVideo view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_compressed_video_from_cdr(
    data: *const u8,
    len: usize,
) -> *mut ros_compressed_video_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match foxglove_msgs::FoxgloveCompressedVideo::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_compressed_video_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_compressed_video_free(view: *mut ros_compressed_video_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_compressed_video_get_stamp_sec(view: *const ros_compressed_video_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_compressed_video_get_stamp_nanosec(
    view: *const ros_compressed_video_t,
) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_compressed_video_get_frame_id(
    view: *const ros_compressed_video_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_compressed_video_get_data(
    view: *const ros_compressed_video_t,
    out_len: *mut usize,
) -> *const u8 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    let data = unsafe { (*view).0.data() };
    unsafe {
        if !out_len.is_null() {
            *out_len = data.len();
        }
    }
    data.as_ptr()
}

#[no_mangle]
pub extern "C" fn ros_compressed_video_get_format(
    view: *const ros_compressed_video_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.format() })
}

#[no_mangle]
pub extern "C" fn ros_compressed_video_encode(
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: *const c_char,
    data: *const u8,
    data_len: usize,
    format: *const c_char,
) -> i32 {
    let fid = unsafe { c_to_str(frame_id) };
    let fmt = unsafe { c_to_str(format) };
    let d = if data.is_null() {
        &[]
    } else {
        unsafe { slice::from_raw_parts(data, data_len) }
    };
    let v = match foxglove_msgs::FoxgloveCompressedVideo::builder()
        .stamp(Time::new(stamp_sec, stamp_nanosec))
        .frame_id(fid)
        .data(d)
        .format(fmt)
        .build()
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    return_cdr_bytes(v.into_cdr(), out_bytes, out_len)
}

// =============================================================================
// Mask (buffer-backed)
// =============================================================================

pub struct ros_mask_t {
    view: edgefirst_msgs::MaskView<'static>,
    /// `true` if this handle was heap-allocated by `ros_mask_from_cdr`
    /// (and must be freed by `ros_mask_free`). `false` if this handle
    /// lives inside a parent `ros_model_t`'s `child_masks` vector and
    /// is borrowed from there; calling `ros_mask_free` on a borrowed
    /// handle would `Box::from_raw` an address inside someone else's
    /// allocation and corrupt the heap, so `ros_mask_free` checks this
    /// flag and no-ops with `errno=EINVAL` when it's `false`.
    owned: bool,
}

/// @brief Create a Mask view from a standalone CDR buffer.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_mask_from_cdr(data: *const u8, len: usize) -> *mut ros_mask_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    let static_slice: &'static [u8] = unsafe { erase_lifetime(slice) };
    // Parse directly into a `MaskView<'static>`. The helper calls
    // `scan_mask_element` which returns a view whose `&str` / `&[u8]` fields
    // are structurally tied to the buffer's `'static` lifetime, so no unsafe
    // `mem::transmute` is required to widen method-returned references.
    match edgefirst_msgs::Mask::from_cdr_as_view(static_slice) {
        Ok(view) => Box::into_raw(Box::new(ros_mask_t { view, owned: true })),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

/// @brief Free a Mask handle obtained from `ros_mask_from_cdr`.
///
/// Safe to call with a NULL pointer. If the handle was obtained from
/// `ros_model_get_mask` (a parent-borrowed child), this function does
/// **not** free it — the parent `ros_model_t` owns the child storage,
/// and freeing here would corrupt the parent's `child_masks` vector.
/// In that case the function sets `errno=EINVAL` and returns without
/// touching the pointer. Passing a borrowed handle here is an API
/// misuse (Rule 5 in CAPI.md); this is defense-in-depth against it.
#[no_mangle]
pub extern "C" fn ros_mask_free(view: *mut ros_mask_t) {
    if view.is_null() {
        return;
    }
    unsafe {
        if (*view).owned {
            drop(Box::from_raw(view));
        } else {
            // Parent-borrowed child — must not free.
            set_errno(EINVAL);
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_mask_get_height(view: *const ros_mask_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).view.height }
}

#[no_mangle]
pub extern "C" fn ros_mask_get_width(view: *const ros_mask_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).view.width }
}

#[no_mangle]
pub extern "C" fn ros_mask_get_length(view: *const ros_mask_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).view.length }
}

#[no_mangle]
pub extern "C" fn ros_mask_get_encoding(view: *const ros_mask_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).view.encoding })
}

#[no_mangle]
pub extern "C" fn ros_mask_get_data(view: *const ros_mask_t, out_len: *mut usize) -> *const u8 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    let data = unsafe { (*view).view.mask };
    unsafe {
        if !out_len.is_null() {
            *out_len = data.len();
        }
    }
    data.as_ptr()
}

#[no_mangle]
pub extern "C" fn ros_mask_get_boxed(view: *const ros_mask_t) -> bool {
    if view.is_null() {
        return false;
    }
    unsafe { (*view).view.boxed }
}

#[no_mangle]
pub extern "C" fn ros_mask_encode(
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
    height: u32,
    width: u32,
    length: u32,
    encoding: *const c_char,
    data: *const u8,
    data_len: usize,
    boxed: bool,
) -> i32 {
    let enc = unsafe { c_to_str(encoding) };
    let d = if data.is_null() {
        &[]
    } else {
        unsafe { slice::from_raw_parts(data, data_len) }
    };
    let v = match edgefirst_msgs::Mask::builder()
        .height(height)
        .width(width)
        .length(length)
        .encoding(enc)
        .mask(d)
        .boxed(boxed)
        .build()
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    return_cdr_bytes(v.into_cdr(), out_bytes, out_len)
}

// =============================================================================
// DmaBuffer (buffer-backed) — DEPRECATED, use CameraFrame instead
// =============================================================================
//
// The `ros_dmabuffer_*` C API is kept for binary compatibility throughout the
// 3.x series and will be removed in 4.0.0. New code should use the
// `ros_camera_frame_*` API defined below.
#[allow(deprecated)]
pub struct ros_dmabuffer_t(edgefirst_msgs::DmaBuffer<&'static [u8]>);

/// @brief Create a DmaBuffer view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[allow(deprecated)]
#[no_mangle]
pub extern "C" fn ros_dmabuffer_from_cdr(data: *const u8, len: usize) -> *mut ros_dmabuffer_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match edgefirst_msgs::DmaBuffer::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_dmabuffer_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_dmabuffer_free(view: *mut ros_dmabuffer_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_dmabuffer_get_stamp_sec(view: *const ros_dmabuffer_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_dmabuffer_get_stamp_nanosec(view: *const ros_dmabuffer_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_dmabuffer_get_frame_id(view: *const ros_dmabuffer_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_dmabuffer_get_pid(view: *const ros_dmabuffer_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.pid() }
}

#[no_mangle]
pub extern "C" fn ros_dmabuffer_get_fd(view: *const ros_dmabuffer_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.fd() }
}

#[no_mangle]
pub extern "C" fn ros_dmabuffer_get_width(view: *const ros_dmabuffer_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.width() }
}

#[no_mangle]
pub extern "C" fn ros_dmabuffer_get_height(view: *const ros_dmabuffer_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.height() }
}

#[no_mangle]
pub extern "C" fn ros_dmabuffer_get_stride(view: *const ros_dmabuffer_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stride() }
}

#[no_mangle]
pub extern "C" fn ros_dmabuffer_get_fourcc(view: *const ros_dmabuffer_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.fourcc() }
}

#[no_mangle]
pub extern "C" fn ros_dmabuffer_get_length(view: *const ros_dmabuffer_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.length() }
}

#[allow(deprecated)]
#[no_mangle]
pub extern "C" fn ros_dmabuffer_encode(
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: *const c_char,
    pid: u32,
    fd: i32,
    width: u32,
    height: u32,
    stride: u32,
    fourcc: u32,
    length: u32,
) -> i32 {
    let fid = unsafe { c_to_str(frame_id) };
    let v = match edgefirst_msgs::DmaBuffer::new(
        Time::new(stamp_sec, stamp_nanosec),
        fid,
        pid,
        fd,
        width,
        height,
        stride,
        fourcc,
        length,
    ) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    return_cdr_bytes(v.into_cdr(), out_bytes, out_len)
}

// =============================================================================
// CameraFrame / CameraPlane (buffer-backed, multi-plane)
// =============================================================================
//
// CameraFrame supersedes DmaBuffer. Child planes are pre-materialized at
// `from_cdr` time into `child_planes`, mirroring the Detect → Box pattern.

/// Opaque handle for a single CameraPlane view.
///
/// Either heap-allocated directly (standalone, unused today) or borrowed from
/// a parent `ros_camera_frame_t`. The `owned` flag keeps
/// `ros_camera_plane_free` safe if a caller mistakenly frees a borrowed
/// child.
pub struct ros_camera_plane_t {
    view: edgefirst_msgs::CameraPlaneView<'static>,
    owned: bool,
}

pub struct ros_camera_frame_t {
    inner: edgefirst_msgs::CameraFrame<&'static [u8]>,
    child_planes: Vec<ros_camera_plane_t>,
}

/// @brief Create a CameraFrame view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set to EINVAL or EBADMSG)
#[no_mangle]
pub extern "C" fn ros_camera_frame_from_cdr(
    data: *const u8,
    len: usize,
) -> *mut ros_camera_frame_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match edgefirst_msgs::CameraFrame::from_cdr_collect_planes(unsafe { erase_lifetime(slice) }) {
        Ok((inner, plane_views)) => {
            let child_planes: Vec<ros_camera_plane_t> = plane_views
                .into_iter()
                .map(|view| ros_camera_plane_t { view, owned: false })
                .collect();
            Box::into_raw(Box::new(ros_camera_frame_t {
                inner,
                child_planes,
            }))
        }
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_free(view: *mut ros_camera_frame_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_stamp_sec(view: *const ros_camera_frame_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).inner.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_stamp_nanosec(view: *const ros_camera_frame_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).inner.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_frame_id(view: *const ros_camera_frame_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).inner.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_seq(view: *const ros_camera_frame_t) -> u64 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).inner.seq() }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_pid(view: *const ros_camera_frame_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).inner.pid() }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_width(view: *const ros_camera_frame_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).inner.width() }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_height(view: *const ros_camera_frame_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).inner.height() }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_fence_fd(view: *const ros_camera_frame_t) -> i32 {
    if view.is_null() {
        return -1;
    }
    unsafe { (*view).inner.fence_fd() }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_format(view: *const ros_camera_frame_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).inner.format() })
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_color_space(
    view: *const ros_camera_frame_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).inner.color_space() })
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_color_transfer(
    view: *const ros_camera_frame_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).inner.color_transfer() })
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_color_encoding(
    view: *const ros_camera_frame_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).inner.color_encoding() })
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_color_range(
    view: *const ros_camera_frame_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).inner.color_range() })
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_get_planes_len(view: *const ros_camera_frame_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).child_planes.len() as u32 }
}

/// @brief Get a borrowed view of the i-th plane.
///
/// The returned pointer is NOT a separately-owned handle: do NOT pass it to
/// `ros_camera_plane_free()`. It becomes invalid when the parent
/// `ros_camera_frame_t` handle is freed, and the CDR buffer passed to
/// `ros_camera_frame_from_cdr` must remain valid as well.
#[no_mangle]
pub extern "C" fn ros_camera_frame_get_plane(
    view: *const ros_camera_frame_t,
    index: u32,
) -> *const ros_camera_plane_t {
    if view.is_null() {
        set_errno(EINVAL);
        return ptr::null();
    }
    let v = unsafe { &*view };
    let idx = index as usize;
    if idx >= v.child_planes.len() {
        set_errno(EINVAL);
        return ptr::null();
    }
    &v.child_planes[idx] as *const ros_camera_plane_t
}

/// @brief Free a CameraPlane handle.
///
/// Safe to call with NULL. If the handle is parent-borrowed (via
/// `ros_camera_frame_get_plane`), this no-ops with `errno=EINVAL`
/// — the parent owns it.
#[no_mangle]
pub extern "C" fn ros_camera_plane_free(view: *mut ros_camera_plane_t) {
    if view.is_null() {
        return;
    }
    unsafe {
        if (*view).owned {
            drop(Box::from_raw(view));
        } else {
            set_errno(EINVAL);
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_plane_get_fd(view: *const ros_camera_plane_t) -> i32 {
    if view.is_null() {
        return -1;
    }
    unsafe { (*view).view.fd }
}

#[no_mangle]
pub extern "C" fn ros_camera_plane_get_offset(view: *const ros_camera_plane_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).view.offset }
}

#[no_mangle]
pub extern "C" fn ros_camera_plane_get_stride(view: *const ros_camera_plane_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).view.stride }
}

#[no_mangle]
pub extern "C" fn ros_camera_plane_get_size(view: *const ros_camera_plane_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).view.size }
}

#[no_mangle]
pub extern "C" fn ros_camera_plane_get_used(view: *const ros_camera_plane_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).view.used }
}

/// @brief Get the inlined plane data (only populated when fd == -1).
/// @param out_len Pointer to usize receiving the byte length (may be NULL).
/// @return Pointer to the plane bytes inside the parent's CDR buffer, or
///         NULL if the plane has no inlined data / invalid handle.
#[no_mangle]
pub extern "C" fn ros_camera_plane_get_data(
    view: *const ros_camera_plane_t,
    out_len: *mut usize,
) -> *const u8 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    let data = unsafe { (*view).view.data };
    if !out_len.is_null() {
        unsafe {
            *out_len = data.len();
        }
    }
    if data.is_empty() {
        ptr::null()
    } else {
        data.as_ptr()
    }
}

// =============================================================================
// IMU (buffer-backed)
// =============================================================================

pub struct ros_imu_t(sensor_msgs::Imu<&'static [u8]>);

/// @brief Create an Imu view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_imu_from_cdr(data: *const u8, len: usize) -> *mut ros_imu_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match sensor_msgs::Imu::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_imu_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_imu_free(view: *mut ros_imu_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_imu_get_stamp_sec(view: *const ros_imu_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_imu_get_stamp_nanosec(view: *const ros_imu_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_imu_get_frame_id(view: *const ros_imu_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

/// Write the IMU orientation quaternion (x, y, z, w) to the provided output pointers.
#[no_mangle]
pub extern "C" fn ros_imu_get_orientation(
    view: *const ros_imu_t,
    x: *mut f64,
    y: *mut f64,
    z: *mut f64,
    w: *mut f64,
) {
    if view.is_null() {
        return;
    }
    let q = unsafe { (*view).0.orientation() };
    unsafe {
        if !x.is_null() {
            *x = q.x;
        }
        if !y.is_null() {
            *y = q.y;
        }
        if !z.is_null() {
            *z = q.z;
        }
        if !w.is_null() {
            *w = q.w;
        }
    }
}

/// Write the 9-element orientation covariance to `out` (row-major 3×3).
#[no_mangle]
pub extern "C" fn ros_imu_get_orientation_covariance(view: *const ros_imu_t, out: *mut f64) {
    if view.is_null() || out.is_null() {
        return;
    }
    let cov = unsafe { (*view).0.orientation_covariance() };
    unsafe {
        ptr::copy_nonoverlapping(cov.as_ptr(), out, 9);
    }
}

/// Write the IMU angular velocity (x, y, z) to the provided output pointers.
#[no_mangle]
pub extern "C" fn ros_imu_get_angular_velocity(
    view: *const ros_imu_t,
    x: *mut f64,
    y: *mut f64,
    z: *mut f64,
) {
    if view.is_null() {
        return;
    }
    let v = unsafe { (*view).0.angular_velocity() };
    unsafe {
        if !x.is_null() {
            *x = v.x;
        }
        if !y.is_null() {
            *y = v.y;
        }
        if !z.is_null() {
            *z = v.z;
        }
    }
}

/// Write the 9-element angular velocity covariance to `out` (row-major 3×3).
#[no_mangle]
pub extern "C" fn ros_imu_get_angular_velocity_covariance(view: *const ros_imu_t, out: *mut f64) {
    if view.is_null() || out.is_null() {
        return;
    }
    let cov = unsafe { (*view).0.angular_velocity_covariance() };
    unsafe {
        ptr::copy_nonoverlapping(cov.as_ptr(), out, 9);
    }
}

/// Write the IMU linear acceleration (x, y, z) to the provided output pointers.
#[no_mangle]
pub extern "C" fn ros_imu_get_linear_acceleration(
    view: *const ros_imu_t,
    x: *mut f64,
    y: *mut f64,
    z: *mut f64,
) {
    if view.is_null() {
        return;
    }
    let v = unsafe { (*view).0.linear_acceleration() };
    unsafe {
        if !x.is_null() {
            *x = v.x;
        }
        if !y.is_null() {
            *y = v.y;
        }
        if !z.is_null() {
            *z = v.z;
        }
    }
}

/// Write the 9-element linear acceleration covariance to `out` (row-major 3×3).
#[no_mangle]
pub extern "C" fn ros_imu_get_linear_acceleration_covariance(
    view: *const ros_imu_t,
    out: *mut f64,
) {
    if view.is_null() || out.is_null() {
        return;
    }
    let cov = unsafe { (*view).0.linear_acceleration_covariance() };
    unsafe {
        ptr::copy_nonoverlapping(cov.as_ptr(), out, 9);
    }
}

// =============================================================================
// NavSatFix (buffer-backed)
// =============================================================================

pub struct ros_nav_sat_fix_t(sensor_msgs::NavSatFix<&'static [u8]>);

/// @brief Create a NavSatFix view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_from_cdr(data: *const u8, len: usize) -> *mut ros_nav_sat_fix_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match sensor_msgs::NavSatFix::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_nav_sat_fix_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_free(view: *mut ros_nav_sat_fix_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_stamp_sec(view: *const ros_nav_sat_fix_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_stamp_nanosec(view: *const ros_nav_sat_fix_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_frame_id(view: *const ros_nav_sat_fix_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_latitude(view: *const ros_nav_sat_fix_t) -> f64 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.latitude() }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_longitude(view: *const ros_nav_sat_fix_t) -> f64 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.longitude() }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_get_altitude(view: *const ros_nav_sat_fix_t) -> f64 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.altitude() }
}

// =============================================================================
// TransformStamped (buffer-backed)
// =============================================================================

pub struct ros_transform_stamped_t(geometry_msgs::TransformStamped<&'static [u8]>);

/// @brief Create a TransformStamped view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_transform_stamped_from_cdr(
    data: *const u8,
    len: usize,
) -> *mut ros_transform_stamped_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match geometry_msgs::TransformStamped::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_transform_stamped_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_transform_stamped_free(view: *mut ros_transform_stamped_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_transform_stamped_get_stamp_sec(view: *const ros_transform_stamped_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_transform_stamped_get_stamp_nanosec(
    view: *const ros_transform_stamped_t,
) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_transform_stamped_get_frame_id(
    view: *const ros_transform_stamped_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_transform_stamped_get_child_frame_id(
    view: *const ros_transform_stamped_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.child_frame_id() })
}

// =============================================================================
// RadarCube (buffer-backed)
// =============================================================================

pub struct ros_radar_cube_t(edgefirst_msgs::RadarCube<&'static [u8]>);

/// @brief Create a RadarCube view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_radar_cube_from_cdr(data: *const u8, len: usize) -> *mut ros_radar_cube_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match edgefirst_msgs::RadarCube::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_radar_cube_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_free(view: *mut ros_radar_cube_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_get_stamp_sec(view: *const ros_radar_cube_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_get_stamp_nanosec(view: *const ros_radar_cube_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_get_frame_id(view: *const ros_radar_cube_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_get_timestamp(view: *const ros_radar_cube_t) -> u64 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.timestamp() }
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_get_layout(
    view: *const ros_radar_cube_t,
    out_len: *mut usize,
) -> *const u8 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    let data = unsafe { (*view).0.layout() };
    unsafe {
        if !out_len.is_null() {
            *out_len = data.len();
        }
    }
    data.as_ptr()
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_get_cube_raw(
    view: *const ros_radar_cube_t,
    out_len: *mut usize,
) -> *const u8 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    let data = unsafe { (*view).0.cube_raw() };
    unsafe {
        if !out_len.is_null() {
            *out_len = data.len();
        }
    }
    data.as_ptr()
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_get_cube_len(view: *const ros_radar_cube_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.cube_len() }
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_get_is_complex(view: *const ros_radar_cube_t) -> bool {
    if view.is_null() {
        return false;
    }
    unsafe { (*view).0.is_complex() }
}

// =============================================================================
// RadarInfo (buffer-backed)
// =============================================================================

pub struct ros_radar_info_t(edgefirst_msgs::RadarInfo<&'static [u8]>);

/// @brief Create a RadarInfo view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_radar_info_from_cdr(data: *const u8, len: usize) -> *mut ros_radar_info_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match edgefirst_msgs::RadarInfo::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_radar_info_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_radar_info_free(view: *mut ros_radar_info_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_radar_info_get_stamp_sec(view: *const ros_radar_info_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_radar_info_get_stamp_nanosec(view: *const ros_radar_info_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_radar_info_get_frame_id(view: *const ros_radar_info_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_radar_info_get_center_frequency(
    view: *const ros_radar_info_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.center_frequency() })
}

#[no_mangle]
pub extern "C" fn ros_radar_info_get_frequency_sweep(
    view: *const ros_radar_info_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frequency_sweep() })
}

#[no_mangle]
pub extern "C" fn ros_radar_info_get_range_toggle(view: *const ros_radar_info_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.range_toggle() })
}

#[no_mangle]
pub extern "C" fn ros_radar_info_get_detection_sensitivity(
    view: *const ros_radar_info_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.detection_sensitivity() })
}

#[no_mangle]
pub extern "C" fn ros_radar_info_get_cube(view: *const ros_radar_info_t) -> bool {
    if view.is_null() {
        return false;
    }
    unsafe { (*view).0.cube() }
}

// =============================================================================
// Detect (buffer-backed)
// =============================================================================

pub struct ros_detect_t {
    inner: edgefirst_msgs::Detect<&'static [u8]>,
    /// Pre-materialized child box views, one per box in the parent. Each view's
    /// `&str` fields borrow directly from `inner`'s CDR buffer — no data
    /// duplication. This is bookkeeping allocation (Vec header + N*sizeof(ros_box_t))
    /// in the same category as the offset tables already built during from_cdr.
    child_boxes: Vec<ros_box_t>,
}

/// @brief Create a Detect view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_detect_from_cdr(data: *const u8, len: usize) -> *mut ros_detect_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match edgefirst_msgs::Detect::from_cdr_collect_boxes(unsafe { erase_lifetime(slice) }) {
        Ok((v, box_views)) => {
            // box_views were collected during the single validation walk;
            // no second pass over the CDR buffer is needed here. Each child
            // is marked `owned: false` so ros_box_free safely no-ops if a
            // caller mistakenly casts away const and passes a borrowed child.
            let child_boxes: Vec<ros_box_t> = box_views
                .into_iter()
                .map(|bv| ros_box_t {
                    view: bv,
                    owned: false,
                })
                .collect();
            Box::into_raw(Box::new(ros_detect_t {
                inner: v,
                child_boxes,
            }))
        }
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_free(view: *mut ros_detect_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_get_stamp_sec(view: *const ros_detect_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).inner.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_detect_get_stamp_nanosec(view: *const ros_detect_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).inner.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_detect_get_frame_id(view: *const ros_detect_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).inner.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_detect_get_boxes_len(view: *const ros_detect_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).child_boxes.len() as u32 }
}

/// @brief Get a borrowed view of the i-th detection box.
/// @param view Detect handle
/// @param index Zero-based box index (must be < ros_detect_get_boxes_len(view))
/// @return Borrowed ros_box_t* whose lifetime is tied to the parent Detect handle,
///         or NULL on error (errno set to EINVAL).
///
/// The returned pointer is NOT a separately-owned handle: do NOT pass it to
/// ros_box_free(). It becomes invalid when the parent Detect handle is freed.
/// The parent's CDR buffer (passed to ros_detect_from_cdr) must also remain
/// valid for as long as the returned pointer is used.
///
/// Defense-in-depth: the returned `ros_box_t` has its internal `owned` tag
/// set to `false` (populated at parent `from_cdr` time). If a caller
/// mistakenly casts away `const` and passes this pointer to `ros_box_free`,
/// the free function detects the borrowed tag, sets `errno=EINVAL`, and
/// safely no-ops — the parent's internal `child_boxes` Vec is never
/// touched, so heap corruption from misuse is not possible.
#[no_mangle]
pub extern "C" fn ros_detect_get_box(view: *const ros_detect_t, index: u32) -> *const ros_box_t {
    if view.is_null() {
        set_errno(EINVAL);
        return ptr::null();
    }
    let v = unsafe { &*view };
    let idx = index as usize;
    if idx >= v.child_boxes.len() {
        set_errno(EINVAL);
        return ptr::null();
    }
    &v.child_boxes[idx] as *const ros_box_t
}

// =============================================================================
// Model (buffer-backed)
// =============================================================================

pub struct ros_model_t {
    inner: edgefirst_msgs::Model<&'static [u8]>,
    /// Pre-materialized child box views, one per box in the parent. Each view's
    /// `&str` fields borrow directly from `inner`'s CDR buffer — no data
    /// duplication.
    child_boxes: Vec<ros_box_t>,
    /// Pre-materialized child mask views, one per mask in the parent. Each view's
    /// `&str` and `&[u8]` fields borrow directly from `inner`'s CDR buffer.
    child_masks: Vec<ros_mask_t>,
}

/// @brief Create a Model view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_model_from_cdr(data: *const u8, len: usize) -> *mut ros_model_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match edgefirst_msgs::Model::from_cdr_collect_children(unsafe { erase_lifetime(slice) }) {
        Ok((v, box_views, mask_views)) => {
            // box_views and mask_views were collected during the single
            // validation walk; no second pass over the CDR buffer is needed.
            // Each child is marked `owned: false` so ros_box_free /
            // ros_mask_free safely no-op if a caller mistakenly casts
            // away const and passes a borrowed child.
            let child_boxes: Vec<ros_box_t> = box_views
                .into_iter()
                .map(|bv| ros_box_t {
                    view: bv,
                    owned: false,
                })
                .collect();
            let child_masks: Vec<ros_mask_t> = mask_views
                .into_iter()
                .map(|mv| ros_mask_t {
                    view: mv,
                    owned: false,
                })
                .collect();
            Box::into_raw(Box::new(ros_model_t {
                inner: v,
                child_boxes,
                child_masks,
            }))
        }
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_model_free(view: *mut ros_model_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_model_get_stamp_sec(view: *const ros_model_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).inner.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_model_get_stamp_nanosec(view: *const ros_model_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).inner.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_model_get_frame_id(view: *const ros_model_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).inner.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_model_get_boxes_len(view: *const ros_model_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).child_boxes.len() as u32 }
}

#[no_mangle]
pub extern "C" fn ros_model_get_masks_len(view: *const ros_model_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).child_masks.len() as u32 }
}

/// @brief Get a borrowed view of the i-th model box.
/// @param view Model handle
/// @param index Zero-based box index (must be < ros_model_get_boxes_len(view))
/// @return Borrowed ros_box_t* whose lifetime is tied to the parent Model handle,
///         or NULL on error (errno set to EINVAL).
///
/// The returned pointer is NOT a separately-owned handle: do NOT pass it to
/// ros_box_free(). It becomes invalid when the parent Model handle is freed.
///
/// Defense-in-depth: the returned `ros_box_t` has its internal `owned` tag
/// set to `false`. If a caller mistakenly casts away `const` and passes
/// this pointer to `ros_box_free`, the free function detects the borrowed
/// tag, sets `errno=EINVAL`, and safely no-ops — the parent's internal
/// `child_boxes` Vec is never touched, so heap corruption from misuse is
/// not possible.
#[no_mangle]
pub extern "C" fn ros_model_get_box(view: *const ros_model_t, index: u32) -> *const ros_box_t {
    if view.is_null() {
        set_errno(EINVAL);
        return ptr::null();
    }
    let v = unsafe { &*view };
    let idx = index as usize;
    if idx >= v.child_boxes.len() {
        set_errno(EINVAL);
        return ptr::null();
    }
    &v.child_boxes[idx] as *const ros_box_t
}

/// @brief Get a borrowed view of the i-th model mask.
/// @param view Model handle
/// @param index Zero-based mask index (must be < ros_model_get_masks_len(view))
/// @return Borrowed ros_mask_t* whose lifetime is tied to the parent Model handle,
///         or NULL on error (errno set to EINVAL).
///
/// The returned pointer is NOT a separately-owned handle: do NOT pass it to
/// ros_mask_free(). It becomes invalid when the parent Model handle is freed.
///
/// Defense-in-depth: the returned `ros_mask_t` has its internal `owned` tag
/// set to `false`. If a caller mistakenly casts away `const` and passes
/// this pointer to `ros_mask_free`, the free function detects the borrowed
/// tag, sets `errno=EINVAL`, and safely no-ops — the parent's internal
/// `child_masks` Vec is never touched, so heap corruption from misuse is
/// not possible.
#[no_mangle]
pub extern "C" fn ros_model_get_mask(view: *const ros_model_t, index: u32) -> *const ros_mask_t {
    if view.is_null() {
        set_errno(EINVAL);
        return ptr::null();
    }
    let v = unsafe { &*view };
    let idx = index as usize;
    if idx >= v.child_masks.len() {
        set_errno(EINVAL);
        return ptr::null();
    }
    &v.child_masks[idx] as *const ros_mask_t
}

// =============================================================================
// ModelInfo (buffer-backed)
// =============================================================================

pub struct ros_model_info_t(edgefirst_msgs::ModelInfo<&'static [u8]>);

/// @brief Create a ModelInfo view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_model_info_from_cdr(data: *const u8, len: usize) -> *mut ros_model_info_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match edgefirst_msgs::ModelInfo::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_model_info_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_model_info_free(view: *mut ros_model_info_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_model_info_get_stamp_sec(view: *const ros_model_info_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_model_info_get_stamp_nanosec(view: *const ros_model_info_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_model_info_get_frame_id(view: *const ros_model_info_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_model_info_get_model_type(view: *const ros_model_info_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.model_type() })
}

#[no_mangle]
pub extern "C" fn ros_model_info_get_model_format(view: *const ros_model_info_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.model_format() })
}

#[no_mangle]
pub extern "C" fn ros_model_info_get_model_name(view: *const ros_model_info_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.model_name() })
}

#[no_mangle]
pub extern "C" fn ros_model_info_get_input_type(view: *const ros_model_info_t) -> u8 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.input_type() }
}

#[no_mangle]
pub extern "C" fn ros_model_info_get_output_type(view: *const ros_model_info_t) -> u8 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.output_type() }
}

#[no_mangle]
pub extern "C" fn ros_model_info_get_input_shape(
    view: *const ros_model_info_t,
    out_len: *mut usize,
) -> *const u32 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    let data = unsafe { (*view).0.input_shape() };
    unsafe {
        if !out_len.is_null() {
            *out_len = data.len();
        }
    }
    data.as_ptr()
}

#[no_mangle]
pub extern "C" fn ros_model_info_get_output_shape(
    view: *const ros_model_info_t,
    out_len: *mut usize,
) -> *const u32 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    let data = unsafe { (*view).0.output_shape() };
    unsafe {
        if !out_len.is_null() {
            *out_len = data.len();
        }
    }
    data.as_ptr()
}

#[no_mangle]
pub extern "C" fn ros_model_info_get_labels_len(view: *const ros_model_info_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.labels_len() }
}

#[no_mangle]
pub extern "C" fn ros_model_info_get_label(
    view: *const ros_model_info_t,
    index: u32,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    let labels = unsafe { (*view).0.labels() };
    match labels.get(index as usize) {
        Some(s) => str_as_c(s),
        None => {
            set_errno(EINVAL);
            ptr::null()
        }
    }
}

// =============================================================================
// PointCloud2 (buffer-backed)
// =============================================================================

pub struct ros_point_cloud2_t(sensor_msgs::PointCloud2<&'static [u8]>);

/// @brief Create a PointCloud2 view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_point_cloud2_from_cdr(
    data: *const u8,
    len: usize,
) -> *mut ros_point_cloud2_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match sensor_msgs::PointCloud2::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_point_cloud2_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_free(view: *mut ros_point_cloud2_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_stamp_sec(view: *const ros_point_cloud2_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_stamp_nanosec(view: *const ros_point_cloud2_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_frame_id(view: *const ros_point_cloud2_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_height(view: *const ros_point_cloud2_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.height() }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_width(view: *const ros_point_cloud2_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.width() }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_point_step(view: *const ros_point_cloud2_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.point_step() }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_row_step(view: *const ros_point_cloud2_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.row_step() }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_data(
    view: *const ros_point_cloud2_t,
    out_len: *mut usize,
) -> *const u8 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    let data = unsafe { (*view).0.data() };
    unsafe {
        if !out_len.is_null() {
            *out_len = data.len();
        }
    }
    data.as_ptr()
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_is_dense(view: *const ros_point_cloud2_t) -> bool {
    if view.is_null() {
        return false;
    }
    unsafe { (*view).0.is_dense() }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_is_bigendian(view: *const ros_point_cloud2_t) -> bool {
    if view.is_null() {
        return false;
    }
    unsafe { (*view).0.is_bigendian() }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_get_fields_len(view: *const ros_point_cloud2_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.fields_len() }
}

// =============================================================================
// CameraInfo (buffer-backed)
// =============================================================================

pub struct ros_camera_info_t(sensor_msgs::CameraInfo<&'static [u8]>);

/// @brief Create a CameraInfo view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_camera_info_from_cdr(data: *const u8, len: usize) -> *mut ros_camera_info_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match sensor_msgs::CameraInfo::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_camera_info_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_free(view: *mut ros_camera_info_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_get_stamp_sec(view: *const ros_camera_info_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_get_stamp_nanosec(view: *const ros_camera_info_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_get_frame_id(view: *const ros_camera_info_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_camera_info_get_height(view: *const ros_camera_info_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.height() }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_get_width(view: *const ros_camera_info_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.width() }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_get_distortion_model(
    view: *const ros_camera_info_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.distortion_model() })
}

#[no_mangle]
pub extern "C" fn ros_camera_info_get_binning_x(view: *const ros_camera_info_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.binning_x() }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_get_binning_y(view: *const ros_camera_info_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.binning_y() }
}

// =============================================================================
// Track (buffer-backed)
// =============================================================================

pub struct ros_track_t(edgefirst_msgs::Track<&'static [u8]>);

/// @brief Create a Track view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_track_from_cdr(data: *const u8, len: usize) -> *mut ros_track_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match edgefirst_msgs::Track::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_track_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_track_free(view: *mut ros_track_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_track_get_id(view: *const ros_track_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.id() })
}

#[no_mangle]
pub extern "C" fn ros_track_get_lifetime(view: *const ros_track_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.lifetime() }
}

// =============================================================================
// DetectBox (buffer-backed)
// =============================================================================

pub struct ros_box_t {
    view: edgefirst_msgs::DetectBoxView<'static>,
    /// `true` if this handle was heap-allocated by `ros_box_from_cdr`
    /// (and must be freed by `ros_box_free`). `false` if this handle
    /// lives inside a parent `ros_detect_t` / `ros_model_t`'s
    /// `child_boxes` vector and is borrowed from there; calling
    /// `ros_box_free` on a borrowed handle would `Box::from_raw` an
    /// address inside someone else's allocation and corrupt the heap,
    /// so `ros_box_free` checks this flag and no-ops with
    /// `errno=EINVAL` when it's `false`.
    owned: bool,
}

/// @brief Create a DetectBox view from a standalone CDR buffer.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_box_from_cdr(data: *const u8, len: usize) -> *mut ros_box_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    let static_slice: &'static [u8] = unsafe { erase_lifetime(slice) };
    // Parse directly into a `DetectBoxView<'static>`. The helper calls
    // `scan_box_element` which returns a view whose `&str` fields are
    // structurally tied to the buffer's `'static` lifetime, so no unsafe
    // `mem::transmute` is required to widen method-returned references.
    match edgefirst_msgs::DetectBox::from_cdr_as_view(static_slice) {
        Ok(view) => Box::into_raw(Box::new(ros_box_t { view, owned: true })),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

/// @brief Free a DetectBox handle obtained from `ros_box_from_cdr`.
///
/// Safe to call with a NULL pointer. If the handle was obtained from
/// `ros_detect_get_box` / `ros_model_get_box` (a parent-borrowed
/// child), this function does **not** free it — the parent
/// `ros_detect_t` / `ros_model_t` owns the child storage, and freeing
/// here would corrupt the parent's `child_boxes` vector. In that case
/// the function sets `errno=EINVAL` and returns without touching the
/// pointer. Passing a borrowed handle here is an API misuse (Rule 5
/// in CAPI.md); this is defense-in-depth against it.
#[no_mangle]
pub extern "C" fn ros_box_free(view: *mut ros_box_t) {
    if view.is_null() {
        return;
    }
    unsafe {
        if (*view).owned {
            drop(Box::from_raw(view));
        } else {
            // Parent-borrowed child — must not free.
            set_errno(EINVAL);
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_box_get_center_x(view: *const ros_box_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).view.center_x }
}

#[no_mangle]
pub extern "C" fn ros_box_get_center_y(view: *const ros_box_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).view.center_y }
}

#[no_mangle]
pub extern "C" fn ros_box_get_width(view: *const ros_box_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).view.width }
}

#[no_mangle]
pub extern "C" fn ros_box_get_height(view: *const ros_box_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).view.height }
}

#[no_mangle]
pub extern "C" fn ros_box_get_label(view: *const ros_box_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).view.label })
}

#[no_mangle]
pub extern "C" fn ros_box_get_score(view: *const ros_box_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).view.score }
}

#[no_mangle]
pub extern "C" fn ros_box_get_distance(view: *const ros_box_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).view.distance }
}

#[no_mangle]
pub extern "C" fn ros_box_get_speed(view: *const ros_box_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).view.speed }
}

#[no_mangle]
pub extern "C" fn ros_box_get_track_id(view: *const ros_box_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).view.track_id })
}

#[no_mangle]
pub extern "C" fn ros_box_get_track_lifetime(view: *const ros_box_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).view.track_lifetime }
}

/// @brief Get the box's track_created timestamp seconds component.
/// @param view Box handle
/// @return Time.sec or 0 if view is NULL
#[no_mangle]
pub extern "C" fn ros_box_get_track_created_sec(view: *const ros_box_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).view.track_created.sec }
}

/// @brief Get the box's track_created timestamp nanoseconds component.
/// @param view Box handle
/// @return Time.nanosec or 0 if view is NULL
#[no_mangle]
pub extern "C" fn ros_box_get_track_created_nanosec(view: *const ros_box_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).view.track_created.nanosec }
}

// =============================================================================
// LocalTime (buffer-backed)
// =============================================================================

pub struct ros_local_time_t(edgefirst_msgs::LocalTime<&'static [u8]>);

/// @brief Create a LocalTime view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set)
#[no_mangle]
pub extern "C" fn ros_local_time_from_cdr(data: *const u8, len: usize) -> *mut ros_local_time_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match edgefirst_msgs::LocalTime::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_local_time_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_local_time_free(view: *mut ros_local_time_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_local_time_get_stamp_sec(view: *const ros_local_time_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_local_time_get_stamp_nanosec(view: *const ros_local_time_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_local_time_get_frame_id(view: *const ros_local_time_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_local_time_get_timezone(view: *const ros_local_time_t) -> i16 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.timezone() }
}

// =============================================================================
// Generic CDR as_cdr accessor for any view type
// =============================================================================

/// Get the raw CDR bytes from any view handle.
/// The returned pointer is valid as long as the view handle lives.
/// `out_len` receives the byte count.
macro_rules! impl_as_cdr {
    ($fn_name:ident, $view_type:ty) => {
        #[no_mangle]
        pub extern "C" fn $fn_name(view: *const $view_type, out_len: *mut usize) -> *const u8 {
            if view.is_null() {
                if !out_len.is_null() {
                    unsafe {
                        *out_len = 0;
                    }
                }
                return ptr::null();
            }
            let cdr = unsafe { (*view).0.as_cdr() };
            unsafe {
                if !out_len.is_null() {
                    *out_len = cdr.len();
                }
            }
            cdr.as_ptr()
        }
    };
}

impl_as_cdr!(ros_header_as_cdr, ros_header_t);
impl_as_cdr!(ros_image_as_cdr, ros_image_t);
impl_as_cdr!(ros_compressed_image_as_cdr, ros_compressed_image_t);
impl_as_cdr!(ros_compressed_video_as_cdr, ros_compressed_video_t);
impl_as_cdr!(ros_dmabuffer_as_cdr, ros_dmabuffer_t);
impl_as_cdr!(ros_imu_as_cdr, ros_imu_t);
impl_as_cdr!(ros_nav_sat_fix_as_cdr, ros_nav_sat_fix_t);
impl_as_cdr!(ros_transform_stamped_as_cdr, ros_transform_stamped_t);
impl_as_cdr!(ros_radar_cube_as_cdr, ros_radar_cube_t);
impl_as_cdr!(ros_radar_info_as_cdr, ros_radar_info_t);
impl_as_cdr!(ros_model_info_as_cdr, ros_model_info_t);
impl_as_cdr!(ros_point_cloud2_as_cdr, ros_point_cloud2_t);
impl_as_cdr!(ros_camera_info_as_cdr, ros_camera_info_t);
impl_as_cdr!(ros_track_as_cdr, ros_track_t);
impl_as_cdr!(ros_local_time_as_cdr, ros_local_time_t);
impl_as_cdr!(ros_magnetic_field_as_cdr, ros_magnetic_field_t);
impl_as_cdr!(ros_fluid_pressure_as_cdr, ros_fluid_pressure_t);
impl_as_cdr!(ros_temperature_as_cdr, ros_temperature_t);
impl_as_cdr!(ros_battery_state_as_cdr, ros_battery_state_t);
impl_as_cdr!(ros_odometry_as_cdr, ros_odometry_t);
impl_as_cdr!(ros_vibration_as_cdr, ros_vibration_t);

// ros_detect_t and ros_model_t use named fields, so they need manual as_cdr impls.

#[no_mangle]
pub extern "C" fn ros_detect_as_cdr(view: *const ros_detect_t, out_len: *mut usize) -> *const u8 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    let cdr = unsafe { (*view).inner.as_cdr() };
    unsafe {
        if !out_len.is_null() {
            *out_len = cdr.len();
        }
    }
    cdr.as_ptr()
}

#[no_mangle]
pub extern "C" fn ros_model_as_cdr(view: *const ros_model_t, out_len: *mut usize) -> *const u8 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe {
                *out_len = 0;
            }
        }
        return ptr::null();
    }
    let cdr = unsafe { (*view).inner.as_cdr() };
    unsafe {
        if !out_len.is_null() {
            *out_len = cdr.len();
        }
    }
    cdr.as_ptr()
}

// ros_box_as_cdr and ros_mask_as_cdr have been removed. Forwarding an embedded
// child box/mask as a standalone CDR would require re-encoding, which violates
// the zero-copy contract. See CAPI.md for details.

// =============================================================================
// PoseWithCovariance (CdrFixed)
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_pose_with_covariance_encode(
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
    px: f64,
    py: f64,
    pz: f64,
    ox: f64,
    oy: f64,
    oz: f64,
    ow: f64,
    covariance: *const f64,
) -> i32 {
    if covariance.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let mut cov = [0.0_f64; 36];
    unsafe {
        ptr::copy_nonoverlapping(covariance, cov.as_mut_ptr(), 36);
    }
    let val = PoseWithCovariance {
        pose: Pose {
            position: Point {
                x: px,
                y: py,
                z: pz,
            },
            orientation: Quaternion {
                x: ox,
                y: oy,
                z: oz,
                w: ow,
            },
        },
        covariance: cov,
    };
    encode_fixed_to_buf(&val, buf, cap, written)
}

#[no_mangle]
pub extern "C" fn ros_pose_with_covariance_decode(
    data: *const u8,
    len: usize,
    px: *mut f64,
    py: *mut f64,
    pz: *mut f64,
    ox: *mut f64,
    oy: *mut f64,
    oz: *mut f64,
    ow: *mut f64,
    covariance_out: *mut f64,
) -> i32 {
    match decode_fixed_from_buf::<PoseWithCovariance>(data, len) {
        Ok(v) => unsafe {
            if !px.is_null() {
                *px = v.pose.position.x;
            }
            if !py.is_null() {
                *py = v.pose.position.y;
            }
            if !pz.is_null() {
                *pz = v.pose.position.z;
            }
            if !ox.is_null() {
                *ox = v.pose.orientation.x;
            }
            if !oy.is_null() {
                *oy = v.pose.orientation.y;
            }
            if !oz.is_null() {
                *oz = v.pose.orientation.z;
            }
            if !ow.is_null() {
                *ow = v.pose.orientation.w;
            }
            if !covariance_out.is_null() {
                ptr::copy_nonoverlapping(v.covariance.as_ptr(), covariance_out, 36);
            }
            0
        },
        Err(()) => -1,
    }
}

// =============================================================================
// TwistWithCovariance (CdrFixed)
// =============================================================================

#[no_mangle]
pub extern "C" fn ros_twist_with_covariance_encode(
    buf: *mut u8,
    cap: usize,
    written: *mut usize,
    lx: f64,
    ly: f64,
    lz: f64,
    ax: f64,
    ay: f64,
    az: f64,
    covariance: *const f64,
) -> i32 {
    if covariance.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let mut cov = [0.0_f64; 36];
    unsafe {
        ptr::copy_nonoverlapping(covariance, cov.as_mut_ptr(), 36);
    }
    let val = TwistWithCovariance {
        twist: Twist {
            linear: Vector3 {
                x: lx,
                y: ly,
                z: lz,
            },
            angular: Vector3 {
                x: ax,
                y: ay,
                z: az,
            },
        },
        covariance: cov,
    };
    encode_fixed_to_buf(&val, buf, cap, written)
}

#[no_mangle]
pub extern "C" fn ros_twist_with_covariance_decode(
    data: *const u8,
    len: usize,
    lx: *mut f64,
    ly: *mut f64,
    lz: *mut f64,
    ax: *mut f64,
    ay: *mut f64,
    az: *mut f64,
    covariance_out: *mut f64,
) -> i32 {
    match decode_fixed_from_buf::<TwistWithCovariance>(data, len) {
        Ok(v) => unsafe {
            if !lx.is_null() {
                *lx = v.twist.linear.x;
            }
            if !ly.is_null() {
                *ly = v.twist.linear.y;
            }
            if !lz.is_null() {
                *lz = v.twist.linear.z;
            }
            if !ax.is_null() {
                *ax = v.twist.angular.x;
            }
            if !ay.is_null() {
                *ay = v.twist.angular.y;
            }
            if !az.is_null() {
                *az = v.twist.angular.z;
            }
            if !covariance_out.is_null() {
                ptr::copy_nonoverlapping(v.covariance.as_ptr(), covariance_out, 36);
            }
            0
        },
        Err(()) => -1,
    }
}

// =============================================================================
// MagneticField (buffer-backed)
// =============================================================================

pub struct ros_magnetic_field_t(sensor_msgs::MagneticField<&'static [u8]>);

#[no_mangle]
pub extern "C" fn ros_magnetic_field_from_cdr(
    data: *const u8,
    len: usize,
) -> *mut ros_magnetic_field_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match sensor_msgs::MagneticField::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_magnetic_field_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_magnetic_field_free(view: *mut ros_magnetic_field_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_magnetic_field_get_stamp_sec(view: *const ros_magnetic_field_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_magnetic_field_get_stamp_nanosec(view: *const ros_magnetic_field_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_magnetic_field_get_frame_id(
    view: *const ros_magnetic_field_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_magnetic_field_get_magnetic_field(
    view: *const ros_magnetic_field_t,
    x: *mut f64,
    y: *mut f64,
    z: *mut f64,
) {
    if view.is_null() {
        return;
    }
    let v = unsafe { (*view).0.magnetic_field() };
    unsafe {
        if !x.is_null() {
            *x = v.x;
        }
        if !y.is_null() {
            *y = v.y;
        }
        if !z.is_null() {
            *z = v.z;
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_magnetic_field_get_magnetic_field_covariance(
    view: *const ros_magnetic_field_t,
    out: *mut f64,
) {
    if view.is_null() || out.is_null() {
        return;
    }
    let cov = unsafe { (*view).0.magnetic_field_covariance() };
    unsafe {
        ptr::copy_nonoverlapping(cov.as_ptr(), out, 9);
    }
}

// =============================================================================
// FluidPressure (buffer-backed)
// =============================================================================

pub struct ros_fluid_pressure_t(sensor_msgs::FluidPressure<&'static [u8]>);

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_from_cdr(
    data: *const u8,
    len: usize,
) -> *mut ros_fluid_pressure_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match sensor_msgs::FluidPressure::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_fluid_pressure_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_free(view: *mut ros_fluid_pressure_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_get_stamp_sec(view: *const ros_fluid_pressure_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_get_stamp_nanosec(view: *const ros_fluid_pressure_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_get_frame_id(
    view: *const ros_fluid_pressure_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_get_fluid_pressure(view: *const ros_fluid_pressure_t) -> f64 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.fluid_pressure() }
}

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_get_variance(view: *const ros_fluid_pressure_t) -> f64 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.variance() }
}

// =============================================================================
// Temperature (buffer-backed)
// =============================================================================

pub struct ros_temperature_t(sensor_msgs::Temperature<&'static [u8]>);

#[no_mangle]
pub extern "C" fn ros_temperature_from_cdr(data: *const u8, len: usize) -> *mut ros_temperature_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match sensor_msgs::Temperature::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_temperature_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_temperature_free(view: *mut ros_temperature_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_temperature_get_stamp_sec(view: *const ros_temperature_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_temperature_get_stamp_nanosec(view: *const ros_temperature_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_temperature_get_frame_id(view: *const ros_temperature_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_temperature_get_temperature(view: *const ros_temperature_t) -> f64 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.temperature() }
}

#[no_mangle]
pub extern "C" fn ros_temperature_get_variance(view: *const ros_temperature_t) -> f64 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.variance() }
}

// =============================================================================
// BatteryState (buffer-backed)
// =============================================================================

pub struct ros_battery_state_t(sensor_msgs::BatteryState<&'static [u8]>);

#[no_mangle]
pub extern "C" fn ros_battery_state_from_cdr(
    data: *const u8,
    len: usize,
) -> *mut ros_battery_state_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match sensor_msgs::BatteryState::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_battery_state_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_free(view: *mut ros_battery_state_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_stamp_sec(view: *const ros_battery_state_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_stamp_nanosec(view: *const ros_battery_state_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_frame_id(
    view: *const ros_battery_state_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_voltage(view: *const ros_battery_state_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.voltage() }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_temperature(view: *const ros_battery_state_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.temperature() }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_current(view: *const ros_battery_state_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.current() }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_charge(view: *const ros_battery_state_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.charge() }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_capacity(view: *const ros_battery_state_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.capacity() }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_design_capacity(view: *const ros_battery_state_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.design_capacity() }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_percentage(view: *const ros_battery_state_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.percentage() }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_power_supply_status(
    view: *const ros_battery_state_t,
) -> u8 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.power_supply_status() }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_power_supply_health(
    view: *const ros_battery_state_t,
) -> u8 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.power_supply_health() }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_power_supply_technology(
    view: *const ros_battery_state_t,
) -> u8 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.power_supply_technology() }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_present(view: *const ros_battery_state_t) -> bool {
    if view.is_null() {
        return false;
    }
    unsafe { (*view).0.present() }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_cell_voltage_len(view: *const ros_battery_state_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.cell_voltage_len() }
}

/// Copy up to `cap` cell voltages into `out`; returns the total element count.
///
/// Allocation-free: decodes directly from the backing CDR slice using the
/// cached sequence offset.
#[no_mangle]
pub extern "C" fn ros_battery_state_get_cell_voltage(
    view: *const ros_battery_state_t,
    out: *mut f32,
    cap: usize,
) -> u32 {
    if view.is_null() {
        return 0;
    }
    let msg = unsafe { &(*view).0 };
    copy_le_f32_seq(msg.as_cdr(), msg.cell_voltage_seq_offset(), out, cap)
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_cell_temperature_len(
    view: *const ros_battery_state_t,
) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.cell_temperature_len() }
}

/// Copy up to `cap` cell temperatures into `out`; returns the total element count.
///
/// Allocation-free: decodes directly from the backing CDR slice using the
/// cached sequence offset.
#[no_mangle]
pub extern "C" fn ros_battery_state_get_cell_temperature(
    view: *const ros_battery_state_t,
    out: *mut f32,
    cap: usize,
) -> u32 {
    if view.is_null() {
        return 0;
    }
    let msg = unsafe { &(*view).0 };
    copy_le_f32_seq(msg.as_cdr(), msg.cell_temperature_seq_offset(), out, cap)
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_location(
    view: *const ros_battery_state_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.location() })
}

#[no_mangle]
pub extern "C" fn ros_battery_state_get_serial_number(
    view: *const ros_battery_state_t,
) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.serial_number() })
}

// =============================================================================
// Odometry (buffer-backed)
// =============================================================================

pub struct ros_odometry_t(nav_msgs::Odometry<&'static [u8]>);

#[no_mangle]
pub extern "C" fn ros_odometry_from_cdr(data: *const u8, len: usize) -> *mut ros_odometry_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match nav_msgs::Odometry::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_odometry_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_odometry_free(view: *mut ros_odometry_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_odometry_get_stamp_sec(view: *const ros_odometry_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_odometry_get_stamp_nanosec(view: *const ros_odometry_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_odometry_get_frame_id(view: *const ros_odometry_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_odometry_get_child_frame_id(view: *const ros_odometry_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.child_frame_id() })
}

/// Write pose position (x,y,z) + orientation (x,y,z,w) out.
#[no_mangle]
pub extern "C" fn ros_odometry_get_pose(
    view: *const ros_odometry_t,
    px: *mut f64,
    py: *mut f64,
    pz: *mut f64,
    ox: *mut f64,
    oy: *mut f64,
    oz: *mut f64,
    ow: *mut f64,
) {
    if view.is_null() {
        return;
    }
    let p = unsafe { (*view).0.pose() };
    unsafe {
        if !px.is_null() {
            *px = p.pose.position.x;
        }
        if !py.is_null() {
            *py = p.pose.position.y;
        }
        if !pz.is_null() {
            *pz = p.pose.position.z;
        }
        if !ox.is_null() {
            *ox = p.pose.orientation.x;
        }
        if !oy.is_null() {
            *oy = p.pose.orientation.y;
        }
        if !oz.is_null() {
            *oz = p.pose.orientation.z;
        }
        if !ow.is_null() {
            *ow = p.pose.orientation.w;
        }
    }
}

/// Write 36-element pose covariance.
#[no_mangle]
pub extern "C" fn ros_odometry_get_pose_covariance(view: *const ros_odometry_t, out: *mut f64) {
    if view.is_null() || out.is_null() {
        return;
    }
    let p = unsafe { (*view).0.pose() };
    unsafe {
        ptr::copy_nonoverlapping(p.covariance.as_ptr(), out, 36);
    }
}

/// Write twist linear (x,y,z) + angular (x,y,z).
#[no_mangle]
pub extern "C" fn ros_odometry_get_twist(
    view: *const ros_odometry_t,
    lx: *mut f64,
    ly: *mut f64,
    lz: *mut f64,
    ax: *mut f64,
    ay: *mut f64,
    az: *mut f64,
) {
    if view.is_null() {
        return;
    }
    let t = unsafe { (*view).0.twist() };
    unsafe {
        if !lx.is_null() {
            *lx = t.twist.linear.x;
        }
        if !ly.is_null() {
            *ly = t.twist.linear.y;
        }
        if !lz.is_null() {
            *lz = t.twist.linear.z;
        }
        if !ax.is_null() {
            *ax = t.twist.angular.x;
        }
        if !ay.is_null() {
            *ay = t.twist.angular.y;
        }
        if !az.is_null() {
            *az = t.twist.angular.z;
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_odometry_get_twist_covariance(view: *const ros_odometry_t, out: *mut f64) {
    if view.is_null() || out.is_null() {
        return;
    }
    let t = unsafe { (*view).0.twist() };
    unsafe {
        ptr::copy_nonoverlapping(t.covariance.as_ptr(), out, 36);
    }
}

// =============================================================================
// Vibration (buffer-backed)
// =============================================================================

pub struct ros_vibration_t(edgefirst_msgs::Vibration<&'static [u8]>);

#[no_mangle]
pub extern "C" fn ros_vibration_from_cdr(data: *const u8, len: usize) -> *mut ros_vibration_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match edgefirst_msgs::Vibration::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(ros_vibration_t(v))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_vibration_free(view: *mut ros_vibration_t) {
    if !view.is_null() {
        unsafe {
            drop(Box::from_raw(view));
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_vibration_get_stamp_sec(view: *const ros_vibration_t) -> i32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().sec }
}

#[no_mangle]
pub extern "C" fn ros_vibration_get_stamp_nanosec(view: *const ros_vibration_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.stamp().nanosec }
}

#[no_mangle]
pub extern "C" fn ros_vibration_get_frame_id(view: *const ros_vibration_t) -> *const c_char {
    if view.is_null() {
        return ptr::null();
    }
    str_as_c(unsafe { (*view).0.frame_id() })
}

#[no_mangle]
pub extern "C" fn ros_vibration_get_measurement_type(view: *const ros_vibration_t) -> u8 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.measurement_type() }
}

#[no_mangle]
pub extern "C" fn ros_vibration_get_unit(view: *const ros_vibration_t) -> u8 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.unit() }
}

#[no_mangle]
pub extern "C" fn ros_vibration_get_band_lower_hz(view: *const ros_vibration_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.band_lower_hz() }
}

#[no_mangle]
pub extern "C" fn ros_vibration_get_band_upper_hz(view: *const ros_vibration_t) -> f32 {
    if view.is_null() {
        return 0.0;
    }
    unsafe { (*view).0.band_upper_hz() }
}

#[no_mangle]
pub extern "C" fn ros_vibration_get_vibration(
    view: *const ros_vibration_t,
    x: *mut f64,
    y: *mut f64,
    z: *mut f64,
) {
    if view.is_null() {
        return;
    }
    let v = unsafe { (*view).0.vibration() };
    unsafe {
        if !x.is_null() {
            *x = v.x;
        }
        if !y.is_null() {
            *y = v.y;
        }
        if !z.is_null() {
            *z = v.z;
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_vibration_get_clipping_len(view: *const ros_vibration_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).0.clipping_len() }
}

/// Copy up to `cap` clipping counters into `out`; returns the total element count.
///
/// Allocation-free: decodes directly from the backing CDR slice using the
/// cached sequence offset.
#[no_mangle]
pub extern "C" fn ros_vibration_get_clipping(
    view: *const ros_vibration_t,
    out: *mut u32,
    cap: usize,
) -> u32 {
    if view.is_null() {
        return 0;
    }
    let msg = unsafe { &(*view).0 };
    copy_le_u32_seq(msg.as_cdr(), msg.clipping_seq_offset(), out, cap)
}
