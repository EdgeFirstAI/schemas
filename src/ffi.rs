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
use crate::sensor_msgs::{self, NavSatStatus, PointFieldView, RegionOfInterest};
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

/// Validate that a C string pointer is non-NULL and points at valid UTF-8.
///
/// Used by builder setters that take `*const c_char`. On NULL or invalid
/// UTF-8 input, sets `errno = EINVAL` and returns `Err(())` — the caller
/// returns `-1` to the C ABI. The returned `&str` borrows from the caller's
/// buffer; the caller must copy it (e.g. via `.to_string()`) before the
/// pointer can be invalidated.
///
/// # Safety
/// `s` must either be NULL or a valid NUL-terminated C string.
unsafe fn c_to_str_checked<'a>(s: *const c_char) -> Result<&'a str, ()> {
    if s.is_null() {
        set_errno(EINVAL);
        return Err(());
    }
    let bytes = std::ffi::CStr::from_ptr(s).to_bytes();
    match std::str::from_utf8(bytes) {
        Ok(v) => Ok(v),
        Err(_) => {
            set_errno(EINVAL);
            Err(())
        }
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
    let v = match sensor_msgs::CompressedImage::builder()
        .stamp(Time::new(stamp_sec, stamp_nanosec))
        .frame_id(fid)
        .format(fmt)
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

/// Alias for `ros_compressed_video_get_stamp_sec`; matches the Foxglove schema field name.
#[no_mangle]
pub unsafe extern "C" fn ros_compressed_video_get_timestamp_sec(
    view: *const ros_compressed_video_t,
) -> i32 {
    ros_compressed_video_get_stamp_sec(view)
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

/// Alias for `ros_compressed_video_get_stamp_nanosec`; matches the Foxglove schema field name.
#[no_mangle]
pub unsafe extern "C" fn ros_compressed_video_get_timestamp_nanosec(
    view: *const ros_compressed_video_t,
) -> u32 {
    ros_compressed_video_get_stamp_nanosec(view)
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

// =============================================================================
// Builder handles (3.2.0+)
//
// Opaque handle + per-field setters + build/encode_into finalizers.
//
// Internal state owns strings (copied from the C string at set-time) and
// borrows bulk byte / view sequences as raw `*const u8 + usize` pairs. The
// caller contract is that any borrowed data remains valid until the next
// setter on that field, the next `build` / `encode_into`, or `free`.
//
// The legacy `ros_<type>_encode` one-shot functions remain in 3.2.0 for
// compatibility but are deprecated and slated for removal in 4.0; new code
// should prefer the builder API to avoid argument-list explosion as message
// shapes grow.
//
// Errno conventions match the existing FFI:
//   * EINVAL  — NULL handle or required-NULL argument.
//   * EBADMSG — encoder rejected the staged fields.
//   * ENOBUFS — `encode_into` was called with too small a destination.
// =============================================================================

// ── std_msgs::Header ────────────────────────────────────────────────

struct HeaderBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
}

pub struct ros_header_builder_t(HeaderBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_header_builder_new() -> *mut ros_header_builder_t {
    Box::into_raw(Box::new(ros_header_builder_t(HeaderBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
    })))
}

#[no_mangle]
pub extern "C" fn ros_header_builder_free(b: *mut ros_header_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_header_builder_set_stamp(
    b: *mut ros_header_builder_t,
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
pub extern "C" fn ros_header_builder_set_frame_id(
    b: *mut ros_header_builder_t,
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
pub extern "C" fn ros_header_builder_build(
    b: *mut ros_header_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = std_msgs::Header::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_header_builder_encode_into(
    b: *mut ros_header_builder_t,
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
    let r = std_msgs::Header::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── sensor_msgs::Image ──────────────────────────────────────────────

struct ImageBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    height: u32,
    width: u32,
    encoding: String,
    is_bigendian: u8,
    step: u32,
    data: *const u8,
    data_len: usize,
}

pub struct ros_image_builder_t(ImageBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_image_builder_new() -> *mut ros_image_builder_t {
    Box::into_raw(Box::new(ros_image_builder_t(ImageBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        height: 0,
        width: 0,
        encoding: String::new(),
        is_bigendian: 0,
        step: 0,
        data: ptr::null(),
        data_len: 0,
    })))
}

#[no_mangle]
pub extern "C" fn ros_image_builder_free(b: *mut ros_image_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_image_builder_set_stamp(b: *mut ros_image_builder_t, sec: i32, nanosec: u32) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.stamp_sec = sec;
    inner.stamp_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_image_builder_set_frame_id(
    b: *mut ros_image_builder_t,
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
pub extern "C" fn ros_image_builder_set_height(b: *mut ros_image_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.height = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_image_builder_set_width(b: *mut ros_image_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.width = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_image_builder_set_encoding(
    b: *mut ros_image_builder_t,
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
        (*b).0.encoding = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_image_builder_set_is_bigendian(b: *mut ros_image_builder_t, v: u8) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.is_bigendian = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_image_builder_set_step(b: *mut ros_image_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.step = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_image_builder_set_data(
    b: *mut ros_image_builder_t,
    data: *const u8,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.data = data;
        (*b).0.data_len = len;
    }
    0
}

fn ros_image_builder_data_slice(inner: &ImageBuilderOwned) -> &[u8] {
    if inner.data.is_null() || inner.data_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.data, inner.data_len) }
    }
}

#[no_mangle]
pub extern "C" fn ros_image_builder_build(
    b: *mut ros_image_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let data_slice = ros_image_builder_data_slice(inner);
    let r = sensor_msgs::Image::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .height(inner.height)
        .width(inner.width)
        .encoding(inner.encoding.as_str())
        .is_bigendian(inner.is_bigendian)
        .step(inner.step)
        .data(data_slice)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_image_builder_encode_into(
    b: *mut ros_image_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let data_slice = ros_image_builder_data_slice(inner);
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    let r = sensor_msgs::Image::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .height(inner.height)
        .width(inner.width)
        .encoding(inner.encoding.as_str())
        .is_bigendian(inner.is_bigendian)
        .step(inner.step)
        .data(data_slice)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── sensor_msgs::FluidPressure ──────────────────────────────────────

struct FluidPressureBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    fluid_pressure: f64,
    variance: f64,
}

pub struct ros_fluid_pressure_builder_t(FluidPressureBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_builder_new() -> *mut ros_fluid_pressure_builder_t {
    Box::into_raw(Box::new(ros_fluid_pressure_builder_t(
        FluidPressureBuilderOwned {
            stamp_sec: 0,
            stamp_nanosec: 0,
            frame_id: String::new(),
            fluid_pressure: 0.0,
            variance: 0.0,
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_builder_free(b: *mut ros_fluid_pressure_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_builder_set_stamp(
    b: *mut ros_fluid_pressure_builder_t,
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
pub extern "C" fn ros_fluid_pressure_builder_set_frame_id(
    b: *mut ros_fluid_pressure_builder_t,
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
pub extern "C" fn ros_fluid_pressure_builder_set_fluid_pressure(
    b: *mut ros_fluid_pressure_builder_t,
    v: f64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.fluid_pressure = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_builder_set_variance(
    b: *mut ros_fluid_pressure_builder_t,
    v: f64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.variance = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_builder_build(
    b: *mut ros_fluid_pressure_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = sensor_msgs::FluidPressure::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .fluid_pressure(inner.fluid_pressure)
        .variance(inner.variance)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_fluid_pressure_builder_encode_into(
    b: *mut ros_fluid_pressure_builder_t,
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
    let r = sensor_msgs::FluidPressure::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .fluid_pressure(inner.fluid_pressure)
        .variance(inner.variance)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── sensor_msgs::CompressedImage ────────────────────────────────────

struct CompressedImageBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    format: String,
    data: *const u8,
    data_len: usize,
}

pub struct ros_compressed_image_builder_t(CompressedImageBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_compressed_image_builder_new() -> *mut ros_compressed_image_builder_t {
    Box::into_raw(Box::new(ros_compressed_image_builder_t(
        CompressedImageBuilderOwned {
            stamp_sec: 0,
            stamp_nanosec: 0,
            frame_id: String::new(),
            format: String::new(),
            data: ptr::null(),
            data_len: 0,
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_builder_free(b: *mut ros_compressed_image_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_builder_set_stamp(
    b: *mut ros_compressed_image_builder_t,
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
pub extern "C" fn ros_compressed_image_builder_set_frame_id(
    b: *mut ros_compressed_image_builder_t,
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
pub extern "C" fn ros_compressed_image_builder_set_format(
    b: *mut ros_compressed_image_builder_t,
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
        (*b).0.format = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_builder_set_data(
    b: *mut ros_compressed_image_builder_t,
    data: *const u8,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.data = data;
        (*b).0.data_len = len;
    }
    0
}

fn ros_compressed_image_builder_data_slice(inner: &CompressedImageBuilderOwned) -> &[u8] {
    if inner.data.is_null() || inner.data_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.data, inner.data_len) }
    }
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_builder_build(
    b: *mut ros_compressed_image_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let data_slice = ros_compressed_image_builder_data_slice(inner);
    let r = sensor_msgs::CompressedImage::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .format(inner.format.as_str())
        .data(data_slice)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_compressed_image_builder_encode_into(
    b: *mut ros_compressed_image_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let data_slice = ros_compressed_image_builder_data_slice(inner);
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    let r = sensor_msgs::CompressedImage::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .format(inner.format.as_str())
        .data(data_slice)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── sensor_msgs::Imu ────────────────────────────────────────────────

struct ImuBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    orientation: Quaternion,
    orientation_covariance: [f64; 9],
    angular_velocity: Vector3,
    angular_velocity_covariance: [f64; 9],
    linear_acceleration: Vector3,
    linear_acceleration_covariance: [f64; 9],
}

pub struct ros_imu_builder_t(ImuBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_imu_builder_new() -> *mut ros_imu_builder_t {
    Box::into_raw(Box::new(ros_imu_builder_t(ImuBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        orientation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        },
        orientation_covariance: [0.0; 9],
        angular_velocity: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        angular_velocity_covariance: [0.0; 9],
        linear_acceleration: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        linear_acceleration_covariance: [0.0; 9],
    })))
}

#[no_mangle]
pub extern "C" fn ros_imu_builder_free(b: *mut ros_imu_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_imu_builder_set_stamp(b: *mut ros_imu_builder_t, sec: i32, nanosec: u32) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.stamp_sec = sec;
    inner.stamp_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_imu_builder_set_frame_id(b: *mut ros_imu_builder_t, s: *const c_char) -> i32 {
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
pub extern "C" fn ros_imu_builder_set_orientation(
    b: *mut ros_imu_builder_t,
    x: f64,
    y: f64,
    z: f64,
    w: f64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.orientation = Quaternion { x, y, z, w };
    }
}

/// Copies 9 f64 elements from `cov` into the builder's orientation_covariance.
/// Caller contract: `cov` must point to at least 9 valid f64 values.
#[no_mangle]
pub extern "C" fn ros_imu_builder_set_orientation_covariance(
    b: *mut ros_imu_builder_t,
    cov: *const f64,
) -> i32 {
    if b.is_null() || cov.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        let src = slice::from_raw_parts(cov, 9);
        (*b).0.orientation_covariance.copy_from_slice(src);
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_imu_builder_set_angular_velocity(
    b: *mut ros_imu_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.angular_velocity = Vector3 { x, y, z };
    }
}

#[no_mangle]
pub extern "C" fn ros_imu_builder_set_angular_velocity_covariance(
    b: *mut ros_imu_builder_t,
    cov: *const f64,
) -> i32 {
    if b.is_null() || cov.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        let src = slice::from_raw_parts(cov, 9);
        (*b).0.angular_velocity_covariance.copy_from_slice(src);
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_imu_builder_set_linear_acceleration(
    b: *mut ros_imu_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.linear_acceleration = Vector3 { x, y, z };
    }
}

#[no_mangle]
pub extern "C" fn ros_imu_builder_set_linear_acceleration_covariance(
    b: *mut ros_imu_builder_t,
    cov: *const f64,
) -> i32 {
    if b.is_null() || cov.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        let src = slice::from_raw_parts(cov, 9);
        (*b).0.linear_acceleration_covariance.copy_from_slice(src);
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_imu_builder_build(
    b: *mut ros_imu_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = sensor_msgs::Imu::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .orientation(inner.orientation)
        .orientation_covariance(inner.orientation_covariance)
        .angular_velocity(inner.angular_velocity)
        .angular_velocity_covariance(inner.angular_velocity_covariance)
        .linear_acceleration(inner.linear_acceleration)
        .linear_acceleration_covariance(inner.linear_acceleration_covariance)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_imu_builder_encode_into(
    b: *mut ros_imu_builder_t,
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
    let r = sensor_msgs::Imu::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .orientation(inner.orientation)
        .orientation_covariance(inner.orientation_covariance)
        .angular_velocity(inner.angular_velocity)
        .angular_velocity_covariance(inner.angular_velocity_covariance)
        .linear_acceleration(inner.linear_acceleration)
        .linear_acceleration_covariance(inner.linear_acceleration_covariance)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── sensor_msgs::NavSatFix ──────────────────────────────────────────

struct NavSatFixBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    status: NavSatStatus,
    latitude: f64,
    longitude: f64,
    altitude: f64,
    position_covariance: [f64; 9],
    position_covariance_type: u8,
}

pub struct ros_nav_sat_fix_builder_t(NavSatFixBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_builder_new() -> *mut ros_nav_sat_fix_builder_t {
    Box::into_raw(Box::new(ros_nav_sat_fix_builder_t(NavSatFixBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        status: NavSatStatus {
            status: 0,
            service: 0,
        },
        latitude: 0.0,
        longitude: 0.0,
        altitude: 0.0,
        position_covariance: [0.0; 9],
        position_covariance_type: 0,
    })))
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_builder_free(b: *mut ros_nav_sat_fix_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_builder_set_stamp(
    b: *mut ros_nav_sat_fix_builder_t,
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
pub extern "C" fn ros_nav_sat_fix_builder_set_frame_id(
    b: *mut ros_nav_sat_fix_builder_t,
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
pub extern "C" fn ros_nav_sat_fix_builder_set_status(
    b: *mut ros_nav_sat_fix_builder_t,
    status: i8,
    service: u16,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.status = NavSatStatus { status, service };
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_builder_set_latitude(b: *mut ros_nav_sat_fix_builder_t, v: f64) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.latitude = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_builder_set_longitude(b: *mut ros_nav_sat_fix_builder_t, v: f64) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.longitude = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_builder_set_altitude(b: *mut ros_nav_sat_fix_builder_t, v: f64) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.altitude = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_builder_set_position_covariance(
    b: *mut ros_nav_sat_fix_builder_t,
    cov: *const f64,
) -> i32 {
    if b.is_null() || cov.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        let src = slice::from_raw_parts(cov, 9);
        (*b).0.position_covariance.copy_from_slice(src);
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_builder_set_position_covariance_type(
    b: *mut ros_nav_sat_fix_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.position_covariance_type = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_builder_build(
    b: *mut ros_nav_sat_fix_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = sensor_msgs::NavSatFix::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .status(inner.status)
        .latitude(inner.latitude)
        .longitude(inner.longitude)
        .altitude(inner.altitude)
        .position_covariance(inner.position_covariance)
        .position_covariance_type(inner.position_covariance_type)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_builder_encode_into(
    b: *mut ros_nav_sat_fix_builder_t,
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
    let r = sensor_msgs::NavSatFix::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .status(inner.status)
        .latitude(inner.latitude)
        .longitude(inner.longitude)
        .altitude(inner.altitude)
        .position_covariance(inner.position_covariance)
        .position_covariance_type(inner.position_covariance_type)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── sensor_msgs::PointField ─────────────────────────────────────────

struct PointFieldBuilderOwned {
    name: String,
    offset: u32,
    datatype: u8,
    count: u32,
}

pub struct ros_point_field_builder_t(PointFieldBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_point_field_builder_new() -> *mut ros_point_field_builder_t {
    Box::into_raw(Box::new(ros_point_field_builder_t(
        PointFieldBuilderOwned {
            name: String::new(),
            offset: 0,
            datatype: 0,
            count: 0,
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_point_field_builder_free(b: *mut ros_point_field_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_builder_set_name(
    b: *mut ros_point_field_builder_t,
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
        (*b).0.name = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_point_field_builder_set_offset(b: *mut ros_point_field_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.offset = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_builder_set_datatype(b: *mut ros_point_field_builder_t, v: u8) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.datatype = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_builder_set_count(b: *mut ros_point_field_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.count = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_builder_build(
    b: *mut ros_point_field_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = sensor_msgs::PointField::builder()
        .name(inner.name.as_str())
        .offset(inner.offset)
        .datatype(inner.datatype)
        .count(inner.count)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_point_field_builder_encode_into(
    b: *mut ros_point_field_builder_t,
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
    let r = sensor_msgs::PointField::builder()
        .name(inner.name.as_str())
        .offset(inner.offset)
        .datatype(inner.datatype)
        .count(inner.count)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── sensor_msgs::PointCloud2 ────────────────────────────────────────
//
// Field-sequence elements are passed as a C-POD array of descriptors; each
// descriptor names a field whose `name` string must outlive the next builder
// setter/build/encode_into/free.

/// C-POD descriptor for a single PointField element used by
/// `ros_point_cloud2_builder_set_fields`. The `name` pointer is borrowed:
/// the caller must keep the backing string alive until the next setter on
/// the fields slot or the builder is freed.
#[repr(C)]
pub struct ros_point_field_elem_t {
    pub name: *const c_char,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

struct PointCloud2BuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    height: u32,
    width: u32,
    fields: *const ros_point_field_elem_t,
    fields_count: usize,
    is_bigendian: bool,
    point_step: u32,
    row_step: u32,
    data: *const u8,
    data_len: usize,
    is_dense: bool,
}

pub struct ros_point_cloud2_builder_t(PointCloud2BuilderOwned);

#[no_mangle]
pub extern "C" fn ros_point_cloud2_builder_new() -> *mut ros_point_cloud2_builder_t {
    Box::into_raw(Box::new(ros_point_cloud2_builder_t(
        PointCloud2BuilderOwned {
            stamp_sec: 0,
            stamp_nanosec: 0,
            frame_id: String::new(),
            height: 0,
            width: 0,
            fields: ptr::null(),
            fields_count: 0,
            is_bigendian: false,
            point_step: 0,
            row_step: 0,
            data: ptr::null(),
            data_len: 0,
            is_dense: false,
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_builder_free(b: *mut ros_point_cloud2_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_builder_set_stamp(
    b: *mut ros_point_cloud2_builder_t,
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
pub extern "C" fn ros_point_cloud2_builder_set_frame_id(
    b: *mut ros_point_cloud2_builder_t,
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
pub extern "C" fn ros_point_cloud2_builder_set_height(b: *mut ros_point_cloud2_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.height = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_builder_set_width(b: *mut ros_point_cloud2_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.width = v;
    }
}

/// Set the field descriptor sequence (BORROWED — `fields` and every `name`
/// pointer inside it must remain valid until the next setter on the fields
/// slot, a subsequent build/encode_into, or free).
#[no_mangle]
pub extern "C" fn ros_point_cloud2_builder_set_fields(
    b: *mut ros_point_cloud2_builder_t,
    fields: *const ros_point_field_elem_t,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if fields.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.fields = fields;
        (*b).0.fields_count = count;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_builder_set_is_bigendian(
    b: *mut ros_point_cloud2_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.is_bigendian = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_builder_set_point_step(
    b: *mut ros_point_cloud2_builder_t,
    v: u32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.point_step = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_builder_set_row_step(
    b: *mut ros_point_cloud2_builder_t,
    v: u32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.row_step = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_builder_set_data(
    b: *mut ros_point_cloud2_builder_t,
    data: *const u8,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.data = data;
        (*b).0.data_len = len;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_builder_set_is_dense(
    b: *mut ros_point_cloud2_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.is_dense = v;
    }
}

fn ros_point_cloud2_builder_data_slice(inner: &PointCloud2BuilderOwned) -> &[u8] {
    if inner.data.is_null() || inner.data_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.data, inner.data_len) }
    }
}

/// Materialise PointFieldView borrowers from the owned C-POD descriptor array.
///
/// # Safety
/// Each descriptor's `name` pointer must be a valid NUL-terminated C string
/// (or NULL, treated as "") whose backing storage outlives the returned Vec.
unsafe fn point_cloud2_fields_to_views(
    inner: &PointCloud2BuilderOwned,
) -> Result<Vec<PointFieldView<'_>>, ()> {
    if inner.fields.is_null() || inner.fields_count == 0 {
        return Ok(Vec::new());
    }
    let descs = slice::from_raw_parts(inner.fields, inner.fields_count);
    descs
        .iter()
        .map(|d| {
            let name = c_to_str_checked(d.name)?;
            Ok(PointFieldView {
                name,
                offset: d.offset,
                datatype: d.datatype,
                count: d.count,
            })
        })
        .collect()
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_builder_build(
    b: *mut ros_point_cloud2_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let data_slice = ros_point_cloud2_builder_data_slice(inner);
    let fields = match unsafe { point_cloud2_fields_to_views(inner) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    let r = sensor_msgs::PointCloud2::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .height(inner.height)
        .width(inner.width)
        .fields(&fields)
        .is_bigendian(inner.is_bigendian)
        .point_step(inner.point_step)
        .row_step(inner.row_step)
        .data(data_slice)
        .is_dense(inner.is_dense)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_point_cloud2_builder_encode_into(
    b: *mut ros_point_cloud2_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let data_slice = ros_point_cloud2_builder_data_slice(inner);
    let fields = match unsafe { point_cloud2_fields_to_views(inner) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    let r = sensor_msgs::PointCloud2::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .height(inner.height)
        .width(inner.width)
        .fields(&fields)
        .is_bigendian(inner.is_bigendian)
        .point_step(inner.point_step)
        .row_step(inner.row_step)
        .data(data_slice)
        .is_dense(inner.is_dense)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── sensor_msgs::CameraInfo ─────────────────────────────────────────

struct CameraInfoBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    height: u32,
    width: u32,
    distortion_model: String,
    d: *const f64,
    d_len: usize,
    k: [f64; 9],
    r: [f64; 9],
    p: [f64; 12],
    binning_x: u32,
    binning_y: u32,
    roi: RegionOfInterest,
}

pub struct ros_camera_info_builder_t(CameraInfoBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_camera_info_builder_new() -> *mut ros_camera_info_builder_t {
    Box::into_raw(Box::new(ros_camera_info_builder_t(
        CameraInfoBuilderOwned {
            stamp_sec: 0,
            stamp_nanosec: 0,
            frame_id: String::new(),
            height: 0,
            width: 0,
            distortion_model: String::new(),
            d: ptr::null(),
            d_len: 0,
            k: [0.0; 9],
            r: [0.0; 9],
            p: [0.0; 12],
            binning_x: 0,
            binning_y: 0,
            roi: RegionOfInterest {
                x_offset: 0,
                y_offset: 0,
                height: 0,
                width: 0,
                do_rectify: false,
            },
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_camera_info_builder_free(b: *mut ros_camera_info_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_builder_set_stamp(
    b: *mut ros_camera_info_builder_t,
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
pub extern "C" fn ros_camera_info_builder_set_frame_id(
    b: *mut ros_camera_info_builder_t,
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
pub extern "C" fn ros_camera_info_builder_set_height(b: *mut ros_camera_info_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.height = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_builder_set_width(b: *mut ros_camera_info_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.width = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_builder_set_distortion_model(
    b: *mut ros_camera_info_builder_t,
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
        (*b).0.distortion_model = s_str.to_string();
    }
    0
}

/// Set the distortion coefficients (BORROWED `[f64; d_len]` slice —
/// the pointer must remain valid until the next setter on this slot,
/// a subsequent build/encode_into, or free).
#[no_mangle]
pub extern "C" fn ros_camera_info_builder_set_d(
    b: *mut ros_camera_info_builder_t,
    data: *const f64,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.d = data;
        (*b).0.d_len = len;
    }
    0
}

/// Copy 9 f64 elements from `k` into the intrinsics matrix (row-major 3x3).
#[no_mangle]
pub extern "C" fn ros_camera_info_builder_set_k(
    b: *mut ros_camera_info_builder_t,
    k: *const f64,
) -> i32 {
    if b.is_null() || k.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        let src = slice::from_raw_parts(k, 9);
        (*b).0.k.copy_from_slice(src);
    }
    0
}

/// Copy 9 f64 elements from `r` into the rectification matrix (row-major 3x3).
#[no_mangle]
pub extern "C" fn ros_camera_info_builder_set_r(
    b: *mut ros_camera_info_builder_t,
    r: *const f64,
) -> i32 {
    if b.is_null() || r.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        let src = slice::from_raw_parts(r, 9);
        (*b).0.r.copy_from_slice(src);
    }
    0
}

/// Copy 12 f64 elements from `p` into the projection matrix (row-major 3x4).
#[no_mangle]
pub extern "C" fn ros_camera_info_builder_set_p(
    b: *mut ros_camera_info_builder_t,
    p: *const f64,
) -> i32 {
    if b.is_null() || p.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        let src = slice::from_raw_parts(p, 12);
        (*b).0.p.copy_from_slice(src);
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_camera_info_builder_set_binning_x(b: *mut ros_camera_info_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.binning_x = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_builder_set_binning_y(b: *mut ros_camera_info_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.binning_y = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_builder_set_roi(
    b: *mut ros_camera_info_builder_t,
    x_offset: u32,
    y_offset: u32,
    height: u32,
    width: u32,
    do_rectify: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.roi = RegionOfInterest {
            x_offset,
            y_offset,
            height,
            width,
            do_rectify: do_rectify != 0,
        };
    }
}

fn ros_camera_info_builder_d_slice(inner: &CameraInfoBuilderOwned) -> &[f64] {
    if inner.d.is_null() || inner.d_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.d, inner.d_len) }
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_builder_build(
    b: *mut ros_camera_info_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let d_slice = ros_camera_info_builder_d_slice(inner);
    let r = sensor_msgs::CameraInfo::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .height(inner.height)
        .width(inner.width)
        .distortion_model(inner.distortion_model.as_str())
        .d(d_slice)
        .k(inner.k)
        .r(inner.r)
        .p(inner.p)
        .binning_x(inner.binning_x)
        .binning_y(inner.binning_y)
        .roi(inner.roi)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_info_builder_encode_into(
    b: *mut ros_camera_info_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let d_slice = ros_camera_info_builder_d_slice(inner);
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    let r = sensor_msgs::CameraInfo::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .height(inner.height)
        .width(inner.width)
        .distortion_model(inner.distortion_model.as_str())
        .d(d_slice)
        .k(inner.k)
        .r(inner.r)
        .p(inner.p)
        .binning_x(inner.binning_x)
        .binning_y(inner.binning_y)
        .roi(inner.roi)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── sensor_msgs::MagneticField ──────────────────────────────────────

struct MagneticFieldBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    magnetic_field: Vector3,
    magnetic_field_covariance: [f64; 9],
}

pub struct ros_magnetic_field_builder_t(MagneticFieldBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_magnetic_field_builder_new() -> *mut ros_magnetic_field_builder_t {
    Box::into_raw(Box::new(ros_magnetic_field_builder_t(
        MagneticFieldBuilderOwned {
            stamp_sec: 0,
            stamp_nanosec: 0,
            frame_id: String::new(),
            magnetic_field: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            magnetic_field_covariance: [0.0; 9],
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_magnetic_field_builder_free(b: *mut ros_magnetic_field_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_magnetic_field_builder_set_stamp(
    b: *mut ros_magnetic_field_builder_t,
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
pub extern "C" fn ros_magnetic_field_builder_set_frame_id(
    b: *mut ros_magnetic_field_builder_t,
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
pub extern "C" fn ros_magnetic_field_builder_set_magnetic_field(
    b: *mut ros_magnetic_field_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.magnetic_field = Vector3 { x, y, z };
    }
}

#[no_mangle]
pub extern "C" fn ros_magnetic_field_builder_set_magnetic_field_covariance(
    b: *mut ros_magnetic_field_builder_t,
    cov: *const f64,
) -> i32 {
    if b.is_null() || cov.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        let src = slice::from_raw_parts(cov, 9);
        (*b).0.magnetic_field_covariance.copy_from_slice(src);
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_magnetic_field_builder_build(
    b: *mut ros_magnetic_field_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = sensor_msgs::MagneticField::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .magnetic_field(inner.magnetic_field)
        .magnetic_field_covariance(inner.magnetic_field_covariance)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_magnetic_field_builder_encode_into(
    b: *mut ros_magnetic_field_builder_t,
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
    let r = sensor_msgs::MagneticField::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .magnetic_field(inner.magnetic_field)
        .magnetic_field_covariance(inner.magnetic_field_covariance)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── sensor_msgs::BatteryState ───────────────────────────────────────

#[allow(clippy::struct_excessive_bools)]
struct BatteryStateBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    voltage: f32,
    temperature: f32,
    current: f32,
    charge: f32,
    capacity: f32,
    design_capacity: f32,
    percentage: f32,
    power_supply_status: u8,
    power_supply_health: u8,
    power_supply_technology: u8,
    present: bool,
    cell_voltage: *const f32,
    cell_voltage_len: usize,
    cell_temperature: *const f32,
    cell_temperature_len: usize,
    location: String,
    serial_number: String,
}

pub struct ros_battery_state_builder_t(BatteryStateBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_new() -> *mut ros_battery_state_builder_t {
    Box::into_raw(Box::new(ros_battery_state_builder_t(
        BatteryStateBuilderOwned {
            stamp_sec: 0,
            stamp_nanosec: 0,
            frame_id: String::new(),
            voltage: 0.0,
            temperature: 0.0,
            current: 0.0,
            charge: 0.0,
            capacity: 0.0,
            design_capacity: 0.0,
            percentage: 0.0,
            power_supply_status: 0,
            power_supply_health: 0,
            power_supply_technology: 0,
            present: false,
            cell_voltage: ptr::null(),
            cell_voltage_len: 0,
            cell_temperature: ptr::null(),
            cell_temperature_len: 0,
            location: String::new(),
            serial_number: String::new(),
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_free(b: *mut ros_battery_state_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_stamp(
    b: *mut ros_battery_state_builder_t,
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
pub extern "C" fn ros_battery_state_builder_set_frame_id(
    b: *mut ros_battery_state_builder_t,
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
pub extern "C" fn ros_battery_state_builder_set_voltage(
    b: *mut ros_battery_state_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.voltage = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_temperature(
    b: *mut ros_battery_state_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.temperature = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_current(
    b: *mut ros_battery_state_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.current = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_charge(
    b: *mut ros_battery_state_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.charge = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_capacity(
    b: *mut ros_battery_state_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.capacity = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_design_capacity(
    b: *mut ros_battery_state_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.design_capacity = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_percentage(
    b: *mut ros_battery_state_builder_t,
    v: f32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.percentage = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_power_supply_status(
    b: *mut ros_battery_state_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.power_supply_status = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_power_supply_health(
    b: *mut ros_battery_state_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.power_supply_health = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_power_supply_technology(
    b: *mut ros_battery_state_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.power_supply_technology = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_present(
    b: *mut ros_battery_state_builder_t,
    v: bool,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.present = v;
    }
}

/// Set the cell_voltage sequence (BORROWED `*const f32` — pointer must
/// remain valid until the next setter on this slot, a subsequent
/// build/encode_into, or free).
#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_cell_voltage(
    b: *mut ros_battery_state_builder_t,
    data: *const f32,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.cell_voltage = data;
        (*b).0.cell_voltage_len = len;
    }
    0
}

/// Set the cell_temperature sequence (BORROWED `*const f32` — pointer must
/// remain valid until the next setter on this slot, a subsequent
/// build/encode_into, or free).
#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_cell_temperature(
    b: *mut ros_battery_state_builder_t,
    data: *const f32,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.cell_temperature = data;
        (*b).0.cell_temperature_len = len;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_location(
    b: *mut ros_battery_state_builder_t,
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
        (*b).0.location = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_set_serial_number(
    b: *mut ros_battery_state_builder_t,
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
        (*b).0.serial_number = s_str.to_string();
    }
    0
}

fn ros_battery_state_cell_voltage_slice(inner: &BatteryStateBuilderOwned) -> &[f32] {
    if inner.cell_voltage.is_null() || inner.cell_voltage_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.cell_voltage, inner.cell_voltage_len) }
    }
}

fn ros_battery_state_cell_temperature_slice(inner: &BatteryStateBuilderOwned) -> &[f32] {
    if inner.cell_temperature.is_null() || inner.cell_temperature_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.cell_temperature, inner.cell_temperature_len) }
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_build(
    b: *mut ros_battery_state_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let cv = ros_battery_state_cell_voltage_slice(inner);
    let ct = ros_battery_state_cell_temperature_slice(inner);
    let r = sensor_msgs::BatteryState::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .voltage(inner.voltage)
        .temperature(inner.temperature)
        .current(inner.current)
        .charge(inner.charge)
        .capacity(inner.capacity)
        .design_capacity(inner.design_capacity)
        .percentage(inner.percentage)
        .power_supply_status(inner.power_supply_status)
        .power_supply_health(inner.power_supply_health)
        .power_supply_technology(inner.power_supply_technology)
        .present(inner.present)
        .cell_voltage(cv)
        .cell_temperature(ct)
        .location(inner.location.as_str())
        .serial_number(inner.serial_number.as_str())
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_battery_state_builder_encode_into(
    b: *mut ros_battery_state_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let cv = ros_battery_state_cell_voltage_slice(inner);
    let ct = ros_battery_state_cell_temperature_slice(inner);
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    let r = sensor_msgs::BatteryState::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .voltage(inner.voltage)
        .temperature(inner.temperature)
        .current(inner.current)
        .charge(inner.charge)
        .capacity(inner.capacity)
        .design_capacity(inner.design_capacity)
        .percentage(inner.percentage)
        .power_supply_status(inner.power_supply_status)
        .power_supply_health(inner.power_supply_health)
        .power_supply_technology(inner.power_supply_technology)
        .present(inner.present)
        .cell_voltage(cv)
        .cell_temperature(ct)
        .location(inner.location.as_str())
        .serial_number(inner.serial_number.as_str())
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── sensor_msgs::Temperature ────────────────────────────────────────

struct TemperatureBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    temperature: f64,
    variance: f64,
}

pub struct ros_temperature_builder_t(TemperatureBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_temperature_builder_new() -> *mut ros_temperature_builder_t {
    Box::into_raw(Box::new(ros_temperature_builder_t(
        TemperatureBuilderOwned {
            stamp_sec: 0,
            stamp_nanosec: 0,
            frame_id: String::new(),
            temperature: 0.0,
            variance: 0.0,
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_temperature_builder_free(b: *mut ros_temperature_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_temperature_builder_set_stamp(
    b: *mut ros_temperature_builder_t,
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
pub extern "C" fn ros_temperature_builder_set_frame_id(
    b: *mut ros_temperature_builder_t,
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
pub extern "C" fn ros_temperature_builder_set_temperature(
    b: *mut ros_temperature_builder_t,
    v: f64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.temperature = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_temperature_builder_set_variance(b: *mut ros_temperature_builder_t, v: f64) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.variance = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_temperature_builder_build(
    b: *mut ros_temperature_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = sensor_msgs::Temperature::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .temperature(inner.temperature)
        .variance(inner.variance)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_temperature_builder_encode_into(
    b: *mut ros_temperature_builder_t,
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
    let r = sensor_msgs::Temperature::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .temperature(inner.temperature)
        .variance(inner.variance)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── edgefirst_msgs::Mask ────────────────────────────────────────────

struct MaskBuilderOwned {
    height: u32,
    width: u32,
    length: u32,
    encoding: String,
    mask: *const u8,
    mask_len: usize,
    boxed: bool,
}

pub struct ros_mask_builder_t(MaskBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_mask_builder_new() -> *mut ros_mask_builder_t {
    Box::into_raw(Box::new(ros_mask_builder_t(MaskBuilderOwned {
        height: 0,
        width: 0,
        length: 0,
        encoding: String::new(),
        mask: ptr::null(),
        mask_len: 0,
        boxed: false,
    })))
}

#[no_mangle]
pub extern "C" fn ros_mask_builder_free(b: *mut ros_mask_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_mask_builder_set_height(b: *mut ros_mask_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.height = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_mask_builder_set_width(b: *mut ros_mask_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.width = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_mask_builder_set_length(b: *mut ros_mask_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.length = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_mask_builder_set_encoding(
    b: *mut ros_mask_builder_t,
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
        (*b).0.encoding = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_mask_builder_set_mask(
    b: *mut ros_mask_builder_t,
    data: *const u8,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.mask = data;
        (*b).0.mask_len = len;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_mask_builder_set_boxed(b: *mut ros_mask_builder_t, v: bool) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.boxed = v;
    }
}

fn ros_mask_builder_mask_slice(inner: &MaskBuilderOwned) -> &[u8] {
    if inner.mask.is_null() || inner.mask_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.mask, inner.mask_len) }
    }
}

#[no_mangle]
pub extern "C" fn ros_mask_builder_build(
    b: *mut ros_mask_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let mask_slice = ros_mask_builder_mask_slice(inner);
    let r = edgefirst_msgs::Mask::builder()
        .height(inner.height)
        .width(inner.width)
        .length(inner.length)
        .encoding(inner.encoding.as_str())
        .mask(mask_slice)
        .boxed(inner.boxed)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_mask_builder_encode_into(
    b: *mut ros_mask_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let mask_slice = ros_mask_builder_mask_slice(inner);
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    let r = edgefirst_msgs::Mask::builder()
        .height(inner.height)
        .width(inner.width)
        .length(inner.length)
        .encoding(inner.encoding.as_str())
        .mask(mask_slice)
        .boxed(inner.boxed)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── edgefirst_msgs::LocalTime ───────────────────────────────────────

struct LocalTimeBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    date_year: u16,
    date_month: u8,
    date_day: u8,
    time_sec: i32,
    time_nanosec: u32,
    timezone: i16,
}

pub struct ros_local_time_builder_t(LocalTimeBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_local_time_builder_new() -> *mut ros_local_time_builder_t {
    Box::into_raw(Box::new(ros_local_time_builder_t(LocalTimeBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        date_year: 0,
        date_month: 0,
        date_day: 0,
        time_sec: 0,
        time_nanosec: 0,
        timezone: 0,
    })))
}

#[no_mangle]
pub extern "C" fn ros_local_time_builder_free(b: *mut ros_local_time_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_local_time_builder_set_stamp(
    b: *mut ros_local_time_builder_t,
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
pub extern "C" fn ros_local_time_builder_set_frame_id(
    b: *mut ros_local_time_builder_t,
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
pub extern "C" fn ros_local_time_builder_set_date(
    b: *mut ros_local_time_builder_t,
    year: u16,
    month: u8,
    day: u8,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.date_year = year;
    inner.date_month = month;
    inner.date_day = day;
}

#[no_mangle]
pub extern "C" fn ros_local_time_builder_set_time(
    b: *mut ros_local_time_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.time_sec = sec;
    inner.time_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_local_time_builder_set_timezone(b: *mut ros_local_time_builder_t, v: i16) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.timezone = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_local_time_builder_build(
    b: *mut ros_local_time_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = edgefirst_msgs::LocalTime::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .date(edgefirst_msgs::Date {
            year: inner.date_year,
            month: inner.date_month,
            day: inner.date_day,
        })
        .time(Time::new(inner.time_sec, inner.time_nanosec))
        .timezone(inner.timezone)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_local_time_builder_encode_into(
    b: *mut ros_local_time_builder_t,
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
    let r = edgefirst_msgs::LocalTime::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .date(edgefirst_msgs::Date {
            year: inner.date_year,
            month: inner.date_month,
            day: inner.date_day,
        })
        .time(Time::new(inner.time_sec, inner.time_nanosec))
        .timezone(inner.timezone)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── edgefirst_msgs::RadarCube ───────────────────────────────────────

struct RadarCubeBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    timestamp: u64,
    layout: *const u8,
    layout_len: usize,
    shape: *const u16,
    shape_len: usize,
    scales: *const f32,
    scales_len: usize,
    cube: *const i16,
    cube_len: usize,
    is_complex: bool,
}

pub struct ros_radar_cube_builder_t(RadarCubeBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_radar_cube_builder_new() -> *mut ros_radar_cube_builder_t {
    Box::into_raw(Box::new(ros_radar_cube_builder_t(RadarCubeBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        timestamp: 0,
        layout: ptr::null(),
        layout_len: 0,
        shape: ptr::null(),
        shape_len: 0,
        scales: ptr::null(),
        scales_len: 0,
        cube: ptr::null(),
        cube_len: 0,
        is_complex: false,
    })))
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_builder_free(b: *mut ros_radar_cube_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_builder_set_stamp(
    b: *mut ros_radar_cube_builder_t,
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
pub extern "C" fn ros_radar_cube_builder_set_frame_id(
    b: *mut ros_radar_cube_builder_t,
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
pub extern "C" fn ros_radar_cube_builder_set_timestamp(b: *mut ros_radar_cube_builder_t, v: u64) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.timestamp = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_builder_set_layout(
    b: *mut ros_radar_cube_builder_t,
    data: *const u8,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.layout = data;
        (*b).0.layout_len = len;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_builder_set_shape(
    b: *mut ros_radar_cube_builder_t,
    data: *const u16,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.shape = data;
        (*b).0.shape_len = len;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_builder_set_scales(
    b: *mut ros_radar_cube_builder_t,
    data: *const f32,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.scales = data;
        (*b).0.scales_len = len;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_builder_set_cube(
    b: *mut ros_radar_cube_builder_t,
    data: *const i16,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.cube = data;
        (*b).0.cube_len = len;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_builder_set_is_complex(b: *mut ros_radar_cube_builder_t, v: bool) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.is_complex = v;
    }
}

fn radar_cube_layout_slice(inner: &RadarCubeBuilderOwned) -> &[u8] {
    if inner.layout.is_null() || inner.layout_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.layout, inner.layout_len) }
    }
}
fn radar_cube_shape_slice(inner: &RadarCubeBuilderOwned) -> &[u16] {
    if inner.shape.is_null() || inner.shape_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.shape, inner.shape_len) }
    }
}
fn radar_cube_scales_slice(inner: &RadarCubeBuilderOwned) -> &[f32] {
    if inner.scales.is_null() || inner.scales_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.scales, inner.scales_len) }
    }
}
fn radar_cube_cube_slice(inner: &RadarCubeBuilderOwned) -> &[i16] {
    if inner.cube.is_null() || inner.cube_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.cube, inner.cube_len) }
    }
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_builder_build(
    b: *mut ros_radar_cube_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = edgefirst_msgs::RadarCube::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .timestamp(inner.timestamp)
        .layout(radar_cube_layout_slice(inner))
        .shape(radar_cube_shape_slice(inner))
        .scales(radar_cube_scales_slice(inner))
        .cube(radar_cube_cube_slice(inner))
        .is_complex(inner.is_complex)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_radar_cube_builder_encode_into(
    b: *mut ros_radar_cube_builder_t,
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
    let r = edgefirst_msgs::RadarCube::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .timestamp(inner.timestamp)
        .layout(radar_cube_layout_slice(inner))
        .shape(radar_cube_shape_slice(inner))
        .scales(radar_cube_scales_slice(inner))
        .cube(radar_cube_cube_slice(inner))
        .is_complex(inner.is_complex)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── edgefirst_msgs::RadarInfo ───────────────────────────────────────

struct RadarInfoBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    center_frequency: String,
    frequency_sweep: String,
    range_toggle: String,
    detection_sensitivity: String,
    cube: bool,
}

pub struct ros_radar_info_builder_t(RadarInfoBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_radar_info_builder_new() -> *mut ros_radar_info_builder_t {
    Box::into_raw(Box::new(ros_radar_info_builder_t(RadarInfoBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        center_frequency: String::new(),
        frequency_sweep: String::new(),
        range_toggle: String::new(),
        detection_sensitivity: String::new(),
        cube: false,
    })))
}

#[no_mangle]
pub extern "C" fn ros_radar_info_builder_free(b: *mut ros_radar_info_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_radar_info_builder_set_stamp(
    b: *mut ros_radar_info_builder_t,
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
pub extern "C" fn ros_radar_info_builder_set_frame_id(
    b: *mut ros_radar_info_builder_t,
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
pub extern "C" fn ros_radar_info_builder_set_center_frequency(
    b: *mut ros_radar_info_builder_t,
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
        (*b).0.center_frequency = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_radar_info_builder_set_frequency_sweep(
    b: *mut ros_radar_info_builder_t,
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
        (*b).0.frequency_sweep = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_radar_info_builder_set_range_toggle(
    b: *mut ros_radar_info_builder_t,
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
        (*b).0.range_toggle = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_radar_info_builder_set_detection_sensitivity(
    b: *mut ros_radar_info_builder_t,
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
        (*b).0.detection_sensitivity = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_radar_info_builder_set_cube(b: *mut ros_radar_info_builder_t, v: bool) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.cube = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_radar_info_builder_build(
    b: *mut ros_radar_info_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = edgefirst_msgs::RadarInfo::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .center_frequency(inner.center_frequency.as_str())
        .frequency_sweep(inner.frequency_sweep.as_str())
        .range_toggle(inner.range_toggle.as_str())
        .detection_sensitivity(inner.detection_sensitivity.as_str())
        .cube(inner.cube)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_radar_info_builder_encode_into(
    b: *mut ros_radar_info_builder_t,
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
    let r = edgefirst_msgs::RadarInfo::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .center_frequency(inner.center_frequency.as_str())
        .frequency_sweep(inner.frequency_sweep.as_str())
        .range_toggle(inner.range_toggle.as_str())
        .detection_sensitivity(inner.detection_sensitivity.as_str())
        .cube(inner.cube)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── edgefirst_msgs::Track ───────────────────────────────────────────

struct TrackBuilderOwned {
    id: String,
    lifetime: i32,
    created_sec: i32,
    created_nanosec: u32,
}

pub struct ros_track_builder_t(TrackBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_track_builder_new() -> *mut ros_track_builder_t {
    Box::into_raw(Box::new(ros_track_builder_t(TrackBuilderOwned {
        id: String::new(),
        lifetime: 0,
        created_sec: 0,
        created_nanosec: 0,
    })))
}

#[no_mangle]
pub extern "C" fn ros_track_builder_free(b: *mut ros_track_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_track_builder_set_id(b: *mut ros_track_builder_t, s: *const c_char) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let s_str = match unsafe { c_to_str_checked(s) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    unsafe {
        (*b).0.id = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_track_builder_set_lifetime(b: *mut ros_track_builder_t, v: i32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.lifetime = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_track_builder_set_created(
    b: *mut ros_track_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.created_sec = sec;
    inner.created_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_track_builder_build(
    b: *mut ros_track_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = edgefirst_msgs::Track::builder()
        .id(inner.id.as_str())
        .lifetime(inner.lifetime)
        .created(Time::new(inner.created_sec, inner.created_nanosec))
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_track_builder_encode_into(
    b: *mut ros_track_builder_t,
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
    let r = edgefirst_msgs::Track::builder()
        .id(inner.id.as_str())
        .lifetime(inner.lifetime)
        .created(Time::new(inner.created_sec, inner.created_nanosec))
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── edgefirst_msgs::DetectBox / Detect / Model shared element ───────
//
// `ros_detect_box_elem_t` is the C-POD descriptor used both as the standalone
// DetectBox builder's input shape and as the element type for the `boxes`
// nested sequence in Detect and Model. The `label` and `track_id` pointers
// are borrowed: they must outlive the next setter call on the field that
// borrows them, the next build/encode_into, or builder_free.

/// C-POD descriptor for a single DetectBox element. `label` and `track_id`
/// are borrowed C strings; both must remain valid until the consuming
/// builder is finalised (build/encode_into) or freed.
#[repr(C)]
pub struct ros_detect_box_elem_t {
    pub center_x: f32,
    pub center_y: f32,
    pub width: f32,
    pub height: f32,
    pub label: *const c_char,
    pub score: f32,
    pub distance: f32,
    pub speed: f32,
    pub track_id: *const c_char,
    pub track_lifetime: i32,
    pub track_created_sec: i32,
    pub track_created_nanosec: u32,
}

/// Materialise `DetectBoxView` borrowers from the C-POD descriptor array.
///
/// # Safety
/// Each descriptor's `label` / `track_id` must be a valid NUL-terminated C
/// string (or NULL, treated as "") whose backing storage outlives the
/// returned Vec.
unsafe fn detect_box_descs_to_views(
    descs: *const ros_detect_box_elem_t,
    count: usize,
) -> Result<Vec<edgefirst_msgs::DetectBoxView<'static>>, ()> {
    if descs.is_null() || count == 0 {
        return Ok(Vec::new());
    }
    let descs = slice::from_raw_parts(descs, count);
    descs
        .iter()
        .map(|d| {
            let label = c_to_str_checked(d.label)?;
            let track_id = c_to_str_checked(d.track_id)?;
            Ok(edgefirst_msgs::DetectBoxView {
                center_x: d.center_x,
                center_y: d.center_y,
                width: d.width,
                height: d.height,
                label,
                score: d.score,
                distance: d.distance,
                speed: d.speed,
                track_id,
                track_lifetime: d.track_lifetime,
                track_created: Time::new(d.track_created_sec, d.track_created_nanosec),
            })
        })
        .collect()
}

// ── edgefirst_msgs::DetectBox (standalone) ──────────────────────────

struct DetectBoxBuilderOwned {
    center_x: f32,
    center_y: f32,
    width: f32,
    height: f32,
    label: String,
    score: f32,
    distance: f32,
    speed: f32,
    track_id: String,
    track_lifetime: i32,
    track_created_sec: i32,
    track_created_nanosec: u32,
}

pub struct ros_detect_box_builder_t(DetectBoxBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_new() -> *mut ros_detect_box_builder_t {
    Box::into_raw(Box::new(ros_detect_box_builder_t(DetectBoxBuilderOwned {
        center_x: 0.0,
        center_y: 0.0,
        width: 0.0,
        height: 0.0,
        label: String::new(),
        score: 0.0,
        distance: 0.0,
        speed: 0.0,
        track_id: String::new(),
        track_lifetime: 0,
        track_created_sec: 0,
        track_created_nanosec: 0,
    })))
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_free(b: *mut ros_detect_box_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_set_center_x(b: *mut ros_detect_box_builder_t, v: f32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.center_x = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_set_center_y(b: *mut ros_detect_box_builder_t, v: f32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.center_y = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_set_width(b: *mut ros_detect_box_builder_t, v: f32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.width = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_set_height(b: *mut ros_detect_box_builder_t, v: f32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.height = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_set_label(
    b: *mut ros_detect_box_builder_t,
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
        (*b).0.label = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_set_score(b: *mut ros_detect_box_builder_t, v: f32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.score = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_set_distance(b: *mut ros_detect_box_builder_t, v: f32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.distance = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_set_speed(b: *mut ros_detect_box_builder_t, v: f32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.speed = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_set_track_id(
    b: *mut ros_detect_box_builder_t,
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
        (*b).0.track_id = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_set_track_lifetime(
    b: *mut ros_detect_box_builder_t,
    v: i32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.track_lifetime = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_set_track_created(
    b: *mut ros_detect_box_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.track_created_sec = sec;
    inner.track_created_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_build(
    b: *mut ros_detect_box_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = edgefirst_msgs::DetectBox::builder()
        .center_x(inner.center_x)
        .center_y(inner.center_y)
        .width(inner.width)
        .height(inner.height)
        .label(inner.label.as_str())
        .score(inner.score)
        .distance(inner.distance)
        .speed(inner.speed)
        .track_id(inner.track_id.as_str())
        .track_lifetime(inner.track_lifetime)
        .track_created(Time::new(
            inner.track_created_sec,
            inner.track_created_nanosec,
        ))
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_box_builder_encode_into(
    b: *mut ros_detect_box_builder_t,
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
    let r = edgefirst_msgs::DetectBox::builder()
        .center_x(inner.center_x)
        .center_y(inner.center_y)
        .width(inner.width)
        .height(inner.height)
        .label(inner.label.as_str())
        .score(inner.score)
        .distance(inner.distance)
        .speed(inner.speed)
        .track_id(inner.track_id.as_str())
        .track_lifetime(inner.track_lifetime)
        .track_created(Time::new(
            inner.track_created_sec,
            inner.track_created_nanosec,
        ))
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── edgefirst_msgs::Detect ──────────────────────────────────────────

struct DetectBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    input_sec: i32,
    input_nanosec: u32,
    model_sec: i32,
    model_nanosec: u32,
    output_sec: i32,
    output_nanosec: u32,
    boxes: *const ros_detect_box_elem_t,
    boxes_count: usize,
}

pub struct ros_detect_builder_t(DetectBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_detect_builder_new() -> *mut ros_detect_builder_t {
    Box::into_raw(Box::new(ros_detect_builder_t(DetectBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        input_sec: 0,
        input_nanosec: 0,
        model_sec: 0,
        model_nanosec: 0,
        output_sec: 0,
        output_nanosec: 0,
        boxes: ptr::null(),
        boxes_count: 0,
    })))
}

#[no_mangle]
pub extern "C" fn ros_detect_builder_free(b: *mut ros_detect_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_builder_set_stamp(
    b: *mut ros_detect_builder_t,
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
pub extern "C" fn ros_detect_builder_set_frame_id(
    b: *mut ros_detect_builder_t,
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
pub extern "C" fn ros_detect_builder_set_input_timestamp(
    b: *mut ros_detect_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.input_sec = sec;
    inner.input_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_detect_builder_set_model_time(
    b: *mut ros_detect_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.model_sec = sec;
    inner.model_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_detect_builder_set_output_time(
    b: *mut ros_detect_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.output_sec = sec;
    inner.output_nanosec = nanosec;
}

/// Set the boxes descriptor sequence (BORROWED — `boxes` and every
/// `label`/`track_id` pointer inside it must remain valid until the next
/// setter on the boxes slot, a subsequent build/encode_into, or free).
#[no_mangle]
pub extern "C" fn ros_detect_builder_set_boxes(
    b: *mut ros_detect_builder_t,
    boxes: *const ros_detect_box_elem_t,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if boxes.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.boxes = boxes;
        (*b).0.boxes_count = count;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_detect_builder_build(
    b: *mut ros_detect_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let boxes = match unsafe { detect_box_descs_to_views(inner.boxes, inner.boxes_count) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    let r = edgefirst_msgs::Detect::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .input_timestamp(Time::new(inner.input_sec, inner.input_nanosec))
        .model_time(Time::new(inner.model_sec, inner.model_nanosec))
        .output_time(Time::new(inner.output_sec, inner.output_nanosec))
        .boxes(&boxes)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_detect_builder_encode_into(
    b: *mut ros_detect_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let boxes = match unsafe { detect_box_descs_to_views(inner.boxes, inner.boxes_count) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    let r = edgefirst_msgs::Detect::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .input_timestamp(Time::new(inner.input_sec, inner.input_nanosec))
        .model_time(Time::new(inner.model_sec, inner.model_nanosec))
        .output_time(Time::new(inner.output_sec, inner.output_nanosec))
        .boxes(&boxes)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── edgefirst_msgs::CameraFrame ─────────────────────────────────────

/// C-POD descriptor for a single CameraPlane element. `data` is a borrowed
/// byte slice; it must remain valid until the consuming builder is
/// finalised (build/encode_into) or freed.
#[repr(C)]
pub struct ros_camera_plane_elem_t {
    pub fd: i32,
    pub offset: u32,
    pub stride: u32,
    pub size: u32,
    pub used: u32,
    pub data: *const u8,
    pub data_len: usize,
}

/// Materialise `CameraPlaneView` borrowers from the C-POD descriptor array.
///
/// # Safety
/// Each descriptor's `data` pointer must be valid for `data_len` bytes (or
/// NULL when `data_len == 0`); the backing storage must outlive the returned Vec.
unsafe fn camera_plane_descs_to_views(
    descs: *const ros_camera_plane_elem_t,
    count: usize,
) -> Vec<edgefirst_msgs::CameraPlaneView<'static>> {
    if descs.is_null() || count == 0 {
        return Vec::new();
    }
    let descs = slice::from_raw_parts(descs, count);
    descs
        .iter()
        .map(|d| edgefirst_msgs::CameraPlaneView {
            fd: d.fd,
            offset: d.offset,
            stride: d.stride,
            size: d.size,
            used: d.used,
            data: if d.data.is_null() || d.data_len == 0 {
                &[][..]
            } else {
                slice::from_raw_parts(d.data, d.data_len)
            },
        })
        .collect()
}

struct CameraFrameBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    seq: u64,
    pid: u32,
    width: u32,
    height: u32,
    format: String,
    color_space: String,
    color_transfer: String,
    color_encoding: String,
    color_range: String,
    fence_fd: i32,
    planes: *const ros_camera_plane_elem_t,
    planes_count: usize,
}

pub struct ros_camera_frame_builder_t(CameraFrameBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_new() -> *mut ros_camera_frame_builder_t {
    Box::into_raw(Box::new(ros_camera_frame_builder_t(
        CameraFrameBuilderOwned {
            stamp_sec: 0,
            stamp_nanosec: 0,
            frame_id: String::new(),
            seq: 0,
            pid: 0,
            width: 0,
            height: 0,
            format: String::new(),
            color_space: String::new(),
            color_transfer: String::new(),
            color_encoding: String::new(),
            color_range: String::new(),
            fence_fd: -1,
            planes: ptr::null(),
            planes_count: 0,
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_free(b: *mut ros_camera_frame_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_set_stamp(
    b: *mut ros_camera_frame_builder_t,
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
pub extern "C" fn ros_camera_frame_builder_set_frame_id(
    b: *mut ros_camera_frame_builder_t,
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
pub extern "C" fn ros_camera_frame_builder_set_seq(b: *mut ros_camera_frame_builder_t, v: u64) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.seq = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_set_pid(b: *mut ros_camera_frame_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.pid = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_set_width(b: *mut ros_camera_frame_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.width = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_set_height(b: *mut ros_camera_frame_builder_t, v: u32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.height = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_set_format(
    b: *mut ros_camera_frame_builder_t,
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
        (*b).0.format = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_set_color_space(
    b: *mut ros_camera_frame_builder_t,
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
        (*b).0.color_space = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_set_color_transfer(
    b: *mut ros_camera_frame_builder_t,
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
        (*b).0.color_transfer = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_set_color_encoding(
    b: *mut ros_camera_frame_builder_t,
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
        (*b).0.color_encoding = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_set_color_range(
    b: *mut ros_camera_frame_builder_t,
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
        (*b).0.color_range = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_set_fence_fd(
    b: *mut ros_camera_frame_builder_t,
    v: i32,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.fence_fd = v;
    }
}

/// Set the planes descriptor sequence (BORROWED — `planes` and every
/// `data` pointer inside it must remain valid until the next setter on
/// the planes slot, a subsequent build/encode_into, or free).
#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_set_planes(
    b: *mut ros_camera_frame_builder_t,
    planes: *const ros_camera_plane_elem_t,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if planes.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.planes = planes;
        (*b).0.planes_count = count;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_build(
    b: *mut ros_camera_frame_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let planes = unsafe { camera_plane_descs_to_views(inner.planes, inner.planes_count) };
    let r = edgefirst_msgs::CameraFrame::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .seq(inner.seq)
        .pid(inner.pid)
        .width(inner.width)
        .height(inner.height)
        .format(inner.format.as_str())
        .color_space(inner.color_space.as_str())
        .color_transfer(inner.color_transfer.as_str())
        .color_encoding(inner.color_encoding.as_str())
        .color_range(inner.color_range.as_str())
        .fence_fd(inner.fence_fd)
        .planes(&planes)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_camera_frame_builder_encode_into(
    b: *mut ros_camera_frame_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let planes = unsafe { camera_plane_descs_to_views(inner.planes, inner.planes_count) };
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    let r = edgefirst_msgs::CameraFrame::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .seq(inner.seq)
        .pid(inner.pid)
        .width(inner.width)
        .height(inner.height)
        .format(inner.format.as_str())
        .color_space(inner.color_space.as_str())
        .color_transfer(inner.color_transfer.as_str())
        .color_encoding(inner.color_encoding.as_str())
        .color_range(inner.color_range.as_str())
        .fence_fd(inner.fence_fd)
        .planes(&planes)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── edgefirst_msgs::Model ───────────────────────────────────────────
//
// Shares `ros_detect_box_elem_t` for boxes; uses `ros_mask_elem_t` for masks.
// The mask descriptor mirrors `MaskView` exactly.

/// C-POD descriptor for a single Mask element. `encoding` is a borrowed C
/// string and `mask` is a borrowed byte slice; both must remain valid until
/// the consuming builder is finalised (build/encode_into) or freed.
#[repr(C)]
pub struct ros_mask_elem_t {
    pub height: u32,
    pub width: u32,
    pub length: u32,
    pub encoding: *const c_char,
    pub mask: *const u8,
    pub mask_len: usize,
    pub boxed: bool,
}

/// Materialise `MaskView` borrowers from the C-POD descriptor array.
///
/// # Safety
/// Each descriptor's `encoding` must be a valid NUL-terminated C string (or
/// NULL → "") and `mask` must be valid for `mask_len` bytes (or NULL when
/// `mask_len == 0`); backing storage outlives the returned Vec.
unsafe fn mask_descs_to_views(
    descs: *const ros_mask_elem_t,
    count: usize,
) -> Result<Vec<edgefirst_msgs::MaskView<'static>>, ()> {
    if descs.is_null() || count == 0 {
        return Ok(Vec::new());
    }
    let descs = slice::from_raw_parts(descs, count);
    descs
        .iter()
        .map(|d| {
            let encoding = c_to_str_checked(d.encoding)?;
            let mask = if d.mask.is_null() {
                if d.mask_len > 0 {
                    set_errno(EINVAL);
                    return Err(());
                }
                &[][..]
            } else {
                slice::from_raw_parts(d.mask, d.mask_len)
            };
            Ok(edgefirst_msgs::MaskView {
                height: d.height,
                width: d.width,
                length: d.length,
                encoding,
                mask,
                boxed: d.boxed,
            })
        })
        .collect()
}

struct ModelBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    input_sec: i32,
    input_nanosec: u32,
    model_sec: i32,
    model_nanosec: u32,
    output_sec: i32,
    output_nanosec: u32,
    decode_sec: i32,
    decode_nanosec: u32,
    boxes: *const ros_detect_box_elem_t,
    boxes_count: usize,
    masks: *const ros_mask_elem_t,
    masks_count: usize,
}

pub struct ros_model_builder_t(ModelBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_model_builder_new() -> *mut ros_model_builder_t {
    Box::into_raw(Box::new(ros_model_builder_t(ModelBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        input_sec: 0,
        input_nanosec: 0,
        model_sec: 0,
        model_nanosec: 0,
        output_sec: 0,
        output_nanosec: 0,
        decode_sec: 0,
        decode_nanosec: 0,
        boxes: ptr::null(),
        boxes_count: 0,
        masks: ptr::null(),
        masks_count: 0,
    })))
}

#[no_mangle]
pub extern "C" fn ros_model_builder_free(b: *mut ros_model_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_model_builder_set_stamp(b: *mut ros_model_builder_t, sec: i32, nanosec: u32) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.stamp_sec = sec;
    inner.stamp_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_model_builder_set_frame_id(
    b: *mut ros_model_builder_t,
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
pub extern "C" fn ros_model_builder_set_input_time(
    b: *mut ros_model_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.input_sec = sec;
    inner.input_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_model_builder_set_model_time(
    b: *mut ros_model_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.model_sec = sec;
    inner.model_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_model_builder_set_output_time(
    b: *mut ros_model_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.output_sec = sec;
    inner.output_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_model_builder_set_decode_time(
    b: *mut ros_model_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.decode_sec = sec;
    inner.decode_nanosec = nanosec;
}

/// Set the boxes descriptor sequence (BORROWED — see
/// `ros_detect_builder_set_boxes`).
#[no_mangle]
pub extern "C" fn ros_model_builder_set_boxes(
    b: *mut ros_model_builder_t,
    boxes: *const ros_detect_box_elem_t,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if boxes.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.boxes = boxes;
        (*b).0.boxes_count = count;
    }
    0
}

/// Set the masks descriptor sequence (BORROWED — `masks` and every
/// `encoding`/`mask` pointer inside it must remain valid until the next
/// setter on the masks slot, a subsequent build/encode_into, or free).
#[no_mangle]
pub extern "C" fn ros_model_builder_set_masks(
    b: *mut ros_model_builder_t,
    masks: *const ros_mask_elem_t,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if masks.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.masks = masks;
        (*b).0.masks_count = count;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_model_builder_build(
    b: *mut ros_model_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let boxes = match unsafe { detect_box_descs_to_views(inner.boxes, inner.boxes_count) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    let masks = match unsafe { mask_descs_to_views(inner.masks, inner.masks_count) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    let r = edgefirst_msgs::Model::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .input_time(Duration {
            sec: inner.input_sec,
            nanosec: inner.input_nanosec,
        })
        .model_time(Duration {
            sec: inner.model_sec,
            nanosec: inner.model_nanosec,
        })
        .output_time(Duration {
            sec: inner.output_sec,
            nanosec: inner.output_nanosec,
        })
        .decode_time(Duration {
            sec: inner.decode_sec,
            nanosec: inner.decode_nanosec,
        })
        .boxes(&boxes)
        .masks(&masks)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_model_builder_encode_into(
    b: *mut ros_model_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let boxes = match unsafe { detect_box_descs_to_views(inner.boxes, inner.boxes_count) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    let masks = match unsafe { mask_descs_to_views(inner.masks, inner.masks_count) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    let r = edgefirst_msgs::Model::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .input_time(Duration {
            sec: inner.input_sec,
            nanosec: inner.input_nanosec,
        })
        .model_time(Duration {
            sec: inner.model_sec,
            nanosec: inner.model_nanosec,
        })
        .output_time(Duration {
            sec: inner.output_sec,
            nanosec: inner.output_nanosec,
        })
        .decode_time(Duration {
            sec: inner.decode_sec,
            nanosec: inner.decode_nanosec,
        })
        .boxes(&boxes)
        .masks(&masks)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── edgefirst_msgs::ModelInfo ───────────────────────────────────────

struct ModelInfoBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    input_shape: *const u32,
    input_shape_len: usize,
    input_type: u8,
    output_shape: *const u32,
    output_shape_len: usize,
    output_type: u8,
    // Labels are owned: each call to set_labels copies the C strings into a
    // Vec<String> so the borrow contract is "labels valid for the duration
    // of one set_labels call". Not borrowed across the FFI boundary.
    labels: Vec<String>,
    model_type: String,
    model_format: String,
    model_name: String,
}

pub struct ros_model_info_builder_t(ModelInfoBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_model_info_builder_new() -> *mut ros_model_info_builder_t {
    Box::into_raw(Box::new(ros_model_info_builder_t(ModelInfoBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        input_shape: ptr::null(),
        input_shape_len: 0,
        input_type: 0,
        output_shape: ptr::null(),
        output_shape_len: 0,
        output_type: 0,
        labels: Vec::new(),
        model_type: String::new(),
        model_format: String::new(),
        model_name: String::new(),
    })))
}

#[no_mangle]
pub extern "C" fn ros_model_info_builder_free(b: *mut ros_model_info_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_model_info_builder_set_stamp(
    b: *mut ros_model_info_builder_t,
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
pub extern "C" fn ros_model_info_builder_set_frame_id(
    b: *mut ros_model_info_builder_t,
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
pub extern "C" fn ros_model_info_builder_set_input_shape(
    b: *mut ros_model_info_builder_t,
    data: *const u32,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.input_shape = data;
        (*b).0.input_shape_len = len;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_model_info_builder_set_input_type(b: *mut ros_model_info_builder_t, v: u8) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.input_type = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_model_info_builder_set_output_shape(
    b: *mut ros_model_info_builder_t,
    data: *const u32,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.output_shape = data;
        (*b).0.output_shape_len = len;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_model_info_builder_set_output_type(b: *mut ros_model_info_builder_t, v: u8) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.output_type = v;
    }
}

/// Set labels by copying each C string into builder-owned storage. Returns 0
/// on success, -1 on error (errno: EINVAL for NULL handle or NULL element
/// pointer when count > 0).
#[no_mangle]
pub extern "C" fn ros_model_info_builder_set_labels(
    b: *mut ros_model_info_builder_t,
    labels: *const *const c_char,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if count == 0 {
        unsafe {
            (*b).0.labels.clear();
        }
        return 0;
    }
    if labels.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slc = unsafe { slice::from_raw_parts(labels, count) };
    let mut out: Vec<String> = Vec::with_capacity(count);
    for &p in slc {
        out.push(unsafe { c_to_str(p) }.to_string());
    }
    unsafe {
        (*b).0.labels = out;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_model_info_builder_set_model_type(
    b: *mut ros_model_info_builder_t,
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
        (*b).0.model_type = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_model_info_builder_set_model_format(
    b: *mut ros_model_info_builder_t,
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
        (*b).0.model_format = s_str.to_string();
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_model_info_builder_set_model_name(
    b: *mut ros_model_info_builder_t,
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
        (*b).0.model_name = s_str.to_string();
    }
    0
}

fn model_info_input_shape(inner: &ModelInfoBuilderOwned) -> &[u32] {
    if inner.input_shape.is_null() || inner.input_shape_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.input_shape, inner.input_shape_len) }
    }
}
fn model_info_output_shape(inner: &ModelInfoBuilderOwned) -> &[u32] {
    if inner.output_shape.is_null() || inner.output_shape_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.output_shape, inner.output_shape_len) }
    }
}

#[no_mangle]
pub extern "C" fn ros_model_info_builder_build(
    b: *mut ros_model_info_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let label_refs: Vec<&str> = inner.labels.iter().map(String::as_str).collect();
    let r = edgefirst_msgs::ModelInfo::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .input_shape(model_info_input_shape(inner))
        .input_type(inner.input_type)
        .output_shape(model_info_output_shape(inner))
        .output_type(inner.output_type)
        .labels(&label_refs)
        .model_type(inner.model_type.as_str())
        .model_format(inner.model_format.as_str())
        .model_name(inner.model_name.as_str())
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_model_info_builder_encode_into(
    b: *mut ros_model_info_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let label_refs: Vec<&str> = inner.labels.iter().map(String::as_str).collect();
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    let r = edgefirst_msgs::ModelInfo::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .input_shape(model_info_input_shape(inner))
        .input_type(inner.input_type)
        .output_shape(model_info_output_shape(inner))
        .output_type(inner.output_type)
        .labels(&label_refs)
        .model_type(inner.model_type.as_str())
        .model_format(inner.model_format.as_str())
        .model_name(inner.model_name.as_str())
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── edgefirst_msgs::Vibration ───────────────────────────────────────

struct VibrationBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    vib_x: f64,
    vib_y: f64,
    vib_z: f64,
    band_lower_hz: f32,
    band_upper_hz: f32,
    measurement_type: u8,
    unit: u8,
    clipping: *const u32,
    clipping_len: usize,
}

pub struct ros_vibration_builder_t(VibrationBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_vibration_builder_new() -> *mut ros_vibration_builder_t {
    Box::into_raw(Box::new(ros_vibration_builder_t(VibrationBuilderOwned {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: String::new(),
        vib_x: 0.0,
        vib_y: 0.0,
        vib_z: 0.0,
        band_lower_hz: 0.0,
        band_upper_hz: 0.0,
        measurement_type: 0,
        unit: 0,
        clipping: ptr::null(),
        clipping_len: 0,
    })))
}

#[no_mangle]
pub extern "C" fn ros_vibration_builder_free(b: *mut ros_vibration_builder_t) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_vibration_builder_set_stamp(
    b: *mut ros_vibration_builder_t,
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
pub extern "C" fn ros_vibration_builder_set_frame_id(
    b: *mut ros_vibration_builder_t,
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
pub extern "C" fn ros_vibration_builder_set_vibration(
    b: *mut ros_vibration_builder_t,
    x: f64,
    y: f64,
    z: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.vib_x = x;
    inner.vib_y = y;
    inner.vib_z = z;
}

#[no_mangle]
pub extern "C" fn ros_vibration_builder_set_band_lower_hz(b: *mut ros_vibration_builder_t, v: f32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.band_lower_hz = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_vibration_builder_set_band_upper_hz(b: *mut ros_vibration_builder_t, v: f32) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.band_upper_hz = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_vibration_builder_set_measurement_type(
    b: *mut ros_vibration_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.measurement_type = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_vibration_builder_set_unit(b: *mut ros_vibration_builder_t, v: u8) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.unit = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_vibration_builder_set_clipping(
    b: *mut ros_vibration_builder_t,
    data: *const u32,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.clipping = data;
        (*b).0.clipping_len = len;
    }
    0
}

fn vibration_clipping_slice(inner: &VibrationBuilderOwned) -> &[u32] {
    if inner.clipping.is_null() || inner.clipping_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.clipping, inner.clipping_len) }
    }
}

#[no_mangle]
pub extern "C" fn ros_vibration_builder_build(
    b: *mut ros_vibration_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = edgefirst_msgs::Vibration::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .vibration(crate::geometry_msgs::Vector3 {
            x: inner.vib_x,
            y: inner.vib_y,
            z: inner.vib_z,
        })
        .band_lower_hz(inner.band_lower_hz)
        .band_upper_hz(inner.band_upper_hz)
        .measurement_type(inner.measurement_type)
        .unit(inner.unit)
        .clipping(vibration_clipping_slice(inner))
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_vibration_builder_encode_into(
    b: *mut ros_vibration_builder_t,
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
    let r = edgefirst_msgs::Vibration::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .vibration(crate::geometry_msgs::Vector3 {
            x: inner.vib_x,
            y: inner.vib_y,
            z: inner.vib_z,
        })
        .band_lower_hz(inner.band_lower_hz)
        .band_upper_hz(inner.band_upper_hz)
        .measurement_type(inner.measurement_type)
        .unit(inner.unit)
        .clipping(vibration_clipping_slice(inner))
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── foxglove_msgs::FoxgloveCompressedVideo ──────────────────────────

struct FoxgloveCompressedVideoBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    data: *const u8,
    data_len: usize,
    format: String,
}

pub struct ros_foxglove_compressed_video_builder_t(FoxgloveCompressedVideoBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_foxglove_compressed_video_builder_new(
) -> *mut ros_foxglove_compressed_video_builder_t {
    Box::into_raw(Box::new(ros_foxglove_compressed_video_builder_t(
        FoxgloveCompressedVideoBuilderOwned {
            stamp_sec: 0,
            stamp_nanosec: 0,
            frame_id: String::new(),
            data: ptr::null(),
            data_len: 0,
            format: String::new(),
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_foxglove_compressed_video_builder_free(
    b: *mut ros_foxglove_compressed_video_builder_t,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_foxglove_compressed_video_builder_set_stamp(
    b: *mut ros_foxglove_compressed_video_builder_t,
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

/// Alias for `ros_foxglove_compressed_video_builder_set_stamp`; matches the Foxglove schema field name.
#[no_mangle]
pub unsafe extern "C" fn ros_foxglove_compressed_video_builder_set_timestamp(
    b: *mut ros_foxglove_compressed_video_builder_t,
    sec: i32,
    nanosec: u32,
) {
    ros_foxglove_compressed_video_builder_set_stamp(b, sec, nanosec)
}

#[no_mangle]
pub extern "C" fn ros_foxglove_compressed_video_builder_set_frame_id(
    b: *mut ros_foxglove_compressed_video_builder_t,
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
pub extern "C" fn ros_foxglove_compressed_video_builder_set_data(
    b: *mut ros_foxglove_compressed_video_builder_t,
    data: *const u8,
    len: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if data.is_null() && len > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.data = data;
        (*b).0.data_len = len;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_foxglove_compressed_video_builder_set_format(
    b: *mut ros_foxglove_compressed_video_builder_t,
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
        (*b).0.format = s_str.to_string();
    }
    0
}

fn foxglove_compressed_video_data_slice(inner: &FoxgloveCompressedVideoBuilderOwned) -> &[u8] {
    if inner.data.is_null() || inner.data_len == 0 {
        &[][..]
    } else {
        unsafe { slice::from_raw_parts(inner.data, inner.data_len) }
    }
}

#[no_mangle]
pub extern "C" fn ros_foxglove_compressed_video_builder_build(
    b: *mut ros_foxglove_compressed_video_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = foxglove_msgs::FoxgloveCompressedVideo::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .data(foxglove_compressed_video_data_slice(inner))
        .format(inner.format.as_str())
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_foxglove_compressed_video_builder_encode_into(
    b: *mut ros_foxglove_compressed_video_builder_t,
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
    let r = foxglove_msgs::FoxgloveCompressedVideo::builder()
        .stamp(Time::new(inner.stamp_sec, inner.stamp_nanosec))
        .frame_id(inner.frame_id.as_str())
        .data(foxglove_compressed_video_data_slice(inner))
        .format(inner.format.as_str())
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── foxglove_msgs::FoxgloveTextAnnotation ───────────────────────────

struct FoxgloveTextAnnotationBuilderOwned {
    timestamp_sec: i32,
    timestamp_nanosec: u32,
    pos_x: f64,
    pos_y: f64,
    text: String,
    font_size: f64,
    text_color_r: f64,
    text_color_g: f64,
    text_color_b: f64,
    text_color_a: f64,
    bg_color_r: f64,
    bg_color_g: f64,
    bg_color_b: f64,
    bg_color_a: f64,
}

pub struct ros_foxglove_text_annotation_builder_t(FoxgloveTextAnnotationBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_builder_new(
) -> *mut ros_foxglove_text_annotation_builder_t {
    Box::into_raw(Box::new(ros_foxglove_text_annotation_builder_t(
        FoxgloveTextAnnotationBuilderOwned {
            timestamp_sec: 0,
            timestamp_nanosec: 0,
            pos_x: 0.0,
            pos_y: 0.0,
            text: String::new(),
            font_size: 0.0,
            text_color_r: 0.0,
            text_color_g: 0.0,
            text_color_b: 0.0,
            text_color_a: 0.0,
            bg_color_r: 0.0,
            bg_color_g: 0.0,
            bg_color_b: 0.0,
            bg_color_a: 0.0,
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_builder_free(
    b: *mut ros_foxglove_text_annotation_builder_t,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_builder_set_timestamp(
    b: *mut ros_foxglove_text_annotation_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.timestamp_sec = sec;
    inner.timestamp_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_builder_set_position(
    b: *mut ros_foxglove_text_annotation_builder_t,
    x: f64,
    y: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.pos_x = x;
    inner.pos_y = y;
}

#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_builder_set_text(
    b: *mut ros_foxglove_text_annotation_builder_t,
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
pub extern "C" fn ros_foxglove_text_annotation_builder_set_font_size(
    b: *mut ros_foxglove_text_annotation_builder_t,
    v: f64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.font_size = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_builder_set_text_color(
    b: *mut ros_foxglove_text_annotation_builder_t,
    r: f64,
    g: f64,
    bc: f64,
    a: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.text_color_r = r;
    inner.text_color_g = g;
    inner.text_color_b = bc;
    inner.text_color_a = a;
}

#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_builder_set_background_color(
    b: *mut ros_foxglove_text_annotation_builder_t,
    r: f64,
    g: f64,
    bc: f64,
    a: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.bg_color_r = r;
    inner.bg_color_g = g;
    inner.bg_color_b = bc;
    inner.bg_color_a = a;
}

#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_builder_build(
    b: *mut ros_foxglove_text_annotation_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let r = foxglove_msgs::FoxgloveTextAnnotation::builder()
        .timestamp(Time::new(inner.timestamp_sec, inner.timestamp_nanosec))
        .position(foxglove_msgs::FoxglovePoint2 {
            x: inner.pos_x,
            y: inner.pos_y,
        })
        .text(inner.text.as_str())
        .font_size(inner.font_size)
        .text_color(foxglove_msgs::FoxgloveColor {
            r: inner.text_color_r,
            g: inner.text_color_g,
            b: inner.text_color_b,
            a: inner.text_color_a,
        })
        .background_color(foxglove_msgs::FoxgloveColor {
            r: inner.bg_color_r,
            g: inner.bg_color_g,
            b: inner.bg_color_b,
            a: inner.bg_color_a,
        })
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_builder_encode_into(
    b: *mut ros_foxglove_text_annotation_builder_t,
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
    let r = foxglove_msgs::FoxgloveTextAnnotation::builder()
        .timestamp(Time::new(inner.timestamp_sec, inner.timestamp_nanosec))
        .position(foxglove_msgs::FoxglovePoint2 {
            x: inner.pos_x,
            y: inner.pos_y,
        })
        .text(inner.text.as_str())
        .font_size(inner.font_size)
        .text_color(foxglove_msgs::FoxgloveColor {
            r: inner.text_color_r,
            g: inner.text_color_g,
            b: inner.text_color_b,
            a: inner.text_color_a,
        })
        .background_color(foxglove_msgs::FoxgloveColor {
            r: inner.bg_color_r,
            g: inner.bg_color_g,
            b: inner.bg_color_b,
            a: inner.bg_color_a,
        })
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── foxglove_msgs::FoxglovePointAnnotation ──────────────────────────
//
// `points` is a borrowed array of `ros_foxglove_point2_elem_t` descriptors
// (plain f64 pairs, no inner borrow). The builder also exposes optional
// `outline_colors` (per-point color overrides); both arrays are borrowed.

/// C-POD descriptor for a single FoxglovePoint2 element.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct ros_foxglove_point2_elem_t {
    pub x: f64,
    pub y: f64,
}

/// C-POD descriptor for a single FoxgloveColor element.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct ros_foxglove_color_elem_t {
    pub r: f64,
    pub g: f64,
    pub b: f64,
    pub a: f64,
}

unsafe fn foxglove_point2_descs_to_vec(
    descs: *const ros_foxglove_point2_elem_t,
    count: usize,
) -> Vec<foxglove_msgs::FoxglovePoint2> {
    if descs.is_null() || count == 0 {
        return Vec::new();
    }
    let descs = slice::from_raw_parts(descs, count);
    descs
        .iter()
        .map(|d| foxglove_msgs::FoxglovePoint2 { x: d.x, y: d.y })
        .collect()
}

unsafe fn foxglove_color_descs_to_vec(
    descs: *const ros_foxglove_color_elem_t,
    count: usize,
) -> Vec<foxglove_msgs::FoxgloveColor> {
    if descs.is_null() || count == 0 {
        return Vec::new();
    }
    let descs = slice::from_raw_parts(descs, count);
    descs
        .iter()
        .map(|d| foxglove_msgs::FoxgloveColor {
            r: d.r,
            g: d.g,
            b: d.b,
            a: d.a,
        })
        .collect()
}

struct FoxglovePointAnnotationBuilderOwned {
    timestamp_sec: i32,
    timestamp_nanosec: u32,
    type_: u8,
    points: *const ros_foxglove_point2_elem_t,
    points_count: usize,
    outline_color_r: f64,
    outline_color_g: f64,
    outline_color_b: f64,
    outline_color_a: f64,
    outline_colors: *const ros_foxglove_color_elem_t,
    outline_colors_count: usize,
    fill_color_r: f64,
    fill_color_g: f64,
    fill_color_b: f64,
    fill_color_a: f64,
    thickness: f64,
}

pub struct ros_foxglove_point_annotation_builder_t(FoxglovePointAnnotationBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_builder_new(
) -> *mut ros_foxglove_point_annotation_builder_t {
    Box::into_raw(Box::new(ros_foxglove_point_annotation_builder_t(
        FoxglovePointAnnotationBuilderOwned {
            timestamp_sec: 0,
            timestamp_nanosec: 0,
            type_: 0,
            points: ptr::null(),
            points_count: 0,
            outline_color_r: 0.0,
            outline_color_g: 0.0,
            outline_color_b: 0.0,
            outline_color_a: 0.0,
            outline_colors: ptr::null(),
            outline_colors_count: 0,
            fill_color_r: 0.0,
            fill_color_g: 0.0,
            fill_color_b: 0.0,
            fill_color_a: 0.0,
            thickness: 0.0,
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_builder_free(
    b: *mut ros_foxglove_point_annotation_builder_t,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_builder_set_timestamp(
    b: *mut ros_foxglove_point_annotation_builder_t,
    sec: i32,
    nanosec: u32,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.timestamp_sec = sec;
    inner.timestamp_nanosec = nanosec;
}

#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_builder_set_type(
    b: *mut ros_foxglove_point_annotation_builder_t,
    v: u8,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.type_ = v;
    }
}

/// Set the points descriptor sequence (BORROWED).
#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_builder_set_points(
    b: *mut ros_foxglove_point_annotation_builder_t,
    points: *const ros_foxglove_point2_elem_t,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if points.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.points = points;
        (*b).0.points_count = count;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_builder_set_outline_color(
    b: *mut ros_foxglove_point_annotation_builder_t,
    r: f64,
    g: f64,
    bc: f64,
    a: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.outline_color_r = r;
    inner.outline_color_g = g;
    inner.outline_color_b = bc;
    inner.outline_color_a = a;
}

/// Set the outline_colors descriptor sequence (BORROWED).
#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_builder_set_outline_colors(
    b: *mut ros_foxglove_point_annotation_builder_t,
    colors: *const ros_foxglove_color_elem_t,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if colors.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.outline_colors = colors;
        (*b).0.outline_colors_count = count;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_builder_set_fill_color(
    b: *mut ros_foxglove_point_annotation_builder_t,
    r: f64,
    g: f64,
    bc: f64,
    a: f64,
) {
    if b.is_null() {
        return;
    }
    let inner = unsafe { &mut (*b).0 };
    inner.fill_color_r = r;
    inner.fill_color_g = g;
    inner.fill_color_b = bc;
    inner.fill_color_a = a;
}

#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_builder_set_thickness(
    b: *mut ros_foxglove_point_annotation_builder_t,
    v: f64,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        (*b).0.thickness = v;
    }
}

#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_builder_build(
    b: *mut ros_foxglove_point_annotation_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let pts = unsafe { foxglove_point2_descs_to_vec(inner.points, inner.points_count) };
    let ocs =
        unsafe { foxglove_color_descs_to_vec(inner.outline_colors, inner.outline_colors_count) };
    let r = foxglove_msgs::FoxglovePointAnnotation::builder()
        .timestamp(Time::new(inner.timestamp_sec, inner.timestamp_nanosec))
        .type_(inner.type_)
        .points(&pts)
        .outline_color(foxglove_msgs::FoxgloveColor {
            r: inner.outline_color_r,
            g: inner.outline_color_g,
            b: inner.outline_color_b,
            a: inner.outline_color_a,
        })
        .outline_colors(&ocs)
        .fill_color(foxglove_msgs::FoxgloveColor {
            r: inner.fill_color_r,
            g: inner.fill_color_g,
            b: inner.fill_color_b,
            a: inner.fill_color_a,
        })
        .thickness(inner.thickness)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_builder_encode_into(
    b: *mut ros_foxglove_point_annotation_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let pts = unsafe { foxglove_point2_descs_to_vec(inner.points, inner.points_count) };
    let ocs =
        unsafe { foxglove_color_descs_to_vec(inner.outline_colors, inner.outline_colors_count) };
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    let r = foxglove_msgs::FoxglovePointAnnotation::builder()
        .timestamp(Time::new(inner.timestamp_sec, inner.timestamp_nanosec))
        .type_(inner.type_)
        .points(&pts)
        .outline_color(foxglove_msgs::FoxgloveColor {
            r: inner.outline_color_r,
            g: inner.outline_color_g,
            b: inner.outline_color_b,
            a: inner.outline_color_a,
        })
        .outline_colors(&ocs)
        .fill_color(foxglove_msgs::FoxgloveColor {
            r: inner.fill_color_r,
            g: inner.fill_color_g,
            b: inner.fill_color_b,
            a: inner.fill_color_a,
        })
        .thickness(inner.thickness)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// ── foxglove_msgs::FoxgloveImageAnnotation ──────────────────────────
//
// Three nested sequences: circles (CdrFixed FoxgloveCircleAnnotations),
// points (FoxglovePointAnnotationView with owned Vec<Point2>/Vec<Color>),
// texts (FoxgloveTextAnnotationView with borrowed text string).

/// C-POD descriptor for a FoxgloveCircleAnnotations element. All fields
/// are plain values; nothing is borrowed.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct ros_foxglove_circle_annotation_elem_t {
    pub timestamp_sec: i32,
    pub timestamp_nanosec: u32,
    pub position_x: f64,
    pub position_y: f64,
    pub diameter: f64,
    pub thickness: f64,
    pub fill_color_r: f64,
    pub fill_color_g: f64,
    pub fill_color_b: f64,
    pub fill_color_a: f64,
    pub outline_color_r: f64,
    pub outline_color_g: f64,
    pub outline_color_b: f64,
    pub outline_color_a: f64,
}

/// C-POD descriptor for a FoxglovePointAnnotation element. Inner `points`
/// and `outline_colors` arrays are borrowed and must outlive the build/
/// encode_into call. Each individual element is a plain f64 record.
#[repr(C)]
pub struct ros_foxglove_point_annotation_elem_t {
    pub timestamp_sec: i32,
    pub timestamp_nanosec: u32,
    pub type_: u8,
    pub points: *const ros_foxglove_point2_elem_t,
    pub points_count: usize,
    pub outline_color_r: f64,
    pub outline_color_g: f64,
    pub outline_color_b: f64,
    pub outline_color_a: f64,
    pub outline_colors: *const ros_foxglove_color_elem_t,
    pub outline_colors_count: usize,
    pub fill_color_r: f64,
    pub fill_color_g: f64,
    pub fill_color_b: f64,
    pub fill_color_a: f64,
    pub thickness: f64,
}

/// C-POD descriptor for a FoxgloveTextAnnotation element. `text` is a
/// borrowed C string and must outlive the build/encode_into call.
#[repr(C)]
pub struct ros_foxglove_text_annotation_elem_t {
    pub timestamp_sec: i32,
    pub timestamp_nanosec: u32,
    pub position_x: f64,
    pub position_y: f64,
    pub text: *const c_char,
    pub font_size: f64,
    pub text_color_r: f64,
    pub text_color_g: f64,
    pub text_color_b: f64,
    pub text_color_a: f64,
    pub background_color_r: f64,
    pub background_color_g: f64,
    pub background_color_b: f64,
    pub background_color_a: f64,
}

unsafe fn circle_descs_to_vec(
    descs: *const ros_foxglove_circle_annotation_elem_t,
    count: usize,
) -> Vec<foxglove_msgs::FoxgloveCircleAnnotations> {
    if descs.is_null() || count == 0 {
        return Vec::new();
    }
    let descs = slice::from_raw_parts(descs, count);
    descs
        .iter()
        .map(|d| foxglove_msgs::FoxgloveCircleAnnotations {
            timestamp: Time::new(d.timestamp_sec, d.timestamp_nanosec),
            position: foxglove_msgs::FoxglovePoint2 {
                x: d.position_x,
                y: d.position_y,
            },
            diameter: d.diameter,
            thickness: d.thickness,
            fill_color: foxglove_msgs::FoxgloveColor {
                r: d.fill_color_r,
                g: d.fill_color_g,
                b: d.fill_color_b,
                a: d.fill_color_a,
            },
            outline_color: foxglove_msgs::FoxgloveColor {
                r: d.outline_color_r,
                g: d.outline_color_g,
                b: d.outline_color_b,
                a: d.outline_color_a,
            },
        })
        .collect()
}

unsafe fn point_annotation_descs_to_vec(
    descs: *const ros_foxglove_point_annotation_elem_t,
    count: usize,
) -> Vec<foxglove_msgs::FoxglovePointAnnotationView> {
    if descs.is_null() || count == 0 {
        return Vec::new();
    }
    let descs = slice::from_raw_parts(descs, count);
    descs
        .iter()
        .map(|d| foxglove_msgs::FoxglovePointAnnotationView {
            timestamp: Time::new(d.timestamp_sec, d.timestamp_nanosec),
            type_: d.type_,
            points: foxglove_point2_descs_to_vec(d.points, d.points_count),
            outline_color: foxglove_msgs::FoxgloveColor {
                r: d.outline_color_r,
                g: d.outline_color_g,
                b: d.outline_color_b,
                a: d.outline_color_a,
            },
            outline_colors: foxglove_color_descs_to_vec(d.outline_colors, d.outline_colors_count),
            fill_color: foxglove_msgs::FoxgloveColor {
                r: d.fill_color_r,
                g: d.fill_color_g,
                b: d.fill_color_b,
                a: d.fill_color_a,
            },
            thickness: d.thickness,
        })
        .collect()
}

unsafe fn text_annotation_descs_to_vec(
    descs: *const ros_foxglove_text_annotation_elem_t,
    count: usize,
) -> Result<Vec<foxglove_msgs::FoxgloveTextAnnotationView<'static>>, ()> {
    if descs.is_null() || count == 0 {
        return Ok(Vec::new());
    }
    let descs = slice::from_raw_parts(descs, count);
    descs
        .iter()
        .map(|d| {
            let text = c_to_str_checked(d.text)?;
            Ok(foxglove_msgs::FoxgloveTextAnnotationView {
                timestamp: Time::new(d.timestamp_sec, d.timestamp_nanosec),
                position: foxglove_msgs::FoxglovePoint2 {
                    x: d.position_x,
                    y: d.position_y,
                },
                text,
                font_size: d.font_size,
                text_color: foxglove_msgs::FoxgloveColor {
                    r: d.text_color_r,
                    g: d.text_color_g,
                    b: d.text_color_b,
                    a: d.text_color_a,
                },
                background_color: foxglove_msgs::FoxgloveColor {
                    r: d.background_color_r,
                    g: d.background_color_g,
                    b: d.background_color_b,
                    a: d.background_color_a,
                },
            })
        })
        .collect()
}

struct FoxgloveImageAnnotationBuilderOwned {
    circles: *const ros_foxglove_circle_annotation_elem_t,
    circles_count: usize,
    points: *const ros_foxglove_point_annotation_elem_t,
    points_count: usize,
    texts: *const ros_foxglove_text_annotation_elem_t,
    texts_count: usize,
}

pub struct ros_foxglove_image_annotation_builder_t(FoxgloveImageAnnotationBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_foxglove_image_annotation_builder_new(
) -> *mut ros_foxglove_image_annotation_builder_t {
    Box::into_raw(Box::new(ros_foxglove_image_annotation_builder_t(
        FoxgloveImageAnnotationBuilderOwned {
            circles: ptr::null(),
            circles_count: 0,
            points: ptr::null(),
            points_count: 0,
            texts: ptr::null(),
            texts_count: 0,
        },
    )))
}

#[no_mangle]
pub extern "C" fn ros_foxglove_image_annotation_builder_free(
    b: *mut ros_foxglove_image_annotation_builder_t,
) {
    if b.is_null() {
        return;
    }
    unsafe {
        drop(Box::from_raw(b));
    }
}

/// Set the circles descriptor sequence (BORROWED).
#[no_mangle]
pub extern "C" fn ros_foxglove_image_annotation_builder_set_circles(
    b: *mut ros_foxglove_image_annotation_builder_t,
    circles: *const ros_foxglove_circle_annotation_elem_t,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if circles.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.circles = circles;
        (*b).0.circles_count = count;
    }
    0
}

/// Set the points descriptor sequence (BORROWED — including each
/// element's inner `points`/`outline_colors` arrays).
#[no_mangle]
pub extern "C" fn ros_foxglove_image_annotation_builder_set_points(
    b: *mut ros_foxglove_image_annotation_builder_t,
    points: *const ros_foxglove_point_annotation_elem_t,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if points.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.points = points;
        (*b).0.points_count = count;
    }
    0
}

/// Set the texts descriptor sequence (BORROWED — including each
/// element's `text` C string).
#[no_mangle]
pub extern "C" fn ros_foxglove_image_annotation_builder_set_texts(
    b: *mut ros_foxglove_image_annotation_builder_t,
    texts: *const ros_foxglove_text_annotation_elem_t,
    count: usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if texts.is_null() && count > 0 {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.texts = texts;
        (*b).0.texts_count = count;
    }
    0
}

#[no_mangle]
pub extern "C" fn ros_foxglove_image_annotation_builder_build(
    b: *mut ros_foxglove_image_annotation_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let circles = unsafe { circle_descs_to_vec(inner.circles, inner.circles_count) };
    let points = unsafe { point_annotation_descs_to_vec(inner.points, inner.points_count) };
    let texts = match unsafe { text_annotation_descs_to_vec(inner.texts, inner.texts_count) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    let r = foxglove_msgs::FoxgloveImageAnnotation::builder()
        .circles(&circles)
        .points(&points)
        .texts(&texts)
        .build();
    match r {
        Ok(v) => return_cdr_bytes(v.into_cdr(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn ros_foxglove_image_annotation_builder_encode_into(
    b: *mut ros_foxglove_image_annotation_builder_t,
    buf: *mut u8,
    cap: usize,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() || out_len.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*b).0 };
    let circles = unsafe { circle_descs_to_vec(inner.circles, inner.circles_count) };
    let points = unsafe { point_annotation_descs_to_vec(inner.points, inner.points_count) };
    let texts = match unsafe { text_annotation_descs_to_vec(inner.texts, inner.texts_count) } {
        Ok(v) => v,
        Err(_) => return -1,
    };
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    let r = foxglove_msgs::FoxgloveImageAnnotation::builder()
        .circles(&circles)
        .points(&points)
        .texts(&texts)
        .encode_into_slice(dst);
    match r {
        Ok(n) => {
            unsafe {
                *out_len = n;
            }
            0
        }
        Err(crate::cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

// =============================================================================
// In-place scalar setters (3.2.0+)
//
// Stateless mutators over a caller-owned CDR buffer. Each call re-parses the
// buffer via from_cdr to locate the field, then writes the new value in place.
// Only fixed-size fields are exposed — variable-length fields (strings, bulk
// data, nested sequences) require the builder API.
//
// Signature: ros_<type>_set_<field>(buf, len, value...) -> i32
//   buf:  *mut u8  — CDR buffer to mutate
//   len:  usize    — buffer length (must match the encoded CDR length)
//   ...:  primitive field args
// Returns:
//   0          — success
//   -1, EINVAL — NULL buf
//   -1, EBADMSG — buffer not a valid encoded message of this type
// =============================================================================

/// Set the stamp field in place on an existing CDR-encoded Header buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_header_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: std_msgs::Header<&mut [u8]> = match std_msgs::Header::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on an existing CDR-encoded Image buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_image_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Image<&mut [u8]> = match sensor_msgs::Image::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the height field in place on an Image buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_image_set_height(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Image<&mut [u8]> = match sensor_msgs::Image::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_height(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the width field in place on an Image buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_image_set_width(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Image<&mut [u8]> = match sensor_msgs::Image::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_width(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the is_bigendian field in place on an Image buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_image_set_is_bigendian(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Image<&mut [u8]> = match sensor_msgs::Image::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_is_bigendian(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the step (row stride) field in place on an Image buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_image_set_step(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Image<&mut [u8]> = match sensor_msgs::Image::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_step(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a CompressedImage buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_compressed_image_set_stamp(
    buf: *mut u8,
    len: usize,
    sec: i32,
    nsec: u32,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::CompressedImage<&mut [u8]> =
        match sensor_msgs::CompressedImage::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on an Imu buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_imu_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Imu<&mut [u8]> = match sensor_msgs::Imu::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the orientation (Quaternion) field in place on an Imu buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_imu_set_orientation(
    buf: *mut u8,
    len: usize,
    x: f64,
    y: f64,
    z: f64,
    w: f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Imu<&mut [u8]> = match sensor_msgs::Imu::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_orientation(Quaternion { x, y, z, w }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the 3x3 orientation covariance (row-major, 9 f64 elements) in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_imu_set_orientation_covariance(
    buf: *mut u8,
    len: usize,
    c: *const f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if c.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let mut arr = [0.0f64; 9];
    unsafe {
        ptr::copy_nonoverlapping(c, arr.as_mut_ptr(), 9);
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Imu<&mut [u8]> = match sensor_msgs::Imu::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_orientation_covariance(arr) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the angular_velocity (Vector3) field in place on an Imu buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_imu_set_angular_velocity(
    buf: *mut u8,
    len: usize,
    x: f64,
    y: f64,
    z: f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Imu<&mut [u8]> = match sensor_msgs::Imu::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_angular_velocity(Vector3 { x, y, z }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the 3x3 angular velocity covariance (row-major, 9 f64 elements) in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_imu_set_angular_velocity_covariance(
    buf: *mut u8,
    len: usize,
    c: *const f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if c.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let mut arr = [0.0f64; 9];
    unsafe {
        ptr::copy_nonoverlapping(c, arr.as_mut_ptr(), 9);
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Imu<&mut [u8]> = match sensor_msgs::Imu::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_angular_velocity_covariance(arr) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the linear_acceleration (Vector3) field in place on an Imu buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_imu_set_linear_acceleration(
    buf: *mut u8,
    len: usize,
    x: f64,
    y: f64,
    z: f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Imu<&mut [u8]> = match sensor_msgs::Imu::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_linear_acceleration(Vector3 { x, y, z }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the 3x3 linear acceleration covariance (row-major, 9 f64 elements) in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_imu_set_linear_acceleration_covariance(
    buf: *mut u8,
    len: usize,
    c: *const f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if c.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let mut arr = [0.0f64; 9];
    unsafe {
        ptr::copy_nonoverlapping(c, arr.as_mut_ptr(), 9);
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Imu<&mut [u8]> = match sensor_msgs::Imu::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_linear_acceleration_covariance(arr) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a NavSatFix buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::NavSatFix<&mut [u8]> = match sensor_msgs::NavSatFix::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the NavSatStatus in place on a NavSatFix buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_set_status(
    buf: *mut u8,
    len: usize,
    status: i8,
    service: u16,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::NavSatFix<&mut [u8]> = match sensor_msgs::NavSatFix::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_status(NavSatStatus { status, service }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the latitude field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_set_latitude(buf: *mut u8, len: usize, v: f64) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::NavSatFix<&mut [u8]> = match sensor_msgs::NavSatFix::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_latitude(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the longitude field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_set_longitude(buf: *mut u8, len: usize, v: f64) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::NavSatFix<&mut [u8]> = match sensor_msgs::NavSatFix::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_longitude(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the altitude field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_set_altitude(buf: *mut u8, len: usize, v: f64) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::NavSatFix<&mut [u8]> = match sensor_msgs::NavSatFix::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_altitude(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the 3x3 position covariance (row-major, 9 f64 elements) in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_set_position_covariance(
    buf: *mut u8,
    len: usize,
    c: *const f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if c.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let mut arr = [0.0f64; 9];
    unsafe {
        ptr::copy_nonoverlapping(c, arr.as_mut_ptr(), 9);
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::NavSatFix<&mut [u8]> = match sensor_msgs::NavSatFix::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_position_covariance(arr) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the position_covariance_type in place on a NavSatFix buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_nav_sat_fix_set_position_covariance_type(
    buf: *mut u8,
    len: usize,
    v: u8,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::NavSatFix<&mut [u8]> = match sensor_msgs::NavSatFix::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_position_covariance_type(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the offset field in place on a PointField buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_point_field_set_offset(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::PointField<&mut [u8]> = match sensor_msgs::PointField::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_offset(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the datatype field in place on a PointField buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_point_field_set_datatype(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::PointField<&mut [u8]> = match sensor_msgs::PointField::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_datatype(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the count field in place on a PointField buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_point_field_set_count(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::PointField<&mut [u8]> = match sensor_msgs::PointField::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_count(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a PointCloud2 buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::PointCloud2<&mut [u8]> = match sensor_msgs::PointCloud2::from_cdr(slice)
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the height field in place on a PointCloud2 buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_height(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::PointCloud2<&mut [u8]> = match sensor_msgs::PointCloud2::from_cdr(slice)
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_height(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the width field in place on a PointCloud2 buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_width(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::PointCloud2<&mut [u8]> = match sensor_msgs::PointCloud2::from_cdr(slice)
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_width(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the is_bigendian field in place on a PointCloud2 buffer (0=false, nonzero=true).
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_is_bigendian(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::PointCloud2<&mut [u8]> = match sensor_msgs::PointCloud2::from_cdr(slice)
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_is_bigendian(v != 0) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the point_step field in place on a PointCloud2 buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_point_step(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::PointCloud2<&mut [u8]> = match sensor_msgs::PointCloud2::from_cdr(slice)
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_point_step(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the row_step field in place on a PointCloud2 buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_row_step(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::PointCloud2<&mut [u8]> = match sensor_msgs::PointCloud2::from_cdr(slice)
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_row_step(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the is_dense field in place on a PointCloud2 buffer (0=false, nonzero=true).
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_point_cloud2_set_is_dense(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::PointCloud2<&mut [u8]> = match sensor_msgs::PointCloud2::from_cdr(slice)
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_is_dense(v != 0) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a CameraInfo buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_info_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::CameraInfo<&mut [u8]> = match sensor_msgs::CameraInfo::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the height field in place on a CameraInfo buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_info_set_height(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::CameraInfo<&mut [u8]> = match sensor_msgs::CameraInfo::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_height(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the width field in place on a CameraInfo buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_info_set_width(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::CameraInfo<&mut [u8]> = match sensor_msgs::CameraInfo::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_width(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the 3x3 intrinsic matrix K (row-major, 9 f64 elements) in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_info_set_k(buf: *mut u8, len: usize, k: *const f64) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if k.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let mut arr = [0.0f64; 9];
    unsafe {
        ptr::copy_nonoverlapping(k, arr.as_mut_ptr(), 9);
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::CameraInfo<&mut [u8]> = match sensor_msgs::CameraInfo::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_k(arr) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the 3x3 rectification matrix R (row-major, 9 f64 elements) in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_info_set_r(buf: *mut u8, len: usize, r: *const f64) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if r.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let mut arr = [0.0f64; 9];
    unsafe {
        ptr::copy_nonoverlapping(r, arr.as_mut_ptr(), 9);
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::CameraInfo<&mut [u8]> = match sensor_msgs::CameraInfo::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_r(arr) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the 3x4 projection matrix P (row-major, 12 f64 elements) in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_info_set_p(buf: *mut u8, len: usize, p: *const f64) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if p.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let mut arr = [0.0f64; 12];
    unsafe {
        ptr::copy_nonoverlapping(p, arr.as_mut_ptr(), 12);
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::CameraInfo<&mut [u8]> = match sensor_msgs::CameraInfo::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_p(arr) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the binning_x field in place on a CameraInfo buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_info_set_binning_x(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::CameraInfo<&mut [u8]> = match sensor_msgs::CameraInfo::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_binning_x(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the binning_y field in place on a CameraInfo buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_info_set_binning_y(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::CameraInfo<&mut [u8]> = match sensor_msgs::CameraInfo::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_binning_y(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the RegionOfInterest (roi) field in place on a CameraInfo buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_info_set_roi(
    buf: *mut u8,
    len: usize,
    x_offset: u32,
    y_offset: u32,
    height: u32,
    width: u32,
    do_rectify: u8,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::CameraInfo<&mut [u8]> = match sensor_msgs::CameraInfo::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_roi(RegionOfInterest {
        x_offset,
        y_offset,
        height,
        width,
        do_rectify: do_rectify != 0,
    }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a MagneticField buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_magnetic_field_set_stamp(
    buf: *mut u8,
    len: usize,
    sec: i32,
    nsec: u32,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::MagneticField<&mut [u8]> =
        match sensor_msgs::MagneticField::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the magnetic_field (Vector3) in place on a MagneticField buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_magnetic_field_set_magnetic_field(
    buf: *mut u8,
    len: usize,
    x: f64,
    y: f64,
    z: f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::MagneticField<&mut [u8]> =
        match sensor_msgs::MagneticField::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_magnetic_field(Vector3 { x, y, z }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the 3x3 magnetic field covariance (row-major, 9 f64 elements) in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_magnetic_field_set_magnetic_field_covariance(
    buf: *mut u8,
    len: usize,
    c: *const f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    if c.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let mut arr = [0.0f64; 9];
    unsafe {
        ptr::copy_nonoverlapping(c, arr.as_mut_ptr(), 9);
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::MagneticField<&mut [u8]> =
        match sensor_msgs::MagneticField::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_magnetic_field_covariance(arr) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a FluidPressure buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_fluid_pressure_set_stamp(
    buf: *mut u8,
    len: usize,
    sec: i32,
    nsec: u32,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::FluidPressure<&mut [u8]> =
        match sensor_msgs::FluidPressure::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the fluid_pressure field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_fluid_pressure_set_fluid_pressure(buf: *mut u8, len: usize, v: f64) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::FluidPressure<&mut [u8]> =
        match sensor_msgs::FluidPressure::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_fluid_pressure(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the variance field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_fluid_pressure_set_variance(buf: *mut u8, len: usize, v: f64) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::FluidPressure<&mut [u8]> =
        match sensor_msgs::FluidPressure::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_variance(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a Temperature buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_temperature_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Temperature<&mut [u8]> = match sensor_msgs::Temperature::from_cdr(slice)
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the temperature field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_temperature_set_temperature(buf: *mut u8, len: usize, v: f64) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Temperature<&mut [u8]> = match sensor_msgs::Temperature::from_cdr(slice)
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_temperature(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the variance field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_temperature_set_variance(buf: *mut u8, len: usize, v: f64) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::Temperature<&mut [u8]> = match sensor_msgs::Temperature::from_cdr(slice)
    {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_variance(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a BatteryState buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_battery_state_set_stamp(
    buf: *mut u8,
    len: usize,
    sec: i32,
    nsec: u32,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::BatteryState<&mut [u8]> =
        match sensor_msgs::BatteryState::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the voltage field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_battery_state_set_voltage(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::BatteryState<&mut [u8]> =
        match sensor_msgs::BatteryState::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_voltage(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the temperature field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_battery_state_set_temperature(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::BatteryState<&mut [u8]> =
        match sensor_msgs::BatteryState::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_temperature(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the current field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_battery_state_set_current(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::BatteryState<&mut [u8]> =
        match sensor_msgs::BatteryState::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_current(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the charge field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_battery_state_set_charge(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::BatteryState<&mut [u8]> =
        match sensor_msgs::BatteryState::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_charge(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the capacity field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_battery_state_set_capacity(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::BatteryState<&mut [u8]> =
        match sensor_msgs::BatteryState::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_capacity(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the design_capacity field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_battery_state_set_design_capacity(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::BatteryState<&mut [u8]> =
        match sensor_msgs::BatteryState::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_design_capacity(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the percentage field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_battery_state_set_percentage(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::BatteryState<&mut [u8]> =
        match sensor_msgs::BatteryState::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_percentage(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the power_supply_status field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_battery_state_set_power_supply_status(
    buf: *mut u8,
    len: usize,
    v: u8,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::BatteryState<&mut [u8]> =
        match sensor_msgs::BatteryState::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_power_supply_status(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the power_supply_health field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_battery_state_set_power_supply_health(
    buf: *mut u8,
    len: usize,
    v: u8,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::BatteryState<&mut [u8]> =
        match sensor_msgs::BatteryState::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_power_supply_health(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the power_supply_technology field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_battery_state_set_power_supply_technology(
    buf: *mut u8,
    len: usize,
    v: u8,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::BatteryState<&mut [u8]> =
        match sensor_msgs::BatteryState::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_power_supply_technology(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the present field in place (0=false, nonzero=true).
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_battery_state_set_present(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: sensor_msgs::BatteryState<&mut [u8]> =
        match sensor_msgs::BatteryState::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_present(v != 0) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the height field in place on a Mask buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_mask_set_height(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Mask<&mut [u8]> = match edgefirst_msgs::Mask::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_height(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the width field in place on a Mask buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_mask_set_width(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Mask<&mut [u8]> = match edgefirst_msgs::Mask::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_width(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the length field in place on a Mask buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_mask_set_length(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Mask<&mut [u8]> = match edgefirst_msgs::Mask::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_length(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the boxed field in place (0=false, nonzero=true).
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_mask_set_boxed(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Mask<&mut [u8]> = match edgefirst_msgs::Mask::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_boxed(v != 0) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a LocalTime buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_local_time_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::LocalTime<&mut [u8]> =
        match edgefirst_msgs::LocalTime::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the Date field in place on a LocalTime buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_local_time_set_date(
    buf: *mut u8,
    len: usize,
    year: u16,
    month: u8,
    day: u8,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::LocalTime<&mut [u8]> =
        match edgefirst_msgs::LocalTime::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_date(edgefirst_msgs::Date { year, month, day }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the time (Time) field in place on a LocalTime buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_local_time_set_time(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::LocalTime<&mut [u8]> =
        match edgefirst_msgs::LocalTime::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_time(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the timezone field in place on a LocalTime buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_local_time_set_timezone(buf: *mut u8, len: usize, v: i16) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::LocalTime<&mut [u8]> =
        match edgefirst_msgs::LocalTime::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_timezone(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a RadarCube buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_radar_cube_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::RadarCube<&mut [u8]> =
        match edgefirst_msgs::RadarCube::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the timestamp (u64 ns) field in place on a RadarCube buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_radar_cube_set_timestamp(buf: *mut u8, len: usize, v: u64) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::RadarCube<&mut [u8]> =
        match edgefirst_msgs::RadarCube::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_timestamp(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the is_complex field in place (0=false, nonzero=true).
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_radar_cube_set_is_complex(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::RadarCube<&mut [u8]> =
        match edgefirst_msgs::RadarCube::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_is_complex(v != 0) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a RadarInfo buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_radar_info_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::RadarInfo<&mut [u8]> =
        match edgefirst_msgs::RadarInfo::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the cube field in place on a RadarInfo buffer (0=false, nonzero=true).
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_radar_info_set_cube(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::RadarInfo<&mut [u8]> =
        match edgefirst_msgs::RadarInfo::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_cube(v != 0) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the lifetime field in place on a Track buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_track_set_lifetime(buf: *mut u8, len: usize, v: i32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Track<&mut [u8]> = match edgefirst_msgs::Track::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_lifetime(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the created (Time) field in place on a Track buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_track_set_created(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Track<&mut [u8]> = match edgefirst_msgs::Track::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_created(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the center_x field in place on a DetectBox buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_box_set_center_x(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::DetectBox<&mut [u8]> =
        match edgefirst_msgs::DetectBox::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_center_x(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the center_y field in place on a DetectBox buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_box_set_center_y(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::DetectBox<&mut [u8]> =
        match edgefirst_msgs::DetectBox::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_center_y(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the width field in place on a DetectBox buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_box_set_width(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::DetectBox<&mut [u8]> =
        match edgefirst_msgs::DetectBox::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_width(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the height field in place on a DetectBox buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_box_set_height(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::DetectBox<&mut [u8]> =
        match edgefirst_msgs::DetectBox::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_height(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the score field in place on a DetectBox buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_box_set_score(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::DetectBox<&mut [u8]> =
        match edgefirst_msgs::DetectBox::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_score(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the distance field in place on a DetectBox buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_box_set_distance(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::DetectBox<&mut [u8]> =
        match edgefirst_msgs::DetectBox::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_distance(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the speed field in place on a DetectBox buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_box_set_speed(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::DetectBox<&mut [u8]> =
        match edgefirst_msgs::DetectBox::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_speed(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the track_lifetime field in place on a DetectBox buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_box_set_track_lifetime(buf: *mut u8, len: usize, v: i32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::DetectBox<&mut [u8]> =
        match edgefirst_msgs::DetectBox::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_track_lifetime(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the track_created (Time) field in place on a DetectBox buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_box_set_track_created(
    buf: *mut u8,
    len: usize,
    sec: i32,
    nsec: u32,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::DetectBox<&mut [u8]> =
        match edgefirst_msgs::DetectBox::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_track_created(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp (Time) field in place on a Detect buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Detect<&mut [u8]> = match edgefirst_msgs::Detect::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the input_timestamp (Time) field in place on a Detect buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_set_input_timestamp(
    buf: *mut u8,
    len: usize,
    sec: i32,
    nsec: u32,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Detect<&mut [u8]> = match edgefirst_msgs::Detect::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_input_timestamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the model_time (Time) field in place on a Detect buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_set_model_time(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Detect<&mut [u8]> = match edgefirst_msgs::Detect::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_model_time(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the output_time (Time) field in place on a Detect buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_detect_set_output_time(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Detect<&mut [u8]> = match edgefirst_msgs::Detect::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_output_time(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a CameraFrame buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_frame_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::CameraFrame<&mut [u8]> =
        match edgefirst_msgs::CameraFrame::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the seq field in place on a CameraFrame buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_frame_set_seq(buf: *mut u8, len: usize, v: u64) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::CameraFrame<&mut [u8]> =
        match edgefirst_msgs::CameraFrame::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_seq(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the pid field in place on a CameraFrame buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_frame_set_pid(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::CameraFrame<&mut [u8]> =
        match edgefirst_msgs::CameraFrame::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_pid(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the width field in place on a CameraFrame buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_frame_set_width(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::CameraFrame<&mut [u8]> =
        match edgefirst_msgs::CameraFrame::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_width(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the height field in place on a CameraFrame buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_frame_set_height(buf: *mut u8, len: usize, v: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::CameraFrame<&mut [u8]> =
        match edgefirst_msgs::CameraFrame::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_height(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the fence_fd field in place on a CameraFrame buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_camera_frame_set_fence_fd(buf: *mut u8, len: usize, v: i32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::CameraFrame<&mut [u8]> =
        match edgefirst_msgs::CameraFrame::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_fence_fd(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp (Time) field in place on a Model buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_model_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Model<&mut [u8]> = match edgefirst_msgs::Model::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the input_time (Duration) field in place on a Model buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_model_set_input_time(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Model<&mut [u8]> = match edgefirst_msgs::Model::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_input_time(Duration::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the model_time (Duration) field in place on a Model buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_model_set_model_time(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Model<&mut [u8]> = match edgefirst_msgs::Model::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_model_time(Duration::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the output_time (Duration) field in place on a Model buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_model_set_output_time(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Model<&mut [u8]> = match edgefirst_msgs::Model::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_output_time(Duration::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the decode_time (Duration) field in place on a Model buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_model_set_decode_time(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Model<&mut [u8]> = match edgefirst_msgs::Model::from_cdr(slice) {
        Ok(v) => v,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };
    match m.set_decode_time(Duration::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a ModelInfo buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_model_info_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::ModelInfo<&mut [u8]> =
        match edgefirst_msgs::ModelInfo::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the input_type field in place on a ModelInfo buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_model_info_set_input_type(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::ModelInfo<&mut [u8]> =
        match edgefirst_msgs::ModelInfo::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_input_type(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the output_type field in place on a ModelInfo buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_model_info_set_output_type(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::ModelInfo<&mut [u8]> =
        match edgefirst_msgs::ModelInfo::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_output_type(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a Vibration buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_vibration_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Vibration<&mut [u8]> =
        match edgefirst_msgs::Vibration::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the vibration (Vector3) field in place on a Vibration buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_vibration_set_vibration(
    buf: *mut u8,
    len: usize,
    x: f64,
    y: f64,
    z: f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Vibration<&mut [u8]> =
        match edgefirst_msgs::Vibration::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_vibration(Vector3 { x, y, z }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the band_lower_hz field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_vibration_set_band_lower_hz(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Vibration<&mut [u8]> =
        match edgefirst_msgs::Vibration::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_band_lower_hz(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the band_upper_hz field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_vibration_set_band_upper_hz(buf: *mut u8, len: usize, v: f32) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Vibration<&mut [u8]> =
        match edgefirst_msgs::Vibration::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_band_upper_hz(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the measurement_type field in place on a Vibration buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_vibration_set_measurement_type(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Vibration<&mut [u8]> =
        match edgefirst_msgs::Vibration::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_measurement_type(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the unit field in place on a Vibration buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_vibration_set_unit(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: edgefirst_msgs::Vibration<&mut [u8]> =
        match edgefirst_msgs::Vibration::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_unit(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the stamp field in place on a FoxgloveCompressedVideo buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_foxglove_compressed_video_set_stamp(
    buf: *mut u8,
    len: usize,
    sec: i32,
    nsec: u32,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: foxglove_msgs::FoxgloveCompressedVideo<&mut [u8]> =
        match foxglove_msgs::FoxgloveCompressedVideo::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_stamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Alias for `ros_foxglove_compressed_video_set_stamp`; matches the Foxglove schema field name.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub unsafe extern "C" fn ros_foxglove_compressed_video_set_timestamp(
    buf: *mut u8,
    len: usize,
    sec: i32,
    nsec: u32,
) -> i32 {
    ros_foxglove_compressed_video_set_stamp(buf, len, sec, nsec)
}

/// Set the timestamp field in place on a FoxgloveTextAnnotation buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_set_timestamp(
    buf: *mut u8,
    len: usize,
    sec: i32,
    nsec: u32,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: foxglove_msgs::FoxgloveTextAnnotation<&mut [u8]> =
        match foxglove_msgs::FoxgloveTextAnnotation::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_timestamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the position (FoxglovePoint2) field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_set_position(
    buf: *mut u8,
    len: usize,
    x: f64,
    y: f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: foxglove_msgs::FoxgloveTextAnnotation<&mut [u8]> =
        match foxglove_msgs::FoxgloveTextAnnotation::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_position(foxglove_msgs::FoxglovePoint2 { x, y }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the font_size field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_set_font_size(
    buf: *mut u8,
    len: usize,
    v: f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: foxglove_msgs::FoxgloveTextAnnotation<&mut [u8]> =
        match foxglove_msgs::FoxgloveTextAnnotation::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_font_size(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the text_color (FoxgloveColor) field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_set_text_color(
    buf: *mut u8,
    len: usize,
    r: f64,
    g: f64,
    b: f64,
    a: f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: foxglove_msgs::FoxgloveTextAnnotation<&mut [u8]> =
        match foxglove_msgs::FoxgloveTextAnnotation::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_text_color(foxglove_msgs::FoxgloveColor { r, g, b, a }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the background_color (FoxgloveColor) field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_foxglove_text_annotation_set_background_color(
    buf: *mut u8,
    len: usize,
    r: f64,
    g: f64,
    b: f64,
    a: f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: foxglove_msgs::FoxgloveTextAnnotation<&mut [u8]> =
        match foxglove_msgs::FoxgloveTextAnnotation::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_background_color(foxglove_msgs::FoxgloveColor { r, g, b, a }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the timestamp field in place on a FoxglovePointAnnotation buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_set_timestamp(
    buf: *mut u8,
    len: usize,
    sec: i32,
    nsec: u32,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: foxglove_msgs::FoxglovePointAnnotation<&mut [u8]> =
        match foxglove_msgs::FoxglovePointAnnotation::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_timestamp(Time::new(sec, nsec)) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the type_ field in place on a FoxglovePointAnnotation buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_set_type(buf: *mut u8, len: usize, v: u8) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: foxglove_msgs::FoxglovePointAnnotation<&mut [u8]> =
        match foxglove_msgs::FoxglovePointAnnotation::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_type_(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the outline_color (FoxgloveColor) field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_set_outline_color(
    buf: *mut u8,
    len: usize,
    r: f64,
    g: f64,
    b: f64,
    a: f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: foxglove_msgs::FoxglovePointAnnotation<&mut [u8]> =
        match foxglove_msgs::FoxglovePointAnnotation::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_outline_color(foxglove_msgs::FoxgloveColor { r, g, b, a }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the fill_color (FoxgloveColor) field in place.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_set_fill_color(
    buf: *mut u8,
    len: usize,
    r: f64,
    g: f64,
    b: f64,
    a: f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: foxglove_msgs::FoxglovePointAnnotation<&mut [u8]> =
        match foxglove_msgs::FoxglovePointAnnotation::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_fill_color(foxglove_msgs::FoxgloveColor { r, g, b, a }) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Set the thickness field in place on a FoxglovePointAnnotation buffer.
///
/// Returns 0 on success, -1 on error (errno: EINVAL for NULL buf,
/// EBADMSG if buf is not a valid encoded message of this type).
#[no_mangle]
pub extern "C" fn ros_foxglove_point_annotation_set_thickness(
    buf: *mut u8,
    len: usize,
    v: f64,
) -> i32 {
    if buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
    let mut m: foxglove_msgs::FoxglovePointAnnotation<&mut [u8]> =
        match foxglove_msgs::FoxglovePointAnnotation::from_cdr(slice) {
            Ok(v) => v,
            Err(_) => {
                set_errno(EBADMSG);
                return -1;
            }
        };
    match m.set_thickness(v) {
        Ok(()) => 0,
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}
