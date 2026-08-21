// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! C FFI for the composed tensor message family.
//!
//! `Tensor` is the payload; `TensorStamped` and `CameraFrame` are byte-identical
//! wrappers around it. The C API mirrors that composition rather than flattening
//! it: a wrapper hands out a borrowed `ros_tensor_t*`, and every tensor accessor
//! is written once and reused by both wrappers.
//!
//! ```c
//! ros_camera_frame_t *f = ros_camera_frame_from_cdr(buf, len);
//! const ros_tensor_t *t = ros_camera_frame_get_tensor(f);
//! uint32_t dtype = ros_tensor_get_dtype(t);
//! const ros_tensor_plane_t *p = ros_tensor_get_plane(t, 0);
//! int64_t handle = ros_tensor_plane_get_handle(p);
//! ros_camera_frame_free(f);   /* frees the borrowed tensor and planes too */
//! ```
//!
//! ## Ownership
//!
//! `ros_tensor_get_plane` and `ros_<wrapper>_get_tensor` return *borrowed*
//! pointers into the parent handle. They must not be freed, and they die with
//! the parent. Each child carries an `owned` flag so that a mistaken
//! `_free()` on a borrowed child is a no-op with `errno = EINVAL` rather than
//! a double free.
//!
//! Every handle also borrows the CDR bytes passed to `from_cdr`; that buffer
//! must outlive the handle.

// These repeat `ffi.rs`'s module-level attributes rather than relying on them.
// Rust propagates lint attributes into child modules, so the parent's would
// cover this file — but a reader (and SonarCloud's Rust analyser, which works
// file-by-file and does not model module nesting) should not have to know the
// parent's attributes to understand this one. The `not_unsafe_ptr_arg_deref`
// allow is the whole point of a C FFI boundary: every `pub extern "C" fn` here
// takes raw pointers from C and null-checks them before use, exactly as the
// ~900 entry points in `ffi.rs` do.
#![allow(non_camel_case_types)]
#![allow(clippy::not_unsafe_ptr_arg_deref)]

use super::*;

use crate::edgefirst_msgs::{CameraFrame, TensorStamped};
use crate::tensor::{Tensor, TensorFields, TensorPlaneView};

// =============================================================================
// Opaque handles
// =============================================================================

/// Opaque handle for a single `TensorPlane` view.
///
/// Always borrowed from a parent `ros_tensor_t` today; `owned` exists so that
/// `ros_tensor_plane_free` stays safe if a caller frees a borrowed child.
pub struct ros_tensor_plane_t {
    view: TensorPlaneView<'static>,
    owned: bool,
}

/// Opaque handle for a `Tensor` view, standalone or embedded in a wrapper.
///
/// Child planes are materialized once at parse time, mirroring the
/// `Detect` → `Box` pattern used elsewhere in this FFI.
pub struct ros_tensor_t {
    inner: Tensor<&'static [u8]>,
    child_planes: Vec<ros_tensor_plane_t>,
    owned: bool,
}

/// Materialize an FFI tensor handle, collecting its planes.
///
/// Uses `planes_borrowed`, whose views carry the buffer's lifetime rather than
/// a borrow of the `Tensor` — no lifetime widening, no `transmute`.
fn make_tensor(inner: Tensor<&'static [u8]>, owned: bool) -> ros_tensor_t {
    let child_planes = inner
        .planes_borrowed()
        .map(|view| ros_tensor_plane_t { view, owned: false })
        .collect();
    ros_tensor_t {
        inner,
        child_planes,
        owned,
    }
}

// =============================================================================
// Tensor (buffer-backed)
// =============================================================================

/// @brief Create a standalone Tensor view from CDR bytes.
/// @param data CDR encoded bytes (borrowed; must outlive the returned handle)
/// @param len Length of data
/// @return Opaque handle or NULL on error (errno set to EINVAL or EBADMSG)
#[no_mangle]
pub extern "C" fn ros_tensor_from_cdr(data: *const u8, len: usize) -> *mut ros_tensor_t {
    check_null_ret_null!(data);
    let slice = unsafe { slice::from_raw_parts(data, len) };
    match Tensor::from_cdr(unsafe { erase_lifetime(slice) }) {
        Ok(v) => Box::into_raw(Box::new(make_tensor(v, true))),
        Err(_) => {
            set_errno(EBADMSG);
            ptr::null_mut()
        }
    }
}

/// @brief Free a Tensor handle.
///
/// Safe to call with NULL. If the handle is parent-borrowed (via
/// `ros_tensor_stamped_get_tensor` / `ros_camera_frame_get_tensor`), this
/// no-ops with `errno = EINVAL` — the parent owns it.
#[no_mangle]
pub extern "C" fn ros_tensor_free(view: *mut ros_tensor_t) {
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

macro_rules! tensor_scalar_getter {
    ($name:ident, $method:ident, $ty:ty, $default:expr) => {
        #[no_mangle]
        pub extern "C" fn $name(view: *const ros_tensor_t) -> $ty {
            if view.is_null() {
                return $default;
            }
            unsafe { (*view).inner.$method() }
        }
    };
}

tensor_scalar_getter!(ros_tensor_get_storage_kind, storage_kind, u32, 0);
tensor_scalar_getter!(ros_tensor_get_pid, pid, u32, 0);
tensor_scalar_getter!(ros_tensor_get_fence_fd, fence_fd, i32, -1);
tensor_scalar_getter!(ros_tensor_get_dtype, dtype, u32, 0);
tensor_scalar_getter!(ros_tensor_get_quant_axis, quant_axis, i32, -2);
tensor_scalar_getter!(ros_tensor_get_num_planes, num_planes, u32, 0);

macro_rules! tensor_string_getter {
    ($name:ident, $method:ident) => {
        #[no_mangle]
        pub extern "C" fn $name(view: *const ros_tensor_t) -> *const c_char {
            if view.is_null() {
                return ptr::null();
            }
            str_as_c(unsafe { (*view).inner.$method() })
        }
    };
}

tensor_string_getter!(ros_tensor_get_format, format);
tensor_string_getter!(ros_tensor_get_color_space, color_space);
tensor_string_getter!(ros_tensor_get_color_transfer, color_transfer);
tensor_string_getter!(ros_tensor_get_color_encoding, color_encoding);
tensor_string_getter!(ros_tensor_get_color_range, color_range);

// ── shape / strides ─────────────────────────────────────────────────
//
// `shape` and `strides` cannot be returned as pointers into the CDR buffer.
// CDR 8-byte alignment is relative to the data start at absolute offset 4, so
// their elements sit at absolute offset ≡ 4 (mod 8) and never satisfy
// `alignof(uint64_t)`. Handing C a `const uint64_t*` there would invite
// misaligned loads that are undefined behaviour on strict-alignment targets
// and silently slow on others. The API is therefore index/copy based: element
// reads go through `from_le_bytes`, which is alignment-safe by construction.

#[no_mangle]
pub extern "C" fn ros_tensor_get_shape_len(view: *const ros_tensor_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).inner.shape_len() as u32 }
}

#[no_mangle]
pub extern "C" fn ros_tensor_get_strides_len(view: *const ros_tensor_t) -> u32 {
    if view.is_null() {
        return 0;
    }
    unsafe { (*view).inner.strides_len() as u32 }
}

/// @brief Read one `shape` element.
/// @param out Receives the dimension.
/// @return 0 on success, -1 on NULL/out-of-range (errno = EINVAL).
#[no_mangle]
pub extern "C" fn ros_tensor_get_shape_at(
    view: *const ros_tensor_t,
    index: u32,
    out: *mut u64,
) -> i32 {
    if view.is_null() || out.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    match unsafe { (*view).inner.shape_at(index as usize) } {
        Some(v) => {
            unsafe { *out = v };
            0
        }
        None => {
            set_errno(EINVAL);
            -1
        }
    }
}

/// @brief Read one `strides` element (in bytes).
/// @param out Receives the stride.
/// @return 0 on success, -1 on NULL/out-of-range (errno = EINVAL).
#[no_mangle]
pub extern "C" fn ros_tensor_get_strides_at(
    view: *const ros_tensor_t,
    index: u32,
    out: *mut i64,
) -> i32 {
    if view.is_null() || out.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    match unsafe { (*view).inner.strides_at(index as usize) } {
        Some(v) => {
            unsafe { *out = v };
            0
        }
        None => {
            set_errno(EINVAL);
            -1
        }
    }
}

/// @brief Copy the whole `shape` into a caller buffer.
/// @param out Destination array of at least `cap` elements.
/// @param cap Capacity of `out` in elements.
/// @return Number of elements written, or -1 on error (EINVAL for NULL,
///         ENOBUFS when `cap` is smaller than `shape_len`).
///
/// O(n) for the whole sequence — prefer this to calling
/// `ros_tensor_get_shape_at` in a loop, which rescans and is O(n²).
#[no_mangle]
pub extern "C" fn ros_tensor_copy_shape(
    view: *const ros_tensor_t,
    out: *mut u64,
    cap: usize,
) -> isize {
    if view.is_null() || (out.is_null() && cap > 0) {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*view).inner };
    let n = inner.shape_len();
    if cap < n {
        set_errno(ENOBUFS);
        return -1;
    }
    for (i, v) in inner.shape().enumerate() {
        unsafe { *out.add(i) = v };
    }
    n as isize
}

/// @brief Copy the whole `strides` sequence into a caller buffer.
/// @param out Destination array of at least `cap` elements.
/// @param cap Capacity of `out` in elements.
/// @return Number of elements written, or -1 on error (EINVAL for NULL,
///         ENOBUFS when `cap` is smaller than `strides_len`).
#[no_mangle]
pub extern "C" fn ros_tensor_copy_strides(
    view: *const ros_tensor_t,
    out: *mut i64,
    cap: usize,
) -> isize {
    if view.is_null() || (out.is_null() && cap > 0) {
        set_errno(EINVAL);
        return -1;
    }
    let inner = unsafe { &(*view).inner };
    let n = inner.strides_len();
    if cap < n {
        set_errno(ENOBUFS);
        return -1;
    }
    for (i, v) in inner.strides().enumerate() {
        unsafe { *out.add(i) = v };
    }
    n as isize
}

// ── quantization sequences ──────────────────────────────────────────
//
// Unlike shape/strides these are 4-byte elements, whose CDR alignment does
// coincide with the natural alignment, so they can be borrowed directly.

/// @brief Borrow the `quant_scales` sequence.
/// @param out_len Receives the element count (may be NULL).
/// @return Pointer into the CDR buffer, or NULL when empty.
#[no_mangle]
pub extern "C" fn ros_tensor_get_quant_scales(
    view: *const ros_tensor_t,
    out_len: *mut usize,
) -> *const f32 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe { *out_len = 0 };
        }
        return ptr::null();
    }
    let s = unsafe { (*view).inner.quant_scales() };
    if !out_len.is_null() {
        unsafe { *out_len = s.len() };
    }
    if s.is_empty() {
        ptr::null()
    } else {
        s.as_ptr()
    }
}

/// @brief Borrow the `quant_zero_points` sequence.
/// @param out_len Receives the element count (may be NULL).
/// @return Pointer into the CDR buffer, or NULL when empty.
#[no_mangle]
pub extern "C" fn ros_tensor_get_quant_zero_points(
    view: *const ros_tensor_t,
    out_len: *mut usize,
) -> *const i32 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe { *out_len = 0 };
        }
        return ptr::null();
    }
    let s = unsafe { (*view).inner.quant_zero_points() };
    if !out_len.is_null() {
        unsafe { *out_len = s.len() };
    }
    if s.is_empty() {
        ptr::null()
    } else {
        s.as_ptr()
    }
}

// ── planes ──────────────────────────────────────────────────────────

/// @brief Get a borrowed view of the i-th plane.
///
/// The returned pointer is NOT a separately-owned handle: do NOT pass it to
/// `ros_tensor_plane_free()`. It becomes invalid when the owning tensor (or
/// its parent wrapper) is freed, and the CDR buffer must remain valid too.
#[no_mangle]
pub extern "C" fn ros_tensor_get_plane(
    view: *const ros_tensor_t,
    index: u32,
) -> *const ros_tensor_plane_t {
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
    &v.child_planes[idx] as *const ros_tensor_plane_t
}

/// @brief This tensor's own bytes, from its base to the end of the buffer.
/// @param out_len Receives the byte length (may be NULL).
#[no_mangle]
pub extern "C" fn ros_tensor_get_tensor_bytes(
    view: *const ros_tensor_t,
    out_len: *mut usize,
) -> *const u8 {
    if view.is_null() {
        if !out_len.is_null() {
            unsafe { *out_len = 0 };
        }
        return ptr::null();
    }
    let b = unsafe { (*view).inner.tensor_bytes() };
    if !out_len.is_null() {
        unsafe { *out_len = b.len() };
    }
    b.as_ptr()
}

/// @brief Re-head an embedded tensor as a standalone Tensor CDR message.
///
/// Allocates; free with `ros_bytes_free`. Copies metadata only — plane
/// payloads stay behind their handles. This is the republish path.
/// @return 0 on success, -1 on error (errno = EINVAL).
#[no_mangle]
pub extern "C" fn ros_tensor_to_standalone_cdr(
    view: *const ros_tensor_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if view.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let cdr = unsafe { (*view).inner.to_standalone_cdr() };
    return_cdr_bytes(cdr, out_bytes, out_len)
}

// =============================================================================
// TensorPlane
// =============================================================================

/// @brief Free a TensorPlane handle.
///
/// Safe to call with NULL. Plane handles obtained from `ros_tensor_get_plane`
/// are parent-borrowed, so this no-ops with `errno = EINVAL`.
#[no_mangle]
pub extern "C" fn ros_tensor_plane_free(view: *mut ros_tensor_plane_t) {
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

macro_rules! plane_scalar_getter {
    ($name:ident, $field:ident, $ty:ty, $default:expr) => {
        #[no_mangle]
        pub extern "C" fn $name(view: *const ros_tensor_plane_t) -> $ty {
            if view.is_null() {
                return $default;
            }
            unsafe { (*view).view.$field }
        }
    };
}

plane_scalar_getter!(ros_tensor_plane_get_handle, handle, i64, -1);
plane_scalar_getter!(ros_tensor_plane_get_offset, offset, u64, 0);
plane_scalar_getter!(ros_tensor_plane_get_stride, stride, u64, 0);
plane_scalar_getter!(ros_tensor_plane_get_size, size, u64, 0);
plane_scalar_getter!(ros_tensor_plane_get_used, used, u64, 0);
plane_scalar_getter!(ros_tensor_plane_get_modifier, modifier, u64, 0);

/// @brief True when this plane's bytes travel inline rather than by handle.
#[no_mangle]
pub extern "C" fn ros_tensor_plane_is_inline(view: *const ros_tensor_plane_t) -> bool {
    if view.is_null() {
        return false;
    }
    unsafe { (*view).view.is_inline() }
}

macro_rules! plane_bytes_getter {
    ($name:ident, $field:ident, $doc:literal) => {
        #[doc = $doc]
        #[no_mangle]
        pub extern "C" fn $name(view: *const ros_tensor_plane_t, out_len: *mut usize) -> *const u8 {
            if view.is_null() {
                if !out_len.is_null() {
                    unsafe { *out_len = 0 };
                }
                return ptr::null();
            }
            let b = unsafe { (*view).view.$field };
            if !out_len.is_null() {
                unsafe { *out_len = b.len() };
            }
            if b.is_empty() {
                ptr::null()
            } else {
                b.as_ptr()
            }
        }
    };
}

plane_bytes_getter!(
    ros_tensor_plane_get_handle_bytes,
    handle_bytes,
    "@brief Opaque platform handle bytes (empty for an inline plane).\n@param out_len Receives the byte length (may be NULL)."
);
plane_bytes_getter!(
    ros_tensor_plane_get_data,
    data,
    "@brief Inlined plane bytes (only populated when handle == -1).\n@param out_len Receives the byte length (may be NULL)."
);

// =============================================================================
// Stamped wrappers — TensorStamped and CameraFrame
// =============================================================================
//
// The two wrappers are byte-identical, so their FFI is generated from one
// macro exactly as their Rust types are generated from `stamped_tensor_message!`.
// Keeping the generation shape aligned is deliberate: a field added to one
// wrapper cannot silently reach only one language surface.

macro_rules! stamped_tensor_ffi {
    (
        $handle:ident,
        $rust:ty,
        $from_cdr:ident, $free:ident,
        $get_stamp_sec:ident, $get_stamp_nanosec:ident,
        $get_frame_id:ident, $get_seq:ident,
        $get_tensor:ident, $get_cdr:ident,
        $set_stamp:ident, $set_seq:ident,
        $name:literal
    ) => {
        #[doc = concat!("Opaque handle for a `", $name, "` view.")]
        pub struct $handle {
            inner: $rust,
            child: ros_tensor_t,
        }

        #[doc = concat!("@brief Create a ", $name, " view from CDR bytes.\n",
                        "@param data CDR encoded bytes (borrowed; must outlive the handle)\n",
                        "@param len Length of data\n",
                        "@return Opaque handle or NULL on error (errno EINVAL or EBADMSG)")]
        #[no_mangle]
        pub extern "C" fn $from_cdr(data: *const u8, len: usize) -> *mut $handle {
            check_null_ret_null!(data);
            let slice = unsafe { slice::from_raw_parts(data, len) };
            match <$rust>::from_cdr(unsafe { erase_lifetime(slice) }) {
                Ok(inner) => {
                    // `tensor_borrowed` reuses the offset table built above —
                    // no rescan — and yields a tensor whose lifetime is the
                    // buffer's, which is what the child handle must store.
                    let child = make_tensor(inner.tensor_borrowed(), false);
                    Box::into_raw(Box::new($handle { inner, child }))
                }
                Err(_) => {
                    set_errno(EBADMSG);
                    ptr::null_mut()
                }
            }
        }

        #[doc = concat!("@brief Free a ", $name, " handle (also frees its borrowed tensor and planes).")]
        #[no_mangle]
        pub extern "C" fn $free(view: *mut $handle) {
            if !view.is_null() {
                unsafe {
                    drop(Box::from_raw(view));
                }
            }
        }

        #[no_mangle]
        pub extern "C" fn $get_stamp_sec(view: *const $handle) -> i32 {
            if view.is_null() {
                return 0;
            }
            unsafe { (*view).inner.stamp().sec }
        }

        #[no_mangle]
        pub extern "C" fn $get_stamp_nanosec(view: *const $handle) -> u32 {
            if view.is_null() {
                return 0;
            }
            unsafe { (*view).inner.stamp().nanosec }
        }

        #[no_mangle]
        pub extern "C" fn $get_frame_id(view: *const $handle) -> *const c_char {
            if view.is_null() {
                return ptr::null();
            }
            str_as_c(unsafe { (*view).inner.frame_id() })
        }

        #[no_mangle]
        pub extern "C" fn $get_seq(view: *const $handle) -> u64 {
            if view.is_null() {
                return 0;
            }
            unsafe { (*view).inner.seq() }
        }

        #[doc = concat!("@brief Borrowed view of the embedded tensor.\n\n",
                        "The returned pointer is owned by the parent: do NOT pass it to\n",
                        "`ros_tensor_free()`. It dies with the parent handle.")]
        #[no_mangle]
        pub extern "C" fn $get_tensor(view: *const $handle) -> *const ros_tensor_t {
            if view.is_null() {
                set_errno(EINVAL);
                return ptr::null();
            }
            unsafe { &(*view).child as *const ros_tensor_t }
        }

        #[doc = concat!("@brief Borrow the whole CDR buffer backing this ", $name, ".\n",
                        "@param out_len Receives the byte length (may be NULL).")]
        #[no_mangle]
        pub extern "C" fn $get_cdr(view: *const $handle, out_len: *mut usize) -> *const u8 {
            if view.is_null() {
                if !out_len.is_null() {
                    unsafe { *out_len = 0 };
                }
                return ptr::null();
            }
            let b = unsafe { (*view).inner.as_cdr() };
            if !out_len.is_null() {
                unsafe { *out_len = b.len() };
            }
            b.as_ptr()
        }

        #[doc = concat!("@brief In-place stamp update on a mutable ", $name, " CDR buffer.\n",
                        "@return 0 on success, -1 on error (errno EINVAL or EBADMSG).")]
        #[no_mangle]
        pub extern "C" fn $set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32 {
            if buf.is_null() {
                set_errno(EINVAL);
                return -1;
            }
            let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
            let mut v = match <$rust>::from_cdr(unsafe { erase_lifetime(slice) }) {
                Ok(v) => v.map_buffer(|_| unsafe { slice::from_raw_parts_mut(buf, len) }),
                Err(_) => {
                    set_errno(EBADMSG);
                    return -1;
                }
            };
            match v.set_stamp(Time::new(sec, nsec)) {
                Ok(()) => 0,
                Err(_) => {
                    set_errno(EBADMSG);
                    -1
                }
            }
        }

        #[doc = concat!("@brief In-place seq update on a mutable ", $name, " CDR buffer.\n",
                        "@return 0 on success, -1 on error (errno EINVAL or EBADMSG).")]
        #[no_mangle]
        pub extern "C" fn $set_seq(buf: *mut u8, len: usize, v: u64) -> i32 {
            if buf.is_null() {
                set_errno(EINVAL);
                return -1;
            }
            let slice = unsafe { slice::from_raw_parts_mut(buf, len) };
            let mut m = match <$rust>::from_cdr(unsafe { erase_lifetime(slice) }) {
                Ok(m) => m.map_buffer(|_| unsafe { slice::from_raw_parts_mut(buf, len) }),
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
    };
}

stamped_tensor_ffi!(
    ros_tensor_stamped_t,
    TensorStamped<&'static [u8]>,
    ros_tensor_stamped_from_cdr,
    ros_tensor_stamped_free,
    ros_tensor_stamped_get_stamp_sec,
    ros_tensor_stamped_get_stamp_nanosec,
    ros_tensor_stamped_get_frame_id,
    ros_tensor_stamped_get_seq,
    ros_tensor_stamped_get_tensor,
    ros_tensor_stamped_get_cdr,
    ros_tensor_stamped_set_stamp,
    ros_tensor_stamped_set_seq,
    "TensorStamped"
);

stamped_tensor_ffi!(
    ros_camera_frame_t,
    CameraFrame<&'static [u8]>,
    ros_camera_frame_from_cdr,
    ros_camera_frame_free,
    ros_camera_frame_get_stamp_sec,
    ros_camera_frame_get_stamp_nanosec,
    ros_camera_frame_get_frame_id,
    ros_camera_frame_get_seq,
    ros_camera_frame_get_tensor,
    ros_camera_frame_get_cdr,
    ros_camera_frame_set_stamp,
    ros_camera_frame_set_seq,
    "CameraFrame"
);

// =============================================================================
// Builders
// =============================================================================
//
// Following the convention used by every other builder in this FFI: scalars
// and strings are copied into the builder, sequences are stored as borrowed
// caller pointers and materialized at build/encode time. The caller's arrays
// must stay valid until the builder is finalized or freed.

/// C-POD descriptor for a single `TensorPlane` element.
///
/// `handle_bytes` and `data` are borrowed byte slices; both must remain valid
/// until the consuming builder is finalized (`build`/`encode_into`) or freed.
///
/// The two transport modes are exclusive and validated at build time:
/// * `handle >= 0` — bytes live behind the handle; `data` must be empty.
/// * `handle == -1` — bytes are inline in `data`; `size` must equal
///   `data_len`, `modifier` must be 0, and `handle_bytes` must be empty.
#[repr(C)]
pub struct ros_tensor_plane_elem_t {
    pub handle: i64,
    pub offset: u64,
    pub stride: u64,
    pub size: u64,
    pub used: u64,
    pub modifier: u64,
    pub handle_bytes: *const u8,
    pub handle_bytes_len: usize,
    pub data: *const u8,
    pub data_len: usize,
}

/// Materialize `TensorPlaneView` borrowers from the C-POD descriptor array.
///
/// # Safety
/// Each descriptor's `handle_bytes`/`data` pointers must be valid for their
/// stated lengths (or NULL when the length is 0), and the backing storage must
/// outlive the returned Vec.
unsafe fn tensor_plane_descs_to_views(
    descs: *const ros_tensor_plane_elem_t,
    count: usize,
) -> Vec<TensorPlaneView<'static>> {
    if descs.is_null() || count == 0 {
        return Vec::new();
    }
    let descs = slice::from_raw_parts(descs, count);
    descs
        .iter()
        .map(|d| TensorPlaneView {
            handle: d.handle,
            offset: d.offset,
            stride: d.stride,
            size: d.size,
            used: d.used,
            modifier: d.modifier,
            handle_bytes: if d.handle_bytes.is_null() || d.handle_bytes_len == 0 {
                &[][..]
            } else {
                slice::from_raw_parts(d.handle_bytes, d.handle_bytes_len)
            },
            data: if d.data.is_null() || d.data_len == 0 {
                &[][..]
            } else {
                slice::from_raw_parts(d.data, d.data_len)
            },
        })
        .collect()
}

struct TensorBuilderOwned {
    storage_kind: u32,
    pid: u32,
    fence_fd: i32,
    dtype: u32,
    quant_axis: i32,
    shape: *const u64,
    shape_len: usize,
    strides: *const i64,
    strides_len: usize,
    quant_scales: *const f32,
    quant_scales_len: usize,
    quant_zero_points: *const i32,
    quant_zero_points_len: usize,
    format: String,
    color_space: String,
    color_transfer: String,
    color_encoding: String,
    color_range: String,
    planes: *const ros_tensor_plane_elem_t,
    planes_count: usize,
}

impl TensorBuilderOwned {
    fn new() -> Self {
        // Mirrors `TensorFields::default()` — notably fence_fd = -1 (no fence)
        // and quant_axis = -2 (unquantized), which are NOT zero.
        Self {
            storage_kind: 0,
            pid: 0,
            fence_fd: -1,
            dtype: 0,
            quant_axis: -2,
            shape: ptr::null(),
            shape_len: 0,
            strides: ptr::null(),
            strides_len: 0,
            quant_scales: ptr::null(),
            quant_scales_len: 0,
            quant_zero_points: ptr::null(),
            quant_zero_points_len: 0,
            format: String::new(),
            color_space: String::new(),
            color_transfer: String::new(),
            color_encoding: String::new(),
            color_range: String::new(),
            planes: ptr::null(),
            planes_count: 0,
        }
    }

    /// Build a borrowed `TensorFields` over the caller's arrays.
    ///
    /// # Safety
    /// Every stored pointer must still be valid for its recorded length.
    unsafe fn fields<'a>(&'a self, planes: &'a [TensorPlaneView<'a>]) -> TensorFields<'a> {
        unsafe fn as_slice<'a, T>(p: *const T, n: usize) -> &'a [T] {
            if p.is_null() || n == 0 {
                &[]
            } else {
                slice::from_raw_parts(p, n)
            }
        }
        TensorFields {
            storage_kind: self.storage_kind,
            pid: self.pid,
            fence_fd: self.fence_fd,
            dtype: self.dtype,
            quant_axis: self.quant_axis,
            shape: as_slice(self.shape, self.shape_len),
            strides: as_slice(self.strides, self.strides_len),
            quant_scales: as_slice(self.quant_scales, self.quant_scales_len),
            quant_zero_points: as_slice(self.quant_zero_points, self.quant_zero_points_len),
            format: self.format.as_str().into(),
            color_space: self.color_space.as_str().into(),
            color_transfer: self.color_transfer.as_str().into(),
            color_encoding: self.color_encoding.as_str().into(),
            color_range: self.color_range.as_str().into(),
            planes,
        }
    }
}

pub struct ros_tensor_builder_t(TensorBuilderOwned);

#[no_mangle]
pub extern "C" fn ros_tensor_builder_new() -> *mut ros_tensor_builder_t {
    Box::into_raw(Box::new(ros_tensor_builder_t(TensorBuilderOwned::new())))
}

#[no_mangle]
pub extern "C" fn ros_tensor_builder_free(b: *mut ros_tensor_builder_t) {
    if !b.is_null() {
        unsafe {
            drop(Box::from_raw(b));
        }
    }
}

macro_rules! tensor_builder_scalar_setter {
    ($name:ident, $field:ident, $ty:ty) => {
        #[no_mangle]
        pub extern "C" fn $name(b: *mut ros_tensor_builder_t, v: $ty) {
            if b.is_null() {
                set_errno(EINVAL);
                return;
            }
            unsafe {
                (*b).0.$field = v;
            }
        }
    };
}

tensor_builder_scalar_setter!(ros_tensor_builder_set_storage_kind, storage_kind, u32);
tensor_builder_scalar_setter!(ros_tensor_builder_set_pid, pid, u32);
tensor_builder_scalar_setter!(ros_tensor_builder_set_fence_fd, fence_fd, i32);
tensor_builder_scalar_setter!(ros_tensor_builder_set_dtype, dtype, u32);
tensor_builder_scalar_setter!(ros_tensor_builder_set_quant_axis, quant_axis, i32);

macro_rules! tensor_builder_string_setter {
    ($name:ident, $field:ident) => {
        /// @return 0 on success, -1 on NULL/invalid UTF-8 (errno = EINVAL).
        #[no_mangle]
        pub extern "C" fn $name(b: *mut ros_tensor_builder_t, s: *const c_char) -> i32 {
            if b.is_null() {
                set_errno(EINVAL);
                return -1;
            }
            match unsafe { c_to_str_checked(s) } {
                Ok(v) => {
                    unsafe {
                        (*b).0.$field = v.to_string();
                    }
                    0
                }
                Err(()) => -1,
            }
        }
    };
}

tensor_builder_string_setter!(ros_tensor_builder_set_format, format);
tensor_builder_string_setter!(ros_tensor_builder_set_color_space, color_space);
tensor_builder_string_setter!(ros_tensor_builder_set_color_transfer, color_transfer);
tensor_builder_string_setter!(ros_tensor_builder_set_color_encoding, color_encoding);
tensor_builder_string_setter!(ros_tensor_builder_set_color_range, color_range);

macro_rules! tensor_builder_seq_setter {
    ($name:ident, $ptr_field:ident, $len_field:ident, $ty:ty, $doc:literal) => {
        #[doc = $doc]
        #[no_mangle]
        pub extern "C" fn $name(b: *mut ros_tensor_builder_t, v: *const $ty, len: usize) -> i32 {
            if b.is_null() || (v.is_null() && len > 0) {
                set_errno(EINVAL);
                return -1;
            }
            unsafe {
                (*b).0.$ptr_field = v;
                (*b).0.$len_field = len;
            }
            0
        }
    };
}

tensor_builder_seq_setter!(
    ros_tensor_builder_set_shape,
    shape,
    shape_len,
    u64,
    "@brief Set the addressing grid. Borrowed until build/free.\n\n@note `shape` is the addressing grid, NOT the byte layout: an NV12 frame\ncarries shape [h, w] with dtype U8 against an h*w*3/2 allocation. It is\ndeliberately never validated against any buffer size.\n@return 0 on success, -1 on error (errno = EINVAL)."
);
tensor_builder_seq_setter!(
    ros_tensor_builder_set_strides,
    strides,
    strides_len,
    i64,
    "@brief Set strides, in BYTES (not elements). Borrowed until build/free.\n\nEither empty, or exactly as long as `shape`.\n@return 0 on success, -1 on error (errno = EINVAL)."
);
tensor_builder_seq_setter!(
    ros_tensor_builder_set_quant_scales,
    quant_scales,
    quant_scales_len,
    f32,
    "@brief Set quantization scales. Borrowed until build/free.\n@return 0 on success, -1 on error (errno = EINVAL)."
);
tensor_builder_seq_setter!(
    ros_tensor_builder_set_quant_zero_points,
    quant_zero_points,
    quant_zero_points_len,
    i32,
    "@brief Set quantization zero points. Borrowed until build/free.\n@return 0 on success, -1 on error (errno = EINVAL)."
);

/// @brief Set the plane array. Borrowed until build/free.
/// @return 0 on success, -1 on error (errno = EINVAL).
#[no_mangle]
pub extern "C" fn ros_tensor_builder_set_planes(
    b: *mut ros_tensor_builder_t,
    planes: *const ros_tensor_plane_elem_t,
    count: usize,
) -> i32 {
    if b.is_null() || (planes.is_null() && count > 0) {
        set_errno(EINVAL);
        return -1;
    }
    unsafe {
        (*b).0.planes = planes;
        (*b).0.planes_count = count;
    }
    0
}

/// @brief Encode a standalone Tensor message.
///
/// Allocates; free with `ros_bytes_free`.
/// @return 0 on success, -1 on error (errno = EINVAL for NULL, EBADMSG when
///         the plane set fails validation).
#[no_mangle]
pub extern "C" fn ros_tensor_builder_build(
    b: *mut ros_tensor_builder_t,
    out_bytes: *mut *mut u8,
    out_len: *mut usize,
) -> i32 {
    if b.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let owned = unsafe { &(*b).0 };
    let planes = unsafe { tensor_plane_descs_to_views(owned.planes, owned.planes_count) };
    let fields = unsafe { owned.fields(&planes) };
    let mut builder = Tensor::builder();
    apply_fields(&mut builder, &fields);
    match builder.build() {
        Ok(v) => return_cdr_bytes(v.as_cdr().to_vec(), out_bytes, out_len),
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// @brief Encode a standalone Tensor message into a caller-provided buffer.
/// @param out_written Receives the number of bytes written (may be NULL).
/// @return 0 on success, -1 on error (errno = EINVAL, ENOBUFS or EBADMSG).
#[no_mangle]
pub extern "C" fn ros_tensor_builder_encode_into(
    b: *mut ros_tensor_builder_t,
    buf: *mut u8,
    cap: usize,
    out_written: *mut usize,
) -> i32 {
    if b.is_null() || buf.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    let owned = unsafe { &(*b).0 };
    let planes = unsafe { tensor_plane_descs_to_views(owned.planes, owned.planes_count) };
    let fields = unsafe { owned.fields(&planes) };
    let mut builder = Tensor::builder();
    apply_fields(&mut builder, &fields);
    let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
    match builder.encode_into_slice(dst) {
        Ok(n) => {
            if !out_written.is_null() {
                unsafe { *out_written = n };
            }
            0
        }
        Err(cdr::CdrError::BufferTooShort { .. }) => {
            set_errno(ENOBUFS);
            -1
        }
        Err(_) => {
            set_errno(EBADMSG);
            -1
        }
    }
}

/// Copy a borrowed `TensorFields` into a `TensorBuilder`.
///
/// `TensorBuilder` is the only public path to `build`/`encode_into_slice`, and
/// it takes fields one setter at a time; this keeps the FFI's single
/// descriptor-to-fields conversion in one place rather than repeating the
/// seventeen setter calls at each call site.
fn apply_fields<'a>(b: &mut crate::tensor::TensorBuilder<'a>, f: &TensorFields<'a>) {
    b.storage_kind(f.storage_kind)
        .pid(f.pid)
        .fence_fd(f.fence_fd)
        .dtype(f.dtype)
        .quant_axis(f.quant_axis)
        .shape(f.shape)
        .strides(f.strides)
        .quant_scales(f.quant_scales)
        .quant_zero_points(f.quant_zero_points)
        .format(f.format.clone())
        .color_space(f.color_space.clone())
        .color_transfer(f.color_transfer.clone())
        .color_encoding(f.color_encoding.clone())
        .color_range(f.color_range.clone())
        .planes(f.planes);
}

// ── stamped wrapper builders ────────────────────────────────────────
//
// A wrapper builder carries the header fields plus a pointer to a
// `ros_tensor_builder_t` holding the payload. Composing the two builders
// mirrors the message composition, and means the seventeen tensor setters
// exist once rather than once per wrapper.

struct StampedBuilderOwned {
    stamp_sec: i32,
    stamp_nanosec: u32,
    frame_id: String,
    seq: u64,
    tensor: *const ros_tensor_builder_t,
}

impl StampedBuilderOwned {
    fn new() -> Self {
        Self {
            stamp_sec: 0,
            stamp_nanosec: 0,
            frame_id: String::new(),
            seq: 0,
            tensor: ptr::null(),
        }
    }
}

macro_rules! stamped_tensor_builder_ffi {
    (
        $handle:ident, $rust:ty,
        $new:ident, $free:ident,
        $set_stamp:ident, $set_frame_id:ident, $set_seq:ident, $set_tensor:ident,
        $build:ident, $encode_into:ident,
        $name:literal
    ) => {
        #[doc = concat!("Opaque builder for `", $name, "`.")]
        pub struct $handle(StampedBuilderOwned);

        #[no_mangle]
        pub extern "C" fn $new() -> *mut $handle {
            Box::into_raw(Box::new($handle(StampedBuilderOwned::new())))
        }

        #[no_mangle]
        pub extern "C" fn $free(b: *mut $handle) {
            if !b.is_null() {
                unsafe {
                    drop(Box::from_raw(b));
                }
            }
        }

        #[no_mangle]
        pub extern "C" fn $set_stamp(b: *mut $handle, sec: i32, nsec: u32) {
            if b.is_null() {
                set_errno(EINVAL);
                return;
            }
            unsafe {
                (*b).0.stamp_sec = sec;
                (*b).0.stamp_nanosec = nsec;
            }
        }

        /// @return 0 on success, -1 on NULL/invalid UTF-8 (errno = EINVAL).
        #[no_mangle]
        pub extern "C" fn $set_frame_id(b: *mut $handle, s: *const c_char) -> i32 {
            if b.is_null() {
                set_errno(EINVAL);
                return -1;
            }
            match unsafe { c_to_str_checked(s) } {
                Ok(v) => {
                    unsafe {
                        (*b).0.frame_id = v.to_string();
                    }
                    0
                }
                Err(()) => -1,
            }
        }

        #[no_mangle]
        pub extern "C" fn $set_seq(b: *mut $handle, v: u64) {
            if b.is_null() {
                set_errno(EINVAL);
                return;
            }
            unsafe {
                (*b).0.seq = v;
            }
        }

        #[doc = concat!("@brief Attach the tensor payload.\n\n",
                        "The tensor builder is BORROWED: it must outlive this builder's\n",
                        "`build`/`encode_into` call and must not be freed before then.\n",
                        "@return 0 on success, -1 on error (errno = EINVAL).")]
        #[no_mangle]
        pub extern "C" fn $set_tensor(b: *mut $handle, t: *const ros_tensor_builder_t) -> i32 {
            if b.is_null() || t.is_null() {
                set_errno(EINVAL);
                return -1;
            }
            unsafe {
                (*b).0.tensor = t;
            }
            0
        }

        #[doc = concat!("@brief Encode a ", $name, " message.\n\n",
                        "Allocates; free with `ros_bytes_free`.\n",
                        "@return 0 on success, -1 on error (errno = EINVAL when no tensor\n",
                        "        payload was attached, EBADMSG on validation failure).")]
        #[no_mangle]
        pub extern "C" fn $build(
            b: *mut $handle,
            out_bytes: *mut *mut u8,
            out_len: *mut usize,
        ) -> i32 {
            if b.is_null() {
                set_errno(EINVAL);
                return -1;
            }
            let owned = unsafe { &(*b).0 };
            if owned.tensor.is_null() {
                set_errno(EINVAL);
                return -1;
            }
            let t = unsafe { &(*owned.tensor).0 };
            let planes = unsafe { tensor_plane_descs_to_views(t.planes, t.planes_count) };
            let fields = unsafe { t.fields(&planes) };
            let mut builder = <$rust>::builder();
            builder
                .stamp(Time::new(owned.stamp_sec, owned.stamp_nanosec))
                .frame_id(owned.frame_id.as_str())
                .seq(owned.seq)
                .tensor(&fields);
            match builder.build() {
                Ok(v) => return_cdr_bytes(v.to_cdr(), out_bytes, out_len),
                Err(_) => {
                    set_errno(EBADMSG);
                    -1
                }
            }
        }

        #[doc = concat!("@brief Encode a ", $name, " into a caller-provided buffer.\n",
                        "@param out_written Receives the byte count (may be NULL).\n",
                        "@return 0 on success, -1 on error (errno = EINVAL, ENOBUFS or EBADMSG).")]
        #[no_mangle]
        pub extern "C" fn $encode_into(
            b: *mut $handle,
            buf: *mut u8,
            cap: usize,
            out_written: *mut usize,
        ) -> i32 {
            if b.is_null() || buf.is_null() {
                set_errno(EINVAL);
                return -1;
            }
            let owned = unsafe { &(*b).0 };
            if owned.tensor.is_null() {
                set_errno(EINVAL);
                return -1;
            }
            let t = unsafe { &(*owned.tensor).0 };
            let planes = unsafe { tensor_plane_descs_to_views(t.planes, t.planes_count) };
            let fields = unsafe { t.fields(&planes) };
            let mut builder = <$rust>::builder();
            builder
                .stamp(Time::new(owned.stamp_sec, owned.stamp_nanosec))
                .frame_id(owned.frame_id.as_str())
                .seq(owned.seq)
                .tensor(&fields);
            let dst = unsafe { slice::from_raw_parts_mut(buf, cap) };
            match builder.encode_into_slice(dst) {
                Ok(n) => {
                    if !out_written.is_null() {
                        unsafe { *out_written = n };
                    }
                    0
                }
                Err(cdr::CdrError::BufferTooShort { .. }) => {
                    set_errno(ENOBUFS);
                    -1
                }
                Err(_) => {
                    set_errno(EBADMSG);
                    -1
                }
            }
        }
    };
}

stamped_tensor_builder_ffi!(
    ros_tensor_stamped_builder_t,
    TensorStamped<Vec<u8>>,
    ros_tensor_stamped_builder_new,
    ros_tensor_stamped_builder_free,
    ros_tensor_stamped_builder_set_stamp,
    ros_tensor_stamped_builder_set_frame_id,
    ros_tensor_stamped_builder_set_seq,
    ros_tensor_stamped_builder_set_tensor,
    ros_tensor_stamped_builder_build,
    ros_tensor_stamped_builder_encode_into,
    "TensorStamped"
);

stamped_tensor_builder_ffi!(
    ros_camera_frame_builder_t,
    CameraFrame<Vec<u8>>,
    ros_camera_frame_builder_new,
    ros_camera_frame_builder_free,
    ros_camera_frame_builder_set_stamp,
    ros_camera_frame_builder_set_frame_id,
    ros_camera_frame_builder_set_seq,
    ros_camera_frame_builder_set_tensor,
    ros_camera_frame_builder_build,
    ros_camera_frame_builder_encode_into,
    "CameraFrame"
);
