// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Python bindings for `edgefirst-schemas`.
//!
// pyo3 `#[new]` constructors often exceed 7 args and `to_bytes(&self)` on
// Copy types is the expected pyo3 pattern. Silence these crate-wide.
#![allow(clippy::too_many_arguments, clippy::wrong_self_convention)]
//! The compiled binary IS the `edgefirst.schemas` package — no Python
//! wrapper layer, no `_native` indirection. Submodules
//! (`sensor_msgs`, `std_msgs`, `geometry_msgs`, `edgefirst_msgs`,
//! `foxglove_msgs`, `builtin_interfaces`, `rosgraph_msgs`) are
//! registered into `sys.modules` at module-init time so dotted imports
//! (`from edgefirst.schemas.sensor_msgs import Image`) resolve directly
//! to the in-binary objects.
//!
//! Heavy types accept any object implementing the buffer protocol
//! (`bytes`, `bytearray`, `memoryview`, `numpy.ndarray`, `array.array`)
//! for their bulk payload via [`PyBuffer<u8>`]. Phase 1 copies the payload
//! into the CDR buffer once with the GIL released; phase 2 will add a
//! `borrow=True` opt-in for true zero-copy via composite buffers.

#[cfg(any(not(Py_LIMITED_API), Py_3_11))]
use std::ffi::{c_int, c_void, CString};
// `c_char` is only used inside the buffer-protocol gates; the cfg
// keeps it from triggering an unused-import warning on abi3-py38.
#[cfg(any(not(Py_LIMITED_API), Py_3_11))]
use std::os::raw::c_char;

use edgefirst_schemas::builtin_interfaces::{Duration, Time};
use edgefirst_schemas::cdr::{decode_fixed, encode_fixed};
#[allow(deprecated)]
use edgefirst_schemas::edgefirst_msgs::DmaBuffer;
use edgefirst_schemas::edgefirst_msgs::{
    CameraFrame, CameraPlaneView, Date, Detect, DetectBoxView, LocalTime, Mask, MaskView, Model,
    ModelInfo, RadarCube, RadarInfo, Track, Vibration,
};
use edgefirst_schemas::foxglove_msgs::{
    FoxgloveCircleAnnotations, FoxgloveColor, FoxgloveCompressedVideo, FoxgloveImageAnnotation,
    FoxglovePoint2, FoxglovePointAnnotation, FoxglovePointAnnotationView, FoxgloveTextAnnotation,
    FoxgloveTextAnnotationView,
};
use edgefirst_schemas::geometry_msgs::{
    Accel, AccelStamped, Inertia, InertiaStamped, Point, Point32, PointStamped, Pose, Pose2D,
    PoseWithCovariance, Quaternion, Transform, TransformStamped, Twist, TwistStamped,
    TwistWithCovariance, Vector3,
};
use edgefirst_schemas::nav_msgs::Odometry;
use edgefirst_schemas::rosgraph_msgs::Clock;
use edgefirst_schemas::sensor_msgs::{
    BatteryState, CameraInfo, CompressedImage, FluidPressure, Image, Imu, MagneticField, NavSatFix,
    NavSatStatus, PointCloud2, PointFieldView, RegionOfInterest, Temperature,
};
use edgefirst_schemas::std_msgs::{ColorRGBA, Header};

#[cfg(any(not(Py_LIMITED_API), Py_3_11))]
use pyo3::exceptions::PyBufferError;
use pyo3::exceptions::PyValueError;
#[cfg(any(not(Py_LIMITED_API), Py_3_11))]
use pyo3::ffi::{self, Py_buffer};
use pyo3::prelude::*;
use pyo3::types::{PyBytes, PyType};

// ── Error mapping ────────────────────────────────────────────────────

fn map_cdr_err(e: edgefirst_schemas::cdr::CdrError) -> PyErr {
    PyValueError::new_err(format!("CDR error: {e}"))
}

/// RAII guard for a Python buffer view.
///
/// On non-`abi3` builds and `abi3-py311` the buffer is acquired via
/// `PyObject_GetBuffer` (zero-copy: the slice points directly at the
/// source object's memory and the guard releases it on drop). On
/// `abi3-py38` (where `Py_buffer` isn't in the limited API) we fall back
/// to extracting an owned `Vec<u8>` and exposing its pointer; the slice
/// then points into the guard's owned storage. Either way the
/// `(*const u8, usize)` returned by `buffer_as_bytes()` stays valid for
/// the lifetime of the guard.
#[cfg(any(not(Py_LIMITED_API), Py_3_11))]
pub struct BufferGuard {
    view: Py_buffer,
    acquired: bool,
}

#[cfg(any(not(Py_LIMITED_API), Py_3_11))]
impl BufferGuard {
    fn ptr(&self) -> *const u8 {
        self.view.buf as *const u8
    }
    fn len(&self) -> usize {
        self.view.len as usize
    }
}

#[cfg(any(not(Py_LIMITED_API), Py_3_11))]
impl Drop for BufferGuard {
    fn drop(&mut self) {
        if self.acquired {
            unsafe { ffi::PyBuffer_Release(&mut self.view) };
        }
    }
}

/// Owned-storage variant of `BufferGuard` used on `abi3-py38` where the
/// buffer protocol isn't in the limited API. Holds the bytes alive for
/// the duration of the borrow.
#[cfg(all(Py_LIMITED_API, not(Py_3_11)))]
pub struct BufferGuard {
    _owned: Vec<u8>,
}

/// Acquire a contiguous byte view over a Python buffer-like object.
///
/// Accepts:
///   * `bytes` / `bytearray` — zero-copy on non-abi3, single copy on abi3-py38.
///   * `memoryview` — zero-copy on non-abi3 / abi3-py311; iterated copy on abi3-py38.
///   * `numpy.ndarray` of any dtype — zero-copy on non-abi3 / abi3-py311
///     (the byte payload is exposed regardless of element type); copy on abi3-py38.
///
/// On abi3-py38 the implementation goes through `Vec<u8>::extract()`,
/// which `pyo3` implements over `bytes`/`bytearray` / sequence protocol.
/// Multi-dtype numpy support on abi3-py38 requires the user to call
/// `arr.tobytes()` first.
#[cfg(any(not(Py_LIMITED_API), Py_3_11))]
fn buffer_as_bytes(obj: &Bound<'_, PyAny>) -> PyResult<(BufferGuard, *const u8, usize)> {
    let mut guard = BufferGuard {
        view: unsafe { std::mem::zeroed() },
        acquired: false,
    };
    let rc = unsafe {
        ffi::PyObject_GetBuffer(
            obj.as_ptr(),
            &mut guard.view as *mut Py_buffer,
            ffi::PyBUF_SIMPLE,
        )
    };
    if rc != 0 {
        return Err(PyErr::fetch(obj.py()));
    }
    guard.acquired = true;
    let ptr = guard.ptr();
    let len = guard.len();
    Ok((guard, ptr, len))
}

#[cfg(all(Py_LIMITED_API, not(Py_3_11)))]
fn buffer_as_bytes(obj: &Bound<'_, PyAny>) -> PyResult<(BufferGuard, *const u8, usize)> {
    // Typed numpy / array.array inputs would otherwise be silently
    // truncated: `Vec<u8>::extract` iterates the sequence and casts each
    // element to `u8`, so `np.array([1,2,3], dtype=np.uint16)` produces
    // 3 bytes instead of 6. The shape of the output looks plausible until
    // a value > 255 raises an OverflowError.
    //
    // We detect this BEFORE extract by checking the `itemsize` attribute
    // (numpy ndarrays, array.array, and memoryview all expose it; bytes
    // and bytearray don't). Anything with itemsize > 1 is rejected with
    // a clear message pointing at `arr.tobytes()` as the portable path.
    if let Ok(itemsize_attr) = obj.getattr("itemsize") {
        if let Ok(itemsize) = itemsize_attr.extract::<usize>() {
            if itemsize > 1 {
                return Err(PyValueError::new_err(format!(
                    "input has itemsize={} bytes per element; abi3-py38 builds \
                     reject typed buffers (Py_buffer is not in the limited API \
                     at this floor). Pass `arr.tobytes()` for typed numpy \
                     arrays, or rebuild with `--no-default-features --features \
                     abi3-py311` for the zero-copy typed-buffer path.",
                    itemsize
                )));
            }
        }
    }
    let owned: Vec<u8> = obj.extract().map_err(|e| {
        PyValueError::new_err(format!("expected bytes / bytearray / sequence of u8: {e}"))
    })?;
    let ptr = owned.as_ptr();
    let len = owned.len();
    Ok((BufferGuard { _owned: owned }, ptr, len))
}

// ── BorrowedBuf — buffer-protocol view with parent lifetime ─────────

/// A read-only byte view that exposes a slice of a parent message's CDR
/// buffer via the Python buffer protocol. Lifetime is tied to the parent
/// through the `obj` field of `Py_buffer`, which Python's buffer machinery
/// keeps alive for the duration of any consumer (numpy, memoryview, etc.).
#[pyclass(name = "BorrowedBuf", module = "edgefirst.schemas", frozen)]
pub struct BorrowedBuf {
    /// Owner kept alive while this view exists. `()` is a placeholder so
    /// instances constructed from raw pointers + an explicit Python parent
    /// stay sound; in practice the parent ref is held via the buffer
    /// protocol's `obj` slot once `__getbuffer__` runs.
    parent: Py<PyAny>,
    /// Stored as `usize` to be `Send` — reconstructed in `__getbuffer__`.
    ptr: usize,
    len: usize,
}

unsafe impl Send for BorrowedBuf {}
unsafe impl Sync for BorrowedBuf {}

impl BorrowedBuf {
    /// SAFETY: `ptr` must point to `len` bytes that remain valid as long
    /// as `parent` is alive. The parent must be the Python wrapper around
    /// the Rust message that owns the buffer.
    unsafe fn new(parent: Py<PyAny>, ptr: *const u8, len: usize) -> Self {
        Self {
            parent,
            ptr: ptr as usize,
            len,
        }
    }
}

#[pymethods]
impl BorrowedBuf {
    fn __len__(&self) -> usize {
        self.len
    }

    fn __repr__(&self) -> String {
        format!("BorrowedBuf(len={})", self.len)
    }

    /// Materialize a Python `bytes` copy of the view.
    fn tobytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        let slice = unsafe { std::slice::from_raw_parts(self.ptr as *const u8, self.len) };
        PyBytes::new(py, slice)
    }

    /// Return a memoryview-or-bytes view of the parent's bytes.
    ///
    /// On non-`abi3` builds and `abi3-py311`+ this returns a
    /// ``memoryview`` that aliases the parent's bytes (zero-copy). The
    /// memoryview's lifetime is anchored to the parent message via the
    /// buffer protocol's ``obj`` slot, so the parent stays alive for as
    /// long as the memoryview is held — even if the local
    /// ``BorrowedBuf`` reference is dropped first.
    ///
    /// On `abi3-py38` the buffer protocol isn't in the limited API, so
    /// there is no safe way to construct a parent-anchored memoryview.
    /// In that build mode `view()` returns a fresh ``bytes`` copy
    /// instead — one memcpy as the safety tradeoff for the older Python
    /// floor. The same call site works on both ABI levels: callers can
    /// always treat the result as a bytes-like object.
    #[cfg(any(not(Py_LIMITED_API), Py_3_11))]
    fn view(slf: Bound<'_, Self>) -> PyResult<Py<PyAny>> {
        // `PyMemoryView_FromObject` invokes our `__getbuffer__`, which
        // sets the `obj` slot to keep the parent alive. The resulting
        // memoryview holds a strong reference to this `BorrowedBuf`
        // (and therefore to the message that owns the bytes) for its
        // entire lifetime.
        let py = slf.py();
        let obj_ptr = slf.into_ptr();
        let mem = unsafe { pyo3::ffi::PyMemoryView_FromObject(obj_ptr) };
        // `PyMemoryView_FromObject` does NOT steal `obj_ptr`; release
        // our scratch reference now that the memoryview owns its own.
        unsafe { pyo3::ffi::Py_DECREF(obj_ptr) };
        unsafe { Py::<PyAny>::from_owned_ptr_or_err(py, mem) }
    }

    #[cfg(all(Py_LIMITED_API, not(Py_3_11)))]
    fn view<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        // abi3-py38 has no buffer protocol in the limited API and
        // `PyMemoryView_FromMemory` returns a memoryview without any
        // owner — that's a use-after-free waiting to happen if the
        // parent is collected before the memoryview. Return `bytes`
        // (one copy) instead, matching the call shape on Py 3.11+
        // (both branches return bytes-like objects).
        self.tobytes(py)
    }

    /// Strong reference to the parent message keeping this view valid.
    #[getter]
    fn parent(&self, py: Python<'_>) -> Py<PyAny> {
        self.parent.clone_ref(py)
    }

    /// `__getbuffer__` is part of the buffer protocol but not in the
    /// limited API until Python 3.11. When building with abi3-py38 the
    /// cfg below evaluates false and the slot is omitted; consumers
    /// must use ``tobytes()`` (one copy) or ``view()`` (also one copy
    /// on that ABI) for byte access. On non-abi3 / abi3-py311 builds
    /// this stays in and ``np.frombuffer(buf)`` / ``memoryview(buf)``
    /// work directly with proper parent lifetime anchoring.
    #[cfg(any(not(Py_LIMITED_API), Py_3_11))]
    unsafe fn __getbuffer__(
        slf: Bound<'_, Self>,
        view: *mut Py_buffer,
        _flags: c_int,
    ) -> PyResult<()> {
        if view.is_null() {
            return Err(PyBufferError::new_err("null buffer view"));
        }
        // Allocate state (format CString + shape array) before any
        // fallible op on `slf` — if `slf.into_ptr()` later fails, we
        // leak only the box, not a half-initialised Py_buffer.
        let inner = slf.borrow();
        let state = Box::new(BufferViewState {
            format: CString::new("B").unwrap(),
            shape: [inner.len as isize],
        });
        // `Box::into_raw` transfers ownership; `__releasebuffer__`
        // reclaims via `Box::from_raw` on the same pointer.
        let state_ptr = Box::into_raw(state);
        unsafe {
            (*view).buf = inner.ptr as *mut c_void;
            (*view).len = inner.len as isize;
            (*view).itemsize = 1;
            (*view).readonly = 1;
            (*view).ndim = 1;
            (*view).format = (*state_ptr).format.as_ptr() as *mut c_char;
            // PEP 3118: when `ndim >= 1`, `shape` MUST be a non-null
            // pointer to `ndim` `Py_ssize_t` values. NumPy's
            // `PyArray_FromBuffer` (and `np.frombuffer(...)`)
            // dereferences `shape[0]` unconditionally — leaving it
            // null would crash the consumer. We carry the storage in
            // the boxed `BufferViewState`.
            (*view).shape = (*state_ptr).shape.as_mut_ptr();
            // `strides == NULL` is valid for C-contiguous buffers per
            // PEP 3118; consumers compute strides from
            // `shape × itemsize`. No allocation needed.
            (*view).strides = std::ptr::null_mut();
            (*view).suboffsets = std::ptr::null_mut();
            (*view).internal = state_ptr as *mut c_void;
            (*view).obj = slf.into_ptr();
        }
        Ok(())
    }

    #[cfg(any(not(Py_LIMITED_API), Py_3_11))]
    unsafe fn __releasebuffer__(&self, view: *mut Py_buffer) {
        if view.is_null() {
            return;
        }
        let state_ptr = unsafe { (*view).internal as *mut BufferViewState };
        if !state_ptr.is_null() {
            // Reclaim the boxed CString + shape allocated in
            // `__getbuffer__`. Single drop handles both fields.
            drop(unsafe { Box::from_raw(state_ptr) });
            unsafe { (*view).internal = std::ptr::null_mut() };
        }
    }
}

/// Per-view state owned by `Py_buffer.internal` — one allocation
/// covers the format CString and the 1-element shape array. PEP 3118
/// requires `shape` to be a non-null pointer to `ndim` `Py_ssize_t`
/// values when `ndim >= 1`; this struct holds that storage while the
/// parent `BorrowedBuf` keeps the byte payload alive via
/// `Py_buffer.obj`.
#[cfg(any(not(Py_LIMITED_API), Py_3_11))]
struct BufferViewState {
    /// `as_ptr()` lifetime is tied to the box; `Py_buffer.format`
    /// reads the C string via that pointer for the consumer's lifetime.
    format: CString,
    /// 1-D byte buffer ⇒ shape is a single-element array `[len]`.
    shape: [isize; 1],
}

// ── Time / Duration ─────────────────────────────────────────────────

/// Common helper used by every CdrFixed pyclass for the round-trip
/// encode/decode pair so we don't repeat the boilerplate per type.
///
/// `T: Send` is required because the closure passed to `py.detach()` runs
/// without the GIL and the result has to cross that boundary; every
/// `CdrFixed` we wrap is `Copy + 'static`, so this bound is satisfied
/// trivially.
fn cdrfixed_decode<T: edgefirst_schemas::cdr::CdrFixed + Send>(
    py: Python<'_>,
    buf: &Bound<'_, PyAny>,
) -> PyResult<T> {
    let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
    let ptr_addr = ptr as usize;
    let val = py.detach(|| {
        let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
        decode_fixed::<T>(slice)
    });
    drop(pybuf);
    val.map_err(map_cdr_err)
}

fn cdrfixed_encode<'py, T: edgefirst_schemas::cdr::CdrFixed>(
    py: Python<'py>,
    val: &T,
) -> PyResult<Bound<'py, PyBytes>> {
    let bytes = encode_fixed(val).map_err(map_cdr_err)?;
    Ok(PyBytes::new(py, &bytes))
}

/// `builtin_interfaces.Time` — seconds (i32) + nanoseconds (u32).
#[pyclass(name = "Time", module = "edgefirst.schemas.builtin_interfaces", frozen)]
#[derive(Clone, Copy)]
pub struct PyTime(pub Time);

#[pymethods]
impl PyTime {
    #[new]
    #[pyo3(signature = (sec=0, nanosec=0))]
    fn new(sec: i32, nanosec: u32) -> Self {
        Self(Time { sec, nanosec })
    }

    #[staticmethod]
    fn from_nanos(nanos: u64) -> Self {
        Self(Time::from_nanos(nanos))
    }

    fn to_nanos(&self) -> Option<u64> {
        self.0.to_nanos()
    }

    #[getter]
    fn sec(&self) -> i32 {
        self.0.sec
    }
    #[getter]
    fn nanosec(&self) -> u32 {
        self.0.nanosec
    }

    /// Encode this `Time` as a standalone CDR buffer (header + 8 bytes).
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Time>(py, buf)?))
    }

    fn __repr__(&self) -> String {
        format!("Time(sec={}, nanosec={})", self.0.sec, self.0.nanosec)
    }

    fn __eq__(&self, other: &Self) -> bool {
        self.0 == other.0
    }

    fn __hash__(&self) -> u64 {
        ((self.0.sec as u64) << 32) | self.0.nanosec as u64
    }
}

/// `builtin_interfaces.Duration` — seconds (i32) + nanoseconds (u32).
#[pyclass(
    name = "Duration",
    module = "edgefirst.schemas.builtin_interfaces",
    frozen
)]
#[derive(Clone, Copy)]
pub struct PyDuration(pub Duration);

#[pymethods]
impl PyDuration {
    #[new]
    #[pyo3(signature = (sec=0, nanosec=0))]
    fn new(sec: i32, nanosec: u32) -> Self {
        Self(Duration { sec, nanosec })
    }

    #[getter]
    fn sec(&self) -> i32 {
        self.0.sec
    }
    #[getter]
    fn nanosec(&self) -> u32 {
        self.0.nanosec
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Duration>(py, buf)?))
    }

    fn __repr__(&self) -> String {
        format!("Duration(sec={}, nanosec={})", self.0.sec, self.0.nanosec)
    }

    fn __eq__(&self, other: &Self) -> bool {
        self.0 == other.0
    }

    // Hash matches PyTime's encoding so a (sec, nanosec) pair maps the
    // same way regardless of which type wraps it. Required for Python
    // hash/eq consistency: classes that define __eq__ without __hash__
    // are unhashable, breaking dict/set membership.
    fn __hash__(&self) -> u64 {
        ((self.0.sec as u64) << 32) | self.0.nanosec as u64
    }
}

// ── ColorRGBA (CdrFixed, std_msgs) ──────────────────────────────────

#[pyclass(name = "ColorRGBA", module = "edgefirst.schemas.std_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyColorRGBA(pub ColorRGBA);

#[pymethods]
impl PyColorRGBA {
    #[new]
    #[pyo3(signature = (r=0.0, g=0.0, b=0.0, a=0.0))]
    fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self(ColorRGBA { r, g, b, a })
    }

    #[getter]
    fn r(&self) -> f32 {
        self.0.r
    }
    #[getter]
    fn g(&self) -> f32 {
        self.0.g
    }
    #[getter]
    fn b(&self) -> f32 {
        self.0.b
    }
    #[getter]
    fn a(&self) -> f32 {
        self.0.a
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<ColorRGBA>(py, buf)?))
    }

    fn __repr__(&self) -> String {
        format!(
            "ColorRGBA(r={}, g={}, b={}, a={})",
            self.0.r, self.0.g, self.0.b, self.0.a
        )
    }
}

// ── geometry_msgs CdrFixed types ────────────────────────────────────

#[pyclass(name = "Vector3", module = "edgefirst.schemas.geometry_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyVector3(pub Vector3);

#[pymethods]
impl PyVector3 {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0))]
    fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Vector3 { x, y, z })
    }
    #[getter]
    fn x(&self) -> f64 {
        self.0.x
    }
    #[getter]
    fn y(&self) -> f64 {
        self.0.y
    }
    #[getter]
    fn z(&self) -> f64 {
        self.0.z
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Vector3>(py, buf)?))
    }
    fn __repr__(&self) -> String {
        format!("Vector3(x={}, y={}, z={})", self.0.x, self.0.y, self.0.z)
    }
}

#[pyclass(name = "Point", module = "edgefirst.schemas.geometry_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyPoint(pub Point);

#[pymethods]
impl PyPoint {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0))]
    fn new(x: f64, y: f64, z: f64) -> Self {
        Self(Point { x, y, z })
    }
    #[getter]
    fn x(&self) -> f64 {
        self.0.x
    }
    #[getter]
    fn y(&self) -> f64 {
        self.0.y
    }
    #[getter]
    fn z(&self) -> f64 {
        self.0.z
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Point>(py, buf)?))
    }
    fn __repr__(&self) -> String {
        format!("Point(x={}, y={}, z={})", self.0.x, self.0.y, self.0.z)
    }
}

#[pyclass(name = "Point32", module = "edgefirst.schemas.geometry_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyPoint32(pub Point32);

#[pymethods]
impl PyPoint32 {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0))]
    fn new(x: f32, y: f32, z: f32) -> Self {
        Self(Point32 { x, y, z })
    }
    #[getter]
    fn x(&self) -> f32 {
        self.0.x
    }
    #[getter]
    fn y(&self) -> f32 {
        self.0.y
    }
    #[getter]
    fn z(&self) -> f32 {
        self.0.z
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Point32>(py, buf)?))
    }
    fn __repr__(&self) -> String {
        format!("Point32(x={}, y={}, z={})", self.0.x, self.0.y, self.0.z)
    }
}

#[pyclass(
    name = "Quaternion",
    module = "edgefirst.schemas.geometry_msgs",
    frozen
)]
#[derive(Clone, Copy)]
pub struct PyQuaternion(pub Quaternion);

#[pymethods]
impl PyQuaternion {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, z=0.0, w=1.0))]
    fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self(Quaternion { x, y, z, w })
    }
    #[getter]
    fn x(&self) -> f64 {
        self.0.x
    }
    #[getter]
    fn y(&self) -> f64 {
        self.0.y
    }
    #[getter]
    fn z(&self) -> f64 {
        self.0.z
    }
    #[getter]
    fn w(&self) -> f64 {
        self.0.w
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Quaternion>(py, buf)?))
    }
    fn __repr__(&self) -> String {
        format!(
            "Quaternion(x={}, y={}, z={}, w={})",
            self.0.x, self.0.y, self.0.z, self.0.w
        )
    }
}

#[pyclass(name = "Pose2D", module = "edgefirst.schemas.geometry_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyPose2D(pub Pose2D);

#[pymethods]
impl PyPose2D {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0, theta=0.0))]
    fn new(x: f64, y: f64, theta: f64) -> Self {
        Self(Pose2D { x, y, theta })
    }
    #[getter]
    fn x(&self) -> f64 {
        self.0.x
    }
    #[getter]
    fn y(&self) -> f64 {
        self.0.y
    }
    #[getter]
    fn theta(&self) -> f64 {
        self.0.theta
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Pose2D>(py, buf)?))
    }
    fn __repr__(&self) -> String {
        format!(
            "Pose2D(x={}, y={}, theta={})",
            self.0.x, self.0.y, self.0.theta
        )
    }
}

#[pyclass(name = "Pose", module = "edgefirst.schemas.geometry_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyPose(pub Pose);

#[pymethods]
impl PyPose {
    #[new]
    #[pyo3(signature = (position=None, orientation=None))]
    fn new(position: Option<PyPoint>, orientation: Option<PyQuaternion>) -> Self {
        Self(Pose {
            position: position.map(|p| p.0).unwrap_or(Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
            orientation: orientation.map(|q| q.0).unwrap_or(Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            }),
        })
    }
    #[getter]
    fn position(&self) -> PyPoint {
        PyPoint(self.0.position)
    }
    #[getter]
    fn orientation(&self) -> PyQuaternion {
        PyQuaternion(self.0.orientation)
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Pose>(py, buf)?))
    }
    fn __repr__(&self) -> String {
        format!(
            "Pose(position={:?}, orientation={:?})",
            self.0.position, self.0.orientation
        )
    }
}

#[pyclass(name = "Transform", module = "edgefirst.schemas.geometry_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyTransform(pub Transform);

#[pymethods]
impl PyTransform {
    #[new]
    #[pyo3(signature = (translation=None, rotation=None))]
    fn new(translation: Option<PyVector3>, rotation: Option<PyQuaternion>) -> Self {
        Self(Transform {
            translation: translation.map(|v| v.0).unwrap_or(Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
            rotation: rotation.map(|q| q.0).unwrap_or(Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            }),
        })
    }
    #[getter]
    fn translation(&self) -> PyVector3 {
        PyVector3(self.0.translation)
    }
    #[getter]
    fn rotation(&self) -> PyQuaternion {
        PyQuaternion(self.0.rotation)
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Transform>(py, buf)?))
    }
}

#[pyclass(name = "Twist", module = "edgefirst.schemas.geometry_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyTwist(pub Twist);

#[pymethods]
impl PyTwist {
    #[new]
    #[pyo3(signature = (linear=None, angular=None))]
    fn new(linear: Option<PyVector3>, angular: Option<PyVector3>) -> Self {
        Self(Twist {
            linear: linear.map(|v| v.0).unwrap_or(Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
            angular: angular.map(|v| v.0).unwrap_or(Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
        })
    }
    #[getter]
    fn linear(&self) -> PyVector3 {
        PyVector3(self.0.linear)
    }
    #[getter]
    fn angular(&self) -> PyVector3 {
        PyVector3(self.0.angular)
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Twist>(py, buf)?))
    }
}

// ── Header (buffer-backed) ──────────────────────────────────────────

/// `std_msgs.Header` — `stamp` + `frame_id`. Backs onto an owned CDR
/// buffer; metadata fields read directly from the buffer at known offsets.
#[pyclass(name = "Header", module = "edgefirst.schemas.std_msgs", frozen)]
pub struct PyHeader {
    inner: Header<Vec<u8>>,
}

#[pymethods]
impl PyHeader {
    #[new]
    #[pyo3(signature = (stamp=None, frame_id=""))]
    fn new(stamp: Option<PyTime>, frame_id: &str) -> PyResult<Self> {
        let stamp = stamp.map(|t| t.0).unwrap_or(Time { sec: 0, nanosec: 0 });
        let inner = Header::builder()
            .stamp(stamp)
            .frame_id(frame_id)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    /// Decode an existing CDR buffer. Currently copies the buffer into a
    /// `Vec<u8>` (memcpy path); zero-copy borrow lands in phase 2.
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = Header::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }

    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }

    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.cdr_size()
    }

    /// Return the full CDR buffer as `bytes` (one copy).
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }

    fn __repr__(&self) -> String {
        format!(
            "Header(stamp={:?}, frame_id={:?})",
            self.inner.stamp(),
            self.inner.frame_id()
        )
    }
}

// ── Image (buffer-backed, canonical heavy type) ──────────────────────

/// `sensor_msgs.Image` — owned CDR buffer with O(1) field accessors.
///
/// Construction copies the bulk `data` payload once into the CDR buffer
/// (GIL released). Read-side `data()` returns a `BorrowedBuf` exposing the
/// in-buffer payload via the buffer protocol — zero-copy for `np.frombuffer`,
/// `memoryview`, etc.
#[pyclass(name = "Image", module = "edgefirst.schemas.sensor_msgs", frozen)]
pub struct PyImage {
    inner: Image<Vec<u8>>,
}

#[pymethods]
impl PyImage {
    #[new]
    #[pyo3(signature = (
        header,
        height,
        width,
        encoding,
        is_bigendian,
        step,
        data,
    ))]
    fn new(
        py: Python<'_>,
        header: &PyHeader,
        height: u32,
        width: u32,
        encoding: &str,
        is_bigendian: u8,
        step: u32,
        data: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(data)?;
        let ptr_addr = ptr as usize;
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let encoding_owned = encoding.to_string();

        // Build with GIL released: the only Python-managed memory we touch
        // is the source buffer (kept alive by `pybuf`) and the source frame_id
        // string (already cloned). The destination Vec<u8> is Rust-owned.
        let inner = py.detach(move || {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            Image::builder()
                .stamp(stamp)
                .frame_id(frame_id.as_str())
                .height(height)
                .width(width)
                .encoding(encoding_owned.as_str())
                .is_bigendian(is_bigendian)
                .step(step)
                .data(slice)
                .build()
        });
        drop(pybuf);
        let inner = inner.map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    /// Decode an existing CDR buffer. Phase 1 copies; phase 2 will offer a
    /// zero-copy borrow path.
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = Image::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn height(&self) -> u32 {
        self.inner.height()
    }
    #[getter]
    fn width(&self) -> u32 {
        self.inner.width()
    }
    #[getter]
    fn encoding(&self) -> &str {
        self.inner.encoding()
    }
    #[getter]
    fn is_bigendian(&self) -> u8 {
        self.inner.is_bigendian()
    }
    #[getter]
    fn step(&self) -> u32 {
        self.inner.step()
    }

    /// Zero-copy view of the pixel data backed by this Image's CDR buffer.
    /// Returns a `BorrowedBuf` that exposes the buffer protocol.
    #[getter]
    fn data(slf: Bound<'_, Self>) -> PyResult<Py<BorrowedBuf>> {
        let py = slf.py();
        let (ptr, len) = {
            let s = slf.borrow();
            let d = s.inner.data();
            (d.as_ptr(), d.len())
        };
        let parent: Py<PyAny> = slf.into_any().unbind();
        let view = unsafe { BorrowedBuf::new(parent, ptr, len) };
        Py::new(py, view)
    }

    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.cdr_size()
    }

    /// Return the full CDR buffer as `bytes` (one copy). Use this when
    /// publishing to a transport that requires owned bytes.
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }

    /// Zero-copy view of the full CDR buffer (header + payload).
    fn cdr_view(slf: Bound<'_, Self>) -> PyResult<Py<BorrowedBuf>> {
        let py = slf.py();
        let (ptr, len) = {
            let s = slf.borrow();
            let b = s.inner.as_cdr();
            (b.as_ptr(), b.len())
        };
        let parent: Py<PyAny> = slf.into_any().unbind();
        let view = unsafe { BorrowedBuf::new(parent, ptr, len) };
        Py::new(py, view)
    }

    fn __repr__(&self) -> String {
        format!(
            "Image(stamp={:?}, frame_id={:?}, height={}, width={}, encoding={:?}, step={}, data=<{} bytes>)",
            self.inner.stamp(),
            self.inner.frame_id(),
            self.inner.height(),
            self.inner.width(),
            self.inner.encoding(),
            self.inner.step(),
            self.inner.data().len(),
        )
    }
}

// ── Clock (rosgraph_msgs, CdrFixed) ─────────────────────────────────

/// `rosgraph_msgs.Clock` — a single `Time` value used for the ROS 2
/// /clock topic.
#[pyclass(name = "Clock", module = "edgefirst.schemas.rosgraph_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyClock(pub Clock);

#[pymethods]
impl PyClock {
    #[new]
    #[pyo3(signature = (clock=None))]
    fn new(clock: Option<PyTime>) -> Self {
        Self(Clock {
            clock: clock.map(|t| t.0).unwrap_or(Time { sec: 0, nanosec: 0 }),
        })
    }

    #[getter]
    fn clock(&self) -> PyTime {
        PyTime(self.0.clock)
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Clock>(py, buf)?))
    }

    fn __repr__(&self) -> String {
        format!(
            "Clock(clock=Time(sec={}, nanosec={}))",
            self.0.clock.sec, self.0.clock.nanosec
        )
    }
}

// ── NavSatStatus (sensor_msgs, CdrFixed) ────────────────────────────

/// `sensor_msgs.NavSatStatus` — GNSS fix status + service bitmask.
///
/// Status constants (per ROS 2 spec):
///     -1 = NO_FIX, 0 = FIX, 1 = SBAS_FIX, 2 = GBAS_FIX
///
/// Service bitmask:
///     0x01 = GPS, 0x02 = GLONASS, 0x04 = COMPASS, 0x08 = GALILEO
#[pyclass(
    name = "NavSatStatus",
    module = "edgefirst.schemas.sensor_msgs",
    frozen
)]
#[derive(Clone, Copy)]
pub struct PyNavSatStatus(pub NavSatStatus);

#[pymethods]
impl PyNavSatStatus {
    #[new]
    #[pyo3(signature = (status=0, service=0))]
    fn new(status: i8, service: u16) -> Self {
        Self(NavSatStatus { status, service })
    }

    #[getter]
    fn status(&self) -> i8 {
        self.0.status
    }
    #[getter]
    fn service(&self) -> u16 {
        self.0.service
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<NavSatStatus>(py, buf)?))
    }

    fn __repr__(&self) -> String {
        format!(
            "NavSatStatus(status={}, service={})",
            self.0.status, self.0.service
        )
    }
}

// ── RegionOfInterest (sensor_msgs, CdrFixed) ────────────────────────

/// `sensor_msgs.RegionOfInterest` — sub-window of a camera image.
#[pyclass(
    name = "RegionOfInterest",
    module = "edgefirst.schemas.sensor_msgs",
    frozen
)]
#[derive(Clone, Copy)]
pub struct PyRegionOfInterest(pub RegionOfInterest);

#[pymethods]
impl PyRegionOfInterest {
    #[new]
    #[pyo3(signature = (x_offset=0, y_offset=0, height=0, width=0, do_rectify=false))]
    fn new(x_offset: u32, y_offset: u32, height: u32, width: u32, do_rectify: bool) -> Self {
        Self(RegionOfInterest {
            x_offset,
            y_offset,
            height,
            width,
            do_rectify,
        })
    }

    #[getter]
    fn x_offset(&self) -> u32 {
        self.0.x_offset
    }
    #[getter]
    fn y_offset(&self) -> u32 {
        self.0.y_offset
    }
    #[getter]
    fn height(&self) -> u32 {
        self.0.height
    }
    #[getter]
    fn width(&self) -> u32 {
        self.0.width
    }
    #[getter]
    fn do_rectify(&self) -> bool {
        self.0.do_rectify
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<RegionOfInterest>(py, buf)?))
    }

    fn __repr__(&self) -> String {
        format!(
            "RegionOfInterest(x_offset={}, y_offset={}, height={}, width={}, do_rectify={})",
            self.0.x_offset, self.0.y_offset, self.0.height, self.0.width, self.0.do_rectify
        )
    }
}

// ── Date (edgefirst_msgs, CdrFixed) ─────────────────────────────────

/// `edgefirst_msgs.Date` — calendar date used by `LocalTime`.
#[pyclass(name = "Date", module = "edgefirst.schemas.edgefirst_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyDate(pub Date);

#[pymethods]
impl PyDate {
    #[new]
    #[pyo3(signature = (year=1970, month=1, day=1))]
    fn new(year: u16, month: u8, day: u8) -> Self {
        Self(Date { year, month, day })
    }

    #[getter]
    fn year(&self) -> u16 {
        self.0.year
    }
    #[getter]
    fn month(&self) -> u8 {
        self.0.month
    }
    #[getter]
    fn day(&self) -> u8 {
        self.0.day
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Date>(py, buf)?))
    }

    fn __repr__(&self) -> String {
        format!(
            "Date(year={}, month={}, day={})",
            self.0.year, self.0.month, self.0.day
        )
    }
}

// ── CompressedImage (sensor_msgs, buffer-backed) ────────────────────

/// `sensor_msgs.CompressedImage` — JPEG/PNG/etc. encoded image.
#[pyclass(
    name = "CompressedImage",
    module = "edgefirst.schemas.sensor_msgs",
    frozen
)]
pub struct PyCompressedImage {
    inner: CompressedImage<Vec<u8>>,
}

#[pymethods]
impl PyCompressedImage {
    #[new]
    #[pyo3(signature = (header, format, data))]
    fn new(
        py: Python<'_>,
        header: &PyHeader,
        format: &str,
        data: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(data)?;
        let ptr_addr = ptr as usize;
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let format_owned = format.to_string();

        let inner = py.detach(move || {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            CompressedImage::builder()
                .stamp(stamp)
                .frame_id(frame_id.as_str())
                .format(format_owned.as_str())
                .data(slice)
                .build()
        });
        drop(pybuf);
        let inner = inner.map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = CompressedImage::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn format(&self) -> &str {
        self.inner.format()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.cdr_size()
    }

    /// Zero-copy view of the encoded image bytes.
    #[getter]
    fn data(slf: Bound<'_, Self>) -> PyResult<Py<BorrowedBuf>> {
        let py = slf.py();
        let (ptr, len) = {
            let s = slf.borrow();
            let d = s.inner.data();
            (d.as_ptr(), d.len())
        };
        let parent: Py<PyAny> = slf.into_any().unbind();
        let view = unsafe { BorrowedBuf::new(parent, ptr, len) };
        Py::new(py, view)
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }

    fn __repr__(&self) -> String {
        format!(
            "CompressedImage(stamp={:?}, frame_id={:?}, format={:?}, data=<{} bytes>)",
            self.inner.stamp(),
            self.inner.frame_id(),
            self.inner.format(),
            self.inner.data().len(),
        )
    }
}

// ── Mask (edgefirst_msgs, buffer-backed) ────────────────────────────

/// `edgefirst_msgs.Mask` — segmentation mask: HxWxL bytes plus encoding
/// (e.g. "" or "zstd") and a `boxed` flag.
#[pyclass(name = "Mask", module = "edgefirst.schemas.edgefirst_msgs", frozen)]
pub struct PyMask {
    inner: Mask<Vec<u8>>,
}

#[pymethods]
impl PyMask {
    #[new]
    #[pyo3(signature = (height, width, length, encoding, mask, boxed=false))]
    fn new(
        py: Python<'_>,
        height: u32,
        width: u32,
        length: u32,
        encoding: &str,
        mask: &Bound<'_, PyAny>,
        boxed: bool,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(mask)?;
        let ptr_addr = ptr as usize;
        let encoding_owned = encoding.to_string();

        let inner = py.detach(move || {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            Mask::builder()
                .height(height)
                .width(width)
                .length(length)
                .encoding(encoding_owned.as_str())
                .mask(slice)
                .boxed(boxed)
                .build()
        });
        drop(pybuf);
        let inner = inner.map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = Mask::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn height(&self) -> u32 {
        self.inner.height()
    }
    #[getter]
    fn width(&self) -> u32 {
        self.inner.width()
    }
    #[getter]
    fn length(&self) -> u32 {
        self.inner.length()
    }
    #[getter]
    fn encoding(&self) -> &str {
        self.inner.encoding()
    }
    #[getter]
    fn boxed(&self) -> bool {
        self.inner.boxed()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.cdr_size()
    }

    /// Zero-copy view of the mask bytes. For typed mask data, wrap with
    /// `np.frombuffer(mask.mask, dtype=np.uint8).reshape(L, H, W)`.
    #[getter]
    fn mask(slf: Bound<'_, Self>) -> PyResult<Py<BorrowedBuf>> {
        let py = slf.py();
        let (ptr, len) = {
            let s = slf.borrow();
            let d = s.inner.mask_data();
            (d.as_ptr(), d.len())
        };
        let parent: Py<PyAny> = slf.into_any().unbind();
        let view = unsafe { BorrowedBuf::new(parent, ptr, len) };
        Py::new(py, view)
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }

    fn __repr__(&self) -> String {
        format!(
            "Mask(height={}, width={}, length={}, encoding={:?}, boxed={}, mask=<{} bytes>)",
            self.inner.height(),
            self.inner.width(),
            self.inner.length(),
            self.inner.encoding(),
            self.inner.boxed(),
            self.inner.mask_data().len(),
        )
    }
}

// ── DmaBuffer (edgefirst_msgs, buffer-backed; deprecated) ───────────

/// `edgefirst_msgs.DmaBuffer` — DMA-buf reference (header + 7 small
/// metadata fields). Deprecated upstream in favour of `CameraFrame`,
/// kept here for API parity with pycdr2 benches.
#[pyclass(
    name = "DmaBuffer",
    module = "edgefirst.schemas.edgefirst_msgs",
    frozen
)]
#[allow(deprecated)]
pub struct PyDmaBuffer {
    inner: DmaBuffer<Vec<u8>>,
}

#[pymethods]
#[allow(deprecated)]
impl PyDmaBuffer {
    #[new]
    #[pyo3(signature = (header, pid, fd, width, height, stride, fourcc, length))]
    fn new(
        header: &PyHeader,
        pid: u32,
        fd: i32,
        width: u32,
        height: u32,
        stride: u32,
        fourcc: u32,
        length: u32,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id();
        let inner = DmaBuffer::new(
            stamp, frame_id, pid, fd, width, height, stride, fourcc, length,
        )
        .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = DmaBuffer::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn pid(&self) -> u32 {
        self.inner.pid()
    }
    #[getter]
    fn fd(&self) -> i32 {
        self.inner.fd()
    }
    #[getter]
    fn width(&self) -> u32 {
        self.inner.width()
    }
    #[getter]
    fn height(&self) -> u32 {
        self.inner.height()
    }
    #[getter]
    fn stride(&self) -> u32 {
        self.inner.stride()
    }
    #[getter]
    fn fourcc(&self) -> u32 {
        self.inner.fourcc()
    }
    #[getter]
    fn length(&self) -> u32 {
        self.inner.length()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── RadarCube (edgefirst_msgs, buffer-backed) ───────────────────────

/// `edgefirst_msgs.RadarCube` — the bulk i16 cube plus typed metadata
/// arrays (layout u8, shape u16, scales f32). Bulk accessors return
/// `BorrowedBuf` views; reinterpret with the documented dtype:
///
/// - `cube` → `np.frombuffer(rc.cube, dtype=np.int16)`
/// - `shape` → `np.frombuffer(rc.shape, dtype=np.uint16)`
/// - `scales` → `np.frombuffer(rc.scales, dtype=np.float32)`
/// - `layout` → `bytes(rc.layout)` (single bytes; small)
#[pyclass(
    name = "RadarCube",
    module = "edgefirst.schemas.edgefirst_msgs",
    frozen
)]
pub struct PyRadarCube {
    inner: RadarCube<Vec<u8>>,
}

#[pymethods]
impl PyRadarCube {
    #[new]
    #[pyo3(signature = (
        header,
        timestamp,
        layout,
        shape,
        scales,
        cube,
        is_complex=false,
    ))]
    fn new(
        py: Python<'_>,
        header: &PyHeader,
        timestamp: u64,
        layout: &Bound<'_, PyAny>,
        shape: &Bound<'_, PyAny>,
        scales: &Bound<'_, PyAny>,
        cube: &Bound<'_, PyAny>,
        is_complex: bool,
    ) -> PyResult<Self> {
        let (lbuf, lptr, llen) = buffer_as_bytes(layout)?;
        let (sbuf, sptr, slen_bytes) = buffer_as_bytes(shape)?;
        let (scbuf, scptr, sclen_bytes) = buffer_as_bytes(scales)?;
        let (cbuf, cptr, clen_bytes) = buffer_as_bytes(cube)?;

        if slen_bytes % 2 != 0 {
            return Err(PyValueError::new_err(
                "shape buffer length must be a multiple of 2 (uint16)",
            ));
        }
        if sclen_bytes % 4 != 0 {
            return Err(PyValueError::new_err(
                "scales buffer length must be a multiple of 4 (float32)",
            ));
        }
        if clen_bytes % 2 != 0 {
            return Err(PyValueError::new_err(
                "cube buffer length must be a multiple of 2 (int16)",
            ));
        }

        let lptr_addr = lptr as usize;
        let sptr_addr = sptr as usize;
        let scptr_addr = scptr as usize;
        let cptr_addr = cptr as usize;
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let shape_count = slen_bytes / 2;
        let scales_count = sclen_bytes / 4;
        let cube_count = clen_bytes / 2;

        let inner = py.detach(move || {
            let layout_slice = unsafe { std::slice::from_raw_parts(lptr_addr as *const u8, llen) };
            let shape_slice =
                unsafe { std::slice::from_raw_parts(sptr_addr as *const u16, shape_count) };
            let scales_slice =
                unsafe { std::slice::from_raw_parts(scptr_addr as *const f32, scales_count) };
            let cube_slice =
                unsafe { std::slice::from_raw_parts(cptr_addr as *const i16, cube_count) };
            RadarCube::builder()
                .stamp(stamp)
                .frame_id(frame_id.as_str())
                .timestamp(timestamp)
                .layout(layout_slice)
                .shape(shape_slice)
                .scales(scales_slice)
                .cube(cube_slice)
                .is_complex(is_complex)
                .build()
        });
        drop(lbuf);
        drop(sbuf);
        drop(scbuf);
        drop(cbuf);
        let inner = inner.map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = RadarCube::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn timestamp(&self) -> u64 {
        self.inner.timestamp()
    }
    #[getter]
    fn is_complex(&self) -> bool {
        self.inner.is_complex()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }

    /// Zero-copy view of the layout bytes (uint8 sequence).
    #[getter]
    fn layout(slf: Bound<'_, Self>) -> PyResult<Py<BorrowedBuf>> {
        let py = slf.py();
        let (ptr, len) = {
            let s = slf.borrow();
            let d = s.inner.layout();
            (d.as_ptr(), d.len())
        };
        let parent: Py<PyAny> = slf.into_any().unbind();
        let view = unsafe { BorrowedBuf::new(parent, ptr, len) };
        Py::new(py, view)
    }

    /// Zero-copy view of the shape array (interpret as uint16).
    #[getter]
    fn shape(slf: Bound<'_, Self>) -> PyResult<Py<BorrowedBuf>> {
        let py = slf.py();
        let (ptr, len) = {
            let s = slf.borrow();
            let arr = s.inner.shape();
            (arr.as_ptr() as *const u8, arr.len() * 2)
        };
        let parent: Py<PyAny> = slf.into_any().unbind();
        let view = unsafe { BorrowedBuf::new(parent, ptr, len) };
        Py::new(py, view)
    }

    /// Zero-copy view of the scales array (interpret as float32).
    #[getter]
    fn scales(slf: Bound<'_, Self>) -> PyResult<Py<BorrowedBuf>> {
        let py = slf.py();
        let (ptr, len) = {
            let s = slf.borrow();
            let arr = s.inner.scales();
            (arr.as_ptr() as *const u8, arr.len() * 4)
        };
        let parent: Py<PyAny> = slf.into_any().unbind();
        let view = unsafe { BorrowedBuf::new(parent, ptr, len) };
        Py::new(py, view)
    }

    /// Zero-copy view of the cube data (interpret as int16).
    #[getter]
    fn cube(slf: Bound<'_, Self>) -> PyResult<Py<BorrowedBuf>> {
        let py = slf.py();
        let (ptr, len) = {
            let s = slf.borrow();
            let raw = s.inner.cube_raw();
            (raw.as_ptr(), raw.len())
        };
        let parent: Py<PyAny> = slf.into_any().unbind();
        let view = unsafe { BorrowedBuf::new(parent, ptr, len) };
        Py::new(py, view)
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }

    fn __repr__(&self) -> String {
        format!(
            "RadarCube(stamp={:?}, frame_id={:?}, timestamp={}, shape={:?}, is_complex={})",
            self.inner.stamp(),
            self.inner.frame_id(),
            self.inner.timestamp(),
            self.inner.shape(),
            self.inner.is_complex(),
        )
    }
}

// ── PointCloud2 (sensor_msgs, buffer-backed) ────────────────────────

/// `sensor_msgs.PointField` — Python-side dataclass-style holder for one
/// field descriptor used when constructing a PointCloud2. Carries the
/// owned `name` so it survives builder borrow lifetimes.
#[pyclass(name = "PointField", module = "edgefirst.schemas.sensor_msgs", frozen)]
#[derive(Clone)]
pub struct PyPointField {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

#[pymethods]
impl PyPointField {
    #[new]
    #[pyo3(signature = (name="", offset=0, datatype=0, count=1))]
    fn new(name: &str, offset: u32, datatype: u8, count: u32) -> Self {
        Self {
            name: name.to_string(),
            offset,
            datatype,
            count,
        }
    }

    #[getter]
    fn name(&self) -> &str {
        &self.name
    }
    #[getter]
    fn offset(&self) -> u32 {
        self.offset
    }
    #[getter]
    fn datatype(&self) -> u8 {
        self.datatype
    }
    #[getter]
    fn count(&self) -> u32 {
        self.count
    }

    fn __repr__(&self) -> String {
        format!(
            "PointField(name={:?}, offset={}, datatype={}, count={})",
            self.name, self.offset, self.datatype, self.count
        )
    }
}

#[pyclass(name = "PointCloud2", module = "edgefirst.schemas.sensor_msgs", frozen)]
pub struct PyPointCloud2 {
    inner: PointCloud2<Vec<u8>>,
}

#[pymethods]
impl PyPointCloud2 {
    #[new]
    #[pyo3(signature = (
        header,
        height,
        width,
        fields,
        is_bigendian,
        point_step,
        row_step,
        data,
        is_dense=true,
    ))]
    fn new(
        py: Python<'_>,
        header: &PyHeader,
        height: u32,
        width: u32,
        fields: Vec<PyPointField>,
        is_bigendian: bool,
        point_step: u32,
        row_step: u32,
        data: &Bound<'_, PyAny>,
        is_dense: bool,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(data)?;
        let ptr_addr = ptr as usize;
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();

        // Build PointFieldView slice; names borrow from fields' owned strings.
        let field_views: Vec<PointFieldView<'_>> = fields
            .iter()
            .map(|f| PointFieldView {
                name: f.name.as_str(),
                offset: f.offset,
                datatype: f.datatype,
                count: f.count,
            })
            .collect();

        let field_views_ptr = field_views.as_ptr() as usize;
        let field_views_len = field_views.len();

        let inner = py.detach(move || {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            let fields_slice = unsafe {
                std::slice::from_raw_parts(
                    field_views_ptr as *const PointFieldView<'_>,
                    field_views_len,
                )
            };
            PointCloud2::builder()
                .stamp(stamp)
                .frame_id(frame_id.as_str())
                .height(height)
                .width(width)
                .fields(fields_slice)
                .is_bigendian(is_bigendian)
                .point_step(point_step)
                .row_step(row_step)
                .data(slice)
                .is_dense(is_dense)
                .build()
        });
        drop(pybuf);
        // Keep `fields` alive past the closure boundary: Vec dropped here.
        drop(field_views);
        drop(fields);
        let inner = inner.map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = PointCloud2::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn height(&self) -> u32 {
        self.inner.height()
    }
    #[getter]
    fn width(&self) -> u32 {
        self.inner.width()
    }
    #[getter]
    fn is_bigendian(&self) -> bool {
        self.inner.is_bigendian()
    }
    #[getter]
    fn point_step(&self) -> u32 {
        self.inner.point_step()
    }
    #[getter]
    fn row_step(&self) -> u32 {
        self.inner.row_step()
    }
    #[getter]
    fn is_dense(&self) -> bool {
        self.inner.is_dense()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }

    /// PointField descriptors as a list of `PointField` Python objects.
    #[getter]
    fn fields(&self) -> Vec<PyPointField> {
        self.inner
            .fields_iter()
            .map(|f| PyPointField {
                name: f.name.to_string(),
                offset: f.offset,
                datatype: f.datatype,
                count: f.count,
            })
            .collect()
    }

    /// Zero-copy view of the point data (interpret per the field layout).
    #[getter]
    fn data(slf: Bound<'_, Self>) -> PyResult<Py<BorrowedBuf>> {
        let py = slf.py();
        let (ptr, len) = {
            let s = slf.borrow();
            let d = s.inner.data();
            (d.as_ptr(), d.len())
        };
        let parent: Py<PyAny> = slf.into_any().unbind();
        let view = unsafe { BorrowedBuf::new(parent, ptr, len) };
        Py::new(py, view)
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }

    fn __repr__(&self) -> String {
        format!(
            "PointCloud2(stamp={:?}, frame_id={:?}, height={}, width={}, point_step={}, data=<{} bytes>)",
            self.inner.stamp(),
            self.inner.frame_id(),
            self.inner.height(),
            self.inner.width(),
            self.inner.point_step(),
            self.inner.data().len(),
        )
    }
}

// ── FoxgloveCompressedVideo (foxglove_msgs) ─────────────────────────

/// `foxglove_msgs.CompressedVideo` — H.264/H.265 NAL units plus a format
/// string and timestamp.
#[pyclass(
    name = "CompressedVideo",
    module = "edgefirst.schemas.foxglove_msgs",
    frozen
)]
pub struct PyFoxgloveCompressedVideo {
    inner: FoxgloveCompressedVideo<Vec<u8>>,
}

#[pymethods]
impl PyFoxgloveCompressedVideo {
    #[new]
    #[pyo3(signature = (timestamp, frame_id, data, format))]
    fn new(
        py: Python<'_>,
        timestamp: PyTime,
        frame_id: &str,
        data: &Bound<'_, PyAny>,
        format: &str,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(data)?;
        let ptr_addr = ptr as usize;
        let stamp = timestamp.0;
        let frame_id_owned = frame_id.to_string();
        let format_owned = format.to_string();

        let inner = py.detach(move || {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            FoxgloveCompressedVideo::builder()
                .stamp(stamp)
                .frame_id(frame_id_owned.as_str())
                .data(slice)
                .format(format_owned.as_str())
                .build()
        });
        drop(pybuf);
        let inner = inner.map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = FoxgloveCompressedVideo::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn timestamp(&self) -> PyTime {
        PyTime(self.inner.timestamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn format(&self) -> &str {
        self.inner.format()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.cdr_size()
    }

    /// Zero-copy view of the encoded video payload.
    #[getter]
    fn data(slf: Bound<'_, Self>) -> PyResult<Py<BorrowedBuf>> {
        let py = slf.py();
        let (ptr, len) = {
            let s = slf.borrow();
            let d = s.inner.data();
            (d.as_ptr(), d.len())
        };
        let parent: Py<PyAny> = slf.into_any().unbind();
        let view = unsafe { BorrowedBuf::new(parent, ptr, len) };
        Py::new(py, view)
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }

    fn __repr__(&self) -> String {
        format!(
            "CompressedVideo(timestamp={:?}, frame_id={:?}, format={:?}, data=<{} bytes>)",
            self.inner.timestamp(),
            self.inner.frame_id(),
            self.inner.format(),
            self.inner.data().len(),
        )
    }
}

// ── Geometry composite CdrFixed types ──────────────────────────────

/// Helper: convert `[f64; N]` to a Python list of floats.
fn array9_to_list(py: Python<'_>, arr: [f64; 9]) -> PyResult<Py<PyAny>> {
    use pyo3::types::PyList;
    Ok(PyList::new(py, arr.iter())?.into_any().unbind())
}

fn array36_to_list(py: Python<'_>, arr: [f64; 36]) -> PyResult<Py<PyAny>> {
    use pyo3::types::PyList;
    Ok(PyList::new(py, arr.iter())?.into_any().unbind())
}

/// Convert any sequence (list/tuple/numpy 1D float array) into a fixed-N
/// f64 array. Used by the Imu/NavSatFix/Pose covariance setters.
fn extract_array9(seq: Option<Vec<f64>>) -> PyResult<[f64; 9]> {
    let v = seq.unwrap_or_else(|| vec![0.0; 9]);
    if v.len() != 9 {
        return Err(PyValueError::new_err(format!(
            "expected 9-element sequence, got {}",
            v.len()
        )));
    }
    let mut out = [0.0_f64; 9];
    out.copy_from_slice(&v);
    Ok(out)
}

fn extract_array36(seq: Option<Vec<f64>>) -> PyResult<[f64; 36]> {
    let v = seq.unwrap_or_else(|| vec![0.0; 36]);
    if v.len() != 36 {
        return Err(PyValueError::new_err(format!(
            "expected 36-element sequence, got {}",
            v.len()
        )));
    }
    let mut out = [0.0_f64; 36];
    out.copy_from_slice(&v);
    Ok(out)
}

#[pyclass(name = "Accel", module = "edgefirst.schemas.geometry_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyAccel(pub Accel);

#[pymethods]
impl PyAccel {
    #[new]
    #[pyo3(signature = (linear=None, angular=None))]
    fn new(linear: Option<PyVector3>, angular: Option<PyVector3>) -> Self {
        Self(Accel {
            linear: linear.map(|v| v.0).unwrap_or(Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
            angular: angular.map(|v| v.0).unwrap_or(Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
        })
    }
    #[getter]
    fn linear(&self) -> PyVector3 {
        PyVector3(self.0.linear)
    }
    #[getter]
    fn angular(&self) -> PyVector3 {
        PyVector3(self.0.angular)
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Accel>(py, buf)?))
    }
}

#[pyclass(name = "Inertia", module = "edgefirst.schemas.geometry_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyInertia(pub Inertia);

#[pymethods]
impl PyInertia {
    #[new]
    #[pyo3(signature = (m=0.0, com=None, ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0))]
    fn new(
        m: f64,
        com: Option<PyVector3>,
        ixx: f64,
        ixy: f64,
        ixz: f64,
        iyy: f64,
        iyz: f64,
        izz: f64,
    ) -> Self {
        Self(Inertia {
            m,
            com: com.map(|v| v.0).unwrap_or(Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
            ixx,
            ixy,
            ixz,
            iyy,
            iyz,
            izz,
        })
    }
    #[getter]
    fn m(&self) -> f64 {
        self.0.m
    }
    #[getter]
    fn com(&self) -> PyVector3 {
        PyVector3(self.0.com)
    }
    #[getter]
    fn ixx(&self) -> f64 {
        self.0.ixx
    }
    #[getter]
    fn ixy(&self) -> f64 {
        self.0.ixy
    }
    #[getter]
    fn ixz(&self) -> f64 {
        self.0.ixz
    }
    #[getter]
    fn iyy(&self) -> f64 {
        self.0.iyy
    }
    #[getter]
    fn iyz(&self) -> f64 {
        self.0.iyz
    }
    #[getter]
    fn izz(&self) -> f64 {
        self.0.izz
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<Inertia>(py, buf)?))
    }
}

#[pyclass(
    name = "PoseWithCovariance",
    module = "edgefirst.schemas.geometry_msgs",
    frozen
)]
#[derive(Clone, Copy)]
pub struct PyPoseWithCovariance(pub PoseWithCovariance);

#[pymethods]
impl PyPoseWithCovariance {
    #[new]
    #[pyo3(signature = (pose=None, covariance=None))]
    fn new(pose: Option<PyPose>, covariance: Option<Vec<f64>>) -> PyResult<Self> {
        let pose = pose.map(|p| p.0).unwrap_or(Pose {
            position: Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        });
        Ok(Self(PoseWithCovariance {
            pose,
            covariance: extract_array36(covariance)?,
        }))
    }
    #[getter]
    fn pose(&self) -> PyPose {
        PyPose(self.0.pose)
    }
    #[getter]
    fn covariance(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        array36_to_list(py, self.0.covariance)
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<PoseWithCovariance>(py, buf)?))
    }
}

#[pyclass(
    name = "TwistWithCovariance",
    module = "edgefirst.schemas.geometry_msgs",
    frozen
)]
#[derive(Clone, Copy)]
pub struct PyTwistWithCovariance(pub TwistWithCovariance);

#[pymethods]
impl PyTwistWithCovariance {
    #[new]
    #[pyo3(signature = (twist=None, covariance=None))]
    fn new(twist: Option<PyTwist>, covariance: Option<Vec<f64>>) -> PyResult<Self> {
        let twist = twist.map(|t| t.0).unwrap_or(Twist {
            linear: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        });
        Ok(Self(TwistWithCovariance {
            twist,
            covariance: extract_array36(covariance)?,
        }))
    }
    #[getter]
    fn twist(&self) -> PyTwist {
        PyTwist(self.0.twist)
    }
    #[getter]
    fn covariance(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        array36_to_list(py, self.0.covariance)
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<TwistWithCovariance>(py, buf)?))
    }
}

// ── geometry_msgs stamped types (buffer-backed) ─────────────────────

macro_rules! stamped_boilerplate {
    ($py_name:ident, $rust_ty:ident, $py_str_name:literal, $mod_name:literal, $payload_getter:ident, $py_payload:ident, $payload_ty:ident, $new_fn:expr, $default_payload:expr) => {
        #[pyclass(name = $py_str_name, module = $mod_name, frozen)]
        pub struct $py_name {
            inner: $rust_ty<Vec<u8>>,
        }

        #[pymethods]
        impl $py_name {
            #[new]
            #[pyo3(signature = (header, $payload_getter=None))]
            fn new(header: &PyHeader, $payload_getter: Option<$py_payload>) -> PyResult<Self> {
                let stamp = header.inner.stamp();
                let frame_id = header.inner.frame_id().to_string();
                let payload = $payload_getter.map(|v| v.0).unwrap_or($default_payload);
                #[allow(deprecated)]
                let inner = $new_fn(stamp, &frame_id, payload).map_err(map_cdr_err)?;
                Ok(Self { inner })
            }

            #[classmethod]
            fn from_cdr(
                _cls: &Bound<'_, PyType>,
                py: Python<'_>,
                buf: &Bound<'_, PyAny>,
            ) -> PyResult<Self> {
                let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
                let ptr_addr = ptr as usize;
                let owned: Vec<u8> = py.detach(|| {
                    let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
                    slice.to_vec()
                });
                drop(pybuf);
                let inner = $rust_ty::from_cdr(owned).map_err(map_cdr_err)?;
                Ok(Self { inner })
            }

            #[getter]
            fn stamp(&self) -> PyTime {
                PyTime(self.inner.stamp())
            }
            #[getter]
            fn frame_id(&self) -> &str {
                self.inner.frame_id()
            }
            #[getter]
            fn $payload_getter(&self) -> $py_payload {
                $py_payload(self.inner.$payload_getter())
            }
            #[getter]
            fn cdr_size(&self) -> usize {
                self.inner.as_cdr().len()
            }
            fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
                Ok(PyBytes::new(py, self.inner.as_cdr()))
            }
        }
    };
}

stamped_boilerplate!(
    PyTwistStamped,
    TwistStamped,
    "TwistStamped",
    "edgefirst.schemas.geometry_msgs",
    twist,
    PyTwist,
    Twist,
    TwistStamped::new,
    Twist {
        linear: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        },
        angular: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        }
    }
);
stamped_boilerplate!(
    PyAccelStamped,
    AccelStamped,
    "AccelStamped",
    "edgefirst.schemas.geometry_msgs",
    accel,
    PyAccel,
    Accel,
    AccelStamped::new,
    Accel {
        linear: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        },
        angular: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        }
    }
);
stamped_boilerplate!(
    PyInertiaStamped,
    InertiaStamped,
    "InertiaStamped",
    "edgefirst.schemas.geometry_msgs",
    inertia,
    PyInertia,
    Inertia,
    InertiaStamped::new,
    Inertia {
        m: 0.0,
        com: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        },
        ixx: 0.0,
        ixy: 0.0,
        ixz: 0.0,
        iyy: 0.0,
        iyz: 0.0,
        izz: 0.0
    }
);
stamped_boilerplate!(
    PyPointStamped,
    PointStamped,
    "PointStamped",
    "edgefirst.schemas.geometry_msgs",
    point,
    PyPoint,
    Point,
    PointStamped::new,
    Point {
        x: 0.0,
        y: 0.0,
        z: 0.0
    }
);

// ── geometry_msgs.TransformStamped (two string fields) ──────────────

#[pyclass(
    name = "TransformStamped",
    module = "edgefirst.schemas.geometry_msgs",
    frozen
)]
pub struct PyTransformStamped {
    inner: TransformStamped<Vec<u8>>,
}

#[pymethods]
impl PyTransformStamped {
    #[new]
    #[pyo3(signature = (header, child_frame_id="", transform=None))]
    fn new(
        header: &PyHeader,
        child_frame_id: &str,
        transform: Option<PyTransform>,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let t = transform.map(|v| v.0).unwrap_or(Transform {
            translation: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            rotation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        });
        #[allow(deprecated)]
        let inner =
            TransformStamped::new(stamp, &frame_id, child_frame_id, t).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = TransformStamped::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn child_frame_id(&self) -> &str {
        self.inner.child_frame_id()
    }
    #[getter]
    fn transform(&self) -> PyTransform {
        PyTransform(self.inner.transform())
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── sensor_msgs.Imu (buffer-backed) ─────────────────────────────────

/// `sensor_msgs.Imu` — orientation + angular velocity + linear
/// acceleration, each with a 3×3 covariance.
#[pyclass(name = "Imu", module = "edgefirst.schemas.sensor_msgs", frozen)]
pub struct PyImu {
    inner: Imu<Vec<u8>>,
}

#[pymethods]
impl PyImu {
    #[new]
    #[pyo3(signature = (
        header,
        orientation=None,
        orientation_covariance=None,
        angular_velocity=None,
        angular_velocity_covariance=None,
        linear_acceleration=None,
        linear_acceleration_covariance=None,
    ))]
    fn new(
        header: &PyHeader,
        orientation: Option<PyQuaternion>,
        orientation_covariance: Option<Vec<f64>>,
        angular_velocity: Option<PyVector3>,
        angular_velocity_covariance: Option<Vec<f64>>,
        linear_acceleration: Option<PyVector3>,
        linear_acceleration_covariance: Option<Vec<f64>>,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        // Identity quaternion (w=1) — matches the default in PyPose,
        // PyTransform, PyOdometry, and the ROS 2 IMU convention. The
        // zero quaternion (w=0) isn't a rotation and using it as a
        // default would silently corrupt downstream consumers that
        // re-normalize.
        let q = orientation.map(|q| q.0).unwrap_or(Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        });
        let av = angular_velocity.map(|v| v.0).unwrap_or(Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        let la = linear_acceleration.map(|v| v.0).unwrap_or(Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        let oc = extract_array9(orientation_covariance)?;
        let avc = extract_array9(angular_velocity_covariance)?;
        let lac = extract_array9(linear_acceleration_covariance)?;
        let inner = Imu::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .orientation(q)
            .orientation_covariance(oc)
            .angular_velocity(av)
            .angular_velocity_covariance(avc)
            .linear_acceleration(la)
            .linear_acceleration_covariance(lac)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = Imu::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn orientation(&self) -> PyQuaternion {
        PyQuaternion(self.inner.orientation())
    }
    #[getter]
    fn orientation_covariance(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        array9_to_list(py, self.inner.orientation_covariance())
    }
    #[getter]
    fn angular_velocity(&self) -> PyVector3 {
        PyVector3(self.inner.angular_velocity())
    }
    #[getter]
    fn angular_velocity_covariance(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        array9_to_list(py, self.inner.angular_velocity_covariance())
    }
    #[getter]
    fn linear_acceleration(&self) -> PyVector3 {
        PyVector3(self.inner.linear_acceleration())
    }
    #[getter]
    fn linear_acceleration_covariance(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        array9_to_list(py, self.inner.linear_acceleration_covariance())
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── sensor_msgs.NavSatFix ───────────────────────────────────────────

#[pyclass(name = "NavSatFix", module = "edgefirst.schemas.sensor_msgs", frozen)]
pub struct PyNavSatFix {
    inner: NavSatFix<Vec<u8>>,
}

#[pymethods]
impl PyNavSatFix {
    #[new]
    #[pyo3(signature = (
        header,
        status=None,
        latitude=0.0,
        longitude=0.0,
        altitude=0.0,
        position_covariance=None,
        position_covariance_type=0,
    ))]
    fn new(
        header: &PyHeader,
        status: Option<PyNavSatStatus>,
        latitude: f64,
        longitude: f64,
        altitude: f64,
        position_covariance: Option<Vec<f64>>,
        position_covariance_type: u8,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let st = status.map(|s| s.0).unwrap_or(NavSatStatus {
            status: 0,
            service: 0,
        });
        let cov = extract_array9(position_covariance)?;
        let inner = NavSatFix::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .status(st)
            .latitude(latitude)
            .longitude(longitude)
            .altitude(altitude)
            .position_covariance(cov)
            .position_covariance_type(position_covariance_type)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = NavSatFix::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn status(&self) -> PyNavSatStatus {
        PyNavSatStatus(self.inner.status())
    }
    #[getter]
    fn latitude(&self) -> f64 {
        self.inner.latitude()
    }
    #[getter]
    fn longitude(&self) -> f64 {
        self.inner.longitude()
    }
    #[getter]
    fn altitude(&self) -> f64 {
        self.inner.altitude()
    }
    #[getter]
    fn position_covariance(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        array9_to_list(py, self.inner.position_covariance())
    }
    #[getter]
    fn position_covariance_type(&self) -> u8 {
        self.inner.position_covariance_type()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }

    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── sensor_msgs.MagneticField ───────────────────────────────────────

#[pyclass(
    name = "MagneticField",
    module = "edgefirst.schemas.sensor_msgs",
    frozen
)]
pub struct PyMagneticField {
    inner: MagneticField<Vec<u8>>,
}

#[pymethods]
impl PyMagneticField {
    #[new]
    #[pyo3(signature = (header, magnetic_field=None, magnetic_field_covariance=None))]
    fn new(
        header: &PyHeader,
        magnetic_field: Option<PyVector3>,
        magnetic_field_covariance: Option<Vec<f64>>,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let mf = magnetic_field.map(|v| v.0).unwrap_or(Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        let cov = extract_array9(magnetic_field_covariance)?;
        let inner = MagneticField::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .magnetic_field(mf)
            .magnetic_field_covariance(cov)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = MagneticField::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn magnetic_field(&self) -> PyVector3 {
        PyVector3(self.inner.magnetic_field())
    }
    #[getter]
    fn magnetic_field_covariance(&self, py: Python<'_>) -> PyResult<Py<PyAny>> {
        array9_to_list(py, self.inner.magnetic_field_covariance())
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── sensor_msgs.FluidPressure ───────────────────────────────────────

#[pyclass(
    name = "FluidPressure",
    module = "edgefirst.schemas.sensor_msgs",
    frozen
)]
pub struct PyFluidPressure {
    inner: FluidPressure<Vec<u8>>,
}

#[pymethods]
impl PyFluidPressure {
    #[new]
    #[pyo3(signature = (header, fluid_pressure=0.0, variance=0.0))]
    fn new(header: &PyHeader, fluid_pressure: f64, variance: f64) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let inner = FluidPressure::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .fluid_pressure(fluid_pressure)
            .variance(variance)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = FluidPressure::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn fluid_pressure(&self) -> f64 {
        self.inner.fluid_pressure()
    }
    #[getter]
    fn variance(&self) -> f64 {
        self.inner.variance()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── sensor_msgs.Temperature ─────────────────────────────────────────

#[pyclass(name = "Temperature", module = "edgefirst.schemas.sensor_msgs", frozen)]
pub struct PyTemperature {
    inner: Temperature<Vec<u8>>,
}

#[pymethods]
impl PyTemperature {
    #[new]
    #[pyo3(signature = (header, temperature=0.0, variance=0.0))]
    fn new(header: &PyHeader, temperature: f64, variance: f64) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let inner = Temperature::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .temperature(temperature)
            .variance(variance)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = Temperature::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn temperature(&self) -> f64 {
        self.inner.temperature()
    }
    #[getter]
    fn variance(&self) -> f64 {
        self.inner.variance()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── sensor_msgs.CameraInfo (buffer-backed) ──────────────────────────

#[pyclass(name = "CameraInfo", module = "edgefirst.schemas.sensor_msgs", frozen)]
pub struct PyCameraInfo {
    inner: CameraInfo<Vec<u8>>,
}

#[pymethods]
impl PyCameraInfo {
    #[new]
    #[pyo3(signature = (
        header,
        height=0,
        width=0,
        distortion_model="",
        d=None,
        k=None,
        r=None,
        p=None,
        binning_x=0,
        binning_y=0,
        roi=None,
    ))]
    fn new(
        header: &PyHeader,
        height: u32,
        width: u32,
        distortion_model: &str,
        d: Option<Vec<f64>>,
        k: Option<Vec<f64>>,
        r: Option<Vec<f64>>,
        p: Option<Vec<f64>>,
        binning_x: u32,
        binning_y: u32,
        roi: Option<PyRegionOfInterest>,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let d_vec = d.unwrap_or_default();
        let k_arr = extract_f64_array::<9>(k)?;
        let r_arr = extract_f64_array::<9>(r)?;
        let p_arr = extract_f64_array::<12>(p)?;
        let roi_val = roi.map(|r| r.0).unwrap_or(RegionOfInterest {
            x_offset: 0,
            y_offset: 0,
            height: 0,
            width: 0,
            do_rectify: false,
        });
        let inner = CameraInfo::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .height(height)
            .width(width)
            .distortion_model(distortion_model)
            .d(&d_vec)
            .k(k_arr)
            .r(r_arr)
            .p(p_arr)
            .binning_x(binning_x)
            .binning_y(binning_y)
            .roi(roi_val)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = CameraInfo::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn height(&self) -> u32 {
        self.inner.height()
    }
    #[getter]
    fn width(&self) -> u32 {
        self.inner.width()
    }
    #[getter]
    fn distortion_model(&self) -> &str {
        self.inner.distortion_model()
    }
    #[getter]
    fn d(&self) -> Vec<f64> {
        (0..self.inner.d_len())
            .map(|i| self.inner.d_get(i))
            .collect()
    }
    #[getter]
    fn k(&self) -> [f64; 9] {
        self.inner.k()
    }
    #[getter]
    fn r(&self) -> [f64; 9] {
        self.inner.r()
    }
    #[getter]
    fn p(&self) -> [f64; 12] {
        self.inner.p()
    }
    #[getter]
    fn binning_x(&self) -> u32 {
        self.inner.binning_x()
    }
    #[getter]
    fn binning_y(&self) -> u32 {
        self.inner.binning_y()
    }
    #[getter]
    fn roi(&self) -> PyRegionOfInterest {
        PyRegionOfInterest(self.inner.roi())
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

/// Extract a fixed-size f64 array from an optional Vec, defaulting to zeros.
fn extract_f64_array<const N: usize>(v: Option<Vec<f64>>) -> PyResult<[f64; N]> {
    match v {
        None => Ok([0.0; N]),
        Some(vec) => {
            if vec.len() != N {
                return Err(PyValueError::new_err(format!(
                    "expected {} elements, got {}",
                    N,
                    vec.len()
                )));
            }
            let mut arr = [0.0; N];
            arr.copy_from_slice(&vec);
            Ok(arr)
        }
    }
}

// ── sensor_msgs.BatteryState (buffer-backed) ────────────────────────

#[pyclass(
    name = "BatteryState",
    module = "edgefirst.schemas.sensor_msgs",
    frozen
)]
pub struct PyBatteryState {
    inner: BatteryState<Vec<u8>>,
}

#[pymethods]
impl PyBatteryState {
    #[new]
    #[pyo3(signature = (
        header,
        voltage=0.0,
        temperature=0.0,
        current=0.0,
        charge=0.0,
        capacity=0.0,
        design_capacity=0.0,
        percentage=0.0,
        power_supply_status=0,
        power_supply_health=0,
        power_supply_technology=0,
        present=false,
        cell_voltage=None,
        cell_temperature=None,
        location="",
        serial_number="",
    ))]
    fn new(
        header: &PyHeader,
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
        cell_voltage: Option<Vec<f32>>,
        cell_temperature: Option<Vec<f32>>,
        location: &str,
        serial_number: &str,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let cv = cell_voltage.unwrap_or_default();
        let ct = cell_temperature.unwrap_or_default();
        let inner = BatteryState::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .voltage(voltage)
            .temperature(temperature)
            .current(current)
            .charge(charge)
            .capacity(capacity)
            .design_capacity(design_capacity)
            .percentage(percentage)
            .power_supply_status(power_supply_status)
            .power_supply_health(power_supply_health)
            .power_supply_technology(power_supply_technology)
            .present(present)
            .cell_voltage(&cv)
            .cell_temperature(&ct)
            .location(location)
            .serial_number(serial_number)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = BatteryState::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn voltage(&self) -> f32 {
        self.inner.voltage()
    }
    #[getter]
    fn temperature(&self) -> f32 {
        self.inner.temperature()
    }
    #[getter]
    fn current(&self) -> f32 {
        self.inner.current()
    }
    #[getter]
    fn charge(&self) -> f32 {
        self.inner.charge()
    }
    #[getter]
    fn capacity(&self) -> f32 {
        self.inner.capacity()
    }
    #[getter]
    fn design_capacity(&self) -> f32 {
        self.inner.design_capacity()
    }
    #[getter]
    fn percentage(&self) -> f32 {
        self.inner.percentage()
    }
    #[getter]
    fn power_supply_status(&self) -> u8 {
        self.inner.power_supply_status()
    }
    #[getter]
    fn power_supply_health(&self) -> u8 {
        self.inner.power_supply_health()
    }
    #[getter]
    fn power_supply_technology(&self) -> u8 {
        self.inner.power_supply_technology()
    }
    #[getter]
    fn present(&self) -> bool {
        self.inner.present()
    }
    #[getter]
    fn cell_voltage(&self) -> Vec<f32> {
        self.inner.cell_voltage()
    }
    #[getter]
    fn cell_temperature(&self) -> Vec<f32> {
        self.inner.cell_temperature()
    }
    #[getter]
    fn location(&self) -> &str {
        self.inner.location()
    }
    #[getter]
    fn serial_number(&self) -> &str {
        self.inner.serial_number()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── nav_msgs.Odometry ───────────────────────────────────────────────

#[pyclass(name = "Odometry", module = "edgefirst.schemas.nav_msgs", frozen)]
pub struct PyOdometry {
    inner: Odometry<Vec<u8>>,
}

#[pymethods]
impl PyOdometry {
    #[new]
    #[pyo3(signature = (header, child_frame_id="", pose=None, twist=None))]
    fn new(
        header: &PyHeader,
        child_frame_id: &str,
        pose: Option<PyPoseWithCovariance>,
        twist: Option<PyTwistWithCovariance>,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let p = pose.map(|x| x.0).unwrap_or(PoseWithCovariance {
            pose: Pose {
                position: Point {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
            covariance: [0.0; 36],
        });
        let t = twist.map(|x| x.0).unwrap_or(TwistWithCovariance {
            twist: Twist {
                linear: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            },
            covariance: [0.0; 36],
        });
        // Odometry doesn't expose a builder in the Rust crate; use the
        // deprecated `new()` constructor (intentional — it's the only
        // public constructor for this type).
        #[allow(deprecated)]
        let inner = Odometry::new(stamp, &frame_id, child_frame_id, p, t).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = Odometry::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn child_frame_id(&self) -> &str {
        self.inner.child_frame_id()
    }
    #[getter]
    fn pose(&self) -> PyPoseWithCovariance {
        PyPoseWithCovariance(self.inner.pose())
    }
    #[getter]
    fn twist(&self) -> PyTwistWithCovariance {
        PyTwistWithCovariance(self.inner.twist())
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }

    fn __repr__(&self) -> String {
        format!(
            "Odometry(stamp={:?}, frame_id={:?}, child_frame_id={:?}, pose=PoseWithCovariance(...), twist=TwistWithCovariance(...))",
            self.inner.stamp(),
            self.inner.frame_id(),
            self.inner.child_frame_id(),
        )
    }
}

// ── edgefirst_msgs.LocalTime ────────────────────────────────────────

#[pyclass(
    name = "LocalTime",
    module = "edgefirst.schemas.edgefirst_msgs",
    frozen
)]
pub struct PyLocalTime {
    inner: LocalTime<Vec<u8>>,
}

#[pymethods]
impl PyLocalTime {
    #[new]
    #[pyo3(signature = (header, date=None, time=None, timezone=0))]
    fn new(
        header: &PyHeader,
        date: Option<PyDate>,
        time: Option<PyTime>,
        timezone: i16,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let d = date.map(|d| d.0).unwrap_or(Date {
            year: 1970,
            month: 1,
            day: 1,
        });
        let t = time.map(|t| t.0).unwrap_or(Time { sec: 0, nanosec: 0 });
        let inner = LocalTime::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .date(d)
            .time(t)
            .timezone(timezone)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = LocalTime::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn date(&self) -> PyDate {
        PyDate(self.inner.date())
    }
    #[getter]
    fn time(&self) -> PyTime {
        PyTime(self.inner.time())
    }
    #[getter]
    fn timezone(&self) -> i16 {
        self.inner.timezone()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── edgefirst_msgs.Track ────────────────────────────────────────────

#[pyclass(name = "Track", module = "edgefirst.schemas.edgefirst_msgs", frozen)]
pub struct PyTrack {
    inner: Track<Vec<u8>>,
}

#[pymethods]
impl PyTrack {
    #[new]
    #[pyo3(signature = (id="", lifetime=0, created=None))]
    fn new(id: &str, lifetime: i32, created: Option<PyTime>) -> PyResult<Self> {
        let c = created.map(|t| t.0).unwrap_or(Time { sec: 0, nanosec: 0 });
        let inner = Track::builder()
            .id(id)
            .lifetime(lifetime)
            .created(c)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = Track::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn id(&self) -> &str {
        self.inner.id()
    }
    #[getter]
    fn lifetime(&self) -> i32 {
        self.inner.lifetime()
    }
    #[getter]
    fn created(&self) -> PyTime {
        PyTime(self.inner.created())
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── edgefirst_msgs.RadarInfo (buffer-backed) ────────────────────────

#[pyclass(
    name = "RadarInfo",
    module = "edgefirst.schemas.edgefirst_msgs",
    frozen
)]
pub struct PyRadarInfo {
    inner: RadarInfo<Vec<u8>>,
}

#[pymethods]
impl PyRadarInfo {
    #[new]
    #[pyo3(signature = (header, center_frequency="", frequency_sweep="", range_toggle="", detection_sensitivity="", cube=false))]
    fn new(
        header: &PyHeader,
        center_frequency: &str,
        frequency_sweep: &str,
        range_toggle: &str,
        detection_sensitivity: &str,
        cube: bool,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let inner = RadarInfo::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .center_frequency(center_frequency)
            .frequency_sweep(frequency_sweep)
            .range_toggle(range_toggle)
            .detection_sensitivity(detection_sensitivity)
            .cube(cube)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = RadarInfo::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn center_frequency(&self) -> &str {
        self.inner.center_frequency()
    }
    #[getter]
    fn frequency_sweep(&self) -> &str {
        self.inner.frequency_sweep()
    }
    #[getter]
    fn range_toggle(&self) -> &str {
        self.inner.range_toggle()
    }
    #[getter]
    fn detection_sensitivity(&self) -> &str {
        self.inner.detection_sensitivity()
    }
    #[getter]
    fn cube(&self) -> bool {
        self.inner.cube()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── edgefirst_msgs.Vibration (buffer-backed) ────────────────────────

#[pyclass(
    name = "Vibration",
    module = "edgefirst.schemas.edgefirst_msgs",
    frozen
)]
pub struct PyVibration {
    inner: Vibration<Vec<u8>>,
}

#[pymethods]
impl PyVibration {
    #[new]
    #[pyo3(signature = (
        header,
        vibration=None,
        band_lower_hz=0.0,
        band_upper_hz=0.0,
        measurement_type=0,
        unit=0,
        clipping=None,
    ))]
    fn new(
        header: &PyHeader,
        vibration: Option<PyVector3>,
        band_lower_hz: f32,
        band_upper_hz: f32,
        measurement_type: u8,
        unit: u8,
        clipping: Option<Vec<u32>>,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let v = vibration.map(|v| v.0).unwrap_or(Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        let clip = clipping.unwrap_or_default();
        let inner = Vibration::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .vibration(v)
            .band_lower_hz(band_lower_hz)
            .band_upper_hz(band_upper_hz)
            .measurement_type(measurement_type)
            .unit(unit)
            .clipping(&clip)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = Vibration::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn vibration(&self) -> PyVector3 {
        PyVector3(self.inner.vibration())
    }
    #[getter]
    fn band_lower_hz(&self) -> f32 {
        self.inner.band_lower_hz()
    }
    #[getter]
    fn band_upper_hz(&self) -> f32 {
        self.inner.band_upper_hz()
    }
    #[getter]
    fn measurement_type(&self) -> u8 {
        self.inner.measurement_type()
    }
    #[getter]
    fn unit(&self) -> u8 {
        self.inner.unit()
    }
    #[getter]
    fn clipping(&self) -> Vec<u32> {
        self.inner.clipping()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── edgefirst_msgs.ModelInfo (buffer-backed) ────────────────────────

#[pyclass(
    name = "ModelInfo",
    module = "edgefirst.schemas.edgefirst_msgs",
    frozen
)]
pub struct PyModelInfo {
    inner: ModelInfo<Vec<u8>>,
}

#[pymethods]
impl PyModelInfo {
    #[new]
    #[pyo3(signature = (
        header,
        input_shape=None,
        input_type=0,
        output_shape=None,
        output_type=0,
        labels=None,
        model_type="",
        model_format="",
        model_name="",
    ))]
    fn new(
        header: &PyHeader,
        input_shape: Option<Vec<u32>>,
        input_type: u8,
        output_shape: Option<Vec<u32>>,
        output_type: u8,
        labels: Option<Vec<String>>,
        model_type: &str,
        model_format: &str,
        model_name: &str,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let is = input_shape.unwrap_or_default();
        let os = output_shape.unwrap_or_default();
        let labels_owned = labels.unwrap_or_default();
        let labels_refs: Vec<&str> = labels_owned.iter().map(String::as_str).collect();
        let inner = ModelInfo::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .input_shape(&is)
            .input_type(input_type)
            .output_shape(&os)
            .output_type(output_type)
            .labels(&labels_refs)
            .model_type(model_type)
            .model_format(model_format)
            .model_name(model_name)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = ModelInfo::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn input_shape(&self) -> Vec<u32> {
        self.inner.input_shape().to_vec()
    }
    #[getter]
    fn input_type(&self) -> u8 {
        self.inner.input_type()
    }
    #[getter]
    fn output_shape(&self) -> Vec<u32> {
        self.inner.output_shape().to_vec()
    }
    #[getter]
    fn output_type(&self) -> u8 {
        self.inner.output_type()
    }
    #[getter]
    fn labels(&self) -> Vec<String> {
        self.inner.labels().into_iter().map(String::from).collect()
    }
    #[getter]
    fn model_type(&self) -> &str {
        self.inner.model_type()
    }
    #[getter]
    fn model_format(&self) -> &str {
        self.inner.model_format()
    }
    #[getter]
    fn model_name(&self) -> &str {
        self.inner.model_name()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── edgefirst_msgs view/helper types ────────────────────────────────

/// Python-facing DetectBox (read from CDR or used as constructor parameter).
#[pyclass(
    name = "DetectBox",
    module = "edgefirst.schemas.edgefirst_msgs",
    frozen
)]
#[derive(Clone)]
pub struct PyDetectBox {
    pub center_x: f32,
    pub center_y: f32,
    pub width: f32,
    pub height: f32,
    pub label: String,
    pub score: f32,
    pub distance: f32,
    pub speed: f32,
    pub track_id: String,
    pub track_lifetime: i32,
    pub track_created: Time,
}

impl PyDetectBox {
    fn from_view(v: &DetectBoxView<'_>) -> Self {
        Self {
            center_x: v.center_x,
            center_y: v.center_y,
            width: v.width,
            height: v.height,
            label: v.label.to_string(),
            score: v.score,
            distance: v.distance,
            speed: v.speed,
            track_id: v.track_id.to_string(),
            track_lifetime: v.track_lifetime,
            track_created: v.track_created,
        }
    }

    fn to_view(&self) -> DetectBoxView<'_> {
        DetectBoxView {
            center_x: self.center_x,
            center_y: self.center_y,
            width: self.width,
            height: self.height,
            label: &self.label,
            score: self.score,
            distance: self.distance,
            speed: self.speed,
            track_id: &self.track_id,
            track_lifetime: self.track_lifetime,
            track_created: self.track_created,
        }
    }
}

#[pymethods]
impl PyDetectBox {
    #[new]
    #[pyo3(signature = (
        center_x=0.0, center_y=0.0, width=0.0, height=0.0,
        label="", score=0.0, distance=0.0, speed=0.0,
        track_id="", track_lifetime=0, track_created=None,
    ))]
    fn new(
        center_x: f32,
        center_y: f32,
        width: f32,
        height: f32,
        label: &str,
        score: f32,
        distance: f32,
        speed: f32,
        track_id: &str,
        track_lifetime: i32,
        track_created: Option<PyTime>,
    ) -> Self {
        Self {
            center_x,
            center_y,
            width,
            height,
            label: label.to_string(),
            score,
            distance,
            speed,
            track_id: track_id.to_string(),
            track_lifetime,
            track_created: track_created
                .map(|t| t.0)
                .unwrap_or(Time { sec: 0, nanosec: 0 }),
        }
    }

    #[getter]
    fn center_x(&self) -> f32 {
        self.center_x
    }
    #[getter]
    fn center_y(&self) -> f32 {
        self.center_y
    }
    #[getter]
    fn width(&self) -> f32 {
        self.width
    }
    #[getter]
    fn height(&self) -> f32 {
        self.height
    }
    #[getter]
    fn label(&self) -> &str {
        &self.label
    }
    #[getter]
    fn score(&self) -> f32 {
        self.score
    }
    #[getter]
    fn distance(&self) -> f32 {
        self.distance
    }
    #[getter]
    fn speed(&self) -> f32 {
        self.speed
    }
    #[getter]
    fn track_id(&self) -> &str {
        &self.track_id
    }
    #[getter]
    fn track_lifetime(&self) -> i32 {
        self.track_lifetime
    }
    #[getter]
    fn track_created(&self) -> PyTime {
        PyTime(self.track_created)
    }
}

// ── edgefirst_msgs.Detect (buffer-backed) ───────────────────────────

#[pyclass(name = "Detect", module = "edgefirst.schemas.edgefirst_msgs", frozen)]
pub struct PyDetect {
    inner: Detect<Vec<u8>>,
}

#[pymethods]
impl PyDetect {
    #[new]
    #[pyo3(signature = (
        header,
        input_timestamp=None,
        model_time=None,
        output_time=None,
        boxes=None,
    ))]
    fn new(
        header: &PyHeader,
        input_timestamp: Option<PyTime>,
        model_time: Option<PyTime>,
        output_time: Option<PyTime>,
        boxes: Option<Vec<PyDetectBox>>,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let it = input_timestamp
            .map(|t| t.0)
            .unwrap_or(Time { sec: 0, nanosec: 0 });
        let mt = model_time
            .map(|t| t.0)
            .unwrap_or(Time { sec: 0, nanosec: 0 });
        let ot = output_time
            .map(|t| t.0)
            .unwrap_or(Time { sec: 0, nanosec: 0 });
        let box_views: Vec<DetectBoxView<'_>> = boxes
            .as_ref()
            .map(|b| b.iter().map(|bx| bx.to_view()).collect())
            .unwrap_or_default();
        let inner = Detect::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .input_timestamp(it)
            .model_time(mt)
            .output_time(ot)
            .boxes(&box_views)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = Detect::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn input_timestamp(&self) -> PyTime {
        PyTime(self.inner.input_timestamp())
    }
    #[getter]
    fn model_time(&self) -> PyTime {
        PyTime(self.inner.model_time())
    }
    #[getter]
    fn output_time(&self) -> PyTime {
        PyTime(self.inner.output_time())
    }
    #[getter]
    fn boxes(&self) -> Vec<PyDetectBox> {
        self.inner
            .boxes()
            .iter()
            .map(PyDetectBox::from_view)
            .collect()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── edgefirst_msgs.CameraPlane (view helper) ────────────────────────

#[pyclass(
    name = "CameraPlane",
    module = "edgefirst.schemas.edgefirst_msgs",
    frozen
)]
#[derive(Clone)]
pub struct PyCameraPlane {
    pub fd: i32,
    pub offset: u32,
    pub stride: u32,
    pub size: u32,
    pub used: u32,
    pub data_bytes: Vec<u8>,
}

impl PyCameraPlane {
    fn from_view(v: &CameraPlaneView<'_>) -> Self {
        Self {
            fd: v.fd,
            offset: v.offset,
            stride: v.stride,
            size: v.size,
            used: v.used,
            data_bytes: v.data.to_vec(),
        }
    }

    fn to_view(&self) -> CameraPlaneView<'_> {
        CameraPlaneView {
            fd: self.fd,
            offset: self.offset,
            stride: self.stride,
            size: self.size,
            used: self.used,
            data: &self.data_bytes,
        }
    }
}

#[pymethods]
impl PyCameraPlane {
    #[new]
    #[pyo3(signature = (fd=-1, offset=0, stride=0, size=0, used=0))]
    fn new(fd: i32, offset: u32, stride: u32, size: u32, used: u32) -> Self {
        Self {
            fd,
            offset,
            stride,
            size,
            used,
            data_bytes: Vec::new(),
        }
    }

    #[getter]
    fn fd(&self) -> i32 {
        self.fd
    }
    #[getter]
    fn offset(&self) -> u32 {
        self.offset
    }
    #[getter]
    fn stride(&self) -> u32 {
        self.stride
    }
    #[getter]
    fn size(&self) -> u32 {
        self.size
    }
    #[getter]
    fn used(&self) -> u32 {
        self.used
    }
    #[getter]
    fn data<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, &self.data_bytes)
    }
}

// ── edgefirst_msgs.CameraFrame (buffer-backed) ─────────────────────

#[pyclass(
    name = "CameraFrame",
    module = "edgefirst.schemas.edgefirst_msgs",
    frozen
)]
pub struct PyCameraFrame {
    inner: CameraFrame<Vec<u8>>,
}

#[pymethods]
impl PyCameraFrame {
    #[new]
    #[pyo3(signature = (
        header,
        seq=0,
        pid=0,
        width=0,
        height=0,
        format="",
        color_space="",
        color_transfer="",
        color_encoding="",
        color_range="",
        fence_fd=-1,
        planes=None,
    ))]
    fn new(
        header: &PyHeader,
        seq: u64,
        pid: u32,
        width: u32,
        height: u32,
        format: &str,
        color_space: &str,
        color_transfer: &str,
        color_encoding: &str,
        color_range: &str,
        fence_fd: i32,
        planes: Option<Vec<PyCameraPlane>>,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let plane_views: Vec<CameraPlaneView<'_>> = planes
            .as_ref()
            .map(|p| p.iter().map(|pl| pl.to_view()).collect())
            .unwrap_or_default();
        let inner = CameraFrame::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .seq(seq)
            .pid(pid)
            .width(width)
            .height(height)
            .format(format)
            .color_space(color_space)
            .color_transfer(color_transfer)
            .color_encoding(color_encoding)
            .color_range(color_range)
            .fence_fd(fence_fd)
            .planes(&plane_views)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = CameraFrame::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn seq(&self) -> u64 {
        self.inner.seq()
    }
    #[getter]
    fn pid(&self) -> u32 {
        self.inner.pid()
    }
    #[getter]
    fn width(&self) -> u32 {
        self.inner.width()
    }
    #[getter]
    fn height(&self) -> u32 {
        self.inner.height()
    }
    #[getter]
    fn format(&self) -> &str {
        self.inner.format()
    }
    #[getter]
    fn color_space(&self) -> &str {
        self.inner.color_space()
    }
    #[getter]
    fn color_transfer(&self) -> &str {
        self.inner.color_transfer()
    }
    #[getter]
    fn color_encoding(&self) -> &str {
        self.inner.color_encoding()
    }
    #[getter]
    fn color_range(&self) -> &str {
        self.inner.color_range()
    }
    #[getter]
    fn fence_fd(&self) -> i32 {
        self.inner.fence_fd()
    }
    #[getter]
    fn num_planes(&self) -> u32 {
        self.inner.num_planes()
    }
    #[getter]
    fn planes(&self) -> Vec<PyCameraPlane> {
        self.inner
            .planes()
            .iter()
            .map(PyCameraPlane::from_view)
            .collect()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── edgefirst_msgs.Model (buffer-backed) ────────────────────────────

/// Python-facing MaskView (for Model.masks() results).
#[pyclass(name = "MaskBox", module = "edgefirst.schemas.edgefirst_msgs", frozen)]
#[derive(Clone)]
pub struct PyMaskBox {
    pub mask_height: u32,
    pub mask_width: u32,
    pub length: u32,
    pub mask_encoding: String,
    pub mask_data: Vec<u8>,
    pub boxed: bool,
}

impl PyMaskBox {
    fn from_view(v: &MaskView<'_>) -> Self {
        Self {
            mask_height: v.height,
            mask_width: v.width,
            length: v.length,
            mask_encoding: v.encoding.to_string(),
            mask_data: v.mask.to_vec(),
            boxed: v.boxed,
        }
    }

    fn to_view(&self) -> MaskView<'_> {
        MaskView {
            height: self.mask_height,
            width: self.mask_width,
            length: self.length,
            encoding: &self.mask_encoding,
            mask: &self.mask_data,
            boxed: self.boxed,
        }
    }
}

#[pymethods]
impl PyMaskBox {
    #[getter]
    fn height(&self) -> u32 {
        self.mask_height
    }
    #[getter]
    fn width(&self) -> u32 {
        self.mask_width
    }
    #[getter]
    fn length(&self) -> u32 {
        self.length
    }
    #[getter]
    fn encoding(&self) -> &str {
        &self.mask_encoding
    }
    #[getter]
    fn mask<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, &self.mask_data)
    }
    #[getter]
    fn boxed(&self) -> bool {
        self.boxed
    }
}

#[pyclass(name = "Model", module = "edgefirst.schemas.edgefirst_msgs", frozen)]
pub struct PyModel {
    inner: Model<Vec<u8>>,
}

#[pymethods]
impl PyModel {
    #[new]
    #[pyo3(signature = (
        header,
        input_time=None,
        model_time=None,
        output_time=None,
        decode_time=None,
        boxes=None,
        masks=None,
    ))]
    fn new(
        header: &PyHeader,
        input_time: Option<PyDuration>,
        model_time: Option<PyDuration>,
        output_time: Option<PyDuration>,
        decode_time: Option<PyDuration>,
        boxes: Option<Vec<PyDetectBox>>,
        masks: Option<Vec<PyMaskBox>>,
    ) -> PyResult<Self> {
        let stamp = header.inner.stamp();
        let frame_id = header.inner.frame_id().to_string();
        let zero_dur = Duration { sec: 0, nanosec: 0 };
        let it = input_time.map(|d| d.0).unwrap_or(zero_dur);
        let mt = model_time.map(|d| d.0).unwrap_or(zero_dur);
        let ot = output_time.map(|d| d.0).unwrap_or(zero_dur);
        let dt = decode_time.map(|d| d.0).unwrap_or(zero_dur);
        let box_views: Vec<DetectBoxView<'_>> = boxes
            .as_ref()
            .map(|b| b.iter().map(|bx| bx.to_view()).collect())
            .unwrap_or_default();
        let mask_views: Vec<MaskView<'_>> = masks
            .as_ref()
            .map(|m| m.iter().map(|mx| mx.to_view()).collect())
            .unwrap_or_default();
        let inner = Model::builder()
            .stamp(stamp)
            .frame_id(frame_id.as_str())
            .input_time(it)
            .model_time(mt)
            .output_time(ot)
            .decode_time(dt)
            .boxes(&box_views)
            .masks(&mask_views)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = Model::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn stamp(&self) -> PyTime {
        PyTime(self.inner.stamp())
    }
    #[getter]
    fn frame_id(&self) -> &str {
        self.inner.frame_id()
    }
    #[getter]
    fn input_time(&self) -> PyDuration {
        PyDuration(self.inner.input_time())
    }
    #[getter]
    fn model_time(&self) -> PyDuration {
        PyDuration(self.inner.model_time())
    }
    #[getter]
    fn output_time(&self) -> PyDuration {
        PyDuration(self.inner.output_time())
    }
    #[getter]
    fn decode_time(&self) -> PyDuration {
        PyDuration(self.inner.decode_time())
    }
    #[getter]
    fn boxes(&self) -> Vec<PyDetectBox> {
        self.inner
            .boxes()
            .iter()
            .map(PyDetectBox::from_view)
            .collect()
    }
    #[getter]
    fn masks(&self) -> Vec<PyMaskBox> {
        self.inner
            .masks()
            .iter()
            .map(PyMaskBox::from_view)
            .collect()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── foxglove_msgs CdrFixed value types ──────────────────────────────

#[pyclass(name = "Point2", module = "edgefirst.schemas.foxglove_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyFoxglovePoint2(pub FoxglovePoint2);

#[pymethods]
impl PyFoxglovePoint2 {
    #[new]
    #[pyo3(signature = (x=0.0, y=0.0))]
    fn new(x: f64, y: f64) -> Self {
        Self(FoxglovePoint2 { x, y })
    }
    #[getter]
    fn x(&self) -> f64 {
        self.0.x
    }
    #[getter]
    fn y(&self) -> f64 {
        self.0.y
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<FoxglovePoint2>(py, buf)?))
    }
}

#[pyclass(name = "Color", module = "edgefirst.schemas.foxglove_msgs", frozen)]
#[derive(Clone, Copy)]
pub struct PyFoxgloveColor(pub FoxgloveColor);

#[pymethods]
impl PyFoxgloveColor {
    #[new]
    #[pyo3(signature = (r=0.0, g=0.0, b=0.0, a=0.0))]
    fn new(r: f64, g: f64, b: f64, a: f64) -> Self {
        Self(FoxgloveColor { r, g, b, a })
    }
    #[getter]
    fn r(&self) -> f64 {
        self.0.r
    }
    #[getter]
    fn g(&self) -> f64 {
        self.0.g
    }
    #[getter]
    fn b(&self) -> f64 {
        self.0.b
    }
    #[getter]
    fn a(&self) -> f64 {
        self.0.a
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<FoxgloveColor>(py, buf)?))
    }
}

// ── foxglove_msgs.CircleAnnotations (CdrFixed) ──────────────────────

#[pyclass(
    name = "CircleAnnotations",
    module = "edgefirst.schemas.foxglove_msgs",
    frozen
)]
#[derive(Clone, Copy)]
pub struct PyFoxgloveCircleAnnotations(pub FoxgloveCircleAnnotations);

#[pymethods]
impl PyFoxgloveCircleAnnotations {
    #[new]
    #[pyo3(signature = (
        timestamp=None, position=None, diameter=0.0, thickness=0.0,
        fill_color=None, outline_color=None,
    ))]
    fn new(
        timestamp: Option<PyTime>,
        position: Option<PyFoxglovePoint2>,
        diameter: f64,
        thickness: f64,
        fill_color: Option<PyFoxgloveColor>,
        outline_color: Option<PyFoxgloveColor>,
    ) -> Self {
        Self(FoxgloveCircleAnnotations {
            timestamp: timestamp
                .map(|t| t.0)
                .unwrap_or(Time { sec: 0, nanosec: 0 }),
            position: position
                .map(|p| p.0)
                .unwrap_or(FoxglovePoint2 { x: 0.0, y: 0.0 }),
            diameter,
            thickness,
            fill_color: fill_color.map(|c| c.0).unwrap_or(FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.0,
            }),
            outline_color: outline_color.map(|c| c.0).unwrap_or(FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.0,
            }),
        })
    }

    #[getter]
    fn timestamp(&self) -> PyTime {
        PyTime(self.0.timestamp)
    }
    #[getter]
    fn position(&self) -> PyFoxglovePoint2 {
        PyFoxglovePoint2(self.0.position)
    }
    #[getter]
    fn diameter(&self) -> f64 {
        self.0.diameter
    }
    #[getter]
    fn thickness(&self) -> f64 {
        self.0.thickness
    }
    #[getter]
    fn fill_color(&self) -> PyFoxgloveColor {
        PyFoxgloveColor(self.0.fill_color)
    }
    #[getter]
    fn outline_color(&self) -> PyFoxgloveColor {
        PyFoxgloveColor(self.0.outline_color)
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyBytes>> {
        cdrfixed_encode(py, &self.0)
    }
    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        Ok(Self(cdrfixed_decode::<FoxgloveCircleAnnotations>(py, buf)?))
    }
}

// ── foxglove_msgs.TextAnnotation (buffer-backed) ────────────────────

#[pyclass(
    name = "TextAnnotation",
    module = "edgefirst.schemas.foxglove_msgs",
    frozen
)]
pub struct PyFoxgloveTextAnnotation {
    inner: FoxgloveTextAnnotation<Vec<u8>>,
}

#[pymethods]
impl PyFoxgloveTextAnnotation {
    #[new]
    #[pyo3(signature = (
        timestamp=None, position=None, text="", font_size=0.0,
        text_color=None, background_color=None,
    ))]
    fn new(
        timestamp: Option<PyTime>,
        position: Option<PyFoxglovePoint2>,
        text: &str,
        font_size: f64,
        text_color: Option<PyFoxgloveColor>,
        background_color: Option<PyFoxgloveColor>,
    ) -> PyResult<Self> {
        let ts = timestamp
            .map(|t| t.0)
            .unwrap_or(Time { sec: 0, nanosec: 0 });
        let pos = position
            .map(|p| p.0)
            .unwrap_or(FoxglovePoint2 { x: 0.0, y: 0.0 });
        let tc = text_color.map(|c| c.0).unwrap_or(FoxgloveColor {
            r: 0.0,
            g: 0.0,
            b: 0.0,
            a: 0.0,
        });
        let bc = background_color.map(|c| c.0).unwrap_or(FoxgloveColor {
            r: 0.0,
            g: 0.0,
            b: 0.0,
            a: 0.0,
        });
        let inner = FoxgloveTextAnnotation::builder()
            .timestamp(ts)
            .position(pos)
            .text(text)
            .font_size(font_size)
            .text_color(tc)
            .background_color(bc)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = FoxgloveTextAnnotation::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn timestamp(&self) -> PyTime {
        PyTime(self.inner.timestamp())
    }
    #[getter]
    fn position(&self) -> PyFoxglovePoint2 {
        PyFoxglovePoint2(self.inner.position())
    }
    #[getter]
    fn text(&self) -> &str {
        self.inner.text()
    }
    #[getter]
    fn font_size(&self) -> f64 {
        self.inner.font_size()
    }
    #[getter]
    fn text_color(&self) -> PyFoxgloveColor {
        PyFoxgloveColor(self.inner.text_color())
    }
    #[getter]
    fn background_color(&self) -> PyFoxgloveColor {
        PyFoxgloveColor(self.inner.background_color())
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── foxglove_msgs.PointAnnotation (buffer-backed) ───────────────────

/// Python-facing PointAnnotation view (for ImageAnnotation.points()).
#[pyclass(
    name = "PointAnnotation",
    module = "edgefirst.schemas.foxglove_msgs",
    frozen
)]
pub struct PyFoxglovePointAnnotation {
    inner: FoxglovePointAnnotation<Vec<u8>>,
}

#[pymethods]
impl PyFoxglovePointAnnotation {
    #[new]
    #[pyo3(signature = (
        timestamp=None, type_=0, points=None, outline_color=None,
        outline_colors=None, fill_color=None, thickness=0.0,
    ))]
    fn new(
        timestamp: Option<PyTime>,
        type_: u8,
        points: Option<Vec<PyFoxglovePoint2>>,
        outline_color: Option<PyFoxgloveColor>,
        outline_colors: Option<Vec<PyFoxgloveColor>>,
        fill_color: Option<PyFoxgloveColor>,
        thickness: f64,
    ) -> PyResult<Self> {
        let ts = timestamp
            .map(|t| t.0)
            .unwrap_or(Time { sec: 0, nanosec: 0 });
        let pts: Vec<FoxglovePoint2> = points
            .map(|p| p.iter().map(|pt| pt.0).collect())
            .unwrap_or_default();
        let oc = outline_color.map(|c| c.0).unwrap_or(FoxgloveColor {
            r: 0.0,
            g: 0.0,
            b: 0.0,
            a: 0.0,
        });
        let ocs: Vec<FoxgloveColor> = outline_colors
            .map(|c| c.iter().map(|cc| cc.0).collect())
            .unwrap_or_default();
        let fc = fill_color.map(|c| c.0).unwrap_or(FoxgloveColor {
            r: 0.0,
            g: 0.0,
            b: 0.0,
            a: 0.0,
        });
        let inner = FoxglovePointAnnotation::builder()
            .timestamp(ts)
            .type_(type_)
            .points(&pts)
            .outline_color(oc)
            .outline_colors(&ocs)
            .fill_color(fc)
            .thickness(thickness)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = FoxglovePointAnnotation::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn timestamp(&self) -> PyTime {
        PyTime(self.inner.timestamp())
    }
    #[getter]
    fn type_(&self) -> u8 {
        self.inner.type_()
    }
    #[getter]
    fn points(&self) -> Vec<PyFoxglovePoint2> {
        self.inner
            .points()
            .into_iter()
            .map(PyFoxglovePoint2)
            .collect()
    }
    #[getter]
    fn outline_color(&self) -> PyFoxgloveColor {
        PyFoxgloveColor(self.inner.outline_color())
    }
    #[getter]
    fn outline_colors(&self) -> Vec<PyFoxgloveColor> {
        self.inner
            .outline_colors()
            .into_iter()
            .map(PyFoxgloveColor)
            .collect()
    }
    #[getter]
    fn fill_color(&self) -> PyFoxgloveColor {
        PyFoxgloveColor(self.inner.fill_color())
    }
    #[getter]
    fn thickness(&self) -> f64 {
        self.inner.thickness()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── foxglove_msgs.ImageAnnotation (buffer-backed) ───────────────────

/// Helper to convert FoxgloveTextAnnotationView → Python dict-like object.
fn text_annotation_view_to_py(
    v: &FoxgloveTextAnnotationView<'_>,
) -> PyResult<PyFoxgloveTextAnnotation> {
    let inner = FoxgloveTextAnnotation::builder()
        .timestamp(v.timestamp)
        .position(v.position)
        .text(v.text)
        .font_size(v.font_size)
        .text_color(v.text_color)
        .background_color(v.background_color)
        .build()
        .map_err(map_cdr_err)?;
    Ok(PyFoxgloveTextAnnotation { inner })
}

/// Helper to convert FoxglovePointAnnotationView → Python object.
fn point_annotation_view_to_py(
    v: &FoxglovePointAnnotationView,
) -> PyResult<PyFoxglovePointAnnotation> {
    let inner = FoxglovePointAnnotation::builder()
        .timestamp(v.timestamp)
        .type_(v.type_)
        .points(&v.points)
        .outline_color(v.outline_color)
        .outline_colors(&v.outline_colors)
        .fill_color(v.fill_color)
        .thickness(v.thickness)
        .build()
        .map_err(map_cdr_err)?;
    Ok(PyFoxglovePointAnnotation { inner })
}

#[pyclass(
    name = "ImageAnnotation",
    module = "edgefirst.schemas.foxglove_msgs",
    frozen
)]
pub struct PyFoxgloveImageAnnotation {
    inner: FoxgloveImageAnnotation<Vec<u8>>,
}

#[pymethods]
impl PyFoxgloveImageAnnotation {
    #[new]
    #[pyo3(signature = (circles=None, points=None, texts=None))]
    fn new(
        py: Python<'_>,
        circles: Option<Vec<PyFoxgloveCircleAnnotations>>,
        points: Option<Bound<'_, pyo3::types::PyList>>,
        texts: Option<Bound<'_, pyo3::types::PyList>>,
    ) -> PyResult<Self> {
        let c: Vec<FoxgloveCircleAnnotations> = circles
            .map(|cs| cs.iter().map(|ci| ci.0).collect())
            .unwrap_or_default();
        // Extract point annotations from Python list
        let point_objs: Vec<PyRef<'_, PyFoxglovePointAnnotation>> = match &points {
            Some(list) => list
                .iter()
                .map(|item| item.extract::<PyRef<'_, PyFoxglovePointAnnotation>>())
                .collect::<PyResult<Vec<_>>>()?,
            None => Vec::new(),
        };
        let point_views: Vec<FoxglovePointAnnotationView> = point_objs
            .iter()
            .map(|p| FoxglovePointAnnotationView {
                timestamp: p.inner.timestamp(),
                type_: p.inner.type_(),
                points: p.inner.points(),
                outline_color: p.inner.outline_color(),
                outline_colors: p.inner.outline_colors(),
                fill_color: p.inner.fill_color(),
                thickness: p.inner.thickness(),
            })
            .collect();
        // Extract text annotations from Python list
        let text_objs: Vec<PyRef<'_, PyFoxgloveTextAnnotation>> = match &texts {
            Some(list) => list
                .iter()
                .map(|item| item.extract::<PyRef<'_, PyFoxgloveTextAnnotation>>())
                .collect::<PyResult<Vec<_>>>()?,
            None => Vec::new(),
        };
        let text_views: Vec<FoxgloveTextAnnotationView<'_>> = text_objs
            .iter()
            .map(|t| FoxgloveTextAnnotationView {
                timestamp: t.inner.timestamp(),
                position: t.inner.position(),
                text: t.inner.text(),
                font_size: t.inner.font_size(),
                text_color: t.inner.text_color(),
                background_color: t.inner.background_color(),
            })
            .collect();
        let _ = py;
        let inner = FoxgloveImageAnnotation::builder()
            .circles(&c)
            .points(&point_views)
            .texts(&text_views)
            .build()
            .map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[classmethod]
    fn from_cdr(
        _cls: &Bound<'_, PyType>,
        py: Python<'_>,
        buf: &Bound<'_, PyAny>,
    ) -> PyResult<Self> {
        let (pybuf, ptr, len) = buffer_as_bytes(buf)?;
        let ptr_addr = ptr as usize;
        let owned: Vec<u8> = py.detach(|| {
            let slice = unsafe { std::slice::from_raw_parts(ptr_addr as *const u8, len) };
            slice.to_vec()
        });
        drop(pybuf);
        let inner = FoxgloveImageAnnotation::from_cdr(owned).map_err(map_cdr_err)?;
        Ok(Self { inner })
    }

    #[getter]
    fn circles(&self) -> Vec<PyFoxgloveCircleAnnotations> {
        self.inner
            .circles()
            .into_iter()
            .map(PyFoxgloveCircleAnnotations)
            .collect()
    }
    #[getter]
    fn points(&self) -> PyResult<Vec<PyFoxglovePointAnnotation>> {
        self.inner
            .points()
            .iter()
            .map(point_annotation_view_to_py)
            .collect()
    }
    #[getter]
    fn texts(&self) -> PyResult<Vec<PyFoxgloveTextAnnotation>> {
        self.inner
            .texts()
            .iter()
            .map(text_annotation_view_to_py)
            .collect()
    }
    #[getter]
    fn cdr_size(&self) -> usize {
        self.inner.as_cdr().len()
    }
    fn to_bytes<'py>(&self, py: Python<'py>) -> Bound<'py, PyBytes> {
        PyBytes::new(py, self.inner.as_cdr())
    }
}

// ── Module setup ────────────────────────────────────────────────────

#[pymodule]
fn schemas(m: &Bound<'_, PyModule>) -> PyResult<()> {
    let py = m.py();

    m.add_class::<BorrowedBuf>()?;

    // builtin_interfaces submodule
    let bi = PyModule::new(py, "builtin_interfaces")?;
    bi.add_class::<PyTime>()?;
    bi.add_class::<PyDuration>()?;
    m.add_submodule(&bi)?;
    register_submodule(py, m, "builtin_interfaces", &bi)?;

    // std_msgs submodule
    let stdm = PyModule::new(py, "std_msgs")?;
    stdm.add_class::<PyHeader>()?;
    stdm.add_class::<PyColorRGBA>()?;
    m.add_submodule(&stdm)?;
    register_submodule(py, m, "std_msgs", &stdm)?;

    // sensor_msgs submodule
    let sensor = PyModule::new(py, "sensor_msgs")?;
    sensor.add_class::<PyImage>()?;
    sensor.add_class::<PyCompressedImage>()?;
    sensor.add_class::<PyPointCloud2>()?;
    sensor.add_class::<PyPointField>()?;
    sensor.add_class::<PyNavSatStatus>()?;
    sensor.add_class::<PyRegionOfInterest>()?;
    sensor.add_class::<PyImu>()?;
    sensor.add_class::<PyNavSatFix>()?;
    sensor.add_class::<PyMagneticField>()?;
    sensor.add_class::<PyFluidPressure>()?;
    sensor.add_class::<PyTemperature>()?;
    sensor.add_class::<PyCameraInfo>()?;
    sensor.add_class::<PyBatteryState>()?;
    m.add_submodule(&sensor)?;
    register_submodule(py, m, "sensor_msgs", &sensor)?;

    // geometry_msgs submodule
    let geom = PyModule::new(py, "geometry_msgs")?;
    geom.add_class::<PyVector3>()?;
    geom.add_class::<PyPoint>()?;
    geom.add_class::<PyPoint32>()?;
    geom.add_class::<PyQuaternion>()?;
    geom.add_class::<PyPose>()?;
    geom.add_class::<PyPose2D>()?;
    geom.add_class::<PyTransform>()?;
    geom.add_class::<PyTwist>()?;
    geom.add_class::<PyAccel>()?;
    geom.add_class::<PyInertia>()?;
    geom.add_class::<PyPoseWithCovariance>()?;
    geom.add_class::<PyTwistWithCovariance>()?;
    geom.add_class::<PyTwistStamped>()?;
    geom.add_class::<PyAccelStamped>()?;
    geom.add_class::<PyInertiaStamped>()?;
    geom.add_class::<PyPointStamped>()?;
    geom.add_class::<PyTransformStamped>()?;
    m.add_submodule(&geom)?;
    register_submodule(py, m, "geometry_msgs", &geom)?;

    // nav_msgs submodule
    let nav = PyModule::new(py, "nav_msgs")?;
    nav.add_class::<PyOdometry>()?;
    m.add_submodule(&nav)?;
    register_submodule(py, m, "nav_msgs", &nav)?;

    // edgefirst_msgs submodule
    let edgef = PyModule::new(py, "edgefirst_msgs")?;
    edgef.add_class::<PyMask>()?;
    edgef.add_class::<PyRadarCube>()?;
    edgef.add_class::<PyDmaBuffer>()?;
    edgef.add_class::<PyDate>()?;
    edgef.add_class::<PyLocalTime>()?;
    edgef.add_class::<PyTrack>()?;
    edgef.add_class::<PyRadarInfo>()?;
    edgef.add_class::<PyVibration>()?;
    edgef.add_class::<PyModelInfo>()?;
    edgef.add_class::<PyDetectBox>()?;
    edgef.add_class::<PyDetect>()?;
    edgef.add_class::<PyCameraPlane>()?;
    edgef.add_class::<PyCameraFrame>()?;
    edgef.add_class::<PyMaskBox>()?;
    edgef.add_class::<PyModel>()?;
    m.add_submodule(&edgef)?;
    register_submodule(py, m, "edgefirst_msgs", &edgef)?;

    // rosgraph_msgs submodule
    let rosgraph = PyModule::new(py, "rosgraph_msgs")?;
    rosgraph.add_class::<PyClock>()?;
    m.add_submodule(&rosgraph)?;
    register_submodule(py, m, "rosgraph_msgs", &rosgraph)?;

    // foxglove_msgs submodule
    let foxg = PyModule::new(py, "foxglove_msgs")?;
    foxg.add_class::<PyFoxgloveCompressedVideo>()?;
    foxg.add_class::<PyFoxglovePoint2>()?;
    foxg.add_class::<PyFoxgloveColor>()?;
    foxg.add_class::<PyFoxgloveCircleAnnotations>()?;
    foxg.add_class::<PyFoxgloveTextAnnotation>()?;
    foxg.add_class::<PyFoxglovePointAnnotation>()?;
    foxg.add_class::<PyFoxgloveImageAnnotation>()?;
    m.add_submodule(&foxg)?;
    register_submodule(py, m, "foxglove_msgs", &foxg)?;

    Ok(())
}

/// Register a submodule in `sys.modules` so `from edgefirst.schemas.X import …`
/// works. pyo3's `add_submodule` only attaches the child as an attribute; without
/// this, dotted imports raise `ModuleNotFoundError` because Python's import
/// machinery looks in `sys.modules`, not on the parent module's attributes.
/// Register a submodule into `sys.modules` under its dotted path so
/// `from edgefirst.schemas.sensor_msgs import Image` resolves directly
/// to the in-binary submodule object. Without this, Python's import
/// machinery looks in `sys.modules` first and only falls through to
/// attribute lookup if the dotted name isn't registered — which would
/// raise `ModuleNotFoundError` for our pyo3-defined children.
fn register_submodule(
    py: Python<'_>,
    parent: &Bound<'_, PyModule>,
    name: &str,
    child: &Bound<'_, PyModule>,
) -> PyResult<()> {
    let parent_name: String = parent.getattr("__name__")?.extract()?;
    let full_name = format!("{parent_name}.{name}");
    child.setattr("__name__", &full_name)?;
    let sys_modules = py.import("sys")?.getattr("modules")?;
    sys_modules.set_item(&full_name, child)?;
    Ok(())
}
