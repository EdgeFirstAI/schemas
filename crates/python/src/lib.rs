// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Python bindings for `edgefirst-schemas`.
//!
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
use edgefirst_schemas::edgefirst_msgs::{Date, LocalTime, Mask, RadarCube, Track};
use edgefirst_schemas::foxglove_msgs::{FoxgloveColor, FoxgloveCompressedVideo, FoxglovePoint2};
use edgefirst_schemas::geometry_msgs::{
    Accel, Inertia, Point, Point32, Pose, Pose2D, PoseWithCovariance, Quaternion, Transform, Twist,
    TwistWithCovariance, Vector3,
};
use edgefirst_schemas::nav_msgs::Odometry;
use edgefirst_schemas::rosgraph_msgs::Clock;
use edgefirst_schemas::sensor_msgs::{
    CompressedImage, FluidPressure, Image, Imu, MagneticField, NavSatFix, NavSatStatus,
    PointCloud2, PointFieldView, RegionOfInterest, Temperature,
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
