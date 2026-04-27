# Changelog

All notable changes to EdgeFirst Perception Schemas will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed (BREAKING)

- **Python module rewritten as a pyo3 binding (EDGEAI-1295).** The pure-Python
  `pycdr2`-based `edgefirst.schemas` package is replaced by a Rust extension
  module that exposes the same wire shapes through zero-copy CDR encode/decode.
  Wheels ship as `cp311-abi3` (Python 3.11+, single wheel per OS/arch) covering
  Linux x86_64/aarch64, macOS x86_64/aarch64, and Windows x86_64. An
  `abi3-py38` opt-in is available for embedded targets that pin an older
  Python.

  **Migration impact for `edgefirst.schemas` users:**

  - **Frozen pyclasses replace dataclasses.** Field assignment after
    construction (`img.width = 1280`) now raises `AttributeError`. Build a
    new instance instead. Constructor kwargs are unchanged (same field names,
    same order).
  - **`serialize()` → `to_bytes()`, `deserialize(buf)` → `from_cdr(buf)`** on
    every message type.
  - **Bulk byte payloads** (`Image.data`, `Mask.mask`, `RadarCube.cube`, …)
    return a zero-copy `BorrowedBuf` instead of `bytes`. Use
    `np.frombuffer(borrowed_buf, dtype=...)` for a zero-copy ndarray, or
    `borrowed_buf.tobytes()` for an explicit copy.
  - **Schema-registry helpers** (`from_schema`, `is_supported`,
    `list_schemas`, `schema_name`, `decode_pcd`, `colormap`) and the
    primitive `std_msgs` wrappers (`String`, `Int32`, `Float64`, …) are
    removed. The legacy module is preserved at
    [`benches/python/legacy/`](benches/python/legacy/) for benchmark
    comparison only.
  - **`requires-python` floors at 3.11** (default `cp311-abi3` wheel). Older
    Python support requires building with `--no-default-features --features
    abi3-py38`.

  Wire-format compatibility is preserved: bytes encoded by the previous
  pycdr2 module decode through `from_cdr()` and vice-versa. See
  [`crates/python/README.md`](crates/python/README.md) for the full
  migration guide.

### Added

- **pyo3 bindings for 39 message types** with a comprehensive `.pyi` stub
  package (mypy/pyright-friendly). New types beyond the legacy pycdr2
  surface: `BorrowedBuf` (zero-copy view primitive), full `Imu`/`NavSatFix`/
  `MagneticField`/`FluidPressure`/`Temperature` (with covariance arrays),
  `Odometry`, `LocalTime`, `Track`, `Date`, `Clock`, foxglove `Point2`/`Color`.
- **`BENCHMARKS_PYTHON.md`** — three-way comparison (native pyo3 /
  cyclonedds-python / pycdr2) measured on rpi5-hailo, mirroring the C++-tier
  `BENCHMARKS.md` chart structure. Native pyo3 lands 25–870× faster than the
  Python alternatives across all message types.
- **Multi-platform wheel CI** ([`.github/workflows/wheels.yml`](.github/workflows/wheels.yml))
  building cp311-abi3 wheels for Linux x86_64/aarch64 (zig + manylinux2014),
  macOS x86_64/aarch64, and Windows x86_64 on every push and tag. Release
  workflow downloads pre-built wheels and publishes to PyPI via trusted
  publishing.
- **Coverage via `cargo-llvm-cov`** — Python tests now contribute to Rust
  source coverage by instrumenting the cdylib at build time and accumulating
  profraw across pytest invocations. SonarCloud merges Rust unit + C tests +
  Python-driven LCOV reports. Replaces the obsolete `pytest --cov=edgefirst`
  flow that couldn't see into a binary module.

## [3.2.0] - 2026-04-24

### Added

- **Publisher buffer reuse via Builder pattern (EDGEAI-1289)** — every
  buffer-backed message type now exposes a `Foo::builder()` entry
  point returning a `FooBuilder<'a>` with three finalizers:

  - `build() -> Result<Foo<Vec<u8>>, CdrError>` — allocates a fresh
    buffer (drop-in replacement for the legacy `Foo::new(...)`).
  - `encode_into_vec(&mut Vec<u8>) -> Result<(), CdrError>` — resizes
    the caller's `Vec` to exactly the encoded size and writes. Reuses
    the existing allocation across publishes when capacity suffices.
  - `encode_into_slice(&mut [u8]) -> Result<usize, CdrError>` — writes
    into a fixed-capacity slice, returns bytes written, errors
    `BufferTooShort` without mutating the destination on error.

  Setters are `&mut self -> &mut Self`, supporting both chained
  one-shot and named-reuse idioms. Strings use `Cow<'a, str>`, bulk
  data uses `&'a [u8]`, nested views use `&'a [View<'a>]`. Fixed
  scalars and `CdrFixed` types pass by value.

  27 builders total — every buffer-backed message type in the crate:
  `Header`, `CompressedImage`, `Image`, `Imu`, `NavSatFix`,
  `PointField`, `PointCloud2`, `CameraInfo`, `MagneticField`,
  `FluidPressure`, `Temperature`, `BatteryState`, `Mask`, `LocalTime`,
  `RadarCube`, `RadarInfo`, `Track`, `DetectBox`, `Detect`,
  `CameraFrame`, `Model`, `ModelInfo`, `Vibration`,
  `FoxgloveCompressedVideo`, `FoxgloveTextAnnotation`,
  `FoxglovePointAnnotation`, `FoxgloveImageAnnotation`.

- **In-place scalar setter audit** — every fixed-size field on every
  buffer-backed message now has a `set_*` mutator for publishers that
  update scalars between frames without paying for full
  re-serialisation. `CameraFrame::set_stamp` is ~8 byte writes vs. the
  builder's full re-encode of ~2 MB of pixel data.

- **C FFI builder surface** — every Rust builder has a parallel C
  handle-based API following the `ros_*_builder_t` convention: opaque
  handle, one setter per field (strings copy internally, bulk data
  borrows with a documented lifetime contract), `build()` /
  `encode_into()` finalizers. See `CAPI.md` for the migration pattern.

### Deprecated

- **All legacy `Foo::new(...)` constructors** on buffer-backed message
  types are marked `#[deprecated(since = "3.2.0")]` and will be
  removed in 4.0. Use `Foo::builder()` instead.

- **All legacy `ros_*_encode(...)` C FFI functions** continue to work
  but route through the builder internally. The `ros_*_builder_*` API
  is the recommended replacement.

### Migration

```rust
// Before
let img = Image::new(stamp, "camera", h, w, "rgb8", 0, stride, &pixels)?;

// After (drop-in)
let img = Image::builder()
    .stamp(stamp).frame_id("camera").height(h).width(w)
    .encoding("rgb8").is_bigendian(0).step(stride).data(&pixels)
    .build()?;

// Or, with buffer reuse across publishes
let mut cdr_buf = Vec::new();
let mut b = Image::builder();
b.frame_id("camera").height(h).width(w).encoding("rgb8").step(stride);
loop {
    b.stamp(now()).data(&pixels);
    b.encode_into_vec(&mut cdr_buf)?;
    publish(&cdr_buf);
}
```

## [3.1.0] - 2026-04-22

### Added

- **UAV telemetry schemas (TOP2-770)** — seven ROS 2 common_interfaces
  types plus one new EdgeFirst type, all with full Rust / Python /
  C / C++ surface and golden CDR fixtures:

  - `sensor_msgs/msg/MagneticField` — Header + Vector3 + float64[9].
  - `sensor_msgs/msg/FluidPressure` — Header + float64 + variance.
  - `sensor_msgs/msg/Temperature` — Header + float64 + variance.
  - `sensor_msgs/msg/BatteryState` — Header + 7 f32 + 3 enum u8 + bool
    + 2 f32 sequences + 2 strings. `POWER_SUPPLY_STATUS_*`,
    `POWER_SUPPLY_HEALTH_*`, `POWER_SUPPLY_TECHNOLOGY_*` constants
    exposed in all four languages.
  - `nav_msgs/msg/Odometry` (new module) — Header + child_frame_id +
    `PoseWithCovariance` + `TwistWithCovariance`.
  - `geometry_msgs/msg/PoseWithCovariance` — `CdrFixed` (344 B).
  - `geometry_msgs/msg/TwistWithCovariance` — `CdrFixed` (336 B).
  - `edgefirst_msgs/msg/Vibration` (new schema) — Header + 2 u8 enums
    (`MEASUREMENT_*`, `UNIT_*`) + f32 band + `Vector3 vibration` +
    `uint32[] clipping`. Generalizes MAVLink VIBRATION, ArduPilot
    VIBE, PX4 `vehicle_imu_status`, and ISO 10816/20816 industrial
    broadband vibration sensors. Drives the `adis-uav-mavlink`
    bridge's `flight/vibration` topic (TOP2-766).

  Bindings across all four languages:

  - Rust: `edgefirst_schemas::sensor_msgs::{MagneticField,
    FluidPressure, Temperature, BatteryState}`,
    `edgefirst_schemas::nav_msgs::Odometry`,
    `edgefirst_schemas::edgefirst_msgs::Vibration`,
    `geometry_msgs::{PoseWithCovariance, TwistWithCovariance}`.
  - C: `ros_<type>_t` + `ros_<type>_from_cdr` / `_free` /
    `_get_*` / `_as_cdr` accessors for buffer-backed types; `encode` /
    `decode` pair for CdrFixed types. `ros_*` prefix retained for
    consistency with the rest of the 3.x C API (per-namespace prefix
    normalization deferred to 4.0.0).
  - C++: header-only view classes `MagneticFieldView`,
    `FluidPressureView`, `TemperatureView`, `BatteryStateView`,
    `OdometryView`, `VibrationView` (move-only, non-owning), plus
    value classes `PoseWithCovariance`, `TwistWithCovariance`.
    `BatteryStateView` and `VibrationView` expose their constants as
    `static constexpr` members.
  - Python: dataclasses already present in
    `edgefirst.schemas.{sensor_msgs, nav_msgs, geometry_msgs}`;
    `edgefirst.schemas.edgefirst_msgs.Vibration` added with
    `VibrationMeasurement` and `VibrationUnit` enums.

  All six Header-prefixed buffer-backed types ship with a parametric
  `frame_id`-length sweep test (lengths 0..=16) to lock in freedom
  from the EDGEAI-1243 class of alignment bug.

- **CameraFrame / CameraPlane** — new multi-plane video frame schema
  (`edgefirst_msgs/msg/CameraFrame`, `edgefirst_msgs/msg/CameraPlane`).
  Supports planar formats (NV12, I420, planar RGB HWC/NCHW, signed i8
  model inputs), hardware codec bitstreams (H.264/H.265/MJPEG with
  `used < size`), DMA-fence sync fd, monotonic frame sequence counter,
  four-axis colorimetry (primaries / transfer / encoding matrix / range),
  and an off-device bridge path via per-plane inlined `data[]` when
  `fd == -1`.

  Bindings added in all four languages:

  - Rust: `edgefirst_schemas::edgefirst_msgs::CameraFrame<B>`,
    `CameraPlaneView`, plus `scan_plane_element` / `write_plane_element`
    / `size_plane_element` helpers matching the `Detect` / `DetectBox`
    precedent.
  - C: `ros_camera_frame_t`, `ros_camera_plane_t`, and the
    associated `ros_camera_frame_*` / `ros_camera_plane_*`
    accessors. Parent-borrowed plane pointers protected by an
    `owned=false` flag mirroring `ros_box_free` defense-in-depth.
    (The `ros_` prefix is retained for consistency with the rest of
    the 3.x C API; a full prefix normalization to per-namespace
    prefixes — `edgefirst_*`, `foxglove_*`, `geometry_*`, etc. — is
    planned for 4.0.0.)
  - C++: `edgefirst::schemas::CameraFrameView`,
    `edgefirst::schemas::detail::BorrowedCameraPlaneView`, with
    `planes()` yielding a range over parent-borrowed plane views.
  - Python: `edgefirst.schemas.edgefirst_msgs.CameraFrame`,
    `CameraPlane` dataclasses.

- **Eight golden CDR fixtures** for CameraFrame in
  `testdata/cdr/edgefirst_msgs/`: metadata-only `CameraFrame_empty`,
  single-plane RGB8, NV12 (shared fd), I420 (three planes), planar
  RGB NCHW, split-fd MPLANE (distinct fd per plane) with GPU fence,
  H.264 bitstream oversized buffer, and inlined-data off-device
  bridge. Round-tripped in Rust, Python, C, and C++ tests.

- **Schema registry** registers `edgefirst_msgs/msg/CameraFrame` and
  `edgefirst_msgs/msg/CameraPlane`.

### Fixed

- **`NavSatFix` accessor alignment bug (EDGEAI-1243).** `latitude()`,
  `longitude()`, `altitude()`, `position_covariance()`, and
  `position_covariance_type()` returned wrong values when the
  `frame_id` string length caused `offsets[0]` to land at an absolute
  buffer offset ≡ 1 (mod 8) — e.g. `frame_id = "gps_link"` (8 chars, the
  canonical ROS GPS frame). Root cause: `fixed_base` used
  `cdr_align(offsets[0] + NavSatStatus::CDR_SIZE, 8)` assuming
  `NavSatStatus` is always 4 bytes, but `int8 + uint16` collapses to
  3 bytes at odd CDR-relative start positions. Fixed by capturing the
  true post-`NavSatStatus` position from the cursor during `from_cdr`
  / `new` into `offsets[1]`, mirroring the pattern used elsewhere in
  the crate (`CameraFrame::planes_pos`, `CompressedImage::offsets[1]`).

  The on-wire bytes produced by `NavSatFix::new(...).to_cdr()` were
  always correct — any spec-compliant CDR decoder read them correctly.
  The defect was confined to the in-crate offset-math accessor shortcut.

  Regression test `navsatfix_accessors_robust_across_frame_id_alignments`
  sweeps `frame_id` lengths 0..=16 (every mod-8 residue). Previous
  golden coverage used a single `frame_id = "test_frame"` (10 chars)
  which landed in a non-bug region by coincidence.

- **`CdrFixed` trait docs** now warn that `CDR_SIZE` is only a reliable
  constant for types whose alignment requirements do not increase within
  the type. `NavSatStatus` is the sole counterexample in the crate and
  is marked with an in-source warning flagging the position-dependent
  size.

### Deprecated

- **`edgefirst_msgs/msg/DmaBuffer`** — superseded by `CameraFrame`.
  Removed in 4.0.0. Deprecated APIs (all retained through 3.x):
  - Rust: `DmaBuffer<B>`, `DmaBuffer::new`, `DmaBuffer::from_cdr`, and
    the `pid` / `fd` / `width` / `height` / `stride` / `fourcc` /
    `length` accessors.
  - C: `ros_dmabuffer_t`, `ros_dmabuffer_from_cdr`,
    `ros_dmabuffer_free`, `ros_dmabuffer_encode`, and all
    `ros_dmabuffer_get_*` accessors.
  - C++: `edgefirst::schemas::DmaBufferView`,
    `edgefirst::schemas::DmaBuffer`.
  - Python: `edgefirst.schemas.edgefirst_msgs.DmaBuffer`.

### Notes

- The canonical grammar for the `format` string (e.g. `"NV12"`,
  `"rgb8_planar_nchw"`, `"h264"`, `"nv12:amlogic_fbc"`) is documented
  separately in a follow-up design spec informed by edgefirst-camera,
  HAL, and fusion-trainer consumer input.
- New topics introduced by consumers of this schema should omit the
  `rt/` prefix (e.g. `camera/frame`); the prefix is optional and is
  being phased out project-wide.

## [3.0.0] - 2026-04-10

This release introduces a header-only C++17 wrapper around the C API and
refactors the way embedded child messages (boxes inside Detect/Model, masks
inside Model) are exposed. The refactor is **source- and binary-incompatible**
for C code that called `ros_box_as_cdr` or `ros_mask_as_cdr`, and the shared
library SONAME changes from `libedgefirst_schemas.so.2` to
`libedgefirst_schemas.so.3`. See the Migration section below.

### Added

- **C++ header-only wrapper** — `include/edgefirst/schemas.hpp` provides a
  complete C++17 API over the C library: RAII move-only view types
  (`ImageView`, `HeaderView`, `DetectView`, `ModelView`, …) and owning types
  (`Image`, `Header`, `Mask`, `DmaBuffer`, …), `expected<T, Error>` for all
  fallible operations (no exceptions), and range-based iteration for array
  children (`for (auto box : det.boxes()) { ... }`). Links against the same
  `libedgefirst_schemas.so` as the C API — no separate library to build.
- **Indexed child accessors in the C API** — three new functions return
  parent-borrowed child handles, replacing the previous bulk-array pattern
  and enabling the new C++ iteration API:
  - `ros_detect_get_box(view, i)` — returns a `const ros_box_t*` borrowed
    from the parent Detect.
  - `ros_model_get_box(view, i)` — same, for Model.
  - `ros_model_get_mask(view, i)` — returns a `const ros_mask_t*` borrowed
    from the parent Model.
  Returned pointers are owned by the parent handle; do **not** pass them to
  `ros_box_free()` / `ros_mask_free()`. Lifetime is tied to both the parent
  handle and the parent's CDR buffer. See [CAPI.md § Memory Management,
  Rule 5](CAPI.md#memory-management).
  - **`ros_box_get_track_created_sec(box)` / `ros_box_get_track_created_nanosec(box)`**
    — expose the box's track_created timestamp. Previously populated in the
    view but lacked a C accessor. Also exposed via C++ `BoxView::track_created()`
    and `BorrowedBoxView::track_created()`.
- `examples/cpp/example.cpp` — C++ example mirroring the C example:
  CdrFixed encode/decode, buffer-backed encode/view round-trips, zero-copy
  field access via `std::string_view` / `span<const uint8_t>`.
- `include/edgefirst/stdlib/expected.hpp` — vendored tl::expected (CC0-1.0)
  providing `std::expected`-compatible semantics on C++17 targets. The
  wrapper picks up `std::expected` automatically on C++23+. Namespace unified
  under `edgefirst::stdlib::` (renamed from upstream `tl::`).
- `include/edgefirst/stdlib/span.hpp` — minimal C++17 span shim that
  aliases `std::span` on C++20+ and supplies a two-pointer fallback otherwise.
  Moved from the former `detail/` directory; namespace unified under `edgefirst::stdlib::`.
- Makefile targets: `test-cpp`, `test-cpp-asan`, `test-cpp-xml`,
  `test-cpp-asan-xml` (Catch2-based C++ test suite with JUnit output),
  `example-cpp` (build the C++ example), and `install` (installs headers,
  C++ stdlib headers, and the versioned library symlink chain to `PREFIX`,
  default `/usr/local`).
- **`release()` ownership transfer on owning types** — `Header`, `Image`,
  `CompressedImage`, `CompressedVideo`, `DmaBuffer`, and `Mask` now expose
  `Released release() && noexcept` returning a raw `(data, size)` POD.
  This transfers ownership of the encoded CDR byte buffer out of the
  wrapper so the caller can hand it to a downstream sink (for example
  `zenoh::Bytes` with a custom deleter) without an intermediate copy.
  After `release()` the wrapper is empty and its destructor is a safe
  no-op. See `Released` in `schemas.hpp` and the "publishing" code
  example in the file-level Doxygen block for the zenoh-cpp pattern.

### Changed — BREAKING (for C callers caching child handles)

- `ros_box_t` and `ros_mask_t` are now **view types backed by the parent
  handle's CDR buffer** rather than standalone allocated structs. Child
  handles returned by `ros_detect_get_box`, `ros_model_get_box`, and
  `ros_model_get_mask` are owned by the parent and must not be freed
  independently. Top-level `ros_box_from_cdr` / `ros_mask_from_cdr` (decoding
  a standalone box or mask CDR slice) continue to work and still produce
  caller-owned handles that **must** be freed with `ros_box_free` /
  `ros_mask_free`.

### Removed — BREAKING

- `ros_box_as_cdr` and `ros_mask_as_cdr` are removed. Embedded child boxes
  and masks live inside the parent's CDR buffer and have no independent CDR
  encoding; producing one would require **re-encoding** the child into a
  fresh allocation, which violates the library's zero-copy contract.

  **Migration.** If you previously called `ros_box_as_cdr(box)` or
  `ros_mask_as_cdr(mask)` to forward an embedded child as a standalone
  message, forward the **entire parent** instead and let downstream code
  iterate it:

  ```c
  // Before (2.2.x):
  size_t box_len;
  const uint8_t* box_cdr = ros_box_as_cdr(box, &box_len);
  publish("topic/box", box_cdr, box_len);   // no longer possible

  // After (3.0.0):
  size_t parent_len;
  const uint8_t* parent_cdr = ros_detect_as_cdr(detect, &parent_len);
  publish("topic/detect", parent_cdr, parent_len);
  // Subscribers iterate with ros_detect_get_box(detect, i).
  ```

  Standalone box/mask CDR slices (decoded with `ros_box_from_cdr` /
  `ros_mask_from_cdr`) remain fully supported; only the *embedded-child*
  forwarding path is affected.

  **Trade-offs to note:**
  - **Wire size**: forwarding the full parent CDR carries the header, other
    boxes, and any additional parent-level fields — a multiplier of N× for a
    Detect with N boxes compared to the old single-box path.
  - **Iteration order**: subscribers must now iterate to find the specific
    box they want; if the old pattern relied on "the i-th message is the i-th
    box", consider encoding the box index into a sidecar field or splitting
    topics by box.

### Fixed

- Stale `@param data` doc comments on all `ros_<type>_from_cdr` functions
  (both `src/ffi.rs` and `include/edgefirst/schemas.h`) now correctly read
  "borrowed; must outlive the returned handle" instead of the previous
  misleading "copied internally" wording.
- **Zero-copy blob getters and `as_cdr` functions now initialize `*out_len = 0`
  on the NULL-view path** — previously left uninitialized, which could lead to
  undefined behavior in defensively-written C callers. Affects ~25 functions
  including `ros_image_get_data`, `ros_mask_get_data`, all 15 `ros_*_as_cdr`
  variants, and `ros_radar_cube_get_layout`.
- **Detect/Model decode is now single-pass** — the FFI layer previously walked
  the CDR buffer twice (once during validation in `Detect::from_cdr`, again in
  `inner.boxes()` to materialize child views for the `ros_detect_get_box` /
  `ros_model_get_box` / `ros_model_get_mask` accessors). The FFI now uses new
  crate-private `from_cdr_collect_boxes` / `from_cdr_collect_children` helpers
  that fuse the two passes. Typical speedup proportional to the number of
  boxes/masks per message.
- **Testing and documentation improvements:**
  - Comprehensive Doxygen coverage in `schemas.hpp` (~280 per-symbol briefs).
  - Doxyfile + `make docs` target (new).
  - Runtime `ChildRange` iteration tests replace the previous compile-time stubs.
  - Pointer-identity zero-copy coverage extended to 8 types.
  - Real move-semantics tests for 13 view types (replaced stub tests).
  - CI matrix extended to `[gcc, clang] × [c++17, c++20]`.

### Notes for downstream packagers

This release removes exported symbols (`ros_box_as_cdr`, `ros_mask_as_cdr`).
Per SemVer 2.0.0 this mandates a MAJOR version bump; the library's SONAME is
therefore bumped from `libedgefirst_schemas.so.2` to `libedgefirst_schemas.so.3`.
Distribution packagers maintaining their own SOVERSION mapping should treat
3.0.0 as ABI-incompatible with 2.2.x. The 2.x and 3.x series can coexist on
disk via the distinct SONAMEs.

## [2.2.1] - 2026-04-10

### Fixed

- C library SOVERSION layout now follows the standard GNU/Linux convention.
  The Makefile previously created a single backwards symlink
  (`libedgefirst_schemas.so.MAJOR → libedgefirst_schemas.so`) and never
  produced the intermediate `.so.MAJOR.MINOR` / `.so.MAJOR.MINOR.PATCH`
  names. Both `make lib` and the release workflow now produce the chained
  layout `libedgefirst_schemas.so → .so.MAJOR → .so.MAJOR.MINOR →
  .so.MAJOR.MINOR.PATCH` with the fully-qualified name as the real file.
- `CAPI.md` installation snippet now installs the versioned file and the
  full symlink chain (the previous instructions copied only `.so`, leaving
  the loader unable to resolve the embedded SONAME without a manual
  `ldconfig` fixup).

## [2.2.0] - 2026-03-22

### Added

- Type-coercing field reads via `PointFieldType::read_as_f64` — converts any stored
  numeric type to `f64` without precision loss; used by all coercion methods below
- `FieldDesc::read_as_f64` and `read_as_f32` — per-field coercion from any
  `PointFieldType` to a common float target (`f32` may lose precision for
  `Int32`/`Uint32` >24-bit or `Float64` narrowing)
- Public accessors `DynPoint::cloud()` and `DynPoint::data()` enabling zero-overhead
  hot-loop pattern with pre-resolved field descriptors
- Convenience methods on `DynPoint`: `read_as_f64` and `read_as_f32`
  (name-based coercing reads)
- Bulk coercing methods on `DynPointCloud`: `gather_as_f64` and `gather_as_f32`

## [2.1.0] - 2026-03-20

### Added

- Zero-copy `pointcloud` access layer over PointCloud2 data buffers (`sensor_msgs::pointcloud`)
  - `DynPointCloud`: Runtime field inspection with dynamic typed access to individual point fields
  - `DynPoint`: Zero-copy single-point view with by-name and pre-resolved descriptor access
  - `PointCloud<P>`: Compile-time typed access via the `Point` trait and `define_point!` macro
  - `PointFieldIter`: Non-allocating iterator over PointCloud2 field descriptors
  - `FieldDesc`, `PointFieldType`, and `PointCloudError` public types
- Convenience methods on `PointCloud2`: `as_dyn_cloud()` and `as_typed_cloud::<P>()`
  for direct construction of pointcloud views from a decoded message
- Complete `DynPoint` accessor coverage for all scalar types:
  `read_f32`, `read_f64`, `read_u8`, `read_u16`, `read_u32`,
  `read_i8`, `read_i16`, `read_i32` (plus `_at` variants)
- Bulk `gather_*` methods for all scalar types on `DynPointCloud`
- PointCloud access benchmarks for `DynPointCloud` and typed `PointCloud<P>`

### Changed

- Refactored `sensor_msgs` into a multi-file module (`sensor_msgs/mod.rs` +
  `sensor_msgs/pointcloud.rs`). Existing `use edgefirst_schemas::sensor_msgs::*`
  imports are unaffected.

## [2.0.1] - 2026-03-12

### Added

- C API functions for ModelInfo labels access: `ros_model_info_get_labels_len()`, `ros_model_info_get_label()`
- C API functions for ModelInfo shape access: `ros_model_info_get_input_shape()`, `ros_model_info_get_output_shape()`
- `CdrCursor::check_seq_count()` for sequence count validation against buffer bounds
- Golden CDR test data for multi-box Detect, ModelInfo with labels, and empty ModelInfo
- Comprehensive edge case tests for ModelInfo labels and DetectBox variable-length fields

### Changed

- Reduced ModelInfo offset table from 7 to 6 entries (removed unused slot)
- Reduced FoxgloveImageAnnotation offset table from 3 to 2 entries (removed unused slot)

## [2.0.0] - 2026-03-10

### Changed — BREAKING

**Replaced serde-based CDR with zero-copy CDR infrastructure.**

This release replaces the `serde` + `cdr` crate serialization with a hand-written
zero-copy CDR1-LE implementation. Wire format is unchanged — existing CDR
buffers from ROS 2 DDS remain fully compatible. The Rust and C APIs have changed
substantially.

**Removed:**
- `serde`, `cdr`, and `serde_derive` dependencies
- `src/serde_cdr.rs` module (replaced by `src/cdr.rs`)
- `Serialize` / `Deserialize` derives on all message types
- Owned-field struct constructors (replaced by buffer-backed generics)

**Added:**
- `cdr` module — `CdrCursor`, `CdrWriter`, `CdrSizer`, `CdrFixed` trait,
  `encode_fixed` / `decode_fixed` helpers, zero-copy `rd_*` / `wr_*` primitives
- Buffer-backed generic types `Type<B: AsRef<[u8]>>` for all variable-length
  messages — zero-copy `from_cdr(&[u8])` borrows the buffer with no allocation
- `CdrFixed` implementations for all fixed-size types (Time, Duration, Vector3,
  Quaternion, Pose, ColorRGBA, NavSatStatus, RegionOfInterest, etc.)
- O(1) field accessors via pre-computed offset tables
- Zero-copy typed array views (`&[i16]`, `&[f32]`, `&[f64]`) on LE targets
- IMU FFI accessors: orientation, angular velocity, linear acceleration, and
  all three covariance matrices now accessible from C API
- 46 golden CDR test files generated by Python pycdr2 for cross-language
  validation (3-phase: decode, construct byte-identical, modify roundtrip)
- `CdrError` type with proper `Result`-based error handling throughout:
  `encode_fixed`, buffer-backed constructors, and setter methods all return
  `Result`. Zero panics in production library code — validated-buffer
  accessors use `expect()` with documented invariants.
- [`CAPI.md`](CAPI.md) — comprehensive C API reference with architecture
  overview, memory management rules, error handling, full API signatures
  for all types, and working examples
- Updated `examples/c/example.c` to demonstrate the new zero-copy C API

**Performance** (measured on Cortex-A53 @ 1.8 GHz, see [`BENCHMARKS.md`](BENCHMARKS.md)):
- Zero-copy borrow (`from_cdr`) in 38–166 ns regardless of payload size —
  applies to both Rust and C APIs; the C FFI `ros_*_from_cdr()` borrows the
  caller's buffer directly with no intermediate copy
- Rust encoding 270–530× faster than Python pycdr2 serialization
- Rust decoding effectively unbounded speedup for large payloads — 1.8 billion×
  faster for 1.5 MB radar cubes, growing proportionally with message size

### Migration Guide

**Rust API — CdrFixed types (Time, Vector3, Pose, etc.):**

```rust
// Before (1.x): serde-based
use edgefirst_schemas::serde_cdr;
let bytes = serde_cdr::to_cdr_bytes(&time).unwrap();
let time: Time = serde_cdr::from_cdr_bytes(&bytes).unwrap();

// After (2.0): CdrFixed
use edgefirst_schemas::cdr;
let bytes = cdr::encode_fixed(&time)?;
let time: Time = cdr::decode_fixed(&bytes)?;
```

**Rust API — Buffer-backed types (Image, PointCloud2, etc.):**

```rust
// Before (1.x): Owned structs with serde
let img = sensor_msgs::Image {
    header: Header { stamp, frame_id: "camera".into() },
    height: 480, width: 640, encoding: "rgb8".into(),
    is_bigendian: 0, step: 1920, data: pixel_data,
};
let bytes = serde_cdr::to_cdr_bytes(&img).unwrap();
let decoded: Image = serde_cdr::from_cdr_bytes(&bytes).unwrap();

// After (2.0): Buffer-backed constructors
let img = sensor_msgs::Image::new(
    stamp, "camera", 480, 640, "rgb8", 0, 1920, &pixel_data,
)?;
let bytes = img.to_cdr();                          // Get the CDR buffer
let decoded = sensor_msgs::Image::from_cdr(&bytes).unwrap(); // Zero-copy borrow
let height = decoded.height();                     // O(1) accessor
```

**Rust API — Zero-copy borrow (new capability):**

```rust
// Borrow existing bytes — no allocation, no copy
let view = sensor_msgs::Image::from_cdr(cdr_slice)?;
let pixels: &[u8] = view.data();     // Points into cdr_slice
let frame: &str = view.frame_id();   // Points into cdr_slice
```

**C API — Completely refactored for zero-copy CDR:**

The C API has been redesigned around the same zero-copy CDR patterns as the
Rust API. The old `_new()` + `_set_*()` + `_serialize()` mutable-handle
pattern is replaced by:

- **CdrFixed types** — `ros_<type>_encode(buf, cap, &written, ...fields)` /
  `ros_<type>_decode(data, len, ...out_fields)` directly into caller buffers.
  No opaque handles, no heap allocation.
- **Buffer-backed types** — `ros_<type>_encode(...)` for construction,
  `ros_<type>_from_cdr(data, len)` for decoding into immutable view handles,
  `ros_<type>_get_*()` for O(1) field access.
- **Borrowed pointers** — String and byte-array getters now return `const`
  pointers into the handle's internal buffer. Do **not** `free()` them
  (the old API returned allocated copies that required `free()`).
- **`ros_bytes_free()`** — Encode output must be freed with `ros_bytes_free()`
  instead of `free()`.

See [`CAPI.md`](CAPI.md) for the complete C API reference, memory management
rules, error handling conventions, and working examples.

**Python API — No changes.** Python bindings continue to use pycdr2 for
CDR serialization. No migration required.

## [1.5.5] - 2026-02-17

### Fixed
- Release workflow race condition: build jobs now upload workflow artifacts
  and a final job publishes the release in one shot
- Changelog extraction in release workflow not matching version due to `v` prefix

## [1.5.4] - 2026-02-17

### Added
- pkg-config configuration file (`edgefirst-schemas.pc`) included in C API release packages
- SOVERSION symlinks in C API release packages (GNU/Linux `libedgefirst_schemas.so.1` convention)

### Fixed
- C API test builds failing due to missing SONAME symlink for runtime linker

## [1.5.3] - 2026-02-12

### Fixed
- **C API: Replaced bulk array getters with indexed accessors for opaque types**
  - `ros_point_cloud2_get_fields()` → `ros_point_cloud2_get_num_fields()` + `ros_point_cloud2_get_field_at()`
  - `edgefirst_detect_get_boxes()` → `edgefirst_detect_get_num_boxes()` + `edgefirst_detect_get_box_at()`
  - Returning `Vec::as_ptr()` for opaque Rust-layout structs (containing `String`) was unusable from C
    since forward-declared types have unknown `sizeof`, making `fields[i]` impossible to compile
- C API: Fixed out-of-bounds handling in `edgefirst_detect_get_box_at()` and
  `ros_point_cloud2_get_field_at()` — replaced `assert!` (which panics across FFI, causing UB)
  with safe `match .get(index)` returning `NULL`
- C API: Added 5 missing declarations to `schemas.h`:
  - `foxglove_point_annotations_get_num_outline_colors()`
  - `foxglove_point_annotations_get_outline_color_at()`
  - `foxglove_point_annotations_set_outline_colors()`
  - `foxglove_point_annotations_clear_outline_colors()`
  - `edgefirst_detect_clear_boxes()`
- Fixed `EdgeFirstDmaBuf` → `EdgeFirstDmaBuffer` type name in C example (`examples/c/example.c`)

## [1.5.2] - 2026-01-27

### Fixed
- C API errno handling now uses portable `errno` crate for Windows and macOS support
- Replaced platform-specific set_errno implementations with cross-platform solution

## [1.5.1] - 2026-01-26

### Fixed
- Fixed PyPI trusted publisher configuration for automated releases
- Removed redundant ROS workflow trigger on tag pushes
- Added cross-workflow Debian package publishing to GitHub Releases

## [1.5.0] - 2026-01-26

### Added

**C API Expansion:**
- Comprehensive C FFI bindings for all message types via `libedgefirst_schemas.so`
- New C header `include/edgefirst/schemas.h` with full Doxygen documentation
- Support for 60+ message types across all namespaces:
  - `builtin_interfaces`: Time, Duration
  - `std_msgs`: Header, ColorRGBA
  - `geometry_msgs`: Vector3, Point, Point32, Quaternion, Pose, Pose2D, PoseStamped, Transform, TransformStamped, Twist, TwistStamped, Accel, AccelStamped, Wrench, WrenchStamped, Inertia, InertiaStamped, Polygon, PolygonStamped, PoseArray, PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovariance, TwistWithCovarianceStamped, AccelWithCovariance, AccelWithCovarianceStamped
  - `sensor_msgs`: Image, PointCloud2, PointField, CompressedImage, CameraInfo, RegionOfInterest, Imu, NavSatFix, NavSatStatus, Range, Temperature, RelativeHumidity, Illuminance, FluidPressure, BatteryState, JointState, Joy, JoyFeedback, JoyFeedbackArray, TimeReference, LaserScan, MultiEchoLaserScan, LaserEcho, MagneticField
  - `nav_msgs`: Odometry, Path, OccupancyGrid, MapMetaData, GridCells
  - `rosgraph_msgs`: Log, Clock
  - `foxglove_msgs`: CompressedVideo
  - `edgefirst_msgs`: DmaBuf, RadarCube, RadarInfo, Mask, Model, ModelInfo, Detect, Box, Track
- Service message types: SetBool, Trigger, Empty
- Complete unit tests for C API using Criterion test framework

**Benchmark Suite:**
- Comprehensive CDR serialization benchmarks for all heavy message types
- On-target benchmark execution on NXP i.MX 8M Plus (Cortex-A53)
- SmartMicro DRVEGRD radar configurations:
  - DRVEGRD-169 (Corner Radar): Ultra-Short, Short, Medium, Long modes
  - DRVEGRD-171 (Front Radar): Short, Medium, Long modes
- Image benchmarks with RGB8, YUYV, NV12 encodings at VGA/HD/FHD
- Mask benchmarks at 320/640/1280 with 8 and 32 classes
- CompressedMask benchmarks with zstd compression
- DmaBuf zero-copy reference benchmarks (nanosecond scale)
- PointCloud2 benchmarks from sparse (1K) to very dense (131K points)

**Schema Registry:**
- Runtime schema name registry (`schema_registry.rs`, `registry.py`) for type lookup by ROS 2 schema name

**MCAP Validation:**
- Rust MCAP integration tests for real-world CDR validation against hardware recordings
- Python MCAP tests (`test_mcap.py`) and schema registry tests (`test_schema_registry.py`)

**Release Packaging:**
- Pre-built C API binary packages for Linux x86_64 and aarch64 attached to GitHub Releases
- Archives include shared library (.so), static library (.a), C header, README, and LICENSE

**CI/CD Improvements:**
- QuickChart integration for grouped bar charts in GitHub Actions Summary
- Criterion JSON parsing for reliable benchmark data extraction
- EnricoMi/publish-unit-test-result-action for C API test reporting
- SonarCloud integration with Rust and C coverage reporting via llvm-cov
- On-target ARM64 benchmark execution via self-hosted runner

### Changed
- Benchmark charts now auto-scale Y-axis to actual data
- DmaBuf benchmarks display in nanoseconds for precision
- All chart values show 2 decimal places for readability
- Updated `sonar-project.properties` to include C API sources and tests

### Fixed
- Criterion `slope: null` handling for benchmarks without linear fit
- UTF-8 encoding issues in Mermaid charts (switched to QuickChart)
- Stale benchmark cache causing legacy names to appear

## [1.4.1] - 2025-11-18

### Changed
- **All packages now licensed under Apache-2.0**: Updated Rust crate, Python package, and ROS2 package metadata to reflect Apache-2.0 license across all distributions
- SBOM generation is now 3x faster (17s → 5-6s), reducing build times for downstream users
- Updated dependency scanning for improved security and compliance

### Fixed
- Improved reliability of license policy enforcement

## [1.4.0] - 2025-11-17

### Added

**Version Management:**
- Implemented cargo-release for automated version management across Rust, Python, and ROS2
- Added `release.toml` with pre-release-replacements for version synchronization
- Created `.github/scripts/check_version_sync.sh` to verify version consistency
- Added dedicated `version-check.yml` GitHub Actions workflow to block PRs with version mismatches

**CI/CD Improvements:**
- Created automated GitHub Release workflow (`release.yml`)
- Fixed GitHub Actions workflow syntax errors (branches array format)
- Removed version manipulation from rust.yml and pypi.yml (now managed by cargo-release)
- Added automated CHANGELOG extraction for GitHub Releases

**Documentation:**
- Updated CONTRIBUTING.md with comprehensive cargo-release workflow documentation
- Added visual workflow diagram showing developer actions vs automated steps
- Clarified that GitHub Release is auto-created by tag push, not manually
- Added troubleshooting section for failed releases

**Package Updates:**
- Updated ROS2 package.xml to use real version instead of 0.0.0 placeholder
- Updated debian/rules to extract version from package.xml instead of git tags

### Changed
- Python version now sourced from `__init__.py` __version__ (industry standard)
- All three version sources (Cargo.toml, __init__.py, package.xml) now synchronized via cargo-release
- Release process now uses standard cargo-release steps: version, replace, commit, tag, push

### Fixed
- Version synchronization issues between Rust, Python, and ROS2 packages
- GitHub Actions workflow schema validation errors

## [1.3.1] - 2025-05-14

### Changed
- Optimized PointCloud2 decode to decode entire point in one call instead of one call per field
- Use named tuple for even faster decode
- Added decoding fields where count > 1

## [1.3.0] - 2025-05-12

### Added
- PointCloud2 parsing utilities moved from samples to schemas for better reuse
- Boxes field to mask in edgefirst_msgs.py

### Changed
- Migrated to pyproject.toml for Python packaging
- Removed Git hash from version for Python
- More explicit version string parsing for Python __init__.py

### Fixed
- Rust formatting fixes
- Python build module installation
- ModelInfo added to Python schemas
- LocalTime import issues
- Import error: cannot import name 'default_field'
- Added missing PCD fields to p.fields dictionary

## [1.2.11] - 2025-05-06

### Fixed
- Explicitly added edgefirst.schemas module to Python package

## [1.2.10] - 2025-05-04

### Changed
- Removed edgefirst/__init__.py to avoid issues with multiple packages using the edgefirst namespace

## [1.2.9] - 2025-05-03

### Fixed
- RadarInfo Duration initialization

## [1.2.8] - 2025-05-03

### Fixed
- RadarInfo and Model messages not using default_field for Header initialization

## [1.2.7] - 2025-04-30

### Added
- Boxed boolean to mask for instanced segmentation

### Fixed
- Comment about unused encoding
- Duration not using ROS Duration type

## [1.2.6] - 2025-03-27

### Changed
- Formatting and documentation updates

## [1.2.5] - 2025-03-27

### Added
- DmaBuffer message schema and Python API
- Python variants of all provided messages
- Float16 datatype support
- Additional datatypes for model_info
- ModelInfo to Rust schemas
- Custom Detect schema for Python
- CompressedImage schema for JPEG handling
- CompressedVideo schema
- CameraInfo and ImageAnnotation schema for Python parser
- Time conversion traits to u64

### Changed
- Renamed DetectBoxes2D to Detect
- Changed label type from String to u32, then back to String
- Moved Box and Track to separate files
- Corrected Detect to be a sequence of boxes
- Updated comments in detect schema
- Removed is_tracked field from Detect
- Re-organized Rust library to bring all name_msgs into base module

### Fixed
- Fixed edgefirst typo
- Moved detect.py to correct directory
- Updated setup.py to include all Python messages
- Python package long description using README.md
- Python build issues with wheel generation
- Removed auxiliary files from ROS2 schemas not required for this project

[Unreleased]: https://github.com/EdgeFirstAI/schemas/compare/v3.1.0...HEAD
[3.1.0]: https://github.com/EdgeFirstAI/schemas/compare/v3.0.0...v3.1.0
[3.0.0]: https://github.com/EdgeFirstAI/schemas/compare/v2.2.1...v3.0.0
[2.2.1]: https://github.com/EdgeFirstAI/schemas/compare/v2.2.0...v2.2.1
[2.2.0]: https://github.com/EdgeFirstAI/schemas/compare/v2.1.0...v2.2.0
[2.1.0]: https://github.com/EdgeFirstAI/schemas/compare/v2.0.1...v2.1.0
[2.0.1]: https://github.com/EdgeFirstAI/schemas/compare/v2.0.0...v2.0.1
[2.0.0]: https://github.com/EdgeFirstAI/schemas/compare/v1.5.5...v2.0.0
[1.5.5]: https://github.com/EdgeFirstAI/schemas/compare/v1.5.4...v1.5.5
[1.5.4]: https://github.com/EdgeFirstAI/schemas/compare/v1.5.3...v1.5.4
[1.5.3]: https://github.com/EdgeFirstAI/schemas/compare/v1.5.2...v1.5.3
[1.5.2]: https://github.com/EdgeFirstAI/schemas/compare/v1.5.1...v1.5.2
[1.5.1]: https://github.com/EdgeFirstAI/schemas/compare/v1.5.0...v1.5.1
[1.5.0]: https://github.com/EdgeFirstAI/schemas/compare/v1.4.1...v1.5.0
[1.4.1]: https://github.com/EdgeFirstAI/schemas/compare/v1.4.0...v1.4.1
[1.4.0]: https://github.com/EdgeFirstAI/schemas/compare/v1.3.1...v1.4.0
[1.3.1]: https://github.com/EdgeFirstAI/schemas/compare/v1.3.0...v1.3.1
[1.3.0]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.11...v1.3.0
[1.2.11]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.10...v1.2.11
[1.2.10]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.9...v1.2.10
[1.2.9]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.8...v1.2.9
[1.2.8]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.7...v1.2.8
[1.2.7]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.6...v1.2.7
[1.2.6]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.5...v1.2.6
[1.2.5]: https://github.com/EdgeFirstAI/schemas/releases/tag/v1.2.5
