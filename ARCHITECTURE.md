# EdgeFirst Perception Schemas - Architecture

**Version:** 3.5.0
**Last Updated:** June 2026
**Target Audience:** Developers implementing or integrating EdgeFirst Perception

---

## Table of Contents

- [Overview](#overview)
- [Design Principles](#design-principles)
- [Core Components](#core-components)
- [Message Serialization](#message-serialization)
- [Language Bindings](#language-bindings)
- [PointCloud Access Layer](#pointcloud-access-layer)
- [Zenoh Communication](#zenoh-communication)
- [ROS2 Interoperability](#ros2-interoperability)
- [Zero-Copy DMA Buffers](#zero-copy-dma-buffers)
- [Source Code Reference](#source-code-reference)

---

## Overview

EdgeFirst Perception Schemas is a **schema library** providing message type definitions and high-performance language bindings for the EdgeFirst Perception middleware. It serves as the communication contract between EdgeFirst Perception services.

### Key Characteristics

- **Schema-first design**: Message definitions drive code generation
- **Multi-language support**: Rust (native), C (FFI), C++ (header wrappers), and Python (PyO3)
- **Standards-based**: ROS2 CDR serialization, ROS2 message compatibility
- **Transport-agnostic**: Works over Zenoh, can bridge to ROS2 DDS
- **Zero-copy capable**: DMA buffer sharing on embedded platforms

### Scope

This is **NOT** a middleware framework - it provides:
- ✅ Message type definitions (.msg files, IDL)
- ✅ Serialization/deserialization (CDR encoding)
- ✅ Language-specific bindings (Rust structs, Python dataclasses)

This is **NOT** included:
- ❌ Transport layer (provided by Zenoh)
- ❌ Service discovery (provided by Zenoh)
- ❌ Message routing (provided by Zenoh)

---

## Design Principles

### 1. ROS2 Compatibility Without ROS2 Dependency

**Goal**: Enable ROS2 message interchange without requiring ROS2 installation.

**Implementation**:
- Use standard ROS2 message definitions
- Implement CDR (Common Data Representation) serialization
- Compatible with ROS2 Humble Hawksbill LTS
- Can bridge to ROS2 via Zenoh ROS2 DDS plugin when needed

**Benefits**:
- Works on Windows, macOS, Linux
- No ROS2 installation required for most users
- Smaller deployment footprint
- Easier cross-platform development

### 2. Performance-First — Zero-Copy CDR

**Goal**: Minimize serialization overhead for real-time perception systems.

**Techniques**:
- **Serialization is a no-op**: Messages are constructed directly in-place within a buffer. The buffer *is* the serialized form.
- **Deserialization builds offset tables**: Receiving a CDR buffer constructs an offset table mapping variable-length fields to their positions. No data is copied.
- **Field access reads directly from the buffer**: Fixed-size fields use computed offsets; variable-length fields use the offset table.
- Zero-copy DMA buffer sharing on embedded platforms
- Compile-time type checking (Rust)
- Minimal allocations during hot paths

### 3. Hardware Integration

**Goal**: Enable efficient data flow from hardware accelerators (NPU, ISP, DSP).

**Approach**:
- DMA buffer file descriptor sharing
- Hardware-specific metadata (radar, camera)
- Platform-agnostic abstractions

---

## Core Components

### Message Schema Hierarchy

```
EdgeFirst Perception Schemas
├── ROS2 Common Interfaces (standard)
│   ├── std_msgs (Header, String, primitives)
│   ├── geometry_msgs (Pose, Transform, Twist, etc.)
│   ├── sensor_msgs (PointCloud2, Image, CameraInfo, Imu, etc.)
│   ├── nav_msgs (MapMetaData, GridCells, OccupancyGrid, Odometry, Path)
│   └── builtin_interfaces (Time, Duration)
│
├── Foxglove Schemas (visualization)
│   ├── SceneUpdate (3D visualization)
│   ├── ImageAnnotations
│   └── Grid, Pose markers
│
└── EdgeFirst Custom Messages (edge AI)
    ├── Detect (object detection results)
    ├── Box (2D bounding box)
    ├── Track (object tracking)
    ├── Tensor (dtype, shape, strides, quantization, colorimetry, planes)
    ├── TensorPlane (one plane; behind a handle, or inline)
    ├── TensorStamped (Header + seq + Tensor)
    ├── CameraFrame (Header + seq + Tensor; byte-identical to TensorStamped)
    ├── RadarCube (raw radar FFT data)
    ├── RadarInfo (radar configuration)
    ├── Model (inference metadata)
    └── ModelInfo (performance instrumentation)
```

### Language Binding Structure

The repository is a Cargo workspace with three member crates. The split exists
to keep each binding's build artifacts isolated — most importantly, the C
library's `build.rs` (which emits `DT_SONAME = libedgefirst_schemas.so.MAJOR`)
runs only inside the capi crate, so it never leaks into the PyO3 wheel.

```
crates/
├── schemas/   pure-Rust rlib  (`edgefirst-schemas`)
├── capi/      C lib            (`edgefirst-schemas-capi` package,
│                                produces `libedgefirst_schemas.{a,so}`)
└── python/    PyO3 wheel       (`edgefirst-schemas-python` package,
                                 importable as `edgefirst.schemas`)
```

**Rust schemas crate** (`crates/schemas/src/`):
```
crates/schemas/src/
├── lib.rs                  # Public API, re-exports
├── cdr.rs                  # Zero-copy CDR1-LE: CdrCursor, CdrWriter, CdrSizer, CdrFixed
├── std_msgs.rs             # ROS2 standard messages
├── geometry_msgs.rs        # ROS2 geometry
├── sensor_msgs/            # ROS2 sensor messages
│   ├── mod.rs              # Image, PointCloud2, CameraInfo, Imu, etc.
│   └── pointcloud.rs       # Zero-copy PointCloud access (DynPointCloud, PointCloud<P>)
├── nav_msgs.rs             # ROS2 navigation
├── builtin_interfaces.rs   # ROS2 time types
├── rosgraph_msgs.rs        # ROS2 graph (Clock)
├── foxglove_msgs.rs        # Foxglove visualization
├── edgefirst_msgs.rs       # EdgeFirst custom messages
├── tensor.rs               # Tensor / TensorPlane payload, plane codec, builders
├── schema_registry.rs      # Runtime schema name registry
└── service.rs              # ROS2 service wrapper
```

**C library crate** (`crates/capi/`):
```
crates/capi/
├── src/lib.rs              # #[no_mangle] ros_* / cdr_* FFI surface (was src/ffi.rs)
├── src/tensor.rs           # Tensor-family C bindings (was src/ffi/tensor.rs)
├── build.rs                # cargo:rustc-cdylib-link-arg=-Wl,-soname,...
├── include/edgefirst/
│   ├── schemas.h           # Hand-maintained C header
│   ├── schemas.hpp         # C++ wrapper
│   └── stdlib/{expected,span}.hpp
├── edgefirst-schemas.pc.in # pkg-config template
└── tests/{c,cpp}/          # Criterion-based C and C++ tests
```

**Python** (`edgefirst/schemas/`):
```
edgefirst/schemas/
├── __init__.py              # Public API, decode_pcd utility
├── std_msgs.py
├── geometry_msgs.py
├── sensor_msgs.py
├── nav_msgs.py
├── builtin_interfaces.py
├── foxglove_msgs.py
└── edgefirst_msgs.py
```

**Message Definitions** (`edgefirst_msgs/msg/`):
```
edgefirst_msgs/msg/
├── Box.msg
├── Detect.msg
├── Track.msg
├── CameraFrame.msg
├── Tensor.msg
├── TensorPlane.msg
├── TensorStamped.msg
├── RadarCube.msg
├── RadarInfo.msg
├── Model.msg
├── ModelInfo.msg
├── Mask.msg
├── Date.msg
└── LocalTime.msg
```

---

## Message Serialization

### CDR (Common Data Representation)

EdgeFirst uses **CDR** for binary serialization, the same format as ROS2 DDS.

#### CDR Encoding Rules

- **Primitives**: Little-endian by default
- **Alignment**: Natural alignment (4-byte for int32, 8-byte for float64)
- **Strings**: Length-prefixed UTF-8
- **Arrays**: Length-prefixed elements
- **Structs**: Sequential field encoding

**Example — Fixed-size type (CdrFixed):**

```rust
// Fixed-size types use repr(C) — their memory layout IS the CDR encoding.
use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::cdr;

let time = Time::new(1234567890, 123456789);
let bytes = cdr::encode_fixed(&time).unwrap();
let decoded: Time = cdr::decode_fixed(&bytes).unwrap();
```

**Example — Buffer-backed type (zero-copy):**

```rust
use edgefirst_schemas::std_msgs::Header;
use edgefirst_schemas::builtin_interfaces::Time;

// Construction writes directly into an internal buffer
let header = Header::new(Time::new(1, 0), "camera").unwrap();
let cdr_bytes = header.to_cdr();

// Deserialization builds an offset table — no data copied
let view = Header::from_cdr(&cdr_bytes).unwrap();
assert_eq!(view.frame_id(), "camera"); // reads directly from buffer
```

#### Why CDR?

- ✅ **ROS2 compatible**: Same encoding as ROS2 DDS
- ✅ **Well-specified**: OMG standard
- ✅ **Efficient**: Compact binary representation
- ✅ **Language-agnostic**: Works across Rust, Python, C
- ✅ **Zero-copy**: Offset-table deserialization with no allocation

### Serialization Libraries

**Rust**:
- Custom zero-copy CDR1-LE implementation (`crates/schemas/src/cdr.rs`):
  `CdrCursor` (reader), `CdrWriter` (builder), `CdrSizer` (size calculator), `CdrFixed` (trait for fixed-size types)
- No `serde` dependency — all serialization is hand-written for zero-copy

**C API**:
- FFI bindings via `cbindgen` — same zero-copy CDR under the hood
- Header hand-maintained at `crates/capi/include/edgefirst/schemas.h` (cbindgen is wired for future auto-generation but not invoked today)

**Python**:
- PyO3 compiled Rust extension (built via `maturin`) — CDR logic lives in Rust; no separate CDR library is needed at runtime
- `pycdr2` is used only as an independent golden-fixture oracle in the test suite, not at runtime

### Publisher Buffer Reuse (Builder Pattern)

Buffer-backed message types expose a `Foo::builder()` entry point that
returns a `FooBuilder<'a>`. The builder holds field values (strings as
`Cow<'a, str>`, bulk data as `&'a [u8]`) and offers three finalizers:

- `build() -> Result<Foo<Vec<u8>>, CdrError>` — allocates a fresh buffer.
  Drop-in replacement for the legacy `Foo::new(...)` constructor.
- `encode_into_vec(&mut Vec<u8>) -> Result<(), CdrError>` — resizes the
  caller's `Vec` to exactly the encoded size and writes into it. Reuses
  existing allocation when capacity suffices. After return, `buf.len()`
  is the CDR size and `&buf[..]` is a complete CDR message.
- `encode_into_slice(&mut [u8]) -> Result<usize, CdrError>` — writes into
  a fixed-capacity slice, returning the number of bytes written. Errors
  with `BufferTooShort` when the slice is smaller than the required size;
  nothing is mutated in the error case.

Setters return `&mut Self`, supporting both the chained one-shot style
and the named-builder reuse style without a separate consuming variant:

```rust
// One-shot: chained on the temporary (same mechanism as std::process::Command)
let img = Image::builder()
    .stamp(now())
    .frame_id("camera")
    .height(h).width(w).encoding("rgb8").step(stride)
    .data(&pixels)
    .build()?;

// Reuse: set metadata once, overwrite hot fields per frame, reuse cdr_buf
let mut b = Image::builder();
b.frame_id("camera").height(h).width(w).encoding("rgb8").step(stride);
let mut cdr_buf = Vec::new();
loop {
    b.stamp(now()).data(&pixels);
    b.encode_into_vec(&mut cdr_buf)?;
    publish(&cdr_buf);
}
```

As of 3.2.0 the builder pattern is applied to **every** buffer-backed
message type in the crate (30+ types across `std_msgs`, `sensor_msgs`,
`nav_msgs`, `edgefirst_msgs`, `foxglove_msgs`). The legacy `Foo::new(...)`
constructors remain as `#[deprecated(since = "3.2.0")]` shims and are
scheduled for removal in 4.0. The C FFI exposes a parallel
`ros_<type>_builder_*` handle-based API with the same semantics.

**Scalar fast path (in-place setters).** The builder re-serialises the
whole buffer, which is wasteful when only a scalar field changes
between frames. Every fixed-size field on every buffer-backed message
has an in-place `set_*` mutator on `impl<B: AsRef<[u8]> + AsMut<[u8]>> Foo<B>`
that writes directly into the existing CDR buffer — no re-encoding.
For a 2 MB camera frame, `camera.set_stamp(now())` is ~8 byte writes
vs. the builder's full re-encode. Use the scalar setter when only
scalars change; use the builder when any variable-length field
(string, byte sequence, nested view) changes.

**Important:** the bulk-data borrow (`data: &'a [u8]`) must not alias the
destination `&mut Vec<u8>`. Typical publishers keep pixel / point-cloud
data in a separate allocation from the CDR buffer, which satisfies this
trivially. Aliasing is not detected at compile time.

---

## Language Bindings

### Rust Implementation

**Key Features**:
- Zero-copy CDR: serialization writes in-place, deserialization builds offset tables
- Buffer-backed generic types `Type<B: AsRef<[u8]>>` — `from_cdr()` borrows with no allocation
- `CdrFixed` implementations for all fixed-size types (direct memory-mapped CDR)
- Type-safe at compile time, `Result`-based error handling throughout

**Example (PointCloud2 — buffer-backed):**

```rust
use edgefirst_schemas::sensor_msgs::PointCloud2;

// Zero-copy deserialization — builds offset table, borrows buffer
let pcd = PointCloud2::from_cdr(cdr_bytes).unwrap();
let height = pcd.height();       // O(1) fixed-offset read
let data: &[u8] = pcd.data();   // Points into original buffer

// Field iteration — non-allocating CDR cursor walk
for field in pcd.fields_iter() {
    println!("{}: offset={}, datatype={}", field.name, field.offset, field.datatype);
}
```

**PointCloud Access Layer** (`crates/schemas/src/sensor_msgs/pointcloud.rs`):

Two-tier zero-copy access over PointCloud2 data buffers — see [PointCloud Access Layer](#pointcloud-access-layer) below.

### C API Prefix Convention

All 3.x C-API symbols share the `ros_*` prefix regardless of which message
namespace the type originates from — e.g. `ros_camera_info_t` (sensor_msgs),
`ros_compressed_video_t` (foxglove_msgs), `ros_detect_t` (edgefirst_msgs).
This is a historical artifact: early development treated every ROS-ecosystem
type as belonging to a single `ros_` namespace.

The correct convention is a per-namespace prefix matching the `.msg` source:
`sensor_*`, `foxglove_*`, `edgefirst_*`, `geometry_*`, `std_*`. New C symbols
added during the 3.x line continue to use `ros_*` for within-release
consistency — mixing conventions inside 3.x would be worse than retaining the
wart. The tensor family (`ros_tensor_*`, `ros_camera_frame_*`) follows the
same `ros_*` convention for the same reason.

A full rename of all C symbols to their namespace-correct prefixes remains
planned for **4.0.0**, which is already a breaking boundary (the DmaBuffer and
CameraPlane removal, the CameraFrame redefinition, and a SOVERSION bump).
Rust, C++, and Python surfaces already use per-namespace module paths
(`edgefirst_schemas::foxglove_msgs::...`) and are unaffected by the C API
rename.

### Python Implementation

**Key Features**:
- PyO3 compiled Rust extension — Rust CDR logic is exposed directly as a native Python module via `maturin`; no `pycdr2` runtime dependency
- Type hints for IDE support
- Named tuples for decoded point clouds

**Example (PointCloud2 decode):**

```python
# edgefirst/schemas/__init__.py
def decode_pcd(pcd: PointCloud2) -> list[NamedTuple]:
    """Decodes PointCloud2 to list of named tuples."""
    # Parse field layout
    fields.sort(key=lambda f: f.offset)

    # Build struct format for decoding
    struct_format = endian_format + field_types...

    # Decode points
    Point_ = namedtuple("Point_", field_names)
    for i in range(pcd.height * pcd.width):
        p = Point_._make(struct.unpack_from(struct_format, data, offset))
        points.append(p)

    return points
```

**Accessing decoded points:**

```python
points = decode_pcd(point_cloud_msg)
for point in points:
    x, y, z = point.x, point.y, point.z
    rgb = point.rgb  # If RGB field exists
```

---

## PointCloud Access Layer

The `sensor_msgs::pointcloud` module provides zero-copy typed access over PointCloud2 data buffers. It sits above the raw PointCloud2 CDR type and provides two tiers of access:

### Two-Tier Design

**`DynPointCloud`** (runtime/dynamic tier):
- Resolves field metadata at construction time from PointCloud2 field descriptors
- Stores up to `MAX_FIELDS` (16) field descriptors in a fixed-size array — no heap allocation
- Access by field name (`read_f32("x")`) or pre-resolved descriptor (`read_f32_at(&desc)`)
- Suitable when the point layout is not known at compile time

**`PointCloud<P>`** (compile-time/static tier):
- Validates field layout against a user-defined `Point` type at construction time
- The `define_point!` macro generates `Point` implementations with compile-time offsets
- Direct struct field access (`point.x`) with no runtime overhead beyond `from_le_bytes`
- Suitable when the point layout is known at compile time

### Why Two Tiers?

| Scenario | Use |
|----------|-----|
| Sensor driver known, fixed layout (e.g., `x, y, z, intensity`) | `PointCloud<P>` — compile-time, fastest |
| Generic tool (visualizer, recorder), layout varies at runtime | `DynPointCloud` — flexible, still zero-copy |

Both tiers share no state and have minimal coupling. Neither copies data out of the PointCloud2 buffer — they read directly via `from_le_bytes` on small byte arrays.

The dynamic tier also provides **type-coercing access** via `FieldDesc::read_as_f64` and `read_as_f32`, which convert any `PointFieldType` to a common float target. This is useful when the same field (e.g., `vision_class`) has different storage types across services (UINT8, UINT16, or UINT32).

### Example

```rust
use edgefirst_schemas::define_point;
use edgefirst_schemas::sensor_msgs::PointCloud2;
use edgefirst_schemas::sensor_msgs::pointcloud::PointCloud;

define_point! {
    pub struct XyzPoint { x: f32 => 0, y: f32 => 4, z: f32 => 8 }
}

let pcd2 = PointCloud2::from_cdr(cdr_bytes).unwrap();

// Dynamic access (runtime field lookup)
let dyn_cloud = pcd2.as_dyn_cloud().unwrap();
let x_desc = dyn_cloud.field("x").unwrap();
for point in dyn_cloud.iter() {
    let x = point.read_f32_at(x_desc).unwrap(); // pre-resolved, avoids name lookup
}

// Typed access (compile-time offsets)
let cloud = pcd2.as_typed_cloud::<XyzPoint>().unwrap();
for point in cloud.iter() {
    println!("{}, {}, {}", point.x, point.y, point.z);
}
```

---

## Zenoh Communication

### Zenoh Integration

Zenoh is the primary transport for EdgeFirst Perception.

**Why Zenoh?**
- ✅ **Cross-platform**: Linux, Windows, macOS, embedded
- ✅ **High performance**: Sub-microsecond latency
- ✅ **Flexible**: Pub/sub, query, storage
- ✅ **ROS2 bridge**: Interoperates with ROS2 when needed
- ✅ **Discovery**: Automatic peer discovery
- ✅ **Routing**: Multi-hop, edge-to-cloud

### Topic Naming Convention

EdgeFirst Perception uses hierarchical topic naming with the `rt/` prefix (runtime topics):

```
rt/service/data_type
```

**Actual EdgeFirst Perception Topics:**

**Camera:**
- `rt/camera/h264` - H.264 compressed camera frames
- `rt/camera/dma` - Zero-copy DMA buffer frames
- `rt/camera/info` - Camera metadata (resolution, calibration)
- `rt/camera/jpeg` - JPEG compressed frames

**Radar:**
- `rt/radar/cube` - Raw radar FFT cube data
- `rt/radar/targets` - Detected radar targets
- `rt/radar/clusters` - Clustered radar detections
- `rt/radar/info` - Radar configuration metadata

**Sensors:**
- `rt/imu` - Inertial measurement unit data
- `rt/gps` - GPS location data

**Transforms:**
- `rt/tf_static` - Static coordinate frame transforms

### Message Flow Example

Real-world EdgeFirst Perception pipeline showing camera and radar data flow:

```mermaid
graph LR
    Camera[Camera Service] -->|Zenoh pub| H264["rt/camera/h264"]
    Camera -->|Zenoh pub| Info["rt/camera/info"]
    H264 -->|Zenoh sub| Perception[Perception Service]
    Info -->|Zenoh sub| Perception

    Radar[Radar Service] -->|Zenoh pub| Targets["rt/radar/targets"]
    Targets -->|Zenoh sub| Fusion[Sensor Fusion]
    Perception -->|Zenoh sub| Fusion

    Fusion -->|Zenoh pub| Results["rt/perception/objects"]
    Results -->|Zenoh sub| Studio[EdgeFirst Studio]
    Results -->|Zenoh sub| App[Application]
```

### Code Example (Rust + Zenoh)

```rust
use edgefirst_schemas::sensor_msgs::CompressedImage;
use edgefirst_schemas::builtin_interfaces::Time;
use zenoh::prelude::*;

// Publisher — construct directly into CDR buffer
let session = zenoh::open(config).await?;
let publisher = session.declare_publisher("rt/camera/h264").await?;

let img = CompressedImage::new(Time::new(1, 0), "camera", "h264", &frame_data)?;
publisher.put(img.to_cdr()).await?;

// Subscriber — zero-copy decode
let subscriber = session.declare_subscriber("rt/camera/h264").await?;
while let Ok(sample) = subscriber.recv_async().await {
    let img = CompressedImage::from_cdr(&sample.payload)?; // borrows payload
    let data: &[u8] = img.data(); // zero-copy reference
    process_image(data);
}
```

---

## ROS2 Interoperability

### Zenoh ROS2 DDS Bridge

EdgeFirst Perception does NOT require ROS2, but can interoperate when needed via the [Zenoh ROS2 DDS Bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds).

**Architecture:**

```mermaid
graph TD
    EF[EdgeFirst Perception Services] <-->|Zenoh| Bridge[ROS2 Bridge]
    Bridge <-->|DDS| ROS2[ROS2 Nodes<br/>rviz, rqt, etc]
```

**Use Cases:**
- Visualize EdgeFirst data in RViz
- Record MCAP files with ROS2 tools
- Integrate with existing ROS2 systems
- Use ROS2 diagnostic tools

**Setup:**

```bash
# Install Zenoh ROS2 bridge
# Configure bridge to map EdgeFirst topics to ROS2
zenoh-bridge-ros2dds -c bridge_config.json
```

**Important**: ROS2 DDS has different security characteristics than Zenoh. Use SROS2 (Secure ROS2) in production.

---

## Zero-Copy Buffer Sharing

### The Tensor family

`Tensor` is the schema for sharing hardware buffers across processes without
copying. `TensorStamped` and `CameraFrame` are byte-identical wrappers around
it — a header, a `seq`, and the embedded tensor. It supersedes the
single-plane `DmaBuffer` and the earlier `CameraFrame`/`CameraPlane` pair,
both removed in 4.0.0, and supports:

- Multi-plane formats (NV12, I420, planar RGB HWC/NCHW) via one `TensorPlane`
  per plane, each with its own `handle` / `offset` / `stride` / `size` /
  `used` / `modifier`.
- Tiled and compressed layouts, through the per-plane `modifier` (a DRM format
  modifier; 0 is linear).
- Non-fd backends, through the per-plane opaque `handle_bytes`.
- Hardware codec bitstreams (H.264/H.265/MJPEG) where the buffer is oversized
  relative to the valid payload — consumers read `[0, used)`, not `[0, size)`.
- GPU pipeline synchronization via `fence_fd` — a `sync_file` fd consumers
  `pidfd_getfd` into their process and `poll(POLLIN)` before touching pixels.
  `-1` means no fence.
- Quantized model tensors, via `quant_axis` / `quant_scales` /
  `quant_zero_points`.
- Four-axis colorimetry (`color_space` / `color_transfer` / `color_encoding` /
  `color_range`) matching V4L2 / libcamera / DRM vocabulary.
- Off-device bridging: when `handle == -1` the plane's bytes are inlined in
  `data[]`, so a sidecar can publish a self-contained message across host
  boundaries where `pidfd_getfd` is not usable.

**What sits where, and why.** `storage_kind`, `pid` and `fence_fd` are on the
tensor, not the plane. A frame has one backing store and one acquire point:
`export_native_fence_fd` fences the command stream and takes no plane
argument, and planes at different offsets in one dma-buf share a single
`dma_resv`, so a per-plane fence could only ever be the same fd duplicated.
A field belongs on the plane only when it describes that plane's bytes —
which is why `handle`, `offset`, `stride`, `size`, `used`, `modifier` and
`handle_bytes` are per-plane.

**`shape` is the addressing grid, not the byte layout.** An NV12 frame carries
`shape == [h, w]` with a U8 `dtype` against an `h*w*3/2` allocation. Nothing
validates `shape` against any buffer size, deliberately. `strides` is in
**bytes**.

**Position independence.** `seq` is a `uint64`, which forces the embedded
tensor to an 8-aligned offset regardless of `frame_id` length. The tensor's
bytes are therefore identical in every wrapper, and an embedded tensor can be
re-headed onto a tensor topic without re-encoding — the result is
byte-identical to encoding that tensor standalone.

**Typical consumer flow (raw NV12 from the camera service):**

```rust
let cf = CameraFrame::from_cdr(&payload)?;
let t = cf.tensor();

// (1) wait for GPU/DMA completion if the producer set a fence
if t.fence_fd() >= 0 {
    let local = pidfd_getfd(t.pid(), t.fence_fd())?;
    poll(local, POLLIN, timeout)?;   // always bound the timeout
}

// (2) map each plane and process its valid payload
for plane in t.planes() {
    if plane.handle >= 0 {
        let local_fd = pidfd_getfd(t.pid(), plane.handle as RawFd)?;
        let ptr = mmap(plane.size as usize, PROT_READ, MAP_SHARED,
                       local_fd, plane.offset as off_t)?;
        process(unsafe { std::slice::from_raw_parts(ptr, plane.used as usize) });
        munmap(ptr, plane.size as usize)?;
    } else {
        // handle == -1: bytes are inlined (off-device bridge path)
        process(plane.data);
    }
}
```

**Republishing a camera frame's tensor onto a tensor topic:**

```rust
let standalone: Vec<u8> = cf.tensor().to_standalone_cdr();
```

This copies metadata only — plane payloads stay behind their handles.

**Producer language.** Unlike the 3.x `CameraFrame`, the tensor family is
encodable from all four languages: `ros_tensor_builder_*` and
`ros_camera_frame_builder_*` in C, `TensorBuilder`/`CameraFrameBuilder` in
C++, the `Tensor`/`CameraFrame` constructors in Python, and the Rust builders.
Composing the tensor builder into the wrapper builder keeps the argument
surface manageable in C, which was the original reason the 3.x multi-plane
CameraFrame stayed view-only there.

### Acquiring the handle

```mermaid
sequenceDiagram
    participant ISP as Camera ISP
    participant Camera as Camera Service
    participant Consumer as Consumer Service

    ISP->>Camera: DMA buffer (fd=42)
    Camera->>Consumer: CameraFrame (tensor.pid, plane.handle=42)
    Consumer->>Consumer: pidfd_open(pid, 0)
    Consumer->>Consumer: pidfd_getfd(pidfd, handle, 0)
    Consumer->>Consumer: mmap(local_fd, offset)
    Consumer->>Consumer: Process [0, used) in-place
    Consumer->>Consumer: munmap(local_fd)
```

The camera service publishes its process ID (`tensor.pid`) and each plane's
file descriptor (`plane.handle`). Consumers acquire a local copy with:

1. **Get process FD**: `pidfd_open(pid, 0)` — a descriptor referencing the
   camera service process.
2. **Duplicate**: `pidfd_getfd(pidfd, handle, 0)` — a local duplicate of the
   plane's descriptor.

**Consumer-side implementation:**

```rust
use nix::sys::mman::{mmap, munmap, ProtFlags, MapFlags};
use std::os::unix::io::RawFd;

let t = frame.tensor();
let plane = t.plane_at(0).expect("at least one plane");

// Requires pidfd_open / pidfd_getfd (Linux 5.6+)
let pidfd = unsafe { libc::syscall(libc::SYS_pidfd_open, t.pid(), 0) as RawFd };
if pidfd < 0 {
    return Err(std::io::Error::last_os_error());
}

let local_fd = unsafe {
    libc::syscall(libc::SYS_pidfd_getfd, pidfd, plane.handle, 0) as RawFd
};
if local_fd < 0 {
    return Err(std::io::Error::last_os_error());
}

// Map the plane at its offset within the allocation
let ptr = unsafe {
    mmap(
        None,
        plane.size as usize,
        ProtFlags::PROT_READ,
        MapFlags::MAP_SHARED,
        local_fd,
        plane.offset as i64,
    )?
};

// Process the VALID payload only — `used`, not `size`
let data = unsafe { std::slice::from_raw_parts(ptr as *const u8, plane.used as usize) };

// Cleanup
unsafe {
    munmap(ptr, plane.size as usize)?;
    libc::close(local_fd);
    libc::close(pidfd);
}
```

**Permission requirements:** `pidfd_getfd` fails if the consumer runs at a
lower permission level than the camera service. Running as a system service
resolves this.

**Reference:** See the [EdgeFirst Camera Documentation](https://doc.edgefirst.ai/latest/perception/topics/camera/#cameradma) for complete examples.

### Security considerations

- Validate `handle`, `offset`, `size` and `used` before mapping — a hostile
  producer controls all of them. See `SECURITY.md`.
- Always bound the timeout when polling a `fence_fd` from an untrusted
  producer; an un-signalled fence otherwise hangs the consumer forever.
- Implement access control (SELinux, AppArmor).
- Don't expose buffers to untrusted processes.
- Clear sensitive data after use.

---

## Source Code Reference


| Component | Location | Purpose |
|-----------|----------|---------|
| **CDR infrastructure** | `crates/schemas/src/cdr.rs` | Zero-copy CDR1-LE: CdrCursor, CdrWriter, CdrSizer, CdrFixed |
| **PointCloud2 message** | `crates/schemas/src/sensor_msgs/mod.rs` | Buffer-backed PointCloud2 with field iteration |
| **PointCloud access** | `crates/schemas/src/sensor_msgs/pointcloud.rs` | DynPointCloud, PointCloud\<P\>, define_point! |
| **Tensor / TensorPlane** | `crates/schemas/src/tensor.rs` | Tensor payload, plane codec, builders |
| **TensorStamped / CameraFrame** | `crates/schemas/src/edgefirst_msgs.rs` | Byte-identical stamped wrappers over `Tensor` |
| **Tensor C API** | `crates/capi/src/tensor.rs` | C bindings for the tensor family |
| **Detect message** | `crates/schemas/src/edgefirst_msgs.rs` | Object detection results |
| **C API (FFI)** | `crates/capi/src/lib.rs` | C bindings, header hand-maintained at `crates/capi/include/edgefirst/schemas.h` |
| **Schema registry** | `crates/schemas/src/schema_registry.rs` | Runtime type lookup by ROS2 schema name |
| **Python decode_pcd** | `edgefirst/schemas/__init__.py` | Python point cloud decode |
| **Message definitions** | `edgefirst_msgs/msg/*.msg` | Source IDL definitions |

---

## Additional Resources

- **[EdgeFirst Perception Documentation](https://doc.edgefirst.ai/latest/perception/)**: Complete developer guides
- **[EdgeFirst Samples](https://github.com/EdgeFirstAI/samples)**: Example applications
- **[Zenoh Documentation](https://zenoh.io/docs/)**: Zenoh pub/sub details
- **[ROS2 Message Spec](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)**: ROS2 message format
- **[CDR Specification](https://www.omg.org/spec/DDSI-RTPS/)**: OMG CDR standard

---

**For questions or clarifications:** support@au-zone.com
