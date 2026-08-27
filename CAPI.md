# EdgeFirst Schemas C API

C bindings for EdgeFirst Perception message schemas with zero-copy CDR
serialization. The library provides encode/decode functions for fixed-size
types and opaque view handles for variable-length buffer-backed types.

**Header:** `<edgefirst/schemas.h>`
**Library:** `libedgefirst_schemas.so` (Linux), `libedgefirst_schemas.dylib` (macOS), `edgefirst_schemas.dll` (Windows)
**Static library:** `libedgefirst_schemas.a` (Linux/macOS), `edgefirst_schemas.lib` (Windows)
**Wire format:** CDR1 Little-Endian, compatible with ROS 2 DDS
**Dependencies:** None (only standard C library)

## Getting Started

### Include and Link

```c
#include <edgefirst/schemas.h>
```

```bash
gcc -I/path/to/include -o myapp myapp.c -L/path/to/lib -ledgefirst_schemas
```

At runtime the dynamic linker must find the shared library:

```bash
export LD_LIBRARY_PATH=/path/to/lib:$LD_LIBRARY_PATH
./myapp
```

### pkg-config

Pre-built release packages include `edgefirst-schemas.pc`:

```bash
gcc $(pkg-config --cflags edgefirst-schemas) -o myapp myapp.c \
    $(pkg-config --libs edgefirst-schemas)
```

## Installation

### System-wide (Linux/macOS)

```bash
# Install header
sudo mkdir -p /usr/local/include/edgefirst
sudo cp crates/capi/include/edgefirst/schemas.h /usr/local/include/edgefirst/

# Install library (Linux) — install the versioned real file and create the
# symlink chain expected by the GNU/Linux dynamic linker. Replace 2.2.1 with
# the version you built. (`make install` does all of this for you.)
VERSION=2.2.1; MAJOR=${VERSION%%.*}; MM=${VERSION%.*}
sudo install -m 644 target/release/libedgefirst_schemas.so \
    /usr/local/lib/libedgefirst_schemas.so.${VERSION}
sudo ln -sf libedgefirst_schemas.so.${VERSION} /usr/local/lib/libedgefirst_schemas.so.${MM}
sudo ln -sf libedgefirst_schemas.so.${MM}      /usr/local/lib/libedgefirst_schemas.so.${MAJOR}
sudo ln -sf libedgefirst_schemas.so.${MAJOR}   /usr/local/lib/libedgefirst_schemas.so

# Install library (macOS) — the version goes before the extension, and the
# install name is retargeted to the absolute path. `make install` handles both.
sudo cp target/release/libedgefirst_schemas.dylib \
    /usr/local/lib/libedgefirst_schemas.${VERSION}.dylib
sudo ln -sf libedgefirst_schemas.${VERSION}.dylib /usr/local/lib/libedgefirst_schemas.${MM}.dylib
sudo ln -sf libedgefirst_schemas.${MM}.dylib      /usr/local/lib/libedgefirst_schemas.${MAJOR}.dylib
sudo ln -sf libedgefirst_schemas.${MAJOR}.dylib   /usr/local/lib/libedgefirst_schemas.dylib

# Update library cache (Linux only)
sudo ldconfig
```

### Project-local

Copy the header and library into your project tree:

```
my_project/
├── include/
│   └── edgefirst/
│       └── schemas.h
├── lib/
│   └── libedgefirst_schemas.so
└── src/
    └── main.c
```

```bash
gcc -Iinclude -o myapp src/main.c -Llib -ledgefirst_schemas -Wl,-rpath,'$ORIGIN/../lib'
```

### Runtime Library Path

The dynamic linker must find the shared library at runtime:

```bash
# Linux
export LD_LIBRARY_PATH=/path/to/lib:$LD_LIBRARY_PATH

# macOS
export DYLD_LIBRARY_PATH=/path/to/lib:$DYLD_LIBRARY_PATH

# Windows
set PATH=C:\path\to\lib;%PATH%
```

## Naming Conventions

Every C symbol uses the **ROS package name** as its prefix. Library-level
helpers (not tied to a message package) use `edgefirst_schemas_`.

| Package | C prefix | C++ namespace | Source |
|---------|----------|----------------|--------|
| builtin_interfaces | `builtin_interfaces_` | `edgefirst::schemas::builtin_interfaces` | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) |
| std_msgs | `std_msgs_` | `edgefirst::schemas::std_msgs` | same |
| sensor_msgs | `sensor_msgs_` | `edgefirst::schemas::sensor_msgs` | same |
| geometry_msgs | `geometry_msgs_` | `edgefirst::schemas::geometry_msgs` | same |
| nav_msgs | `nav_msgs_` | `edgefirst::schemas::nav_msgs` | same |
| foxglove_msgs | `foxglove_msgs_` | `edgefirst::schemas::foxglove_msgs` | [foxglove-sdk](https://github.com/foxglove/foxglove-sdk) |
| edgefirst_msgs | `edgefirst_msgs_` | `edgefirst::schemas::edgefirst_msgs` | edgefirst_msgs/msg |
| mavros_msgs | `mavros_msgs_` | `edgefirst::schemas::mavros_msgs` | mavros |
| (library helpers) | `edgefirst_schemas_` | n/a | e.g. `edgefirst_schemas_bytes_free` |

**Snake-case conversion rules:**

| Rust / ROS type | C function prefix |
|-----------------|-------------------|
| `PointCloud2` | `sensor_msgs_point_cloud2_` |
| `NavSatFix` | `sensor_msgs_nav_sat_fix_` |
| `CameraFrame` | `edgefirst_msgs_camera_frame_` |
| `TensorPlane` | `edgefirst_msgs_tensor_plane_` |
| `CompressedImage` (sensor) | `sensor_msgs_compressed_image_` |
| `CompressedImage` (foxglove) | `foxglove_msgs_compressed_image_` |
| `TransformStamped` | `geometry_msgs_transform_stamped_` |
| `Altitude` (mavros) | `mavros_msgs_altitude_` |

Acronyms are treated as single words: `Sat` → `sat`, `Nav` → `nav`.
Numbers stay attached to the preceding word: `Cloud2` → `cloud2`.

There are **no** `ros_*` compatibility aliases. C++ collision prefixes
(`Foxglove*`, `Mavros*`) are dropped once types live under nested package
namespaces.

## Architecture

The API splits message types into two categories based on their wire layout.

### CdrFixed Types

Small structs whose CDR size is known at compile time. All fields are
primitive values — no strings, no variable-length arrays.

| Type | Namespace | CDR Size |
|------|-----------|----------|
| Time | builtin_interfaces | 12 B |
| Duration | builtin_interfaces | 12 B |
| Vector3 | geometry_msgs | 28 B |
| Point | geometry_msgs | 28 B |
| Quaternion | geometry_msgs | 36 B |
| Pose | geometry_msgs | 60 B |
| Transform | geometry_msgs | 60 B |
| Twist | geometry_msgs | 52 B |
| Accel | geometry_msgs | 52 B |
| NavSatStatus | sensor_msgs | 8 B |
| MapMetaData | nav_msgs | 84 B† |

†MapMetaData is position-dependent: 80 B payload (84 B total) when starting at a CDR data offset divisible by 8; 76 B payload when embedded at offset ≡ 4 (mod 8). The standalone `encode`/`decode` functions always use the 84 B form.

**Pattern:**

```c
// Encode into a stack buffer
uint8_t buf[64];
size_t written;
builtin_interfaces_time_encode(buf, sizeof(buf), &written, sec, nanosec);

// Decode from CDR bytes
int32_t sec;
uint32_t nanosec;
builtin_interfaces_time_decode(data, len, &sec, &nanosec);
```

Encode functions write CDR bytes into a caller-provided buffer. Pass
`buf = NULL` to query the required size without writing. Decode functions
read individual fields from CDR bytes into output pointers; pass `NULL`
for any field you do not need.

### Buffer-backed Types

Variable-length messages containing strings or byte arrays. These are
represented as opaque handles wrapping an internal CDR byte buffer.

| Type | Namespace |
|------|-----------|
| Header | std_msgs |
| Image | sensor_msgs |
| CompressedImage | sensor_msgs |
| Imu | sensor_msgs |
| NavSatFix | sensor_msgs |
| PointCloud2 | sensor_msgs |
| CameraInfo | sensor_msgs |
| RelativeHumidity | sensor_msgs |
| TimeReference | sensor_msgs |
| AccelStamped | geometry_msgs |
| TwistStamped | geometry_msgs |
| InertiaStamped | geometry_msgs |
| PointStamped | geometry_msgs |
| Vector3Stamped | geometry_msgs |
| PoseStamped | geometry_msgs |
| QuaternionStamped | geometry_msgs |
| WrenchStamped | geometry_msgs |
| PoseWithCovarianceStamped | geometry_msgs |
| TwistWithCovarianceStamped | geometry_msgs |
| AccelWithCovarianceStamped | geometry_msgs |
| TransformStamped | geometry_msgs |
| Polygon | geometry_msgs |
| PolygonStamped | geometry_msgs |
| PoseArray | geometry_msgs |
| Odometry | nav_msgs |
| Altitude | mavros_msgs |
| VfrHud | mavros_msgs |
| EstimatorStatus | mavros_msgs |
| ExtendedState | mavros_msgs |
| SysStatus | mavros_msgs |
| State | mavros_msgs |
| StatusText | mavros_msgs |
| GpsRaw | mavros_msgs |
| TimesyncStatus | mavros_msgs |
| GridCells | nav_msgs |
| OccupancyGrid | nav_msgs |
| Path | nav_msgs |
| CompressedVideo | foxglove_msgs |
| CompressedImage | foxglove_msgs |
| Tensor | edgefirst_msgs |
| TensorPlane | edgefirst_msgs |
| TensorStamped | edgefirst_msgs |
| CameraFrame | edgefirst_msgs |
| Mask | edgefirst_msgs |
| RadarCube | edgefirst_msgs |
| RadarInfo | edgefirst_msgs |
| Detect | edgefirst_msgs |
| Model | edgefirst_msgs |
| ModelInfo | edgefirst_msgs |
| Track | edgefirst_msgs |
| DetectBox | edgefirst_msgs |
| LocalTime | edgefirst_msgs |

**Pattern:**

```c
// Decode: create a zero-copy view over the caller's CDR buffer.
// IMPORTANT: `data` must remain valid until sensor_msgs_image_free(img).
sensor_msgs_image_t* img = sensor_msgs_image_from_cdr(data, len);

// Access fields — O(1), pointers borrow from the caller's buffer
uint32_t w = sensor_msgs_image_get_width(img);
const char* enc = sensor_msgs_image_get_encoding(img);      // borrowed from data
const uint8_t* px = sensor_msgs_image_get_data(img, &pxlen); // borrowed from data

// Re-export the raw CDR bytes (points into the original data buffer)
const uint8_t* cdr = sensor_msgs_image_as_cdr(img, &cdr_len); // borrowed from data

// Free the handle, THEN free the source data buffer
sensor_msgs_image_free(img);
// data can now be freed safely
```

**Encode: builder pattern (recommended, 3.2.0+).** Construct a message
via a stateful builder handle that can reuse a caller-owned buffer
across publishes:

```c
// Create a reusable builder handle
sensor_msgs_image_builder_t* b = sensor_msgs_image_builder_new();

// Set metadata once (strings are copied into the builder)
sensor_msgs_image_builder_set_frame_id(b, "camera");
sensor_msgs_image_builder_set_height(b, 480);
sensor_msgs_image_builder_set_width(b, 640);
sensor_msgs_image_builder_set_encoding(b, "rgb8");
sensor_msgs_image_builder_set_is_bigendian(b, 0);
sensor_msgs_image_builder_set_step(b, 1920);

uint8_t cdr_buf[MAX_CDR_SIZE];
for (int frame = 0; frame < N; ++frame) {
    sensor_msgs_image_builder_set_stamp(b, now_sec(), now_nsec());
    // Bulk data is BORROWED — must remain valid until the next
    // encode_into / build / free / set_data call on this handle.
    sensor_msgs_image_builder_set_data(b, pixels, pixel_len);

    size_t out_len = 0;
    if (sensor_msgs_image_builder_encode_into(b, cdr_buf, sizeof(cdr_buf), &out_len) != 0) {
        // errno set; handle error
    }
    publish(cdr_buf, out_len);
}

sensor_msgs_image_builder_free(b);
```

Or allocate a fresh buffer per call:

```c
uint8_t* bytes = NULL;
size_t len = 0;
sensor_msgs_image_builder_build(b, &bytes, &len);
// ... use bytes ...
edgefirst_schemas_bytes_free(bytes, len);
```

Every buffer-backed type provides these entry points:

| Function | Purpose |
|----------|---------|
| `<package>_<type>_from_cdr(data, len)` | Zero-copy view over caller's CDR buffer (data must outlive handle) |
| `<package>_<type>_get_<field>(handle)` | Read a field (O(1), zero-copy for strings/blobs) |
| `<package>_<type>_set_<field>(buf, len, ...)` | Write a fixed-size field in place on a raw CDR buffer (no re-serialisation) |
| `<package>_<type>_as_cdr(handle, &len)` | Borrow the raw CDR buffer (points into caller's data) |
| `<package>_<type>_free(handle)` | Release the handle (does NOT free the source data) |
| `<package>_<type>_builder_new()` | Create a builder handle |
| `<package>_<type>_builder_set_<field>(b, ...)` | Set a field on the builder (strings copy; bulk data borrows) |
| `<package>_<type>_builder_build(b, &bytes, &len)` | Allocate a fresh CDR buffer and encode |
| `<package>_<type>_builder_encode_into(b, buf, cap, &out_len)` | Encode into caller-owned buffer; errors if too small |
| `<package>_<type>_builder_free(b)` | Release the builder handle |

**When to use in-place setters vs. the builder.** Fixed-size fields
(scalars, fixed-size arrays, CdrFixed struct fields like `Vector3`)
can be mutated cheaply via `<package>_<type>_set_<field>(buf, len, ...)` —
this re-parses the CDR buffer to locate the field, then writes
directly in place. No allocation, no re-serialisation.

Variable-length fields (strings, byte sequences, nested view
sequences) **cannot** be mutated in place because changing their
length would shift every subsequent field. Use the builder API
(`<package>_<type>_builder_*`) for those — it re-serialises the whole
buffer, reusing the existing allocation via `encode_into`.

Rule of thumb: if the field's wire size can change, use the builder.
Otherwise, use the in-place setter.

In-place setters are stateless: the contract is `(buf, len,
value...) -> int32_t` with `0` on success, `-1` with `errno=EINVAL`
on NULL buffer, and `-1` with `errno=EBADMSG` when the buffer is
not a valid encoding of that message type. They never allocate.

## Memory Management

**Rule 1 — Free handles with the matching `_free()` function.**
Every `from_cdr` or `encode` call that returns a handle must be balanced by
a `_free` call. Passing `NULL` to any `_free` function is safe (no-op).

**Rule 1a — Keep source data alive until the handle is freed.**
`from_cdr` creates a zero-copy view — the handle borrows the caller's `data`
buffer directly. The `data` pointer must remain valid until `_free()` is called.
Freeing `data` before the handle causes undefined behavior.

**Rule 2 — Free encode output with `edgefirst_schemas_bytes_free()`.**
Buffer-backed `_encode` functions allocate output via `uint8_t**`.
Free this memory with `edgefirst_schemas_bytes_free(bytes, len)`. Do **not** call
`free()` directly — the memory is allocated by the Rust runtime.

**Rule 3 — Do NOT free borrowed pointers.**
String getters (`const char*`) and blob getters (`const uint8_t*`) return
pointers into the caller's original CDR buffer. These are valid as long as
both the handle and the source data remain alive. Do not free them.

```c
std_msgs_header_t* hdr = std_msgs_header_from_cdr(data, len);

const char* frame = std_msgs_header_get_frame_id(hdr);
printf("%s\n", frame);
// Do NOT call free(frame) — it points into the caller's data buffer

std_msgs_header_free(hdr);
// frame is now dangling — do not use after free
// data can now be freed safely
```

**Rule 4 — CdrFixed types use caller-owned buffers.**
`_encode` writes into a `uint8_t[]` you provide. `_decode` reads from
a `const uint8_t*` you provide. No heap allocation, nothing to free.

**Rule 5 — Parent-borrowed child handles.**
Indexed accessors like `edgefirst_msgs_detect_get_box(view, i)`, `edgefirst_msgs_model_get_box(view, i)`,
and `edgefirst_msgs_model_get_mask(view, i)` return pointers to child view handles owned by
the parent. These are NOT separately allocated — they live inside the parent's
internal `child_boxes`/`child_masks` vector, which is populated once during
`from_cdr` and freed with the parent. Do not pass them to `edgefirst_msgs_box_free()` /
`edgefirst_msgs_mask_free()`; the parent owns them. They become invalid when the parent is
freed. The parent's CDR buffer (passed to `<package>_<parent>_from_cdr`) must also remain
valid for as long as the child pointers are used.

### Summary

| Source | Who owns it | How to free |
|--------|-------------|-------------|
| `<package>_<type>_from_cdr(...)` | Caller (handle + source data) | `<package>_<type>_free(handle)`, then free source data |
| `<package>_<type>_builder_build(b, &bytes, ...)` | Caller | `edgefirst_schemas_bytes_free(bytes, len)` |
| `<package>_<type>_get_<field>(handle)` | Source data (via handle) | Do not free |
| `<package>_<type>_as_cdr(handle, ...)` | Source data (via handle) | Do not free |
| `edgefirst_msgs_detect_get_box(handle, i)` | Parent handle | Do NOT free; owned by parent |
| `edgefirst_msgs_model_get_box(handle, i)` | Parent handle | Do NOT free; owned by parent |
| `edgefirst_msgs_model_get_mask(handle, i)` | Parent handle | Do NOT free; owned by parent |
| CdrFixed `_encode(buf, ...)` | Caller | Stack/caller buffer, no free needed |

## Functions removed in 3.0.0

The following C API functions were removed in 3.0.0 as part of the refactor
that made `edgefirst_msgs_box_t` and `edgefirst_msgs_mask_t` parent-borrowed view types:

- **`edgefirst_msgs_box_as_cdr(box, &len)`** — removed. Child boxes inside a Detect have
  no independent CDR encoding; producing one would require re-encoding the
  box into a fresh buffer, which violates the library's zero-copy contract.
  **Migration**: forward the parent Detect via `edgefirst_msgs_detect_as_cdr(detect, &len)`
  and let subscribers iterate with `edgefirst_msgs_detect_get_box(detect, i)`.
- **`edgefirst_msgs_mask_as_cdr(mask, &len)`** — removed for the same reason. Forward
  the parent Model via `edgefirst_msgs_model_as_cdr` and iterate with `edgefirst_msgs_model_get_mask`.

Standalone `edgefirst_msgs_box_from_cdr(bytes, len)` and `edgefirst_msgs_mask_from_cdr(bytes, len)`
remain available for decoding box/mask CDR buffers that were encoded
standalone (not embedded in a Detect/Model).

## Error Handling

All functions use POSIX `errno` conventions:

- **Functions returning `int`:** `0` on success, `-1` on error with `errno` set.
- **Functions returning a pointer:** valid pointer on success, `NULL` on error
  with `errno` set.

### Error Codes

| errno | Meaning |
|-------|---------|
| `EINVAL` | NULL pointer passed where non-NULL is required |
| `ENOBUFS` | Buffer too small (CdrFixed `_encode` with insufficient capacity) |
| `EBADMSG` | CDR decoding failure — corrupted, truncated, or zero-length data |

**Note on string inputs:** Invalid UTF-8 in C string arguments (e.g.,
`frame_id`, `encoding`) is silently coerced to an empty string `""` rather
than returning an error. This matches the convention that C callers passing
a non-NULL `const char*` expect the call to succeed.

```c
#include <errno.h>
#include <string.h>

std_msgs_header_t* hdr = std_msgs_header_from_cdr(data, len);
if (!hdr) {
    fprintf(stderr, "decode failed: %s\n", strerror(errno));
    // errno == EINVAL for NULL data pointer, EBADMSG for anything else
}
```

Thread safety: `errno` is thread-local on POSIX systems. Functions are
thread-safe for distinct handle instances. Concurrent access to the same
handle requires external synchronization.

## API Reference

### Memory

```c
void edgefirst_schemas_bytes_free(uint8_t* bytes, size_t len);
```

Free a byte buffer returned by any `<package>_*_encode()` function. Passing
`NULL` is safe.

---

### builtin_interfaces

#### Time

```c
int builtin_interfaces_time_encode(uint8_t* buf, size_t cap, size_t* written,
                    int32_t sec, uint32_t nanosec);

int builtin_interfaces_time_decode(const uint8_t* data, size_t len,
                    int32_t* sec, uint32_t* nanosec);
```

**Fields:** `sec` (seconds since epoch), `nanosec` (nanosecond component).

#### Duration

```c
int builtin_interfaces_duration_encode(uint8_t* buf, size_t cap, size_t* written,
                        int32_t sec, uint32_t nanosec);

int builtin_interfaces_duration_decode(const uint8_t* data, size_t len,
                        int32_t* sec, uint32_t* nanosec);
```

**Fields:** `sec`, `nanosec`.

---

### geometry_msgs — CdrFixed

#### Vector3

```c
int geometry_msgs_vector3_encode(uint8_t* buf, size_t cap, size_t* written,
                       double x, double y, double z);

int geometry_msgs_vector3_decode(const uint8_t* data, size_t len,
                       double* x, double* y, double* z);
```

#### Point

```c
int geometry_msgs_point_encode(uint8_t* buf, size_t cap, size_t* written,
                     double x, double y, double z);

int geometry_msgs_point_decode(const uint8_t* data, size_t len,
                     double* x, double* y, double* z);
```

#### Quaternion

```c
int geometry_msgs_quaternion_encode(uint8_t* buf, size_t cap, size_t* written,
                          double x, double y, double z, double w);

int geometry_msgs_quaternion_decode(const uint8_t* data, size_t len,
                          double* x, double* y, double* z, double* w);
```

#### Pose

```c
int geometry_msgs_pose_encode(uint8_t* buf, size_t cap, size_t* written,
                    double px, double py, double pz,
                    double ox, double oy, double oz, double ow);

int geometry_msgs_pose_decode(const uint8_t* data, size_t len,
                    double* px, double* py, double* pz,
                    double* ox, double* oy, double* oz, double* ow);
```

**Fields:** `px/py/pz` = position, `ox/oy/oz/ow` = orientation quaternion.

#### Transform

```c
int geometry_msgs_transform_encode(uint8_t* buf, size_t cap, size_t* written,
                         double tx, double ty, double tz,
                         double rx, double ry, double rz, double rw);

int geometry_msgs_transform_decode(const uint8_t* data, size_t len,
                         double* tx, double* ty, double* tz,
                         double* rx, double* ry, double* rz, double* rw);
```

**Fields:** `tx/ty/tz` = translation, `rx/ry/rz/rw` = rotation quaternion.

#### Twist

```c
int geometry_msgs_twist_encode(uint8_t* buf, size_t cap, size_t* written,
                     double lx, double ly, double lz,
                     double ax, double ay, double az);

int geometry_msgs_twist_decode(const uint8_t* data, size_t len,
                     double* lx, double* ly, double* lz,
                     double* ax, double* ay, double* az);
```

**Fields:** `lx/ly/lz` = linear velocity, `ax/ay/az` = angular velocity.

#### Accel

```c
int geometry_msgs_accel_encode(uint8_t* buf, size_t cap, size_t* written,
                     double lx, double ly, double lz,
                     double ax, double ay, double az);

int geometry_msgs_accel_decode(const uint8_t* data, size_t len,
                     double* lx, double* ly, double* lz,
                     double* ax, double* ay, double* az);
```

**Fields:** `lx/ly/lz` = linear acceleration, `ax/ay/az` = angular acceleration.

---

### sensor_msgs — CdrFixed

#### NavSatStatus

```c
int sensor_msgs_nav_sat_status_encode(uint8_t* buf, size_t cap, size_t* written,
                              int8_t status, uint16_t service);

int sensor_msgs_nav_sat_status_decode(const uint8_t* data, size_t len,
                              int8_t* status, uint16_t* service);
```

**Fields:** `status` (fix status), `service` (service bitmask).

---

### std_msgs — Buffer-backed

#### Header

```c
std_msgs_header_t* std_msgs_header_from_cdr(const uint8_t* data, size_t len);
void          std_msgs_header_free(std_msgs_header_t* view);

int32_t      std_msgs_header_get_stamp_sec(const std_msgs_header_t* view);
uint32_t     std_msgs_header_get_stamp_nanosec(const std_msgs_header_t* view);
const char*  std_msgs_header_get_frame_id(const std_msgs_header_t* view);     // borrowed

const uint8_t* std_msgs_header_as_cdr(const std_msgs_header_t* view, size_t* out_len);

// Builder API: std_msgs_header_builder_new / _set_* / _build / _encode_into
```

---

### sensor_msgs — Buffer-backed

#### Image

```c
sensor_msgs_image_t* sensor_msgs_image_from_cdr(const uint8_t* data, size_t len);
void         sensor_msgs_image_free(sensor_msgs_image_t* view);

int32_t      sensor_msgs_image_get_stamp_sec(const sensor_msgs_image_t* view);
uint32_t     sensor_msgs_image_get_stamp_nanosec(const sensor_msgs_image_t* view);
const char*  sensor_msgs_image_get_frame_id(const sensor_msgs_image_t* view);       // borrowed
uint32_t     sensor_msgs_image_get_height(const sensor_msgs_image_t* view);
uint32_t     sensor_msgs_image_get_width(const sensor_msgs_image_t* view);
const char*  sensor_msgs_image_get_encoding(const sensor_msgs_image_t* view);       // borrowed
uint8_t      sensor_msgs_image_get_is_bigendian(const sensor_msgs_image_t* view);
uint32_t     sensor_msgs_image_get_step(const sensor_msgs_image_t* view);
const uint8_t* sensor_msgs_image_get_data(const sensor_msgs_image_t* view, size_t* out_len); // borrowed

const uint8_t* sensor_msgs_image_as_cdr(const sensor_msgs_image_t* view, size_t* out_len);

// Builder API: sensor_msgs_image_builder_new / _set_* / _build / _encode_into
```

#### CompressedImage

```c
sensor_msgs_compressed_image_t* sensor_msgs_compressed_image_from_cdr(const uint8_t* data, size_t len);
void                    sensor_msgs_compressed_image_free(sensor_msgs_compressed_image_t* view);

int32_t      sensor_msgs_compressed_image_get_stamp_sec(const sensor_msgs_compressed_image_t* view);
uint32_t     sensor_msgs_compressed_image_get_stamp_nanosec(const sensor_msgs_compressed_image_t* view);
const char*  sensor_msgs_compressed_image_get_frame_id(const sensor_msgs_compressed_image_t* view);
const char*  sensor_msgs_compressed_image_get_format(const sensor_msgs_compressed_image_t* view);
const uint8_t* sensor_msgs_compressed_image_get_data(const sensor_msgs_compressed_image_t* view,
                                              size_t* out_len);

const uint8_t* sensor_msgs_compressed_image_as_cdr(const sensor_msgs_compressed_image_t* view,
                                            size_t* out_len);

// Builder API: sensor_msgs_compressed_image_builder_new / _set_* / _build
```

#### Imu

```c
sensor_msgs_imu_t* sensor_msgs_imu_from_cdr(const uint8_t* data, size_t len);
void       sensor_msgs_imu_free(sensor_msgs_imu_t* view);

int32_t     sensor_msgs_imu_get_stamp_sec(const sensor_msgs_imu_t* view);
uint32_t    sensor_msgs_imu_get_stamp_nanosec(const sensor_msgs_imu_t* view);
const char* sensor_msgs_imu_get_frame_id(const sensor_msgs_imu_t* view);

void sensor_msgs_imu_get_orientation(const sensor_msgs_imu_t* view,
                             double* x, double* y, double* z, double* w);
void sensor_msgs_imu_get_orientation_covariance(const sensor_msgs_imu_t* view, double* out);

void sensor_msgs_imu_get_angular_velocity(const sensor_msgs_imu_t* view,
                                  double* x, double* y, double* z);
void sensor_msgs_imu_get_angular_velocity_covariance(const sensor_msgs_imu_t* view, double* out);

void sensor_msgs_imu_get_linear_acceleration(const sensor_msgs_imu_t* view,
                                     double* x, double* y, double* z);
void sensor_msgs_imu_get_linear_acceleration_covariance(const sensor_msgs_imu_t* view, double* out);

const uint8_t* sensor_msgs_imu_as_cdr(const sensor_msgs_imu_t* view, size_t* out_len);
```

Covariance functions write 9 `double` values (row-major 3x3 matrix) into
the caller-provided `out` array. The caller must ensure `out` points to
at least 9 doubles.

**Note:** The Imu type has no `_encode` function. Construct Imu messages
from the Rust API or from CDR bytes received over Zenoh/DDS.

#### NavSatFix

```c
sensor_msgs_nav_sat_fix_t* sensor_msgs_nav_sat_fix_from_cdr(const uint8_t* data, size_t len);
void               sensor_msgs_nav_sat_fix_free(sensor_msgs_nav_sat_fix_t* view);

int32_t     sensor_msgs_nav_sat_fix_get_stamp_sec(const sensor_msgs_nav_sat_fix_t* view);
uint32_t    sensor_msgs_nav_sat_fix_get_stamp_nanosec(const sensor_msgs_nav_sat_fix_t* view);
const char* sensor_msgs_nav_sat_fix_get_frame_id(const sensor_msgs_nav_sat_fix_t* view);
double      sensor_msgs_nav_sat_fix_get_latitude(const sensor_msgs_nav_sat_fix_t* view);
double      sensor_msgs_nav_sat_fix_get_longitude(const sensor_msgs_nav_sat_fix_t* view);
double      sensor_msgs_nav_sat_fix_get_altitude(const sensor_msgs_nav_sat_fix_t* view);

const uint8_t* sensor_msgs_nav_sat_fix_as_cdr(const sensor_msgs_nav_sat_fix_t* view, size_t* out_len);
```

#### PointCloud2

```c
sensor_msgs_point_cloud2_t* sensor_msgs_point_cloud2_from_cdr(const uint8_t* data, size_t len);
void                sensor_msgs_point_cloud2_free(sensor_msgs_point_cloud2_t* view);

int32_t     sensor_msgs_point_cloud2_get_stamp_sec(const sensor_msgs_point_cloud2_t* view);
uint32_t    sensor_msgs_point_cloud2_get_stamp_nanosec(const sensor_msgs_point_cloud2_t* view);
const char* sensor_msgs_point_cloud2_get_frame_id(const sensor_msgs_point_cloud2_t* view);
uint32_t    sensor_msgs_point_cloud2_get_height(const sensor_msgs_point_cloud2_t* view);
uint32_t    sensor_msgs_point_cloud2_get_width(const sensor_msgs_point_cloud2_t* view);
uint32_t    sensor_msgs_point_cloud2_get_point_step(const sensor_msgs_point_cloud2_t* view);
uint32_t    sensor_msgs_point_cloud2_get_row_step(const sensor_msgs_point_cloud2_t* view);
const uint8_t* sensor_msgs_point_cloud2_get_data(const sensor_msgs_point_cloud2_t* view,
                                          size_t* out_len);
bool        sensor_msgs_point_cloud2_get_is_dense(const sensor_msgs_point_cloud2_t* view);
bool        sensor_msgs_point_cloud2_get_is_bigendian(const sensor_msgs_point_cloud2_t* view);
uint32_t    sensor_msgs_point_cloud2_get_fields_len(const sensor_msgs_point_cloud2_t* view);

const uint8_t* sensor_msgs_point_cloud2_as_cdr(const sensor_msgs_point_cloud2_t* view,
                                        size_t* out_len);
```

Point data is returned as raw `uint8_t*` bytes. Interpret them using
`point_step`, `row_step`, and the field definitions to extract typed
point values (x, y, z, intensity, etc.).

#### CameraInfo

```c
sensor_msgs_camera_info_t* sensor_msgs_camera_info_from_cdr(const uint8_t* data, size_t len);
void               sensor_msgs_camera_info_free(sensor_msgs_camera_info_t* view);

int32_t     sensor_msgs_camera_info_get_stamp_sec(const sensor_msgs_camera_info_t* view);
uint32_t    sensor_msgs_camera_info_get_stamp_nanosec(const sensor_msgs_camera_info_t* view);
const char* sensor_msgs_camera_info_get_frame_id(const sensor_msgs_camera_info_t* view);
uint32_t    sensor_msgs_camera_info_get_height(const sensor_msgs_camera_info_t* view);
uint32_t    sensor_msgs_camera_info_get_width(const sensor_msgs_camera_info_t* view);
const char* sensor_msgs_camera_info_get_distortion_model(const sensor_msgs_camera_info_t* view);
uint32_t    sensor_msgs_camera_info_get_binning_x(const sensor_msgs_camera_info_t* view);
uint32_t    sensor_msgs_camera_info_get_binning_y(const sensor_msgs_camera_info_t* view);

const uint8_t* sensor_msgs_camera_info_as_cdr(const sensor_msgs_camera_info_t* view,
                                       size_t* out_len);
```

#### RelativeHumidity

```c
sensor_msgs_relative_humidity_t* sensor_msgs_relative_humidity_from_cdr(const uint8_t* data, size_t len);
void                     sensor_msgs_relative_humidity_free(sensor_msgs_relative_humidity_t* view);

int32_t     sensor_msgs_relative_humidity_get_stamp_sec(const sensor_msgs_relative_humidity_t* view);
uint32_t    sensor_msgs_relative_humidity_get_stamp_nanosec(const sensor_msgs_relative_humidity_t* view);
const char* sensor_msgs_relative_humidity_get_frame_id(const sensor_msgs_relative_humidity_t* view);
double      sensor_msgs_relative_humidity_get_relative_humidity(const sensor_msgs_relative_humidity_t* view);
double      sensor_msgs_relative_humidity_get_variance(const sensor_msgs_relative_humidity_t* view);

const uint8_t* sensor_msgs_relative_humidity_as_cdr(const sensor_msgs_relative_humidity_t* view,
                                             size_t* out_len);
```

**Fields:** `relative_humidity` (dimensionless ratio, 0–1), `variance` (measurement variance, 0 if unknown).

**Builder:**

```c
sensor_msgs_relative_humidity_builder_t* sensor_msgs_relative_humidity_builder_new(void);
void sensor_msgs_relative_humidity_builder_free(sensor_msgs_relative_humidity_builder_t* b);

void sensor_msgs_relative_humidity_builder_set_stamp(sensor_msgs_relative_humidity_builder_t* b,
                                              int32_t sec, uint32_t nanosec);
int  sensor_msgs_relative_humidity_builder_set_frame_id(sensor_msgs_relative_humidity_builder_t* b,
                                                 const char* s);
void sensor_msgs_relative_humidity_builder_set_relative_humidity(sensor_msgs_relative_humidity_builder_t* b,
                                                          double v);
void sensor_msgs_relative_humidity_builder_set_variance(sensor_msgs_relative_humidity_builder_t* b, double v);

int  sensor_msgs_relative_humidity_builder_build(sensor_msgs_relative_humidity_builder_t* b,
                                          uint8_t** out_bytes, size_t* out_len);
int  sensor_msgs_relative_humidity_builder_encode_into(sensor_msgs_relative_humidity_builder_t* b,
                                                uint8_t* buf, size_t cap, size_t* out_len);
```

#### TimeReference

```c
sensor_msgs_time_reference_t* sensor_msgs_time_reference_from_cdr(const uint8_t* data, size_t len);
void                  sensor_msgs_time_reference_free(sensor_msgs_time_reference_t* view);

int32_t     sensor_msgs_time_reference_get_stamp_sec(const sensor_msgs_time_reference_t* view);
uint32_t    sensor_msgs_time_reference_get_stamp_nanosec(const sensor_msgs_time_reference_t* view);
const char* sensor_msgs_time_reference_get_frame_id(const sensor_msgs_time_reference_t* view);
int32_t     sensor_msgs_time_reference_get_time_ref_sec(const sensor_msgs_time_reference_t* view);
uint32_t    sensor_msgs_time_reference_get_time_ref_nanosec(const sensor_msgs_time_reference_t* view);
const char* sensor_msgs_time_reference_get_source(const sensor_msgs_time_reference_t* view);     // borrowed

const uint8_t* sensor_msgs_time_reference_as_cdr(const sensor_msgs_time_reference_t* view, size_t* out_len);
```

**Fields:** `time_ref` (the referenced external clock time as sec/nanosec), `source` (human-readable string identifying the time source; borrowed from the CDR buffer).

**Builder:**

```c
sensor_msgs_time_reference_builder_t* sensor_msgs_time_reference_builder_new(void);
void sensor_msgs_time_reference_builder_free(sensor_msgs_time_reference_builder_t* b);

void sensor_msgs_time_reference_builder_set_stamp(sensor_msgs_time_reference_builder_t* b,
                                          int32_t sec, uint32_t nanosec);
int  sensor_msgs_time_reference_builder_set_frame_id(sensor_msgs_time_reference_builder_t* b,
                                             const char* s);
void sensor_msgs_time_reference_builder_set_time_ref(sensor_msgs_time_reference_builder_t* b,
                                             int32_t sec, uint32_t nanosec);
int  sensor_msgs_time_reference_builder_set_source(sensor_msgs_time_reference_builder_t* b,
                                           const char* s);

int  sensor_msgs_time_reference_builder_build(sensor_msgs_time_reference_builder_t* b,
                                      uint8_t** out_bytes, size_t* out_len);
int  sensor_msgs_time_reference_builder_encode_into(sensor_msgs_time_reference_builder_t* b,
                                            uint8_t* buf, size_t cap, size_t* out_len);
```

---

### geometry_msgs — Buffer-backed

All 15 buffer-backed geometry types expose the same builder stack:
`<package>_<type>_builder_{new,free,set_*,build,encode_into}`. There is no
one-shot `<package>_<type>_encode` for these messages. Builder calls return 0 on
success, -1 on error (`errno`: `EINVAL` for NULL or invalid UTF-8,
`ENOBUFS` for a short `encode_into` buffer, `EBADMSG` for encoding failure).
Call `edgefirst_schemas_bytes_free(*out_bytes, *out_len)` after `build`.

#### AccelStamped (template for simple stamped types)

Simple stamped types (`TwistStamped`, `InertiaStamped`, `PointStamped`,
`Vector3Stamped`, `PoseStamped`, `QuaternionStamped`, `WrenchStamped`,
`PoseWithCovarianceStamped`, `TwistWithCovarianceStamped`,
`AccelWithCovarianceStamped`) follow the same header + payload pattern.
Only the payload setters differ; see `schemas.h` for the full list.

```c
geometry_msgs_accel_stamped_t* geometry_msgs_accel_stamped_from_cdr(const uint8_t* data, size_t len);
void geometry_msgs_accel_stamped_free(geometry_msgs_accel_stamped_t* view);

int32_t     geometry_msgs_accel_stamped_get_stamp_sec(const geometry_msgs_accel_stamped_t* view);
uint32_t    geometry_msgs_accel_stamped_get_stamp_nanosec(const geometry_msgs_accel_stamped_t* view);
const char* geometry_msgs_accel_stamped_get_frame_id(const geometry_msgs_accel_stamped_t* view);

const uint8_t* geometry_msgs_accel_stamped_as_cdr(const geometry_msgs_accel_stamped_t* view,
                                          size_t* out_len);
```

**Builder:**

```c
geometry_msgs_accel_stamped_builder_t* geometry_msgs_accel_stamped_builder_new(void);
void geometry_msgs_accel_stamped_builder_free(geometry_msgs_accel_stamped_builder_t* b);

void geometry_msgs_accel_stamped_builder_set_stamp(geometry_msgs_accel_stamped_builder_t* b,
                                         int32_t sec, uint32_t nanosec);
int  geometry_msgs_accel_stamped_builder_set_frame_id(geometry_msgs_accel_stamped_builder_t* b,
                                            const char* s);
void geometry_msgs_accel_stamped_builder_set_linear_acceleration(geometry_msgs_accel_stamped_builder_t* b,
                                                       double x, double y, double z);
void geometry_msgs_accel_stamped_builder_set_angular_acceleration(geometry_msgs_accel_stamped_builder_t* b,
                                                        double x, double y, double z);

int  geometry_msgs_accel_stamped_builder_build(geometry_msgs_accel_stamped_builder_t* b,
                                    uint8_t** out_bytes, size_t* out_len);
int  geometry_msgs_accel_stamped_builder_encode_into(geometry_msgs_accel_stamped_builder_t* b,
                                           uint8_t* buf, size_t cap, size_t* out_len);
```

Covariance stamped types take `<package>_*_builder_set_covariance(b, cov)` with a
36-element `double` array (copied into the builder). `InertiaStamped` uses
field-wise inertia setters (`set_mass`, `set_com`, `set_inertia_tensor`).

#### TransformStamped

Adds `child_frame_id` and transform translation/rotation setters on top of
the stamped header pattern.

```c
geometry_msgs_transform_stamped_t* geometry_msgs_transform_stamped_from_cdr(const uint8_t* data,
                                                         size_t len);
void geometry_msgs_transform_stamped_free(geometry_msgs_transform_stamped_t* view);

int32_t     geometry_msgs_transform_stamped_get_stamp_sec(const geometry_msgs_transform_stamped_t* view);
uint32_t    geometry_msgs_transform_stamped_get_stamp_nanosec(const geometry_msgs_transform_stamped_t* view);
const char* geometry_msgs_transform_stamped_get_frame_id(const geometry_msgs_transform_stamped_t* view);
const char* geometry_msgs_transform_stamped_get_child_frame_id(const geometry_msgs_transform_stamped_t* view);

const uint8_t* geometry_msgs_transform_stamped_as_cdr(const geometry_msgs_transform_stamped_t* view,
                                             size_t* out_len);
```

**Builder:**

```c
geometry_msgs_transform_stamped_builder_t* geometry_msgs_transform_stamped_builder_new(void);
void geometry_msgs_transform_stamped_builder_free(geometry_msgs_transform_stamped_builder_t* b);

void geometry_msgs_transform_stamped_builder_set_stamp(geometry_msgs_transform_stamped_builder_t* b,
                                             int32_t sec, uint32_t nanosec);
int  geometry_msgs_transform_stamped_builder_set_frame_id(geometry_msgs_transform_stamped_builder_t* b,
                                                const char* s);
int  geometry_msgs_transform_stamped_builder_set_child_frame_id(geometry_msgs_transform_stamped_builder_t* b,
                                                      const char* s);
void geometry_msgs_transform_stamped_builder_set_translation(geometry_msgs_transform_stamped_builder_t* b,
                                                   double x, double y, double z);
void geometry_msgs_transform_stamped_builder_set_rotation(geometry_msgs_transform_stamped_builder_t* b,
                                                double x, double y, double z, double w);

int  geometry_msgs_transform_stamped_builder_build(geometry_msgs_transform_stamped_builder_t* b,
                                         uint8_t** out_bytes, size_t* out_len);
int  geometry_msgs_transform_stamped_builder_encode_into(geometry_msgs_transform_stamped_builder_t* b,
                                               uint8_t* buf, size_t cap, size_t* out_len);
```

#### Polygon / PolygonStamped / PoseArray (sequences)

Sequence setters borrow their argument until `build` or `encode_into`:

```c
/* count = number of Point32 vertices; xyz = count * 3 contiguous floats */
int geometry_msgs_polygon_builder_set_points(geometry_msgs_polygon_builder_t* b,
                                   const float* xyz, size_t count);

/* count = number of Pose elements; poses = count * 7 contiguous doubles
   (px, py, pz, ox, oy, oz, ow per pose) */
int geometry_msgs_pose_array_builder_set_poses(geometry_msgs_pose_array_builder_t* b,
                                     const double* poses, size_t count);
```

`PolygonStamped` combines the stamped header with `set_points`. See
`schemas.h` for `geometry_msgs_polygon_*` and `geometry_msgs_pose_array_*` view and builder
entry points.

---

### nav_msgs — Buffer-backed

#### Odometry

```c
nav_msgs_odometry_t* nav_msgs_odometry_from_cdr(const uint8_t* data, size_t len);
void            nav_msgs_odometry_free(nav_msgs_odometry_t* view);

int32_t     nav_msgs_odometry_get_stamp_sec(const nav_msgs_odometry_t* view);
uint32_t    nav_msgs_odometry_get_stamp_nanosec(const nav_msgs_odometry_t* view);
const char* nav_msgs_odometry_get_frame_id(const nav_msgs_odometry_t* view);
const char* nav_msgs_odometry_get_child_frame_id(const nav_msgs_odometry_t* view);

void nav_msgs_odometry_get_pose(const nav_msgs_odometry_t* view,
                           double* px, double* py, double* pz,
                           double* ox, double* oy, double* oz, double* ow);
void nav_msgs_odometry_get_pose_covariance(const nav_msgs_odometry_t* view, double* out);
void nav_msgs_odometry_get_twist(const nav_msgs_odometry_t* view,
                            double* lx, double* ly, double* lz,
                            double* ax, double* ay, double* az);
void nav_msgs_odometry_get_twist_covariance(const nav_msgs_odometry_t* view, double* out);

const uint8_t* nav_msgs_odometry_as_cdr(const nav_msgs_odometry_t* view, size_t* out_len);
```

**Builder:**

```c
nav_msgs_odometry_builder_t* nav_msgs_odometry_builder_new(void);
void nav_msgs_odometry_builder_free(nav_msgs_odometry_builder_t* b);

void nav_msgs_odometry_builder_set_stamp(nav_msgs_odometry_builder_t* b,
                                    int32_t sec, uint32_t nanosec);
int  nav_msgs_odometry_builder_set_frame_id(nav_msgs_odometry_builder_t* b, const char* s);
int  nav_msgs_odometry_builder_set_child_frame_id(nav_msgs_odometry_builder_t* b, const char* s);
void nav_msgs_odometry_builder_set_pose(nav_msgs_odometry_builder_t* b,
                                   double px, double py, double pz,
                                   double ox, double oy, double oz, double ow);
int  nav_msgs_odometry_builder_set_pose_covariance(nav_msgs_odometry_builder_t* b,
                                              const double* cov);
void nav_msgs_odometry_builder_set_twist(nav_msgs_odometry_builder_t* b,
                                    double lx, double ly, double lz,
                                    double ax, double ay, double az);
int  nav_msgs_odometry_builder_set_twist_covariance(nav_msgs_odometry_builder_t* b,
                                               const double* cov);

int  nav_msgs_odometry_builder_build(nav_msgs_odometry_builder_t* b,
                                uint8_t** out_bytes, size_t* out_len);
int  nav_msgs_odometry_builder_encode_into(nav_msgs_odometry_builder_t* b,
                                      uint8_t* buf, size_t cap, size_t* out_len);
```

`set_*_covariance` copies 36 doubles into the builder (same layout as
`nav_msgs_odometry_get_pose_covariance` / `get_twist_covariance`).

---

### nav_msgs — CdrFixed

#### MapMetaData

```c
int nav_msgs_map_meta_data_encode(
    uint8_t* buf, size_t cap, size_t* written,
    int32_t  map_load_time_sec, uint32_t map_load_time_nanosec,
    float    resolution, uint32_t width, uint32_t height,
    double   origin_px, double origin_py, double origin_pz,
    double   origin_ox, double origin_oy, double origin_oz, double origin_ow);

int nav_msgs_map_meta_data_decode(
    const uint8_t* data, size_t len,
    int32_t*  map_load_time_sec, uint32_t* map_load_time_nanosec,
    float*    resolution, uint32_t* width, uint32_t* height,
    double*   origin_px, double* origin_py, double* origin_pz,
    double*   origin_ox, double* origin_oy, double* origin_oz, double* origin_ow);
```

**Fields:** `map_load_time` (when the underlying map file was last loaded), `resolution` (metres per cell), `width`/`height` (grid dimensions in cells), `origin` (pose of cell (0, 0) in the map frame: position `px/py/pz` + orientation quaternion `ox/oy/oz/ow`).

Pass `buf = NULL` to `encode` to query the required buffer size via `written`. All output pointers in `decode` may be `NULL` to discard the corresponding field.

> **Size note:** standalone encoding produces 84 B (4 B CDR header + 80 B payload). When
> MapMetaData is embedded inside a composite type at a CDR data offset ≡ 4 (mod 8) the
> payload shrinks to 76 B because the origin's 8-byte alignment padding is not needed.
> Never apply fixed-offset arithmetic to MapMetaData inside composite types; use
> `nav_msgs_occupancy_grid_get_info` to read it out of an OccupancyGrid.

---

### nav_msgs — Buffer-backed

#### GridCells

```c
nav_msgs_grid_cells_t* nav_msgs_grid_cells_from_cdr(const uint8_t* data, size_t len);
void              nav_msgs_grid_cells_free(nav_msgs_grid_cells_t* view);

int32_t     nav_msgs_grid_cells_get_stamp_sec(const nav_msgs_grid_cells_t* view);
uint32_t    nav_msgs_grid_cells_get_stamp_nanosec(const nav_msgs_grid_cells_t* view);
const char* nav_msgs_grid_cells_get_frame_id(const nav_msgs_grid_cells_t* view);
float       nav_msgs_grid_cells_get_cell_width(const nav_msgs_grid_cells_t* view);
float       nav_msgs_grid_cells_get_cell_height(const nav_msgs_grid_cells_t* view);

size_t      nav_msgs_grid_cells_get_len(const nav_msgs_grid_cells_t* view);

int         nav_msgs_grid_cells_get_cell(const nav_msgs_grid_cells_t* view,
                                    size_t index,
                                    double* x, double* y, double* z);

const uint8_t* nav_msgs_grid_cells_as_cdr(const nav_msgs_grid_cells_t* view, size_t* out_len);
```

`nav_msgs_grid_cells_get_cell` returns 0 on success, -1 (errno: `EINVAL`) if `index >= len`. Output pointers `x`, `y`, `z` may be `NULL` to discard the corresponding component.

**Builder:**

```c
nav_msgs_grid_cells_builder_t* nav_msgs_grid_cells_builder_new(void);
void nav_msgs_grid_cells_builder_free(nav_msgs_grid_cells_builder_t* b);

void nav_msgs_grid_cells_builder_set_stamp(nav_msgs_grid_cells_builder_t* b,
                                      int32_t sec, uint32_t nanosec);
int  nav_msgs_grid_cells_builder_set_frame_id(nav_msgs_grid_cells_builder_t* b, const char* s);
void nav_msgs_grid_cells_builder_set_cell_width(nav_msgs_grid_cells_builder_t* b, float v);
void nav_msgs_grid_cells_builder_set_cell_height(nav_msgs_grid_cells_builder_t* b, float v);

int  nav_msgs_grid_cells_builder_set_cells(nav_msgs_grid_cells_builder_t* b,
                                      const double* xyz, size_t count);

int  nav_msgs_grid_cells_builder_build(nav_msgs_grid_cells_builder_t* b,
                                  uint8_t** out_bytes, size_t* out_len);
int  nav_msgs_grid_cells_builder_encode_into(nav_msgs_grid_cells_builder_t* b,
                                        uint8_t* buf, size_t cap, size_t* out_len);
```

`set_cells` expects `count * 3` contiguous doubles `[x₀, y₀, z₀, x₁, y₁, z₁, …]`. The pointer is borrowed and must remain valid until `build` or `encode_into` is called. Pass `NULL` with `count = 0` to clear the cell list.

#### OccupancyGrid

```c
nav_msgs_occupancy_grid_t* nav_msgs_occupancy_grid_from_cdr(const uint8_t* data, size_t len);
void                  nav_msgs_occupancy_grid_free(nav_msgs_occupancy_grid_t* view);

int32_t        nav_msgs_occupancy_grid_get_stamp_sec(const nav_msgs_occupancy_grid_t* view);
uint32_t       nav_msgs_occupancy_grid_get_stamp_nanosec(const nav_msgs_occupancy_grid_t* view);
const char*    nav_msgs_occupancy_grid_get_frame_id(const nav_msgs_occupancy_grid_t* view);

void nav_msgs_occupancy_grid_get_info(
    const nav_msgs_occupancy_grid_t* view,
    int32_t*  map_load_time_sec, uint32_t* map_load_time_nanosec,
    float*    resolution, uint32_t* width, uint32_t* height,
    double*   origin_px, double* origin_py, double* origin_pz,
    double*   origin_ox, double* origin_oy, double* origin_oz, double* origin_ow);

size_t         nav_msgs_occupancy_grid_get_data_len(const nav_msgs_occupancy_grid_t* view);
const int8_t*  nav_msgs_occupancy_grid_get_data(const nav_msgs_occupancy_grid_t* view);

const uint8_t* nav_msgs_occupancy_grid_as_cdr(const nav_msgs_occupancy_grid_t* view,
                                          size_t* out_len);
```

`nav_msgs_occupancy_grid_get_info` takes the same field list as `nav_msgs_map_meta_data_decode`; any output pointer may be `NULL`. `nav_msgs_occupancy_grid_get_data` returns a zero-copy `int8_t*` to the occupancy data (`data_len` cells, row-major). Conventional occupancy values: 0 = free, 100 = occupied, -1 = unknown.

**Builder:**

```c
nav_msgs_occupancy_grid_builder_t* nav_msgs_occupancy_grid_builder_new(void);
void nav_msgs_occupancy_grid_builder_free(nav_msgs_occupancy_grid_builder_t* b);

void nav_msgs_occupancy_grid_builder_set_stamp(nav_msgs_occupancy_grid_builder_t* b,
                                          int32_t sec, uint32_t nanosec);
int  nav_msgs_occupancy_grid_builder_set_frame_id(nav_msgs_occupancy_grid_builder_t* b,
                                             const char* s);
void nav_msgs_occupancy_grid_builder_set_info(
    nav_msgs_occupancy_grid_builder_t* b,
    int32_t  map_load_time_sec, uint32_t map_load_time_nanosec,
    float    resolution, uint32_t width, uint32_t height,
    double   origin_px, double origin_py, double origin_pz,
    double   origin_ox, double origin_oy, double origin_oz, double origin_ow);
int  nav_msgs_occupancy_grid_builder_set_data(nav_msgs_occupancy_grid_builder_t* b,
                                         const int8_t* data, size_t len);

int  nav_msgs_occupancy_grid_builder_build(nav_msgs_occupancy_grid_builder_t* b,
                                      uint8_t** out_bytes, size_t* out_len);
int  nav_msgs_occupancy_grid_builder_encode_into(nav_msgs_occupancy_grid_builder_t* b,
                                            uint8_t* buf, size_t cap, size_t* out_len);
```

`set_info` takes the same argument list as `nav_msgs_map_meta_data_encode`. `set_data` borrows `data`; the pointer must remain valid until `build` or `encode_into` is called.

#### Path

```c
nav_msgs_path_t* nav_msgs_path_from_cdr(const uint8_t* data, size_t len);
void        nav_msgs_path_free(nav_msgs_path_t* view);

int32_t     nav_msgs_path_get_stamp_sec(const nav_msgs_path_t* view);
uint32_t    nav_msgs_path_get_stamp_nanosec(const nav_msgs_path_t* view);
const char* nav_msgs_path_get_frame_id(const nav_msgs_path_t* view);

size_t      nav_msgs_path_get_len(const nav_msgs_path_t* view);

int         nav_msgs_path_get_pose(const nav_msgs_path_t* view,
                              size_t index,
                              int32_t*  stamp_sec, uint32_t* stamp_nanosec,
                              const char** pose_frame_id,
                              double* px, double* py, double* pz,
                              double* ox, double* oy, double* oz, double* ow);

const uint8_t* nav_msgs_path_as_cdr(const nav_msgs_path_t* view, size_t* out_len);
```

`nav_msgs_path_get_pose` returns 0 on success, -1 (errno: `EINVAL`) if `index >= len`. `pose_frame_id` is set to a zero-copy pointer into the CDR buffer (valid for the view lifetime); other out-params may be `NULL`. **Performance:** each call scans the variable-length PoseStamped sequence from its head to `index` — O(n) per call. A `0..len` loop over `nav_msgs_path_get_pose` is O(n²); use `nav_msgs_path_iter_*` for bulk traversal.

**Iterator (O(n) bulk traversal):**

```c
nav_msgs_path_iter_t* nav_msgs_path_iter_new(const nav_msgs_path_t* view);

int  nav_msgs_path_iter_next(nav_msgs_path_iter_t* it,
                        int32_t* out_sec, uint32_t* out_nanosec,
                        const char** out_frame_id,
                        double* out_px, double* out_py, double* out_pz,
                        double* out_ox, double* out_oy, double* out_oz, double* out_ow);

void nav_msgs_path_iter_free(nav_msgs_path_iter_t* it);
```

`nav_msgs_path_iter_new` returns `NULL` (errno: `EINVAL`) if `view` is `NULL`. `nav_msgs_path_iter_next` returns 1 while elements remain, 0 when the sequence is exhausted. `out_frame_id` borrows the per-element string from the CDR buffer (valid while the buffer lives); all other out-params may be `NULL`. The iterator shares the view's CDR buffer and must not outlive the view.

**Builder (accumulator):**

```c
nav_msgs_path_builder_t* nav_msgs_path_builder_new(void);
void nav_msgs_path_builder_free(nav_msgs_path_builder_t* b);

void nav_msgs_path_builder_set_stamp(nav_msgs_path_builder_t* b, int32_t sec, uint32_t nanosec);
int  nav_msgs_path_builder_set_frame_id(nav_msgs_path_builder_t* b, const char* s);

int  nav_msgs_path_builder_add_pose(nav_msgs_path_builder_t* b,
                               int32_t sec, uint32_t nanosec, const char* frame_id,
                               double px, double py, double pz,
                               double ox, double oy, double oz, double ow);

int  nav_msgs_path_builder_build(nav_msgs_path_builder_t* b, uint8_t** out_bytes, size_t* out_len);
int  nav_msgs_path_builder_encode_into(nav_msgs_path_builder_t* b,
                                  uint8_t* buf, size_t cap, size_t* out_len);
```

Unlike other builders, Path uses an accumulator: call `nav_msgs_path_builder_add_pose` once per waypoint before calling `build` or `encode_into`. Each call copies `frame_id` into the builder. Returns 0 on success, -1 (errno: `EINVAL`) if `b` is `NULL` or `frame_id` is `NULL` / not valid UTF-8.

---

### mavros_msgs — Buffer-backed

All nine mavros buffer-backed types use the `mavros_msgs_<type>_…` prefix
(e.g. `mavros_msgs_altitude_*`). There is no one-shot encode; use the builder
stack with the same errno conventions as geometry builders.

#### Altitude (template)

Other mavros types (`VfrHud`, `EstimatorStatus`, `ExtendedState`,
`SysStatus`, `State`, `StatusText`, `GpsRaw`, `TimesyncStatus`) follow the
same view + builder pattern; see `schemas.h` for the full setter list.

```c
mavros_msgs_altitude_t* mavros_msgs_altitude_from_cdr(const uint8_t* data, size_t len);
void                   mavros_msgs_altitude_free(mavros_msgs_altitude_t* view);

int32_t     mavros_msgs_altitude_get_stamp_sec(const mavros_msgs_altitude_t* view);
uint32_t    mavros_msgs_altitude_get_stamp_nanosec(const mavros_msgs_altitude_t* view);
const char* mavros_msgs_altitude_get_frame_id(const mavros_msgs_altitude_t* view);
float       mavros_msgs_altitude_get_monotonic(const mavros_msgs_altitude_t* view);
float       mavros_msgs_altitude_get_amsl(const mavros_msgs_altitude_t* view);
float       mavros_msgs_altitude_get_local(const mavros_msgs_altitude_t* view);
float       mavros_msgs_altitude_get_relative(const mavros_msgs_altitude_t* view);
float       mavros_msgs_altitude_get_terrain(const mavros_msgs_altitude_t* view);
float       mavros_msgs_altitude_get_bottom_clearance(const mavros_msgs_altitude_t* view);

const uint8_t* mavros_msgs_altitude_as_cdr(const mavros_msgs_altitude_t* view, size_t* out_len);
```

**Builder:**

```c
mavros_msgs_altitude_builder_t* mavros_msgs_altitude_builder_new(void);
void mavros_msgs_altitude_builder_free(mavros_msgs_altitude_builder_t* b);

void mavros_msgs_altitude_builder_set_stamp(mavros_msgs_altitude_builder_t* b,
                                           int32_t sec, uint32_t nanosec);
int  mavros_msgs_altitude_builder_set_frame_id(mavros_msgs_altitude_builder_t* b,
                                              const char* s);
void mavros_msgs_altitude_builder_set_monotonic(mavros_msgs_altitude_builder_t* b, float v);
void mavros_msgs_altitude_builder_set_amsl(mavros_msgs_altitude_builder_t* b, float v);
void mavros_msgs_altitude_builder_set_local(mavros_msgs_altitude_builder_t* b, float v);
void mavros_msgs_altitude_builder_set_relative(mavros_msgs_altitude_builder_t* b, float v);
void mavros_msgs_altitude_builder_set_terrain(mavros_msgs_altitude_builder_t* b, float v);
void mavros_msgs_altitude_builder_set_bottom_clearance(mavros_msgs_altitude_builder_t* b, float v);

int  mavros_msgs_altitude_builder_build(mavros_msgs_altitude_builder_t* b,
                                       uint8_t** out_bytes, size_t* out_len);
int  mavros_msgs_altitude_builder_encode_into(mavros_msgs_altitude_builder_t* b,
                                            uint8_t* buf, size_t cap, size_t* out_len);
```

---

### foxglove_msgs — Buffer-backed

#### CompressedVideo

```c
foxglove_msgs_compressed_video_t* foxglove_msgs_compressed_video_from_cdr(const uint8_t* data,
                                                       size_t len);
void foxglove_msgs_compressed_video_free(foxglove_msgs_compressed_video_t* view);

int32_t     foxglove_msgs_compressed_video_get_stamp_sec(const foxglove_msgs_compressed_video_t* view);
uint32_t    foxglove_msgs_compressed_video_get_stamp_nanosec(const foxglove_msgs_compressed_video_t* view);
const char* foxglove_msgs_compressed_video_get_frame_id(const foxglove_msgs_compressed_video_t* view);
const uint8_t* foxglove_msgs_compressed_video_get_data(const foxglove_msgs_compressed_video_t* view,
                                              size_t* out_len);
const char* foxglove_msgs_compressed_video_get_format(const foxglove_msgs_compressed_video_t* view);

const uint8_t* foxglove_msgs_compressed_video_as_cdr(const foxglove_msgs_compressed_video_t* view,
                                            size_t* out_len);

// Builder API: foxglove_msgs_compressed_video_builder_new / _set_* / _build
```

#### CompressedImage

Wire-identical to `CompressedVideo`; only the typename and the meaning of
`format` differ (image media types such as `jpeg`/`png`/`webp` rather than
video codecs). The short `sensor_msgs_compressed_image_` prefix belongs to
`sensor_msgs::CompressedImage`, so the Foxglove variant uses the
`foxglove_msgs_compressed_image_` prefix.

```c
foxglove_msgs_compressed_image_t* foxglove_msgs_compressed_image_from_cdr(const uint8_t* data,
                                                                         size_t len);
void foxglove_msgs_compressed_image_free(foxglove_msgs_compressed_image_t* view);

int32_t     foxglove_msgs_compressed_image_get_stamp_sec(const foxglove_msgs_compressed_image_t* view);
uint32_t    foxglove_msgs_compressed_image_get_stamp_nanosec(const foxglove_msgs_compressed_image_t* view);
const char* foxglove_msgs_compressed_image_get_frame_id(const foxglove_msgs_compressed_image_t* view);
const uint8_t* foxglove_msgs_compressed_image_get_data(const foxglove_msgs_compressed_image_t* view,
                                                      size_t* out_len);
const char* foxglove_msgs_compressed_image_get_format(const foxglove_msgs_compressed_image_t* view);

const uint8_t* foxglove_msgs_compressed_image_as_cdr(const foxglove_msgs_compressed_image_t* view,
                                                    size_t* out_len);

// Builder API: foxglove_msgs_compressed_image_builder_new / _set_* / _build
```

---

### edgefirst_msgs — Buffer-backed

#### Mask

```c
edgefirst_msgs_mask_t* edgefirst_msgs_mask_from_cdr(const uint8_t* data, size_t len);
void        edgefirst_msgs_mask_free(edgefirst_msgs_mask_t* view);

uint32_t    edgefirst_msgs_mask_get_height(const edgefirst_msgs_mask_t* view);
uint32_t    edgefirst_msgs_mask_get_width(const edgefirst_msgs_mask_t* view);
uint32_t    edgefirst_msgs_mask_get_length(const edgefirst_msgs_mask_t* view);
const char* edgefirst_msgs_mask_get_encoding(const edgefirst_msgs_mask_t* view);
const uint8_t* edgefirst_msgs_mask_get_data(const edgefirst_msgs_mask_t* view, size_t* out_len);
bool        edgefirst_msgs_mask_get_boxed(const edgefirst_msgs_mask_t* view);
```

> **Note:** `edgefirst_msgs_mask_as_cdr` has been removed. Forwarding an embedded child mask
> as a standalone CDR would require re-encoding, which violates the zero-copy
> contract. Child masks accessed via `edgefirst_msgs_model_get_mask` are embedded in the
> parent buffer and do not have an independent CDR form.

// Builder API: edgefirst_msgs_mask_builder_new / _set_* / _build / _encode_into
```

#### Tensor family

`Tensor` is the payload; `TensorStamped` and `CameraFrame` are byte-identical
wrappers around it. The C API mirrors that composition rather than flattening
it — a wrapper hands out a borrowed `const edgefirst_msgs_tensor_t*`, so the tensor
accessors are shared rather than duplicated per wrapper.

```c
edgefirst_msgs_camera_frame_t*       f = edgefirst_msgs_camera_frame_from_cdr(buf, len);
const edgefirst_msgs_tensor_t*       t = edgefirst_msgs_camera_frame_get_tensor(f);
const edgefirst_msgs_tensor_plane_t* p = edgefirst_msgs_tensor_get_plane(t, 0);
int64_t                   h = edgefirst_msgs_tensor_plane_get_handle(p);
edgefirst_msgs_camera_frame_free(f);   /* frees the borrowed tensor and planes too */
```

**Ownership.** `<package>_<wrapper>_get_tensor()` and `edgefirst_msgs_tensor_get_plane()`
return BORROWED pointers into the parent handle. Do not free them; they die
with the parent. Freeing one anyway is a no-op that sets `errno = EINVAL`
rather than a double free (Memory Management Rule 5).

##### Tensor

```c
edgefirst_msgs_tensor_t* edgefirst_msgs_tensor_from_cdr(const uint8_t* data, size_t len);
void          edgefirst_msgs_tensor_free(edgefirst_msgs_tensor_t* view);

uint32_t    edgefirst_msgs_tensor_get_storage_kind(const edgefirst_msgs_tensor_t* view);
uint32_t    edgefirst_msgs_tensor_get_pid(const edgefirst_msgs_tensor_t* view);
int32_t     edgefirst_msgs_tensor_get_fence_fd(const edgefirst_msgs_tensor_t* view);   /* -1 = none */
uint32_t    edgefirst_msgs_tensor_get_dtype(const edgefirst_msgs_tensor_t* view);
int32_t     edgefirst_msgs_tensor_get_quant_axis(const edgefirst_msgs_tensor_t* view); /* -2 = unquantized */
uint32_t    edgefirst_msgs_tensor_get_num_planes(const edgefirst_msgs_tensor_t* view);

const char* edgefirst_msgs_tensor_get_format(const edgefirst_msgs_tensor_t* view);
const char* edgefirst_msgs_tensor_get_color_space(const edgefirst_msgs_tensor_t* view);
const char* edgefirst_msgs_tensor_get_color_transfer(const edgefirst_msgs_tensor_t* view);
const char* edgefirst_msgs_tensor_get_color_encoding(const edgefirst_msgs_tensor_t* view);
const char* edgefirst_msgs_tensor_get_color_range(const edgefirst_msgs_tensor_t* view);

uint32_t  edgefirst_msgs_tensor_get_shape_len(const edgefirst_msgs_tensor_t* view);
uint32_t  edgefirst_msgs_tensor_get_strides_len(const edgefirst_msgs_tensor_t* view);
int32_t   edgefirst_msgs_tensor_get_shape_at(const edgefirst_msgs_tensor_t* view, uint32_t i, uint64_t* out);
int32_t   edgefirst_msgs_tensor_get_strides_at(const edgefirst_msgs_tensor_t* view, uint32_t i, int64_t* out);
ptrdiff_t edgefirst_msgs_tensor_copy_shape(const edgefirst_msgs_tensor_t* view, uint64_t* out, size_t cap);
ptrdiff_t edgefirst_msgs_tensor_copy_strides(const edgefirst_msgs_tensor_t* view, int64_t* out, size_t cap);

const float*   edgefirst_msgs_tensor_get_quant_scales(const edgefirst_msgs_tensor_t* view, size_t* out_len);
const int32_t* edgefirst_msgs_tensor_get_quant_zero_points(const edgefirst_msgs_tensor_t* view, size_t* out_len);

const edgefirst_msgs_tensor_plane_t* edgefirst_msgs_tensor_get_plane(const edgefirst_msgs_tensor_t* view, uint32_t index);
const uint8_t* edgefirst_msgs_tensor_get_tensor_bytes(const edgefirst_msgs_tensor_t* view, size_t* out_len);
int32_t edgefirst_msgs_tensor_to_standalone_cdr(const edgefirst_msgs_tensor_t* view,
                                     uint8_t** out_bytes, size_t* out_len);
```

**Why `shape` and `strides` are not borrowed pointers.** CDR 8-byte alignment
is relative to the data start at absolute offset 4, so their elements land at
absolute offset ≡ 4 (mod 8) and can never satisfy `alignof(uint64_t)`.
Returning a `const uint64_t*` into the buffer would invite misaligned loads —
undefined behaviour on strict-alignment targets. Access is index- or
bulk-copy based instead. Prefer `edgefirst_msgs_tensor_copy_shape()` to a loop of
`edgefirst_msgs_tensor_get_shape_at()`: the former is O(n) for the whole sequence, the
latter rescans and is O(n²).

`quant_scales` / `quant_zero_points` are 4-byte elements whose CDR alignment
does coincide with their natural alignment, so those stay borrowed.

**`shape` is the addressing grid, not the byte layout.** An NV12 frame carries
`shape == [h, w]` with a U8 dtype against an `h*w*3/2` allocation; it is
deliberately never validated against any buffer size. `strides` is in bytes.

##### TensorPlane

```c
void edgefirst_msgs_tensor_plane_free(edgefirst_msgs_tensor_plane_t* view);   /* borrowed: no-op + EINVAL */

int64_t  edgefirst_msgs_tensor_plane_get_handle(const edgefirst_msgs_tensor_plane_t* view); /* -1 = inline */
uint64_t edgefirst_msgs_tensor_plane_get_offset(const edgefirst_msgs_tensor_plane_t* view);
uint64_t edgefirst_msgs_tensor_plane_get_stride(const edgefirst_msgs_tensor_plane_t* view);
uint64_t edgefirst_msgs_tensor_plane_get_size(const edgefirst_msgs_tensor_plane_t* view);
uint64_t edgefirst_msgs_tensor_plane_get_used(const edgefirst_msgs_tensor_plane_t* view);   /* <= size */
uint64_t edgefirst_msgs_tensor_plane_get_modifier(const edgefirst_msgs_tensor_plane_t* view);
bool     edgefirst_msgs_tensor_plane_is_inline(const edgefirst_msgs_tensor_plane_t* view);

const uint8_t* edgefirst_msgs_tensor_plane_get_handle_bytes(const edgefirst_msgs_tensor_plane_t* view, size_t* out_len);
const uint8_t* edgefirst_msgs_tensor_plane_get_data(const edgefirst_msgs_tensor_plane_t* view, size_t* out_len);
```

##### TensorStamped and CameraFrame

Identical shapes; substitute `tensor_stamped` for `camera_frame` throughout.

```c
edgefirst_msgs_camera_frame_t* edgefirst_msgs_camera_frame_from_cdr(const uint8_t* data, size_t len);
void                edgefirst_msgs_camera_frame_free(edgefirst_msgs_camera_frame_t* view);

int32_t     edgefirst_msgs_camera_frame_get_stamp_sec(const edgefirst_msgs_camera_frame_t* view);
uint32_t    edgefirst_msgs_camera_frame_get_stamp_nanosec(const edgefirst_msgs_camera_frame_t* view);
const char* edgefirst_msgs_camera_frame_get_frame_id(const edgefirst_msgs_camera_frame_t* view);
uint64_t    edgefirst_msgs_camera_frame_get_seq(const edgefirst_msgs_camera_frame_t* view);

const edgefirst_msgs_tensor_t* edgefirst_msgs_camera_frame_get_tensor(const edgefirst_msgs_camera_frame_t* view);
const uint8_t*      edgefirst_msgs_camera_frame_get_cdr(const edgefirst_msgs_camera_frame_t* view, size_t* out_len);

/* In-place header edits on a mutable CDR buffer */
int32_t edgefirst_msgs_camera_frame_set_stamp(uint8_t* buf, size_t len, int32_t sec, uint32_t nsec);
int32_t edgefirst_msgs_camera_frame_set_seq(uint8_t* buf, size_t len, uint64_t v);
```

##### Builders

Build the payload with a tensor builder, then attach it to a wrapper builder.
The tensor builder is BORROWED and must outlive the wrapper's build call.

```c
edgefirst_msgs_tensor_builder_t* tb = edgefirst_msgs_tensor_builder_new();
edgefirst_msgs_tensor_builder_set_dtype(tb, 1);
edgefirst_msgs_tensor_builder_set_format(tb, "NV12");
edgefirst_msgs_tensor_builder_set_shape(tb, (const uint64_t[]){480, 640}, 2);

edgefirst_msgs_tensor_plane_elem_t planes[2] = {
    { .handle = fd, .offset = 0,         .stride = 640,
      .size = 640 * 480,     .used = 640 * 480 },
    { .handle = fd, .offset = 640 * 480, .stride = 640,
      .size = 640 * 480 / 2, .used = 640 * 480 / 2 },
};
edgefirst_msgs_tensor_builder_set_planes(tb, planes, 2);

edgefirst_msgs_camera_frame_builder_t* fb = edgefirst_msgs_camera_frame_builder_new();
edgefirst_msgs_camera_frame_builder_set_stamp(fb, 42, 0);
edgefirst_msgs_camera_frame_builder_set_frame_id(fb, "camera_0");
edgefirst_msgs_camera_frame_builder_set_seq(fb, 1);
edgefirst_msgs_camera_frame_builder_set_tensor(fb, tb);

uint8_t* bytes = NULL; size_t len = 0;
if (edgefirst_msgs_camera_frame_builder_build(fb, &bytes, &len) == 0) {
    /* publish bytes[0..len) */
    edgefirst_schemas_bytes_free(bytes, len);
}
edgefirst_msgs_camera_frame_builder_free(fb);
edgefirst_msgs_tensor_builder_free(tb);
```

Builder defaults are NOT all zero: `fence_fd` starts at `-1` (no fence) and
`quant_axis` at `-2` (unquantized), matching the schema's "absent" values.
Sequence setters BORROW the caller's arrays until build or free.

Validation happens at build time and reports `errno = EBADMSG`: a plane set
that mixes inline and referenced modes is rejected, as is `used > size`, an
inline plane whose `size` disagrees with its `data`, a `strides` rank that
does not match `shape`, a `quant_scales` length that does not match
`quant_axis`, and colorimetry set without a `format`.

#### RadarCube

```c
edgefirst_msgs_radar_cube_t* edgefirst_msgs_radar_cube_from_cdr(const uint8_t* data, size_t len);
void              edgefirst_msgs_radar_cube_free(edgefirst_msgs_radar_cube_t* view);

int32_t     edgefirst_msgs_radar_cube_get_stamp_sec(const edgefirst_msgs_radar_cube_t* view);
uint32_t    edgefirst_msgs_radar_cube_get_stamp_nanosec(const edgefirst_msgs_radar_cube_t* view);
const char* edgefirst_msgs_radar_cube_get_frame_id(const edgefirst_msgs_radar_cube_t* view);
uint64_t    edgefirst_msgs_radar_cube_get_timestamp(const edgefirst_msgs_radar_cube_t* view);
const uint8_t* edgefirst_msgs_radar_cube_get_layout(const edgefirst_msgs_radar_cube_t* view,
                                          size_t* out_len);
const uint8_t* edgefirst_msgs_radar_cube_get_cube_raw(const edgefirst_msgs_radar_cube_t* view,
                                            size_t* out_len);
uint32_t    edgefirst_msgs_radar_cube_get_cube_len(const edgefirst_msgs_radar_cube_t* view);
bool        edgefirst_msgs_radar_cube_get_is_complex(const edgefirst_msgs_radar_cube_t* view);

const uint8_t* edgefirst_msgs_radar_cube_as_cdr(const edgefirst_msgs_radar_cube_t* view, size_t* out_len);
```

The cube data is raw bytes. For `i16` radar samples, use `memcpy` for
portable access:

```c
size_t raw_len;
const uint8_t* raw = edgefirst_msgs_radar_cube_get_cube_raw(cube, &raw_len);
size_t n_samples = raw_len / sizeof(int16_t);

// Portable: copy to aligned buffer
int16_t* samples = (int16_t*)malloc(raw_len);
memcpy(samples, raw, raw_len);
// ... use samples[0..n_samples-1] ...
free(samples);
```

The returned pointer borrows from an internal `Vec<u8>` (alignment 1).
While real-world allocators typically over-align, this is **not** a
hard guarantee — direct casting (`(int16_t*)raw`) may be undefined
behavior on strictly-aligned targets. Use `memcpy` for portable code.
On little-endian targets the byte order matches CDR1-LE wire format,
so no byte-swapping is needed after copying.

#### RadarInfo

```c
edgefirst_msgs_radar_info_t* edgefirst_msgs_radar_info_from_cdr(const uint8_t* data, size_t len);
void              edgefirst_msgs_radar_info_free(edgefirst_msgs_radar_info_t* view);

int32_t     edgefirst_msgs_radar_info_get_stamp_sec(const edgefirst_msgs_radar_info_t* view);
uint32_t    edgefirst_msgs_radar_info_get_stamp_nanosec(const edgefirst_msgs_radar_info_t* view);
const char* edgefirst_msgs_radar_info_get_frame_id(const edgefirst_msgs_radar_info_t* view);
const char* edgefirst_msgs_radar_info_get_center_frequency(const edgefirst_msgs_radar_info_t* view);
const char* edgefirst_msgs_radar_info_get_frequency_sweep(const edgefirst_msgs_radar_info_t* view);
const char* edgefirst_msgs_radar_info_get_range_toggle(const edgefirst_msgs_radar_info_t* view);
const char* edgefirst_msgs_radar_info_get_detection_sensitivity(const edgefirst_msgs_radar_info_t* view);
bool        edgefirst_msgs_radar_info_get_cube(const edgefirst_msgs_radar_info_t* view);

const uint8_t* edgefirst_msgs_radar_info_as_cdr(const edgefirst_msgs_radar_info_t* view, size_t* out_len);
```

#### Detect

```c
edgefirst_msgs_detect_t* edgefirst_msgs_detect_from_cdr(const uint8_t* data, size_t len);
void          edgefirst_msgs_detect_free(edgefirst_msgs_detect_t* view);

int32_t     edgefirst_msgs_detect_get_stamp_sec(const edgefirst_msgs_detect_t* view);
uint32_t    edgefirst_msgs_detect_get_stamp_nanosec(const edgefirst_msgs_detect_t* view);
const char* edgefirst_msgs_detect_get_frame_id(const edgefirst_msgs_detect_t* view);
uint32_t    edgefirst_msgs_detect_get_boxes_len(const edgefirst_msgs_detect_t* view);
// Returns a borrowed pointer; do NOT free. See Rule 5.
const edgefirst_msgs_box_t* edgefirst_msgs_detect_get_box(const edgefirst_msgs_detect_t* view, uint32_t index);

const uint8_t* edgefirst_msgs_detect_as_cdr(const edgefirst_msgs_detect_t* view, size_t* out_len);
```

#### Model

```c
edgefirst_msgs_model_t* edgefirst_msgs_model_from_cdr(const uint8_t* data, size_t len);
void         edgefirst_msgs_model_free(edgefirst_msgs_model_t* view);

int32_t     edgefirst_msgs_model_get_stamp_sec(const edgefirst_msgs_model_t* view);
uint32_t    edgefirst_msgs_model_get_stamp_nanosec(const edgefirst_msgs_model_t* view);
const char* edgefirst_msgs_model_get_frame_id(const edgefirst_msgs_model_t* view);
uint32_t    edgefirst_msgs_model_get_boxes_len(const edgefirst_msgs_model_t* view);
uint32_t    edgefirst_msgs_model_get_masks_len(const edgefirst_msgs_model_t* view);
// Returns borrowed pointers; do NOT free. See Rule 5.
const edgefirst_msgs_box_t*  edgefirst_msgs_model_get_box(const edgefirst_msgs_model_t* view, uint32_t index);
const edgefirst_msgs_mask_t* edgefirst_msgs_model_get_mask(const edgefirst_msgs_model_t* view, uint32_t index);

const uint8_t* edgefirst_msgs_model_as_cdr(const edgefirst_msgs_model_t* view, size_t* out_len);
```

#### ModelInfo

```c
edgefirst_msgs_model_info_t* edgefirst_msgs_model_info_from_cdr(const uint8_t* data, size_t len);
void              edgefirst_msgs_model_info_free(edgefirst_msgs_model_info_t* view);

int32_t     edgefirst_msgs_model_info_get_stamp_sec(const edgefirst_msgs_model_info_t* view);
uint32_t    edgefirst_msgs_model_info_get_stamp_nanosec(const edgefirst_msgs_model_info_t* view);
const char* edgefirst_msgs_model_info_get_frame_id(const edgefirst_msgs_model_info_t* view);
const char* edgefirst_msgs_model_info_get_model_type(const edgefirst_msgs_model_info_t* view);
const char* edgefirst_msgs_model_info_get_model_format(const edgefirst_msgs_model_info_t* view);
const char* edgefirst_msgs_model_info_get_model_name(const edgefirst_msgs_model_info_t* view);
uint8_t     edgefirst_msgs_model_info_get_input_type(const edgefirst_msgs_model_info_t* view);
uint8_t     edgefirst_msgs_model_info_get_output_type(const edgefirst_msgs_model_info_t* view);

const uint8_t* edgefirst_msgs_model_info_as_cdr(const edgefirst_msgs_model_info_t* view, size_t* out_len);
```

#### Track

```c
edgefirst_msgs_track_t* edgefirst_msgs_track_from_cdr(const uint8_t* data, size_t len);
void         edgefirst_msgs_track_free(edgefirst_msgs_track_t* view);

const char* edgefirst_msgs_track_get_id(const edgefirst_msgs_track_t* view);
int32_t     edgefirst_msgs_track_get_lifetime(const edgefirst_msgs_track_t* view);

const uint8_t* edgefirst_msgs_track_as_cdr(const edgefirst_msgs_track_t* view, size_t* out_len);
```

#### DetectBox

```c
edgefirst_msgs_box_t* edgefirst_msgs_box_from_cdr(const uint8_t* data, size_t len);
void       edgefirst_msgs_box_free(edgefirst_msgs_box_t* view);

float       edgefirst_msgs_box_get_center_x(const edgefirst_msgs_box_t* view);
float       edgefirst_msgs_box_get_center_y(const edgefirst_msgs_box_t* view);
float       edgefirst_msgs_box_get_width(const edgefirst_msgs_box_t* view);
float       edgefirst_msgs_box_get_height(const edgefirst_msgs_box_t* view);
const char* edgefirst_msgs_box_get_label(const edgefirst_msgs_box_t* view);
float       edgefirst_msgs_box_get_score(const edgefirst_msgs_box_t* view);
float       edgefirst_msgs_box_get_distance(const edgefirst_msgs_box_t* view);
float       edgefirst_msgs_box_get_speed(const edgefirst_msgs_box_t* view);
const char* edgefirst_msgs_box_get_track_id(const edgefirst_msgs_box_t* view);
int32_t     edgefirst_msgs_box_get_track_lifetime(const edgefirst_msgs_box_t* view);
int32_t     edgefirst_msgs_box_get_track_created_sec(const edgefirst_msgs_box_t* view);
uint32_t    edgefirst_msgs_box_get_track_created_nanosec(const edgefirst_msgs_box_t* view);
```

> **Note:** `edgefirst_msgs_box_as_cdr` has been removed. Forwarding an embedded child box
> as a standalone CDR would require re-encoding, which violates the zero-copy
> contract. Child boxes accessed via `edgefirst_msgs_detect_get_box`/`edgefirst_msgs_model_get_box`
> are embedded in the parent buffer and do not have an independent CDR form.

#### LocalTime

```c
edgefirst_msgs_local_time_t* edgefirst_msgs_local_time_from_cdr(const uint8_t* data, size_t len);
void              edgefirst_msgs_local_time_free(edgefirst_msgs_local_time_t* view);

int32_t     edgefirst_msgs_local_time_get_stamp_sec(const edgefirst_msgs_local_time_t* view);
uint32_t    edgefirst_msgs_local_time_get_stamp_nanosec(const edgefirst_msgs_local_time_t* view);
const char* edgefirst_msgs_local_time_get_frame_id(const edgefirst_msgs_local_time_t* view);
int16_t     edgefirst_msgs_local_time_get_timezone(const edgefirst_msgs_local_time_t* view);

const uint8_t* edgefirst_msgs_local_time_as_cdr(const edgefirst_msgs_local_time_t* view, size_t* out_len);
```

---

## Examples

### CdrFixed: Encode and Decode a Time

```c
#include <edgefirst/schemas.h>
#include <stdio.h>

int main(void) {
    uint8_t buf[64];
    size_t written;

    // Encode
    if (builtin_interfaces_time_encode(buf, sizeof(buf), &written, 1234567890, 123456789) != 0) {
        perror("encode");
        return 1;
    }
    printf("Encoded Time: %zu bytes\n", written);

    // Decode
    int32_t sec;
    uint32_t nanosec;
    if (builtin_interfaces_time_decode(buf, written, &sec, &nanosec) != 0) {
        perror("decode");
        return 1;
    }
    printf("Decoded: %d.%09u\n", sec, nanosec);
    return 0;
}
```

### CdrFixed: Query Required Size

```c
// Pass buf=NULL to query the required size
size_t needed;
geometry_msgs_pose_encode(NULL, 0, &needed, 0, 0, 0, 0, 0, 0, 0);
uint8_t* buf = malloc(needed);
geometry_msgs_pose_encode(buf, needed, &needed, 1.0, 2.0, 0.5, 0.0, 0.0, 0.707, 0.707);
// ... use buf ...
free(buf);
```

### Buffer-backed: Decode an Image from CDR

```c
#include <edgefirst/schemas.h>
#include <stdio.h>

void process_image(const uint8_t* cdr_data, size_t cdr_len) {
    sensor_msgs_image_t* img = sensor_msgs_image_from_cdr(cdr_data, cdr_len);
    if (!img) {
        perror("sensor_msgs_image_from_cdr");
        return;
    }

    printf("Image: %ux%u %s\n",
           sensor_msgs_image_get_width(img),
           sensor_msgs_image_get_height(img),
           sensor_msgs_image_get_encoding(img));  // borrowed pointer

    size_t data_len;
    const uint8_t* pixels = sensor_msgs_image_get_data(img, &data_len);
    printf("Pixel data: %zu bytes at %p\n", data_len, (const void*)pixels);
    // pixels is borrowed — valid until sensor_msgs_image_free(img)

    sensor_msgs_image_free(img);
    // pixels is now dangling
}
```

### Buffer-backed: Encode a Header

```c
#include <edgefirst/schemas.h>
#include <stdio.h>

int main(void) {
    uint8_t* bytes = NULL;
    size_t len = 0;

    std_msgs_header_builder_t* b = std_msgs_header_builder_new();
    std_msgs_header_builder_set_stamp(b, 42, 0);
    if (std_msgs_header_builder_set_frame_id(b, "camera_frame") != 0 ||
        std_msgs_header_builder_build(b, &bytes, &len) != 0) {
        perror("std_msgs_header_builder");
        std_msgs_header_builder_free(b);
        return 1;
    }
    std_msgs_header_builder_free(b);
    printf("Encoded Header: %zu CDR bytes\n", len);

    // Verify round-trip
    std_msgs_header_t* hdr = std_msgs_header_from_cdr(bytes, len);
    printf("frame_id = %s\n", std_msgs_header_get_frame_id(hdr));

    std_msgs_header_free(hdr);
    edgefirst_schemas_bytes_free(bytes, len);  // NOT free(bytes)
    return 0;
}
```

### Buffer-backed: Forward a Message

A common pattern is receiving CDR bytes from Zenoh, inspecting metadata,
and forwarding the raw bytes unchanged:

```c
void on_message(const uint8_t* cdr_data, size_t cdr_len) {
    sensor_msgs_image_t* img = sensor_msgs_image_from_cdr(cdr_data, cdr_len);
    if (!img) return;

    // Inspect metadata
    int32_t sec = sensor_msgs_image_get_stamp_sec(img);
    const char* frame = sensor_msgs_image_get_frame_id(img);
    printf("[%d] %s: %ux%u\n", sec, frame,
           sensor_msgs_image_get_width(img), sensor_msgs_image_get_height(img));

    // Forward the original CDR bytes (no re-serialization needed)
    size_t fwd_len;
    const uint8_t* fwd = sensor_msgs_image_as_cdr(img, &fwd_len);
    // zenoh_publish(topic, fwd, fwd_len);

    sensor_msgs_image_free(img);
}
```

## C++ Integration

The C header is compatible with C++ via `extern "C"`. A thin RAII wrapper
avoids manual `_free` calls:

```cpp
extern "C" {
#include <edgefirst/schemas.h>
}

#include <stdexcept>

class ImageView {
    sensor_msgs_image_t* view_;
public:
    explicit ImageView(const uint8_t* cdr, size_t len)
        : view_(sensor_msgs_image_from_cdr(cdr, len))
    {
        if (!view_) throw std::runtime_error("sensor_msgs_image_from_cdr failed");
    }
    ~ImageView() { sensor_msgs_image_free(view_); }

    // Non-copyable
    ImageView(const ImageView&) = delete;
    ImageView& operator=(const ImageView&) = delete;

    // Movable
    ImageView(ImageView&& o) noexcept : view_(o.view_) { o.view_ = nullptr; }
    ImageView& operator=(ImageView&& o) noexcept {
        if (this != &o) { sensor_msgs_image_free(view_); view_ = o.view_; o.view_ = nullptr; }
        return *this;
    }

    uint32_t width() const  { return sensor_msgs_image_get_width(view_); }
    uint32_t height() const { return sensor_msgs_image_get_height(view_); }
    const char* encoding() const { return sensor_msgs_image_get_encoding(view_); }

    std::pair<const uint8_t*, size_t> data() const {
        size_t len;
        auto* p = sensor_msgs_image_get_data(view_, &len);
        return {p, len};
    }
};
```

## Zenoh Integration

A typical pattern is subscribing to Zenoh topics carrying CDR payloads:

```c
#include <zenoh-c.h>
#include <edgefirst/schemas.h>

void on_image(z_loaned_sample_t* sample, void* arg) {
    z_view_slice_t payload = z_sample_payload(sample);
    const uint8_t* data = z_slice_data(&payload);
    size_t len = z_slice_len(&payload);

    sensor_msgs_image_t* img = sensor_msgs_image_from_cdr(data, len);
    if (!img) return;

    printf("[%d.%09u] %s: %ux%u %s\n",
           sensor_msgs_image_get_stamp_sec(img),
           sensor_msgs_image_get_stamp_nanosec(img),
           sensor_msgs_image_get_frame_id(img),
           sensor_msgs_image_get_width(img),
           sensor_msgs_image_get_height(img),
           sensor_msgs_image_get_encoding(img));

    // Forward the original CDR bytes (zero re-serialization cost)
    size_t cdr_len;
    const uint8_t* cdr = sensor_msgs_image_as_cdr(img, &cdr_len);
    // z_publisher_put(pub, cdr, cdr_len, NULL);

    sensor_msgs_image_free(img);
}
```

Publishing a constructed message:

```c
void publish_header(z_loaned_publisher_t* pub) {
    uint8_t* bytes = NULL;
    size_t len = 0;

    std_msgs_header_builder_t* b = std_msgs_header_builder_new();
    std_msgs_header_builder_set_stamp(b, 42, 0);
    if (std_msgs_header_builder_set_frame_id(b, "camera_frame") != 0 ||
        std_msgs_header_builder_build(b, &bytes, &len) != 0) {
        std_msgs_header_builder_free(b);
        return;
    }
    std_msgs_header_builder_free(b);

    z_publisher_put(pub, bytes, len, NULL);
    edgefirst_schemas_bytes_free(bytes, len);  // NOT free(bytes)
}
```

## Troubleshooting

### Library Not Found at Runtime

```bash
# Linux: check what the binary expects
ldd myapp | grep edgefirst
# Fix: set LD_LIBRARY_PATH or install to /usr/local/lib + ldconfig

# macOS: check linked paths
otool -L myapp | grep edgefirst
# Fix: set DYLD_LIBRARY_PATH or use install_name_tool
```

### Undefined Symbols at Link Time

```bash
# Verify the library exports the expected symbols
nm -D target/release/libedgefirst_schemas.so | grep std_msgs_header
# If empty, rebuild: make lib
```

### SONAME Versioning

The shared library embeds a SONAME of `libedgefirst_schemas.so.MAJOR` (for
example `libedgefirst_schemas.so.2` for the 2.x series). Releases ship with
the standard GNU/Linux chain so both the linker (`-ledgefirst_schemas`) and
the runtime loader (via SONAME) find the library:

```
libedgefirst_schemas.so                       symlink -> libedgefirst_schemas.so.2
libedgefirst_schemas.so.2                     symlink -> libedgefirst_schemas.so.2.2
libedgefirst_schemas.so.2.2                   symlink -> libedgefirst_schemas.so.2.2.1
libedgefirst_schemas.so.2.2.1                 real file
```

The ELF `DT_SONAME` is `libedgefirst_schemas.so.2` — that is the name
the runtime loader opens, which then resolves through the chain above
to the real file.

`make lib` produces the same layout under `target/release/`, and the release
packages published by the [`release.yml`](.github/workflows/release.yml)
workflow lay the library out this way in `lib/`. After a manual install, run
`sudo ldconfig` so the loader picks up the new SONAME mapping.

### Segfaults and Memory Errors

Common causes:

1. **Use-after-free** — Using a borrowed pointer (`get_frame_id`, `get_data`,
   `as_cdr`) after calling `_free` on the handle.
2. **Double-free** — Calling `free()` on a borrowed pointer. Only call `_free`
   on handles, and `edgefirst_schemas_bytes_free` on encode output.
3. **Wrong free function** — Using `free()` on encode output instead of
   `edgefirst_schemas_bytes_free(bytes, len)`. The memory is allocated by Rust's allocator.
4. **NULL handle** — Getters return 0 or `NULL` for NULL handles rather than
   crashing, but always check `from_cdr` return values.

## C++ Wrapper

A header-only C++17 wrapper is available at
`crates/capi/include/edgefirst/schemas.hpp`. It provides RAII-managed view and owning
types under nested package namespaces — e.g.
`sensor_msgs::ImageView`/`Image`, `std_msgs::HeaderView`/`Header`,
`edgefirst_msgs::DetectView`, `edgefirst_msgs::ModelView`,
`edgefirst_msgs::MaskView`/`Mask` — together with
`expected<T, Error>` error handling (no exceptions) and range-based iteration
for array children:

```cpp
namespace ef = edgefirst::schemas;
using ef::edgefirst_msgs::DetectView;

// Decode a Detect message and iterate its boxes
auto det = DetectView::from_cdr(payload);
for (auto box : det->boxes()) {
    std::cout << "label=" << box.label() << " score=" << box.score() << "\n";
}
```

The range adaptors (`boxes()`, `masks()`) use the same indexed accessor
functions documented in Rule 5 of the [Memory Management](#memory-management)
section above (`edgefirst_msgs_detect_get_box`, `edgefirst_msgs_model_get_box`,
`edgefirst_msgs_model_get_mask`). Child views returned from the range are parent-borrowed
and must not be freed independently — exactly the same lifetime rule that
makes zero-copy iteration possible.

The C++ wrapper links against the same `libedgefirst_schemas` shared library
as the C API — there is no separate build target or additional library. Include
the C++ header and link with `-ledgefirst_schemas` exactly as you would for C.

See [README.md § C++ Usage](README.md#c-usage) for the full introduction,
installation notes, and working examples. The per-type API surface (constructors,
field accessors, encode/decode signatures) is documented inline in
`crates/capi/include/edgefirst/schemas.hpp`.

## Building from Source

```bash
# Build the shared library. `cargo build --release` writes the shipped names
# directly; `make lib` additionally drops the soversion hop that the runtime
# loader needs (libedgefirst_schemas.so.MAJOR / libedgefirst_schemas.MAJOR.dylib),
# so prefer it when you intend to run anything linked against the library.
make lib

# The library is at:
#   target/release/libedgefirst_schemas.so      (Linux)
#   target/release/libedgefirst_schemas.dylib   (macOS)
#   target/release/edgefirst_schemas.dll        (Windows)
#
# The header is at:
#   crates/capi/include/edgefirst/schemas.h

# Run the C API test suite (requires libcriterion-dev)
sudo apt-get install -y libcriterion-dev
mkdir -p build
for src in crates/capi/tests/c/test_*.c; do
    name=$(basename $src .c)
    gcc -Wall -Wextra -Werror -std=c11 -I./crates/capi/include \
        -o build/$name $src \
        -L./target/release -ledgefirst_schemas -lcriterion -lm \
        -Wl,-rpath,./target/release
done
for test in build/test_*; do ./$test; done
```

### Cross-compilation

For ARM64 targets (e.g., NXP i.MX 8M Plus):

```bash
cargo zigbuild --release --target aarch64-unknown-linux-gnu
# Library at: target/aarch64-unknown-linux-gnu/release/libedgefirst_schemas.so
#
# A cross-target build does not run `make lib`, so the soversion hop is absent.
# Create it if you intend to run the result on the target:
#   ln -sf libedgefirst_schemas.so \
#          target/aarch64-unknown-linux-gnu/release/libedgefirst_schemas.so.3
```

See [Zig cross-compilation](https://github.com/nickel-lang/topiary/wiki/Cross-compilation-with-Zig)
for setup details.
