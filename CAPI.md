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
sudo cp include/edgefirst/schemas.h /usr/local/include/edgefirst/

# Install library (Linux) — install the versioned real file and create the
# symlink chain expected by the GNU/Linux dynamic linker. Replace 2.2.1 with
# the version you built.
VERSION=2.2.1; MAJOR=${VERSION%%.*}; MM=${VERSION%.*}
sudo install -m 644 target/release/libedgefirst_schemas.so \
    /usr/local/lib/libedgefirst_schemas.so.${VERSION}
sudo ln -sf libedgefirst_schemas.so.${VERSION} /usr/local/lib/libedgefirst_schemas.so.${MM}
sudo ln -sf libedgefirst_schemas.so.${MM}      /usr/local/lib/libedgefirst_schemas.so.${MAJOR}
sudo ln -sf libedgefirst_schemas.so.${MAJOR}   /usr/local/lib/libedgefirst_schemas.so

# Install library (macOS) — dylib versioning is handled differently; a plain
# copy is sufficient for development installs.
sudo cp target/release/libedgefirst_schemas.dylib /usr/local/lib/

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

Function and type names follow a consistent prefix scheme based on message origin:

| Origin | Function prefix | Type suffix | Source |
|--------|----------------|-------------|--------|
| ROS 2 common_interfaces | `ros_` | `ros_<type>_t` | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) |
| Foxglove schemas | `ros_compressed_video_` | `ros_compressed_video_t` | [foxglove-sdk](https://github.com/foxglove/foxglove-sdk) |
| EdgeFirst custom | `ros_dmabuffer_`, `ros_detect_`, etc. | `ros_<type>_t` | edgefirst_msgs/msg |

**Snake-case conversion rules:**

| Rust / ROS type | C function prefix |
|-----------------|-------------------|
| `PointCloud2` | `ros_point_cloud2_` |
| `NavSatFix` | `ros_nav_sat_fix_` |
| `DmaBuffer` | `ros_dmabuffer_` |
| `CompressedImage` | `ros_compressed_image_` |
| `TransformStamped` | `ros_transform_stamped_` |

Acronyms are treated as single words: `Dma` → `dma`, `Sat` → `sat`.
Numbers stay attached to the preceding word: `Cloud2` → `cloud2`.

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

**Pattern:**

```c
// Encode into a stack buffer
uint8_t buf[64];
size_t written;
ros_time_encode(buf, sizeof(buf), &written, sec, nanosec);

// Decode from CDR bytes
int32_t sec;
uint32_t nanosec;
ros_time_decode(data, len, &sec, &nanosec);
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
| TransformStamped | geometry_msgs |
| CompressedVideo | foxglove_msgs |
| DmaBuffer | edgefirst_msgs |
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
// IMPORTANT: `data` must remain valid until ros_image_free(img).
ros_image_t* img = ros_image_from_cdr(data, len);

// Access fields — O(1), pointers borrow from the caller's buffer
uint32_t w = ros_image_get_width(img);
const char* enc = ros_image_get_encoding(img);      // borrowed from data
const uint8_t* px = ros_image_get_data(img, &pxlen); // borrowed from data

// Re-export the raw CDR bytes (points into the original data buffer)
const uint8_t* cdr = ros_image_as_cdr(img, &cdr_len); // borrowed from data

// Free the handle, THEN free the source data buffer
ros_image_free(img);
// data can now be freed safely
```

```c
// Encode: construct a message directly from field values
uint8_t* bytes = NULL;
size_t len = 0;
ros_image_encode(&bytes, &len,
    stamp_sec, stamp_nanosec, "camera",
    480, 640, "rgb8", 0, 1920,
    pixel_data, pixel_data_len);

// ... use bytes ...
ros_bytes_free(bytes, len);
```

Every buffer-backed type provides five entry points:

| Function | Purpose |
|----------|---------|
| `ros_<type>_from_cdr(data, len)` | Zero-copy view over caller's CDR buffer (data must outlive handle) |
| `ros_<type>_get_<field>(handle)` | Read a field (O(1), zero-copy for strings/blobs) |
| `ros_<type>_as_cdr(handle, &len)` | Borrow the raw CDR buffer (points into caller's data) |
| `ros_<type>_free(handle)` | Release the handle (does NOT free the source data) |
| `ros_<type>_encode(...)` | Construct CDR bytes from field values (allocates) |

## Memory Management

**Rule 1 — Free handles with the matching `_free()` function.**
Every `from_cdr` or `encode` call that returns a handle must be balanced by
a `_free` call. Passing `NULL` to any `_free` function is safe (no-op).

**Rule 1a — Keep source data alive until the handle is freed.**
`from_cdr` creates a zero-copy view — the handle borrows the caller's `data`
buffer directly. The `data` pointer must remain valid until `_free()` is called.
Freeing `data` before the handle causes undefined behavior.

**Rule 2 — Free encode output with `ros_bytes_free()`.**
Buffer-backed `_encode` functions allocate output via `uint8_t**`.
Free this memory with `ros_bytes_free(bytes, len)`. Do **not** call
`free()` directly — the memory is allocated by the Rust runtime.

**Rule 3 — Do NOT free borrowed pointers.**
String getters (`const char*`) and blob getters (`const uint8_t*`) return
pointers into the caller's original CDR buffer. These are valid as long as
both the handle and the source data remain alive. Do not free them.

```c
ros_header_t* hdr = ros_header_from_cdr(data, len);

const char* frame = ros_header_get_frame_id(hdr);
printf("%s\n", frame);
// Do NOT call free(frame) — it points into the caller's data buffer

ros_header_free(hdr);
// frame is now dangling — do not use after free
// data can now be freed safely
```

**Rule 4 — CdrFixed types use caller-owned buffers.**
`_encode` writes into a `uint8_t[]` you provide. `_decode` reads from
a `const uint8_t*` you provide. No heap allocation, nothing to free.

**Rule 5 — Parent-borrowed child handles.**
Indexed accessors like `ros_detect_get_box(view, i)`, `ros_model_get_box(view, i)`,
and `ros_model_get_mask(view, i)` return pointers to child view handles owned by
the parent. These are NOT separately allocated — they live inside the parent's
internal `child_boxes`/`child_masks` vector, which is populated once during
`from_cdr` and freed with the parent. Do not pass them to `ros_box_free()` /
`ros_mask_free()`; the parent owns them. They become invalid when the parent is
freed. The parent's CDR buffer (passed to `ros_<parent>_from_cdr`) must also remain
valid for as long as the child pointers are used.

### Summary

| Source | Who owns it | How to free |
|--------|-------------|-------------|
| `ros_<type>_from_cdr(...)` | Caller (handle + source data) | `ros_<type>_free(handle)`, then free source data |
| `ros_<type>_encode(&bytes, ...)` | Caller | `ros_bytes_free(bytes, len)` |
| `ros_<type>_get_<field>(handle)` | Source data (via handle) | Do not free |
| `ros_<type>_as_cdr(handle, ...)` | Source data (via handle) | Do not free |
| `ros_detect_get_box(handle, i)` | Parent handle | Do NOT free; owned by parent |
| `ros_model_get_box(handle, i)` | Parent handle | Do NOT free; owned by parent |
| `ros_model_get_mask(handle, i)` | Parent handle | Do NOT free; owned by parent |
| CdrFixed `_encode(buf, ...)` | Caller | Stack/caller buffer, no free needed |

## Functions removed in 3.0.0

The following C API functions were removed in 3.0.0 as part of the refactor
that made `ros_box_t` and `ros_mask_t` parent-borrowed view types:

- **`ros_box_as_cdr(box, &len)`** — removed. Child boxes inside a Detect have
  no independent CDR encoding; producing one would require re-encoding the
  box into a fresh buffer, which violates the library's zero-copy contract.
  **Migration**: forward the parent Detect via `ros_detect_as_cdr(detect, &len)`
  and let subscribers iterate with `ros_detect_get_box(detect, i)`.
- **`ros_mask_as_cdr(mask, &len)`** — removed for the same reason. Forward
  the parent Model via `ros_model_as_cdr` and iterate with `ros_model_get_mask`.

Standalone `ros_box_from_cdr(bytes, len)` and `ros_mask_from_cdr(bytes, len)`
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

ros_header_t* hdr = ros_header_from_cdr(data, len);
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
void ros_bytes_free(uint8_t* bytes, size_t len);
```

Free a byte buffer returned by any `ros_*_encode()` function. Passing
`NULL` is safe.

---

### builtin_interfaces

#### Time

```c
int ros_time_encode(uint8_t* buf, size_t cap, size_t* written,
                    int32_t sec, uint32_t nanosec);

int ros_time_decode(const uint8_t* data, size_t len,
                    int32_t* sec, uint32_t* nanosec);
```

**Fields:** `sec` (seconds since epoch), `nanosec` (nanosecond component).

#### Duration

```c
int ros_duration_encode(uint8_t* buf, size_t cap, size_t* written,
                        int32_t sec, uint32_t nanosec);

int ros_duration_decode(const uint8_t* data, size_t len,
                        int32_t* sec, uint32_t* nanosec);
```

**Fields:** `sec`, `nanosec`.

---

### geometry_msgs — CdrFixed

#### Vector3

```c
int ros_vector3_encode(uint8_t* buf, size_t cap, size_t* written,
                       double x, double y, double z);

int ros_vector3_decode(const uint8_t* data, size_t len,
                       double* x, double* y, double* z);
```

#### Point

```c
int ros_point_encode(uint8_t* buf, size_t cap, size_t* written,
                     double x, double y, double z);

int ros_point_decode(const uint8_t* data, size_t len,
                     double* x, double* y, double* z);
```

#### Quaternion

```c
int ros_quaternion_encode(uint8_t* buf, size_t cap, size_t* written,
                          double x, double y, double z, double w);

int ros_quaternion_decode(const uint8_t* data, size_t len,
                          double* x, double* y, double* z, double* w);
```

#### Pose

```c
int ros_pose_encode(uint8_t* buf, size_t cap, size_t* written,
                    double px, double py, double pz,
                    double ox, double oy, double oz, double ow);

int ros_pose_decode(const uint8_t* data, size_t len,
                    double* px, double* py, double* pz,
                    double* ox, double* oy, double* oz, double* ow);
```

**Fields:** `px/py/pz` = position, `ox/oy/oz/ow` = orientation quaternion.

#### Transform

```c
int ros_transform_encode(uint8_t* buf, size_t cap, size_t* written,
                         double tx, double ty, double tz,
                         double rx, double ry, double rz, double rw);

int ros_transform_decode(const uint8_t* data, size_t len,
                         double* tx, double* ty, double* tz,
                         double* rx, double* ry, double* rz, double* rw);
```

**Fields:** `tx/ty/tz` = translation, `rx/ry/rz/rw` = rotation quaternion.

#### Twist

```c
int ros_twist_encode(uint8_t* buf, size_t cap, size_t* written,
                     double lx, double ly, double lz,
                     double ax, double ay, double az);

int ros_twist_decode(const uint8_t* data, size_t len,
                     double* lx, double* ly, double* lz,
                     double* ax, double* ay, double* az);
```

**Fields:** `lx/ly/lz` = linear velocity, `ax/ay/az` = angular velocity.

#### Accel

```c
int ros_accel_encode(uint8_t* buf, size_t cap, size_t* written,
                     double lx, double ly, double lz,
                     double ax, double ay, double az);

int ros_accel_decode(const uint8_t* data, size_t len,
                     double* lx, double* ly, double* lz,
                     double* ax, double* ay, double* az);
```

**Fields:** `lx/ly/lz` = linear acceleration, `ax/ay/az` = angular acceleration.

---

### sensor_msgs — CdrFixed

#### NavSatStatus

```c
int ros_nav_sat_status_encode(uint8_t* buf, size_t cap, size_t* written,
                              int8_t status, uint16_t service);

int ros_nav_sat_status_decode(const uint8_t* data, size_t len,
                              int8_t* status, uint16_t* service);
```

**Fields:** `status` (fix status), `service` (service bitmask).

---

### std_msgs — Buffer-backed

#### Header

```c
ros_header_t* ros_header_from_cdr(const uint8_t* data, size_t len);
void          ros_header_free(ros_header_t* view);

int32_t      ros_header_get_stamp_sec(const ros_header_t* view);
uint32_t     ros_header_get_stamp_nanosec(const ros_header_t* view);
const char*  ros_header_get_frame_id(const ros_header_t* view);     // borrowed

const uint8_t* ros_header_as_cdr(const ros_header_t* view, size_t* out_len);

int ros_header_encode(uint8_t** out_bytes, size_t* out_len,
                      int32_t stamp_sec, uint32_t stamp_nanosec,
                      const char* frame_id);
```

---

### sensor_msgs — Buffer-backed

#### Image

```c
ros_image_t* ros_image_from_cdr(const uint8_t* data, size_t len);
void         ros_image_free(ros_image_t* view);

int32_t      ros_image_get_stamp_sec(const ros_image_t* view);
uint32_t     ros_image_get_stamp_nanosec(const ros_image_t* view);
const char*  ros_image_get_frame_id(const ros_image_t* view);       // borrowed
uint32_t     ros_image_get_height(const ros_image_t* view);
uint32_t     ros_image_get_width(const ros_image_t* view);
const char*  ros_image_get_encoding(const ros_image_t* view);       // borrowed
uint8_t      ros_image_get_is_bigendian(const ros_image_t* view);
uint32_t     ros_image_get_step(const ros_image_t* view);
const uint8_t* ros_image_get_data(const ros_image_t* view, size_t* out_len); // borrowed

const uint8_t* ros_image_as_cdr(const ros_image_t* view, size_t* out_len);

int ros_image_encode(uint8_t** out_bytes, size_t* out_len,
                     int32_t stamp_sec, uint32_t stamp_nanosec,
                     const char* frame_id,
                     uint32_t height, uint32_t width,
                     const char* encoding, uint8_t is_bigendian,
                     uint32_t step, const uint8_t* data, size_t data_len);
```

#### CompressedImage

```c
ros_compressed_image_t* ros_compressed_image_from_cdr(const uint8_t* data, size_t len);
void                    ros_compressed_image_free(ros_compressed_image_t* view);

int32_t      ros_compressed_image_get_stamp_sec(const ros_compressed_image_t* view);
uint32_t     ros_compressed_image_get_stamp_nanosec(const ros_compressed_image_t* view);
const char*  ros_compressed_image_get_frame_id(const ros_compressed_image_t* view);
const char*  ros_compressed_image_get_format(const ros_compressed_image_t* view);
const uint8_t* ros_compressed_image_get_data(const ros_compressed_image_t* view,
                                              size_t* out_len);

const uint8_t* ros_compressed_image_as_cdr(const ros_compressed_image_t* view,
                                            size_t* out_len);

int ros_compressed_image_encode(uint8_t** out_bytes, size_t* out_len,
                                int32_t stamp_sec, uint32_t stamp_nanosec,
                                const char* frame_id, const char* format,
                                const uint8_t* data, size_t data_len);
```

#### Imu

```c
ros_imu_t* ros_imu_from_cdr(const uint8_t* data, size_t len);
void       ros_imu_free(ros_imu_t* view);

int32_t     ros_imu_get_stamp_sec(const ros_imu_t* view);
uint32_t    ros_imu_get_stamp_nanosec(const ros_imu_t* view);
const char* ros_imu_get_frame_id(const ros_imu_t* view);

void ros_imu_get_orientation(const ros_imu_t* view,
                             double* x, double* y, double* z, double* w);
void ros_imu_get_orientation_covariance(const ros_imu_t* view, double* out);

void ros_imu_get_angular_velocity(const ros_imu_t* view,
                                  double* x, double* y, double* z);
void ros_imu_get_angular_velocity_covariance(const ros_imu_t* view, double* out);

void ros_imu_get_linear_acceleration(const ros_imu_t* view,
                                     double* x, double* y, double* z);
void ros_imu_get_linear_acceleration_covariance(const ros_imu_t* view, double* out);

const uint8_t* ros_imu_as_cdr(const ros_imu_t* view, size_t* out_len);
```

Covariance functions write 9 `double` values (row-major 3x3 matrix) into
the caller-provided `out` array. The caller must ensure `out` points to
at least 9 doubles.

**Note:** The Imu type has no `_encode` function. Construct Imu messages
from the Rust API or from CDR bytes received over Zenoh/DDS.

#### NavSatFix

```c
ros_nav_sat_fix_t* ros_nav_sat_fix_from_cdr(const uint8_t* data, size_t len);
void               ros_nav_sat_fix_free(ros_nav_sat_fix_t* view);

int32_t     ros_nav_sat_fix_get_stamp_sec(const ros_nav_sat_fix_t* view);
uint32_t    ros_nav_sat_fix_get_stamp_nanosec(const ros_nav_sat_fix_t* view);
const char* ros_nav_sat_fix_get_frame_id(const ros_nav_sat_fix_t* view);
double      ros_nav_sat_fix_get_latitude(const ros_nav_sat_fix_t* view);
double      ros_nav_sat_fix_get_longitude(const ros_nav_sat_fix_t* view);
double      ros_nav_sat_fix_get_altitude(const ros_nav_sat_fix_t* view);

const uint8_t* ros_nav_sat_fix_as_cdr(const ros_nav_sat_fix_t* view, size_t* out_len);
```

#### PointCloud2

```c
ros_point_cloud2_t* ros_point_cloud2_from_cdr(const uint8_t* data, size_t len);
void                ros_point_cloud2_free(ros_point_cloud2_t* view);

int32_t     ros_point_cloud2_get_stamp_sec(const ros_point_cloud2_t* view);
uint32_t    ros_point_cloud2_get_stamp_nanosec(const ros_point_cloud2_t* view);
const char* ros_point_cloud2_get_frame_id(const ros_point_cloud2_t* view);
uint32_t    ros_point_cloud2_get_height(const ros_point_cloud2_t* view);
uint32_t    ros_point_cloud2_get_width(const ros_point_cloud2_t* view);
uint32_t    ros_point_cloud2_get_point_step(const ros_point_cloud2_t* view);
uint32_t    ros_point_cloud2_get_row_step(const ros_point_cloud2_t* view);
const uint8_t* ros_point_cloud2_get_data(const ros_point_cloud2_t* view,
                                          size_t* out_len);
bool        ros_point_cloud2_get_is_dense(const ros_point_cloud2_t* view);
bool        ros_point_cloud2_get_is_bigendian(const ros_point_cloud2_t* view);
uint32_t    ros_point_cloud2_get_fields_len(const ros_point_cloud2_t* view);

const uint8_t* ros_point_cloud2_as_cdr(const ros_point_cloud2_t* view,
                                        size_t* out_len);
```

Point data is returned as raw `uint8_t*` bytes. Interpret them using
`point_step`, `row_step`, and the field definitions to extract typed
point values (x, y, z, intensity, etc.).

#### CameraInfo

```c
ros_camera_info_t* ros_camera_info_from_cdr(const uint8_t* data, size_t len);
void               ros_camera_info_free(ros_camera_info_t* view);

int32_t     ros_camera_info_get_stamp_sec(const ros_camera_info_t* view);
uint32_t    ros_camera_info_get_stamp_nanosec(const ros_camera_info_t* view);
const char* ros_camera_info_get_frame_id(const ros_camera_info_t* view);
uint32_t    ros_camera_info_get_height(const ros_camera_info_t* view);
uint32_t    ros_camera_info_get_width(const ros_camera_info_t* view);
const char* ros_camera_info_get_distortion_model(const ros_camera_info_t* view);
uint32_t    ros_camera_info_get_binning_x(const ros_camera_info_t* view);
uint32_t    ros_camera_info_get_binning_y(const ros_camera_info_t* view);

const uint8_t* ros_camera_info_as_cdr(const ros_camera_info_t* view,
                                       size_t* out_len);
```

---

### geometry_msgs — Buffer-backed

#### TransformStamped

```c
ros_transform_stamped_t* ros_transform_stamped_from_cdr(const uint8_t* data,
                                                         size_t len);
void ros_transform_stamped_free(ros_transform_stamped_t* view);

int32_t     ros_transform_stamped_get_stamp_sec(const ros_transform_stamped_t* view);
uint32_t    ros_transform_stamped_get_stamp_nanosec(const ros_transform_stamped_t* view);
const char* ros_transform_stamped_get_frame_id(const ros_transform_stamped_t* view);
const char* ros_transform_stamped_get_child_frame_id(const ros_transform_stamped_t* view);

const uint8_t* ros_transform_stamped_as_cdr(const ros_transform_stamped_t* view,
                                             size_t* out_len);
```

---

### foxglove_msgs — Buffer-backed

#### CompressedVideo

```c
ros_compressed_video_t* ros_compressed_video_from_cdr(const uint8_t* data,
                                                       size_t len);
void ros_compressed_video_free(ros_compressed_video_t* view);

int32_t     ros_compressed_video_get_stamp_sec(const ros_compressed_video_t* view);
uint32_t    ros_compressed_video_get_stamp_nanosec(const ros_compressed_video_t* view);
const char* ros_compressed_video_get_frame_id(const ros_compressed_video_t* view);
const uint8_t* ros_compressed_video_get_data(const ros_compressed_video_t* view,
                                              size_t* out_len);
const char* ros_compressed_video_get_format(const ros_compressed_video_t* view);

const uint8_t* ros_compressed_video_as_cdr(const ros_compressed_video_t* view,
                                            size_t* out_len);

int ros_compressed_video_encode(uint8_t** out_bytes, size_t* out_len,
                                int32_t stamp_sec, uint32_t stamp_nanosec,
                                const char* frame_id,
                                const uint8_t* data, size_t data_len,
                                const char* format);
```

---

### edgefirst_msgs — Buffer-backed

#### Mask

```c
ros_mask_t* ros_mask_from_cdr(const uint8_t* data, size_t len);
void        ros_mask_free(ros_mask_t* view);

uint32_t    ros_mask_get_height(const ros_mask_t* view);
uint32_t    ros_mask_get_width(const ros_mask_t* view);
uint32_t    ros_mask_get_length(const ros_mask_t* view);
const char* ros_mask_get_encoding(const ros_mask_t* view);
const uint8_t* ros_mask_get_data(const ros_mask_t* view, size_t* out_len);
bool        ros_mask_get_boxed(const ros_mask_t* view);
```

> **Note:** `ros_mask_as_cdr` has been removed. Forwarding an embedded child mask
> as a standalone CDR would require re-encoding, which violates the zero-copy
> contract. Child masks accessed via `ros_model_get_mask` are embedded in the
> parent buffer and do not have an independent CDR form.

```c
int ros_mask_encode(uint8_t** out_bytes, size_t* out_len,
                    uint32_t height, uint32_t width, uint32_t length,
                    const char* encoding,
                    const uint8_t* data, size_t data_len,
                    bool boxed);
```

#### DmaBuffer

```c
ros_dmabuffer_t* ros_dmabuffer_from_cdr(const uint8_t* data, size_t len);
void             ros_dmabuffer_free(ros_dmabuffer_t* view);

int32_t     ros_dmabuffer_get_stamp_sec(const ros_dmabuffer_t* view);
uint32_t    ros_dmabuffer_get_stamp_nanosec(const ros_dmabuffer_t* view);
const char* ros_dmabuffer_get_frame_id(const ros_dmabuffer_t* view);
uint32_t    ros_dmabuffer_get_pid(const ros_dmabuffer_t* view);
int32_t     ros_dmabuffer_get_fd(const ros_dmabuffer_t* view);
uint32_t    ros_dmabuffer_get_width(const ros_dmabuffer_t* view);
uint32_t    ros_dmabuffer_get_height(const ros_dmabuffer_t* view);
uint32_t    ros_dmabuffer_get_stride(const ros_dmabuffer_t* view);
uint32_t    ros_dmabuffer_get_fourcc(const ros_dmabuffer_t* view);
uint32_t    ros_dmabuffer_get_length(const ros_dmabuffer_t* view);

const uint8_t* ros_dmabuffer_as_cdr(const ros_dmabuffer_t* view, size_t* out_len);

int ros_dmabuffer_encode(uint8_t** out_bytes, size_t* out_len,
                         int32_t stamp_sec, uint32_t stamp_nanosec,
                         const char* frame_id,
                         uint32_t pid, int32_t fd,
                         uint32_t width, uint32_t height,
                         uint32_t stride, uint32_t fourcc, uint32_t length);
```

#### RadarCube

```c
ros_radar_cube_t* ros_radar_cube_from_cdr(const uint8_t* data, size_t len);
void              ros_radar_cube_free(ros_radar_cube_t* view);

int32_t     ros_radar_cube_get_stamp_sec(const ros_radar_cube_t* view);
uint32_t    ros_radar_cube_get_stamp_nanosec(const ros_radar_cube_t* view);
const char* ros_radar_cube_get_frame_id(const ros_radar_cube_t* view);
uint64_t    ros_radar_cube_get_timestamp(const ros_radar_cube_t* view);
const uint8_t* ros_radar_cube_get_layout(const ros_radar_cube_t* view,
                                          size_t* out_len);
const uint8_t* ros_radar_cube_get_cube_raw(const ros_radar_cube_t* view,
                                            size_t* out_len);
uint32_t    ros_radar_cube_get_cube_len(const ros_radar_cube_t* view);
bool        ros_radar_cube_get_is_complex(const ros_radar_cube_t* view);

const uint8_t* ros_radar_cube_as_cdr(const ros_radar_cube_t* view, size_t* out_len);
```

The cube data is raw bytes. For `i16` radar samples, use `memcpy` for
portable access:

```c
size_t raw_len;
const uint8_t* raw = ros_radar_cube_get_cube_raw(cube, &raw_len);
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
ros_radar_info_t* ros_radar_info_from_cdr(const uint8_t* data, size_t len);
void              ros_radar_info_free(ros_radar_info_t* view);

int32_t     ros_radar_info_get_stamp_sec(const ros_radar_info_t* view);
uint32_t    ros_radar_info_get_stamp_nanosec(const ros_radar_info_t* view);
const char* ros_radar_info_get_frame_id(const ros_radar_info_t* view);
const char* ros_radar_info_get_center_frequency(const ros_radar_info_t* view);
const char* ros_radar_info_get_frequency_sweep(const ros_radar_info_t* view);
const char* ros_radar_info_get_range_toggle(const ros_radar_info_t* view);
const char* ros_radar_info_get_detection_sensitivity(const ros_radar_info_t* view);
bool        ros_radar_info_get_cube(const ros_radar_info_t* view);

const uint8_t* ros_radar_info_as_cdr(const ros_radar_info_t* view, size_t* out_len);
```

#### Detect

```c
ros_detect_t* ros_detect_from_cdr(const uint8_t* data, size_t len);
void          ros_detect_free(ros_detect_t* view);

int32_t     ros_detect_get_stamp_sec(const ros_detect_t* view);
uint32_t    ros_detect_get_stamp_nanosec(const ros_detect_t* view);
const char* ros_detect_get_frame_id(const ros_detect_t* view);
uint32_t    ros_detect_get_boxes_len(const ros_detect_t* view);
// Returns a borrowed pointer; do NOT free. See Rule 5.
const ros_box_t* ros_detect_get_box(const ros_detect_t* view, uint32_t index);

const uint8_t* ros_detect_as_cdr(const ros_detect_t* view, size_t* out_len);
```

#### Model

```c
ros_model_t* ros_model_from_cdr(const uint8_t* data, size_t len);
void         ros_model_free(ros_model_t* view);

int32_t     ros_model_get_stamp_sec(const ros_model_t* view);
uint32_t    ros_model_get_stamp_nanosec(const ros_model_t* view);
const char* ros_model_get_frame_id(const ros_model_t* view);
uint32_t    ros_model_get_boxes_len(const ros_model_t* view);
uint32_t    ros_model_get_masks_len(const ros_model_t* view);
// Returns borrowed pointers; do NOT free. See Rule 5.
const ros_box_t*  ros_model_get_box(const ros_model_t* view, uint32_t index);
const ros_mask_t* ros_model_get_mask(const ros_model_t* view, uint32_t index);

const uint8_t* ros_model_as_cdr(const ros_model_t* view, size_t* out_len);
```

#### ModelInfo

```c
ros_model_info_t* ros_model_info_from_cdr(const uint8_t* data, size_t len);
void              ros_model_info_free(ros_model_info_t* view);

int32_t     ros_model_info_get_stamp_sec(const ros_model_info_t* view);
uint32_t    ros_model_info_get_stamp_nanosec(const ros_model_info_t* view);
const char* ros_model_info_get_frame_id(const ros_model_info_t* view);
const char* ros_model_info_get_model_type(const ros_model_info_t* view);
const char* ros_model_info_get_model_format(const ros_model_info_t* view);
const char* ros_model_info_get_model_name(const ros_model_info_t* view);
uint8_t     ros_model_info_get_input_type(const ros_model_info_t* view);
uint8_t     ros_model_info_get_output_type(const ros_model_info_t* view);

const uint8_t* ros_model_info_as_cdr(const ros_model_info_t* view, size_t* out_len);
```

#### Track

```c
ros_track_t* ros_track_from_cdr(const uint8_t* data, size_t len);
void         ros_track_free(ros_track_t* view);

const char* ros_track_get_id(const ros_track_t* view);
int32_t     ros_track_get_lifetime(const ros_track_t* view);

const uint8_t* ros_track_as_cdr(const ros_track_t* view, size_t* out_len);
```

#### DetectBox

```c
ros_box_t* ros_box_from_cdr(const uint8_t* data, size_t len);
void       ros_box_free(ros_box_t* view);

float       ros_box_get_center_x(const ros_box_t* view);
float       ros_box_get_center_y(const ros_box_t* view);
float       ros_box_get_width(const ros_box_t* view);
float       ros_box_get_height(const ros_box_t* view);
const char* ros_box_get_label(const ros_box_t* view);
float       ros_box_get_score(const ros_box_t* view);
float       ros_box_get_distance(const ros_box_t* view);
float       ros_box_get_speed(const ros_box_t* view);
const char* ros_box_get_track_id(const ros_box_t* view);
int32_t     ros_box_get_track_lifetime(const ros_box_t* view);
int32_t     ros_box_get_track_created_sec(const ros_box_t* view);
uint32_t    ros_box_get_track_created_nanosec(const ros_box_t* view);
```

> **Note:** `ros_box_as_cdr` has been removed. Forwarding an embedded child box
> as a standalone CDR would require re-encoding, which violates the zero-copy
> contract. Child boxes accessed via `ros_detect_get_box`/`ros_model_get_box`
> are embedded in the parent buffer and do not have an independent CDR form.

#### LocalTime

```c
ros_local_time_t* ros_local_time_from_cdr(const uint8_t* data, size_t len);
void              ros_local_time_free(ros_local_time_t* view);

int32_t     ros_local_time_get_stamp_sec(const ros_local_time_t* view);
uint32_t    ros_local_time_get_stamp_nanosec(const ros_local_time_t* view);
const char* ros_local_time_get_frame_id(const ros_local_time_t* view);
int16_t     ros_local_time_get_timezone(const ros_local_time_t* view);

const uint8_t* ros_local_time_as_cdr(const ros_local_time_t* view, size_t* out_len);
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
    if (ros_time_encode(buf, sizeof(buf), &written, 1234567890, 123456789) != 0) {
        perror("encode");
        return 1;
    }
    printf("Encoded Time: %zu bytes\n", written);

    // Decode
    int32_t sec;
    uint32_t nanosec;
    if (ros_time_decode(buf, written, &sec, &nanosec) != 0) {
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
ros_pose_encode(NULL, 0, &needed, 0, 0, 0, 0, 0, 0, 0);
uint8_t* buf = malloc(needed);
ros_pose_encode(buf, needed, &needed, 1.0, 2.0, 0.5, 0.0, 0.0, 0.707, 0.707);
// ... use buf ...
free(buf);
```

### Buffer-backed: Decode an Image from CDR

```c
#include <edgefirst/schemas.h>
#include <stdio.h>

void process_image(const uint8_t* cdr_data, size_t cdr_len) {
    ros_image_t* img = ros_image_from_cdr(cdr_data, cdr_len);
    if (!img) {
        perror("ros_image_from_cdr");
        return;
    }

    printf("Image: %ux%u %s\n",
           ros_image_get_width(img),
           ros_image_get_height(img),
           ros_image_get_encoding(img));  // borrowed pointer

    size_t data_len;
    const uint8_t* pixels = ros_image_get_data(img, &data_len);
    printf("Pixel data: %zu bytes at %p\n", data_len, (const void*)pixels);
    // pixels is borrowed — valid until ros_image_free(img)

    ros_image_free(img);
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

    if (ros_header_encode(&bytes, &len, 42, 0, "camera_frame") != 0) {
        perror("ros_header_encode");
        return 1;
    }
    printf("Encoded Header: %zu CDR bytes\n", len);

    // Verify round-trip
    ros_header_t* hdr = ros_header_from_cdr(bytes, len);
    printf("frame_id = %s\n", ros_header_get_frame_id(hdr));

    ros_header_free(hdr);
    ros_bytes_free(bytes, len);  // NOT free(bytes)
    return 0;
}
```

### Buffer-backed: Forward a Message

A common pattern is receiving CDR bytes from Zenoh, inspecting metadata,
and forwarding the raw bytes unchanged:

```c
void on_message(const uint8_t* cdr_data, size_t cdr_len) {
    ros_image_t* img = ros_image_from_cdr(cdr_data, cdr_len);
    if (!img) return;

    // Inspect metadata
    int32_t sec = ros_image_get_stamp_sec(img);
    const char* frame = ros_image_get_frame_id(img);
    printf("[%d] %s: %ux%u\n", sec, frame,
           ros_image_get_width(img), ros_image_get_height(img));

    // Forward the original CDR bytes (no re-serialization needed)
    size_t fwd_len;
    const uint8_t* fwd = ros_image_as_cdr(img, &fwd_len);
    // zenoh_publish(topic, fwd, fwd_len);

    ros_image_free(img);
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
    ros_image_t* view_;
public:
    explicit ImageView(const uint8_t* cdr, size_t len)
        : view_(ros_image_from_cdr(cdr, len))
    {
        if (!view_) throw std::runtime_error("ros_image_from_cdr failed");
    }
    ~ImageView() { ros_image_free(view_); }

    // Non-copyable
    ImageView(const ImageView&) = delete;
    ImageView& operator=(const ImageView&) = delete;

    // Movable
    ImageView(ImageView&& o) noexcept : view_(o.view_) { o.view_ = nullptr; }
    ImageView& operator=(ImageView&& o) noexcept {
        if (this != &o) { ros_image_free(view_); view_ = o.view_; o.view_ = nullptr; }
        return *this;
    }

    uint32_t width() const  { return ros_image_get_width(view_); }
    uint32_t height() const { return ros_image_get_height(view_); }
    const char* encoding() const { return ros_image_get_encoding(view_); }

    std::pair<const uint8_t*, size_t> data() const {
        size_t len;
        auto* p = ros_image_get_data(view_, &len);
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

    ros_image_t* img = ros_image_from_cdr(data, len);
    if (!img) return;

    printf("[%d.%09u] %s: %ux%u %s\n",
           ros_image_get_stamp_sec(img),
           ros_image_get_stamp_nanosec(img),
           ros_image_get_frame_id(img),
           ros_image_get_width(img),
           ros_image_get_height(img),
           ros_image_get_encoding(img));

    // Forward the original CDR bytes (zero re-serialization cost)
    size_t cdr_len;
    const uint8_t* cdr = ros_image_as_cdr(img, &cdr_len);
    // z_publisher_put(pub, cdr, cdr_len, NULL);

    ros_image_free(img);
}
```

Publishing a constructed message:

```c
void publish_header(z_loaned_publisher_t* pub) {
    uint8_t* bytes = NULL;
    size_t len = 0;

    if (ros_header_encode(&bytes, &len, 42, 0, "camera_frame") != 0)
        return;

    z_publisher_put(pub, bytes, len, NULL);
    ros_bytes_free(bytes, len);  // NOT free(bytes)
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
nm -D target/release/libedgefirst_schemas.so | grep ros_header
# If empty, rebuild: cargo build --release
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
   on handles, and `ros_bytes_free` on encode output.
3. **Wrong free function** — Using `free()` on encode output instead of
   `ros_bytes_free(bytes, len)`. The memory is allocated by Rust's allocator.
4. **NULL handle** — Getters return 0 or `NULL` for NULL handles rather than
   crashing, but always check `from_cdr` return values.

## C++ Wrapper

A header-only C++17 wrapper is available at
`include/edgefirst/schemas.hpp`. It provides RAII-managed view and owning
types for every C handle type — `ImageView`/`Image`, `HeaderView`/`Header`,
`DetectView`, `ModelView`, `MaskView`/`Mask`, and so on — together with
`expected<T, Error>` error handling (no exceptions) and range-based iteration
for array children:

```cpp
namespace ef = edgefirst::schemas;

// Decode a Detect message and iterate its boxes
auto det = ef::DetectView::from_cdr(payload);
for (auto box : det->boxes()) {
    std::cout << "label=" << box.label() << " score=" << box.score() << "\n";
}
```

The range adaptors (`boxes()`, `masks()`) use the same indexed accessor
functions documented in Rule 5 of the [Memory Management](#memory-management)
section above (`ros_detect_get_box`, `ros_model_get_box`,
`ros_model_get_mask`). Child views returned from the range are parent-borrowed
and must not be freed independently — exactly the same lifetime rule that
makes zero-copy iteration possible.

The C++ wrapper links against the same `libedgefirst_schemas` shared library
as the C API — there is no separate build target or additional library. Include
the C++ header and link with `-ledgefirst_schemas` exactly as you would for C.

See [README.md § C++ Usage](README.md#c-usage) for the full introduction,
installation notes, and working examples. The per-type API surface (constructors,
field accessors, encode/decode signatures) is documented inline in
`include/edgefirst/schemas.hpp`.

## Building from Source

```bash
# Build the shared library
cargo build --release

# The library is at:
#   target/release/libedgefirst_schemas.so      (Linux)
#   target/release/libedgefirst_schemas.dylib   (macOS)
#
# The header is at:
#   include/edgefirst/schemas.h

# Run the C API test suite (requires libcriterion-dev)
sudo apt-get install -y libcriterion-dev
mkdir -p build
for src in tests/c/test_*.c; do
    name=$(basename $src .c)
    gcc -Wall -Wextra -Werror -std=c11 -I./include \
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
```

See [Zig cross-compilation](https://github.com/nickel-lang/topiary/wiki/Cross-compilation-with-Zig)
for setup details.
