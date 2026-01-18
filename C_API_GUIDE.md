# EdgeFirst Schemas C API Guide

## Overview

The EdgeFirst Schemas C API provides C-compatible bindings to the Rust schema library with CDR (Common Data Representation) serialization support. This enables C and C++ applications to work with EdgeFirst Perception middleware messages.

## Features

- **Opaque Handle Pattern**: All types are opaque pointers for ABI stability
- **Memory Management**: Clear ownership rules with `_new()` and `_free()` functions
- **CDR Serialization**: Built-in serialize/deserialize for all types
- **Type Safety**: Comprehensive getter/setter functions for all fields
- **Zero Dependencies**: Only requires standard C library (no external C dependencies)

## Building

### Prerequisites

- Rust toolchain (1.70+)
- C compiler (GCC, Clang, or MSVC)
- cargo (comes with Rust)

### Build the Library

```bash
# Build debug library
cargo build --lib

# Build release library (optimized)
cargo build --lib --release

# Cross-compile for Linux (manylinux2014)
cargo zigbuild --target x86_64-unknown-linux-gnu --release
```

The library will be generated in `target/release/`:
- **Linux**: `libedgefirst_schemas.so`
- **macOS**: `libedgefirst_schemas.dylib`
- **Windows**: `edgefirst_schemas.dll`

Static library (`libedgefirst_schemas.a` / `.lib`) is also available.

## Installation

### System-wide Installation (Linux/macOS)

```bash
# Install header
sudo cp include/edgefirst/schemas.h /usr/local/include/edgefirst/

# Install library
sudo cp target/release/libedgefirst_schemas.{so,dylib} /usr/local/lib/

# Update library cache (Linux only)
sudo ldconfig
```

### Project-local Installation

Copy the header and library to your project:

```bash
my_project/
├── include/
│   └── edgefirst/
│       └── schemas.h
├── lib/
│   └── libedgefirst_schemas.{so,dylib,dll}
└── src/
    └── main.c
```

## Usage

### Compiling Your Application

```bash
# GCC/Clang
gcc -I/path/to/include -o myapp myapp.c -L/path/to/lib -ledgefirst_schemas

# With pkg-config (if installed system-wide)
gcc $(pkg-config --cflags edgefirst-schemas) -o myapp myapp.c \
    $(pkg-config --libs edgefirst-schemas)
```

### Running Your Application

Ensure the library is in your library path:

```bash
# Linux
export LD_LIBRARY_PATH=/path/to/lib:$LD_LIBRARY_PATH
./myapp

# macOS
export DYLD_LIBRARY_PATH=/path/to/lib:$DYLD_LIBRARY_PATH
./myapp

# Windows
set PATH=C:\path\to\lib;%PATH%
myapp.exe
```

## Memory Management Rules

### Ownership

1. **Message Handles**: Caller owns all handles from `_new()` functions
   ```c
   RosHeader* header = ros_header_new();
   // ... use header ...
   ros_header_free(header);  // Must free!
   ```

2. **Serialized Bytes (Khronos Pattern)**: Caller provides the buffer
   ```c
   // Query required size
   size_t required = 0;
   ros_header_serialize(header, NULL, 0, &required);

   // Allocate and serialize
   uint8_t* buffer = malloc(required);
   ros_header_serialize(header, buffer, required, NULL);

   // ... use buffer ...
   free(buffer);  // Caller owns buffer
   ```

   **Reusable buffer pattern (zero allocations in hot path):**
   ```c
   static uint8_t* buf = NULL;
   static size_t buf_cap = 0;

   size_t size = 0;
   if (ros_header_serialize(header, buf, buf_cap, &size) == -1 && errno == ENOBUFS) {
       buf = realloc(buf, size);
       buf_cap = size;
       ros_header_serialize(header, buf, buf_cap, NULL);
   }
   // buf contains serialized data, caller owns buf
   ```

3. **String Getters**: Caller owns returned strings
   ```c
   char* frame_id = ros_header_get_frame_id(header);
   printf("Frame: %s\n", frame_id);
   free(frame_id);  // Must free!
   ```

### Borrowed References

1. **Nested Object Getters**: Borrowed pointer, valid during parent lifetime
   ```c
   RosHeader* header = ros_header_new();
   const RosTime* stamp = ros_header_get_stamp(header);
   // stamp is valid while header is alive
   ros_header_free(header);
   // stamp is now invalid! Don't use it!
   ```

2. **Array Getters**: Borrowed pointer with output length
   ```c
   size_t len = 0;
   const uint8_t* data = ros_image_get_data(image, &len);
   // data is valid while image is alive
   // Do not free data!
   ```

### Mutable Access

Use `_get_<field>_mut()` for modifying nested objects:

```c
RosHeader* header = ros_header_new();
RosTime* stamp = ros_header_get_stamp_mut(header);
ros_time_set_sec(stamp, 12345);
ros_time_set_nanosec(stamp, 67890);
// stamp modifications affect header
```

## Common Patterns

### Creating and Serializing a Message

```c
#include <edgefirst/schemas.h>
#include <errno.h>

// Create message
RosHeader* header = ros_header_new();
ros_header_set_frame_id(header, "camera");

RosTime* stamp = ros_header_get_stamp_mut(header);
ros_time_set_sec(stamp, 1234567890);
ros_time_set_nanosec(stamp, 123456789);

// Query required size (Khronos pattern)
size_t required = 0;
if (ros_header_serialize(header, NULL, 0, &required) == -1) {
    fprintf(stderr, "Size query failed: %s\n", strerror(errno));
    ros_header_free(header);
    return -1;
}

// Allocate buffer and serialize
uint8_t* buffer = malloc(required);
if (buffer == NULL) {
    fprintf(stderr, "Memory allocation failed\n");
    ros_header_free(header);
    return -1;
}

if (ros_header_serialize(header, buffer, required, NULL) == -1) {
    fprintf(stderr, "Serialization failed: %s\n", strerror(errno));
    free(buffer);
    ros_header_free(header);
    return -1;
}

// ... send buffer (required bytes) over network ...

// Cleanup
free(buffer);
ros_header_free(header);
```

### Deserializing and Reading a Message

```c
// Receive bytes from network
const uint8_t* received_bytes = ...;
size_t received_len = ...;

// Deserialize
RosHeader* header = ros_header_deserialize(received_bytes, received_len);
if (header == NULL) {
    fprintf(stderr, "Deserialization failed\n");
    return -1;
}

// Read fields
const RosTime* stamp = ros_header_get_stamp(header);
int32_t sec = ros_time_get_sec(stamp);
uint32_t nanosec = ros_time_get_nanosec(stamp);

char* frame_id = ros_header_get_frame_id(header);
printf("Timestamp: %d.%09u, Frame: %s\n", sec, nanosec, frame_id);

// Cleanup
free(frame_id);
ros_header_free(header);
```

### Working with Binary Data

```c
// Create image with data
RosImage* image = ros_image_new();
ros_image_set_width(image, 640);
ros_image_set_height(image, 480);
ros_image_set_encoding(image, "rgb8");

// Set pixel data
uint8_t* pixels = malloc(640 * 480 * 3);
// ... fill pixels ...
ros_image_set_data(image, pixels, 640 * 480 * 3);
free(pixels);  // Safe to free after set_data

// Read pixel data
size_t data_len = 0;
const uint8_t* data = ros_image_get_data(image, &data_len);
// Process data...

ros_image_free(image);
```

## API Reference

### Error Handling

All functions use standard Unix conventions:
- Functions returning `int`: return `0` on success, `-1` on error with `errno` set
- Functions returning pointers: return `NULL` on error with `errno` set

**errno values used by this library:**

| errno | Value | Meaning |
|-------|-------|---------|
| `EINVAL` | 22 | NULL pointer or invalid argument |
| `ENOMEM` | 12 | Memory allocation failed |
| `ENOBUFS` | 105 | Buffer too small (size still written if size pointer provided) |
| `EBADMSG` | 74 | CDR serialization/deserialization failed |
| `EILSEQ` | 84 | Invalid UTF-8 in string field |

### Common Functions

All types follow this pattern:

```c
// Lifecycle
EdgeFirstType* edgefirst_type_new(void);
void edgefirst_type_free(EdgeFirstType* obj);

// Serialization (Khronos buffer pattern)
/**
 * @brief Serializes an object to CDR format.
 *
 * @param obj       Object to serialize (must not be NULL)
 * @param buffer    Output buffer for CDR bytes (may be NULL to query size)
 * @param capacity  Size of buffer in bytes (ignored if buffer is NULL)
 * @param size      If non-NULL, receives the number of bytes written/required
 *
 * @return 0 on success, -1 on error with errno set:
 *         - EINVAL: obj is NULL
 *         - ENOBUFS: buffer too small (size still written if provided)
 *         - EBADMSG: serialization encoding error
 */
int edgefirst_type_serialize(
    const EdgeFirstType* obj,
    uint8_t* buffer,
    size_t capacity,
    size_t* size
);

/**
 * @brief Deserializes an object from CDR bytes.
 *
 * @param bytes  CDR byte array (must not be NULL)
 * @param len    Length of byte array
 *
 * @return Newly allocated object on success (caller must free with _free()),
 *         NULL on error with errno set:
 *         - EINVAL: bytes is NULL
 *         - EBADMSG: deserialization decoding error
 *         - ENOMEM: memory allocation failed
 */
EdgeFirstType* edgefirst_type_deserialize(const uint8_t* bytes, size_t len);

// Field access (primitive types)
PrimitiveType edgefirst_type_get_field(const EdgeFirstType* obj);
int edgefirst_type_set_field(EdgeFirstType* obj, PrimitiveType value);

// Field access (strings) - caller must free returned string
char* edgefirst_type_get_string_field(const EdgeFirstType* obj);
int edgefirst_type_set_string_field(EdgeFirstType* obj, const char* value);

// Field access (nested objects) - borrowed pointer
const EdgeFirstNestedType* edgefirst_type_get_nested(const EdgeFirstType* obj);
EdgeFirstNestedType* edgefirst_type_get_nested_mut(EdgeFirstType* obj);

// Field access (arrays) - borrowed pointer with length
const ElemType* edgefirst_type_get_array(const EdgeFirstType* obj, size_t* out_len);
int edgefirst_type_set_array(EdgeFirstType* obj, const ElemType* data, size_t len);
```

### Available Types

#### builtin_interfaces
- `RosTime` - Timestamp (seconds + nanoseconds)
- `RosDuration` - Time duration

#### std_msgs
- `RosHeader` - Standard message header
- `RosColorRGBA` - RGBA color

#### geometry_msgs
- `RosVector3` - 3D vector
- `RosPoint` - 3D point
- `EFPoint32` - 3D point (float32)
- `RosQuaternion` - Rotation quaternion
- `EFPose` - Position + orientation
- `EFTransform` - Translation + rotation
- `EFTwist` - Linear + angular velocity
- `EFAccel` - Linear + angular acceleration

#### sensor_msgs
- `RosImage` - Raw image data
- `EFCompressedImage` - Compressed image
- `EFCameraInfo` - Camera calibration
- `EFIMU` - Inertial measurement unit
- `EFNavSatFix` - GPS/GNSS position
- `EFPointCloud2` - 3D point cloud

#### edgefirst_msgs
- `EdgeFirstDmaBuf` - DMA buffer (zero-copy camera)
- `EFRadarCube` - Radar FFT cube
- `EFRadarInfo` - Radar configuration
- `EFDetect` - Object detection results
- `EFMask` - Segmentation mask
- `EFModel` - Model inference results
- `EFModelInfo` - Model metadata
- `EFLocalTime` - Local time synchronization

#### foxglove_msgs
- `EFFoxgloveCompressedVideo` - H.264/H.265 video
- `EFFoxgloveImageAnnotations` - Image overlays

See `include/edgefirst/schemas.h` for complete API documentation.

## Constants

All message-related constants are defined with `EF_` prefix:

```c
// Radar cube dimensions
#define EF_RADAR_CUBE_DIMENSION_RANGE       1
#define EF_RADAR_CUBE_DIMENSION_DOPPLER     2
#define EF_RADAR_CUBE_DIMENSION_AZIMUTH     3

// Point field data types
#define EF_POINT_FIELD_FLOAT32  7
#define EF_POINT_FIELD_FLOAT64  8

// GPS service flags
#define EF_NAV_SAT_STATUS_SERVICE_GPS       1
#define EF_NAV_SAT_STATUS_SERVICE_GLONASS   2
```

## Error Handling

Always check return codes and NULL pointers. Functions set `errno` on error:

```c
#include <errno.h>
#include <string.h>

// Check int-returning functions
if (ros_header_set_frame_id(header, "test") == -1) {
    fprintf(stderr, "Failed to set frame_id: %s\n", strerror(errno));
    // Handle error based on errno (EINVAL, EILSEQ, etc.)
}

// Check pointer-returning functions
RosHeader* header = ros_header_deserialize(bytes, len);
if (header == NULL) {
    fprintf(stderr, "Deserialization failed: %s\n", strerror(errno));
    // errno will be EINVAL, EBADMSG, or ENOMEM
}

// Check serialization with Khronos pattern
size_t size = 0;
if (ros_header_serialize(header, buffer, capacity, &size) == -1) {
    if (errno == ENOBUFS) {
        // Buffer too small, 'size' contains required capacity
        buffer = realloc(buffer, size);
        capacity = size;
        ros_header_serialize(header, buffer, capacity, NULL);
    } else {
        fprintf(stderr, "Serialization error: %s\n", strerror(errno));
    }
}
```

## Thread Safety

- **Read-only operations** (getters, serialize) are thread-safe for the same object
- **Write operations** (setters, deserialize, new, free) require external synchronization
- **Different objects** can be accessed concurrently without synchronization

## Performance Considerations

1. **Serialization**: O(n) where n is message size, typically < 1ms for typical messages
2. **Memory**: Minimal overhead, messages are plain structs with vectors
3. **Zero-copy**: Arrays and nested objects return borrowed pointers when possible
4. **CDR Format**: Efficient binary encoding matching ROS2 DDS wire protocol

## Examples

See `examples/c/` directory for complete working examples:
- `example.c` - Basic usage of all common types
- Build and run: `cd examples/c && make run`

## Integration with Other Languages

### C++

```cpp
extern "C" {
#include <edgefirst/schemas.h>
}

// RAII wrapper
class Header {
    RosHeader* ptr_;
public:
    Header() : ptr_(ros_header_new()) {}
    ~Header() { ros_header_free(ptr_); }
    RosHeader* get() { return ptr_; }
};
```

### Zenoh Integration

```c
#include <zenoh-c.h>
#include <edgefirst/schemas.h>
#include <errno.h>

// Publisher with reusable buffer (zero allocations after warmup)
static uint8_t* tx_buf = NULL;
static size_t tx_cap = 0;

void publish_header(z_session_t session) {
    RosHeader* header = ros_header_new();
    ros_header_set_frame_id(header, "camera");

    // Serialize with Khronos pattern
    size_t size = 0;
    if (ros_header_serialize(header, tx_buf, tx_cap, &size) == -1 && errno == ENOBUFS) {
        tx_buf = realloc(tx_buf, size);
        tx_cap = size;
        ros_header_serialize(header, tx_buf, tx_cap, NULL);
    }

    z_put(session, "rt/camera/info", tx_buf, size, NULL);
    ros_header_free(header);
    // tx_buf is reused across calls
}

// Subscriber
void on_sample(const z_sample_t* sample, void* arg) {
    const uint8_t* payload = z_sample_payload(sample);
    size_t len = z_sample_payload_len(sample);

    RosHeader* header = ros_header_deserialize(payload, len);
    if (header != NULL) {
        char* frame_id = ros_header_get_frame_id(header);
        printf("Received header: %s\n", frame_id);
        free(frame_id);
        ros_header_free(header);
    } else {
        fprintf(stderr, "Deserialize error: %s\n", strerror(errno));
    }
}
```

## Troubleshooting

### Library Not Found

```bash
# Linux: Check library path
ldd myapp
export LD_LIBRARY_PATH=/path/to/lib:$LD_LIBRARY_PATH

# macOS: Check library path
otool -L myapp
export DYLD_LIBRARY_PATH=/path/to/lib:$DYLD_LIBRARY_PATH

# Install system-wide
sudo cp libedgefirst_schemas.{so,dylib} /usr/local/lib/
sudo ldconfig  # Linux only
```

### Compilation Errors

```bash
# Check header location
gcc -I/path/to/include -E -dM myapp.c | grep EDGEFIRST

# Verify library exists
ls -l /path/to/lib/libedgefirst_schemas.*

# Check for missing symbols
nm -D /path/to/lib/libedgefirst_schemas.so | grep ros_header_new
```

### Runtime Crashes

1. **Memory corruption**: Check that you're freeing strings from getters
2. **Use after free**: Ensure borrowed pointers aren't used after parent is freed
3. **NULL pointers**: Always check return values from `_new()` and `_deserialize()`
4. **Stack smashing**: Don't pass stack buffers to `_set_data()` functions

## License

Apache-2.0 - See LICENSE file for details.

Copyright © 2025 Au-Zone Technologies. All Rights Reserved.
