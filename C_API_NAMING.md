# C API Naming Conventions

**Version:** 2.0  
**Last Updated:** January 16, 2026  
**Status:** Canonical specification

## Overview

EdgeFirst Schemas C API uses consistent prefixes based on message source:

| Category | Source | Type | Function | Constant |
|----------|--------|------|----------|----------|
| **ROS** | [ros2/common_interfaces](https://github.com/ros2/common_interfaces) | `Ros` | `ros_` | `ROS_` |
| **Foxglove** | [foxglove-sdk/schemas/ros2](https://github.com/foxglove/foxglove-sdk/tree/main/schemas/ros2) | `Foxglove` | `foxglove_` | `FOXGLOVE_` |
| **EdgeFirst** | ./edgefirst_msgs/msg | `EdgeFirst` | `edgefirst_` | `EDGEFIRST_` |

## Naming Patterns

### Types (Opaque Pointers)

```c
typedef struct RosTime RosTime;
typedef struct EdgeFirstDetect EdgeFirstDetect;
typedef struct FoxgloveImageAnnotations FoxgloveImageAnnotations;
```

### Functions

**Lifecycle:**
```c
RosTime* ros_time_new(void);
void ros_time_free(RosTime* time);
```

**Getters:**
```c
int32_t ros_time_get_sec(const RosTime* time);                    // primitive
const char* ros_header_get_frame_id(const RosHeader* header);     // string
const uint8_t* edgefirst_mask_get_data(const EdgeFirstMask* mask, size_t* len);  // array
RosHeader* edgefirst_detect_get_header(EdgeFirstDetect* detect);  // nested
```

**Setters:**
```c
void ros_time_set_sec(RosTime* time, int32_t sec);                          // primitive
int ros_header_set_frame_id(RosHeader* header, const char* frame_id);       // string (returns -1 on error)
int edgefirst_mask_set_data(EdgeFirstMask* mask, const uint8_t* data, size_t len);  // array
void ros_header_set_stamp(RosHeader* header, const RosTime* stamp);         // nested
```

**Serialization:**
```c
int ros_time_serialize(const RosTime* time, uint8_t** out_bytes, size_t* out_len);
RosTime* ros_time_deserialize(const uint8_t* bytes, size_t len);
void edgefirst_buffer_free(uint8_t* buffer);
```

### Constants

```c
#define ROS_POINTFIELD_FLOAT32 7
#define ROS_NAVSATSTATUS_SERVICE_GPS 1
#define EDGEFIRST_MASK_ENCODING_ZSTD "zstd"
```

## Snake Case Conversion

| Type | Function Prefix |
|------|----------------|
| `RosPointCloud2` | `ros_point_cloud2_` |
| `RosNavSatFix` | `ros_nav_sat_fix_` |
| `EdgeFirstDetectTrack` | `edgefirst_detect_track_` |
| `EdgeFirstDetectBox2D` | `edgefirst_detect_box2d_` |
| `EdgeFirstDmaBuf` | `edgefirst_dma_buf_` |

**Rules:**
- Acronyms as single word: `DmaBuf` → `dma_buf`
- Numbers with preceding word: `Box2D` → `box2d`, `Cloud2` → `cloud2`

## Error Handling

**Errno Convention:**
- Functions returning `int`: 0 = success, -1 = error (sets errno)
- Functions returning pointers: non-NULL = success, NULL = error (sets errno)
- Getters returning primitives: return 0/empty on NULL (no errno)

**Errno Values:**
- `EINVAL` - NULL pointer, zero length, invalid UTF-8
- `ENOMEM` - Memory allocation failure
- `EBADMSG` - CDR deserialization failure

## Memory Management

1. `_new()` / `_deserialize()`: Caller owns, must call `_free()`
2. `_serialize()`: Caller owns buffer, must call `edgefirst_buffer_free()`
3. Getters: Borrowed pointer, do not free
4. Setters: Data copied, caller retains ownership

## Examples

### ROS Message (builtin_interfaces/Time)
```c
RosTime* time = ros_time_new();
ros_time_set_sec(time, 1234567890);
ros_time_set_nanosec(time, 123456789);

uint8_t* buffer = NULL;
size_t len = 0;
if (ros_time_serialize(time, &buffer, &len) == 0) {
    RosTime* decoded = ros_time_deserialize(buffer, len);
    // Use decoded...
    ros_time_free(decoded);
    edgefirst_buffer_free(buffer);
}
ros_time_free(time);
```

### EdgeFirst Message (edgefirst_msgs/Detect)
```c
EdgeFirstDetect* detect = edgefirst_detect_new();
edgefirst_detect_set_label(detect, "person");
edgefirst_detect_set_score(detect, 0.95);

RosHeader* header = edgefirst_detect_get_header(detect);
ros_header_set_frame_id(header, "camera_frame");

uint8_t* buffer = NULL;
size_t len = 0;
if (edgefirst_detect_serialize(detect, &buffer, &len) == 0) {
    // Use buffer...
    edgefirst_buffer_free(buffer);
}
edgefirst_detect_free(detect);
```

---

See also: [C_API_GUIDE.md](./C_API_GUIDE.md) for complete usage documentation.
