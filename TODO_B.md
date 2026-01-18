# TODO Phase B: C API Expansion

**Phase:** B (split into B.1 and B.2)
**Status:** Not Started
**Estimate:** 16-24 hours total
**Dependencies:** None (can start immediately)
**Blocks:** Phase C (C Coverage Instrumentation), Phase F (Cross-Language Validation)

---

## Phase Split

| Sub-phase | Focus | Estimate | Unblocks |
|-----------|-------|----------|----------|
| **B.1** | Priority 1 types (FoxgloveCompressedVideo, RadarCube) | 6-8 hrs | Phase C |
| **B.2** | Priority 2-4 types (geometry_msgs, sensor_msgs, etc.) | 10-16 hrs | Phase F |

**Rationale:** Splitting allows Phase C (Coverage Instrumentation) to begin earlier, reducing the critical path.

---

## Objective

Expand the C FFI bindings in `src/ffi.rs` to cover all message types, with priority on heavy messages used in production. Update the C header and tests accordingly.

---

## Architectural Requirements

### Khronos-style Serialization Pattern

All `_serialize()` functions MUST use the Khronos buffer pattern instead of returning Rust-allocated memory:

```c
/**
 * @brief Serializes a message to CDR format.
 *
 * @param obj       The object to serialize (must not be NULL)
 * @param buffer    Output buffer for CDR bytes (may be NULL to query size)
 * @param capacity  Size of buffer in bytes (ignored if buffer is NULL)
 * @param size      If non-NULL, receives the number of bytes written/required
 *
 * @return 0 on success, -1 on error with errno set:
 *         - EINVAL: obj is NULL
 *         - ENOBUFS: buffer too small (size still written if provided)
 *         - EBADMSG: serialization failed
 */
int edgefirst_type_serialize(
    const EdgeFirstType* obj,
    uint8_t* buffer,
    size_t capacity,
    size_t* size
);
```

**Rust Implementation Pattern:**
```rust
#[no_mangle]
pub extern "C" fn edgefirst_type_serialize(
    obj: *const EdgeFirstType,
    buffer: *mut u8,
    capacity: usize,
    size: *mut usize,
) -> c_int {
    if obj.is_null() {
        set_errno(EINVAL);
        return -1;
    }

    let obj = unsafe { &*obj };
    let bytes = match crate::serde_cdr::serialize(obj) {
        Ok(b) => b,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };

    // Write size if requested
    if !size.is_null() {
        unsafe { *size = bytes.len(); }
    }

    // If buffer is NULL, caller is querying size only
    if buffer.is_null() {
        return 0;
    }

    // Check capacity
    if capacity < bytes.len() {
        set_errno(ENOBUFS);
        return -1;
    }

    // Copy to caller's buffer
    unsafe {
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), buffer, bytes.len());
    }
    0
}
```

### Error Handling Policy

**CRITICAL:** No `assert!()` in FFI code. All error paths must:
1. Set `errno` appropriately
2. Return `-1` (for int-returning functions) or `NULL` (for pointer-returning functions)
3. Be documented in `schemas.h` doxygen comments

**Standard errno values:**
| errno | When to use |
|-------|-------------|
| `EINVAL` | NULL pointer or invalid argument |
| `ENOMEM` | Memory allocation failed |
| `ENOBUFS` | Buffer too small (size still written) |
| `EBADMSG` | CDR serialization/deserialization failed |
| `EILSEQ` | Invalid UTF-8 in string field |

---

## Current State

### FFI Coverage Analysis

| Category | Defined in Rust | Has C FFI | Coverage |
|----------|-----------------|-----------|----------|
| builtin_interfaces | 2 types | 2 types | 100% |
| std_msgs | 2 types | 2 types | 100% |
| geometry_msgs | 15 types | 4 types | 27% |
| sensor_msgs | 8 types | 4 types | 50% |
| edgefirst_msgs | 12 types | 6 types | 50% |
| foxglove_msgs | 7 types | 0 types | 0% |
| **Total** | **46 types** | **18 types** | **39%** |

### Existing FFI Functions (216 total)

Already implemented and working:
- `builtin_interfaces`: Time (8), Duration (8)
- `std_msgs`: Header (8), ColorRGBA (11)
- `geometry_msgs`: Vector3 (10), Point (10), Quaternion (12)
- `sensor_msgs`: Image (11), PointCloud2 (~30), PointField (~10), NavSatFix (~15), NavSatStatus (~8)
- `edgefirst_msgs`: DmaBuf (24), Detect (~25), DetectBox2D (~15), DetectTrack (~20), Mask (~25), ModelInfo (~10)

---

## Deliverables

### B.1 Priority 1: Heavy Messages (Critical)

These are **required** for production use in EdgeFirst samples.

#### B.1.1 FoxgloveCompressedVideo (~15 functions)

**Rust struct:**
```rust
pub struct FoxgloveCompressedVideo {
    pub header: std_msgs::Header,
    pub data: Vec<u8>,
    pub format: String,
}
```

**Required FFI functions:**
```rust
// Lifecycle
pub extern "C" fn foxglove_compressed_video_new() -> *mut FoxgloveCompressedVideo;
pub extern "C" fn foxglove_compressed_video_free(video: *mut FoxgloveCompressedVideo);

// Header access
pub extern "C" fn foxglove_compressed_video_get_header(video: *mut FoxgloveCompressedVideo) -> *mut Header;

// Data access (Vec<u8>)
pub extern "C" fn foxglove_compressed_video_get_data(video: *const FoxgloveCompressedVideo) -> *const u8;
pub extern "C" fn foxglove_compressed_video_get_data_len(video: *const FoxgloveCompressedVideo) -> usize;
pub extern "C" fn foxglove_compressed_video_set_data(video: *mut FoxgloveCompressedVideo, data: *const u8, len: usize);
pub extern "C" fn foxglove_compressed_video_reserve_data(video: *mut FoxgloveCompressedVideo, capacity: usize);

// Format access (String)
pub extern "C" fn foxglove_compressed_video_get_format(video: *const FoxgloveCompressedVideo) -> *mut c_char;
pub extern "C" fn foxglove_compressed_video_set_format(video: *mut FoxgloveCompressedVideo, format: *const c_char);

// Serialization (Khronos-style buffer pattern)
pub extern "C" fn foxglove_compressed_video_serialize(
    video: *const FoxgloveCompressedVideo,
    buffer: *mut u8,
    capacity: usize,
    size: *mut usize
) -> c_int;
pub extern "C" fn foxglove_compressed_video_deserialize(bytes: *const u8, len: usize) -> *mut FoxgloveCompressedVideo;
```

**C header additions:**
```c
typedef struct FoxgloveCompressedVideo FoxgloveCompressedVideo;

FoxgloveCompressedVideo* foxglove_compressed_video_new(void);
void foxglove_compressed_video_free(FoxgloveCompressedVideo* video);
RosHeader* foxglove_compressed_video_get_header(FoxgloveCompressedVideo* video);
const uint8_t* foxglove_compressed_video_get_data(const FoxgloveCompressedVideo* video);
size_t foxglove_compressed_video_get_data_len(const FoxgloveCompressedVideo* video);
int foxglove_compressed_video_set_data(FoxgloveCompressedVideo* video, const uint8_t* data, size_t len);
int foxglove_compressed_video_reserve_data(FoxgloveCompressedVideo* video, size_t capacity);
char* foxglove_compressed_video_get_format(const FoxgloveCompressedVideo* video);
int foxglove_compressed_video_set_format(FoxgloveCompressedVideo* video, const char* format);

/**
 * @brief Serializes video to CDR format.
 * @param video     Video to serialize (must not be NULL)
 * @param buffer    Output buffer (may be NULL to query size)
 * @param capacity  Buffer size in bytes
 * @param size      Receives bytes written/required (may be NULL)
 * @return 0 on success, -1 on error (errno: EINVAL, ENOBUFS, EBADMSG)
 */
int foxglove_compressed_video_serialize(
    const FoxgloveCompressedVideo* video,
    uint8_t* buffer,
    size_t capacity,
    size_t* size
);
FoxgloveCompressedVideo* foxglove_compressed_video_deserialize(const uint8_t* bytes, size_t len);
```

**Test file:** `tests/c/test_foxglove_msgs.c` (new file)

**Acceptance:**
- [ ] All 11 functions implemented in ffi.rs
- [ ] Header declarations added
- [ ] Test coverage for all functions
- [ ] Serialize/deserialize roundtrip test

---

#### B.1.2 RadarCube (~20 functions)

**Rust struct:**
```rust
pub struct RadarCube {
    pub header: std_msgs::Header,
    pub timestamp: u64,
    pub layout: Vec<u8>,
    pub shape: Vec<u16>,
    pub scales: Vec<f32>,
    pub cube: Vec<i16>,
    pub is_complex: bool,
}
```

**Required FFI functions:**
```rust
// Lifecycle
pub extern "C" fn edgefirst_radar_cube_new() -> *mut RadarCube;
pub extern "C" fn edgefirst_radar_cube_free(cube: *mut RadarCube);

// Header access
pub extern "C" fn edgefirst_radar_cube_get_header(cube: *mut RadarCube) -> *mut Header;

// Scalar fields
pub extern "C" fn edgefirst_radar_cube_get_timestamp(cube: *const RadarCube) -> u64;
pub extern "C" fn edgefirst_radar_cube_set_timestamp(cube: *mut RadarCube, timestamp: u64);
pub extern "C" fn edgefirst_radar_cube_get_is_complex(cube: *const RadarCube) -> bool;
pub extern "C" fn edgefirst_radar_cube_set_is_complex(cube: *mut RadarCube, is_complex: bool);

// Layout (Vec<u8>)
pub extern "C" fn edgefirst_radar_cube_get_layout(cube: *const RadarCube) -> *const u8;
pub extern "C" fn edgefirst_radar_cube_get_layout_len(cube: *const RadarCube) -> usize;
pub extern "C" fn edgefirst_radar_cube_set_layout(cube: *mut RadarCube, layout: *const u8, len: usize);

// Shape (Vec<u16>)
pub extern "C" fn edgefirst_radar_cube_get_shape(cube: *const RadarCube) -> *const u16;
pub extern "C" fn edgefirst_radar_cube_get_shape_len(cube: *const RadarCube) -> usize;
pub extern "C" fn edgefirst_radar_cube_set_shape(cube: *mut RadarCube, shape: *const u16, len: usize);

// Scales (Vec<f32>)
pub extern "C" fn edgefirst_radar_cube_get_scales(cube: *const RadarCube) -> *const f32;
pub extern "C" fn edgefirst_radar_cube_get_scales_len(cube: *const RadarCube) -> usize;
pub extern "C" fn edgefirst_radar_cube_set_scales(cube: *mut RadarCube, scales: *const f32, len: usize);

// Cube data (Vec<i16>)
pub extern "C" fn edgefirst_radar_cube_get_cube(cube: *const RadarCube) -> *const i16;
pub extern "C" fn edgefirst_radar_cube_get_cube_len(cube: *const RadarCube) -> usize;
pub extern "C" fn edgefirst_radar_cube_set_cube(cube: *mut RadarCube, data: *const i16, len: usize);
pub extern "C" fn edgefirst_radar_cube_reserve_cube(cube: *mut RadarCube, capacity: usize);

// Serialization (Khronos-style buffer pattern)
pub extern "C" fn edgefirst_radar_cube_serialize(
    cube: *const RadarCube,
    buffer: *mut u8,
    capacity: usize,
    size: *mut usize
) -> c_int;
pub extern "C" fn edgefirst_radar_cube_deserialize(bytes: *const u8, len: usize) -> *mut RadarCube;
```

**Constants:**
```rust
// radar_cube_dimension constants
pub const EDGEFIRST_RADAR_CUBE_DIM_UNDEFINED: u8 = 0;
pub const EDGEFIRST_RADAR_CUBE_DIM_RANGE: u8 = 1;
pub const EDGEFIRST_RADAR_CUBE_DIM_DOPPLER: u8 = 2;
pub const EDGEFIRST_RADAR_CUBE_DIM_AZIMUTH: u8 = 3;
pub const EDGEFIRST_RADAR_CUBE_DIM_ELEVATION: u8 = 4;
pub const EDGEFIRST_RADAR_CUBE_DIM_RXCHANNEL: u8 = 5;
pub const EDGEFIRST_RADAR_CUBE_DIM_SEQUENCE: u8 = 6;
```

**Acceptance:**
- [ ] All ~22 functions implemented in ffi.rs
- [ ] Dimension constants exposed
- [ ] Header declarations added
- [ ] Test coverage for all functions
- [ ] Large cube serialize/deserialize test

---

### B.2 Priority 2: Common Robotics Types

#### B.2.1 geometry_msgs Expansion (~60 functions)

**Missing types (11 of 15):**
- Pose, PoseStamped, Pose2D
- Transform, TransformStamped
- Twist, TwistStamped
- Accel, AccelStamped
- Wrench, WrenchStamped
- Inertia, InertiaStamped
- Point32, PointStamped

**Pattern for each type:**
```rust
// For Pose (composite type)
pub extern "C" fn ros_pose_new() -> *mut Pose;
pub extern "C" fn ros_pose_free(pose: *mut Pose);
pub extern "C" fn ros_pose_get_position(pose: *mut Pose) -> *mut Point;
pub extern "C" fn ros_pose_get_orientation(pose: *mut Pose) -> *mut Quaternion;
pub extern "C" fn ros_pose_serialize(pose: *const Pose, out_len: *mut usize) -> *mut u8;
pub extern "C" fn ros_pose_deserialize(bytes: *const u8, len: usize) -> *mut Pose;

// For PoseStamped (stamped variant)
pub extern "C" fn ros_pose_stamped_new() -> *mut PoseStamped;
pub extern "C" fn ros_pose_stamped_free(pose: *mut PoseStamped);
pub extern "C" fn ros_pose_stamped_get_header(pose: *mut PoseStamped) -> *mut Header;
pub extern "C" fn ros_pose_stamped_get_pose(pose: *mut PoseStamped) -> *mut Pose;
pub extern "C" fn ros_pose_stamped_serialize(...);
pub extern "C" fn ros_pose_stamped_deserialize(...);
```

**Functions per type:**
| Type | Functions | Total |
|------|-----------|-------|
| Pose | 6 | 6 |
| PoseStamped | 6 | 6 |
| Pose2D | 8 (x,y,theta getters/setters) | 8 |
| Transform | 6 | 6 |
| TransformStamped | 8 (+ child_frame_id) | 8 |
| Twist | 6 | 6 |
| TwistStamped | 6 | 6 |
| Accel | 6 | 6 |
| AccelStamped | 6 | 6 |
| Wrench | 6 | 6 |
| WrenchStamped | 6 | 6 |
| **Total** | | **~70** |

**Acceptance:**
- [ ] All geometry_msgs types have FFI
- [ ] C header updated
- [ ] Tests added to `test_geometry_msgs.c`

---

#### B.2.2 sensor_msgs Expansion (~40 functions)

**Missing types:**
- IMU
- CameraInfo
- CompressedImage
- RegionOfInterest

**IMU (most complex):**
```rust
pub struct IMU {
    pub header: std_msgs::Header,
    pub orientation: geometry_msgs::Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: geometry_msgs::Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: geometry_msgs::Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}
```

```rust
// IMU FFI
pub extern "C" fn ros_imu_new() -> *mut IMU;
pub extern "C" fn ros_imu_free(imu: *mut IMU);
pub extern "C" fn ros_imu_get_header(imu: *mut IMU) -> *mut Header;
pub extern "C" fn ros_imu_get_orientation(imu: *mut IMU) -> *mut Quaternion;
pub extern "C" fn ros_imu_get_angular_velocity(imu: *mut IMU) -> *mut Vector3;
pub extern "C" fn ros_imu_get_linear_acceleration(imu: *mut IMU) -> *mut Vector3;
pub extern "C" fn ros_imu_get_orientation_covariance(imu: *const IMU) -> *const f64;
pub extern "C" fn ros_imu_set_orientation_covariance(imu: *mut IMU, cov: *const f64);
// ... similar for other covariances
pub extern "C" fn ros_imu_serialize(...);
pub extern "C" fn ros_imu_deserialize(...);
```

**CameraInfo:**
```rust
pub extern "C" fn ros_camera_info_new() -> *mut CameraInfo;
pub extern "C" fn ros_camera_info_free(info: *mut CameraInfo);
pub extern "C" fn ros_camera_info_get_header(info: *mut CameraInfo) -> *mut Header;
pub extern "C" fn ros_camera_info_get_width(info: *const CameraInfo) -> u32;
pub extern "C" fn ros_camera_info_get_height(info: *const CameraInfo) -> u32;
pub extern "C" fn ros_camera_info_get_distortion_model(info: *const CameraInfo) -> *mut c_char;
pub extern "C" fn ros_camera_info_get_d(info: *const CameraInfo) -> *const f64;
pub extern "C" fn ros_camera_info_get_d_len(info: *const CameraInfo) -> usize;
pub extern "C" fn ros_camera_info_get_k(info: *const CameraInfo) -> *const f64; // [f64; 9]
pub extern "C" fn ros_camera_info_get_r(info: *const CameraInfo) -> *const f64; // [f64; 9]
pub extern "C" fn ros_camera_info_get_p(info: *const CameraInfo) -> *const f64; // [f64; 12]
pub extern "C" fn ros_camera_info_get_roi(info: *mut CameraInfo) -> *mut RegionOfInterest;
// ... setters, serialize, deserialize
```

**Acceptance:**
- [ ] IMU with all covariance matrices
- [ ] CameraInfo with intrinsics/distortion
- [ ] CompressedImage (similar to FoxgloveCompressedVideo)
- [ ] RegionOfInterest (helper type)

---

### B.3 Priority 3: EdgeFirst Custom Types

**Missing types:**
- Model
- RadarInfo
- LocalTime
- Date

**Model:**
```rust
pub extern "C" fn edgefirst_model_new() -> *mut Model;
pub extern "C" fn edgefirst_model_free(model: *mut Model);
pub extern "C" fn edgefirst_model_get_header(model: *mut Model) -> *mut Header;
pub extern "C" fn edgefirst_model_get_input_time(model: *mut Model) -> *mut Duration;
pub extern "C" fn edgefirst_model_get_model_time(model: *mut Model) -> *mut Duration;
pub extern "C" fn edgefirst_model_get_output_time(model: *mut Model) -> *mut Duration;
pub extern "C" fn edgefirst_model_get_decode_time(model: *mut Model) -> *mut Duration;
pub extern "C" fn edgefirst_model_get_boxes(model: *const Model) -> *const DetectBox2D;
pub extern "C" fn edgefirst_model_get_boxes_len(model: *const Model) -> usize;
// ... masks array access, serialize, deserialize
```

---

### B.4 Priority 4: Foxglove Visualization Types

**Types:**
- FoxgloveImageAnnotations
- FoxgloveCircleAnnotations
- FoxglovePointAnnotations
- FoxgloveTextAnnotations
- FoxglovePoint2
- FoxgloveColor

These are lower priority but needed for complete coverage.

---

## Implementation Guidelines

### FFI Pattern Template

Use this template for consistency:

```rust
// =============================================================================
// TypeName
// =============================================================================

/// Creates a new TypeName with default values.
/// Returns NULL on allocation failure (sets errno to ENOMEM).
#[no_mangle]
pub extern "C" fn prefix_type_name_new() -> *mut TypeName {
    match Box::try_new(TypeName::default()) {
        Ok(boxed) => Box::into_raw(boxed),
        Err(_) => {
            set_errno(ENOMEM);
            std::ptr::null_mut()
        }
    }
}

/// Frees a TypeName. Safe to call with NULL.
#[no_mangle]
pub extern "C" fn prefix_type_name_free(ptr: *mut TypeName) {
    if !ptr.is_null() {
        unsafe { drop(Box::from_raw(ptr)); }
    }
}

/// Gets a field value. Returns default if ptr is NULL (sets errno to EINVAL).
#[no_mangle]
pub extern "C" fn prefix_type_name_get_field(ptr: *const TypeName) -> FieldType {
    if ptr.is_null() {
        set_errno(EINVAL);
        return Default::default();
    }
    unsafe { (*ptr).field }
}

/// Sets a field value. Returns 0 on success, -1 on error.
#[no_mangle]
pub extern "C" fn prefix_type_name_set_field(ptr: *mut TypeName, value: FieldType) -> c_int {
    if ptr.is_null() {
        set_errno(EINVAL);
        return -1;
    }
    unsafe { (*ptr).field = value; }
    0
}

/// Serializes to CDR bytes using Khronos buffer pattern.
///
/// @param ptr      Object to serialize (must not be NULL)
/// @param buffer   Output buffer (may be NULL to query size)
/// @param capacity Buffer size in bytes
/// @param size     Receives bytes written/required (may be NULL)
/// @return 0 on success, -1 on error (errno: EINVAL, ENOBUFS, EBADMSG)
#[no_mangle]
pub extern "C" fn prefix_type_name_serialize(
    ptr: *const TypeName,
    buffer: *mut u8,
    capacity: usize,
    size: *mut usize,
) -> c_int {
    if ptr.is_null() {
        set_errno(EINVAL);
        return -1;
    }

    let obj = unsafe { &*ptr };
    let bytes = match crate::serde_cdr::serialize(obj) {
        Ok(b) => b,
        Err(_) => {
            set_errno(EBADMSG);
            return -1;
        }
    };

    // Write size if requested
    if !size.is_null() {
        unsafe { *size = bytes.len(); }
    }

    // If buffer is NULL, caller is querying size only
    if buffer.is_null() {
        return 0;
    }

    // Check capacity
    if capacity < bytes.len() {
        set_errno(ENOBUFS);
        return -1;
    }

    // Copy to caller's buffer
    unsafe {
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), buffer, bytes.len());
    }
    0
}

/// Deserializes from CDR bytes. Returns NULL on error (sets errno).
#[no_mangle]
pub extern "C" fn prefix_type_name_deserialize(
    bytes: *const u8,
    len: usize,
) -> *mut TypeName {
    if bytes.is_null() {
        set_errno(EINVAL);
        return std::ptr::null_mut();
    }
    let slice = unsafe { std::slice::from_raw_parts(bytes, len) };
    match crate::serde_cdr::deserialize::<TypeName>(slice) {
        Ok(val) => Box::into_raw(Box::new(val)),
        Err(_) => {
            set_errno(EBADMSG);
            std::ptr::null_mut()
        }
    }
}
```

### Naming Convention

Per [C_API_NAMING.md](./C_API_NAMING.md):

| Category | Function Prefix | Type Prefix |
|----------|-----------------|-------------|
| ROS types | `ros_` | `Ros` |
| Foxglove types | `foxglove_` | `Foxglove` |
| EdgeFirst types | `edgefirst_` | `EdgeFirst` |

---

## Test Requirements

### New Test Files

1. **`tests/c/test_foxglove_msgs.c`** (new)
   - FoxgloveCompressedVideo tests (~15 tests)
   - FoxgloveImageAnnotations tests (~10 tests)

2. **Update `tests/c/test_geometry_msgs.c`**
   - Add tests for Pose, Transform, Twist, etc. (~40 tests)

3. **Update `tests/c/test_sensor_msgs.c`**
   - Add tests for IMU, CameraInfo (~20 tests)

4. **Update `tests/c/test_edgefirst_msgs.c`**
   - Add tests for RadarCube, Model (~25 tests)

### Test Pattern

```c
Test(category, type_name_lifecycle) {
    TypeName* obj = prefix_type_name_new();
    cr_assert_not_null(obj, "Failed to create TypeName");
    prefix_type_name_free(obj);
}

Test(category, type_name_getters_setters) {
    TypeName* obj = prefix_type_name_new();
    cr_assert_eq(prefix_type_name_set_field(obj, test_value), 0);
    cr_assert_eq(prefix_type_name_get_field(obj), test_value);
    prefix_type_name_free(obj);
}

Test(category, type_name_serialize_query_size) {
    TypeName* obj = prefix_type_name_new();
    // Set test values...

    // Query required size (Khronos pattern)
    size_t required_size = 0;
    int result = prefix_type_name_serialize(obj, NULL, 0, &required_size);
    cr_assert_eq(result, 0, "Size query failed");
    cr_assert_gt(required_size, 0, "Size should be positive");

    prefix_type_name_free(obj);
}

Test(category, type_name_serialize_buffer_too_small) {
    TypeName* obj = prefix_type_name_new();
    // Set test values...

    uint8_t small_buffer[4];
    size_t size = 0;
    int result = prefix_type_name_serialize(obj, small_buffer, sizeof(small_buffer), &size);
    cr_assert_eq(result, -1, "Should fail with small buffer");
    cr_assert_eq(errno, ENOBUFS, "errno should be ENOBUFS");
    cr_assert_gt(size, sizeof(small_buffer), "Size should indicate required capacity");

    prefix_type_name_free(obj);
}

Test(category, type_name_serialize_roundtrip) {
    TypeName* original = prefix_type_name_new();
    // Set test values...

    // Query size
    size_t required_size = 0;
    cr_assert_eq(prefix_type_name_serialize(original, NULL, 0, &required_size), 0);

    // Allocate and serialize
    uint8_t* buffer = malloc(required_size);
    cr_assert_not_null(buffer);
    size_t written = 0;
    cr_assert_eq(prefix_type_name_serialize(original, buffer, required_size, &written), 0);
    cr_assert_eq(written, required_size);

    // Deserialize and verify
    TypeName* restored = prefix_type_name_deserialize(buffer, written);
    cr_assert_not_null(restored);

    // Verify all fields match
    cr_assert_eq(prefix_type_name_get_field(original),
                 prefix_type_name_get_field(restored));

    free(buffer);  // Caller owns buffer
    prefix_type_name_free(original);
    prefix_type_name_free(restored);
}

Test(category, type_name_null_handling) {
    // All functions should handle NULL gracefully
    cr_assert_eq(prefix_type_name_serialize(NULL, NULL, 0, NULL), -1);
    cr_assert_eq(errno, EINVAL);

    cr_assert_null(prefix_type_name_deserialize(NULL, 0));
    cr_assert_eq(errno, EINVAL);
}
```

---

## Validation Checklist

### Before Merging

- [ ] All new FFI functions compile: `cargo build --release`
- [ ] C header updated: `include/edgefirst/schemas.h`
- [ ] Header matches FFI exactly (function signatures)
- [ ] All tests pass: `cd tests/c && make test`
- [ ] No memory leaks (valgrind optional)
- [ ] Documentation updated

### Function Count Verification

After completion:

| Category | Current | Target | New Functions |
|----------|---------|--------|---------------|
| foxglove_msgs | 0 | ~50 | +50 |
| geometry_msgs | ~32 | ~100 | +68 |
| sensor_msgs | ~75 | ~115 | +40 |
| edgefirst_msgs | ~100 | ~140 | +40 |
| **Total** | 216 | ~415 | +~200 |

---

## File Changes Summary

| File | Changes |
|------|---------|
| `src/ffi.rs` | Add ~200 new FFI functions |
| `include/edgefirst/schemas.h` | Add corresponding declarations |
| `tests/c/test_foxglove_msgs.c` | New file (~25 tests) |
| `tests/c/test_geometry_msgs.c` | Expand (~40 more tests) |
| `tests/c/test_sensor_msgs.c` | Expand (~20 more tests) |
| `tests/c/test_edgefirst_msgs.c` | Expand (~25 more tests) |
| `tests/c/Makefile` | Add new test file |

---

## Success Criteria

- [ ] FoxgloveCompressedVideo FFI complete (11 functions)
- [ ] RadarCube FFI complete (22 functions)
- [ ] All geometry_msgs types have FFI (~70 functions)
- [ ] IMU and CameraInfo FFI complete (~25 functions)
- [ ] All new FFI functions have tests
- [ ] `cargo build --release` succeeds
- [ ] `cd tests/c && make test` passes
- [ ] C header is in sync with FFI

---

**Next Phase:** [TODO_C.md](./TODO_C.md) - C Coverage Instrumentation (depends on this phase)
