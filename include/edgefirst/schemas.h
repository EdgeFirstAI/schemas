/**
 * @file schemas.h
 * @brief EdgeFirst Schemas C API
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.
 *
 * C API for EdgeFirst Perception middleware message schemas with CDR serialization.
 *
 * @section Overview
 *
 * This library provides C bindings to the EdgeFirst Rust schema library, offering:
 * - Zero-copy CDR encode/decode for fixed-size types (Time, Duration, Vector3, etc.)
 * - Opaque buffer-backed view handles for variable-size types (Image, Detect, etc.)
 * - O(1) field accessors on view handles (string pointers borrow into the CDR buffer)
 * - Encode functions that allocate and return serialized CDR bytes
 * - Memory management via ros_bytes_free() for encode output
 *
 * @section API Patterns
 *
 * **CdrFixed types** (Time, Duration, Vector3, Point, Quaternion, Pose, Transform,
 * Twist, Accel, NavSatStatus):
 *   - `ros_<type>_encode(buf, cap, &written, ...fields)` -- write CDR to caller buffer
 *   - `ros_<type>_decode(data, len, ...out_fields)` -- read fields from CDR
 *   - Pass `buf = NULL` to query the required size via `written`.
 *
 * **Buffer-backed types** (Header, Image, CompressedImage, CompressedVideo, Mask,
 * DmaBuffer, Imu, NavSatFix, TransformStamped, RadarCube, RadarInfo, Detect,
 * Model, ModelInfo, PointCloud2, CameraInfo, Track, Box, LocalTime):
 *   - `ros_<type>_from_cdr(data, len)` -- create an opaque handle from CDR bytes
 *   - `ros_<type>_get_<field>(handle)` -- O(1) field access
 *   - `ros_<type>_as_cdr(handle, &out_len)` -- borrow the raw CDR bytes
 *   - `ros_<type>_free(handle)` -- release the handle
 *   - `ros_<type>_encode(&out_bytes, &out_len, ...fields)` -- allocate + write CDR
 *
 * @section Error Handling
 *
 * Functions that can fail use standard POSIX errno conventions:
 * - Functions returning int: 0 on success, -1 on error (errno set)
 * - Functions returning pointers: valid pointer on success, NULL on error (errno set)
 *
 * Standard errno codes used:
 * - EINVAL: Invalid argument (NULL pointer, bad UTF-8, invalid parameter)
 * - ENOBUFS: Buffer too small (for CdrFixed encode with insufficient capacity)
 * - EBADMSG: Bad message (deserialization/decoding failure)
 *
 * Thread Safety: errno is thread-local in POSIX systems. Functions are thread-safe
 * for distinct message instances. Shared instances require external synchronization.
 *
 * @section Memory
 *
 * - View handles are owned by the caller and must be freed with `ros_<type>_free()`.
 * - String getters return `const char*` pointing into the CDR buffer (zero-copy,
 *   valid as long as the handle lives). Callers must NOT free these pointers.
 * - Byte blob getters return `const uint8_t*` with an `out_len` parameter,
 *   also borrowing into the CDR buffer.
 * - Encode functions for buffer-backed types allocate output via `uint8_t**` and
 *   `size_t*`; callers must free with `ros_bytes_free()`.
 *
 * **Parent-borrowed child handles.** Functions like ros_detect_get_box() and
 * ros_model_get_box() / ros_model_get_mask() return borrowed child handles whose
 * lifetime is tied to the parent handle. These pointers become invalid when
 * the parent is freed via its corresponding _free function. Do NOT pass them
 * to ros_box_free() / ros_mask_free() — the parent owns their storage. The
 * parent's underlying CDR buffer (passed to the parent's _from_cdr function)
 * must also remain valid for as long as the child pointers are used.
 *
 * @subsection Example
 * @code
 * #include <edgefirst/schemas.h>
 * #include <errno.h>
 * #include <stdio.h>
 * #include <string.h>
 *
 * // Encode a Header to CDR
 * uint8_t* bytes = NULL;
 * size_t len = 0;
 * if (ros_header_encode(&bytes, &len, 1234, 0, "camera") != 0) {
 *     perror("ros_header_encode");
 *     return 1;
 * }
 *
 * // Decode from CDR
 * ros_header_t* hdr = ros_header_from_cdr(bytes, len);
 * if (!hdr) {
 *     perror("ros_header_from_cdr");
 *     ros_bytes_free(bytes, len);
 *     return 1;
 * }
 *
 * // Access fields (zero-copy, borrowed pointer)
 * const char* frame_id = ros_header_get_frame_id(hdr);
 * printf("Frame ID: %s\n", frame_id);
 * printf("Stamp: %d.%u\n", ros_header_get_stamp_sec(hdr),
 *        ros_header_get_stamp_nanosec(hdr));
 *
 * // Cleanup
 * ros_header_free(hdr);
 * ros_bytes_free(bytes, len);
 * @endcode
 */

#ifndef EDGEFIRST_SCHEMAS_H
#define EDGEFIRST_SCHEMAS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Opaque Handle Types (buffer-backed views)
 * ========================================================================= */

/* std_msgs */
/** @brief Opaque buffer-backed view handle for std_msgs::Header. */
typedef struct ros_header_t ros_header_t;

/* sensor_msgs */
/** @brief Opaque buffer-backed view handle for sensor_msgs::Image. */
typedef struct ros_image_t ros_image_t;
/** @brief Opaque buffer-backed view handle for sensor_msgs::CompressedImage. */
typedef struct ros_compressed_image_t ros_compressed_image_t;
/** @brief Opaque buffer-backed view handle for sensor_msgs::Imu. */
typedef struct ros_imu_t ros_imu_t;
/** @brief Opaque buffer-backed view handle for sensor_msgs::NavSatFix. */
typedef struct ros_nav_sat_fix_t ros_nav_sat_fix_t;
/** @brief Opaque buffer-backed view handle for sensor_msgs::PointCloud2. */
typedef struct ros_point_cloud2_t ros_point_cloud2_t;
/** @brief Opaque buffer-backed view handle for sensor_msgs::CameraInfo. */
typedef struct ros_camera_info_t ros_camera_info_t;

/* geometry_msgs */
/** @brief Opaque buffer-backed view handle for geometry_msgs::TransformStamped. */
typedef struct ros_transform_stamped_t ros_transform_stamped_t;

/* foxglove_msgs */
/** @brief Opaque buffer-backed view handle for foxglove_msgs::CompressedVideo. */
typedef struct ros_compressed_video_t ros_compressed_video_t;

/* edgefirst_msgs */
/**
 * @brief Opaque view handle for an edgefirst_msgs::Mask.
 *
 * Handles may be either standalone-owned (returned by
 * ros_mask_from_cdr() and freed via ros_mask_free()) or parent-borrowed
 * (returned by ros_model_get_mask() — lifetime tied to the parent
 * ros_model_t, must NOT be passed to ros_mask_free(); see Memory
 * Management Rule 5). The same typedef covers both cases; ros_mask_free()
 * internally detects borrowed handles and safely no-ops with errno=EINVAL
 * instead of corrupting the parent's internal storage.
 */
typedef struct ros_mask_t ros_mask_t;
/**
 * @brief Opaque buffer-backed view handle for edgefirst_msgs::DmaBuffer.
 *
 * @deprecated Use ros_camera_frame_t (see below). DmaBuffer is
 * deprecated as of 3.1.0 and will be removed in 4.0.0. CameraFrame adds
 * multi-plane support, compressed bitstream handling, GPU fences, frame
 * sequence counter, colorimetry, and off-device inline-data bridges.
 */
typedef struct ros_dmabuffer_t ros_dmabuffer_t;
/* typedef itself cannot carry a portable deprecation attribute across C
 * dialects; individual functions below carry the attribute so any consumer
 * call site triggers the compile-time warning. */
/** @brief Opaque buffer-backed view handle for edgefirst_msgs::RadarCube. */
typedef struct ros_radar_cube_t ros_radar_cube_t;
/** @brief Opaque buffer-backed view handle for edgefirst_msgs::RadarInfo. */
typedef struct ros_radar_info_t ros_radar_info_t;
/** @brief Opaque buffer-backed view handle for edgefirst_msgs::Detect. */
typedef struct ros_detect_t ros_detect_t;
/** @brief Opaque buffer-backed view handle for edgefirst_msgs::Model. */
typedef struct ros_model_t ros_model_t;
/** @brief Opaque buffer-backed view handle for edgefirst_msgs::ModelInfo. */
typedef struct ros_model_info_t ros_model_info_t;
/** @brief Opaque buffer-backed view handle for edgefirst_msgs::Track. */
typedef struct ros_track_t ros_track_t;
/**
 * @brief Opaque view handle for an edgefirst_msgs::DetectBox.
 *
 * Handles may be either standalone-owned (returned by ros_box_from_cdr()
 * and freed via ros_box_free()) or parent-borrowed (returned by
 * ros_detect_get_box() / ros_model_get_box() — lifetime tied to the parent
 * ros_detect_t / ros_model_t, must NOT be passed to ros_box_free(); see
 * Memory Management Rule 5). The same typedef covers both cases;
 * ros_box_free() internally detects borrowed handles and safely no-ops
 * with errno=EINVAL instead of corrupting the parent's internal storage.
 */
typedef struct ros_box_t ros_box_t;
/** @brief Opaque buffer-backed view handle for edgefirst_msgs::LocalTime. */
typedef struct ros_local_time_t ros_local_time_t;
/** @brief Opaque buffer-backed view handle for edgefirst_msgs::CameraFrame. */
typedef struct ros_camera_frame_t ros_camera_frame_t;
/**
 * @brief Opaque view handle for an edgefirst_msgs::CameraPlane.
 *
 * Returned by ros_camera_frame_get_plane() as a parent-borrowed handle:
 * lifetime is tied to the parent ros_camera_frame_t, and the handle
 * MUST NOT be passed to ros_camera_plane_free() (see Memory Management
 * Rule 5). The free function detects borrowed handles and safely no-ops
 * with errno=EINVAL.
 */
typedef struct ros_camera_plane_t ros_camera_plane_t;

/* ============================================================================
 * Memory Management
 * ========================================================================= */

/**
 * @brief Free a byte buffer returned by any ros_*_encode() function.
 * @param bytes Pointer previously returned by an encode function (NULL is safe)
 * @param len Length that was returned alongside the pointer
 *
 * Passing NULL is a no-op. Passing any pointer not returned by an encode
 * function is undefined behaviour.
 */
void ros_bytes_free(uint8_t* bytes, size_t len);

/* ============================================================================
 * builtin_interfaces - Time (CdrFixed)
 * ========================================================================= */

/**
 * @brief Encode a Time message to CDR.
 * @param buf Destination buffer, or NULL to query required size
 * @param cap Capacity of buf in bytes (ignored when buf is NULL)
 * @param written Receives the number of bytes written (or required)
 * @param sec Seconds since epoch
 * @param nanosec Nanoseconds component
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - ENOBUFS: buf is non-NULL but cap is too small
 * - EBADMSG: Encoding failure
 */
int ros_time_encode(uint8_t* buf, size_t cap, size_t* written,
                    int32_t sec, uint32_t nanosec);

/**
 * @brief Decode a Time message from CDR.
 * @param data CDR encoded bytes
 * @param len Length of data
 * @param sec Receives seconds (may be NULL)
 * @param nanosec Receives nanoseconds (may be NULL)
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EBADMSG: Decoding failure
 */
int ros_time_decode(const uint8_t* data, size_t len,
                    int32_t* sec, uint32_t* nanosec);

/* ============================================================================
 * builtin_interfaces - Duration (CdrFixed)
 * ========================================================================= */

/**
 * @brief Encode a Duration message to CDR.
 * @param buf Destination buffer, or NULL to query required size
 * @param cap Capacity of buf in bytes
 * @param written Receives the number of bytes written (or required)
 * @param sec Seconds
 * @param nanosec Nanoseconds
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - ENOBUFS: buf is non-NULL but cap is too small
 * - EBADMSG: Encoding failure
 */
int ros_duration_encode(uint8_t* buf, size_t cap, size_t* written,
                        int32_t sec, uint32_t nanosec);

/**
 * @brief Decode a Duration message from CDR.
 * @param data CDR encoded bytes
 * @param len Length of data
 * @param sec Receives seconds (may be NULL)
 * @param nanosec Receives nanoseconds (may be NULL)
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EBADMSG: Decoding failure
 */
int ros_duration_decode(const uint8_t* data, size_t len,
                        int32_t* sec, uint32_t* nanosec);

/* ============================================================================
 * geometry_msgs - Vector3 (CdrFixed)
 * ========================================================================= */

/**
 * @brief Encode a Vector3 message to CDR.
 * @param buf Destination buffer, or NULL to query required size
 * @param cap Capacity of buf in bytes
 * @param written Receives the number of bytes written (or required)
 * @param x X component
 * @param y Y component
 * @param z Z component
 * @return 0 on success, -1 on error
 */
int ros_vector3_encode(uint8_t* buf, size_t cap, size_t* written,
                       double x, double y, double z);

/**
 * @brief Decode a Vector3 message from CDR.
 * @param data CDR encoded bytes
 * @param len Length of data
 * @param x Receives X (may be NULL)
 * @param y Receives Y (may be NULL)
 * @param z Receives Z (may be NULL)
 * @return 0 on success, -1 on error
 */
int ros_vector3_decode(const uint8_t* data, size_t len,
                       double* x, double* y, double* z);

/* ============================================================================
 * geometry_msgs - Point (CdrFixed)
 * ========================================================================= */

/**
 * @brief Encode a Point message to CDR.
 * @param buf Destination buffer, or NULL to query required size
 * @param cap Capacity of buf in bytes
 * @param written Receives the number of bytes written (or required)
 * @param x X coordinate
 * @param y Y coordinate
 * @param z Z coordinate
 * @return 0 on success, -1 on error
 */
int ros_point_encode(uint8_t* buf, size_t cap, size_t* written,
                     double x, double y, double z);

/**
 * @brief Decode a Point message from CDR.
 * @param data CDR encoded bytes
 * @param len Length of data
 * @param x Receives X (may be NULL)
 * @param y Receives Y (may be NULL)
 * @param z Receives Z (may be NULL)
 * @return 0 on success, -1 on error
 */
int ros_point_decode(const uint8_t* data, size_t len,
                     double* x, double* y, double* z);

/* ============================================================================
 * geometry_msgs - Quaternion (CdrFixed)
 * ========================================================================= */

/**
 * @brief Encode a Quaternion message to CDR.
 * @param buf Destination buffer, or NULL to query required size
 * @param cap Capacity of buf in bytes
 * @param written Receives the number of bytes written (or required)
 * @param x X component
 * @param y Y component
 * @param z Z component
 * @param w W component
 * @return 0 on success, -1 on error
 */
int ros_quaternion_encode(uint8_t* buf, size_t cap, size_t* written,
                          double x, double y, double z, double w);

/**
 * @brief Decode a Quaternion message from CDR.
 * @param data CDR encoded bytes
 * @param len Length of data
 * @param x Receives X (may be NULL)
 * @param y Receives Y (may be NULL)
 * @param z Receives Z (may be NULL)
 * @param w Receives W (may be NULL)
 * @return 0 on success, -1 on error
 */
int ros_quaternion_decode(const uint8_t* data, size_t len,
                          double* x, double* y, double* z, double* w);

/* ============================================================================
 * geometry_msgs - Pose (CdrFixed)
 * ========================================================================= */

/**
 * @brief Encode a Pose message to CDR.
 * @param buf Destination buffer, or NULL to query required size
 * @param cap Capacity of buf in bytes
 * @param written Receives the number of bytes written (or required)
 * @param px Position X
 * @param py Position Y
 * @param pz Position Z
 * @param ox Orientation X
 * @param oy Orientation Y
 * @param oz Orientation Z
 * @param ow Orientation W
 * @return 0 on success, -1 on error
 */
int ros_pose_encode(uint8_t* buf, size_t cap, size_t* written,
                    double px, double py, double pz,
                    double ox, double oy, double oz, double ow);

/**
 * @brief Decode a Pose message from CDR.
 * @param data CDR encoded bytes
 * @param len Length of data
 * @param px Receives position X (may be NULL)
 * @param py Receives position Y (may be NULL)
 * @param pz Receives position Z (may be NULL)
 * @param ox Receives orientation X (may be NULL)
 * @param oy Receives orientation Y (may be NULL)
 * @param oz Receives orientation Z (may be NULL)
 * @param ow Receives orientation W (may be NULL)
 * @return 0 on success, -1 on error
 */
int ros_pose_decode(const uint8_t* data, size_t len,
                    double* px, double* py, double* pz,
                    double* ox, double* oy, double* oz, double* ow);

/* ============================================================================
 * geometry_msgs - Transform (CdrFixed)
 * ========================================================================= */

/**
 * @brief Encode a Transform message to CDR.
 * @param buf Destination buffer, or NULL to query required size
 * @param cap Capacity of buf in bytes
 * @param written Receives the number of bytes written (or required)
 * @param tx Translation X
 * @param ty Translation Y
 * @param tz Translation Z
 * @param rx Rotation X
 * @param ry Rotation Y
 * @param rz Rotation Z
 * @param rw Rotation W
 * @return 0 on success, -1 on error
 */
int ros_transform_encode(uint8_t* buf, size_t cap, size_t* written,
                         double tx, double ty, double tz,
                         double rx, double ry, double rz, double rw);

/**
 * @brief Decode a Transform message from CDR.
 * @param data CDR encoded bytes
 * @param len Length of data
 * @param tx Receives translation X (may be NULL)
 * @param ty Receives translation Y (may be NULL)
 * @param tz Receives translation Z (may be NULL)
 * @param rx Receives rotation X (may be NULL)
 * @param ry Receives rotation Y (may be NULL)
 * @param rz Receives rotation Z (may be NULL)
 * @param rw Receives rotation W (may be NULL)
 * @return 0 on success, -1 on error
 */
int ros_transform_decode(const uint8_t* data, size_t len,
                         double* tx, double* ty, double* tz,
                         double* rx, double* ry, double* rz, double* rw);

/* ============================================================================
 * geometry_msgs - Twist (CdrFixed)
 * ========================================================================= */

/**
 * @brief Encode a Twist message to CDR.
 * @param buf Destination buffer, or NULL to query required size
 * @param cap Capacity of buf in bytes
 * @param written Receives the number of bytes written (or required)
 * @param lx Linear velocity X
 * @param ly Linear velocity Y
 * @param lz Linear velocity Z
 * @param ax Angular velocity X
 * @param ay Angular velocity Y
 * @param az Angular velocity Z
 * @return 0 on success, -1 on error
 */
int ros_twist_encode(uint8_t* buf, size_t cap, size_t* written,
                     double lx, double ly, double lz,
                     double ax, double ay, double az);

/**
 * @brief Decode a Twist message from CDR.
 * @param data CDR encoded bytes
 * @param len Length of data
 * @param lx Receives linear X (may be NULL)
 * @param ly Receives linear Y (may be NULL)
 * @param lz Receives linear Z (may be NULL)
 * @param ax Receives angular X (may be NULL)
 * @param ay Receives angular Y (may be NULL)
 * @param az Receives angular Z (may be NULL)
 * @return 0 on success, -1 on error
 */
int ros_twist_decode(const uint8_t* data, size_t len,
                     double* lx, double* ly, double* lz,
                     double* ax, double* ay, double* az);

/* ============================================================================
 * geometry_msgs - Accel (CdrFixed)
 * ========================================================================= */

/**
 * @brief Encode an Accel message to CDR.
 * @param buf Destination buffer, or NULL to query required size
 * @param cap Capacity of buf in bytes
 * @param written Receives the number of bytes written (or required)
 * @param lx Linear acceleration X
 * @param ly Linear acceleration Y
 * @param lz Linear acceleration Z
 * @param ax Angular acceleration X
 * @param ay Angular acceleration Y
 * @param az Angular acceleration Z
 * @return 0 on success, -1 on error
 */
int ros_accel_encode(uint8_t* buf, size_t cap, size_t* written,
                     double lx, double ly, double lz,
                     double ax, double ay, double az);

/**
 * @brief Decode an Accel message from CDR.
 * @param data CDR encoded bytes
 * @param len Length of data
 * @param lx Receives linear X (may be NULL)
 * @param ly Receives linear Y (may be NULL)
 * @param lz Receives linear Z (may be NULL)
 * @param ax Receives angular X (may be NULL)
 * @param ay Receives angular Y (may be NULL)
 * @param az Receives angular Z (may be NULL)
 * @return 0 on success, -1 on error
 */
int ros_accel_decode(const uint8_t* data, size_t len,
                     double* lx, double* ly, double* lz,
                     double* ax, double* ay, double* az);

/* ============================================================================
 * sensor_msgs - NavSatStatus (CdrFixed)
 * ========================================================================= */

/**
 * @brief Encode a NavSatStatus message to CDR.
 * @param buf Destination buffer, or NULL to query required size
 * @param cap Capacity of buf in bytes
 * @param written Receives the number of bytes written (or required)
 * @param status Status field (int8)
 * @param service Service bitmask (uint16)
 * @return 0 on success, -1 on error
 */
int ros_nav_sat_status_encode(uint8_t* buf, size_t cap, size_t* written,
                              int8_t status, uint16_t service);

/**
 * @brief Decode a NavSatStatus message from CDR.
 * @param data CDR encoded bytes
 * @param len Length of data
 * @param status Receives status (may be NULL)
 * @param service Receives service bitmask (may be NULL)
 * @return 0 on success, -1 on error
 */
int ros_nav_sat_status_decode(const uint8_t* data, size_t len,
                              int8_t* status, uint16_t* service);

/* ============================================================================
 * std_msgs - Header (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a Header view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: data is NULL
 * - EBADMSG: CDR decoding failed
 */
ros_header_t* ros_header_from_cdr(const uint8_t* data, size_t len);

/**
 * @brief Free a Header view handle.
 * @param view Handle to free (NULL is safe)
 */
void ros_header_free(ros_header_t* view);

/** @brief Get stamp seconds. */
int32_t ros_header_get_stamp_sec(const ros_header_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_header_get_stamp_nanosec(const ros_header_t* view);

/**
 * @brief Get frame_id string (borrowed, valid while handle lives).
 * @param view Header handle
 * @return C string pointer or NULL if view is NULL
 */
const char* ros_header_get_frame_id(const ros_header_t* view);

/**
 * @brief Borrow the raw CDR bytes from the handle.
 * @param view Header handle
 * @param out_len Receives byte count
 * @return Pointer to CDR bytes (valid while handle lives) or NULL
 */
const uint8_t* ros_header_as_cdr(const ros_header_t* view, size_t* out_len);

/**
 * @brief Encode a Header to CDR (allocates output).
 * @param out_bytes Receives allocated byte pointer (free with ros_bytes_free)
 * @param out_len Receives byte count
 * @param stamp_sec Stamp seconds
 * @param stamp_nanosec Stamp nanoseconds
 * @param frame_id Frame ID string (NULL treated as "")
 * @return 0 on success, -1 on error
 */
int ros_header_encode(uint8_t** out_bytes, size_t* out_len,
                      int32_t stamp_sec, uint32_t stamp_nanosec,
                      const char* frame_id);

/* ----------------------------------------------------------------------------
 * std_msgs - Header (builder, 3.2.0+)
 * --------------------------------------------------------------------------*/

/** @brief Opaque builder handle for ros_header_t messages. */
typedef struct ros_header_builder_s ros_header_builder_t;

/**
 * @brief Create a new Header builder with zero-valued defaults.
 * @return Opaque handle, or NULL on allocation failure. Free with
 *         ros_header_builder_free.
 */
ros_header_builder_t* ros_header_builder_new(void);

/** @brief Free a Header builder handle. NULL-safe. */
void ros_header_builder_free(ros_header_builder_t* b);

/** @brief Set the stamp field. */
void ros_header_builder_set_stamp(ros_header_builder_t* b,
                                  int32_t sec, uint32_t nsec);

/**
 * @brief Set the frame_id field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_header_builder_set_frame_id(ros_header_builder_t* b, const char* s);

/**
 * @brief Allocate a fresh CDR buffer and encode the message.
 * @return 0 on success (out_bytes/out_len written; free via ros_bytes_free),
 *         -1 on error (errno set: EINVAL for NULL handle, EBADMSG on encoder error).
 */
int  ros_header_builder_build(ros_header_builder_t* b,
                              uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Encode the message into a caller-owned buffer.
 * @return 0 on success (out_len written), -1 on error (errno: EINVAL for
 *         NULL args, ENOBUFS for buffer too small, EBADMSG for encoding error).
 */
int  ros_header_builder_encode_into(ros_header_builder_t* b,
                                    uint8_t* buf, size_t cap,
                                    size_t* out_len);

/* ============================================================================
 * sensor_msgs - Image (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create an Image view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_image_t* ros_image_from_cdr(const uint8_t* data, size_t len);

/** @brief Free an Image view handle. */
void ros_image_free(ros_image_t* view);

/** @brief Get stamp seconds. */
int32_t ros_image_get_stamp_sec(const ros_image_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_image_get_stamp_nanosec(const ros_image_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_image_get_frame_id(const ros_image_t* view);

/** @brief Get image height in pixels. */
uint32_t ros_image_get_height(const ros_image_t* view);

/** @brief Get image width in pixels. */
uint32_t ros_image_get_width(const ros_image_t* view);

/** @brief Get encoding string (borrowed), e.g. "rgb8", "bgr8". */
const char* ros_image_get_encoding(const ros_image_t* view);

/** @brief Get big-endian flag. */
uint8_t ros_image_get_is_bigendian(const ros_image_t* view);

/** @brief Get row step (bytes per row). */
uint32_t ros_image_get_step(const ros_image_t* view);

/**
 * @brief Get image pixel data (borrowed).
 * @param view Image handle
 * @param out_len Receives byte count
 * @return Pointer to pixel data or NULL
 */
const uint8_t* ros_image_get_data(const ros_image_t* view, size_t* out_len);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_image_as_cdr(const ros_image_t* view, size_t* out_len);

/**
 * @brief Encode an Image to CDR (allocates output).
 * @param out_bytes Receives allocated byte pointer (free with ros_bytes_free)
 * @param out_len Receives byte count
 * @param stamp_sec Stamp seconds
 * @param stamp_nanosec Stamp nanoseconds
 * @param frame_id Frame ID string
 * @param height Image height
 * @param width Image width
 * @param encoding Encoding string
 * @param is_bigendian Big-endian flag
 * @param step Row step in bytes
 * @param data Pixel data
 * @param data_len Length of pixel data
 * @return 0 on success, -1 on error
 */
int ros_image_encode(uint8_t** out_bytes, size_t* out_len,
                     int32_t stamp_sec, uint32_t stamp_nanosec,
                     const char* frame_id,
                     uint32_t height, uint32_t width,
                     const char* encoding, uint8_t is_bigendian,
                     uint32_t step, const uint8_t* data, size_t data_len);

/* ----------------------------------------------------------------------------
 * sensor_msgs - Image (builder, 3.2.0+)
 * --------------------------------------------------------------------------*/

/** @brief Opaque builder handle for ros_image_t messages. */
typedef struct ros_image_builder_s ros_image_builder_t;

/**
 * @brief Create a new Image builder with zero-valued defaults.
 * @return Opaque handle, or NULL on allocation failure. Free with
 *         ros_image_builder_free.
 */
ros_image_builder_t* ros_image_builder_new(void);

/** @brief Free an Image builder handle. NULL-safe. */
void ros_image_builder_free(ros_image_builder_t* b);

/** @brief Set the stamp field. */
void ros_image_builder_set_stamp(ros_image_builder_t* b,
                                 int32_t sec, uint32_t nsec);

/**
 * @brief Set the frame_id field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_image_builder_set_frame_id(ros_image_builder_t* b, const char* s);

/** @brief Set the height field. */
void ros_image_builder_set_height(ros_image_builder_t* b, uint32_t v);

/** @brief Set the width field. */
void ros_image_builder_set_width(ros_image_builder_t* b, uint32_t v);

/**
 * @brief Set the encoding field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_image_builder_set_encoding(ros_image_builder_t* b, const char* s);

/** @brief Set the is_bigendian field. */
void ros_image_builder_set_is_bigendian(ros_image_builder_t* b, uint8_t v);

/** @brief Set the step (row stride in bytes) field. */
void ros_image_builder_set_step(ros_image_builder_t* b, uint32_t v);

/**
 * @brief Set the data bulk byte sequence (BORROWED — must remain valid
 *        until the next setter, build, encode_into, or free call).
 */
void ros_image_builder_set_data(ros_image_builder_t* b,
                                const uint8_t* data, size_t len);

/**
 * @brief Allocate a fresh CDR buffer and encode the message.
 * @return 0 on success (out_bytes/out_len written; free via ros_bytes_free),
 *         -1 on error (errno set: EINVAL for NULL handle, EBADMSG on encoder error).
 */
int  ros_image_builder_build(ros_image_builder_t* b,
                             uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Encode the message into a caller-owned buffer.
 * @return 0 on success (out_len written), -1 on error (errno: EINVAL for
 *         NULL args, ENOBUFS for buffer too small, EBADMSG for encoding error).
 */
int  ros_image_builder_encode_into(ros_image_builder_t* b,
                                   uint8_t* buf, size_t cap,
                                   size_t* out_len);

/* ============================================================================
 * sensor_msgs - CompressedImage (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a CompressedImage view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_compressed_image_t* ros_compressed_image_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a CompressedImage view handle. */
void ros_compressed_image_free(ros_compressed_image_t* view);

/** @brief Get stamp seconds. */
int32_t ros_compressed_image_get_stamp_sec(const ros_compressed_image_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_compressed_image_get_stamp_nanosec(const ros_compressed_image_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_compressed_image_get_frame_id(const ros_compressed_image_t* view);

/** @brief Get format string (borrowed), e.g. "jpeg", "png". */
const char* ros_compressed_image_get_format(const ros_compressed_image_t* view);

/**
 * @brief Get compressed image data (borrowed).
 * @param view CompressedImage handle
 * @param out_len Receives byte count
 * @return Pointer to compressed data or NULL
 */
const uint8_t* ros_compressed_image_get_data(const ros_compressed_image_t* view, size_t* out_len);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_compressed_image_as_cdr(const ros_compressed_image_t* view, size_t* out_len);

/**
 * @brief Encode a CompressedImage to CDR (allocates output).
 * @param out_bytes Receives allocated byte pointer (free with ros_bytes_free)
 * @param out_len Receives byte count
 * @param stamp_sec Stamp seconds
 * @param stamp_nanosec Stamp nanoseconds
 * @param frame_id Frame ID string
 * @param format Format string
 * @param data Compressed image data
 * @param data_len Length of compressed data
 * @return 0 on success, -1 on error
 */
int ros_compressed_image_encode(uint8_t** out_bytes, size_t* out_len,
                                int32_t stamp_sec, uint32_t stamp_nanosec,
                                const char* frame_id, const char* format,
                                const uint8_t* data, size_t data_len);

/* ----------------------------------------------------------------------------
 * sensor_msgs - CompressedImage (builder, 3.2.0+)
 * --------------------------------------------------------------------------*/

/** @brief Opaque builder handle for ros_compressed_image_t messages. */
typedef struct ros_compressed_image_builder_s ros_compressed_image_builder_t;

/**
 * @brief Create a new CompressedImage builder with zero-valued defaults.
 * @return Opaque handle, or NULL on allocation failure. Free with
 *         ros_compressed_image_builder_free.
 */
ros_compressed_image_builder_t* ros_compressed_image_builder_new(void);

/** @brief Free a CompressedImage builder handle. NULL-safe. */
void ros_compressed_image_builder_free(ros_compressed_image_builder_t* b);

/** @brief Set the stamp field. */
void ros_compressed_image_builder_set_stamp(ros_compressed_image_builder_t* b,
                                            int32_t sec, uint32_t nsec);

/**
 * @brief Set the frame_id field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_compressed_image_builder_set_frame_id(
    ros_compressed_image_builder_t* b, const char* s);

/**
 * @brief Set the format field (string is copied), e.g. "jpeg", "png".
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_compressed_image_builder_set_format(
    ros_compressed_image_builder_t* b, const char* s);

/**
 * @brief Set the compressed data bulk byte sequence (BORROWED — must remain
 *        valid until the next setter, build, encode_into, or free call).
 */
void ros_compressed_image_builder_set_data(ros_compressed_image_builder_t* b,
                                           const uint8_t* data, size_t len);

/**
 * @brief Allocate a fresh CDR buffer and encode the message.
 * @return 0 on success (out_bytes/out_len written; free via ros_bytes_free),
 *         -1 on error (errno set: EINVAL for NULL handle, EBADMSG on encoder error).
 */
int  ros_compressed_image_builder_build(ros_compressed_image_builder_t* b,
                                        uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Encode the message into a caller-owned buffer.
 * @return 0 on success (out_len written), -1 on error (errno: EINVAL for
 *         NULL args, ENOBUFS for buffer too small, EBADMSG for encoding error).
 */
int  ros_compressed_image_builder_encode_into(
    ros_compressed_image_builder_t* b, uint8_t* buf, size_t cap,
    size_t* out_len);

/* ============================================================================
 * foxglove_msgs - CompressedVideo (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a CompressedVideo view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_compressed_video_t* ros_compressed_video_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a CompressedVideo view handle. */
void ros_compressed_video_free(ros_compressed_video_t* view);

/** @brief Get stamp seconds. */
int32_t ros_compressed_video_get_stamp_sec(const ros_compressed_video_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_compressed_video_get_stamp_nanosec(const ros_compressed_video_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_compressed_video_get_frame_id(const ros_compressed_video_t* view);

/**
 * @brief Get compressed video data (borrowed).
 * @param view CompressedVideo handle
 * @param out_len Receives byte count
 * @return Pointer to video data or NULL
 */
const uint8_t* ros_compressed_video_get_data(const ros_compressed_video_t* view, size_t* out_len);

/** @brief Get format string (borrowed), e.g. "h264", "h265". */
const char* ros_compressed_video_get_format(const ros_compressed_video_t* view);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_compressed_video_as_cdr(const ros_compressed_video_t* view, size_t* out_len);

/**
 * @brief Encode a CompressedVideo to CDR (allocates output).
 * @param out_bytes Receives allocated byte pointer (free with ros_bytes_free)
 * @param out_len Receives byte count
 * @param stamp_sec Stamp seconds
 * @param stamp_nanosec Stamp nanoseconds
 * @param frame_id Frame ID string
 * @param data Video data
 * @param data_len Length of video data
 * @param format Format string
 * @return 0 on success, -1 on error
 */
int ros_compressed_video_encode(uint8_t** out_bytes, size_t* out_len,
                                int32_t stamp_sec, uint32_t stamp_nanosec,
                                const char* frame_id,
                                const uint8_t* data, size_t data_len,
                                const char* format);

/* ============================================================================
 * edgefirst_msgs - Mask (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a Mask view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_mask_t* ros_mask_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a Mask view handle. */
void ros_mask_free(ros_mask_t* view);

/** @brief Get mask height. */
uint32_t ros_mask_get_height(const ros_mask_t* view);

/** @brief Get mask width. */
uint32_t ros_mask_get_width(const ros_mask_t* view);

/** @brief Get mask length. */
uint32_t ros_mask_get_length(const ros_mask_t* view);

/** @brief Get encoding string (borrowed). */
const char* ros_mask_get_encoding(const ros_mask_t* view);

/**
 * @brief Get mask data (borrowed).
 * @param view Mask handle
 * @param out_len Receives byte count
 * @return Pointer to mask data or NULL
 */
const uint8_t* ros_mask_get_data(const ros_mask_t* view, size_t* out_len);

/** @brief Get boxed flag. */
bool ros_mask_get_boxed(const ros_mask_t* view);

/**
 * @brief Encode a Mask to CDR (allocates output).
 * @param out_bytes Receives allocated byte pointer (free with ros_bytes_free)
 * @param out_len Receives byte count
 * @param height Mask height
 * @param width Mask width
 * @param length Mask length
 * @param encoding Encoding string
 * @param data Mask data
 * @param data_len Length of mask data
 * @param boxed Boxed flag
 * @return 0 on success, -1 on error
 */
int ros_mask_encode(uint8_t** out_bytes, size_t* out_len,
                    uint32_t height, uint32_t width, uint32_t length,
                    const char* encoding,
                    const uint8_t* data, size_t data_len,
                    bool boxed);

/* ============================================================================
 * edgefirst_msgs - DmaBuffer (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a DmaBuffer view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
ros_dmabuffer_t* ros_dmabuffer_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a DmaBuffer view handle. */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
void ros_dmabuffer_free(ros_dmabuffer_t* view);

/** @brief Get stamp seconds. */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
int32_t ros_dmabuffer_get_stamp_sec(const ros_dmabuffer_t* view);

/** @brief Get stamp nanoseconds. */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
uint32_t ros_dmabuffer_get_stamp_nanosec(const ros_dmabuffer_t* view);

/** @brief Get frame_id (borrowed). */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
const char* ros_dmabuffer_get_frame_id(const ros_dmabuffer_t* view);

/** @brief Get process ID. */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
uint32_t ros_dmabuffer_get_pid(const ros_dmabuffer_t* view);

/** @brief Get file descriptor. */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
int32_t ros_dmabuffer_get_fd(const ros_dmabuffer_t* view);

/** @brief Get buffer width. */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
uint32_t ros_dmabuffer_get_width(const ros_dmabuffer_t* view);

/** @brief Get buffer height. */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
uint32_t ros_dmabuffer_get_height(const ros_dmabuffer_t* view);

/** @brief Get buffer stride. */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
uint32_t ros_dmabuffer_get_stride(const ros_dmabuffer_t* view);

/** @brief Get FourCC pixel format code. */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
uint32_t ros_dmabuffer_get_fourcc(const ros_dmabuffer_t* view);

/** @brief Get buffer length in bytes. */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
uint32_t ros_dmabuffer_get_length(const ros_dmabuffer_t* view);

/** @brief Borrow raw CDR bytes from the handle. */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
const uint8_t* ros_dmabuffer_as_cdr(const ros_dmabuffer_t* view, size_t* out_len);

/**
 * @brief Encode a DmaBuffer to CDR (allocates output).
 * @param out_bytes Receives allocated byte pointer (free with ros_bytes_free)
 * @param out_len Receives byte count
 * @param stamp_sec Stamp seconds
 * @param stamp_nanosec Stamp nanoseconds
 * @param frame_id Frame ID string
 * @param pid Process ID
 * @param fd File descriptor
 * @param width Buffer width
 * @param height Buffer height
 * @param stride Buffer stride
 * @param fourcc FourCC pixel format
 * @param length Buffer length in bytes
 * @return 0 on success, -1 on error
 */
__attribute__((deprecated("Use CameraFrame (ros_camera_frame_t) instead; removed in 4.0.0.")))
int ros_dmabuffer_encode(uint8_t** out_bytes, size_t* out_len,
                         int32_t stamp_sec, uint32_t stamp_nanosec,
                         const char* frame_id,
                         uint32_t pid, int32_t fd,
                         uint32_t width, uint32_t height,
                         uint32_t stride, uint32_t fourcc, uint32_t length);

/* ============================================================================
 * edgefirst_msgs - CameraFrame / CameraPlane (buffer-backed, multi-plane)
 * ========================================================================= */

/**
 * @brief Create a CameraFrame view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error (errno set to EINVAL or EBADMSG)
 */
ros_camera_frame_t* ros_camera_frame_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a CameraFrame view handle. Safe to call with NULL. */
void ros_camera_frame_free(ros_camera_frame_t* view);

/** @brief Get stamp seconds. */
int32_t  ros_camera_frame_get_stamp_sec(const ros_camera_frame_t* view);
/** @brief Get stamp nanoseconds. */
uint32_t ros_camera_frame_get_stamp_nanosec(const ros_camera_frame_t* view);
/** @brief Get coordinate frame id (borrowed from CDR buffer). */
const char* ros_camera_frame_get_frame_id(const ros_camera_frame_t* view);

/** @brief Get monotonic frame index (V4L2 sequence / libcamera Request.sequence). */
uint64_t ros_camera_frame_get_seq(const ros_camera_frame_t* view);
/** @brief Get producer process id (0 when all planes use inlined data). */
uint32_t ros_camera_frame_get_pid(const ros_camera_frame_t* view);
/** @brief Get image width in pixels. */
uint32_t ros_camera_frame_get_width(const ros_camera_frame_t* view);
/** @brief Get image height in pixels. */
uint32_t ros_camera_frame_get_height(const ros_camera_frame_t* view);
/** @brief Get DMA-fence sync_file fd (-1 = no fence / already signalled). */
int32_t  ros_camera_frame_get_fence_fd(const ros_camera_frame_t* view);

/** @brief Get format descriptor string (borrowed, e.g. "NV12", "h264"). */
const char* ros_camera_frame_get_format(const ros_camera_frame_t* view);
/** @brief Get color_space string (borrowed). */
const char* ros_camera_frame_get_color_space(const ros_camera_frame_t* view);
/** @brief Get color_transfer string (borrowed). */
const char* ros_camera_frame_get_color_transfer(const ros_camera_frame_t* view);
/** @brief Get color_encoding string (borrowed). */
const char* ros_camera_frame_get_color_encoding(const ros_camera_frame_t* view);
/** @brief Get color_range string (borrowed). */
const char* ros_camera_frame_get_color_range(const ros_camera_frame_t* view);

/** @brief Get the number of planes in this frame. */
uint32_t ros_camera_frame_get_planes_len(const ros_camera_frame_t* view);

/**
 * @brief Get a borrowed view of the i-th plane.
 * @param view CameraFrame handle
 * @param index Zero-based plane index (< ros_camera_frame_get_planes_len)
 * @return Borrowed pointer whose lifetime is tied to the parent CameraFrame
 *         handle, or NULL on error (errno set to EINVAL).
 *
 * The returned pointer is NOT a separately-owned handle: do NOT pass it to
 * ros_camera_plane_free(). It becomes invalid when the parent
 * ros_camera_frame_t handle is freed; the parent's CDR buffer must
 * also remain valid for as long as the returned pointer is used.
 */
const ros_camera_plane_t* ros_camera_frame_get_plane(
    const ros_camera_frame_t* view, uint32_t index);

/**
 * @brief Free a CameraPlane handle. Safe to call with NULL.
 *
 * Parent-borrowed handles (obtained from ros_camera_frame_get_plane())
 * must NOT be freed via this function; doing so sets errno=EINVAL and is a
 * no-op (defense-in-depth against API misuse).
 */
void ros_camera_plane_free(ros_camera_plane_t* view);

/** @brief Get plane fd (-1 = inlined in data[], no DMA-BUF). */
int32_t  ros_camera_plane_get_fd(const ros_camera_plane_t* view);
/** @brief Get plane byte offset within fd (ignored when fd == -1). */
uint32_t ros_camera_plane_get_offset(const ros_camera_plane_t* view);
/** @brief Get plane row stride in bytes. */
uint32_t ros_camera_plane_get_stride(const ros_camera_plane_t* view);
/** @brief Get plane capacity in bytes (buffer span). */
uint32_t ros_camera_plane_get_size(const ros_camera_plane_t* view);
/** @brief Get valid payload byte count (used <= size). */
uint32_t ros_camera_plane_get_used(const ros_camera_plane_t* view);

/**
 * @brief Get inlined plane data (only populated when fd == -1).
 * @param view CameraPlane handle
 * @param out_len Pointer to usize receiving the byte length (may be NULL)
 * @return Pointer to plane bytes inside the parent's CDR buffer, or NULL if
 *         the plane has no inlined data or the handle is invalid.
 */
const uint8_t* ros_camera_plane_get_data(
    const ros_camera_plane_t* view, size_t* out_len);

/* ============================================================================
 * sensor_msgs - Imu (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create an Imu view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_imu_t* ros_imu_from_cdr(const uint8_t* data, size_t len);

/** @brief Free an Imu view handle. */
void ros_imu_free(ros_imu_t* view);

/** @brief Get stamp seconds. */
int32_t ros_imu_get_stamp_sec(const ros_imu_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_imu_get_stamp_nanosec(const ros_imu_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_imu_get_frame_id(const ros_imu_t* view);

/**
 * @brief Get orientation quaternion (x, y, z, w).
 * @param view Imu handle
 * @param x Receives X (may be NULL)
 * @param y Receives Y (may be NULL)
 * @param z Receives Z (may be NULL)
 * @param w Receives W (may be NULL)
 */
void ros_imu_get_orientation(const ros_imu_t* view,
                             double* x, double* y, double* z, double* w);

/**
 * @brief Get 9-element orientation covariance (row-major 3x3).
 * @param view Imu handle
 * @param out Pointer to double[9] array to receive covariance values
 */
void ros_imu_get_orientation_covariance(const ros_imu_t* view, double* out);

/**
 * @brief Get angular velocity (x, y, z).
 * @param view Imu handle
 * @param x Receives X (may be NULL)
 * @param y Receives Y (may be NULL)
 * @param z Receives Z (may be NULL)
 */
void ros_imu_get_angular_velocity(const ros_imu_t* view,
                                  double* x, double* y, double* z);

/**
 * @brief Get 9-element angular velocity covariance (row-major 3x3).
 * @param view Imu handle
 * @param out Pointer to double[9] array to receive covariance values
 */
void ros_imu_get_angular_velocity_covariance(const ros_imu_t* view, double* out);

/**
 * @brief Get linear acceleration (x, y, z).
 * @param view Imu handle
 * @param x Receives X (may be NULL)
 * @param y Receives Y (may be NULL)
 * @param z Receives Z (may be NULL)
 */
void ros_imu_get_linear_acceleration(const ros_imu_t* view,
                                     double* x, double* y, double* z);

/**
 * @brief Get 9-element linear acceleration covariance (row-major 3x3).
 * @param view Imu handle
 * @param out Pointer to double[9] array to receive covariance values
 */
void ros_imu_get_linear_acceleration_covariance(const ros_imu_t* view, double* out);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_imu_as_cdr(const ros_imu_t* view, size_t* out_len);

/* ----------------------------------------------------------------------------
 * sensor_msgs - Imu (builder, 3.2.0+)
 * --------------------------------------------------------------------------*/

/** @brief Opaque builder handle for ros_imu_t messages. */
typedef struct ros_imu_builder_s ros_imu_builder_t;

/**
 * @brief Create a new Imu builder with zero-valued defaults.
 * @return Opaque handle, or NULL on allocation failure. Free with
 *         ros_imu_builder_free.
 */
ros_imu_builder_t* ros_imu_builder_new(void);

/** @brief Free an Imu builder handle. NULL-safe. */
void ros_imu_builder_free(ros_imu_builder_t* b);

/** @brief Set the stamp field. */
void ros_imu_builder_set_stamp(ros_imu_builder_t* b,
                               int32_t sec, uint32_t nsec);

/**
 * @brief Set the frame_id field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_imu_builder_set_frame_id(ros_imu_builder_t* b, const char* s);

/** @brief Set the orientation quaternion (x, y, z, w). */
void ros_imu_builder_set_orientation(ros_imu_builder_t* b,
                                     double x, double y, double z, double w);

/**
 * @brief Copy 9 f64 covariance values (row-major 3x3) into the builder.
 * @param cov Pointer to a double[9] buffer. The caller must ensure at least
 *            9 valid f64 elements are readable; values are copied.
 */
void ros_imu_builder_set_orientation_covariance(ros_imu_builder_t* b,
                                                const double* cov);

/** @brief Set the angular_velocity (rad/s, x, y, z). */
void ros_imu_builder_set_angular_velocity(ros_imu_builder_t* b,
                                          double x, double y, double z);

/** @brief Copy 9 f64 covariance values for angular velocity. */
void ros_imu_builder_set_angular_velocity_covariance(ros_imu_builder_t* b,
                                                     const double* cov);

/** @brief Set the linear_acceleration (m/s², x, y, z). */
void ros_imu_builder_set_linear_acceleration(ros_imu_builder_t* b,
                                             double x, double y, double z);

/** @brief Copy 9 f64 covariance values for linear acceleration. */
void ros_imu_builder_set_linear_acceleration_covariance(ros_imu_builder_t* b,
                                                        const double* cov);

/**
 * @brief Allocate a fresh CDR buffer and encode the message.
 * @return 0 on success (out_bytes/out_len written; free via ros_bytes_free),
 *         -1 on error (errno set: EINVAL for NULL handle, EBADMSG on encoder error).
 */
int  ros_imu_builder_build(ros_imu_builder_t* b,
                           uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Encode the message into a caller-owned buffer.
 * @return 0 on success (out_len written), -1 on error (errno: EINVAL for
 *         NULL args, ENOBUFS for buffer too small, EBADMSG for encoding error).
 */
int  ros_imu_builder_encode_into(ros_imu_builder_t* b,
                                 uint8_t* buf, size_t cap,
                                 size_t* out_len);

/* ============================================================================
 * sensor_msgs - NavSatFix (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a NavSatFix view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_nav_sat_fix_t* ros_nav_sat_fix_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a NavSatFix view handle. */
void ros_nav_sat_fix_free(ros_nav_sat_fix_t* view);

/** @brief Get stamp seconds. */
int32_t ros_nav_sat_fix_get_stamp_sec(const ros_nav_sat_fix_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_nav_sat_fix_get_stamp_nanosec(const ros_nav_sat_fix_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_nav_sat_fix_get_frame_id(const ros_nav_sat_fix_t* view);

/** @brief Get latitude in degrees. */
double ros_nav_sat_fix_get_latitude(const ros_nav_sat_fix_t* view);

/** @brief Get longitude in degrees. */
double ros_nav_sat_fix_get_longitude(const ros_nav_sat_fix_t* view);

/** @brief Get altitude in metres. */
double ros_nav_sat_fix_get_altitude(const ros_nav_sat_fix_t* view);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_nav_sat_fix_as_cdr(const ros_nav_sat_fix_t* view, size_t* out_len);

/* ----------------------------------------------------------------------------
 * sensor_msgs - NavSatFix (builder, 3.2.0+)
 * --------------------------------------------------------------------------*/

/** @brief Opaque builder handle for ros_nav_sat_fix_t messages. */
typedef struct ros_nav_sat_fix_builder_s ros_nav_sat_fix_builder_t;

/**
 * @brief Create a new NavSatFix builder with zero-valued defaults.
 * @return Opaque handle, or NULL on allocation failure. Free with
 *         ros_nav_sat_fix_builder_free.
 */
ros_nav_sat_fix_builder_t* ros_nav_sat_fix_builder_new(void);

/** @brief Free a NavSatFix builder handle. NULL-safe. */
void ros_nav_sat_fix_builder_free(ros_nav_sat_fix_builder_t* b);

/** @brief Set the stamp field. */
void ros_nav_sat_fix_builder_set_stamp(ros_nav_sat_fix_builder_t* b,
                                       int32_t sec, uint32_t nsec);

/**
 * @brief Set the frame_id field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_nav_sat_fix_builder_set_frame_id(ros_nav_sat_fix_builder_t* b,
                                          const char* s);

/**
 * @brief Set the status field (NavSatStatus: int8 status + uint16 service).
 */
void ros_nav_sat_fix_builder_set_status(ros_nav_sat_fix_builder_t* b,
                                        int8_t status, uint16_t service);

/** @brief Set the latitude field (degrees). */
void ros_nav_sat_fix_builder_set_latitude(ros_nav_sat_fix_builder_t* b,
                                          double v);

/** @brief Set the longitude field (degrees). */
void ros_nav_sat_fix_builder_set_longitude(ros_nav_sat_fix_builder_t* b,
                                           double v);

/** @brief Set the altitude field (metres). */
void ros_nav_sat_fix_builder_set_altitude(ros_nav_sat_fix_builder_t* b,
                                          double v);

/** @brief Copy 9 f64 position covariance values (row-major 3x3). */
void ros_nav_sat_fix_builder_set_position_covariance(
    ros_nav_sat_fix_builder_t* b, const double* cov);

/** @brief Set the position_covariance_type field. */
void ros_nav_sat_fix_builder_set_position_covariance_type(
    ros_nav_sat_fix_builder_t* b, uint8_t v);

/**
 * @brief Allocate a fresh CDR buffer and encode the message.
 * @return 0 on success (out_bytes/out_len written; free via ros_bytes_free),
 *         -1 on error (errno set: EINVAL for NULL handle, EBADMSG on encoder error).
 */
int  ros_nav_sat_fix_builder_build(ros_nav_sat_fix_builder_t* b,
                                   uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Encode the message into a caller-owned buffer.
 * @return 0 on success (out_len written), -1 on error (errno: EINVAL for
 *         NULL args, ENOBUFS for buffer too small, EBADMSG for encoding error).
 */
int  ros_nav_sat_fix_builder_encode_into(ros_nav_sat_fix_builder_t* b,
                                         uint8_t* buf, size_t cap,
                                         size_t* out_len);

/* ============================================================================
 * geometry_msgs - TransformStamped (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a TransformStamped view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_transform_stamped_t* ros_transform_stamped_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a TransformStamped view handle. */
void ros_transform_stamped_free(ros_transform_stamped_t* view);

/** @brief Get stamp seconds. */
int32_t ros_transform_stamped_get_stamp_sec(const ros_transform_stamped_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_transform_stamped_get_stamp_nanosec(const ros_transform_stamped_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_transform_stamped_get_frame_id(const ros_transform_stamped_t* view);

/** @brief Get child_frame_id (borrowed). */
const char* ros_transform_stamped_get_child_frame_id(const ros_transform_stamped_t* view);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_transform_stamped_as_cdr(const ros_transform_stamped_t* view, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - RadarCube (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a RadarCube view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_radar_cube_t* ros_radar_cube_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a RadarCube view handle. */
void ros_radar_cube_free(ros_radar_cube_t* view);

/** @brief Get stamp seconds. */
int32_t ros_radar_cube_get_stamp_sec(const ros_radar_cube_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_radar_cube_get_stamp_nanosec(const ros_radar_cube_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_radar_cube_get_frame_id(const ros_radar_cube_t* view);

/** @brief Get radar timestamp. */
uint64_t ros_radar_cube_get_timestamp(const ros_radar_cube_t* view);

/**
 * @brief Get layout data (borrowed).
 * @param view RadarCube handle
 * @param out_len Receives byte count
 * @return Pointer to layout data or NULL
 */
const uint8_t* ros_radar_cube_get_layout(const ros_radar_cube_t* view, size_t* out_len);

/**
 * @brief Get raw cube data (borrowed).
 * @param view RadarCube handle
 * @param out_len Receives byte count
 * @return Pointer to raw cube data or NULL
 */
const uint8_t* ros_radar_cube_get_cube_raw(const ros_radar_cube_t* view, size_t* out_len);

/** @brief Get cube length. */
uint32_t ros_radar_cube_get_cube_len(const ros_radar_cube_t* view);

/** @brief Get is_complex flag. */
bool ros_radar_cube_get_is_complex(const ros_radar_cube_t* view);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_radar_cube_as_cdr(const ros_radar_cube_t* view, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - RadarInfo (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a RadarInfo view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_radar_info_t* ros_radar_info_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a RadarInfo view handle. */
void ros_radar_info_free(ros_radar_info_t* view);

/** @brief Get stamp seconds. */
int32_t ros_radar_info_get_stamp_sec(const ros_radar_info_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_radar_info_get_stamp_nanosec(const ros_radar_info_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_radar_info_get_frame_id(const ros_radar_info_t* view);

/** @brief Get center frequency string (borrowed). */
const char* ros_radar_info_get_center_frequency(const ros_radar_info_t* view);

/** @brief Get frequency sweep string (borrowed). */
const char* ros_radar_info_get_frequency_sweep(const ros_radar_info_t* view);

/** @brief Get range toggle string (borrowed). */
const char* ros_radar_info_get_range_toggle(const ros_radar_info_t* view);

/** @brief Get detection sensitivity string (borrowed). */
const char* ros_radar_info_get_detection_sensitivity(const ros_radar_info_t* view);

/** @brief Get cube enabled flag. */
bool ros_radar_info_get_cube(const ros_radar_info_t* view);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_radar_info_as_cdr(const ros_radar_info_t* view, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - Detect (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a Detect view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_detect_t* ros_detect_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a Detect view handle. */
void ros_detect_free(ros_detect_t* view);

/** @brief Get stamp seconds. */
int32_t ros_detect_get_stamp_sec(const ros_detect_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_detect_get_stamp_nanosec(const ros_detect_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_detect_get_frame_id(const ros_detect_t* view);

/** @brief Get number of detection boxes. */
uint32_t ros_detect_get_boxes_len(const ros_detect_t* view);

/**
 * @brief Get a borrowed view of the i-th detection box.
 * @param view Detect handle
 * @param index Zero-based box index (must be < ros_detect_get_boxes_len(view))
 * @return Borrowed ros_box_t* whose lifetime is tied to the parent Detect handle,
 *         or NULL on error (errno set to EINVAL).
 *
 * @warning Do NOT pass the returned pointer to ros_box_free(). The parent handle
 *          owns the child's storage. The pointer becomes invalid when the parent
 *          is freed via ros_detect_free().
 */
const ros_box_t* ros_detect_get_box(const ros_detect_t* view, uint32_t index);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_detect_as_cdr(const ros_detect_t* view, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - Model (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a Model view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_model_t* ros_model_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a Model view handle. */
void ros_model_free(ros_model_t* view);

/** @brief Get stamp seconds. */
int32_t ros_model_get_stamp_sec(const ros_model_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_model_get_stamp_nanosec(const ros_model_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_model_get_frame_id(const ros_model_t* view);

/** @brief Get number of detection boxes. */
uint32_t ros_model_get_boxes_len(const ros_model_t* view);

/** @brief Get number of masks. */
uint32_t ros_model_get_masks_len(const ros_model_t* view);

/**
 * @brief Get a borrowed view of the i-th model box.
 * @param view Model handle
 * @param index Zero-based box index (must be < ros_model_get_boxes_len(view))
 * @return Borrowed ros_box_t* whose lifetime is tied to the parent Model handle,
 *         or NULL on error (errno set to EINVAL).
 *
 * @warning Do NOT pass the returned pointer to ros_box_free(). The parent handle
 *          owns the child's storage. The pointer becomes invalid when the parent
 *          is freed via ros_model_free().
 */
const ros_box_t* ros_model_get_box(const ros_model_t* view, uint32_t index);

/**
 * @brief Get a borrowed view of the i-th model mask.
 * @param view Model handle
 * @param index Zero-based mask index (must be < ros_model_get_masks_len(view))
 * @return Borrowed ros_mask_t* whose lifetime is tied to the parent Model handle,
 *         or NULL on error (errno set to EINVAL).
 *
 * @warning Do NOT pass the returned pointer to ros_mask_free(). The parent handle
 *          owns the child's storage. The pointer becomes invalid when the parent
 *          is freed via ros_model_free().
 */
const ros_mask_t* ros_model_get_mask(const ros_model_t* view, uint32_t index);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_model_as_cdr(const ros_model_t* view, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - ModelInfo (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a ModelInfo view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_model_info_t* ros_model_info_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a ModelInfo view handle. */
void ros_model_info_free(ros_model_info_t* view);

/** @brief Get stamp seconds. */
int32_t ros_model_info_get_stamp_sec(const ros_model_info_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_model_info_get_stamp_nanosec(const ros_model_info_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_model_info_get_frame_id(const ros_model_info_t* view);

/** @brief Get model type string (borrowed). */
const char* ros_model_info_get_model_type(const ros_model_info_t* view);

/** @brief Get model format string (borrowed). */
const char* ros_model_info_get_model_format(const ros_model_info_t* view);

/** @brief Get model name string (borrowed). */
const char* ros_model_info_get_model_name(const ros_model_info_t* view);

/** @brief Get input type. */
uint8_t ros_model_info_get_input_type(const ros_model_info_t* view);

/** @brief Get output type. */
uint8_t ros_model_info_get_output_type(const ros_model_info_t* view);

/** @brief Get input shape array (zero-copy pointer into CDR buffer). */
const uint32_t* ros_model_info_get_input_shape(const ros_model_info_t* view, size_t* out_len);

/** @brief Get output shape array (zero-copy pointer into CDR buffer). */
const uint32_t* ros_model_info_get_output_shape(const ros_model_info_t* view, size_t* out_len);

/** @brief Get number of labels. */
uint32_t ros_model_info_get_labels_len(const ros_model_info_t* view);

/** @brief Get label at index (zero-copy pointer into CDR buffer). Sets errno EINVAL if out of bounds. */
const char* ros_model_info_get_label(const ros_model_info_t* view, uint32_t index);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_model_info_as_cdr(const ros_model_info_t* view, size_t* out_len);

/* ============================================================================
 * sensor_msgs - PointCloud2 (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a PointCloud2 view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_point_cloud2_t* ros_point_cloud2_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a PointCloud2 view handle. */
void ros_point_cloud2_free(ros_point_cloud2_t* view);

/** @brief Get stamp seconds. */
int32_t ros_point_cloud2_get_stamp_sec(const ros_point_cloud2_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_point_cloud2_get_stamp_nanosec(const ros_point_cloud2_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_point_cloud2_get_frame_id(const ros_point_cloud2_t* view);

/** @brief Get cloud height (number of rows). */
uint32_t ros_point_cloud2_get_height(const ros_point_cloud2_t* view);

/** @brief Get cloud width (number of points per row). */
uint32_t ros_point_cloud2_get_width(const ros_point_cloud2_t* view);

/** @brief Get point step (bytes per point). */
uint32_t ros_point_cloud2_get_point_step(const ros_point_cloud2_t* view);

/** @brief Get row step (bytes per row). */
uint32_t ros_point_cloud2_get_row_step(const ros_point_cloud2_t* view);

/**
 * @brief Get point cloud data (borrowed).
 * @param view PointCloud2 handle
 * @param out_len Receives byte count
 * @return Pointer to point data or NULL
 */
const uint8_t* ros_point_cloud2_get_data(const ros_point_cloud2_t* view, size_t* out_len);

/** @brief Get is_dense flag (true if no invalid points). */
bool ros_point_cloud2_get_is_dense(const ros_point_cloud2_t* view);

/** @brief Get is_bigendian flag. */
bool ros_point_cloud2_get_is_bigendian(const ros_point_cloud2_t* view);

/** @brief Get number of point fields. */
uint32_t ros_point_cloud2_get_fields_len(const ros_point_cloud2_t* view);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_point_cloud2_as_cdr(const ros_point_cloud2_t* view, size_t* out_len);

/* ----------------------------------------------------------------------------
 * sensor_msgs - PointField (builder, 3.2.0+)
 * --------------------------------------------------------------------------*/

/** @brief Opaque builder handle for standalone PointField messages. */
typedef struct ros_point_field_builder_s ros_point_field_builder_t;

/**
 * @brief Create a new PointField builder with zero-valued defaults.
 * @return Opaque handle, or NULL on allocation failure. Free with
 *         ros_point_field_builder_free.
 */
ros_point_field_builder_t* ros_point_field_builder_new(void);

/** @brief Free a PointField builder handle. NULL-safe. */
void ros_point_field_builder_free(ros_point_field_builder_t* b);

/**
 * @brief Set the name field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_point_field_builder_set_name(ros_point_field_builder_t* b,
                                      const char* s);

/** @brief Set the byte offset of this field within a point. */
void ros_point_field_builder_set_offset(ros_point_field_builder_t* b,
                                        uint32_t v);

/** @brief Set the datatype code (INT8=1, UINT8=2, INT16=3, ..., FLOAT32=7, FLOAT64=8). */
void ros_point_field_builder_set_datatype(ros_point_field_builder_t* b,
                                          uint8_t v);

/** @brief Set the number of elements of this datatype per point. */
void ros_point_field_builder_set_count(ros_point_field_builder_t* b,
                                       uint32_t v);

/**
 * @brief Allocate a fresh CDR buffer and encode the message.
 * @return 0 on success (out_bytes/out_len written; free via ros_bytes_free),
 *         -1 on error (errno set: EINVAL for NULL handle, EBADMSG on encoder error).
 */
int  ros_point_field_builder_build(ros_point_field_builder_t* b,
                                   uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Encode the message into a caller-owned buffer.
 * @return 0 on success (out_len written), -1 on error (errno: EINVAL for
 *         NULL args, ENOBUFS for buffer too small, EBADMSG for encoding error).
 */
int  ros_point_field_builder_encode_into(ros_point_field_builder_t* b,
                                         uint8_t* buf, size_t cap,
                                         size_t* out_len);

/* ----------------------------------------------------------------------------
 * sensor_msgs - PointCloud2 (builder, 3.2.0+)
 * --------------------------------------------------------------------------*/

/**
 * @brief C-POD descriptor for one element of a PointCloud2 `fields` sequence.
 *
 * The `name` pointer is BORROWED: the caller must keep the backing
 * NUL-terminated string alive (and the enclosing array) until the next
 * `ros_point_cloud2_builder_set_fields` call on the builder, the next
 * `build`/`encode_into`, or `free`.
 */
typedef struct ros_point_field_elem_s {
    const char* name;
    uint32_t    offset;
    uint8_t     datatype;
    uint32_t    count;
} ros_point_field_elem_t;

/** @brief Opaque builder handle for ros_point_cloud2_t messages. */
typedef struct ros_point_cloud2_builder_s ros_point_cloud2_builder_t;

/**
 * @brief Create a new PointCloud2 builder with zero-valued defaults.
 * @return Opaque handle, or NULL on allocation failure. Free with
 *         ros_point_cloud2_builder_free.
 */
ros_point_cloud2_builder_t* ros_point_cloud2_builder_new(void);

/** @brief Free a PointCloud2 builder handle. NULL-safe. */
void ros_point_cloud2_builder_free(ros_point_cloud2_builder_t* b);

/** @brief Set the stamp field. */
void ros_point_cloud2_builder_set_stamp(ros_point_cloud2_builder_t* b,
                                        int32_t sec, uint32_t nsec);

/**
 * @brief Set the frame_id field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_point_cloud2_builder_set_frame_id(ros_point_cloud2_builder_t* b,
                                           const char* s);

/** @brief Set the cloud height (number of rows). */
void ros_point_cloud2_builder_set_height(ros_point_cloud2_builder_t* b,
                                         uint32_t v);

/** @brief Set the cloud width (number of points per row). */
void ros_point_cloud2_builder_set_width(ros_point_cloud2_builder_t* b,
                                        uint32_t v);

/**
 * @brief Set the field descriptor array (BORROWED — both the array and
 *        each element's `name` pointer must remain valid until the next
 *        setter on this slot, build, encode_into, or free call).
 * @param fields Array of `count` ros_point_field_elem_t descriptors
 * @param count  Number of descriptors
 */
void ros_point_cloud2_builder_set_fields(
    ros_point_cloud2_builder_t* b,
    const ros_point_field_elem_t* fields, size_t count);

/** @brief Set the is_bigendian flag. */
void ros_point_cloud2_builder_set_is_bigendian(
    ros_point_cloud2_builder_t* b, bool v);

/** @brief Set the point_step (bytes per point). */
void ros_point_cloud2_builder_set_point_step(ros_point_cloud2_builder_t* b,
                                             uint32_t v);

/** @brief Set the row_step (bytes per row). */
void ros_point_cloud2_builder_set_row_step(ros_point_cloud2_builder_t* b,
                                           uint32_t v);

/**
 * @brief Set the data bulk byte sequence (BORROWED — must remain valid
 *        until the next setter, build, encode_into, or free call).
 */
void ros_point_cloud2_builder_set_data(ros_point_cloud2_builder_t* b,
                                       const uint8_t* data, size_t len);

/** @brief Set the is_dense flag (true iff no invalid points). */
void ros_point_cloud2_builder_set_is_dense(ros_point_cloud2_builder_t* b,
                                           bool v);

/**
 * @brief Allocate a fresh CDR buffer and encode the message.
 * @return 0 on success (out_bytes/out_len written; free via ros_bytes_free),
 *         -1 on error (errno set: EINVAL for NULL handle, EBADMSG on encoder error).
 */
int  ros_point_cloud2_builder_build(ros_point_cloud2_builder_t* b,
                                    uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Encode the message into a caller-owned buffer.
 * @return 0 on success (out_len written), -1 on error (errno: EINVAL for
 *         NULL args, ENOBUFS for buffer too small, EBADMSG for encoding error).
 */
int  ros_point_cloud2_builder_encode_into(ros_point_cloud2_builder_t* b,
                                          uint8_t* buf, size_t cap,
                                          size_t* out_len);

/* ============================================================================
 * sensor_msgs - CameraInfo (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a CameraInfo view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_camera_info_t* ros_camera_info_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a CameraInfo view handle. */
void ros_camera_info_free(ros_camera_info_t* view);

/** @brief Get stamp seconds. */
int32_t ros_camera_info_get_stamp_sec(const ros_camera_info_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_camera_info_get_stamp_nanosec(const ros_camera_info_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_camera_info_get_frame_id(const ros_camera_info_t* view);

/** @brief Get image height. */
uint32_t ros_camera_info_get_height(const ros_camera_info_t* view);

/** @brief Get image width. */
uint32_t ros_camera_info_get_width(const ros_camera_info_t* view);

/** @brief Get distortion model string (borrowed). */
const char* ros_camera_info_get_distortion_model(const ros_camera_info_t* view);

/** @brief Get horizontal binning factor. */
uint32_t ros_camera_info_get_binning_x(const ros_camera_info_t* view);

/** @brief Get vertical binning factor. */
uint32_t ros_camera_info_get_binning_y(const ros_camera_info_t* view);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_camera_info_as_cdr(const ros_camera_info_t* view, size_t* out_len);

/* ----------------------------------------------------------------------------
 * sensor_msgs - CameraInfo (builder, 3.2.0+)
 * --------------------------------------------------------------------------*/

/** @brief Opaque builder handle for ros_camera_info_t messages. */
typedef struct ros_camera_info_builder_s ros_camera_info_builder_t;

/**
 * @brief Create a new CameraInfo builder with zero-valued defaults.
 * @return Opaque handle, or NULL on allocation failure. Free with
 *         ros_camera_info_builder_free.
 */
ros_camera_info_builder_t* ros_camera_info_builder_new(void);

/** @brief Free a CameraInfo builder handle. NULL-safe. */
void ros_camera_info_builder_free(ros_camera_info_builder_t* b);

/** @brief Set the stamp field. */
void ros_camera_info_builder_set_stamp(ros_camera_info_builder_t* b,
                                       int32_t sec, uint32_t nsec);

/**
 * @brief Set the frame_id field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_camera_info_builder_set_frame_id(ros_camera_info_builder_t* b,
                                          const char* s);

/** @brief Set the image height in pixels. */
void ros_camera_info_builder_set_height(ros_camera_info_builder_t* b,
                                        uint32_t v);

/** @brief Set the image width in pixels. */
void ros_camera_info_builder_set_width(ros_camera_info_builder_t* b,
                                       uint32_t v);

/**
 * @brief Set the distortion model string (copied), e.g. "plumb_bob".
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_camera_info_builder_set_distortion_model(
    ros_camera_info_builder_t* b, const char* s);

/**
 * @brief Set the distortion coefficients `d` (BORROWED `double[len]` — the
 *        pointer must remain valid until the next setter on this slot,
 *        build, encode_into, or free call).
 */
void ros_camera_info_builder_set_d(ros_camera_info_builder_t* b,
                                   const double* data, size_t len);

/** @brief Copy 9 f64 intrinsics matrix elements (row-major 3x3). */
void ros_camera_info_builder_set_k(ros_camera_info_builder_t* b,
                                   const double* k);

/** @brief Copy 9 f64 rectification matrix elements (row-major 3x3). */
void ros_camera_info_builder_set_r(ros_camera_info_builder_t* b,
                                   const double* r);

/** @brief Copy 12 f64 projection matrix elements (row-major 3x4). */
void ros_camera_info_builder_set_p(ros_camera_info_builder_t* b,
                                   const double* p);

/** @brief Set the horizontal binning factor. */
void ros_camera_info_builder_set_binning_x(ros_camera_info_builder_t* b,
                                           uint32_t v);

/** @brief Set the vertical binning factor. */
void ros_camera_info_builder_set_binning_y(ros_camera_info_builder_t* b,
                                           uint32_t v);

/**
 * @brief Set the RegionOfInterest (ROI) via its primitive fields.
 * @param do_rectify Non-zero = true, zero = false.
 */
void ros_camera_info_builder_set_roi(ros_camera_info_builder_t* b,
                                     uint32_t x_offset, uint32_t y_offset,
                                     uint32_t height, uint32_t width,
                                     uint8_t do_rectify);

/**
 * @brief Allocate a fresh CDR buffer and encode the message.
 * @return 0 on success (out_bytes/out_len written; free via ros_bytes_free),
 *         -1 on error (errno set: EINVAL for NULL handle, EBADMSG on encoder error).
 */
int  ros_camera_info_builder_build(ros_camera_info_builder_t* b,
                                   uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Encode the message into a caller-owned buffer.
 * @return 0 on success (out_len written), -1 on error (errno: EINVAL for
 *         NULL args, ENOBUFS for buffer too small, EBADMSG for encoding error).
 */
int  ros_camera_info_builder_encode_into(ros_camera_info_builder_t* b,
                                         uint8_t* buf, size_t cap,
                                         size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - Track (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a Track view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_track_t* ros_track_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a Track view handle. */
void ros_track_free(ros_track_t* view);

/** @brief Get track ID string (borrowed). */
const char* ros_track_get_id(const ros_track_t* view);

/** @brief Get track lifetime. */
int32_t ros_track_get_lifetime(const ros_track_t* view);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_track_as_cdr(const ros_track_t* view, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - DetectBox (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a DetectBox view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_box_t* ros_box_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a DetectBox view handle. */
void ros_box_free(ros_box_t* view);

/** @brief Get bounding box center X. */
float ros_box_get_center_x(const ros_box_t* view);

/** @brief Get bounding box center Y. */
float ros_box_get_center_y(const ros_box_t* view);

/** @brief Get bounding box width. */
float ros_box_get_width(const ros_box_t* view);

/** @brief Get bounding box height. */
float ros_box_get_height(const ros_box_t* view);

/** @brief Get label string (borrowed). */
const char* ros_box_get_label(const ros_box_t* view);

/** @brief Get detection score/confidence. */
float ros_box_get_score(const ros_box_t* view);

/** @brief Get distance. */
float ros_box_get_distance(const ros_box_t* view);

/** @brief Get speed. */
float ros_box_get_speed(const ros_box_t* view);

/** @brief Get track ID string (borrowed). */
const char* ros_box_get_track_id(const ros_box_t* view);

/** @brief Get track lifetime. */
int32_t ros_box_get_track_lifetime(const ros_box_t* view);

/** @brief Get the box's track_created timestamp seconds component. */
int32_t ros_box_get_track_created_sec(const ros_box_t* view);

/** @brief Get the box's track_created timestamp nanoseconds component. */
uint32_t ros_box_get_track_created_nanosec(const ros_box_t* view);

/* ============================================================================
 * edgefirst_msgs - LocalTime (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a LocalTime view from CDR bytes.
 * @param data CDR encoded bytes (borrowed; must outlive the returned handle)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_local_time_t* ros_local_time_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a LocalTime view handle. */
void ros_local_time_free(ros_local_time_t* view);

/** @brief Get stamp seconds. */
int32_t ros_local_time_get_stamp_sec(const ros_local_time_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_local_time_get_stamp_nanosec(const ros_local_time_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_local_time_get_frame_id(const ros_local_time_t* view);

/** @brief Get timezone offset in minutes from UTC. */
int16_t ros_local_time_get_timezone(const ros_local_time_t* view);

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_local_time_as_cdr(const ros_local_time_t* view, size_t* out_len);

/* =========================================================================
 * geometry_msgs/PoseWithCovariance  (CdrFixed)
 * Pose(56 B) + float64[36] covariance = 344 bytes on the wire.
 * =========================================================================
 */
int32_t ros_pose_with_covariance_encode(
    uint8_t* buf, size_t cap, size_t* written,
    double px, double py, double pz,
    double ox, double oy, double oz, double ow,
    const double* covariance);

int32_t ros_pose_with_covariance_decode(
    const uint8_t* data, size_t len,
    double* px, double* py, double* pz,
    double* ox, double* oy, double* oz, double* ow,
    double* covariance_out);

/* =========================================================================
 * geometry_msgs/TwistWithCovariance  (CdrFixed)
 * Twist(48 B) + float64[36] covariance = 336 bytes on the wire.
 * =========================================================================
 */
int32_t ros_twist_with_covariance_encode(
    uint8_t* buf, size_t cap, size_t* written,
    double lx, double ly, double lz,
    double ax, double ay, double az,
    const double* covariance);

int32_t ros_twist_with_covariance_decode(
    const uint8_t* data, size_t len,
    double* lx, double* ly, double* lz,
    double* ax, double* ay, double* az,
    double* covariance_out);

/* =========================================================================
 * sensor_msgs/MagneticField  (buffer-backed, decode-only)
 * =========================================================================
 */
typedef struct ros_magnetic_field_t ros_magnetic_field_t;

ros_magnetic_field_t* ros_magnetic_field_from_cdr(const uint8_t* data, size_t len);
void ros_magnetic_field_free(ros_magnetic_field_t* view);
int32_t ros_magnetic_field_get_stamp_sec(const ros_magnetic_field_t* view);
uint32_t ros_magnetic_field_get_stamp_nanosec(const ros_magnetic_field_t* view);
const char* ros_magnetic_field_get_frame_id(const ros_magnetic_field_t* view);
void ros_magnetic_field_get_magnetic_field(const ros_magnetic_field_t* view,
                                           double* x, double* y, double* z);
void ros_magnetic_field_get_magnetic_field_covariance(
    const ros_magnetic_field_t* view, double* out);
/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_magnetic_field_as_cdr(const ros_magnetic_field_t* view, size_t* out_len);

/* ----------------------------------------------------------------------------
 * sensor_msgs - MagneticField (builder, 3.2.0+)
 * --------------------------------------------------------------------------*/

/** @brief Opaque builder handle for ros_magnetic_field_t messages. */
typedef struct ros_magnetic_field_builder_s ros_magnetic_field_builder_t;

/**
 * @brief Create a new MagneticField builder with zero-valued defaults.
 * @return Opaque handle, or NULL on allocation failure. Free with
 *         ros_magnetic_field_builder_free.
 */
ros_magnetic_field_builder_t* ros_magnetic_field_builder_new(void);

/** @brief Free a MagneticField builder handle. NULL-safe. */
void ros_magnetic_field_builder_free(ros_magnetic_field_builder_t* b);

/** @brief Set the stamp field. */
void ros_magnetic_field_builder_set_stamp(ros_magnetic_field_builder_t* b,
                                          int32_t sec, uint32_t nsec);

/**
 * @brief Set the frame_id field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_magnetic_field_builder_set_frame_id(
    ros_magnetic_field_builder_t* b, const char* s);

/** @brief Set the magnetic_field vector (Tesla, x, y, z). */
void ros_magnetic_field_builder_set_magnetic_field(
    ros_magnetic_field_builder_t* b, double x, double y, double z);

/** @brief Copy 9 f64 covariance values (row-major 3x3). */
void ros_magnetic_field_builder_set_magnetic_field_covariance(
    ros_magnetic_field_builder_t* b, const double* cov);

/**
 * @brief Allocate a fresh CDR buffer and encode the message.
 * @return 0 on success (out_bytes/out_len written; free via ros_bytes_free),
 *         -1 on error (errno set: EINVAL for NULL handle, EBADMSG on encoder error).
 */
int  ros_magnetic_field_builder_build(ros_magnetic_field_builder_t* b,
                                      uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Encode the message into a caller-owned buffer.
 * @return 0 on success (out_len written), -1 on error (errno: EINVAL for
 *         NULL args, ENOBUFS for buffer too small, EBADMSG for encoding error).
 */
int  ros_magnetic_field_builder_encode_into(
    ros_magnetic_field_builder_t* b, uint8_t* buf, size_t cap,
    size_t* out_len);

/* =========================================================================
 * sensor_msgs/FluidPressure  (buffer-backed, decode-only)
 * =========================================================================
 */
typedef struct ros_fluid_pressure_t ros_fluid_pressure_t;

ros_fluid_pressure_t* ros_fluid_pressure_from_cdr(const uint8_t* data, size_t len);
void ros_fluid_pressure_free(ros_fluid_pressure_t* view);
int32_t ros_fluid_pressure_get_stamp_sec(const ros_fluid_pressure_t* view);
uint32_t ros_fluid_pressure_get_stamp_nanosec(const ros_fluid_pressure_t* view);
const char* ros_fluid_pressure_get_frame_id(const ros_fluid_pressure_t* view);
double ros_fluid_pressure_get_fluid_pressure(const ros_fluid_pressure_t* view);
double ros_fluid_pressure_get_variance(const ros_fluid_pressure_t* view);
const uint8_t* ros_fluid_pressure_as_cdr(const ros_fluid_pressure_t* view, size_t* out_len);

/* ----------------------------------------------------------------------------
 * sensor_msgs - FluidPressure (builder, 3.2.0+)
 * --------------------------------------------------------------------------*/

/** @brief Opaque builder handle for ros_fluid_pressure_t messages. */
typedef struct ros_fluid_pressure_builder_s ros_fluid_pressure_builder_t;

/**
 * @brief Create a new FluidPressure builder with zero-valued defaults.
 * @return Opaque handle, or NULL on allocation failure. Free with
 *         ros_fluid_pressure_builder_free.
 */
ros_fluid_pressure_builder_t* ros_fluid_pressure_builder_new(void);

/** @brief Free a FluidPressure builder handle. NULL-safe. */
void ros_fluid_pressure_builder_free(ros_fluid_pressure_builder_t* b);

/** @brief Set the stamp field. */
void ros_fluid_pressure_builder_set_stamp(ros_fluid_pressure_builder_t* b,
                                          int32_t sec, uint32_t nsec);

/**
 * @brief Set the frame_id field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_fluid_pressure_builder_set_frame_id(ros_fluid_pressure_builder_t* b,
                                             const char* s);

/** @brief Set the fluid_pressure field (Pascals). */
void ros_fluid_pressure_builder_set_fluid_pressure(
    ros_fluid_pressure_builder_t* b, double v);

/** @brief Set the variance field (0 indicates unknown). */
void ros_fluid_pressure_builder_set_variance(ros_fluid_pressure_builder_t* b,
                                             double v);

/**
 * @brief Allocate a fresh CDR buffer and encode the message.
 * @return 0 on success (out_bytes/out_len written; free via ros_bytes_free),
 *         -1 on error (errno set: EINVAL for NULL handle, EBADMSG on encoder error).
 */
int  ros_fluid_pressure_builder_build(ros_fluid_pressure_builder_t* b,
                                      uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Encode the message into a caller-owned buffer.
 * @return 0 on success (out_len written), -1 on error (errno: EINVAL for
 *         NULL args, ENOBUFS for buffer too small, EBADMSG for encoding error).
 */
int  ros_fluid_pressure_builder_encode_into(ros_fluid_pressure_builder_t* b,
                                            uint8_t* buf, size_t cap,
                                            size_t* out_len);

/* =========================================================================
 * sensor_msgs/Temperature  (buffer-backed, decode-only)
 * =========================================================================
 */
typedef struct ros_temperature_t ros_temperature_t;

ros_temperature_t* ros_temperature_from_cdr(const uint8_t* data, size_t len);
void ros_temperature_free(ros_temperature_t* view);
int32_t ros_temperature_get_stamp_sec(const ros_temperature_t* view);
uint32_t ros_temperature_get_stamp_nanosec(const ros_temperature_t* view);
const char* ros_temperature_get_frame_id(const ros_temperature_t* view);
double ros_temperature_get_temperature(const ros_temperature_t* view);
double ros_temperature_get_variance(const ros_temperature_t* view);
const uint8_t* ros_temperature_as_cdr(const ros_temperature_t* view, size_t* out_len);

/* ----------------------------------------------------------------------------
 * sensor_msgs - Temperature (builder, 3.2.0+)
 * --------------------------------------------------------------------------*/

/** @brief Opaque builder handle for ros_temperature_t messages. */
typedef struct ros_temperature_builder_s ros_temperature_builder_t;

/**
 * @brief Create a new Temperature builder with zero-valued defaults.
 * @return Opaque handle, or NULL on allocation failure. Free with
 *         ros_temperature_builder_free.
 */
ros_temperature_builder_t* ros_temperature_builder_new(void);

/** @brief Free a Temperature builder handle. NULL-safe. */
void ros_temperature_builder_free(ros_temperature_builder_t* b);

/** @brief Set the stamp field. */
void ros_temperature_builder_set_stamp(ros_temperature_builder_t* b,
                                       int32_t sec, uint32_t nsec);

/**
 * @brief Set the frame_id field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_temperature_builder_set_frame_id(ros_temperature_builder_t* b,
                                          const char* s);

/** @brief Set the temperature field (Celsius). */
void ros_temperature_builder_set_temperature(ros_temperature_builder_t* b,
                                             double v);

/** @brief Set the variance field (0 indicates unknown). */
void ros_temperature_builder_set_variance(ros_temperature_builder_t* b,
                                          double v);

/**
 * @brief Allocate a fresh CDR buffer and encode the message.
 * @return 0 on success (out_bytes/out_len written; free via ros_bytes_free),
 *         -1 on error (errno set: EINVAL for NULL handle, EBADMSG on encoder error).
 */
int  ros_temperature_builder_build(ros_temperature_builder_t* b,
                                   uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Encode the message into a caller-owned buffer.
 * @return 0 on success (out_len written), -1 on error (errno: EINVAL for
 *         NULL args, ENOBUFS for buffer too small, EBADMSG for encoding error).
 */
int  ros_temperature_builder_encode_into(ros_temperature_builder_t* b,
                                         uint8_t* buf, size_t cap,
                                         size_t* out_len);

/* =========================================================================
 * sensor_msgs/BatteryState  (buffer-backed, decode-only)
 * =========================================================================
 */
/** power_supply_status values. */
#define ROS_BATTERY_STATE_POWER_SUPPLY_STATUS_UNKNOWN      0
#define ROS_BATTERY_STATE_POWER_SUPPLY_STATUS_CHARGING     1
#define ROS_BATTERY_STATE_POWER_SUPPLY_STATUS_DISCHARGING  2
#define ROS_BATTERY_STATE_POWER_SUPPLY_STATUS_NOT_CHARGING 3
#define ROS_BATTERY_STATE_POWER_SUPPLY_STATUS_FULL         4

/** power_supply_health values. */
#define ROS_BATTERY_STATE_POWER_SUPPLY_HEALTH_UNKNOWN               0
#define ROS_BATTERY_STATE_POWER_SUPPLY_HEALTH_GOOD                  1
#define ROS_BATTERY_STATE_POWER_SUPPLY_HEALTH_OVERHEAT              2
#define ROS_BATTERY_STATE_POWER_SUPPLY_HEALTH_DEAD                  3
#define ROS_BATTERY_STATE_POWER_SUPPLY_HEALTH_OVERVOLTAGE           4
#define ROS_BATTERY_STATE_POWER_SUPPLY_HEALTH_UNSPEC_FAILURE        5
#define ROS_BATTERY_STATE_POWER_SUPPLY_HEALTH_COLD                  6
#define ROS_BATTERY_STATE_POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE 7
#define ROS_BATTERY_STATE_POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE   8

/** power_supply_technology values. */
#define ROS_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_UNKNOWN 0
#define ROS_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_NIMH    1
#define ROS_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LION    2
#define ROS_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LIPO    3
#define ROS_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LIFE    4
#define ROS_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_NICD    5
#define ROS_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_LIMN    6

typedef struct ros_battery_state_t ros_battery_state_t;

ros_battery_state_t* ros_battery_state_from_cdr(const uint8_t* data, size_t len);
void ros_battery_state_free(ros_battery_state_t* view);
int32_t ros_battery_state_get_stamp_sec(const ros_battery_state_t* view);
uint32_t ros_battery_state_get_stamp_nanosec(const ros_battery_state_t* view);
const char* ros_battery_state_get_frame_id(const ros_battery_state_t* view);
float ros_battery_state_get_voltage(const ros_battery_state_t* view);
float ros_battery_state_get_temperature(const ros_battery_state_t* view);
float ros_battery_state_get_current(const ros_battery_state_t* view);
float ros_battery_state_get_charge(const ros_battery_state_t* view);
float ros_battery_state_get_capacity(const ros_battery_state_t* view);
float ros_battery_state_get_design_capacity(const ros_battery_state_t* view);
float ros_battery_state_get_percentage(const ros_battery_state_t* view);
uint8_t ros_battery_state_get_power_supply_status(const ros_battery_state_t* view);
uint8_t ros_battery_state_get_power_supply_health(const ros_battery_state_t* view);
uint8_t ros_battery_state_get_power_supply_technology(const ros_battery_state_t* view);
bool ros_battery_state_get_present(const ros_battery_state_t* view);
uint32_t ros_battery_state_get_cell_voltage_len(const ros_battery_state_t* view);
/** Copy up to `cap` cell voltages into `out`; returns total element count. */
uint32_t ros_battery_state_get_cell_voltage(const ros_battery_state_t* view,
                                            float* out, size_t cap);
uint32_t ros_battery_state_get_cell_temperature_len(const ros_battery_state_t* view);
uint32_t ros_battery_state_get_cell_temperature(const ros_battery_state_t* view,
                                                float* out, size_t cap);
const char* ros_battery_state_get_location(const ros_battery_state_t* view);
const char* ros_battery_state_get_serial_number(const ros_battery_state_t* view);
const uint8_t* ros_battery_state_as_cdr(const ros_battery_state_t* view, size_t* out_len);

/* ----------------------------------------------------------------------------
 * sensor_msgs - BatteryState (builder, 3.2.0+)
 * --------------------------------------------------------------------------*/

/** @brief Opaque builder handle for ros_battery_state_t messages. */
typedef struct ros_battery_state_builder_s ros_battery_state_builder_t;

/**
 * @brief Create a new BatteryState builder with zero-valued defaults.
 * @return Opaque handle, or NULL on allocation failure. Free with
 *         ros_battery_state_builder_free.
 */
ros_battery_state_builder_t* ros_battery_state_builder_new(void);

/** @brief Free a BatteryState builder handle. NULL-safe. */
void ros_battery_state_builder_free(ros_battery_state_builder_t* b);

/** @brief Set the stamp field. */
void ros_battery_state_builder_set_stamp(ros_battery_state_builder_t* b,
                                         int32_t sec, uint32_t nsec);

/**
 * @brief Set the frame_id field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_battery_state_builder_set_frame_id(
    ros_battery_state_builder_t* b, const char* s);

/** @brief Set the pack voltage (volts). */
void ros_battery_state_builder_set_voltage(ros_battery_state_builder_t* b, float v);
/** @brief Set the pack temperature (Celsius). */
void ros_battery_state_builder_set_temperature(ros_battery_state_builder_t* b, float v);
/** @brief Set the instantaneous current (amps; negative = discharging). */
void ros_battery_state_builder_set_current(ros_battery_state_builder_t* b, float v);
/** @brief Set the current charge (amp-hours). */
void ros_battery_state_builder_set_charge(ros_battery_state_builder_t* b, float v);
/** @brief Set the pack capacity (amp-hours). */
void ros_battery_state_builder_set_capacity(ros_battery_state_builder_t* b, float v);
/** @brief Set the design capacity (amp-hours). */
void ros_battery_state_builder_set_design_capacity(ros_battery_state_builder_t* b, float v);
/** @brief Set the charge percentage (0.0 to 1.0). */
void ros_battery_state_builder_set_percentage(ros_battery_state_builder_t* b, float v);
/** @brief Set the power_supply_status enum (see ROS_BATTERY_STATE_POWER_SUPPLY_STATUS_*). */
void ros_battery_state_builder_set_power_supply_status(
    ros_battery_state_builder_t* b, uint8_t v);
/** @brief Set the power_supply_health enum (see ROS_BATTERY_STATE_POWER_SUPPLY_HEALTH_*). */
void ros_battery_state_builder_set_power_supply_health(
    ros_battery_state_builder_t* b, uint8_t v);
/** @brief Set the power_supply_technology enum (see ROS_BATTERY_STATE_POWER_SUPPLY_TECHNOLOGY_*). */
void ros_battery_state_builder_set_power_supply_technology(
    ros_battery_state_builder_t* b, uint8_t v);
/** @brief Set the present flag. */
void ros_battery_state_builder_set_present(
    ros_battery_state_builder_t* b, bool v);

/**
 * @brief Set the cell_voltage array (BORROWED `float[len]` — must remain
 *        valid until the next setter on this slot, build, encode_into, or free).
 */
void ros_battery_state_builder_set_cell_voltage(
    ros_battery_state_builder_t* b, const float* data, size_t len);

/**
 * @brief Set the cell_temperature array (BORROWED `float[len]` — must remain
 *        valid until the next setter on this slot, build, encode_into, or free).
 */
void ros_battery_state_builder_set_cell_temperature(
    ros_battery_state_builder_t* b, const float* data, size_t len);

/**
 * @brief Set the location field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_battery_state_builder_set_location(
    ros_battery_state_builder_t* b, const char* s);

/**
 * @brief Set the serial_number field (string is copied into the builder).
 * @return 0 on success, -1 on error (errno: EINVAL for NULL handle).
 */
int  ros_battery_state_builder_set_serial_number(
    ros_battery_state_builder_t* b, const char* s);

/**
 * @brief Allocate a fresh CDR buffer and encode the message.
 * @return 0 on success (out_bytes/out_len written; free via ros_bytes_free),
 *         -1 on error (errno set: EINVAL for NULL handle, EBADMSG on encoder error).
 */
int  ros_battery_state_builder_build(ros_battery_state_builder_t* b,
                                     uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Encode the message into a caller-owned buffer.
 * @return 0 on success (out_len written), -1 on error (errno: EINVAL for
 *         NULL args, ENOBUFS for buffer too small, EBADMSG for encoding error).
 */
int  ros_battery_state_builder_encode_into(
    ros_battery_state_builder_t* b, uint8_t* buf, size_t cap,
    size_t* out_len);

/* =========================================================================
 * nav_msgs/Odometry  (buffer-backed, decode-only)
 * =========================================================================
 */
typedef struct ros_odometry_t ros_odometry_t;

ros_odometry_t* ros_odometry_from_cdr(const uint8_t* data, size_t len);
void ros_odometry_free(ros_odometry_t* view);
int32_t ros_odometry_get_stamp_sec(const ros_odometry_t* view);
uint32_t ros_odometry_get_stamp_nanosec(const ros_odometry_t* view);
const char* ros_odometry_get_frame_id(const ros_odometry_t* view);
const char* ros_odometry_get_child_frame_id(const ros_odometry_t* view);
void ros_odometry_get_pose(const ros_odometry_t* view,
                           double* px, double* py, double* pz,
                           double* ox, double* oy, double* oz, double* ow);
void ros_odometry_get_pose_covariance(const ros_odometry_t* view, double* out);
void ros_odometry_get_twist(const ros_odometry_t* view,
                            double* lx, double* ly, double* lz,
                            double* ax, double* ay, double* az);
void ros_odometry_get_twist_covariance(const ros_odometry_t* view, double* out);
const uint8_t* ros_odometry_as_cdr(const ros_odometry_t* view, size_t* out_len);

/* =========================================================================
 * edgefirst_msgs/Vibration  (buffer-backed, decode-only)
 * =========================================================================
 */
/** measurement_type values. */
#define ROS_VIBRATION_MEASUREMENT_UNKNOWN      0
#define ROS_VIBRATION_MEASUREMENT_RMS          1
#define ROS_VIBRATION_MEASUREMENT_PEAK         2
#define ROS_VIBRATION_MEASUREMENT_PEAK_TO_PEAK 3

/** unit values. */
#define ROS_VIBRATION_UNIT_UNKNOWN           0
#define ROS_VIBRATION_UNIT_ACCEL_M_PER_S2    1
#define ROS_VIBRATION_UNIT_ACCEL_G           2
#define ROS_VIBRATION_UNIT_VELOCITY_MM_PER_S 3
#define ROS_VIBRATION_UNIT_DISPLACEMENT_UM   4
#define ROS_VIBRATION_UNIT_VELOCITY_IN_PER_S 5
#define ROS_VIBRATION_UNIT_DISPLACEMENT_MIL  6

typedef struct ros_vibration_t ros_vibration_t;

ros_vibration_t* ros_vibration_from_cdr(const uint8_t* data, size_t len);
void ros_vibration_free(ros_vibration_t* view);
int32_t ros_vibration_get_stamp_sec(const ros_vibration_t* view);
uint32_t ros_vibration_get_stamp_nanosec(const ros_vibration_t* view);
const char* ros_vibration_get_frame_id(const ros_vibration_t* view);
uint8_t ros_vibration_get_measurement_type(const ros_vibration_t* view);
uint8_t ros_vibration_get_unit(const ros_vibration_t* view);
float ros_vibration_get_band_lower_hz(const ros_vibration_t* view);
float ros_vibration_get_band_upper_hz(const ros_vibration_t* view);
void ros_vibration_get_vibration(const ros_vibration_t* view,
                                 double* x, double* y, double* z);
uint32_t ros_vibration_get_clipping_len(const ros_vibration_t* view);
/** Copy up to `cap` clipping counters into `out`; returns total element count. */
uint32_t ros_vibration_get_clipping(const ros_vibration_t* view,
                                    uint32_t* out, size_t cap);
const uint8_t* ros_vibration_as_cdr(const ros_vibration_t* view, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - Mask (builder, 3.2.0+)
 * ========================================================================= */
typedef struct ros_mask_builder_s ros_mask_builder_t;
ros_mask_builder_t* ros_mask_builder_new(void);
void ros_mask_builder_free(ros_mask_builder_t* b);
void ros_mask_builder_set_height(ros_mask_builder_t* b, uint32_t v);
void ros_mask_builder_set_width(ros_mask_builder_t* b, uint32_t v);
void ros_mask_builder_set_length(ros_mask_builder_t* b, uint32_t v);
int  ros_mask_builder_set_encoding(ros_mask_builder_t* b, const char* s);
/** BORROWED — caller keeps `data` valid until next setter / build / free. */
void ros_mask_builder_set_mask(ros_mask_builder_t* b,
                               const uint8_t* data, size_t len);
void ros_mask_builder_set_boxed(ros_mask_builder_t* b, bool v);
int  ros_mask_builder_build(ros_mask_builder_t* b,
                            uint8_t** out_bytes, size_t* out_len);
int  ros_mask_builder_encode_into(ros_mask_builder_t* b,
                                  uint8_t* buf, size_t cap, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - LocalTime (builder, 3.2.0+)
 * ========================================================================= */
typedef struct ros_local_time_builder_s ros_local_time_builder_t;
ros_local_time_builder_t* ros_local_time_builder_new(void);
void ros_local_time_builder_free(ros_local_time_builder_t* b);
void ros_local_time_builder_set_stamp(ros_local_time_builder_t* b,
                                      int32_t sec, uint32_t nsec);
int  ros_local_time_builder_set_frame_id(ros_local_time_builder_t* b,
                                         const char* s);
void ros_local_time_builder_set_date(ros_local_time_builder_t* b,
                                     uint16_t year, uint8_t month, uint8_t day);
void ros_local_time_builder_set_time(ros_local_time_builder_t* b,
                                     int32_t sec, uint32_t nsec);
void ros_local_time_builder_set_timezone(ros_local_time_builder_t* b, int16_t v);
int  ros_local_time_builder_build(ros_local_time_builder_t* b,
                                  uint8_t** out_bytes, size_t* out_len);
int  ros_local_time_builder_encode_into(ros_local_time_builder_t* b,
                                        uint8_t* buf, size_t cap,
                                        size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - RadarCube (builder, 3.2.0+)
 * ========================================================================= */
typedef struct ros_radar_cube_builder_s ros_radar_cube_builder_t;
ros_radar_cube_builder_t* ros_radar_cube_builder_new(void);
void ros_radar_cube_builder_free(ros_radar_cube_builder_t* b);
void ros_radar_cube_builder_set_stamp(ros_radar_cube_builder_t* b,
                                      int32_t sec, uint32_t nsec);
int  ros_radar_cube_builder_set_frame_id(ros_radar_cube_builder_t* b,
                                         const char* s);
void ros_radar_cube_builder_set_timestamp(ros_radar_cube_builder_t* b, uint64_t v);
/** BORROWED — caller keeps pointer valid until next setter / build / free. */
void ros_radar_cube_builder_set_layout(ros_radar_cube_builder_t* b,
                                       const uint8_t* data, size_t len);
/** BORROWED. */
void ros_radar_cube_builder_set_shape(ros_radar_cube_builder_t* b,
                                      const uint16_t* data, size_t len);
/** BORROWED. */
void ros_radar_cube_builder_set_scales(ros_radar_cube_builder_t* b,
                                       const float* data, size_t len);
/** BORROWED. */
void ros_radar_cube_builder_set_cube(ros_radar_cube_builder_t* b,
                                     const int16_t* data, size_t len);
void ros_radar_cube_builder_set_is_complex(ros_radar_cube_builder_t* b, bool v);
int  ros_radar_cube_builder_build(ros_radar_cube_builder_t* b,
                                  uint8_t** out_bytes, size_t* out_len);
int  ros_radar_cube_builder_encode_into(ros_radar_cube_builder_t* b,
                                        uint8_t* buf, size_t cap,
                                        size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - RadarInfo (builder, 3.2.0+)
 * ========================================================================= */
typedef struct ros_radar_info_builder_s ros_radar_info_builder_t;
ros_radar_info_builder_t* ros_radar_info_builder_new(void);
void ros_radar_info_builder_free(ros_radar_info_builder_t* b);
void ros_radar_info_builder_set_stamp(ros_radar_info_builder_t* b,
                                      int32_t sec, uint32_t nsec);
int  ros_radar_info_builder_set_frame_id(ros_radar_info_builder_t* b,
                                         const char* s);
int  ros_radar_info_builder_set_center_frequency(ros_radar_info_builder_t* b,
                                                 const char* s);
int  ros_radar_info_builder_set_frequency_sweep(ros_radar_info_builder_t* b,
                                                const char* s);
int  ros_radar_info_builder_set_range_toggle(ros_radar_info_builder_t* b,
                                             const char* s);
int  ros_radar_info_builder_set_detection_sensitivity(ros_radar_info_builder_t* b,
                                                      const char* s);
void ros_radar_info_builder_set_cube(ros_radar_info_builder_t* b, bool v);
int  ros_radar_info_builder_build(ros_radar_info_builder_t* b,
                                  uint8_t** out_bytes, size_t* out_len);
int  ros_radar_info_builder_encode_into(ros_radar_info_builder_t* b,
                                        uint8_t* buf, size_t cap,
                                        size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - Track (builder, 3.2.0+)
 * ========================================================================= */
typedef struct ros_track_builder_s ros_track_builder_t;
ros_track_builder_t* ros_track_builder_new(void);
void ros_track_builder_free(ros_track_builder_t* b);
int  ros_track_builder_set_id(ros_track_builder_t* b, const char* s);
void ros_track_builder_set_lifetime(ros_track_builder_t* b, int32_t v);
void ros_track_builder_set_created(ros_track_builder_t* b,
                                   int32_t sec, uint32_t nsec);
int  ros_track_builder_build(ros_track_builder_t* b,
                             uint8_t** out_bytes, size_t* out_len);
int  ros_track_builder_encode_into(ros_track_builder_t* b,
                                   uint8_t* buf, size_t cap, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - DetectBox / shared descriptors (builder, 3.2.0+)
 * ========================================================================= */
/**
 * C-POD descriptor for a DetectBox element. Used both for the standalone
 * DetectBox builder and for the `boxes` sequence on Detect and Model.
 *
 * `label` and `track_id` are BORROWED NUL-terminated C strings: the backing
 * storage must remain valid until the consuming builder is finalised
 * (build / encode_into) or freed.
 */
typedef struct ros_detect_box_elem_s {
    float    center_x;
    float    center_y;
    float    width;
    float    height;
    const char* label;
    float    score;
    float    distance;
    float    speed;
    const char* track_id;
    int32_t  track_lifetime;
    int32_t  track_created_sec;
    uint32_t track_created_nanosec;
} ros_detect_box_elem_t;

typedef struct ros_detect_box_builder_s ros_detect_box_builder_t;
ros_detect_box_builder_t* ros_detect_box_builder_new(void);
void ros_detect_box_builder_free(ros_detect_box_builder_t* b);
void ros_detect_box_builder_set_center_x(ros_detect_box_builder_t* b, float v);
void ros_detect_box_builder_set_center_y(ros_detect_box_builder_t* b, float v);
void ros_detect_box_builder_set_width(ros_detect_box_builder_t* b, float v);
void ros_detect_box_builder_set_height(ros_detect_box_builder_t* b, float v);
int  ros_detect_box_builder_set_label(ros_detect_box_builder_t* b, const char* s);
void ros_detect_box_builder_set_score(ros_detect_box_builder_t* b, float v);
void ros_detect_box_builder_set_distance(ros_detect_box_builder_t* b, float v);
void ros_detect_box_builder_set_speed(ros_detect_box_builder_t* b, float v);
int  ros_detect_box_builder_set_track_id(ros_detect_box_builder_t* b,
                                         const char* s);
void ros_detect_box_builder_set_track_lifetime(ros_detect_box_builder_t* b,
                                               int32_t v);
void ros_detect_box_builder_set_track_created(ros_detect_box_builder_t* b,
                                              int32_t sec, uint32_t nsec);
int  ros_detect_box_builder_build(ros_detect_box_builder_t* b,
                                  uint8_t** out_bytes, size_t* out_len);
int  ros_detect_box_builder_encode_into(ros_detect_box_builder_t* b,
                                        uint8_t* buf, size_t cap,
                                        size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - Detect (builder, 3.2.0+)
 * ========================================================================= */
typedef struct ros_detect_builder_s ros_detect_builder_t;
ros_detect_builder_t* ros_detect_builder_new(void);
void ros_detect_builder_free(ros_detect_builder_t* b);
void ros_detect_builder_set_stamp(ros_detect_builder_t* b,
                                  int32_t sec, uint32_t nsec);
int  ros_detect_builder_set_frame_id(ros_detect_builder_t* b, const char* s);
void ros_detect_builder_set_input_timestamp(ros_detect_builder_t* b,
                                            int32_t sec, uint32_t nsec);
void ros_detect_builder_set_model_time(ros_detect_builder_t* b,
                                       int32_t sec, uint32_t nsec);
void ros_detect_builder_set_output_time(ros_detect_builder_t* b,
                                        int32_t sec, uint32_t nsec);
/** BORROWED — each element's label/track_id must remain valid until next
 *  setter, build, encode_into, or free. */
void ros_detect_builder_set_boxes(ros_detect_builder_t* b,
                                  const ros_detect_box_elem_t* boxes,
                                  size_t count);
int  ros_detect_builder_build(ros_detect_builder_t* b,
                              uint8_t** out_bytes, size_t* out_len);
int  ros_detect_builder_encode_into(ros_detect_builder_t* b,
                                    uint8_t* buf, size_t cap, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - CameraFrame (builder, 3.2.0+)
 * ========================================================================= */
/**
 * C-POD descriptor for a CameraPlane element. `data` is BORROWED: it must
 * remain valid until the consuming builder is finalised or freed.
 */
typedef struct ros_camera_plane_elem_s {
    int32_t  fd;
    uint32_t offset;
    uint32_t stride;
    uint32_t size;
    uint32_t used;
    const uint8_t* data;
    size_t   data_len;
} ros_camera_plane_elem_t;

typedef struct ros_camera_frame_builder_s ros_camera_frame_builder_t;
ros_camera_frame_builder_t* ros_camera_frame_builder_new(void);
void ros_camera_frame_builder_free(ros_camera_frame_builder_t* b);
void ros_camera_frame_builder_set_stamp(ros_camera_frame_builder_t* b,
                                        int32_t sec, uint32_t nsec);
int  ros_camera_frame_builder_set_frame_id(ros_camera_frame_builder_t* b,
                                           const char* s);
void ros_camera_frame_builder_set_seq(ros_camera_frame_builder_t* b, uint64_t v);
void ros_camera_frame_builder_set_pid(ros_camera_frame_builder_t* b, uint32_t v);
void ros_camera_frame_builder_set_width(ros_camera_frame_builder_t* b, uint32_t v);
void ros_camera_frame_builder_set_height(ros_camera_frame_builder_t* b, uint32_t v);
int  ros_camera_frame_builder_set_format(ros_camera_frame_builder_t* b,
                                         const char* s);
int  ros_camera_frame_builder_set_color_space(ros_camera_frame_builder_t* b,
                                              const char* s);
int  ros_camera_frame_builder_set_color_transfer(ros_camera_frame_builder_t* b,
                                                 const char* s);
int  ros_camera_frame_builder_set_color_encoding(ros_camera_frame_builder_t* b,
                                                 const char* s);
int  ros_camera_frame_builder_set_color_range(ros_camera_frame_builder_t* b,
                                              const char* s);
void ros_camera_frame_builder_set_fence_fd(ros_camera_frame_builder_t* b, int32_t v);
/** BORROWED — each element's data must remain valid until next setter,
 *  build, encode_into, or free. */
void ros_camera_frame_builder_set_planes(ros_camera_frame_builder_t* b,
                                         const ros_camera_plane_elem_t* planes,
                                         size_t count);
int  ros_camera_frame_builder_build(ros_camera_frame_builder_t* b,
                                    uint8_t** out_bytes, size_t* out_len);
int  ros_camera_frame_builder_encode_into(ros_camera_frame_builder_t* b,
                                          uint8_t* buf, size_t cap,
                                          size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - Model (builder, 3.2.0+)
 * ========================================================================= */
/**
 * C-POD descriptor for a Mask element used by the `masks` sequence on
 * Model. `encoding` and `mask` are BORROWED; both must remain valid until
 * the consuming builder is finalised or freed.
 */
typedef struct ros_mask_elem_s {
    uint32_t height;
    uint32_t width;
    uint32_t length;
    const char* encoding;
    const uint8_t* mask;
    size_t   mask_len;
    bool     boxed;
} ros_mask_elem_t;

typedef struct ros_model_builder_s ros_model_builder_t;
ros_model_builder_t* ros_model_builder_new(void);
void ros_model_builder_free(ros_model_builder_t* b);
void ros_model_builder_set_stamp(ros_model_builder_t* b,
                                 int32_t sec, uint32_t nsec);
int  ros_model_builder_set_frame_id(ros_model_builder_t* b, const char* s);
void ros_model_builder_set_input_time(ros_model_builder_t* b,
                                      int32_t sec, uint32_t nsec);
void ros_model_builder_set_model_time(ros_model_builder_t* b,
                                      int32_t sec, uint32_t nsec);
void ros_model_builder_set_output_time(ros_model_builder_t* b,
                                       int32_t sec, uint32_t nsec);
void ros_model_builder_set_decode_time(ros_model_builder_t* b,
                                       int32_t sec, uint32_t nsec);
/** BORROWED — see ros_detect_builder_set_boxes. */
void ros_model_builder_set_boxes(ros_model_builder_t* b,
                                 const ros_detect_box_elem_t* boxes,
                                 size_t count);
/** BORROWED — each element's encoding/mask must remain valid. */
void ros_model_builder_set_masks(ros_model_builder_t* b,
                                 const ros_mask_elem_t* masks,
                                 size_t count);
int  ros_model_builder_build(ros_model_builder_t* b,
                             uint8_t** out_bytes, size_t* out_len);
int  ros_model_builder_encode_into(ros_model_builder_t* b,
                                   uint8_t* buf, size_t cap, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - ModelInfo (builder, 3.2.0+)
 * ========================================================================= */
typedef struct ros_model_info_builder_s ros_model_info_builder_t;
ros_model_info_builder_t* ros_model_info_builder_new(void);
void ros_model_info_builder_free(ros_model_info_builder_t* b);
void ros_model_info_builder_set_stamp(ros_model_info_builder_t* b,
                                      int32_t sec, uint32_t nsec);
int  ros_model_info_builder_set_frame_id(ros_model_info_builder_t* b,
                                         const char* s);
/** BORROWED until next setter / build / free. */
void ros_model_info_builder_set_input_shape(ros_model_info_builder_t* b,
                                            const uint32_t* data, size_t len);
void ros_model_info_builder_set_input_type(ros_model_info_builder_t* b, uint8_t v);
/** BORROWED until next setter / build / free. */
void ros_model_info_builder_set_output_shape(ros_model_info_builder_t* b,
                                             const uint32_t* data, size_t len);
void ros_model_info_builder_set_output_type(ros_model_info_builder_t* b, uint8_t v);
/**
 * Set the labels sequence. Each C string is copied into builder-owned
 * storage, so the caller's array and strings need only remain valid for the
 * duration of this call.
 * @return 0 on success, -1 on error (errno EINVAL for NULL handle or
 *         NULL element when count > 0).
 */
int  ros_model_info_builder_set_labels(ros_model_info_builder_t* b,
                                       const char* const* labels,
                                       size_t count);
int  ros_model_info_builder_set_model_type(ros_model_info_builder_t* b,
                                           const char* s);
int  ros_model_info_builder_set_model_format(ros_model_info_builder_t* b,
                                             const char* s);
int  ros_model_info_builder_set_model_name(ros_model_info_builder_t* b,
                                           const char* s);
int  ros_model_info_builder_build(ros_model_info_builder_t* b,
                                  uint8_t** out_bytes, size_t* out_len);
int  ros_model_info_builder_encode_into(ros_model_info_builder_t* b,
                                        uint8_t* buf, size_t cap,
                                        size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - Vibration (builder, 3.2.0+)
 * ========================================================================= */
typedef struct ros_vibration_builder_s ros_vibration_builder_t;
ros_vibration_builder_t* ros_vibration_builder_new(void);
void ros_vibration_builder_free(ros_vibration_builder_t* b);
void ros_vibration_builder_set_stamp(ros_vibration_builder_t* b,
                                     int32_t sec, uint32_t nsec);
int  ros_vibration_builder_set_frame_id(ros_vibration_builder_t* b,
                                        const char* s);
void ros_vibration_builder_set_vibration(ros_vibration_builder_t* b,
                                         double x, double y, double z);
void ros_vibration_builder_set_band_lower_hz(ros_vibration_builder_t* b, float v);
void ros_vibration_builder_set_band_upper_hz(ros_vibration_builder_t* b, float v);
void ros_vibration_builder_set_measurement_type(ros_vibration_builder_t* b, uint8_t v);
void ros_vibration_builder_set_unit(ros_vibration_builder_t* b, uint8_t v);
/** BORROWED until next setter / build / free. */
void ros_vibration_builder_set_clipping(ros_vibration_builder_t* b,
                                        const uint32_t* data, size_t len);
int  ros_vibration_builder_build(ros_vibration_builder_t* b,
                                 uint8_t** out_bytes, size_t* out_len);
int  ros_vibration_builder_encode_into(ros_vibration_builder_t* b,
                                       uint8_t* buf, size_t cap,
                                       size_t* out_len);

/* ============================================================================
 * foxglove_msgs - CompressedVideo (builder, 3.2.0+)
 * ========================================================================= */
typedef struct ros_foxglove_compressed_video_builder_s
    ros_foxglove_compressed_video_builder_t;
ros_foxglove_compressed_video_builder_t*
ros_foxglove_compressed_video_builder_new(void);
void ros_foxglove_compressed_video_builder_free(
    ros_foxglove_compressed_video_builder_t* b);
void ros_foxglove_compressed_video_builder_set_stamp(
    ros_foxglove_compressed_video_builder_t* b, int32_t sec, uint32_t nsec);
int  ros_foxglove_compressed_video_builder_set_frame_id(
    ros_foxglove_compressed_video_builder_t* b, const char* s);
/** BORROWED until next setter / build / free. */
void ros_foxglove_compressed_video_builder_set_data(
    ros_foxglove_compressed_video_builder_t* b,
    const uint8_t* data, size_t len);
int  ros_foxglove_compressed_video_builder_set_format(
    ros_foxglove_compressed_video_builder_t* b, const char* s);
int  ros_foxglove_compressed_video_builder_build(
    ros_foxglove_compressed_video_builder_t* b,
    uint8_t** out_bytes, size_t* out_len);
int  ros_foxglove_compressed_video_builder_encode_into(
    ros_foxglove_compressed_video_builder_t* b,
    uint8_t* buf, size_t cap, size_t* out_len);

/* ============================================================================
 * foxglove_msgs - FoxgloveTextAnnotation (builder, 3.2.0+)
 * ========================================================================= */
typedef struct ros_foxglove_text_annotation_builder_s
    ros_foxglove_text_annotation_builder_t;
ros_foxglove_text_annotation_builder_t*
ros_foxglove_text_annotation_builder_new(void);
void ros_foxglove_text_annotation_builder_free(
    ros_foxglove_text_annotation_builder_t* b);
void ros_foxglove_text_annotation_builder_set_timestamp(
    ros_foxglove_text_annotation_builder_t* b, int32_t sec, uint32_t nsec);
void ros_foxglove_text_annotation_builder_set_position(
    ros_foxglove_text_annotation_builder_t* b, double x, double y);
int  ros_foxglove_text_annotation_builder_set_text(
    ros_foxglove_text_annotation_builder_t* b, const char* s);
void ros_foxglove_text_annotation_builder_set_font_size(
    ros_foxglove_text_annotation_builder_t* b, double v);
void ros_foxglove_text_annotation_builder_set_text_color(
    ros_foxglove_text_annotation_builder_t* b,
    double r, double g, double b_, double a);
void ros_foxglove_text_annotation_builder_set_background_color(
    ros_foxglove_text_annotation_builder_t* b,
    double r, double g, double b_, double a);
int  ros_foxglove_text_annotation_builder_build(
    ros_foxglove_text_annotation_builder_t* b,
    uint8_t** out_bytes, size_t* out_len);
int  ros_foxglove_text_annotation_builder_encode_into(
    ros_foxglove_text_annotation_builder_t* b,
    uint8_t* buf, size_t cap, size_t* out_len);

/* ============================================================================
 * foxglove_msgs - FoxglovePointAnnotation (builder, 3.2.0+)
 * ========================================================================= */
/** C-POD descriptor for a FoxglovePoint2 element (no borrowed fields). */
typedef struct ros_foxglove_point2_elem_s {
    double x;
    double y;
} ros_foxglove_point2_elem_t;

/** C-POD descriptor for a FoxgloveColor element (no borrowed fields). */
typedef struct ros_foxglove_color_elem_s {
    double r;
    double g;
    double b;
    double a;
} ros_foxglove_color_elem_t;

typedef struct ros_foxglove_point_annotation_builder_s
    ros_foxglove_point_annotation_builder_t;
ros_foxglove_point_annotation_builder_t*
ros_foxglove_point_annotation_builder_new(void);
void ros_foxglove_point_annotation_builder_free(
    ros_foxglove_point_annotation_builder_t* b);
void ros_foxglove_point_annotation_builder_set_timestamp(
    ros_foxglove_point_annotation_builder_t* b, int32_t sec, uint32_t nsec);
void ros_foxglove_point_annotation_builder_set_type(
    ros_foxglove_point_annotation_builder_t* b, uint8_t v);
/** BORROWED until next setter / build / free. */
void ros_foxglove_point_annotation_builder_set_points(
    ros_foxglove_point_annotation_builder_t* b,
    const ros_foxglove_point2_elem_t* points, size_t count);
void ros_foxglove_point_annotation_builder_set_outline_color(
    ros_foxglove_point_annotation_builder_t* b,
    double r, double g, double b_, double a);
/** BORROWED until next setter / build / free. */
void ros_foxglove_point_annotation_builder_set_outline_colors(
    ros_foxglove_point_annotation_builder_t* b,
    const ros_foxglove_color_elem_t* colors, size_t count);
void ros_foxglove_point_annotation_builder_set_fill_color(
    ros_foxglove_point_annotation_builder_t* b,
    double r, double g, double b_, double a);
void ros_foxglove_point_annotation_builder_set_thickness(
    ros_foxglove_point_annotation_builder_t* b, double v);
int  ros_foxglove_point_annotation_builder_build(
    ros_foxglove_point_annotation_builder_t* b,
    uint8_t** out_bytes, size_t* out_len);
int  ros_foxglove_point_annotation_builder_encode_into(
    ros_foxglove_point_annotation_builder_t* b,
    uint8_t* buf, size_t cap, size_t* out_len);

/* ============================================================================
 * foxglove_msgs - FoxgloveImageAnnotation (builder, 3.2.0+)
 * ========================================================================= */
/** C-POD descriptor for a FoxgloveCircleAnnotations element (no borrows). */
typedef struct ros_foxglove_circle_annotation_elem_s {
    int32_t  timestamp_sec;
    uint32_t timestamp_nanosec;
    double   position_x;
    double   position_y;
    double   diameter;
    double   thickness;
    double   fill_color_r;
    double   fill_color_g;
    double   fill_color_b;
    double   fill_color_a;
    double   outline_color_r;
    double   outline_color_g;
    double   outline_color_b;
    double   outline_color_a;
} ros_foxglove_circle_annotation_elem_t;

/**
 * C-POD descriptor for a FoxglovePointAnnotation element. Inner `points`
 * and `outline_colors` arrays are BORROWED: they must remain valid until
 * the consuming builder is finalised or freed.
 */
typedef struct ros_foxglove_point_annotation_elem_s {
    int32_t  timestamp_sec;
    uint32_t timestamp_nanosec;
    uint8_t  type_;
    const ros_foxglove_point2_elem_t* points;
    size_t   points_count;
    double   outline_color_r;
    double   outline_color_g;
    double   outline_color_b;
    double   outline_color_a;
    const ros_foxglove_color_elem_t* outline_colors;
    size_t   outline_colors_count;
    double   fill_color_r;
    double   fill_color_g;
    double   fill_color_b;
    double   fill_color_a;
    double   thickness;
} ros_foxglove_point_annotation_elem_t;

/**
 * C-POD descriptor for a FoxgloveTextAnnotation element. `text` is a
 * BORROWED NUL-terminated C string.
 */
typedef struct ros_foxglove_text_annotation_elem_s {
    int32_t  timestamp_sec;
    uint32_t timestamp_nanosec;
    double   position_x;
    double   position_y;
    const char* text;
    double   font_size;
    double   text_color_r;
    double   text_color_g;
    double   text_color_b;
    double   text_color_a;
    double   background_color_r;
    double   background_color_g;
    double   background_color_b;
    double   background_color_a;
} ros_foxglove_text_annotation_elem_t;

typedef struct ros_foxglove_image_annotation_builder_s
    ros_foxglove_image_annotation_builder_t;
ros_foxglove_image_annotation_builder_t*
ros_foxglove_image_annotation_builder_new(void);
void ros_foxglove_image_annotation_builder_free(
    ros_foxglove_image_annotation_builder_t* b);
/** BORROWED — caller keeps `circles` valid until next setter / build / free. */
void ros_foxglove_image_annotation_builder_set_circles(
    ros_foxglove_image_annotation_builder_t* b,
    const ros_foxglove_circle_annotation_elem_t* circles, size_t count);
/** BORROWED — each element's inner `points`/`outline_colors` arrays must
 *  remain valid until next setter / build / free. */
void ros_foxglove_image_annotation_builder_set_points(
    ros_foxglove_image_annotation_builder_t* b,
    const ros_foxglove_point_annotation_elem_t* points, size_t count);
/** BORROWED — each element's `text` C string must remain valid until next
 *  setter / build / free. */
void ros_foxglove_image_annotation_builder_set_texts(
    ros_foxglove_image_annotation_builder_t* b,
    const ros_foxglove_text_annotation_elem_t* texts, size_t count);
int  ros_foxglove_image_annotation_builder_build(
    ros_foxglove_image_annotation_builder_t* b,
    uint8_t** out_bytes, size_t* out_len);
int  ros_foxglove_image_annotation_builder_encode_into(
    ros_foxglove_image_annotation_builder_t* b,
    uint8_t* buf, size_t cap, size_t* out_len);

#ifdef __cplusplus
}
#endif

#endif /* EDGEFIRST_SCHEMAS_H */
