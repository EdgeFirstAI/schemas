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
typedef struct ros_header_t ros_header_t;

/* sensor_msgs */
typedef struct ros_image_t ros_image_t;
typedef struct ros_compressed_image_t ros_compressed_image_t;
typedef struct ros_imu_t ros_imu_t;
typedef struct ros_nav_sat_fix_t ros_nav_sat_fix_t;
typedef struct ros_point_cloud2_t ros_point_cloud2_t;
typedef struct ros_camera_info_t ros_camera_info_t;

/* geometry_msgs */
typedef struct ros_transform_stamped_t ros_transform_stamped_t;

/* foxglove_msgs */
typedef struct ros_compressed_video_t ros_compressed_video_t;

/* edgefirst_msgs */
typedef struct ros_mask_t ros_mask_t;
typedef struct ros_dmabuffer_t ros_dmabuffer_t;
typedef struct ros_radar_cube_t ros_radar_cube_t;
typedef struct ros_radar_info_t ros_radar_info_t;
typedef struct ros_detect_t ros_detect_t;
typedef struct ros_model_t ros_model_t;
typedef struct ros_model_info_t ros_model_info_t;
typedef struct ros_track_t ros_track_t;
typedef struct ros_box_t ros_box_t;
typedef struct ros_local_time_t ros_local_time_t;

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
 * @param data CDR encoded bytes (copied internally)
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

/* ============================================================================
 * sensor_msgs - Image (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create an Image view from CDR bytes.
 * @param data CDR encoded bytes (copied internally)
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

/* ============================================================================
 * sensor_msgs - CompressedImage (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a CompressedImage view from CDR bytes.
 * @param data CDR encoded bytes (copied internally)
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

/* ============================================================================
 * foxglove_msgs - CompressedVideo (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a CompressedVideo view from CDR bytes.
 * @param data CDR encoded bytes (copied internally)
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
 * @param data CDR encoded bytes (copied internally)
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

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_mask_as_cdr(const ros_mask_t* view, size_t* out_len);

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
 * @param data CDR encoded bytes (copied internally)
 * @param len Length of data
 * @return Opaque handle or NULL on error
 */
ros_dmabuffer_t* ros_dmabuffer_from_cdr(const uint8_t* data, size_t len);

/** @brief Free a DmaBuffer view handle. */
void ros_dmabuffer_free(ros_dmabuffer_t* view);

/** @brief Get stamp seconds. */
int32_t ros_dmabuffer_get_stamp_sec(const ros_dmabuffer_t* view);

/** @brief Get stamp nanoseconds. */
uint32_t ros_dmabuffer_get_stamp_nanosec(const ros_dmabuffer_t* view);

/** @brief Get frame_id (borrowed). */
const char* ros_dmabuffer_get_frame_id(const ros_dmabuffer_t* view);

/** @brief Get process ID. */
uint32_t ros_dmabuffer_get_pid(const ros_dmabuffer_t* view);

/** @brief Get file descriptor. */
int32_t ros_dmabuffer_get_fd(const ros_dmabuffer_t* view);

/** @brief Get buffer width. */
uint32_t ros_dmabuffer_get_width(const ros_dmabuffer_t* view);

/** @brief Get buffer height. */
uint32_t ros_dmabuffer_get_height(const ros_dmabuffer_t* view);

/** @brief Get buffer stride. */
uint32_t ros_dmabuffer_get_stride(const ros_dmabuffer_t* view);

/** @brief Get FourCC pixel format code. */
uint32_t ros_dmabuffer_get_fourcc(const ros_dmabuffer_t* view);

/** @brief Get buffer length in bytes. */
uint32_t ros_dmabuffer_get_length(const ros_dmabuffer_t* view);

/** @brief Borrow raw CDR bytes from the handle. */
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
int ros_dmabuffer_encode(uint8_t** out_bytes, size_t* out_len,
                         int32_t stamp_sec, uint32_t stamp_nanosec,
                         const char* frame_id,
                         uint32_t pid, int32_t fd,
                         uint32_t width, uint32_t height,
                         uint32_t stride, uint32_t fourcc, uint32_t length);

/* ============================================================================
 * sensor_msgs - Imu (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create an Imu view from CDR bytes.
 * @param data CDR encoded bytes (copied internally)
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

/* ============================================================================
 * sensor_msgs - NavSatFix (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a NavSatFix view from CDR bytes.
 * @param data CDR encoded bytes (copied internally)
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

/* ============================================================================
 * geometry_msgs - TransformStamped (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a TransformStamped view from CDR bytes.
 * @param data CDR encoded bytes (copied internally)
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
 * @param data CDR encoded bytes (copied internally)
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
 * @param data CDR encoded bytes (copied internally)
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
 * @param data CDR encoded bytes (copied internally)
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

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_detect_as_cdr(const ros_detect_t* view, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - Model (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a Model view from CDR bytes.
 * @param data CDR encoded bytes (copied internally)
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

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_model_as_cdr(const ros_model_t* view, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - ModelInfo (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a ModelInfo view from CDR bytes.
 * @param data CDR encoded bytes (copied internally)
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
 * @param data CDR encoded bytes (copied internally)
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

/* ============================================================================
 * sensor_msgs - CameraInfo (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a CameraInfo view from CDR bytes.
 * @param data CDR encoded bytes (copied internally)
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

/* ============================================================================
 * edgefirst_msgs - Track (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a Track view from CDR bytes.
 * @param data CDR encoded bytes (copied internally)
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
 * @param data CDR encoded bytes (copied internally)
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

/** @brief Borrow raw CDR bytes from the handle. */
const uint8_t* ros_box_as_cdr(const ros_box_t* view, size_t* out_len);

/* ============================================================================
 * edgefirst_msgs - LocalTime (buffer-backed)
 * ========================================================================= */

/**
 * @brief Create a LocalTime view from CDR bytes.
 * @param data CDR encoded bytes (copied internally)
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

#ifdef __cplusplus
}
#endif

#endif /* EDGEFIRST_SCHEMAS_H */
