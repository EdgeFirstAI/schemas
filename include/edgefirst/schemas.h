/**
 * @file schemas.h
 * @brief EdgeFirst Schemas C API
 * 
 * SPDX-License-Identifier: Apache-2.0
 * Copyright © 2025 Au-Zone Technologies. All Rights Reserved.
 * 
 * C API for EdgeFirst Perception middleware message schemas with CDR serialization.
 * 
 * @section Overview
 * 
 * This library provides C bindings to the EdgeFirst Rust schema library, offering:
 * - Opaque handles to schema message types
 * - Getter/setter functions for all message fields
 * - CDR serialization and deserialization
 * - Memory management functions
 * 
 * @section Error Handling
 * 
 * Functions that can fail use standard POSIX errno conventions:
 * - Functions returning int: 0 on success, -1 on error (errno set)
 * - Functions returning pointers: valid pointer on success, NULL on error (errno set)
 * 
 * Standard errno codes used:
 * - EINVAL: Invalid argument (NULL pointer, bad UTF-8, invalid parameter)
 * - ENOMEM: Out of memory (allocation failure)
 * - EBADMSG: Bad message (deserialization/decoding failure)
 * 
 * Thread Safety: errno is thread-local in POSIX systems. Functions are thread-safe
 * for distinct message instances. Shared instances require external synchronization.
 * 
 * @section Usage
 * 
 * All message types are represented as opaque pointers. Create messages using
 * the <type>_new() functions, access fields using getters/setters, and free
 * memory using <type>_free().
 * 
 * @subsection Memory Management
 * 
 * - Message handles are owned by the caller and must be freed
 * - String getters return newly allocated C strings that must be freed by caller
 * - Nested message getters return borrowed pointers valid only during parent lifetime
 * - Array getters return borrowed pointers with length output parameter
 * 
 * @subsection Example
 * @code
 * #include <edgefirst/schemas.h>
 * #include <errno.h>
 * #include <stdio.h>
 * #include <string.h>
 * 
 * // Create a new Header
 * RosHeader* header = ros_header_new();
 * if (!header) {
 *     perror("ros_header_new");
 *     return 1;
 * }
 * 
 * if (ros_header_set_frame_id(header, "camera") != 0) {
 *     perror("ros_header_set_frame_id");
 *     ros_header_free(header);
 *     return 1;
 * }
 * 
 * // Serialize to CDR
 * uint8_t* bytes = NULL;
 * size_t len = 0;
 * if (ros_header_serialize(header, &bytes, &len) != 0) {
 *     perror("ros_header_serialize");
 *     ros_header_free(header);
 *     return 1;
 * }
 * 
 * // Deserialize
 * RosHeader* header2 = ros_header_deserialize(bytes, len);
 * if (!header2) {
 *     fprintf(stderr, "Deserialization failed: %s\n", strerror(errno));
 *     free(bytes);
 *     ros_header_free(header);
 *     return 1;
 * }
 * 
 * // Get frame_id (caller must free)
 * char* frame_id = ros_header_get_frame_id(header2);
 * printf("Frame ID: %s\n", frame_id);
 * free(frame_id);
 * 
 * // Cleanup
 * ros_header_free(header);
 * ros_header_free(header2);
 * free(bytes);
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
 * Opaque Type Declarations
 * ========================================================================= */

/* builtin_interfaces */
typedef struct RosTime RosTime;
typedef struct RosDuration RosDuration;

/* std_msgs */
typedef struct RosHeader RosHeader;
typedef struct RosColorRGBA RosColorRGBA;

/* geometry_msgs */
typedef struct RosVector3 RosVector3;
typedef struct RosPoint RosPoint;
typedef struct RosPoint32 RosPoint32;
typedef struct RosQuaternion RosQuaternion;
typedef struct RosPose RosPose;
typedef struct RosPose2D RosPose2D;
typedef struct RosTransform RosTransform;
typedef struct RosTransformStamped RosTransformStamped;
typedef struct RosTwist RosTwist;
typedef struct RosTwistStamped RosTwistStamped;
typedef struct RosAccel RosAccel;
typedef struct RosAccelStamped RosAccelStamped;
typedef struct RosInertia RosInertia;
typedef struct RosInertiaStamped RosInertiaStamped;
typedef struct RosPointStamped RosPointStamped;

/* sensor_msgs */
typedef struct RosImage RosImage;
typedef struct RosCompressedImage RosCompressedImage;
typedef struct RosCameraInfo RosCameraInfo;
typedef struct RosImu RosImu;
typedef struct RosNavSatFix RosNavSatFix;
typedef struct RosNavSatStatus RosNavSatStatus;
typedef struct RosPointCloud2 RosPointCloud2;
typedef struct RosPointField RosPointField;
typedef struct RosRegionOfInterest RosRegionOfInterest;

/* edgefirst_msgs */
typedef struct EdgeFirstDmaBuf EdgeFirstDmaBuf;
typedef struct EdgeFirstRadarCube EdgeFirstRadarCube;
typedef struct EdgeFirstRadarInfo EdgeFirstRadarInfo;
typedef struct EdgeFirstDetect EdgeFirstDetect;
typedef struct EdgeFirstDetectBox2D EdgeFirstDetectBox2D;
typedef struct EdgeFirstDetectTrack EdgeFirstDetectTrack;
typedef struct EdgeFirstMask EdgeFirstMask;
typedef struct EdgeFirstModel EdgeFirstModel;
typedef struct EdgeFirstModelInfo EdgeFirstModelInfo;
typedef struct EdgeFirstLocalTime EdgeFirstLocalTime;
typedef struct EdgeFirstDate EdgeFirstDate;

/* foxglove_msgs */
typedef struct FoxgloveCompressedVideo FoxgloveCompressedVideo;
typedef struct FoxgloveImageAnnotations FoxgloveImageAnnotations;
typedef struct FoxgloveCircleAnnotations FoxgloveCircleAnnotations;
typedef struct FoxglovePointAnnotations FoxglovePointAnnotations;
typedef struct FoxgloveTextAnnotations FoxgloveTextAnnotations;
typedef struct FoxglovePoint2 FoxglovePoint2;
typedef struct FoxgloveColor FoxgloveColor;

/* ============================================================================
 * builtin_interfaces - Time
 * ========================================================================= */

/**
 * @brief Create a new Time message
 * @return Pointer to new Time or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
RosTime* ros_time_new(void);

/**
 * @brief Free a Time message
 * @param time Pointer to Time to free (can be NULL)
 */
void ros_time_free(RosTime* time);

/**
 * @brief Get seconds field
 * @param time Time message
 * @return Seconds since epoch
 */
int32_t ros_time_get_sec(const RosTime* time);

/**
 * @brief Get nanoseconds field
 * @param time Time message
 * @return Nanoseconds component
 */
uint32_t ros_time_get_nanosec(const RosTime* time);

/**
 * @brief Set seconds field
 * @param time Time message
 * @param sec Seconds since epoch
 */
void ros_time_set_sec(RosTime* time, int32_t sec);

/**
 * @brief Set nanoseconds field
 * @param time Time message
 * @param nanosec Nanoseconds component
 */
void ros_time_set_nanosec(RosTime* time, uint32_t nanosec);

/**
 * @brief Serialize Time to CDR format
 * @param time Time message to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_time_serialize(const RosTime* time, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize Time from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized Time or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
RosTime* ros_time_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * builtin_interfaces - Duration
 * ========================================================================= */

/**
 * @brief Create a new Duration message
 * @return Pointer to new Duration or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
RosDuration* ros_duration_new(void);

/**
 * @brief Free a Duration message
 * @param duration Pointer to Duration to free (can be NULL)
 */
void ros_duration_free(RosDuration* duration);

/**
 * @brief Get seconds field
 * @param duration Duration message
 * @return Seconds
 */
int32_t ros_duration_get_sec(const RosDuration* duration);

/**
 * @brief Get nanoseconds field
 * @param duration Duration message
 * @return Nanoseconds
 */
uint32_t ros_duration_get_nanosec(const RosDuration* duration);

/**
 * @brief Set seconds field
 * @param duration Duration message
 * @param sec Seconds
 */
void ros_duration_set_sec(RosDuration* duration, int32_t sec);

/**
 * @brief Set nanoseconds field
 * @param duration Duration message
 * @param nanosec Nanoseconds
 */
void ros_duration_set_nanosec(RosDuration* duration, uint32_t nanosec);

/**
 * @brief Serialize Duration to CDR format
 * @param duration Duration message to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_duration_serialize(const RosDuration* duration, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize Duration from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized Duration or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
RosDuration* ros_duration_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * std_msgs - Header
 * ========================================================================= */

/**
 * @brief Create a new Header message
 * @return Pointer to new Header or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
RosHeader* ros_header_new(void);

/**
 * @brief Free a Header message
 * @param header Pointer to Header to free (can be NULL)
 */
void ros_header_free(RosHeader* header);

/**
 * @brief Get stamp field (borrowed pointer, valid during header lifetime)
 * @param header Header message
 * @return Pointer to Time stamp
 */
const RosTime* ros_header_get_stamp(const RosHeader* header);

/**
 * @brief Get mutable stamp field for modification
 * @param header Header message
 * @return Mutable pointer to Time stamp
 */
RosTime* ros_header_get_stamp_mut(RosHeader* header);

/**
 * @brief Get frame_id field (caller must free returned string)
 * @param header Header message
 * @return Newly allocated C string or NULL on error
 */
char* ros_header_get_frame_id(const RosHeader* header);

/**
 * @brief Set frame_id field
 * @param header Header message
 * @param frame_id Frame ID string (copied internally)
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_header_set_frame_id(RosHeader* header, const char* frame_id);

/**
 * @brief Serialize Header to CDR format
 * @param header Header message to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_header_serialize(const RosHeader* header, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize Header from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized Header or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
RosHeader* ros_header_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * std_msgs - ColorRGBA
 * ========================================================================= */

/**
 * @brief Create a new ColorRGBA message
 * @return Pointer to new ColorRGBA or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
RosColorRGBA* ros_color_rgba_new(void);

/**
 * @brief Free a ColorRGBA message
 * @param color Pointer to ColorRGBA to free (can be NULL)
 */
void ros_color_rgba_free(RosColorRGBA* color);

/**
 * @brief Get red component
 * @param color ColorRGBA message
 * @return Red component [0.0, 1.0]
 */
float ros_color_rgba_get_r(const RosColorRGBA* color);

/**
 * @brief Get green component
 * @param color ColorRGBA message
 * @return Green component [0.0, 1.0]
 */
float ros_color_rgba_get_g(const RosColorRGBA* color);

/**
 * @brief Get blue component
 * @param color ColorRGBA message
 * @return Blue component [0.0, 1.0]
 */
float ros_color_rgba_get_b(const RosColorRGBA* color);

/**
 * @brief Get alpha component
 * @param color ColorRGBA message
 * @return Alpha component [0.0, 1.0]
 */
float ros_color_rgba_get_a(const RosColorRGBA* color);

/**
 * @brief Set red component
 * @param color ColorRGBA message
 * @param r Red component [0.0, 1.0]
 */
void ros_color_rgba_set_r(RosColorRGBA* color, float r);

/**
 * @brief Set green component
 * @param color ColorRGBA message
 * @param g Green component [0.0, 1.0]
 */
void ros_color_rgba_set_g(RosColorRGBA* color, float g);

/**
 * @brief Set blue component
 * @param color ColorRGBA message
 * @param b Blue component [0.0, 1.0]
 */
void ros_color_rgba_set_b(RosColorRGBA* color, float b);

/**
 * @brief Set alpha component
 * @param color ColorRGBA message
 * @param a Alpha component [0.0, 1.0]
 */
void ros_color_rgba_set_a(RosColorRGBA* color, float a);

/**
 * @brief Serialize ColorRGBA to CDR format
 * @param color ColorRGBA message to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_color_rgba_serialize(const RosColorRGBA* color, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize ColorRGBA from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized ColorRGBA or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
RosColorRGBA* ros_color_rgba_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * geometry_msgs - Vector3
 * ========================================================================= */

/**
 * @brief Create a new Vector3 message
 * @return Pointer to new Vector3 or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
RosVector3* ros_vector3_new(void);

/**
 * @brief Free a Vector3 message
 * @param vec Pointer to Vector3 to free (can be NULL)
 */
void ros_vector3_free(RosVector3* vec);

/**
 * @brief Get x component
 * @param vec Vector3 message
 * @return X component
 */
double ros_vector3_get_x(const RosVector3* vec);

/**
 * @brief Get y component
 * @param vec Vector3 message
 * @return Y component
 */
double ros_vector3_get_y(const RosVector3* vec);

/**
 * @brief Get z component
 * @param vec Vector3 message
 * @return Z component
 */
double ros_vector3_get_z(const RosVector3* vec);

/**
 * @brief Set x component
 * @param vec Vector3 message
 * @param x X component
 */
void ros_vector3_set_x(RosVector3* vec, double x);

/**
 * @brief Set y component
 * @param vec Vector3 message
 * @param y Y component
 */
void ros_vector3_set_y(RosVector3* vec, double y);

/**
 * @brief Set z component
 * @param vec Vector3 message
 * @param z Z component
 */
void ros_vector3_set_z(RosVector3* vec, double z);

/**
 * @brief Serialize Vector3 to CDR format
 * @param vec Vector3 message to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_vector3_serialize(const RosVector3* vec, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize Vector3 from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized Vector3 or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
RosVector3* ros_vector3_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * geometry_msgs - Point and Quaternion (similar patterns)
 * ========================================================================= */

RosPoint* ros_point_new(void);
void ros_point_free(RosPoint* point);
double ros_point_get_x(const RosPoint* point);
double ros_point_get_y(const RosPoint* point);
double ros_point_get_z(const RosPoint* point);
void ros_point_set_x(RosPoint* point, double x);
void ros_point_set_y(RosPoint* point, double y);
void ros_point_set_z(RosPoint* point, double z);
int ros_point_serialize(const RosPoint* point, uint8_t** out_bytes, size_t* out_len);
RosPoint* ros_point_deserialize(const uint8_t* bytes, size_t len);

RosQuaternion* ros_quaternion_new(void);
void ros_quaternion_free(RosQuaternion* quat);
double ros_quaternion_get_x(const RosQuaternion* quat);
double ros_quaternion_get_y(const RosQuaternion* quat);
double ros_quaternion_get_z(const RosQuaternion* quat);
double ros_quaternion_get_w(const RosQuaternion* quat);
void ros_quaternion_set_x(RosQuaternion* quat, double x);
void ros_quaternion_set_y(RosQuaternion* quat, double y);
void ros_quaternion_set_z(RosQuaternion* quat, double z);
void ros_quaternion_set_w(RosQuaternion* quat, double w);
int ros_quaternion_serialize(const RosQuaternion* quat, uint8_t** out_bytes, size_t* out_len);
RosQuaternion* ros_quaternion_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * sensor_msgs - Image
 * ========================================================================= */

/**
 * @brief Create a new Image message
 * @return Pointer to new Image or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
RosImage* ros_image_new(void);

/**
 * @brief Free an Image message
 * @param image Pointer to Image to free (can be NULL)
 */
void ros_image_free(RosImage* image);

/**
 * @brief Get header field (borrowed pointer)
 * @param image Image message
 * @return Pointer to Header
 */
const RosHeader* ros_image_get_header(const RosImage* image);

/**
 * @brief Get mutable header field
 * @param image Image message
 * @return Mutable pointer to Header
 */
RosHeader* ros_image_get_header_mut(RosImage* image);

/**
 * @brief Get height field
 * @param image Image message
 * @return Image height in pixels
 */
uint32_t ros_image_get_height(const RosImage* image);

/**
 * @brief Get width field
 * @param image Image message
 * @return Image width in pixels
 */
uint32_t ros_image_get_width(const RosImage* image);

/**
 * @brief Get encoding field (caller must free)
 * @param image Image message
 * @return Newly allocated encoding string
 */
char* ros_image_get_encoding(const RosImage* image);

/**
 * @brief Get is_bigendian field
 * @param image Image message
 * @return 1 if big endian, 0 if little endian
 */
uint8_t ros_image_get_is_bigendian(const RosImage* image);

/**
 * @brief Get step field
 * @param image Image message
 * @return Row step in bytes
 */
uint32_t ros_image_get_step(const RosImage* image);

/**
 * @brief Get data field (borrowed pointer)
 * @param image Image message
 * @param out_len Output parameter for data length
 * @return Pointer to image data (valid during image lifetime)
 */
const uint8_t* ros_image_get_data(const RosImage* image, size_t* out_len);

/**
 * @brief Set height field
 * @param image Image message
 * @param height Image height
 */
void ros_image_set_height(RosImage* image, uint32_t height);

/**
 * @brief Set width field
 * @param image Image message
 * @param width Image width
 */
void ros_image_set_width(RosImage* image, uint32_t width);

/**
 * @brief Set encoding field
 * @param image Image message
 * @param encoding Encoding string
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_image_set_encoding(RosImage* image, const char* encoding);

/**
 * @brief Set is_bigendian field
 * @param image Image message
 * @param is_bigendian Endianness flag
 */
void ros_image_set_is_bigendian(RosImage* image, uint8_t is_bigendian);

/**
 * @brief Set step field
 * @param image Image message
 * @param step Row step in bytes
 */
void ros_image_set_step(RosImage* image, uint32_t step);

/**
 * @brief Set data field (copies data)
 * @param image Image message
 * @param data Pointer to image data
 * @param len Length of data in bytes
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_image_set_data(RosImage* image, const uint8_t* data, size_t len);

/**
 * @brief Serialize Image to CDR format
 * @param image Image message to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_image_serialize(const RosImage* image, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize Image from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized Image or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
RosImage* ros_image_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * edgefirst_msgs - DmaBuf
 * ========================================================================= */

/**
 * @brief Create a new DmaBuf message
 * @return Pointer to new DmaBuf or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
EdgeFirstDmaBuf* edgefirst_dmabuf_new(void);

/**
 * @brief Free a DmaBuf message
 * @param dmabuf Pointer to DmaBuf to free (can be NULL)
 */
void edgefirst_dmabuf_free(EdgeFirstDmaBuf* dmabuf);

/**
 * @brief Get header field (borrowed pointer)
 * @param dmabuf DmaBuf message
 * @return Pointer to Header
 */
const RosHeader* edgefirst_dmabuf_get_header(const EdgeFirstDmaBuf* dmabuf);

/**
 * @brief Get mutable header field
 * @param dmabuf DmaBuf message
 * @return Mutable pointer to Header
 */
RosHeader* edgefirst_dmabuf_get_header_mut(EdgeFirstDmaBuf* dmabuf);

/**
 * @brief Get pid field
 * @param dmabuf DmaBuf message
 * @return Process ID
 */
uint32_t edgefirst_dmabuf_get_pid(const EdgeFirstDmaBuf* dmabuf);

/**
 * @brief Get fd field
 * @param dmabuf DmaBuf message
 * @return File descriptor
 */
int32_t edgefirst_dmabuf_get_fd(const EdgeFirstDmaBuf* dmabuf);

/**
 * @brief Get width field
 * @param dmabuf DmaBuf message
 * @return Image width in pixels
 */
uint32_t edgefirst_dmabuf_get_width(const EdgeFirstDmaBuf* dmabuf);

/**
 * @brief Get height field
 * @param dmabuf DmaBuf message
 * @return Image height in pixels
 */
uint32_t edgefirst_dmabuf_get_height(const EdgeFirstDmaBuf* dmabuf);

/**
 * @brief Get stride field
 * @param dmabuf DmaBuf message
 * @return Row stride in bytes
 */
uint32_t edgefirst_dmabuf_get_stride(const EdgeFirstDmaBuf* dmabuf);

/**
 * @brief Get fourcc field
 * @param dmabuf DmaBuf message
 * @return FOURCC pixel format code
 */
uint32_t edgefirst_dmabuf_get_fourcc(const EdgeFirstDmaBuf* dmabuf);

/**
 * @brief Get length field
 * @param dmabuf DmaBuf message
 * @return Buffer length in bytes
 */
uint32_t edgefirst_dmabuf_get_length(const EdgeFirstDmaBuf* dmabuf);

/**
 * @brief Set pid field
 * @param dmabuf DmaBuf message
 * @param pid Process ID
 */
void edgefirst_dmabuf_set_pid(EdgeFirstDmaBuf* dmabuf, uint32_t pid);

/**
 * @brief Set fd field
 * @param dmabuf DmaBuf message
 * @param fd File descriptor
 */
void edgefirst_dmabuf_set_fd(EdgeFirstDmaBuf* dmabuf, int32_t fd);

/**
 * @brief Set width field
 * @param dmabuf DmaBuf message
 * @param width Image width
 */
void edgefirst_dmabuf_set_width(EdgeFirstDmaBuf* dmabuf, uint32_t width);

/**
 * @brief Set height field
 * @param dmabuf DmaBuf message
 * @param height Image height
 */
void edgefirst_dmabuf_set_height(EdgeFirstDmaBuf* dmabuf, uint32_t height);

/**
 * @brief Set stride field
 * @param dmabuf DmaBuf message
 * @param stride Row stride
 */
void edgefirst_dmabuf_set_stride(EdgeFirstDmaBuf* dmabuf, uint32_t stride);

/**
 * @brief Set fourcc field
 * @param dmabuf DmaBuf message
 * @param fourcc FOURCC code
 */
void edgefirst_dmabuf_set_fourcc(EdgeFirstDmaBuf* dmabuf, uint32_t fourcc);

/**
 * @brief Set length field
 * @param dmabuf DmaBuf message
 * @param length Buffer length
 */
void edgefirst_dmabuf_set_length(EdgeFirstDmaBuf* dmabuf, uint32_t length);

/**
 * @brief Serialize DmaBuf to CDR format
 * @param dmabuf DmaBuf message to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int edgefirst_dmabuf_serialize(const EdgeFirstDmaBuf* dmabuf, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize DmaBuf from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized DmaBuf or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
EdgeFirstDmaBuf* edgefirst_dmabuf_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * Constants
 * ========================================================================= */

/* radar_cube_dimension */
#define EDGEFIRST_RADAR_CUBE_DIMENSION_UNDEFINED   0
#define EDGEFIRST_RADAR_CUBE_DIMENSION_RANGE       1
#define EDGEFIRST_RADAR_CUBE_DIMENSION_DOPPLER     2
#define EDGEFIRST_RADAR_CUBE_DIMENSION_AZIMUTH     3
#define EDGEFIRST_RADAR_CUBE_DIMENSION_ELEVATION   4
#define EDGEFIRST_RADAR_CUBE_DIMENSION_RXCHANNEL   5
#define EDGEFIRST_RADAR_CUBE_DIMENSION_SEQUENCE    6

/* model_info data types */
#define EDGEFIRST_MODEL_INFO_RAW       0
#define EDGEFIRST_MODEL_INFO_INT8      1
#define EDGEFIRST_MODEL_INFO_UINT8     2
#define EDGEFIRST_MODEL_INFO_INT16     3
#define EDGEFIRST_MODEL_INFO_UINT16    4
#define EDGEFIRST_MODEL_INFO_FLOAT16   5
#define EDGEFIRST_MODEL_INFO_INT32     6
#define EDGEFIRST_MODEL_INFO_UINT32    7
#define EDGEFIRST_MODEL_INFO_FLOAT32   8
#define EDGEFIRST_MODEL_INFO_INT64     9
#define EDGEFIRST_MODEL_INFO_UINT64    10
#define EDGEFIRST_MODEL_INFO_FLOAT64   11
#define EDGEFIRST_MODEL_INFO_STRING    12

/* point_field data types */
#define ROS_POINT_FIELD_INT8     1
#define ROS_POINT_FIELD_UINT8    2
#define ROS_POINT_FIELD_INT16    3
#define ROS_POINT_FIELD_UINT16   4
#define ROS_POINT_FIELD_INT32    5
#define ROS_POINT_FIELD_UINT32   6
#define ROS_POINT_FIELD_FLOAT32  7
#define ROS_POINT_FIELD_FLOAT64  8

/* nav_sat_fix covariance types */
#define ROS_NAV_SAT_FIX_COVARIANCE_TYPE_UNKNOWN          0
#define ROS_NAV_SAT_FIX_COVARIANCE_TYPE_APPROXIMATED     1
#define ROS_NAV_SAT_FIX_COVARIANCE_TYPE_DIAGONAL_KNOWN   2
#define ROS_NAV_SAT_FIX_COVARIANCE_TYPE_KNOWN            3

/* nav_sat_status constants */
#define ROS_NAV_SAT_STATUS_STATUS_NO_FIX     -1
#define ROS_NAV_SAT_STATUS_STATUS_FIX        0
#define ROS_NAV_SAT_STATUS_STATUS_SBAS_FIX   1
#define ROS_NAV_SAT_STATUS_STATUS_GBAS_FIX   2
#define ROS_NAV_SAT_STATUS_SERVICE_GPS       1
#define ROS_NAV_SAT_STATUS_SERVICE_GLONASS   2
#define ROS_NAV_SAT_STATUS_SERVICE_COMPASS   4
#define ROS_NAV_SAT_STATUS_SERVICE_GALILEO   8

/* foxglove point annotation types */
#define FOXGLOVE_POINT_ANNOTATION_UNKNOWN    0
#define FOXGLOVE_POINT_ANNOTATION_POINTS     1
#define FOXGLOVE_POINT_ANNOTATION_LINE_LOOP  2
#define FOXGLOVE_POINT_ANNOTATION_LINE_STRIP 3
#define FOXGLOVE_POINT_ANNOTATION_LINE_LIST  4
/* ============================================================================
 * TIER 1 API - Most Used Types
 * ========================================================================= */

/* ============================================================================
 * edgefirst_msgs - DetectTrack
 * ========================================================================= */

/**
 * @brief Create a new DetectTrack
 * @return Pointer to new DetectTrack or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
EdgeFirstDetectTrack* edgefirst_detecttrack_new(void);

/**
 * @brief Free a DetectTrack
 * @param track Pointer to DetectTrack to free (can be NULL)
 */
void edgefirst_detecttrack_free(EdgeFirstDetectTrack* track);

/**
 * @brief Get track ID (caller must free returned string)
 * @param track DetectTrack
 * @return Newly allocated C string or NULL on error
 */
char* edgefirst_detecttrack_get_id(const EdgeFirstDetectTrack* track);

/**
 * @brief Get track lifetime in frames
 * @param track DetectTrack
 * @return Lifetime in frames
 */
int32_t edgefirst_detecttrack_get_lifetime(const EdgeFirstDetectTrack* track);

/**
 * @brief Get mutable created timestamp
 * @param track DetectTrack
 * @return Mutable pointer to created Time
 */
RosTime* edgefirst_detecttrack_get_created_mut(EdgeFirstDetectTrack* track);

/**
 * @brief Set track ID
 * @param track DetectTrack
 * @param id Track ID string
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer or invalid UTF-8)
 * - ENOMEM: Memory allocation failed
 */
int edgefirst_detecttrack_set_id(EdgeFirstDetectTrack* track, const char* id);

/**
 * @brief Set track lifetime
 * @param track DetectTrack
 * @param lifetime Lifetime in frames
 */
void edgefirst_detecttrack_set_lifetime(EdgeFirstDetectTrack* track, int32_t lifetime);

/**
 * @brief Serialize DetectTrack to CDR format
 * @param track DetectTrack to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int edgefirst_detecttrack_serialize(const EdgeFirstDetectTrack* track, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize DetectTrack from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized DetectTrack or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
EdgeFirstDetectTrack* edgefirst_detecttrack_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * edgefirst_msgs - DetectBox2D
 * ========================================================================= */

/**
 * @brief Create a new DetectBox2D
 * @return Pointer to new DetectBox2D or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
EdgeFirstDetectBox2D* edgefirst_detectbox2d_new(void);

/**
 * @brief Free a DetectBox2D
 * @param box DetectBox2D to free (can be NULL)
 */
void edgefirst_detectbox2d_free(EdgeFirstDetectBox2D* box);

/**
 * @brief Get center X coordinate
 * @param box DetectBox2D
 * @return Center X coordinate (normalized 0.0-1.0)
 */
float edgefirst_detectbox2d_get_center_x(const EdgeFirstDetectBox2D* box);

/**
 * @brief Get center Y coordinate
 * @param box DetectBox2D
 * @return Center Y coordinate (normalized 0.0-1.0)
 */
float edgefirst_detectbox2d_get_center_y(const EdgeFirstDetectBox2D* box);

/**
 * @brief Get box width
 * @param box DetectBox2D
 * @return Box width (normalized 0.0-1.0)
 */
float edgefirst_detectbox2d_get_width(const EdgeFirstDetectBox2D* box);

/**
 * @brief Get box height
 * @param box DetectBox2D
 * @return Box height (normalized 0.0-1.0)
 */
float edgefirst_detectbox2d_get_height(const EdgeFirstDetectBox2D* box);

/**
 * @brief Get object label (caller must free returned string)
 * @param box DetectBox2D
 * @return Newly allocated label string or NULL on error
 */
char* edgefirst_detectbox2d_get_label(const EdgeFirstDetectBox2D* box);

/**
 * @brief Get detection confidence score
 * @param box DetectBox2D
 * @return Confidence score (0.0-1.0)
 */
float edgefirst_detectbox2d_get_score(const EdgeFirstDetectBox2D* box);

/**
 * @brief Get object distance
 * @param box DetectBox2D
 * @return Distance in meters
 */
float edgefirst_detectbox2d_get_distance(const EdgeFirstDetectBox2D* box);

/**
 * @brief Get object speed
 * @param box DetectBox2D
 * @return Speed in m/s
 */
float edgefirst_detectbox2d_get_speed(const EdgeFirstDetectBox2D* box);

/**
 * @brief Get mutable tracking information
 * @param box DetectBox2D
 * @return Mutable pointer to DetectTrack
 */
EdgeFirstDetectTrack* edgefirst_detectbox2d_get_track_mut(EdgeFirstDetectBox2D* box);

void edgefirst_detectbox2d_set_center_x(EdgeFirstDetectBox2D* box, float center_x);
void edgefirst_detectbox2d_set_center_y(EdgeFirstDetectBox2D* box, float center_y);
void edgefirst_detectbox2d_set_width(EdgeFirstDetectBox2D* box, float width);
void edgefirst_detectbox2d_set_height(EdgeFirstDetectBox2D* box, float height);

/**
 * @brief Set object label
 * @param box DetectBox2D
 * @param label Label string
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer or invalid UTF-8)
 * - ENOMEM: Memory allocation failed
 */
int edgefirst_detectbox2d_set_label(EdgeFirstDetectBox2D* box, const char* label);

void edgefirst_detectbox2d_set_score(EdgeFirstDetectBox2D* box, float score);
void edgefirst_detectbox2d_set_distance(EdgeFirstDetectBox2D* box, float distance);
void edgefirst_detectbox2d_set_speed(EdgeFirstDetectBox2D* box, float speed);

/**
 * @brief Serialize DetectBox2D to CDR format
 * @param box DetectBox2D to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int edgefirst_detectbox2d_serialize(const EdgeFirstDetectBox2D* box, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize DetectBox2D from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized DetectBox2D or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
EdgeFirstDetectBox2D* edgefirst_detectbox2d_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * edgefirst_msgs - Detect
 * ========================================================================= */

/**
 * @brief Create a new Detect message
 * @return Pointer to new Detect or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
EdgeFirstDetect* edgefirst_detect_new(void);

/**
 * @brief Free a Detect message
 * @param detect Detect to free (can be NULL)
 */
void edgefirst_detect_free(EdgeFirstDetect* detect);

/**
 * @brief Get mutable header
 * @param detect Detect message
 * @return Mutable pointer to Header
 */
RosHeader* edgefirst_detect_get_header_mut(EdgeFirstDetect* detect);

/**
 * @brief Get mutable input timestamp
 * @param detect Detect message
 * @return Mutable pointer to input_timestamp
 */
RosTime* edgefirst_detect_get_input_timestamp_mut(EdgeFirstDetect* detect);

/**
 * @brief Get mutable model time
 * @param detect Detect message
 * @return Mutable pointer to model_time
 */
RosTime* edgefirst_detect_get_model_time_mut(EdgeFirstDetect* detect);

/**
 * @brief Get mutable output time
 * @param detect Detect message
 * @return Mutable pointer to output_time
 */
RosTime* edgefirst_detect_get_output_time_mut(EdgeFirstDetect* detect);

/**
 * @brief Get detection boxes array
 * @param detect Detect message
 * @param out_len Output: number of boxes in array
 * @return Borrowed pointer to boxes array (valid during detect lifetime)
 */
const EdgeFirstDetectBox2D* edgefirst_detect_get_boxes(const EdgeFirstDetect* detect, size_t* out_len);

/**
 * @brief Add a detection box (copies box data)
 * @param detect Detect message
 * @param box Box to add
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int edgefirst_detect_add_box(EdgeFirstDetect* detect, const EdgeFirstDetectBox2D* box);

/**
 * @brief Serialize Detect to CDR format
 * @param detect Detect to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int edgefirst_detect_serialize(const EdgeFirstDetect* detect, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize Detect from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized Detect or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
EdgeFirstDetect* edgefirst_detect_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * edgefirst_msgs - Mask
 * ========================================================================= */

/**
 * @brief Create a new Mask
 * @return Pointer to new Mask or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
EdgeFirstMask* edgefirst_mask_new(void);

/**
 * @brief Free a Mask
 * @param mask Mask to free (can be NULL)
 */
void edgefirst_mask_free(EdgeFirstMask* mask);

/**
 * @brief Get mask height
 * @param mask Mask
 * @return Height in pixels
 */
uint32_t edgefirst_mask_get_height(const EdgeFirstMask* mask);

/**
 * @brief Get mask width
 * @param mask Mask
 * @return Width in pixels
 */
uint32_t edgefirst_mask_get_width(const EdgeFirstMask* mask);

/**
 * @brief Get mask length (for 3D masks)
 * @param mask Mask
 * @return Length (depth dimension)
 */
uint32_t edgefirst_mask_get_length(const EdgeFirstMask* mask);

/**
 * @brief Get mask encoding (caller must free returned string)
 * @param mask Mask
 * @return Encoding string ("" for raw, "zstd" for compressed) or NULL on error
 */
char* edgefirst_mask_get_encoding(const EdgeFirstMask* mask);

/**
 * @brief Get mask data array
 * @param mask Mask
 * @param out_len Output: number of bytes in array
 * @return Borrowed pointer to mask data (valid during mask lifetime)
 */
const uint8_t* edgefirst_mask_get_mask(const EdgeFirstMask* mask, size_t* out_len);

/**
 * @brief Get boxed flag (for instance segmentation)
 * @param mask Mask
 * @return true if mask is boxed instance segmentation
 */
bool edgefirst_mask_get_boxed(const EdgeFirstMask* mask);

void edgefirst_mask_set_height(EdgeFirstMask* mask, uint32_t height);
void edgefirst_mask_set_width(EdgeFirstMask* mask, uint32_t width);
void edgefirst_mask_set_length(EdgeFirstMask* mask, uint32_t length);

/**
 * @brief Set mask encoding
 * @param mask Mask
 * @param encoding Encoding string ("" for raw, "zstd" for compressed)
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer or invalid UTF-8)
 * - ENOMEM: Memory allocation failed
 */
int edgefirst_mask_set_encoding(EdgeFirstMask* mask, const char* encoding);

/**
 * @brief Set mask data (copies data)
 * @param mask Mask
 * @param data Mask data bytes
 * @param len Length of data
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int edgefirst_mask_set_mask(EdgeFirstMask* mask, const uint8_t* data, size_t len);

void edgefirst_mask_set_boxed(EdgeFirstMask* mask, bool boxed);

/**
 * @brief Serialize Mask to CDR format
 * @param mask Mask to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int edgefirst_mask_serialize(const EdgeFirstMask* mask, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize Mask from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized Mask or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
EdgeFirstMask* edgefirst_mask_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * sensor_msgs - PointField
 * ========================================================================= */

/* Point field data types (ROS2 sensor_msgs/PointField constants) */
#define ROS_POINT_FIELD_INT8    1
#define ROS_POINT_FIELD_UINT8   2
#define ROS_POINT_FIELD_INT16   3
#define ROS_POINT_FIELD_UINT16  4
#define ROS_POINT_FIELD_INT32   5
#define ROS_POINT_FIELD_UINT32  6
#define ROS_POINT_FIELD_FLOAT32 7
#define ROS_POINT_FIELD_FLOAT64 8

/**
 * @brief Create a new PointField
 * @return Pointer to new PointField or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
RosPointField* ros_point_field_new(void);

/**
 * @brief Free a PointField
 * @param field PointField to free (can be NULL)
 */
void ros_point_field_free(RosPointField* field);

/**
 * @brief Get field name (caller must free returned string)
 * @param field PointField
 * @return Newly allocated field name string or NULL on error
 */
char* ros_point_field_get_name(const RosPointField* field);

/**
 * @brief Get field offset in bytes
 * @param field PointField
 * @return Offset in point structure
 */
uint32_t ros_point_field_get_offset(const RosPointField* field);

/**
 * @brief Get field data type
 * @param field PointField
 * @return Data type constant (ROS_POINT_FIELD_*)
 */
uint8_t ros_point_field_get_datatype(const RosPointField* field);

/**
 * @brief Get field count
 * @param field PointField
 * @return Number of elements (1 for scalars, >1 for arrays)
 */
uint32_t ros_point_field_get_count(const RosPointField* field);

/**
 * @brief Set field name
 * @param field PointField
 * @param name Field name string
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer or invalid UTF-8)
 * - ENOMEM: Memory allocation failed
 */
int ros_point_field_set_name(RosPointField* field, const char* name);

void ros_point_field_set_offset(RosPointField* field, uint32_t offset);
void ros_point_field_set_datatype(RosPointField* field, uint8_t datatype);
void ros_point_field_set_count(RosPointField* field, uint32_t count);

/**
 * @brief Serialize PointField to CDR format
 * @param field PointField to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_point_field_serialize(const RosPointField* field, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize PointField from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized PointField or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
RosPointField* ros_point_field_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * sensor_msgs - PointCloud2
 * ========================================================================= */

/**
 * @brief Create a new PointCloud2
 * @return Pointer to new PointCloud2 or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
RosPointCloud2* ros_point_cloud2_new(void);

/**
 * @brief Free a PointCloud2
 * @param cloud PointCloud2 to free (can be NULL)
 */
void ros_point_cloud2_free(RosPointCloud2* cloud);

/**
 * @brief Get mutable header
 * @param cloud PointCloud2
 * @return Mutable pointer to Header
 */
RosHeader* ros_point_cloud2_get_header_mut(RosPointCloud2* cloud);

/**
 * @brief Get cloud height (number of rows for organized clouds)
 * @param cloud PointCloud2
 * @return Height in rows (1 for unorganized clouds)
 */
uint32_t ros_point_cloud2_get_height(const RosPointCloud2* cloud);

/**
 * @brief Get cloud width (number of points per row)
 * @param cloud PointCloud2
 * @return Width in points
 */
uint32_t ros_point_cloud2_get_width(const RosPointCloud2* cloud);

/**
 * @brief Get point fields array
 * @param cloud PointCloud2
 * @param out_len Output: number of fields in array
 * @return Borrowed pointer to fields array (valid during cloud lifetime)
 */
const RosPointField* ros_point_cloud2_get_fields(const RosPointCloud2* cloud, size_t* out_len);

/**
 * @brief Get big-endian flag
 * @param cloud PointCloud2
 * @return true if data is big-endian
 */
bool ros_point_cloud2_get_is_bigendian(const RosPointCloud2* cloud);

/**
 * @brief Get point step (size of one point in bytes)
 * @param cloud PointCloud2
 * @return Point step in bytes
 */
uint32_t ros_point_cloud2_get_point_step(const RosPointCloud2* cloud);

/**
 * @brief Get row step (size of one row in bytes)
 * @param cloud PointCloud2
 * @return Row step in bytes
 */
uint32_t ros_point_cloud2_get_row_step(const RosPointCloud2* cloud);

/**
 * @brief Get point cloud data array
 * @param cloud PointCloud2
 * @param out_len Output: number of bytes in array
 * @return Borrowed pointer to data (valid during cloud lifetime)
 */
const uint8_t* ros_point_cloud2_get_data(const RosPointCloud2* cloud, size_t* out_len);

/**
 * @brief Get dense flag (true if no invalid points)
 * @param cloud PointCloud2
 * @return true if cloud contains no invalid points
 */
bool ros_point_cloud2_get_is_dense(const RosPointCloud2* cloud);

void ros_point_cloud2_set_height(RosPointCloud2* cloud, uint32_t height);
void ros_point_cloud2_set_width(RosPointCloud2* cloud, uint32_t width);

/**
 * @brief Add a point field (copies field data)
 * @param cloud PointCloud2
 * @param field Field to add
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_point_cloud2_add_field(RosPointCloud2* cloud, const RosPointField* field);

void ros_point_cloud2_set_is_bigendian(RosPointCloud2* cloud, bool is_bigendian);
void ros_point_cloud2_set_point_step(RosPointCloud2* cloud, uint32_t point_step);
void ros_point_cloud2_set_row_step(RosPointCloud2* cloud, uint32_t row_step);

/**
 * @brief Set point cloud data (copies data)
 * @param cloud PointCloud2
 * @param data Point data bytes
 * @param len Length of data
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_point_cloud2_set_data(RosPointCloud2* cloud, const uint8_t* data, size_t len);

void ros_point_cloud2_set_is_dense(RosPointCloud2* cloud, bool is_dense);

/**
 * @brief Serialize PointCloud2 to CDR format
 * @param cloud PointCloud2 to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_point_cloud2_serialize(const RosPointCloud2* cloud, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize PointCloud2 from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized PointCloud2 or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
RosPointCloud2* ros_point_cloud2_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * sensor_msgs - NavSatStatus
 * ========================================================================= */

/* NavSatStatus service constants (ROS2 sensor_msgs/NavSatStatus) */
#define ROS_NAV_SAT_STATUS_SERVICE_GPS     1
#define ROS_NAV_SAT_STATUS_SERVICE_GLONASS 2
#define ROS_NAV_SAT_STATUS_SERVICE_COMPASS 4
#define ROS_NAV_SAT_STATUS_SERVICE_GALILEO 8

/* NavSatStatus status constants */
#define ROS_NAV_SAT_STATUS_STATUS_NO_FIX   -1
#define ROS_NAV_SAT_STATUS_STATUS_FIX       0
#define ROS_NAV_SAT_STATUS_STATUS_SBAS_FIX  1
#define ROS_NAV_SAT_STATUS_STATUS_GBAS_FIX  2

/**
 * @brief Create a new NavSatStatus
 * @return Pointer to new NavSatStatus or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
RosNavSatStatus* ros_nav_sat_status_new(void);

/**
 * @brief Free a NavSatStatus
 * @param status NavSatStatus to free (can be NULL)
 */
void ros_nav_sat_status_free(RosNavSatStatus* status);

/**
 * @brief Get status value
 * @param status NavSatStatus
 * @return Status constant (ROS_NAV_SAT_STATUS_STATUS_*)
 */
int16_t ros_nav_sat_status_get_status(const RosNavSatStatus* status);

/**
 * @brief Get service value
 * @param status NavSatStatus
 * @return Service bitmask (ROS_NAV_SAT_STATUS_SERVICE_*)
 */
uint16_t ros_nav_sat_status_get_service(const RosNavSatStatus* status);

void ros_nav_sat_status_set_status(RosNavSatStatus* status, int16_t value);
void ros_nav_sat_status_set_service(RosNavSatStatus* status, uint16_t value);

/**
 * @brief Serialize NavSatStatus to CDR format
 * @param status NavSatStatus to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_nav_sat_status_serialize(const RosNavSatStatus* status, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize NavSatStatus from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized NavSatStatus or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
RosNavSatStatus* ros_nav_sat_status_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * sensor_msgs - NavSatFix
 * ========================================================================= */

/* NavSatFix covariance type constants (ROS2 sensor_msgs/NavSatFix) */
#define ROS_NAV_SAT_FIX_COVARIANCE_TYPE_UNKNOWN        0
#define ROS_NAV_SAT_FIX_COVARIANCE_TYPE_APPROXIMATED   1
#define ROS_NAV_SAT_FIX_COVARIANCE_TYPE_DIAGONAL_KNOWN 2
#define ROS_NAV_SAT_FIX_COVARIANCE_TYPE_KNOWN          3

/**
 * @brief Create a new NavSatFix
 * @return Pointer to new NavSatFix or NULL on allocation failure
 *
 * @par Errors (errno):
 * - ENOMEM: Memory allocation failed
 */
RosNavSatFix* ros_nav_sat_fix_new(void);

/**
 * @brief Free a NavSatFix
 * @param fix NavSatFix to free (can be NULL)
 */
void ros_nav_sat_fix_free(RosNavSatFix* fix);

/**
 * @brief Get mutable header
 * @param fix NavSatFix
 * @return Mutable pointer to Header
 */
RosHeader* ros_nav_sat_fix_get_header_mut(RosNavSatFix* fix);

/**
 * @brief Get mutable status
 * @param fix NavSatFix
 * @return Mutable pointer to NavSatStatus
 */
RosNavSatStatus* ros_nav_sat_fix_get_status_mut(RosNavSatFix* fix);

/**
 * @brief Get latitude
 * @param fix NavSatFix
 * @return Latitude in degrees
 */
double ros_nav_sat_fix_get_latitude(const RosNavSatFix* fix);

/**
 * @brief Get longitude
 * @param fix NavSatFix
 * @return Longitude in degrees
 */
double ros_nav_sat_fix_get_longitude(const RosNavSatFix* fix);

/**
 * @brief Get altitude
 * @param fix NavSatFix
 * @return Altitude in meters (above WGS84 ellipsoid)
 */
double ros_nav_sat_fix_get_altitude(const RosNavSatFix* fix);

/**
 * @brief Get position covariance matrix
 * @param fix NavSatFix
 * @return Borrowed pointer to 9-element covariance array (valid during fix lifetime)
 * @note Array is 3x3 row-major: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
 */
const double* ros_nav_sat_fix_get_position_covariance(const RosNavSatFix* fix);

/**
 * @brief Get position covariance type
 * @param fix NavSatFix
 * @return Covariance type constant (ROS_NAV_SAT_FIX_COVARIANCE_TYPE_*)
 */
uint8_t ros_nav_sat_fix_get_position_covariance_type(const RosNavSatFix* fix);

void ros_nav_sat_fix_set_latitude(RosNavSatFix* fix, double latitude);
void ros_nav_sat_fix_set_longitude(RosNavSatFix* fix, double longitude);
void ros_nav_sat_fix_set_altitude(RosNavSatFix* fix, double altitude);

/**
 * @brief Set position covariance matrix (copies 9 elements)
 * @param fix NavSatFix
 * @param covariance 9-element array (3x3 row-major)
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 */
int ros_nav_sat_fix_set_position_covariance(RosNavSatFix* fix, const double* covariance);

void ros_nav_sat_fix_set_position_covariance_type(RosNavSatFix* fix, uint8_t type);

/**
 * @brief Serialize NavSatFix to CDR format
 * @param fix NavSatFix to serialize
 * @param out_bytes Output buffer pointer (allocated by function, caller must free)
 * @param out_len Output buffer length
 * @return 0 on success, -1 on error
 *
 * @par Errors (errno):
 * - EINVAL: Invalid parameter (NULL pointer)
 * - ENOMEM: Memory allocation failed
 */
int ros_nav_sat_fix_serialize(const RosNavSatFix* fix, uint8_t** out_bytes, size_t* out_len);

/**
 * @brief Deserialize NavSatFix from CDR format
 * @param bytes CDR encoded bytes
 * @param len Length of bytes
 * @return Pointer to deserialized NavSatFix or NULL on error
 *
 * @par Errors (errno):
 * - EINVAL: bytes is NULL or len is 0
 * - EBADMSG: CDR decoding failed (malformed data)
 * - ENOMEM: Memory allocation failed
 */
RosNavSatFix* ros_nav_sat_fix_deserialize(const uint8_t* bytes, size_t len);

/* ============================================================================
 * Note: Additional types follow similar patterns
 * 
 * This header provides the core types most commonly used. Additional message
 * types (CameraInfo, IMU, PointCloud2, RadarCube, Detect, Model, etc.) follow
 * the same API pattern:
 * - <type>_new() - allocate
 * - <type>_free() - deallocate
 * - <type>_get_<field>() - getter (returns copy for primitives, borrowed ptr
 *   for nested objects, allocated string for String fields)
 * - <type>_get_<field>_mut() - mutable getter for nested objects
 * - <type>_set_<field>() - setter (copies data)
 * - <type>_serialize() - serialize to CDR
 * - <type>_deserialize() - deserialize from CDR
 * 
 * For complete API documentation of all types, see implementation or generate
 * Doxygen documentation.
 * ========================================================================= */

#ifdef __cplusplus
}
#endif

#endif /* EDGEFIRST_SCHEMAS_H */
