/**
 * @file example.c
 * @brief Example usage of EdgeFirst Schemas C API
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.
 *
 * Demonstrates:
 * - CdrFixed encode/decode (Time, Vector3) into stack buffers
 * - Buffer-backed encode (Header, Image) with allocated output
 * - Buffer-backed decode from CDR bytes via opaque view handles
 * - Proper memory management (ros_bytes_free, _free, borrowed pointers)
 * - errno-based error handling
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include "../../include/edgefirst/schemas.h"

void example_time(void) {
    printf("\n=== Example: Time (CdrFixed) ===\n");

    uint8_t buf[64];
    size_t written = 0;

    // Encode Time into a stack buffer
    int ret = ros_time_encode(buf, sizeof(buf), &written, 1234567890, 123456789);
    assert(ret == 0);
    printf("Encoded Time: %zu CDR bytes\n", written);

    // Decode individual fields
    int32_t sec = 0;
    uint32_t nanosec = 0;
    ret = ros_time_decode(buf, written, &sec, &nanosec);
    assert(ret == 0);
    printf("Decoded: %d.%09u\n", sec, nanosec);
    assert(sec == 1234567890 && nanosec == 123456789);

    printf("Time example completed successfully!\n");
}

void example_vector3(void) {
    printf("\n=== Example: Vector3 (CdrFixed) ===\n");

    uint8_t buf[64];
    size_t written = 0;

    // Encode Vector3 into a stack buffer
    int ret = ros_vector3_encode(buf, sizeof(buf), &written, 1.5, 2.5, 3.5);
    assert(ret == 0);
    printf("Encoded Vector3: %zu CDR bytes\n", written);

    // Decode
    double x, y, z;
    ret = ros_vector3_decode(buf, written, &x, &y, &z);
    assert(ret == 0);
    printf("Decoded: (%.1f, %.1f, %.1f)\n", x, y, z);
    assert(x == 1.5 && y == 2.5 && z == 3.5);

    printf("Vector3 example completed successfully!\n");
}

void example_header(void) {
    printf("\n=== Example: Header (buffer-backed) ===\n");

    // Encode a Header — allocates output bytes
    uint8_t* bytes = NULL;
    size_t len = 0;
    int ret = ros_header_encode(&bytes, &len, 1234567890, 123456789, "camera_frame");
    assert(ret == 0);
    printf("Encoded Header: %zu CDR bytes\n", len);

    // Decode from CDR into an opaque view handle
    ros_header_t* hdr = ros_header_from_cdr(bytes, len);
    assert(hdr != NULL);

    // Access fields — borrowed pointers, do NOT free
    int32_t sec = ros_header_get_stamp_sec(hdr);
    uint32_t nanosec = ros_header_get_stamp_nanosec(hdr);
    const char* frame_id = ros_header_get_frame_id(hdr);

    printf("Decoded: stamp=%d.%09u frame_id=\"%s\"\n", sec, nanosec, frame_id);
    assert(sec == 1234567890);
    assert(nanosec == 123456789);
    assert(strcmp(frame_id, "camera_frame") == 0);

    // Cleanup: free the handle, then the encoded bytes
    ros_header_free(hdr);
    ros_bytes_free(bytes, len);  // NOT free(bytes)

    printf("Header example completed successfully!\n");
}

void example_image(void) {
    printf("\n=== Example: Image (buffer-backed) ===\n");

    // Create dummy pixel data
    size_t data_size = 640 * 480 * 3;  // VGA RGB8
    uint8_t* pixel_data = (uint8_t*)malloc(data_size);
    for (size_t i = 0; i < data_size; i++) {
        pixel_data[i] = (uint8_t)(i % 256);
    }

    // Encode an Image — all fields in one call
    uint8_t* bytes = NULL;
    size_t len = 0;
    int ret = ros_image_encode(&bytes, &len,
        1000, 500000, "camera",
        480, 640, "rgb8", 0, 640 * 3,
        pixel_data, data_size);
    assert(ret == 0);
    free(pixel_data);  // Our source data, normal free
    printf("Encoded Image: %zu CDR bytes\n", len);

    // Decode from CDR
    ros_image_t* img = ros_image_from_cdr(bytes, len);
    assert(img != NULL);

    // Access fields
    uint32_t width = ros_image_get_width(img);
    uint32_t height = ros_image_get_height(img);
    const char* encoding = ros_image_get_encoding(img);  // borrowed

    size_t retrieved_len = 0;
    (void)ros_image_get_data(img, &retrieved_len);  // borrowed, used for length check

    printf("Decoded: %ux%u encoding=\"%s\" data=%zu bytes\n",
           width, height, encoding, retrieved_len);

    assert(width == 640 && height == 480);
    assert(strcmp(encoding, "rgb8") == 0);
    assert(retrieved_len == data_size);

    // Forward the raw CDR bytes (zero re-serialization cost)
    size_t cdr_len;
    const uint8_t* cdr = ros_image_as_cdr(img, &cdr_len);
    printf("CDR bytes available for forwarding: %zu bytes at %p\n",
           cdr_len, (const void*)cdr);

    // Cleanup
    ros_image_free(img);
    ros_bytes_free(bytes, len);

    printf("Image example completed successfully!\n");
}

void example_dmabuffer(void) {
    printf("\n=== Example: DmaBuffer (buffer-backed) ===\n");

    // Encode a DmaBuffer
    uint8_t* bytes = NULL;
    size_t len = 0;
    int ret = ros_dmabuffer_encode(&bytes, &len,
        1000, 500000, "camera0",
        12345, 42,               // pid, fd
        1920, 1080,              // width, height
        1920 * 2,                // stride (YUYV)
        0x56595559,              // fourcc = 'YUYV'
        1920 * 1080 * 2);       // length
    assert(ret == 0);
    printf("Encoded DmaBuffer: %zu CDR bytes\n", len);

    // Decode
    ros_dmabuffer_t* dmabuf = ros_dmabuffer_from_cdr(bytes, len);
    assert(dmabuf != NULL);

    uint32_t width = ros_dmabuffer_get_width(dmabuf);
    uint32_t height = ros_dmabuffer_get_height(dmabuf);
    uint32_t pid = ros_dmabuffer_get_pid(dmabuf);
    int32_t fd = ros_dmabuffer_get_fd(dmabuf);

    printf("Decoded: %ux%u pid=%u fd=%d\n", width, height, pid, fd);
    assert(width == 1920 && height == 1080);
    assert(pid == 12345 && fd == 42);

    ros_dmabuffer_free(dmabuf);
    ros_bytes_free(bytes, len);

    printf("DmaBuffer example completed successfully!\n");
}

void example_error_handling(void) {
    printf("\n=== Example: Error Handling ===\n");

    // NULL data → EINVAL
    errno = 0;
    ros_header_t* hdr = ros_header_from_cdr(NULL, 100);
    assert(hdr == NULL);
    printf("NULL data: errno=%d (%s)\n", errno, strerror(errno));
    assert(errno == EINVAL);

    // Corrupt data → EBADMSG
    errno = 0;
    uint8_t bad[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    hdr = ros_header_from_cdr(bad, sizeof(bad));
    assert(hdr == NULL);
    printf("Bad data:  errno=%d (%s)\n", errno, strerror(errno));
    assert(errno == EBADMSG);

    // CdrFixed buffer too small → ENOBUFS
    errno = 0;
    uint8_t tiny[2];
    size_t written;
    int ret = ros_time_encode(tiny, sizeof(tiny), &written, 42, 0);
    assert(ret == -1);
    printf("Too small: errno=%d (%s)\n", errno, strerror(errno));
    assert(errno == ENOBUFS);

    printf("Error handling examples completed successfully!\n");
}

int main(void) {
    printf("EdgeFirst Schemas C API Examples\n");
    printf("=================================\n");

    example_time();
    example_vector3();
    example_header();
    example_image();
    example_dmabuffer();
    example_error_handling();

    printf("\n=================================\n");
    printf("All examples completed successfully!\n");

    return 0;
}
