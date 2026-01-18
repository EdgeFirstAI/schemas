/**
 * @file example.c
 * @brief Example usage of EdgeFirst Schemas C API
 * 
 * SPDX-License-Identifier: Apache-2.0
 * Copyright © 2025 Au-Zone Technologies. All Rights Reserved.
 * 
 * Demonstrates:
 * - Creating message structures
 * - Setting field values
 * - Serializing to CDR format
 * - Deserializing from CDR format
 * - Proper memory management
 * - errno-based error handling
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include "../../include/edgefirst/schemas.h"

void example_header() {
    printf("\n=== Example: Header Message ===\n");
    
    // Create a new header
    RosHeader* header = ros_header_new();
    if (!header) {
        perror("ros_header_new");
        exit(1);
    }
    
    // Set frame_id
    if (ros_header_set_frame_id(header, "camera_frame") != 0) {
        perror("ros_header_set_frame_id");
        ros_header_free(header);
        exit(1);
    }
    
    // Set timestamp through the stamp field
    RosTime* stamp = ros_header_get_stamp_mut(header);
    ros_time_set_sec(stamp, 1234567890);
    ros_time_set_nanosec(stamp, 123456789);
    
    // Serialize to CDR
    uint8_t* bytes = NULL;
    size_t len = 0;
    if (ros_header_serialize(header, &bytes, &len) != 0) {
        perror("ros_header_serialize");
        ros_header_free(header);
        exit(1);
    }
    printf("Serialized header to %zu bytes\n", len);
    
    // Deserialize
    RosHeader* header2 = ros_header_deserialize(bytes, len);
    if (!header2) {
        fprintf(stderr, "ros_header_deserialize failed: %s\n", strerror(errno));
        free(bytes);
        ros_header_free(header);
        exit(1);
    }
    
    // Verify the data
    const RosTime* stamp2 = ros_header_get_stamp(header2);
    int32_t sec = ros_time_get_sec(stamp2);
    uint32_t nanosec = ros_time_get_nanosec(stamp2);
    printf("Deserialized timestamp: %d.%09u\n", sec, nanosec);
    
    char* frame_id = ros_header_get_frame_id(header2);
    printf("Deserialized frame_id: %s\n", frame_id);
    
    // Cleanup
    free(frame_id);
    free(bytes);
    ros_header_free(header);
    ros_header_free(header2);
    
    printf("Header example completed successfully!\n");
}

void example_vector3() {
    printf("\n=== Example: Vector3 Message ===\n");
    
    // Create a new vector
    RosVector3* vec = ros_vector3_new();
    if (!vec) {
        perror("ros_vector3_new");
        exit(1);
    }
    
    // Set components
    ros_vector3_set_x(vec, 1.5);
    ros_vector3_set_y(vec, 2.5);
    ros_vector3_set_z(vec, 3.5);
    
    // Serialize
    uint8_t* bytes = NULL;
    size_t len = 0;
    if (ros_vector3_serialize(vec, &bytes, &len) != 0) {
        perror("ros_vector3_serialize");
        ros_vector3_free(vec);
        exit(1);
    }
    printf("Serialized vector to %zu bytes\n", len);
    
    // Deserialize
    RosVector3* vec2 = ros_vector3_deserialize(bytes, len);
    if (!vec2) {
        fprintf(stderr, "ros_vector3_deserialize failed: %s\n", strerror(errno));
        free(bytes);
        ros_vector3_free(vec);
        exit(1);
    }
    
    // Verify
    double x = ros_vector3_get_x(vec2);
    double y = ros_vector3_get_y(vec2);
    double z = ros_vector3_get_z(vec2);
    printf("Deserialized vector: (%.1f, %.1f, %.1f)\n", x, y, z);
    
    assert(x == 1.5 && y == 2.5 && z == 3.5);
    
    // Cleanup
    free(bytes);
    ros_vector3_free(vec);
    ros_vector3_free(vec2);
    
    printf("Vector3 example completed successfully!\n");
}

void example_dmabuf() {
    printf("\n=== Example: DmaBuf Message ===\n");
    
    // Create a new DmaBuf
    EdgeFirstDmaBuf* dmabuf = edgefirst_dmabuf_new();
    if (!dmabuf) {
        perror("edgefirst_dmabuf_new");
        exit(1);
    }
    
    // Set header
    RosHeader* header = edgefirst_dmabuf_get_header_mut(dmabuf);
    if (ros_header_set_frame_id(header, "camera0") != 0) {
        perror("ros_header_set_frame_id");
        edgefirst_dmabuf_free(dmabuf);
        exit(1);
    }
    RosTime* stamp = ros_header_get_stamp_mut(header);
    ros_time_set_sec(stamp, 1000);
    ros_time_set_nanosec(stamp, 500000);
    
    // Set DmaBuf-specific fields
    edgefirst_dmabuf_set_pid(dmabuf, 12345);
    edgefirst_dmabuf_set_fd(dmabuf, 42);
    edgefirst_dmabuf_set_width(dmabuf, 1920);
    edgefirst_dmabuf_set_height(dmabuf, 1080);
    edgefirst_dmabuf_set_stride(dmabuf, 1920 * 2); // YUYV format
    edgefirst_dmabuf_set_fourcc(dmabuf, 0x56595559); // 'YUYV'
    edgefirst_dmabuf_set_length(dmabuf, 1920 * 1080 * 2);
    
    // Serialize
    uint8_t* bytes = NULL;
    size_t len = 0;
    if (edgefirst_dmabuf_serialize(dmabuf, &bytes, &len) != 0) {
        perror("edgefirst_dmabuf_serialize");
        edgefirst_dmabuf_free(dmabuf);
        exit(1);
    }
    printf("Serialized DmaBuf to %zu bytes\n", len);
    
    // Deserialize
    EdgeFirstDmaBuf* dmabuf2 = edgefirst_dmabuf_deserialize(bytes, len);
    if (!dmabuf2) {
        fprintf(stderr, "edgefirst_dmabuf_deserialize failed: %s\n", strerror(errno));
        free(bytes);
        edgefirst_dmabuf_free(dmabuf);
        exit(1);
    }
    
    // Verify
    uint32_t width = edgefirst_dmabuf_get_width(dmabuf2);
    uint32_t height = edgefirst_dmabuf_get_height(dmabuf2);
    uint32_t pid = edgefirst_dmabuf_get_pid(dmabuf2);
    int32_t fd = edgefirst_dmabuf_get_fd(dmabuf2);
    
    printf("Deserialized DmaBuf: %ux%u, pid=%u, fd=%d\n", 
           width, height, pid, fd);
    
    assert(width == 1920 && height == 1080);
    assert(pid == 12345 && fd == 42);
    
    // Cleanup
    free(bytes);
    edgefirst_dmabuf_free(dmabuf);
    edgefirst_dmabuf_free(dmabuf2);
    
    printf("DmaBuf example completed successfully!\n");
}

void example_image() {
    printf("\n=== Example: Image Message ===\n");
    
    // Create a new image
    RosImage* image = ros_image_new();
    if (!image) {
        perror("ros_image_new");
        exit(1);
    }
    
    // Set header
    RosHeader* header = ros_image_get_header_mut(image);
    if (ros_header_set_frame_id(header, "camera") != 0) {
        perror("ros_header_set_frame_id");
        ros_image_free(image);
        exit(1);
    }
    
    // Set image properties
    ros_image_set_width(image, 640);
    ros_image_set_height(image, 480);
    if (ros_image_set_encoding(image, "rgb8") != 0) {
        perror("ros_image_set_encoding");
        ros_image_free(image);
        exit(1);
    }
    ros_image_set_is_bigendian(image, 0);
    ros_image_set_step(image, 640 * 3);
    
    // Create dummy image data (small for testing)
    size_t data_size = 100;
    uint8_t* image_data = (uint8_t*)malloc(data_size);
    for (size_t i = 0; i < data_size; i++) {
        image_data[i] = (uint8_t)(i % 256);
    }
    
    if (ros_image_set_data(image, image_data, data_size) != 0) {
        perror("ros_image_set_data");
        free(image_data);
        ros_image_free(image);
        exit(1);
    }
    free(image_data); // Can free after set_data copies it
    
    // Serialize
    uint8_t* bytes = NULL;
    size_t len = 0;
    if (ros_image_serialize(image, &bytes, &len) != 0) {
        perror("ros_image_serialize");
        ros_image_free(image);
        exit(1);
    }
    printf("Serialized Image to %zu bytes\n", len);
    
    // Deserialize
    RosImage* image2 = ros_image_deserialize(bytes, len);
    if (!image2) {
        fprintf(stderr, "ros_image_deserialize failed: %s\n", strerror(errno));
        free(bytes);
        ros_image_free(image);
        exit(1);
    }
    
    // Verify
    uint32_t width = ros_image_get_width(image2);
    uint32_t height = ros_image_get_height(image2);
    char* encoding = ros_image_get_encoding(image2);
    
    size_t retrieved_len = 0;
    const uint8_t* retrieved_data = ros_image_get_data(image2, &retrieved_len);
    
    printf("Deserialized Image: %ux%u, encoding=%s, data_len=%zu\n",
           width, height, encoding, retrieved_len);
    
    assert(width == 640 && height == 480);
    assert(strcmp(encoding, "rgb8") == 0);
    assert(retrieved_len == data_size);
    
    // Cleanup
    free(encoding);
    free(bytes);
    ros_image_free(image);
    ros_image_free(image2);
    
    printf("Image example completed successfully!\n");
}

void example_error_handling() {
    printf("\n=== Example: Error Handling with errno ===\n");
    
    // Test NULL pointer handling
    errno = 0;
    if (ros_header_serialize(NULL, NULL, NULL) != 0) {
        printf("Expected error on NULL pointers: %s\n", strerror(errno));
        assert(errno == EINVAL);
    }
    
    // Test deserialize with NULL pointer
    errno = 0;
    RosHeader* header = ros_header_deserialize(NULL, 100);
    if (!header) {
        printf("Expected error on NULL bytes: %s\n", strerror(errno));
        assert(errno == EINVAL);
    }
    
    // Test deserialize with zero length
    errno = 0;
    uint8_t dummy_bytes[1] = {0};
    header = ros_header_deserialize(dummy_bytes, 0);
    if (!header) {
        printf("Expected error on zero length: %s\n", strerror(errno));
        assert(errno == EINVAL);
    }
    
    // Test deserialize with bad data
    errno = 0;
    uint8_t bad_data[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    header = ros_header_deserialize(bad_data, sizeof(bad_data));
    if (!header) {
        printf("Expected error on bad message data: %s\n", strerror(errno));
        assert(errno == EBADMSG);
    }
    
    printf("Error handling tests completed successfully!\n");
}

int main() {
    printf("EdgeFirst Schemas C API Examples\n");
    printf("=================================\n");
    
    example_header();
    example_vector3();
    example_dmabuf();
    example_image();
    example_error_handling();
    
    printf("\n=================================\n");
    printf("All examples completed successfully!\n");
    
    return 0;
}
