/**
 * @file example.cpp
 * @brief Example usage of EdgeFirst Schemas C++ API
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.
 *
 * Demonstrates:
 * - CdrFixed encode/decode (Time, Vector3) into stack buffers
 * - Buffer-backed encode (Header, Image, DmaBuffer) returning owning types
 * - Buffer-backed decode via View types (move-only, RAII)
 * - expected<T, Error> for fallible operations (no exceptions)
 * - Zero-copy field access via std::string_view and span<const uint8_t>
 */

#include <edgefirst/schemas.hpp>

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string_view>
#include <vector>

namespace ef = edgefirst::schemas;

static int example_time() {
    std::cout << "\n=== Example: Time (CdrFixed) ===\n";

    ef::Time t{1234567890, 123456789};

    std::array<std::uint8_t, 64> buf{};
    auto written = t.encode(ef::span<std::uint8_t>{buf.data(), buf.size()});
    if (!written) {
        std::cerr << "ros_time_encode failed: " << written.error().category() << "\n";
        return -1;
    }
    std::cout << "Encoded Time: " << *written << " CDR bytes\n";

    auto decoded = ef::Time::decode(ef::span<const std::uint8_t>{buf.data(), *written});
    if (!decoded) {
        std::cerr << "ros_time_decode failed: " << decoded.error().category() << "\n";
        return -1;
    }
    std::cout << "Decoded: " << decoded->sec << "." << decoded->nanosec << "\n";

    if (decoded->sec != 1234567890 || decoded->nanosec != 123456789) {
        std::cerr << "Time roundtrip mismatch\n";
        return -1;
    }

    std::cout << "Time example completed successfully!\n";
    return 0;
}

static int example_vector3() {
    std::cout << "\n=== Example: Vector3 (CdrFixed) ===\n";

    ef::Vector3 v{1.5, 2.5, 3.5};

    std::array<std::uint8_t, 64> buf{};
    auto written = v.encode(ef::span<std::uint8_t>{buf.data(), buf.size()});
    if (!written) {
        std::cerr << "ros_vector3_encode failed: " << written.error().category() << "\n";
        return -1;
    }
    std::cout << "Encoded Vector3: " << *written << " CDR bytes\n";

    auto decoded = ef::Vector3::decode(ef::span<const std::uint8_t>{buf.data(), *written});
    if (!decoded) {
        std::cerr << "ros_vector3_decode failed: " << decoded.error().category() << "\n";
        return -1;
    }
    std::cout << "Decoded: (" << decoded->x << ", " << decoded->y << ", " << decoded->z << ")\n";

    if (decoded->x != 1.5 || decoded->y != 2.5 || decoded->z != 3.5) {
        std::cerr << "Vector3 roundtrip mismatch\n";
        return -1;
    }

    std::cout << "Vector3 example completed successfully!\n";
    return 0;
}

static int example_header() {
    std::cout << "\n=== Example: Header (buffer-backed) ===\n";

    // Encode a Header — returns owning Header
    auto encoded = ef::Header::encode(
        ef::Time{1234567890, 123456789},
        "camera_frame"
    );
    if (!encoded) {
        std::cerr << "Header::encode failed: " << encoded.error().category() << "\n";
        return -1;
    }
    std::cout << "Encoded Header: " << encoded->as_cdr().size() << " CDR bytes\n";

    // Access fields from owning Header
    auto stamp = encoded->stamp();
    auto frame_id = encoded->frame_id();

    std::cout << "Decoded: stamp=" << stamp.sec << "." << stamp.nanosec
              << " frame_id=\"" << frame_id << "\"\n";

    bool ok = (stamp.sec == 1234567890 && stamp.nanosec == 123456789
               && frame_id == "camera_frame");

    if (!ok) {
        std::cerr << "Header roundtrip mismatch\n";
        return -1;
    }

    std::cout << "Header example completed successfully!\n";
    return 0;
}

static int example_image() {
    std::cout << "\n=== Example: Image (buffer-backed) ===\n";

    // Create dummy pixel data
    std::size_t data_size = 640 * 480 * 3;  // VGA RGB8
    std::vector<std::uint8_t> pixel_data(data_size);
    for (std::size_t i = 0; i < data_size; i++) {
        pixel_data[i] = static_cast<std::uint8_t>(i % 256);
    }

    // Encode an Image — all fields in one call
    auto encoded = ef::Image::encode(
        ef::Time{1000, 500000},
        "camera",
        480, 640,                // height, width
        "rgb8",
        false,                   // is_bigendian
        640 * 3,                 // step
        ef::span<const std::uint8_t>{pixel_data.data(), pixel_data.size()}
    );
    if (!encoded) {
        std::cerr << "Image::encode failed: " << encoded.error().category() << "\n";
        return -1;
    }
    std::cout << "Encoded Image: " << encoded->as_cdr().size() << " CDR bytes\n";

    // Access fields from owning Image
    auto width = encoded->width();
    auto height = encoded->height();
    auto encoding = encoded->encoding();
    auto data = encoded->data();

    std::cout << "Decoded: " << width << "x" << height
              << " encoding=\"" << encoding << "\" data=" << data.size() << " bytes\n";

    bool ok = (width == 640 && height == 480
               && encoding == "rgb8"
               && data.size() == data_size);

    if (!ok) {
        std::cerr << "Image roundtrip mismatch\n";
        return -1;
    }

    // Forward the raw CDR bytes (zero re-serialization cost)
    auto cdr = encoded->as_cdr();
    std::cout << "CDR bytes available for forwarding: " << cdr.size() << " bytes\n";

    std::cout << "Image example completed successfully!\n";
    return 0;
}

static int example_dmabuffer() {
    std::cout << "\n=== Example: DmaBuffer (buffer-backed) ===\n";

    // Encode a DmaBuffer
    auto encoded = ef::DmaBuffer::encode(
        ef::Time{1000, 500000},
        "camera0",
        12345, 42,               // pid, fd
        1920, 1080,              // width, height
        1920 * 2,                // stride (YUYV)
        0x56595559,              // fourcc = 'YUYV'
        1920 * 1080 * 2          // length
    );
    if (!encoded) {
        std::cerr << "DmaBuffer::encode failed: " << encoded.error().category() << "\n";
        return -1;
    }
    std::cout << "Encoded DmaBuffer: " << encoded->as_cdr().size() << " CDR bytes\n";

    // Access fields from owning DmaBuffer
    auto width = encoded->width();
    auto height = encoded->height();
    auto pid = encoded->pid();
    auto fd = encoded->fd();

    std::cout << "Decoded: " << width << "x" << height
              << " pid=" << pid << " fd=" << fd << "\n";

    bool ok = (width == 1920 && height == 1080 && pid == 12345 && fd == 42);

    if (!ok) {
        std::cerr << "DmaBuffer roundtrip mismatch\n";
        return -1;
    }

    std::cout << "DmaBuffer example completed successfully!\n";
    return 0;
}

int main() {
    std::cout << "EdgeFirst Schemas C++ API Examples\n";
    std::cout << "===================================\n";

    int failures = 0;

    if (example_time() != 0)       failures++;
    if (example_vector3() != 0)    failures++;
    if (example_header() != 0)     failures++;
    if (example_image() != 0)      failures++;
    if (example_dmabuffer() != 0)  failures++;

    std::cout << "\n===================================\n";
    if (failures > 0) {
        std::cerr << failures << " example(s) FAILED\n";
        return 1;
    }

    std::cout << "All C++ examples completed successfully!\n";
    return 0;
}
