// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! Basic Types Example
//!
//! Demonstrates EdgeFirst Schemas CDR serialization patterns:
//! - CdrFixed types: compile-time sized, value semantics (Time, Vector3, Pose, ColorRGBA)
//! - Buffer-backed types: zero-copy views over CDR byte buffers (Header, Image)
//!
//! The library uses CDR1 Little-Endian wire format, compatible with ROS 2 DDS.

use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::cdr::{decode_fixed, encode_fixed};
use edgefirst_schemas::geometry_msgs::{Point, Pose, Quaternion, Transform, Vector3};
use edgefirst_schemas::sensor_msgs::Image;
use edgefirst_schemas::std_msgs::{ColorRGBA, Header};

/// CdrFixed types: fixed-size structs serialized with encode_fixed/decode_fixed.
fn example_fixed_types() {
    println!("=== CdrFixed Types ===\n");

    // Time — 8 bytes on the wire (i32 sec + u32 nanosec)
    let time = Time::new(1234567890, 123456789);
    let bytes = encode_fixed(&time).unwrap();
    let decoded: Time = decode_fixed(&bytes).unwrap();
    assert_eq!(time, decoded);
    println!("Time:       {}.{:09}s  ({} CDR bytes)", time.sec, time.nanosec, bytes.len());

    // Vector3 — 24 bytes (3 × f64)
    let vel = Vector3 { x: 1.5, y: 2.0, z: 0.5 };
    let bytes = encode_fixed(&vel).unwrap();
    let decoded: Vector3 = decode_fixed(&bytes).unwrap();
    assert_eq!(vel, decoded);
    let mag = (vel.x * vel.x + vel.y * vel.y + vel.z * vel.z).sqrt();
    println!("Vector3:    ({}, {}, {})  mag={:.3}  ({} bytes)", vel.x, vel.y, vel.z, mag, bytes.len());

    // Pose — 56 bytes (Point 24 + Quaternion 32)
    let pose = Pose {
        position: Point { x: 1.0, y: 2.0, z: 0.5 },
        orientation: Quaternion { x: 0.0, y: 0.0, z: 0.707, w: 0.707 },
    };
    let bytes = encode_fixed(&pose).unwrap();
    let decoded: Pose = decode_fixed(&bytes).unwrap();
    assert_eq!(pose, decoded);
    println!("Pose:       pos=({},{},{}) quat=({},{},{},{})  ({} bytes)",
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        bytes.len());

    // Transform — 56 bytes
    let tf = Transform {
        translation: Vector3 { x: 1.0, y: 2.0, z: 0.5 },
        rotation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
    };
    let bytes = encode_fixed(&tf).unwrap();
    let decoded: Transform = decode_fixed(&bytes).unwrap();
    assert_eq!(tf, decoded);
    println!("Transform:  ({} bytes)", bytes.len());

    // ColorRGBA — 16 bytes (4 × f32)
    let color = ColorRGBA { r: 1.0, g: 0.0, b: 0.0, a: 1.0 };
    let bytes = encode_fixed(&color).unwrap();
    let decoded: ColorRGBA = decode_fixed(&bytes).unwrap();
    assert_eq!(color, decoded);
    println!("ColorRGBA:  r={} g={} b={} a={}  ({} bytes)\n", color.r, color.g, color.b, color.a, bytes.len());
}

/// Buffer-backed types: generic over B: AsRef<[u8]>, wrapping a CDR byte buffer.
/// Construction scans the buffer once to build a small offset table for O(1) field access.
fn example_buffer_backed_types() {
    println!("=== Buffer-Backed Types ===\n");

    // Header<Vec<u8>> — owned buffer, constructed with new()
    let stamp = Time::new(1234567890, 123456789);
    let header = Header::new(stamp, "camera_optical_frame").unwrap();
    println!("Header:     stamp={}.{:09}  frame_id=\"{}\"  ({} CDR bytes)",
        header.stamp().sec, header.stamp().nanosec, header.frame_id(), header.cdr_size());

    // Round-trip: to_cdr() serializes, from_cdr() deserializes
    let cdr_bytes = header.to_cdr();
    let decoded = Header::from_cdr(cdr_bytes).unwrap();
    assert_eq!(decoded.stamp(), stamp);
    assert_eq!(decoded.frame_id(), "camera_optical_frame");
    println!("            round-trip OK");

    // Zero-copy borrow: from_cdr(&[u8]) borrows the buffer without copying
    let owned_bytes = header.to_cdr();
    let borrowed = Header::from_cdr(owned_bytes.as_slice()).unwrap();
    assert_eq!(borrowed.frame_id(), "camera_optical_frame");
    println!("            zero-copy borrow OK");

    // Image<Vec<u8>> — larger buffer-backed type
    let pixel_data = vec![128u8; 640 * 480 * 3]; // VGA RGB8
    let img = Image::new(stamp, "camera", 480, 640, "rgb8", 0, 640 * 3, &pixel_data).unwrap();
    println!("\nImage:      {}x{} encoding=\"{}\"  data={} bytes  CDR={} bytes",
        img.width(), img.height(), img.encoding(), img.data().len(), img.cdr_size());

    // Round-trip
    let cdr_bytes = img.to_cdr();
    let decoded = Image::from_cdr(cdr_bytes).unwrap();
    assert_eq!(decoded.width(), 640);
    assert_eq!(decoded.height(), 480);
    assert_eq!(decoded.encoding(), "rgb8");
    assert_eq!(decoded.data().len(), 640 * 480 * 3);
    println!("            round-trip OK\n");
}

/// Mutation: buffer-backed types with mutable/owned buffers support in-place field updates.
fn example_mutation() {
    println!("=== Mutation ===\n");

    let mut header = Header::new(Time::new(0, 0), "test").unwrap();
    println!("Before:     stamp={}.{:09}", header.stamp().sec, header.stamp().nanosec);

    header.set_stamp(Time::new(42, 123)).unwrap();
    println!("After:      stamp={}.{:09}\n", header.stamp().sec, header.stamp().nanosec);
    assert_eq!(header.stamp(), Time::new(42, 123));
}

fn main() {
    println!("EdgeFirst Schemas - CDR Serialization Examples");
    println!("===============================================\n");

    example_fixed_types();
    example_buffer_backed_types();
    example_mutation();

    println!("===============================================");
    println!("All examples completed successfully!");
}
