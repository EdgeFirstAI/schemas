// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! Basic Types Example
//!
//! Demonstrates fundamental ROS2 message types including:
//! - builtin_interfaces (Time, Duration)
//! - std_msgs (Header, ColorRGBA)
//! - geometry_msgs (Vector3, Point, Quaternion, Pose, Transform)

use edgefirst_schemas::builtin_interfaces::{Duration, Time};
use edgefirst_schemas::geometry_msgs::{Point, Pose, Quaternion, Transform, Vector3};
use edgefirst_schemas::std_msgs::{ColorRGBA, Header};

fn example_time() {
    println!("=== Example: Time ===");

    // Create a timestamp
    let time = Time {
        sec: 1234567890,
        nanosec: 123456789,
    };

    println!("Time: {}.{:09} seconds", time.sec, time.nanosec);
    println!("Time (debug): {:?}\n", time);
}

fn example_duration() {
    println!("=== Example: Duration ===");

    // Create a duration (5.5 seconds)
    let duration = Duration {
        sec: 5,
        nanosec: 500_000_000,
    };

    let total_ns = duration.sec as i64 * 1_000_000_000 + duration.nanosec as i64;
    println!(
        "Duration: {} seconds ({} nanoseconds)\n",
        total_ns as f64 / 1e9,
        total_ns
    );
}

fn example_header() {
    println!("=== Example: Header ===");

    // Create a header with timestamp and frame ID
    let header = Header {
        stamp: Time {
            sec: 1234567890,
            nanosec: 123456789,
        },
        frame_id: "camera_optical_frame".to_string(),
    };

    println!("Header:");
    println!(
        "  timestamp: {}.{:09}",
        header.stamp.sec, header.stamp.nanosec
    );
    println!("  frame_id: {}\n", header.frame_id);

    // Using default values
    let empty_header = Header {
        stamp: Time { sec: 0, nanosec: 0 },
        frame_id: String::new(),
    };
    println!("Empty header: {:?}\n", empty_header);
}

fn example_color() {
    println!("=== Example: ColorRGBA ===");

    // Create colors
    let red = ColorRGBA {
        r: 1.0,
        g: 0.0,
        b: 0.0,
        a: 1.0,
    };

    let transparent_blue = ColorRGBA {
        r: 0.0,
        g: 0.0,
        b: 1.0,
        a: 0.5,
    };

    println!("Red: {:?}", red);
    println!("Transparent blue: {:?}\n", transparent_blue);
}

fn example_vector3() {
    println!("=== Example: Vector3 ===");

    let velocity = Vector3 {
        x: 1.5,
        y: 2.0,
        z: 0.5,
    };

    // Calculate magnitude
    let magnitude = (velocity.x.powi(2) + velocity.y.powi(2) + velocity.z.powi(2)).sqrt();

    println!(
        "Velocity vector: ({}, {}, {})",
        velocity.x, velocity.y, velocity.z
    );
    println!("Magnitude: {:.3}\n", magnitude);
}

fn example_point() {
    println!("=== Example: Point ===");

    let origin = Point {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    let target = Point {
        x: 10.0,
        y: 5.0,
        z: 2.0,
    };

    // Calculate distance
    let dx = target.x - origin.x;
    let dy = target.y - origin.y;
    let dz = target.z - origin.z;
    let distance = (dx * dx + dy * dy + dz * dz).sqrt();

    println!("Origin: {:?}", origin);
    println!("Target: {:?}", target);
    println!("Distance: {:.3}\n", distance);
}

fn example_quaternion() {
    println!("=== Example: Quaternion ===");

    // Identity quaternion (no rotation)
    let identity = Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0,
    };

    // 90 degree rotation around Z axis
    let rotation_z_90 = Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.707, // sin(45°)
        w: 0.707, // cos(45°)
    };

    println!("Identity rotation: {:?}", identity);
    println!("90° Z rotation: {:?}\n", rotation_z_90);
}

fn example_pose() {
    println!("=== Example: Pose ===");

    let pose = Pose {
        position: Point {
            x: 1.0,
            y: 2.0,
            z: 0.5,
        },
        orientation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
    };

    println!("Pose:");
    println!(
        "  position: ({}, {}, {})",
        pose.position.x, pose.position.y, pose.position.z
    );
    println!(
        "  orientation: ({}, {}, {}, {})\n",
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    );
}

fn example_transform() {
    println!("=== Example: Transform ===");

    let transform = Transform {
        translation: Vector3 {
            x: 1.0,
            y: 2.0,
            z: 0.5,
        },
        rotation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
    };

    println!("Transform:");
    println!(
        "  translation: ({}, {}, {})",
        transform.translation.x, transform.translation.y, transform.translation.z
    );
    println!(
        "  rotation: ({}, {}, {}, {})\n",
        transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w
    );
}

fn example_serialization() {
    println!("=== Example: CDR Serialization ===");

    use edgefirst_schemas::serde_cdr::{deserialize, serialize};

    let header = Header {
        stamp: Time {
            sec: 1234567890,
            nanosec: 123456789,
        },
        frame_id: "test_frame".to_string(),
    };

    // Serialize to CDR bytes (ROS2-compatible little-endian format)
    let bytes = serialize(&header).expect("Serialization failed");
    println!("Serialized header to {} bytes", bytes.len());

    // Deserialize from CDR bytes
    let decoded: Header = deserialize(&bytes).expect("Deserialization failed");

    // Verify round-trip
    assert_eq!(header.stamp.sec, decoded.stamp.sec);
    assert_eq!(header.stamp.nanosec, decoded.stamp.nanosec);
    assert_eq!(header.frame_id, decoded.frame_id);

    println!("Round-trip successful!");
    println!("  Original:    {:?}", header);
    println!("  Deserialized: {:?}\n", decoded);
}

fn main() {
    println!("EdgeFirst Schemas - Basic Types Examples");
    println!("==========================================\n");

    example_time();
    example_duration();
    example_header();
    example_color();
    example_vector3();
    example_point();
    example_quaternion();
    example_pose();
    example_transform();
    example_serialization();

    println!("==========================================");
    println!("All examples completed successfully!");
}
