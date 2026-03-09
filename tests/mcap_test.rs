// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! MCAP tests for real-world CDR validation.
//!
//! These tests validate that EdgeFirst Schemas can correctly deserialize
//! CDR-encoded messages from real MCAP recordings captured on hardware devices.
//!
//! Test data should be placed in the `testdata/` directory. All `.mcap` files
//! found there will be automatically tested.
//!
//! Tests FAIL (not skip) for:
//! - Unsupported schema types
//! - Deserialization errors
//! - Round-trip serialization mismatches

use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

use edgefirst_schemas::edgefirst_msgs;
use edgefirst_schemas::foxglove_msgs;
use edgefirst_schemas::geometry_msgs;
use edgefirst_schemas::sensor_msgs;

/// Path to test data directory relative to crate root
const TESTDATA_DIR: &str = "testdata/mcap";

/// Find all MCAP files in the testdata directory
fn find_mcap_files() -> Vec<PathBuf> {
    let testdata_path = Path::new(env!("CARGO_MANIFEST_DIR")).join(TESTDATA_DIR);

    if !testdata_path.exists() {
        return vec![];
    }

    fs::read_dir(&testdata_path)
        .expect("Failed to read testdata directory")
        .filter_map(|entry| {
            let entry = entry.ok()?;
            let path = entry.path();
            if path.extension().is_some_and(|ext| ext == "mcap") {
                Some(path)
            } else {
                None
            }
        })
        .collect()
}

/// Validate a message can be deserialized and round-tripped using zero-copy buffer-backed types.
/// Returns the re-serialized CDR bytes on success.
fn validate_message(schema_name: &str, data: &[u8]) -> Result<Vec<u8>, String> {
    match schema_name {
        // Buffer-backed types — from_cdr validates structure, to_cdr re-serializes
        "sensor_msgs/msg/Image" => {
            let view = sensor_msgs::Image::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "sensor_msgs/msg/CompressedImage" => {
            let view = sensor_msgs::CompressedImage::from_cdr(data.to_vec())
                .map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "sensor_msgs/msg/Imu" => {
            let view = sensor_msgs::Imu::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "sensor_msgs/msg/NavSatFix" => {
            let view =
                sensor_msgs::NavSatFix::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "sensor_msgs/msg/PointCloud2" => {
            let view =
                sensor_msgs::PointCloud2::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "sensor_msgs/msg/CameraInfo" => {
            let view =
                sensor_msgs::CameraInfo::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }

        // Stamped geometry types
        "geometry_msgs/msg/TransformStamped" => {
            let view = geometry_msgs::TransformStamped::from_cdr(data.to_vec())
                .map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "geometry_msgs/msg/TwistStamped" => {
            let view =
                geometry_msgs::TwistStamped::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "geometry_msgs/msg/AccelStamped" => {
            let view =
                geometry_msgs::AccelStamped::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "geometry_msgs/msg/InertiaStamped" => {
            let view = geometry_msgs::InertiaStamped::from_cdr(data.to_vec())
                .map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "geometry_msgs/msg/PointStamped" => {
            let view =
                geometry_msgs::PointStamped::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }

        // CdrFixed geometry types — round-trip via encode/decode
        "geometry_msgs/msg/Transform" => {
            use edgefirst_schemas::cdr::{decode_fixed, encode_fixed};
            use edgefirst_schemas::geometry_msgs::Transform;
            let val: Transform = decode_fixed(data).map_err(|e| format!("{e}"))?;
            encode_fixed(&val).map_err(|e| format!("{e}"))
        }
        "geometry_msgs/msg/Vector3" => {
            use edgefirst_schemas::cdr::{decode_fixed, encode_fixed};
            use edgefirst_schemas::geometry_msgs::Vector3;
            let val: Vector3 = decode_fixed(data).map_err(|e| format!("{e}"))?;
            encode_fixed(&val).map_err(|e| format!("{e}"))
        }
        "geometry_msgs/msg/Quaternion" => {
            use edgefirst_schemas::cdr::{decode_fixed, encode_fixed};
            use edgefirst_schemas::geometry_msgs::Quaternion;
            let val: Quaternion = decode_fixed(data).map_err(|e| format!("{e}"))?;
            encode_fixed(&val).map_err(|e| format!("{e}"))
        }
        "geometry_msgs/msg/Pose" => {
            use edgefirst_schemas::cdr::{decode_fixed, encode_fixed};
            use edgefirst_schemas::geometry_msgs::Pose;
            let val: Pose = decode_fixed(data).map_err(|e| format!("{e}"))?;
            encode_fixed(&val).map_err(|e| format!("{e}"))
        }
        "geometry_msgs/msg/Point" => {
            use edgefirst_schemas::cdr::{decode_fixed, encode_fixed};
            use edgefirst_schemas::geometry_msgs::Point;
            let val: Point = decode_fixed(data).map_err(|e| format!("{e}"))?;
            encode_fixed(&val).map_err(|e| format!("{e}"))
        }
        "geometry_msgs/msg/Twist" => {
            use edgefirst_schemas::cdr::{decode_fixed, encode_fixed};
            use edgefirst_schemas::geometry_msgs::Twist;
            let val: Twist = decode_fixed(data).map_err(|e| format!("{e}"))?;
            encode_fixed(&val).map_err(|e| format!("{e}"))
        }

        // Foxglove types
        "foxglove_msgs/msg/CompressedVideo" => {
            let view = foxglove_msgs::FoxgloveCompressedVideo::from_cdr(data.to_vec())
                .map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }

        // EdgeFirst types
        "edgefirst_msgs/msg/Detect" => {
            let view =
                edgefirst_msgs::Detect::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "edgefirst_msgs/msg/DmaBuffer" => {
            let view =
                edgefirst_msgs::DmaBuffer::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "edgefirst_msgs/msg/Mask" => {
            let view = edgefirst_msgs::Mask::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "edgefirst_msgs/msg/ModelInfo" => {
            let view =
                edgefirst_msgs::ModelInfo::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "edgefirst_msgs/msg/RadarCube" => {
            let view =
                edgefirst_msgs::RadarCube::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "edgefirst_msgs/msg/RadarInfo" => {
            let view =
                edgefirst_msgs::RadarInfo::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "edgefirst_msgs/msg/Box" => {
            let view =
                edgefirst_msgs::DetectBox::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "edgefirst_msgs/msg/Track" => {
            let view =
                edgefirst_msgs::Track::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "edgefirst_msgs/msg/Model" => {
            let view =
                edgefirst_msgs::Model::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }
        "edgefirst_msgs/msg/LocalTime" => {
            let view =
                edgefirst_msgs::LocalTime::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
            Ok(view.to_cdr())
        }

        _ => Err(format!("Unsupported schema: {schema_name}")),
    }
}

/// Check if a schema name is supported
fn is_schema_supported(schema_name: &str) -> bool {
    matches!(
        schema_name,
        "sensor_msgs/msg/CameraInfo"
            | "sensor_msgs/msg/CompressedImage"
            | "sensor_msgs/msg/Image"
            | "sensor_msgs/msg/Imu"
            | "sensor_msgs/msg/NavSatFix"
            | "sensor_msgs/msg/PointCloud2"
            | "geometry_msgs/msg/Transform"
            | "geometry_msgs/msg/TransformStamped"
            | "geometry_msgs/msg/Vector3"
            | "geometry_msgs/msg/Quaternion"
            | "geometry_msgs/msg/Pose"
            | "geometry_msgs/msg/Point"
            | "geometry_msgs/msg/Twist"
            | "geometry_msgs/msg/TwistStamped"
            | "geometry_msgs/msg/AccelStamped"
            | "geometry_msgs/msg/InertiaStamped"
            | "geometry_msgs/msg/PointStamped"
            | "foxglove_msgs/msg/CompressedVideo"
            | "edgefirst_msgs/msg/Box"
            | "edgefirst_msgs/msg/Detect"
            | "edgefirst_msgs/msg/DmaBuffer"
            | "edgefirst_msgs/msg/LocalTime"
            | "edgefirst_msgs/msg/Mask"
            | "edgefirst_msgs/msg/Model"
            | "edgefirst_msgs/msg/ModelInfo"
            | "edgefirst_msgs/msg/RadarCube"
            | "edgefirst_msgs/msg/RadarInfo"
            | "edgefirst_msgs/msg/Track"
    )
}

/// Test that all schema types in MCAP files are supported
#[test]
fn test_all_schemas_supported() {
    let mcap_files = find_mcap_files();
    if mcap_files.is_empty() {
        eprintln!("No MCAP files found in testdata/ - skipping test");
        return;
    }

    for mcap_path in &mcap_files {
        let file = fs::File::open(mcap_path).expect("Failed to open MCAP file");
        // SAFETY: The file is kept alive for the duration of the mmap's lifetime
        // and the data won't be modified during the test.
        let mapped = unsafe { memmap2::Mmap::map(&file) }.expect("Failed to mmap MCAP file");

        let summary = mcap::Summary::read(&mapped)
            .expect("Failed to read MCAP summary")
            .expect("MCAP has no summary");

        let mut unsupported = Vec::new();
        for schema in summary.schemas.values() {
            if !is_schema_supported(&schema.name) {
                unsupported.push(schema.name.clone());
            }
        }

        assert!(
            unsupported.is_empty(),
            "Unsupported schemas in {}: {:?}\nAdd these to validate_message() in tests/mcap_test.rs",
            mcap_path.display(),
            unsupported
        );

        println!(
            "✓ {} - all {} schemas supported",
            mcap_path.file_name().unwrap().to_string_lossy(),
            summary.schemas.len()
        );
    }
}

/// Test deserialization of all messages in MCAP files using zero-copy buffer-backed types
#[test]
fn test_deserialize_all_messages() {
    let mcap_files = find_mcap_files();
    if mcap_files.is_empty() {
        eprintln!("No MCAP files found in testdata/ - skipping test");
        return;
    }

    for mcap_path in &mcap_files {
        let file = fs::File::open(mcap_path).expect("Failed to open MCAP file");
        // SAFETY: The file is kept alive for the duration of the mmap's lifetime
        // and the data won't be modified during the test.
        let mapped = unsafe { memmap2::Mmap::map(&file) }.expect("Failed to mmap MCAP file");

        let mut errors = Vec::new();
        let mut message_counts: HashMap<String, usize> = HashMap::new();

        let msg_stream =
            mcap::MessageStream::new(&mapped).expect("Failed to create message stream");

        for message in msg_stream {
            let message = message.expect("Failed to read message");

            let schema_name = message
                .channel
                .schema
                .as_ref()
                .map(|s| s.name.as_str())
                .unwrap_or("unknown");

            *message_counts.entry(schema_name.to_string()).or_insert(0) += 1;

            if let Err(e) = validate_message(schema_name, &message.data) {
                errors.push(format!(
                    "{} (topic: {}): {}",
                    schema_name, message.channel.topic, e
                ));
                if errors.len() >= 10 {
                    break;
                }
            }
        }

        let total: usize = message_counts.values().sum();
        println!(
            "\nDeserialized {} messages from {}:",
            total,
            mcap_path.file_name().unwrap().to_string_lossy()
        );
        for (schema, count) in message_counts.iter() {
            println!("  {}: {}", schema, count);
        }

        assert!(
            errors.is_empty(),
            "Deserialization errors in {}:\n{}",
            mcap_path.display(),
            errors.join("\n")
        );
    }
}

/// Test round-trip serialization of all messages (from_cdr → to_cdr should preserve bytes)
#[test]
fn test_roundtrip_all_messages() {
    let mcap_files = find_mcap_files();
    if mcap_files.is_empty() {
        eprintln!("No MCAP files found in testdata/ - skipping test");
        return;
    }

    for mcap_path in &mcap_files {
        let file = fs::File::open(mcap_path).expect("Failed to open MCAP file");
        // SAFETY: The file is kept alive for the duration of the mmap's lifetime
        // and the data won't be modified during the test.
        let mapped = unsafe { memmap2::Mmap::map(&file) }.expect("Failed to mmap MCAP file");

        let mut errors = Vec::new();
        let mut success_count = 0;

        let msg_stream =
            mcap::MessageStream::new(&mapped).expect("Failed to create message stream");

        for message in msg_stream {
            let message = message.expect("Failed to read message");

            let schema_name = message
                .channel
                .schema
                .as_ref()
                .map(|s| s.name.as_str())
                .unwrap_or("unknown");

            match validate_message(schema_name, &message.data) {
                Ok(reserialized) => {
                    if reserialized.as_slice() != message.data.as_ref() {
                        errors.push(format!(
                            "{} (topic: {}): round-trip mismatch - original {} bytes, reserialized {} bytes",
                            schema_name,
                            message.channel.topic,
                            message.data.len(),
                            reserialized.len()
                        ));
                    } else {
                        success_count += 1;
                    }
                }
                Err(e) => {
                    errors.push(format!(
                        "{} (topic: {}): {}",
                        schema_name, message.channel.topic, e
                    ));
                }
            }

            if errors.len() >= 10 {
                break;
            }
        }

        println!(
            "\nRound-trip validated {} messages from {}",
            success_count,
            mcap_path.file_name().unwrap().to_string_lossy()
        );

        assert!(
            errors.is_empty(),
            "Round-trip errors in {}:\n{}",
            mcap_path.display(),
            errors.join("\n")
        );
    }
}

/// Deserialize a message (without re-serializing). Returns Ok(()) on success.
fn deserialize_message(schema_name: &str, data: &[u8]) -> Result<(), String> {
    match schema_name {
        "sensor_msgs/msg/Image" => {
            sensor_msgs::Image::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "sensor_msgs/msg/CompressedImage" => {
            sensor_msgs::CompressedImage::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "sensor_msgs/msg/Imu" => {
            sensor_msgs::Imu::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "sensor_msgs/msg/NavSatFix" => {
            sensor_msgs::NavSatFix::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "sensor_msgs/msg/PointCloud2" => {
            sensor_msgs::PointCloud2::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "sensor_msgs/msg/CameraInfo" => {
            sensor_msgs::CameraInfo::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "geometry_msgs/msg/TransformStamped" => {
            geometry_msgs::TransformStamped::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "geometry_msgs/msg/TwistStamped" => {
            geometry_msgs::TwistStamped::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "geometry_msgs/msg/AccelStamped" => {
            geometry_msgs::AccelStamped::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "geometry_msgs/msg/InertiaStamped" => {
            geometry_msgs::InertiaStamped::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "geometry_msgs/msg/PointStamped" => {
            geometry_msgs::PointStamped::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "geometry_msgs/msg/Transform" => {
            edgefirst_schemas::cdr::decode_fixed::<geometry_msgs::Transform>(data)
                .map_err(|e| format!("{e}"))?;
        }
        "geometry_msgs/msg/Vector3" => {
            edgefirst_schemas::cdr::decode_fixed::<geometry_msgs::Vector3>(data)
                .map_err(|e| format!("{e}"))?;
        }
        "geometry_msgs/msg/Quaternion" => {
            edgefirst_schemas::cdr::decode_fixed::<geometry_msgs::Quaternion>(data)
                .map_err(|e| format!("{e}"))?;
        }
        "geometry_msgs/msg/Pose" => {
            edgefirst_schemas::cdr::decode_fixed::<geometry_msgs::Pose>(data)
                .map_err(|e| format!("{e}"))?;
        }
        "geometry_msgs/msg/Point" => {
            edgefirst_schemas::cdr::decode_fixed::<geometry_msgs::Point>(data)
                .map_err(|e| format!("{e}"))?;
        }
        "geometry_msgs/msg/Twist" => {
            edgefirst_schemas::cdr::decode_fixed::<geometry_msgs::Twist>(data)
                .map_err(|e| format!("{e}"))?;
        }
        "foxglove_msgs/msg/CompressedVideo" => {
            foxglove_msgs::FoxgloveCompressedVideo::from_cdr(data.to_vec())
                .map_err(|e| format!("{e}"))?;
        }
        "edgefirst_msgs/msg/Detect" => {
            edgefirst_msgs::Detect::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "edgefirst_msgs/msg/DmaBuffer" => {
            edgefirst_msgs::DmaBuffer::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "edgefirst_msgs/msg/Mask" => {
            edgefirst_msgs::Mask::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "edgefirst_msgs/msg/ModelInfo" => {
            edgefirst_msgs::ModelInfo::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "edgefirst_msgs/msg/RadarCube" => {
            edgefirst_msgs::RadarCube::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "edgefirst_msgs/msg/RadarInfo" => {
            edgefirst_msgs::RadarInfo::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "edgefirst_msgs/msg/Box" => {
            edgefirst_msgs::DetectBox::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "edgefirst_msgs/msg/Track" => {
            edgefirst_msgs::Track::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "edgefirst_msgs/msg/Model" => {
            edgefirst_msgs::Model::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        "edgefirst_msgs/msg/LocalTime" => {
            edgefirst_msgs::LocalTime::from_cdr(data.to_vec()).map_err(|e| format!("{e}"))?;
        }
        _ => return Err(format!("Unsupported schema: {schema_name}")),
    }
    Ok(())
}

/// Benchmark-style timing of CDR deser + reser from MCAP (run with --nocapture)
#[test]
fn test_mcap_cdr_timing() {
    use std::time::Instant;

    let mcap_files = find_mcap_files();
    if mcap_files.is_empty() {
        eprintln!("No MCAP files found in testdata/ - skipping");
        return;
    }

    for mcap_path in &mcap_files {
        let file = fs::File::open(mcap_path).expect("Failed to open MCAP file");
        let mapped = unsafe { memmap2::Mmap::map(&file) }.expect("Failed to mmap MCAP file");

        // Collect messages grouped by schema
        let mut schema_messages: HashMap<String, Vec<Vec<u8>>> = HashMap::new();
        let msg_stream =
            mcap::MessageStream::new(&mapped).expect("Failed to create message stream");
        for message in msg_stream {
            let message = message.expect("Failed to read message");
            let schema_name = message
                .channel
                .schema
                .as_ref()
                .map(|s| s.name.clone())
                .unwrap_or_default();
            schema_messages
                .entry(schema_name)
                .or_default()
                .push(message.data.to_vec());
        }

        let total_msgs: usize = schema_messages.values().map(|v| v.len()).sum();
        let total_bytes: usize = schema_messages
            .values()
            .flat_map(|v| v.iter())
            .map(|m| m.len())
            .sum();

        println!("\n{}", "=".repeat(80));
        println!(
            "Rust CDR timing: {}",
            mcap_path.file_name().unwrap().to_string_lossy()
        );
        println!("{}", "=".repeat(80));
        println!(
            "Loaded {} messages ({:.1} MB)\n",
            total_msgs,
            total_bytes as f64 / 1024.0 / 1024.0
        );
        println!(
            "{:<40} {:>6} {:>10} {:>10} {:>10} {:>10}",
            "Schema", "Count", "Deser ms", "Reser ms", "Per msg", "MB/s"
        );
        println!(
            "{} {} {} {} {} {}",
            "-".repeat(40),
            "-".repeat(6),
            "-".repeat(10),
            "-".repeat(10),
            "-".repeat(10),
            "-".repeat(10)
        );

        let mut grand_deser_us: f64 = 0.0;
        let mut grand_reser_us: f64 = 0.0;
        let mut schema_names: Vec<_> = schema_messages.keys().cloned().collect();
        schema_names.sort();

        for schema_name in &schema_names {
            let messages = &schema_messages[schema_name];
            let schema_bytes: usize = messages.iter().map(|m| m.len()).sum();

            // Warm up
            for msg in messages.iter().take(3) {
                let _ = validate_message(schema_name, msg);
            }

            // Timed deserialization
            let start = Instant::now();
            for msg in messages {
                let _ = deserialize_message(schema_name, msg);
            }
            let deser_elapsed = start.elapsed();

            // Timed reserialization (deser + reser combined, subtract deser)
            let start = Instant::now();
            for msg in messages {
                let _ = validate_message(schema_name, msg);
            }
            let roundtrip_elapsed = start.elapsed();
            let reser_elapsed = roundtrip_elapsed.saturating_sub(deser_elapsed);

            let deser_ms = deser_elapsed.as_secs_f64() * 1000.0;
            let reser_ms = reser_elapsed.as_secs_f64() * 1000.0;
            let total_ns = (deser_elapsed + reser_elapsed).as_nanos() as f64;
            let per_msg_us = total_ns / messages.len() as f64 / 1000.0;
            let throughput_mbs = if total_ns > 0.0 {
                schema_bytes as f64 / (total_ns / 1_000_000_000.0) / (1024.0 * 1024.0)
            } else {
                0.0
            };
            grand_deser_us += deser_elapsed.as_micros() as f64;
            grand_reser_us += reser_elapsed.as_micros() as f64;

            println!(
                "{:<40} {:>6} {:>8.2}ms {:>8.2}ms {:>8.1}us {:>9.1}",
                schema_name,
                messages.len(),
                deser_ms,
                reser_ms,
                per_msg_us,
                throughput_mbs
            );
        }

        println!(
            "\n{:<40} {:>6} {:>8.2}ms {:>8.2}ms",
            "TOTAL",
            total_msgs,
            grand_deser_us / 1000.0,
            grand_reser_us / 1000.0
        );
    }
}
