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

/// Error type for MCAP test validation — distinguishes unsupported schemas
/// from CDR parse/roundtrip failures.
#[derive(Debug)]
enum TestError {
    Unsupported(String),
    Cdr(String),
}

impl std::fmt::Display for TestError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TestError::Unsupported(s) => write!(f, "Unsupported schema: {s}"),
            TestError::Cdr(s) => write!(f, "{s}"),
        }
    }
}

/// Deserialize and re-serialize a message for round-trip validation.
///
/// Buffer-backed types: `from_cdr(data) → to_cdr()`.
/// CdrFixed types: `decode_fixed(data) → encode_fixed()`.
///
/// Returns the re-serialized CDR bytes on success.
fn validate_message(schema_name: &str, data: &[u8]) -> Result<Vec<u8>, TestError> {
    use edgefirst_schemas::cdr::{decode_fixed, encode_fixed};

    /// Helper: from_cdr → to_cdr for a buffer-backed type.
    macro_rules! roundtrip_buf {
        ($ty:ty, $data:expr) => {{
            let view = <$ty>::from_cdr($data.to_vec()).map_err(|e| TestError::Cdr(format!("{e}")))?;
            Ok(view.to_cdr())
        }};
    }
    /// Helper: decode_fixed → encode_fixed for a CdrFixed type.
    macro_rules! roundtrip_fixed {
        ($ty:ty, $data:expr) => {{
            let val: $ty = decode_fixed($data).map_err(|e| TestError::Cdr(format!("{e}")))?;
            encode_fixed(&val).map_err(|e| TestError::Cdr(format!("{e}")))
        }};
    }

    match schema_name {
        // sensor_msgs — buffer-backed
        "sensor_msgs/msg/Image" => roundtrip_buf!(sensor_msgs::Image<Vec<u8>>, data),
        "sensor_msgs/msg/CompressedImage" => {
            roundtrip_buf!(sensor_msgs::CompressedImage<Vec<u8>>, data)
        }
        "sensor_msgs/msg/Imu" => roundtrip_buf!(sensor_msgs::Imu<Vec<u8>>, data),
        "sensor_msgs/msg/NavSatFix" => roundtrip_buf!(sensor_msgs::NavSatFix<Vec<u8>>, data),
        "sensor_msgs/msg/PointCloud2" => roundtrip_buf!(sensor_msgs::PointCloud2<Vec<u8>>, data),
        "sensor_msgs/msg/CameraInfo" => roundtrip_buf!(sensor_msgs::CameraInfo<Vec<u8>>, data),

        // geometry_msgs — buffer-backed (stamped)
        "geometry_msgs/msg/TransformStamped" => {
            roundtrip_buf!(geometry_msgs::TransformStamped<Vec<u8>>, data)
        }
        "geometry_msgs/msg/TwistStamped" => {
            roundtrip_buf!(geometry_msgs::TwistStamped<Vec<u8>>, data)
        }
        "geometry_msgs/msg/AccelStamped" => {
            roundtrip_buf!(geometry_msgs::AccelStamped<Vec<u8>>, data)
        }
        "geometry_msgs/msg/InertiaStamped" => {
            roundtrip_buf!(geometry_msgs::InertiaStamped<Vec<u8>>, data)
        }
        "geometry_msgs/msg/PointStamped" => {
            roundtrip_buf!(geometry_msgs::PointStamped<Vec<u8>>, data)
        }

        // geometry_msgs — CdrFixed
        "geometry_msgs/msg/Transform" => roundtrip_fixed!(geometry_msgs::Transform, data),
        "geometry_msgs/msg/Vector3" => roundtrip_fixed!(geometry_msgs::Vector3, data),
        "geometry_msgs/msg/Quaternion" => roundtrip_fixed!(geometry_msgs::Quaternion, data),
        "geometry_msgs/msg/Pose" => roundtrip_fixed!(geometry_msgs::Pose, data),
        "geometry_msgs/msg/Point" => roundtrip_fixed!(geometry_msgs::Point, data),
        "geometry_msgs/msg/Twist" => roundtrip_fixed!(geometry_msgs::Twist, data),

        // foxglove_msgs — buffer-backed
        "foxglove_msgs/msg/CompressedVideo" => {
            roundtrip_buf!(foxglove_msgs::FoxgloveCompressedVideo<Vec<u8>>, data)
        }

        // edgefirst_msgs — buffer-backed
        "edgefirst_msgs/msg/Detect" => roundtrip_buf!(edgefirst_msgs::Detect<Vec<u8>>, data),
        "edgefirst_msgs/msg/DmaBuffer" => roundtrip_buf!(edgefirst_msgs::DmaBuffer<Vec<u8>>, data),
        "edgefirst_msgs/msg/Mask" => roundtrip_buf!(edgefirst_msgs::Mask<Vec<u8>>, data),
        "edgefirst_msgs/msg/ModelInfo" => roundtrip_buf!(edgefirst_msgs::ModelInfo<Vec<u8>>, data),
        "edgefirst_msgs/msg/RadarCube" => roundtrip_buf!(edgefirst_msgs::RadarCube<Vec<u8>>, data),
        "edgefirst_msgs/msg/RadarInfo" => roundtrip_buf!(edgefirst_msgs::RadarInfo<Vec<u8>>, data),
        "edgefirst_msgs/msg/Box" => roundtrip_buf!(edgefirst_msgs::DetectBox<Vec<u8>>, data),
        "edgefirst_msgs/msg/Track" => roundtrip_buf!(edgefirst_msgs::Track<Vec<u8>>, data),
        "edgefirst_msgs/msg/Model" => roundtrip_buf!(edgefirst_msgs::Model<Vec<u8>>, data),
        "edgefirst_msgs/msg/LocalTime" => roundtrip_buf!(edgefirst_msgs::LocalTime<Vec<u8>>, data),

        _ => Err(TestError::Unsupported(schema_name.to_string())),
    }
}

/// Check if a schema name is supported
fn is_schema_supported(schema_name: &str) -> bool {
    // A recognized schema produces Ok or Cdr error on empty data.
    // Only unrecognized schemas produce Unsupported.
    matches!(
        validate_message(schema_name, &[]),
        Ok(_) | Err(TestError::Cdr(_))
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

/// Deserialize a message without re-serializing. Returns Ok(()) on success.
fn deserialize_message(schema_name: &str, data: &[u8]) -> Result<(), TestError> {
    use edgefirst_schemas::cdr::decode_fixed;

    macro_rules! deser_buf {
        ($ty:ty) => {{
            <$ty>::from_cdr(data.to_vec()).map_err(|e| TestError::Cdr(format!("{e}")))?;
            Ok(())
        }};
    }
    macro_rules! deser_fixed {
        ($ty:ty) => {{
            decode_fixed::<$ty>(data).map_err(|e| TestError::Cdr(format!("{e}")))?;
            Ok(())
        }};
    }

    match schema_name {
        // sensor_msgs — buffer-backed
        "sensor_msgs/msg/Image" => deser_buf!(sensor_msgs::Image<Vec<u8>>),
        "sensor_msgs/msg/CompressedImage" => deser_buf!(sensor_msgs::CompressedImage<Vec<u8>>),
        "sensor_msgs/msg/Imu" => deser_buf!(sensor_msgs::Imu<Vec<u8>>),
        "sensor_msgs/msg/NavSatFix" => deser_buf!(sensor_msgs::NavSatFix<Vec<u8>>),
        "sensor_msgs/msg/PointCloud2" => deser_buf!(sensor_msgs::PointCloud2<Vec<u8>>),
        "sensor_msgs/msg/CameraInfo" => deser_buf!(sensor_msgs::CameraInfo<Vec<u8>>),

        // geometry_msgs — buffer-backed (stamped)
        "geometry_msgs/msg/TransformStamped" => {
            deser_buf!(geometry_msgs::TransformStamped<Vec<u8>>)
        }
        "geometry_msgs/msg/TwistStamped" => deser_buf!(geometry_msgs::TwistStamped<Vec<u8>>),
        "geometry_msgs/msg/AccelStamped" => deser_buf!(geometry_msgs::AccelStamped<Vec<u8>>),
        "geometry_msgs/msg/InertiaStamped" => deser_buf!(geometry_msgs::InertiaStamped<Vec<u8>>),
        "geometry_msgs/msg/PointStamped" => deser_buf!(geometry_msgs::PointStamped<Vec<u8>>),

        // geometry_msgs — CdrFixed
        "geometry_msgs/msg/Transform" => deser_fixed!(geometry_msgs::Transform),
        "geometry_msgs/msg/Vector3" => deser_fixed!(geometry_msgs::Vector3),
        "geometry_msgs/msg/Quaternion" => deser_fixed!(geometry_msgs::Quaternion),
        "geometry_msgs/msg/Pose" => deser_fixed!(geometry_msgs::Pose),
        "geometry_msgs/msg/Point" => deser_fixed!(geometry_msgs::Point),
        "geometry_msgs/msg/Twist" => deser_fixed!(geometry_msgs::Twist),

        // foxglove_msgs — buffer-backed
        "foxglove_msgs/msg/CompressedVideo" => {
            deser_buf!(foxglove_msgs::FoxgloveCompressedVideo<Vec<u8>>)
        }

        // edgefirst_msgs — buffer-backed
        "edgefirst_msgs/msg/Detect" => deser_buf!(edgefirst_msgs::Detect<Vec<u8>>),
        "edgefirst_msgs/msg/DmaBuffer" => deser_buf!(edgefirst_msgs::DmaBuffer<Vec<u8>>),
        "edgefirst_msgs/msg/Mask" => deser_buf!(edgefirst_msgs::Mask<Vec<u8>>),
        "edgefirst_msgs/msg/ModelInfo" => deser_buf!(edgefirst_msgs::ModelInfo<Vec<u8>>),
        "edgefirst_msgs/msg/RadarCube" => deser_buf!(edgefirst_msgs::RadarCube<Vec<u8>>),
        "edgefirst_msgs/msg/RadarInfo" => deser_buf!(edgefirst_msgs::RadarInfo<Vec<u8>>),
        "edgefirst_msgs/msg/Box" => deser_buf!(edgefirst_msgs::DetectBox<Vec<u8>>),
        "edgefirst_msgs/msg/Track" => deser_buf!(edgefirst_msgs::Track<Vec<u8>>),
        "edgefirst_msgs/msg/Model" => deser_buf!(edgefirst_msgs::Model<Vec<u8>>),
        "edgefirst_msgs/msg/LocalTime" => deser_buf!(edgefirst_msgs::LocalTime<Vec<u8>>),

        _ => Err(TestError::Unsupported(schema_name.to_string())),
    }
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

        println!("\n{}", "=".repeat(90));
        println!(
            "Rust CDR timing: {}",
            mcap_path.file_name().unwrap().to_string_lossy()
        );
        println!("{}", "=".repeat(90));
        println!(
            "Loaded {} messages ({:.1} MB)\n",
            total_msgs,
            total_bytes as f64 / 1024.0 / 1024.0
        );
        println!(
            "{:<40} {:>6} {:>10} {:>10} {:>10} {:>10} {:>10}",
            "Schema", "Count", "Deser ms", "Ser ms", "Roundtrip", "Per msg", "MB/s"
        );
        println!(
            "{} {} {} {} {} {} {}",
            "-".repeat(40),
            "-".repeat(6),
            "-".repeat(10),
            "-".repeat(10),
            "-".repeat(10),
            "-".repeat(10),
            "-".repeat(10)
        );

        let mut grand_deser_us: f64 = 0.0;
        let mut grand_ser_us: f64 = 0.0;
        let mut grand_mismatches: usize = 0;
        let mut schema_names: Vec<_> = schema_messages.keys().cloned().collect();
        schema_names.sort();

        for schema_name in &schema_names {
            let messages = &schema_messages[schema_name];
            let schema_bytes: usize = messages.iter().map(|m| m.len()).sum();

            // Warm up
            for msg in messages.iter().take(3) {
                let _ = validate_message(schema_name, msg);
            }

            // Phase 1: Deserialization only
            let start = Instant::now();
            for msg in messages {
                let _ = deserialize_message(schema_name, msg);
            }
            let deser_elapsed = start.elapsed();

            // Phase 2: Full roundtrip (deser + ser)
            let start = Instant::now();
            let mut outputs = Vec::with_capacity(messages.len());
            for msg in messages {
                outputs.push(validate_message(schema_name, msg));
            }
            let roundtrip_elapsed = start.elapsed();
            let ser_elapsed = roundtrip_elapsed.saturating_sub(deser_elapsed);

            // Phase 3: Validation
            let mut mismatches = 0;
            for (msg, output) in messages.iter().zip(outputs.iter()) {
                if let Ok(reserialized) = output {
                    if reserialized.as_slice() != msg.as_slice() {
                        mismatches += 1;
                    }
                }
            }
            grand_mismatches += mismatches;

            let deser_ms = deser_elapsed.as_secs_f64() * 1000.0;
            let ser_ms = ser_elapsed.as_secs_f64() * 1000.0;
            let roundtrip_ms = roundtrip_elapsed.as_secs_f64() * 1000.0;
            let roundtrip_ns = roundtrip_elapsed.as_nanos() as f64;
            let per_msg_us = roundtrip_ns / messages.len() as f64 / 1000.0;
            let throughput_mbs = if roundtrip_ns > 0.0 {
                schema_bytes as f64 / (roundtrip_ns / 1_000_000_000.0) / (1024.0 * 1024.0)
            } else {
                0.0
            };
            grand_deser_us += deser_elapsed.as_micros() as f64;
            grand_ser_us += ser_elapsed.as_micros() as f64;

            println!(
                "{:<40} {:>6} {:>8.2}ms {:>8.2}ms {:>8.2}ms {:>8.1}us {:>9.1}",
                schema_name,
                messages.len(),
                deser_ms,
                ser_ms,
                roundtrip_ms,
                per_msg_us,
                throughput_mbs
            );
        }

        let grand_roundtrip_ms = grand_deser_us / 1000.0 + grand_ser_us / 1000.0;
        println!(
            "\n{:<40} {:>6} {:>8.2}ms {:>8.2}ms {:>8.2}ms",
            "TOTAL",
            total_msgs,
            grand_deser_us / 1000.0,
            grand_ser_us / 1000.0,
            grand_roundtrip_ms
        );
        println!("Validation: {} mismatches", grand_mismatches);
    }
}
