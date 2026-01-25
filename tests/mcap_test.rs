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

use edgefirst_schemas::*;

/// Path to test data directory relative to crate root
const TESTDATA_DIR: &str = "testdata";

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

/// Deserialize a message based on its schema name
fn deserialize_message(schema_name: &str, data: &[u8]) -> Result<Vec<u8>, String> {
    // Deserialize and immediately re-serialize to get round-trip bytes
    match schema_name {
        // sensor_msgs
        "sensor_msgs/msg/CameraInfo" => {
            let msg: sensor_msgs::CameraInfo =
                cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "sensor_msgs/msg/CompressedImage" => {
            let msg: sensor_msgs::CompressedImage =
                cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "sensor_msgs/msg/Image" => {
            let msg: sensor_msgs::Image = cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "sensor_msgs/msg/Imu" => {
            let msg: sensor_msgs::IMU = cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "sensor_msgs/msg/NavSatFix" => {
            let msg: sensor_msgs::NavSatFix = cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "sensor_msgs/msg/PointCloud2" => {
            let msg: sensor_msgs::PointCloud2 =
                cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }

        // geometry_msgs
        "geometry_msgs/msg/Transform" => {
            let msg: geometry_msgs::Transform =
                cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "geometry_msgs/msg/TransformStamped" => {
            let msg: geometry_msgs::TransformStamped =
                cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "geometry_msgs/msg/Vector3" => {
            let msg: geometry_msgs::Vector3 = cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "geometry_msgs/msg/Quaternion" => {
            let msg: geometry_msgs::Quaternion =
                cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "geometry_msgs/msg/Pose" => {
            let msg: geometry_msgs::Pose = cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "geometry_msgs/msg/Point" => {
            let msg: geometry_msgs::Point = cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "geometry_msgs/msg/Twist" => {
            let msg: geometry_msgs::Twist = cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "geometry_msgs/msg/TwistStamped" => {
            let msg: geometry_msgs::TwistStamped =
                cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }

        // foxglove_msgs
        // Note: Only FoxgloveCompressedVideo is currently implemented in Rust
        "foxglove_msgs/msg/CompressedVideo" => {
            let msg: foxglove_msgs::FoxgloveCompressedVideo =
                cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }

        // edgefirst_msgs
        "edgefirst_msgs/msg/Detect" => {
            let msg: edgefirst_msgs::Detect = cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "edgefirst_msgs/msg/DmaBuffer" => {
            let msg: edgefirst_msgs::DmaBuffer =
                cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "edgefirst_msgs/msg/Mask" => {
            let msg: edgefirst_msgs::Mask = cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "edgefirst_msgs/msg/ModelInfo" => {
            let msg: edgefirst_msgs::ModelInfo =
                cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "edgefirst_msgs/msg/RadarCube" => {
            let msg: edgefirst_msgs::RadarCube =
                cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "edgefirst_msgs/msg/RadarInfo" => {
            let msg: edgefirst_msgs::RadarInfo =
                cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "edgefirst_msgs/msg/Box" => {
            let msg: edgefirst_msgs::Box = cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
        }
        "edgefirst_msgs/msg/Track" => {
            let msg: edgefirst_msgs::Track = cdr::deserialize(data).map_err(|e| format!("{e}"))?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite).map_err(|e| format!("{e}"))
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
            | "foxglove_msgs/msg/CompressedVideo"
            | "edgefirst_msgs/msg/Box"
            | "edgefirst_msgs/msg/Detect"
            | "edgefirst_msgs/msg/DmaBuffer"
            | "edgefirst_msgs/msg/Mask"
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
            "Unsupported schemas in {}: {:?}\nAdd these to deserialize_message() in tests/mcap_test.rs",
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

/// Test deserialization of all messages in MCAP files
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

            // Get schema name from channel's schema
            let schema_name = message
                .channel
                .schema
                .as_ref()
                .map(|s| s.name.as_str())
                .unwrap_or("unknown");

            *message_counts.entry(schema_name.to_string()).or_insert(0) += 1;

            if let Err(e) = deserialize_message(schema_name, &message.data) {
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

/// Test round-trip serialization of all messages
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

            match deserialize_message(schema_name, &message.data) {
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
