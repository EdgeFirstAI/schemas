// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! CDR (Common Data Representation) serialization and deserialization support.
//!
//! This module provides serialize/deserialize functions for all schema types
//! using ROS2 CDR encoding format.

use byteorder::LittleEndian;
use serde::{Deserialize, Serialize};

/// CDR encapsulation header for CDR1 Little Endian (RTPS CDR_LE).
/// The `cdr` crate includes this 4-byte prefix; `cdr-encoding` does not.
/// We add/strip it manually to maintain wire compatibility.
const CDR_LE_HEADER: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

/// Error type for serialization/deserialization operations
#[derive(Debug)]
pub enum Error {
    /// CDR serialization error
    Serialization(cdr_encoding::Error),
    /// CDR deserialization error
    Deserialization(cdr_encoding::Error),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Error::Serialization(e) => write!(f, "Serialization error: {}", e),
            Error::Deserialization(e) => write!(f, "Deserialization error: {}", e),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Serialization(e) => Some(e),
            Error::Deserialization(e) => Some(e),
        }
    }
}

/// Serialize a message to CDR format
///
/// # Arguments
/// * `msg` - The message to serialize (must implement `Serialize`)
///
/// # Returns
/// * `Ok(Vec<u8>)` - The serialized message as bytes
/// * `Err(Error)` - Serialization error
///
/// # Example
/// ```
/// use edgefirst_schemas::std_msgs::Header;
/// use edgefirst_schemas::builtin_interfaces::Time;
/// use edgefirst_schemas::serde_cdr::serialize;
///
/// let header = Header {
///     stamp: Time { sec: 0, nanosec: 0 },
///     frame_id: "camera".to_string(),
/// };
/// let bytes = serialize(&header).unwrap();
/// ```
pub fn serialize<T: Serialize>(msg: &T) -> Result<Vec<u8>, Error> {
    let payload = cdr_encoding::to_vec::<T, LittleEndian>(msg).map_err(Error::Serialization)?;
    let mut buf = Vec::with_capacity(CDR_LE_HEADER.len() + payload.len());
    buf.extend_from_slice(&CDR_LE_HEADER);
    buf.extend_from_slice(&payload);
    Ok(buf)
}

/// Deserialize a message from CDR format
///
/// # Arguments
/// * `bytes` - The serialized message bytes
///
/// # Returns
/// * `Ok(T)` - The deserialized message
/// * `Err(Error)` - Deserialization error
///
/// # Example
/// ```
/// use edgefirst_schemas::std_msgs::Header;
/// use edgefirst_schemas::builtin_interfaces::Time;
/// use edgefirst_schemas::serde_cdr::{serialize, deserialize};
///
/// let header = Header {
///     stamp: Time { sec: 0, nanosec: 0 },
///     frame_id: "camera".to_string(),
/// };
/// let bytes = serialize(&header).unwrap();
/// let deserialized: Header = deserialize(&bytes).unwrap();
/// assert_eq!(header, deserialized);
/// ```
pub fn deserialize<'a, T: Deserialize<'a>>(bytes: &'a [u8]) -> Result<T, Error> {
    let payload = if bytes.len() >= CDR_LE_HEADER.len() && bytes[..4] == CDR_LE_HEADER {
        &bytes[CDR_LE_HEADER.len()..]
    } else {
        bytes
    };
    cdr_encoding::from_bytes::<T, LittleEndian>(payload)
        .map(|(value, _size)| value)
        .map_err(Error::Deserialization)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::std_msgs::Header;

    #[test]
    fn serialize_deserialize_roundtrip() {
        // Test with data and empty strings
        let cases = [
            Header {
                stamp: Time::new(42, 123456789),
                frame_id: "test_frame".to_string(),
            },
            Header {
                stamp: Time::new(0, 0),
                frame_id: String::new(),
            },
        ];
        for header in &cases {
            let bytes = serialize(header).unwrap();
            assert_eq!(*header, deserialize::<Header>(&bytes).unwrap());
        }
    }

    #[test]
    fn error_display() {
        // Verify error formatting works
        let bad_bytes = &[0u8; 1];
        let result: Result<Header, _> = deserialize(bad_bytes);
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.to_string().contains("Deserialization error"));
    }

    #[test]
    fn error_source() {
        // Verify Error::source() returns the underlying CDR error
        use std::error::Error as StdError;

        let bad_bytes = &[0u8; 1];
        let result: Result<Header, _> = deserialize(bad_bytes);
        let err = result.unwrap_err();

        // source() should return Some pointing to the underlying cdr_encoding::Error
        let source = err.source();
        assert!(
            source.is_some(),
            "Error::source() should return the underlying error"
        );

        // The source should be a cdr_encoding::Error (we can't match on it directly, but
        // we can verify it exists and has a description)
        let source_str = source.unwrap().to_string();
        assert!(
            !source_str.is_empty(),
            "Source error should have a description"
        );
    }

    /// Wire compatibility test: verify cdr and cdr-encoding produce identical bytes.
    /// Uses `cdr` (dev-dependency) as the reference implementation.
    #[test]
    fn wire_compatibility_with_cdr() {
        use crate::edgefirst_msgs::Model;
        use crate::sensor_msgs::CompressedImage;

        // Time - simple fixed-size struct
        let time = Time {
            sec: 1234567890,
            nanosec: 123456789,
        };
        let cdr_bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&time, cdr::size::Infinite).unwrap();
        let enc_bytes = serialize(&time).unwrap();
        assert_eq!(cdr_bytes, enc_bytes, "Time: wire format mismatch");

        // Header - contains a String (variable-length)
        let header = Header {
            stamp: Time::new(42, 123456789),
            frame_id: "camera_link".to_string(),
        };
        let cdr_bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&header, cdr::size::Infinite).unwrap();
        let enc_bytes = serialize(&header).unwrap();
        assert_eq!(cdr_bytes, enc_bytes, "Header: wire format mismatch");

        // CompressedImage - contains Vec<u8>
        let img = CompressedImage {
            header: header.clone(),
            format: "jpeg".to_string(),
            data: vec![0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10],
        };
        let cdr_bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&img, cdr::size::Infinite).unwrap();
        let enc_bytes = serialize(&img).unwrap();
        assert_eq!(cdr_bytes, enc_bytes, "CompressedImage: wire format mismatch");

        // Model - nested struct with Vec<Box>, Vec<Mask>
        let model = Model {
            header: header.clone(),
            input_time: crate::builtin_interfaces::Duration {
                sec: 0,
                nanosec: 2_500_000,
            },
            model_time: crate::builtin_interfaces::Duration {
                sec: 0,
                nanosec: 15_000_000,
            },
            output_time: crate::builtin_interfaces::Duration {
                sec: 0,
                nanosec: 1_000_000,
            },
            decode_time: crate::builtin_interfaces::Duration {
                sec: 0,
                nanosec: 3_000_000,
            },
            boxes: vec![crate::edgefirst_msgs::Box {
                center_x: 320.0,
                center_y: 240.0,
                width: 100.0,
                height: 80.0,
                label: "person".to_string(),
                score: 0.95,
                distance: 5.0,
                speed: 1.2,
                track: crate::edgefirst_msgs::Track {
                    id: "track_1".to_string(),
                    lifetime: 42,
                    created: Time::new(1000, 0),
                },
            }],
            masks: vec![],
        };
        let cdr_bytes =
            cdr::serialize::<_, _, cdr::CdrLe>(&model, cdr::size::Infinite).unwrap();
        let enc_bytes = serialize(&model).unwrap();
        assert_eq!(cdr_bytes, enc_bytes, "Model: wire format mismatch");
    }
}
