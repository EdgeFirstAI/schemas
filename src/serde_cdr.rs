// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! CDR (Common Data Representation) serialization and deserialization support.
//!
//! This module provides serialize/deserialize functions for all schema types
//! using ROS2 CDR encoding format.

use serde::{Deserialize, Serialize};

/// Error type for serialization/deserialization operations
#[derive(Debug)]
pub enum Error {
    /// CDR serialization error
    Serialization(cdr::Error),
    /// CDR deserialization error
    Deserialization(cdr::Error),
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
    cdr::serialize::<_, _, cdr::CdrLe>(msg, cdr::size::Infinite).map_err(Error::Serialization)
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
    cdr::deserialize(bytes).map_err(Error::Deserialization)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::std_msgs::Header;

    #[test]
    fn test_serialize_deserialize_header() {
        let header = Header {
            stamp: Time {
                sec: 42,
                nanosec: 123456789,
            },
            frame_id: "test_frame".to_string(),
        };

        let bytes = serialize(&header).unwrap();
        let deserialized: Header = deserialize(&bytes).unwrap();

        assert_eq!(header, deserialized);
    }

    #[test]
    fn test_roundtrip_empty_string() {
        let header = Header {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        };

        let bytes = serialize(&header).unwrap();
        let deserialized: Header = deserialize(&bytes).unwrap();

        assert_eq!(header, deserialized);
    }
}
