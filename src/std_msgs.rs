// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

use crate::builtin_interfaces::Time;
use serde_derive::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct ColorRGBA {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serde_cdr::{deserialize, serialize};

    #[test]
    fn test_header_new() {
        let header = Header {
            stamp: Time {
                sec: 100,
                nanosec: 500_000_000,
            },
            frame_id: "camera".to_string(),
        };
        assert_eq!(header.stamp.sec, 100);
        assert_eq!(header.frame_id, "camera");
    }

    #[test]
    fn test_header_serialize_deserialize() {
        let header = Header {
            stamp: Time {
                sec: 1234567890,
                nanosec: 123456789,
            },
            frame_id: "test_frame".to_string(),
        };
        let bytes = serialize(&header).unwrap();
        let decoded: Header = deserialize(&bytes).unwrap();
        assert_eq!(header, decoded);
    }

    #[test]
    fn test_header_empty_frame_id() {
        let header = Header {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: String::new(),
        };
        let bytes = serialize(&header).unwrap();
        let decoded: Header = deserialize(&bytes).unwrap();
        assert_eq!(header, decoded);
    }

    #[test]
    fn test_header_long_frame_id() {
        let long_id = "a".repeat(1000);
        let header = Header {
            stamp: Time {
                sec: 42,
                nanosec: 0,
            },
            frame_id: long_id.clone(),
        };
        let bytes = serialize(&header).unwrap();
        let decoded: Header = deserialize(&bytes).unwrap();
        assert_eq!(decoded.frame_id, long_id);
    }

    #[test]
    fn test_header_special_chars() {
        let header = Header {
            stamp: Time { sec: 1, nanosec: 0 },
            frame_id: "camera/optical_frame".to_string(),
        };
        let bytes = serialize(&header).unwrap();
        let decoded: Header = deserialize(&bytes).unwrap();
        assert_eq!(header, decoded);
    }

    #[test]
    fn test_color_rgba_red() {
        let color = ColorRGBA {
            r: 1.0,
            g: 0.0,
            b: 0.0,
            a: 1.0,
        };
        let bytes = serialize(&color).unwrap();
        let decoded: ColorRGBA = deserialize(&bytes).unwrap();
        assert_eq!(color, decoded);
    }

    #[test]
    fn test_color_rgba_transparent() {
        let color = ColorRGBA {
            r: 0.5,
            g: 0.5,
            b: 0.5,
            a: 0.5,
        };
        let bytes = serialize(&color).unwrap();
        let decoded: ColorRGBA = deserialize(&bytes).unwrap();
        assert_eq!(color, decoded);
    }

    #[test]
    fn test_color_rgba_zero() {
        let color = ColorRGBA {
            r: 0.0,
            g: 0.0,
            b: 0.0,
            a: 0.0,
        };
        let bytes = serialize(&color).unwrap();
        let decoded: ColorRGBA = deserialize(&bytes).unwrap();
        assert_eq!(color, decoded);
    }

    #[test]
    fn test_color_rgba_full() {
        let color = ColorRGBA {
            r: 1.0,
            g: 1.0,
            b: 1.0,
            a: 1.0,
        };
        let bytes = serialize(&color).unwrap();
        let decoded: ColorRGBA = deserialize(&bytes).unwrap();
        assert_eq!(color, decoded);
    }
}
