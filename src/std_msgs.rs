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
    fn header_roundtrip() {
        let cases = [
            (0, 0, "", "empty"),
            (100, 500_000_000, "camera", "typical"),
            (42, 0, &"a".repeat(1000), "long frame_id"),
            (1, 0, "camera/optical_frame", "path separator"),
            (i32::MAX, 999_999_999, "max_frame", "max time"),
        ];
        for (sec, nanosec, frame_id, name) in cases {
            let header = Header {
                stamp: Time::new(sec, nanosec),
                frame_id: frame_id.to_string(),
            };
            let bytes = serialize(&header).unwrap();
            let decoded: Header = deserialize(&bytes).unwrap();
            assert_eq!(header, decoded, "failed for case: {}", name);
        }
    }

    #[test]
    fn color_rgba_roundtrip() {
        let cases = [
            (0.0, 0.0, 0.0, 0.0, "zero/transparent"),
            (1.0, 0.0, 0.0, 1.0, "red"),
            (0.0, 1.0, 0.0, 1.0, "green"),
            (0.0, 0.0, 1.0, 1.0, "blue"),
            (1.0, 1.0, 1.0, 1.0, "white"),
            (0.5, 0.5, 0.5, 0.5, "gray semi-transparent"),
        ];
        for (r, g, b, a, name) in cases {
            let color = ColorRGBA { r, g, b, a };
            let bytes = serialize(&color).unwrap();
            let decoded: ColorRGBA = deserialize(&bytes).unwrap();
            assert_eq!(color, decoded, "failed for case: {}", name);
        }
    }
}
