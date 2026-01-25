// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

use crate::{
    builtin_interfaces,
    std_msgs::{self},
};
use serde_derive::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct FoxgloveCompressedVideo {
    pub header: std_msgs::Header,
    pub data: Vec<u8>,
    pub format: String,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct FoxgloveImageAnnotations {
    pub circles: Vec<FoxgloveCircleAnnotations>,
    pub points: Vec<FoxglovePointAnnotations>,
    pub texts: Vec<FoxgloveTextAnnotations>,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct FoxgloveCircleAnnotations {
    pub timestamp: builtin_interfaces::Time,
    pub position: FoxglovePoint2,
    pub diameter: f64,
    pub thickness: f64,
    pub fill_color: FoxgloveColor,
    pub outline_color: FoxgloveColor,
}

pub mod point_annotation_type {
    pub const UNKNOWN: u8 = 0;

    // Individual points: 0, 1, 2, ...
    pub const POINTS: u8 = 1;

    // Closed polygon: 0-1, 1-2, ..., (n-1)-n, n-0
    pub const LINE_LOOP: u8 = 2;

    // Connected line segments: 0-1, 1-2, ..., (n-1)-n
    pub const LINE_STRIP: u8 = 3;

    // Individual line segments: 0-1, 2-3, 4-5, ...
    pub const LINE_LIST: u8 = 4;
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct FoxglovePointAnnotations {
    pub timestamp: builtin_interfaces::Time,
    pub type_: u8,
    pub points: Vec<FoxglovePoint2>,
    pub outline_color: FoxgloveColor,
    pub outline_colors: Vec<FoxgloveColor>,
    pub fill_color: FoxgloveColor,
    pub thickness: f64,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct FoxgloveTextAnnotations {
    pub timestamp: builtin_interfaces::Time,
    pub position: FoxglovePoint2,
    pub text: String,
    pub font_size: f64,
    pub text_color: FoxgloveColor,
    pub background_color: FoxgloveColor,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct FoxglovePoint2 {
    pub x: f64,
    pub y: f64,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct FoxgloveColor {
    pub r: f64,
    pub g: f64,
    pub b: f64,
    pub a: f64,
}

/// Check if a type name is supported by this module.
pub fn is_type_supported(type_name: &str) -> bool {
    matches!(type_name, "CompressedVideo")
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &["foxglove_msgs/msg/CompressedVideo"]
}

// SchemaType implementations
use crate::schema_registry::SchemaType;

impl SchemaType for FoxgloveCompressedVideo {
    const SCHEMA_NAME: &'static str = "foxglove_msgs/msg/CompressedVideo";
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::serde_cdr::{deserialize, serialize};
    use crate::std_msgs::Header;

    #[test]
    fn foxglove_color_roundtrip() {
        let cases = [
            (0.0, 0.0, 0.0, 0.0, "transparent black"),
            (1.0, 1.0, 1.0, 1.0, "opaque white"),
            (1.0, 0.0, 0.0, 1.0, "red"),
            (0.0, 1.0, 0.0, 0.5, "semi-transparent green"),
            (0.5, 0.5, 0.5, 0.75, "gray"),
        ];
        for (r, g, b, a, name) in cases {
            let color = FoxgloveColor { r, g, b, a };
            let bytes = serialize(&color).unwrap();
            let decoded: FoxgloveColor = deserialize(&bytes).unwrap();
            assert_eq!(color, decoded, "failed for case: {}", name);
        }
    }

    #[test]
    fn foxglove_point2_roundtrip() {
        let cases = [
            (0.0, 0.0, "origin"),
            (100.0, 200.0, "positive"),
            (-50.0, -75.0, "negative"),
            (f64::MAX, f64::MIN, "extremes"),
        ];
        for (x, y, name) in cases {
            let point = FoxglovePoint2 { x, y };
            let bytes = serialize(&point).unwrap();
            let decoded: FoxglovePoint2 = deserialize(&bytes).unwrap();
            assert_eq!(point, decoded, "failed for case: {}", name);
        }
    }

    #[test]
    fn foxglove_compressed_video_roundtrip() {
        // Empty video frame
        let empty = FoxgloveCompressedVideo {
            header: Header {
                stamp: Time::new(0, 0),
                frame_id: String::new(),
            },
            data: vec![],
            format: String::new(),
        };
        let bytes = serialize(&empty).unwrap();
        assert_eq!(
            empty,
            deserialize::<FoxgloveCompressedVideo>(&bytes).unwrap()
        );

        // H.264 video frame
        let video = FoxgloveCompressedVideo {
            header: Header {
                stamp: Time::new(100, 500_000_000),
                frame_id: "camera".to_string(),
            },
            data: vec![0x00, 0x00, 0x00, 0x01, 0x67, 0x42], // H.264 NAL header example
            format: "h264".to_string(),
        };
        let bytes = serialize(&video).unwrap();
        assert_eq!(
            video,
            deserialize::<FoxgloveCompressedVideo>(&bytes).unwrap()
        );
    }

    #[test]
    fn foxglove_circle_annotations_roundtrip() {
        let circle = FoxgloveCircleAnnotations {
            timestamp: Time::new(100, 0),
            position: FoxglovePoint2 { x: 320.0, y: 240.0 },
            diameter: 50.0,
            thickness: 2.0,
            fill_color: FoxgloveColor {
                r: 1.0,
                g: 0.0,
                b: 0.0,
                a: 0.5,
            },
            outline_color: FoxgloveColor {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 1.0,
            },
        };
        let bytes = serialize(&circle).unwrap();
        assert_eq!(
            circle,
            deserialize::<FoxgloveCircleAnnotations>(&bytes).unwrap()
        );
    }

    #[test]
    fn foxglove_point_annotations_roundtrip() {
        // Empty points
        let empty = FoxglovePointAnnotations {
            timestamp: Time::new(0, 0),
            type_: point_annotation_type::UNKNOWN,
            points: vec![],
            outline_color: FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.0,
            },
            outline_colors: vec![],
            fill_color: FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.0,
            },
            thickness: 0.0,
        };
        let bytes = serialize(&empty).unwrap();
        assert_eq!(
            empty,
            deserialize::<FoxglovePointAnnotations>(&bytes).unwrap()
        );

        // Polygon (LINE_LOOP)
        let polygon = FoxglovePointAnnotations {
            timestamp: Time::new(100, 0),
            type_: point_annotation_type::LINE_LOOP,
            points: vec![
                FoxglovePoint2 { x: 0.0, y: 0.0 },
                FoxglovePoint2 { x: 100.0, y: 0.0 },
                FoxglovePoint2 { x: 100.0, y: 100.0 },
                FoxglovePoint2 { x: 0.0, y: 100.0 },
            ],
            outline_color: FoxgloveColor {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 1.0,
            },
            outline_colors: vec![],
            fill_color: FoxgloveColor {
                r: 0.0,
                g: 0.5,
                b: 0.0,
                a: 0.3,
            },
            thickness: 3.0,
        };
        let bytes = serialize(&polygon).unwrap();
        assert_eq!(
            polygon,
            deserialize::<FoxglovePointAnnotations>(&bytes).unwrap()
        );

        // Points with per-point colors
        let colored_points = FoxglovePointAnnotations {
            timestamp: Time::new(200, 0),
            type_: point_annotation_type::POINTS,
            points: vec![
                FoxglovePoint2 { x: 10.0, y: 20.0 },
                FoxglovePoint2 { x: 30.0, y: 40.0 },
            ],
            outline_color: FoxgloveColor {
                r: 1.0,
                g: 1.0,
                b: 1.0,
                a: 1.0,
            },
            outline_colors: vec![
                FoxgloveColor {
                    r: 1.0,
                    g: 0.0,
                    b: 0.0,
                    a: 1.0,
                },
                FoxgloveColor {
                    r: 0.0,
                    g: 0.0,
                    b: 1.0,
                    a: 1.0,
                },
            ],
            fill_color: FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.0,
            },
            thickness: 5.0,
        };
        let bytes = serialize(&colored_points).unwrap();
        assert_eq!(
            colored_points,
            deserialize::<FoxglovePointAnnotations>(&bytes).unwrap()
        );
    }

    #[test]
    fn foxglove_text_annotations_roundtrip() {
        let text = FoxgloveTextAnnotations {
            timestamp: Time::new(100, 0),
            position: FoxglovePoint2 { x: 50.0, y: 50.0 },
            text: "Detection: car (98%)".to_string(),
            font_size: 14.0,
            text_color: FoxgloveColor {
                r: 1.0,
                g: 1.0,
                b: 1.0,
                a: 1.0,
            },
            background_color: FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.7,
            },
        };
        let bytes = serialize(&text).unwrap();
        assert_eq!(
            text,
            deserialize::<FoxgloveTextAnnotations>(&bytes).unwrap()
        );

        // Empty text
        let empty_text = FoxgloveTextAnnotations {
            timestamp: Time::new(0, 0),
            position: FoxglovePoint2 { x: 0.0, y: 0.0 },
            text: String::new(),
            font_size: 0.0,
            text_color: FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.0,
            },
            background_color: FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.0,
            },
        };
        let bytes = serialize(&empty_text).unwrap();
        assert_eq!(
            empty_text,
            deserialize::<FoxgloveTextAnnotations>(&bytes).unwrap()
        );
    }

    #[test]
    fn foxglove_image_annotations_roundtrip() {
        // Empty annotations
        let empty = FoxgloveImageAnnotations {
            circles: vec![],
            points: vec![],
            texts: vec![],
        };
        let bytes = serialize(&empty).unwrap();
        assert_eq!(
            empty,
            deserialize::<FoxgloveImageAnnotations>(&bytes).unwrap()
        );

        // Complex annotation set
        let annotations = FoxgloveImageAnnotations {
            circles: vec![FoxgloveCircleAnnotations {
                timestamp: Time::new(100, 0),
                position: FoxglovePoint2 { x: 100.0, y: 100.0 },
                diameter: 30.0,
                thickness: 2.0,
                fill_color: FoxgloveColor {
                    r: 1.0,
                    g: 0.0,
                    b: 0.0,
                    a: 0.5,
                },
                outline_color: FoxgloveColor {
                    r: 1.0,
                    g: 1.0,
                    b: 0.0,
                    a: 1.0,
                },
            }],
            points: vec![FoxglovePointAnnotations {
                timestamp: Time::new(100, 0),
                type_: point_annotation_type::LINE_STRIP,
                points: vec![
                    FoxglovePoint2 { x: 0.0, y: 0.0 },
                    FoxglovePoint2 { x: 640.0, y: 480.0 },
                ],
                outline_color: FoxgloveColor {
                    r: 0.0,
                    g: 1.0,
                    b: 0.0,
                    a: 1.0,
                },
                outline_colors: vec![],
                fill_color: FoxgloveColor {
                    r: 0.0,
                    g: 0.0,
                    b: 0.0,
                    a: 0.0,
                },
                thickness: 1.0,
            }],
            texts: vec![FoxgloveTextAnnotations {
                timestamp: Time::new(100, 0),
                position: FoxglovePoint2 { x: 10.0, y: 10.0 },
                text: "Label".to_string(),
                font_size: 12.0,
                text_color: FoxgloveColor {
                    r: 1.0,
                    g: 1.0,
                    b: 1.0,
                    a: 1.0,
                },
                background_color: FoxgloveColor {
                    r: 0.0,
                    g: 0.0,
                    b: 0.0,
                    a: 0.5,
                },
            }],
        };
        let bytes = serialize(&annotations).unwrap();
        assert_eq!(
            annotations,
            deserialize::<FoxgloveImageAnnotations>(&bytes).unwrap()
        );
    }
}
