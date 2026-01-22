// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! # EdgeFirst Middleware Schemas
//!
//! This library provides the Rust structs for EdgeFirst Middleware messages.
//!
//! Common Rust struct for ROS 2 messages used by EdgeFirst Middleware Services with Zenoh.
//!
//! Here are some ROS message source:
//!
//! * [common_interface](https://github.com/ros2/common_interfaces): Common-used ROS message
//! * [rcl_interface](https://github.com/ros2/rcl_interfaces): Common interface in RCL
//! * [foxglove_api_msgs](https://github.com/foxglove/schemas/tree/main/ros_foxglove_msgs)
//! * [edgefirst_api_msgs](https://github.com/EdgeFirstAI/schemas): EdgeFirst ROS messages

/// EdgeFirst Messages
pub mod edgefirst_msgs;

/// Foxglove Messages
pub mod foxglove_msgs;

/// ROS 2 Common Interfaces
pub mod geometry_msgs;
pub mod sensor_msgs;
pub mod std_msgs;

/// ROS 2 RCL Interfaces
pub mod builtin_interfaces;
pub mod rosgraph_msgs;

pub mod service;

/// CDR serialization/deserialization support
pub mod serde_cdr;

/// C FFI bindings
mod ffi;

use sensor_msgs::{point_field, PointCloud2, PointField};
use std::collections::HashMap;

const SIZE_OF_DATATYPE: [usize; 9] = [
    0, 1, // pub const INT8: u8 = 1;
    1, // pub const UINT8: u8 = 2;
    2, // pub const INT16: u8 = 3;
    2, // pub const UINT16: u8 = 4;
    4, // pub const INT32: u8 = 5;
    4, // pub const UINT32: u8 = 6;
    4, // pub const FLOAT32: u8 = 7;
    8, //pub const FLOAT64: u8 = 8;
];

pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub id: isize,
    pub fields: HashMap<String, f64>,
}

/// This function takes a PointCloud2 message and decodes it into a vector of Points.
/// Each Point contains the x, y, z coordinates, an id, and a HashMap of additional fields.
pub fn decode_pcd(pcd: &PointCloud2) -> Vec<Point> {
    let mut points = Vec::new();
    for i in 0..pcd.height {
        for j in 0..pcd.width {
            let start = (i * pcd.row_step + j * pcd.point_step) as usize;
            let end = start + pcd.point_step as usize;
            let p = if pcd.is_bigendian {
                parse_point_be(&pcd.fields, &pcd.data[start..end])
            } else {
                parse_point_le(&pcd.fields, &pcd.data[start..end])
            };
            points.push(p);
        }
    }
    points
}

fn parse_point_le(fields: &[PointField], data: &[u8]) -> Point {
    let mut p = Point {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        id: 0,
        fields: HashMap::new(),
    };
    for f in fields {
        let start = f.offset as usize;
        let val = match f.datatype {
            point_field::INT8 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::INT8 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                i8::from_le_bytes(bytes) as f64
            }
            point_field::UINT8 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::UINT8 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                u8::from_le_bytes(bytes) as f64
            }
            point_field::INT16 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::INT16 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                i16::from_le_bytes(bytes) as f64
            }
            point_field::UINT16 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::UINT16 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                u16::from_le_bytes(bytes) as f64
            }
            point_field::INT32 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::INT32 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                i32::from_le_bytes(bytes) as f64
            }
            point_field::UINT32 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::UINT32 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                u32::from_le_bytes(bytes) as f64
            }
            point_field::FLOAT32 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::FLOAT32 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                f32::from_le_bytes(bytes) as f64
            }
            point_field::FLOAT64 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::FLOAT64 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                f64::from_le_bytes(bytes)
            }
            _ => {
                // Unknown datatype in PointField
                continue;
            }
        };
        match f.name.as_str() {
            "x" => p.x = val,
            "y" => p.y = val,
            "z" => p.z = val,
            "cluster_id" => p.id = val as isize,
            _ => {
                p.fields.insert(f.name.clone(), val);
            }
        }
    }
    p
}

fn parse_point_be(fields: &[PointField], data: &[u8]) -> Point {
    let mut p = Point {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        id: 0,
        fields: HashMap::new(),
    };
    for f in fields {
        let start = f.offset as usize;

        let val = match f.datatype {
            point_field::INT8 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::INT8 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                i8::from_be_bytes(bytes) as f64
            }
            point_field::UINT8 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::UINT8 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                u8::from_be_bytes(bytes) as f64
            }
            point_field::INT16 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::INT16 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                i16::from_be_bytes(bytes) as f64
            }
            point_field::UINT16 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::UINT16 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                u16::from_be_bytes(bytes) as f64
            }
            point_field::INT32 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::INT32 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                i32::from_be_bytes(bytes) as f64
            }
            point_field::UINT32 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::UINT32 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                u32::from_be_bytes(bytes) as f64
            }
            point_field::FLOAT32 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::FLOAT32 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                f32::from_be_bytes(bytes) as f64
            }
            point_field::FLOAT64 => {
                let bytes = data[start..start + SIZE_OF_DATATYPE[point_field::FLOAT64 as usize]]
                    .try_into()
                    .unwrap_or_else(|e| panic!("Expected slice with 1 element: {:?}", e));
                f64::from_be_bytes(bytes)
            }
            _ => {
                // "Unknown datatype in PointField
                continue;
            }
        };
        match f.name.as_str() {
            "x" => p.x = val,
            "y" => p.y = val,
            "z" => p.z = val,
            _ => {
                p.fields.insert(f.name.clone(), val);
            }
        }
    }

    p
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::std_msgs::Header;

    /// Helper to create a PointCloud2 with FLOAT32 x/y/z fields
    fn make_xyz_cloud(points_data: &[[f32; 3]], is_bigendian: bool) -> PointCloud2 {
        let fields = vec![
            PointField {
                name: "x".to_string(),
                offset: 0,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "y".to_string(),
                offset: 4,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "z".to_string(),
                offset: 8,
                datatype: point_field::FLOAT32,
                count: 1,
            },
        ];

        let point_step = 12u32;
        let width = points_data.len() as u32;
        let row_step = point_step * width;

        let mut data = Vec::with_capacity(points_data.len() * 12);
        for p in points_data {
            for val in p {
                if is_bigendian {
                    data.extend_from_slice(&val.to_be_bytes());
                } else {
                    data.extend_from_slice(&val.to_le_bytes());
                }
            }
        }

        PointCloud2 {
            header: Header {
                stamp: Time::new(0, 0),
                frame_id: "test".to_string(),
            },
            height: 1,
            width,
            fields,
            is_bigendian,
            point_step,
            row_step,
            data,
            is_dense: true,
        }
    }

    #[test]
    fn decode_pcd_basic_xyz_little_endian() {
        let input = [[1.0f32, 2.0, 3.0], [4.0, 5.0, 6.0], [-1.0, -2.0, -3.0]];
        let cloud = make_xyz_cloud(&input, false);
        let points = decode_pcd(&cloud);

        assert_eq!(points.len(), 3);
        assert!((points[0].x - 1.0).abs() < 1e-6);
        assert!((points[0].y - 2.0).abs() < 1e-6);
        assert!((points[0].z - 3.0).abs() < 1e-6);
        assert!((points[1].x - 4.0).abs() < 1e-6);
        assert!((points[2].x - (-1.0)).abs() < 1e-6);
    }

    #[test]
    fn decode_pcd_basic_xyz_big_endian() {
        let input = [[10.0f32, 20.0, 30.0]];
        let cloud = make_xyz_cloud(&input, true);
        let points = decode_pcd(&cloud);

        assert_eq!(points.len(), 1);
        assert!((points[0].x - 10.0).abs() < 1e-6);
        assert!((points[0].y - 20.0).abs() < 1e-6);
        assert!((points[0].z - 30.0).abs() < 1e-6);
    }

    #[test]
    fn decode_pcd_empty_cloud() {
        let cloud = PointCloud2 {
            header: Header {
                stamp: Time::new(0, 0),
                frame_id: String::new(),
            },
            height: 0,
            width: 0,
            fields: vec![],
            is_bigendian: false,
            point_step: 0,
            row_step: 0,
            data: vec![],
            is_dense: true,
        };
        let points = decode_pcd(&cloud);
        assert!(points.is_empty());
    }

    #[test]
    fn decode_pcd_with_cluster_id() {
        // x(f32) + y(f32) + z(f32) + cluster_id(i32) = 16 bytes per point
        let fields = vec![
            PointField {
                name: "x".to_string(),
                offset: 0,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "y".to_string(),
                offset: 4,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "z".to_string(),
                offset: 8,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "cluster_id".to_string(),
                offset: 12,
                datatype: point_field::INT32,
                count: 1,
            },
        ];

        let mut data = Vec::new();
        // Point 1: (1,2,3) cluster_id=42
        data.extend_from_slice(&1.0f32.to_le_bytes());
        data.extend_from_slice(&2.0f32.to_le_bytes());
        data.extend_from_slice(&3.0f32.to_le_bytes());
        data.extend_from_slice(&42i32.to_le_bytes());
        // Point 2: (4,5,6) cluster_id=-1
        data.extend_from_slice(&4.0f32.to_le_bytes());
        data.extend_from_slice(&5.0f32.to_le_bytes());
        data.extend_from_slice(&6.0f32.to_le_bytes());
        data.extend_from_slice(&(-1i32).to_le_bytes());

        let cloud = PointCloud2 {
            header: Header {
                stamp: Time::new(100, 0),
                frame_id: "lidar".to_string(),
            },
            height: 1,
            width: 2,
            fields,
            is_bigendian: false,
            point_step: 16,
            row_step: 32,
            data,
            is_dense: true,
        };

        let points = decode_pcd(&cloud);
        assert_eq!(points.len(), 2);
        assert_eq!(points[0].id, 42);
        assert_eq!(points[1].id, -1);
    }

    #[test]
    fn decode_pcd_with_custom_fields() {
        // Fusion output with vision_class field (as used in samples)
        let fields = vec![
            PointField {
                name: "x".to_string(),
                offset: 0,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "y".to_string(),
                offset: 4,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "z".to_string(),
                offset: 8,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "vision_class".to_string(),
                offset: 12,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "intensity".to_string(),
                offset: 16,
                datatype: point_field::UINT8,
                count: 1,
            },
        ];

        let mut data = Vec::new();
        data.extend_from_slice(&1.0f32.to_le_bytes());
        data.extend_from_slice(&2.0f32.to_le_bytes());
        data.extend_from_slice(&3.0f32.to_le_bytes());
        data.extend_from_slice(&5.0f32.to_le_bytes()); // vision_class = 5
        data.push(200u8); // intensity = 200
                          // Pad to point_step
        data.extend_from_slice(&[0u8; 3]);

        let cloud = PointCloud2 {
            header: Header {
                stamp: Time::new(0, 0),
                frame_id: "fusion".to_string(),
            },
            height: 1,
            width: 1,
            fields,
            is_bigendian: false,
            point_step: 20,
            row_step: 20,
            data,
            is_dense: true,
        };

        let points = decode_pcd(&cloud);
        assert_eq!(points.len(), 1);
        assert!((points[0].fields["vision_class"] - 5.0).abs() < 1e-6);
        assert!((points[0].fields["intensity"] - 200.0).abs() < 1e-6);
    }

    #[test]
    fn decode_pcd_all_datatypes_little_endian() {
        // Test all supported datatypes
        let fields = vec![
            PointField {
                name: "i8".to_string(),
                offset: 0,
                datatype: point_field::INT8,
                count: 1,
            },
            PointField {
                name: "u8".to_string(),
                offset: 1,
                datatype: point_field::UINT8,
                count: 1,
            },
            PointField {
                name: "i16".to_string(),
                offset: 2,
                datatype: point_field::INT16,
                count: 1,
            },
            PointField {
                name: "u16".to_string(),
                offset: 4,
                datatype: point_field::UINT16,
                count: 1,
            },
            PointField {
                name: "i32".to_string(),
                offset: 6,
                datatype: point_field::INT32,
                count: 1,
            },
            PointField {
                name: "u32".to_string(),
                offset: 10,
                datatype: point_field::UINT32,
                count: 1,
            },
            PointField {
                name: "f32".to_string(),
                offset: 14,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "f64".to_string(),
                offset: 18,
                datatype: point_field::FLOAT64,
                count: 1,
            },
        ];

        let mut data = Vec::new();
        data.extend_from_slice(&(-100i8).to_le_bytes());
        data.extend_from_slice(&200u8.to_le_bytes());
        data.extend_from_slice(&(-1000i16).to_le_bytes());
        data.extend_from_slice(&50000u16.to_le_bytes());
        data.extend_from_slice(&(-100000i32).to_le_bytes());
        data.extend_from_slice(&3000000000u32.to_le_bytes());
        data.extend_from_slice(&std::f32::consts::PI.to_le_bytes());
        data.extend_from_slice(&std::f64::consts::E.to_le_bytes());

        let cloud = PointCloud2 {
            header: Header {
                stamp: Time::new(0, 0),
                frame_id: String::new(),
            },
            height: 1,
            width: 1,
            fields,
            is_bigendian: false,
            point_step: 26,
            row_step: 26,
            data,
            is_dense: true,
        };

        let points = decode_pcd(&cloud);
        assert_eq!(points.len(), 1);
        let p = &points[0];
        assert!((p.fields["i8"] - (-100.0)).abs() < 1e-6);
        assert!((p.fields["u8"] - 200.0).abs() < 1e-6);
        assert!((p.fields["i16"] - (-1000.0)).abs() < 1e-6);
        assert!((p.fields["u16"] - 50000.0).abs() < 1e-6);
        assert!((p.fields["i32"] - (-100000.0)).abs() < 1e-6);
        assert!((p.fields["u32"] - 3000000000.0).abs() < 1e-6);
        assert!((p.fields["f32"] - std::f32::consts::PI as f64).abs() < 1e-6);
        assert!((p.fields["f64"] - std::f64::consts::E).abs() < 1e-9);
    }

    #[test]
    fn decode_pcd_all_datatypes_big_endian() {
        let fields = vec![
            PointField {
                name: "i16".to_string(),
                offset: 0,
                datatype: point_field::INT16,
                count: 1,
            },
            PointField {
                name: "u32".to_string(),
                offset: 2,
                datatype: point_field::UINT32,
                count: 1,
            },
            PointField {
                name: "f64".to_string(),
                offset: 6,
                datatype: point_field::FLOAT64,
                count: 1,
            },
        ];

        let mut data = Vec::new();
        data.extend_from_slice(&(-500i16).to_be_bytes());
        data.extend_from_slice(&123456789u32.to_be_bytes());
        data.extend_from_slice(&1.23456789f64.to_be_bytes());

        let cloud = PointCloud2 {
            header: Header {
                stamp: Time::new(0, 0),
                frame_id: String::new(),
            },
            height: 1,
            width: 1,
            fields,
            is_bigendian: true,
            point_step: 14,
            row_step: 14,
            data,
            is_dense: true,
        };

        let points = decode_pcd(&cloud);
        let p = &points[0];
        assert!((p.fields["i16"] - (-500.0)).abs() < 1e-6);
        assert!((p.fields["u32"] - 123456789.0).abs() < 1e-6);
        assert!((p.fields["f64"] - 1.23456789).abs() < 1e-9);
    }

    #[test]
    fn decode_pcd_unknown_datatype_skipped() {
        // Unknown datatype (99) should be silently skipped
        let fields = vec![
            PointField {
                name: "x".to_string(),
                offset: 0,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "unknown".to_string(),
                offset: 4,
                datatype: 99, // Invalid datatype
                count: 1,
            },
        ];

        let mut data = Vec::new();
        data.extend_from_slice(&42.0f32.to_le_bytes());
        data.extend_from_slice(&[0u8; 4]); // Padding for unknown field

        let cloud = PointCloud2 {
            header: Header {
                stamp: Time::new(0, 0),
                frame_id: String::new(),
            },
            height: 1,
            width: 1,
            fields,
            is_bigendian: false,
            point_step: 8,
            row_step: 8,
            data,
            is_dense: true,
        };

        let points = decode_pcd(&cloud);
        assert_eq!(points.len(), 1);
        assert!((points[0].x - 42.0).abs() < 1e-6);
        // Unknown field should not be in the map
        assert!(!points[0].fields.contains_key("unknown"));
    }

    #[test]
    fn decode_pcd_multi_row() {
        // 2x3 organized point cloud (2 rows, 3 columns)
        let input = [
            [1.0f32, 1.0, 1.0],
            [2.0, 2.0, 2.0],
            [3.0, 3.0, 3.0],
            [4.0, 4.0, 4.0],
            [5.0, 5.0, 5.0],
            [6.0, 6.0, 6.0],
        ];

        let fields = vec![
            PointField {
                name: "x".to_string(),
                offset: 0,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "y".to_string(),
                offset: 4,
                datatype: point_field::FLOAT32,
                count: 1,
            },
            PointField {
                name: "z".to_string(),
                offset: 8,
                datatype: point_field::FLOAT32,
                count: 1,
            },
        ];

        let mut data = Vec::new();
        for p in &input {
            for val in p {
                data.extend_from_slice(&val.to_le_bytes());
            }
        }

        let cloud = PointCloud2 {
            header: Header {
                stamp: Time::new(0, 0),
                frame_id: "camera".to_string(),
            },
            height: 2,
            width: 3,
            fields,
            is_bigendian: false,
            point_step: 12,
            row_step: 36, // 3 points * 12 bytes
            data,
            is_dense: true,
        };

        let points = decode_pcd(&cloud);
        assert_eq!(points.len(), 6);
        for (i, p) in points.iter().enumerate() {
            let expected = (i + 1) as f64;
            assert!((p.x - expected).abs() < 1e-6, "point {} x mismatch", i);
        }
    }
}
