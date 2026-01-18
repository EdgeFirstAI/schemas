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
