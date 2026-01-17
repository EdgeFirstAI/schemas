// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

use crate::{geometry_msgs, std_msgs};
use serde_derive::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct CameraInfo {
    pub header: std_msgs::Header,
    pub height: u32,
    pub width: u32,
    pub distortion_model: String,
    pub d: Vec<f64>,
    pub k: [f64; 9],
    pub r: [f64; 9],
    pub p: [f64; 12],
    pub binning_x: u32,
    pub binning_y: u32,
    pub roi: RegionOfInterest,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct CompressedImage {
    pub header: std_msgs::Header,
    pub format: String,
    pub data: Vec<u8>,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Image {
    pub header: std_msgs::Header,
    pub height: u32,
    pub width: u32,
    pub encoding: String,
    pub is_bigendian: u8,
    pub step: u32,
    pub data: Vec<u8>,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct IMU {
    pub header: std_msgs::Header,
    pub orientation: geometry_msgs::Quaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: geometry_msgs::Vector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: geometry_msgs::Vector3,
    pub linear_acceleration_covariance: [f64; 9],
}

pub mod nav_sat_fix {
    pub const COVARIANCE_TYPE_UNKNOWN: u8 = 0;
    pub const COVARIANCE_TYPE_APPROXIMATED: u8 = 1;
    pub const COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;
    pub const COVARIANCE_TYPE_KNOWN: u8 = 3;
}
#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct NavSatFix {
    pub header: std_msgs::Header,
    pub status: NavSatStatus,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub position_covariance: [f64; 9],
    pub position_covariance_type: u8,
}

pub mod nav_sat_status {
    pub const STATUS_NO_FIX: i8 = -1; // unable to fix position
    pub const STATUS_FIX: i8 = 0; // unaugmented fix
    pub const STATUS_SBAS_FIX: i8 = 1; // with satellite-based augmentation
    pub const STATUS_GBAS_FIX: i8 = 2; // with ground-based augmentation
    pub const SERVICE_GPS: u8 = 1;
    pub const SERVICE_GLONASS: u8 = 2;
    pub const SERVICE_COMPASS: u8 = 4; // includes BeiDou.
    pub const SERVICE_GALILEO: u8 = 8;
}
#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct NavSatStatus {
    pub status: i8,
    pub service: u16,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct PointCloud2 {
    pub header: std_msgs::Header,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}

pub mod point_field {
    pub const INT8: u8 = 1;
    pub const UINT8: u8 = 2;
    pub const INT16: u8 = 3;
    pub const UINT16: u8 = 4;
    pub const INT32: u8 = 5;
    pub const UINT32: u8 = 6;
    pub const FLOAT32: u8 = 7;
    pub const FLOAT64: u8 = 8;
}
#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct RegionOfInterest {
    pub x_offset: u32,
    pub y_offset: u32,
    pub height: u32,
    pub width: u32,
    pub do_rectify: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::serde_cdr::{deserialize, serialize};

    #[test]
    fn test_point_field_serialize() {
        let field = PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: 7, // FLOAT32
            count: 1,
        };
        let bytes = serialize(&field).unwrap();
        let decoded: PointField = deserialize(&bytes).unwrap();
        assert_eq!(field, decoded);
    }

    #[test]
    fn test_point_cloud2_serialize() {
        let cloud = PointCloud2 {
            header: crate::std_msgs::Header {
                stamp: Time { sec: 100, nanosec: 0 },
                frame_id: "lidar".to_string(),
            },
            height: 1,
            width: 1024,
            fields: vec![
                PointField { name: "x".to_string(), offset: 0, datatype: 7, count: 1 },
                PointField { name: "y".to_string(), offset: 4, datatype: 7, count: 1 },
                PointField { name: "z".to_string(), offset: 8, datatype: 7, count: 1 },
            ],
            is_bigendian: false,
            point_step: 12,
            row_step: 12288,
            data: vec![0u8; 100],
            is_dense: true,
        };
        let bytes = serialize(&cloud).unwrap();
        let decoded: PointCloud2 = deserialize(&bytes).unwrap();
        assert_eq!(cloud, decoded);
    }

    #[test]
    fn test_nav_sat_status_serialize() {
        let status = NavSatStatus {
            status: 0,  // FIX
            service: 1, // GPS
        };
        let bytes = serialize(&status).unwrap();
        let decoded: NavSatStatus = deserialize(&bytes).unwrap();
        assert_eq!(status, decoded);
    }

    #[test]
    fn test_nav_sat_fix_serialize() {
        let fix = NavSatFix {
            header: crate::std_msgs::Header {
                stamp: Time { sec: 100, nanosec: 0 },
                frame_id: "gps".to_string(),
            },
            status: NavSatStatus { status: 0, service: 1 },
            latitude: 45.5017,
            longitude: -73.5673,
            altitude: 100.0,
            position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            position_covariance_type: 2, // DIAGONAL_KNOWN
        };
        let bytes = serialize(&fix).unwrap();
        let decoded: NavSatFix = deserialize(&bytes).unwrap();
        assert_eq!(fix, decoded);
    }

    #[test]
    fn test_image_serialize() {
        let image = Image {
            header: crate::std_msgs::Header {
                stamp: Time { sec: 100, nanosec: 0 },
                frame_id: "camera".to_string(),
            },
            height: 480,
            width: 640,
            encoding: "rgb8".to_string(),
            is_bigendian: 0,
            step: 1920,
            data: vec![0u8; 100],
        };
        let bytes = serialize(&image).unwrap();
        let decoded: Image = deserialize(&bytes).unwrap();
        assert_eq!(image, decoded);
    }
}
