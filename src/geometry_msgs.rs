// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

use crate::std_msgs;
use serde_derive::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Accel {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct AccelStamped {
    pub header: std_msgs::Header,
    pub accel: Accel,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Inertia {
    pub m: f64,
    pub com: Vector3,
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct InertiaStamped {
    pub header: std_msgs::Header,
    pub inertia: Inertia,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Point32 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct PointStamped {
    pub header: std_msgs::Header,
    pub point: Point,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct TransformStamped {
    pub header: std_msgs::Header,
    pub child_frame_id: String,
    pub transform: Transform,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct TwistStamped {
    pub header: std_msgs::Header,
    pub twist: Twist,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::serde_cdr::{deserialize, serialize};

    #[test]
    fn test_vector3_serialize() {
        let vec = Vector3 { x: 1.5, y: 2.5, z: 3.5 };
        let bytes = serialize(&vec).unwrap();
        let decoded: Vector3 = deserialize(&bytes).unwrap();
        assert_eq!(vec, decoded);
    }

    #[test]
    fn test_point_origin() {
        let point = Point { x: 0.0, y: 0.0, z: 0.0 };
        let bytes = serialize(&point).unwrap();
        let decoded: Point = deserialize(&bytes).unwrap();
        assert_eq!(point, decoded);
    }

    #[test]
    fn test_point32_serialize() {
        let point = Point32 { x: 1.0, y: 2.0, z: 3.0 };
        let bytes = serialize(&point).unwrap();
        let decoded: Point32 = deserialize(&bytes).unwrap();
        assert_eq!(point, decoded);
    }

    #[test]
    fn test_quaternion_identity() {
        let quat = Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 };
        let bytes = serialize(&quat).unwrap();
        let decoded: Quaternion = deserialize(&bytes).unwrap();
        assert_eq!(quat, decoded);
    }

    #[test]
    fn test_pose_serialize() {
        let pose = Pose {
            position: Point { x: 1.0, y: 2.0, z: 0.5 },
            orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        };
        let bytes = serialize(&pose).unwrap();
        let decoded: Pose = deserialize(&bytes).unwrap();
        assert_eq!(pose, decoded);
    }

    #[test]
    fn test_pose2d_serialize() {
        let pose = Pose2D { x: 10.0, y: 20.0, theta: 1.57 };
        let bytes = serialize(&pose).unwrap();
        let decoded: Pose2D = deserialize(&bytes).unwrap();
        assert_eq!(pose, decoded);
    }

    #[test]
    fn test_transform_serialize() {
        let transform = Transform {
            translation: Vector3 { x: 1.0, y: 2.0, z: 3.0 },
            rotation: Quaternion { x: 0.0, y: 0.0, z: 0.707, w: 0.707 },
        };
        let bytes = serialize(&transform).unwrap();
        let decoded: Transform = deserialize(&bytes).unwrap();
        assert_eq!(transform, decoded);
    }

    #[test]
    fn test_twist_serialize() {
        let twist = Twist {
            linear: Vector3 { x: 1.0, y: 0.0, z: 0.0 },
            angular: Vector3 { x: 0.0, y: 0.0, z: 0.5 },
        };
        let bytes = serialize(&twist).unwrap();
        let decoded: Twist = deserialize(&bytes).unwrap();
        assert_eq!(twist, decoded);
    }

    #[test]
    fn test_accel_serialize() {
        let accel = Accel {
            linear: Vector3 { x: 9.8, y: 0.0, z: 0.0 },
            angular: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
        };
        let bytes = serialize(&accel).unwrap();
        let decoded: Accel = deserialize(&bytes).unwrap();
        assert_eq!(accel, decoded);
    }

    #[test]
    fn test_inertia_serialize() {
        let inertia = Inertia {
            m: 10.0,
            com: Vector3 { x: 0.0, y: 0.0, z: 0.0 },
            ixx: 1.0, ixy: 0.0, ixz: 0.0,
            iyy: 1.0, iyz: 0.0,
            izz: 1.0,
        };
        let bytes = serialize(&inertia).unwrap();
        let decoded: Inertia = deserialize(&bytes).unwrap();
        assert_eq!(inertia, decoded);
    }

    #[test]
    fn test_transform_stamped_serialize() {
        let transform = TransformStamped {
            header: crate::std_msgs::Header {
                stamp: Time { sec: 100, nanosec: 0 },
                frame_id: "map".to_string(),
            },
            child_frame_id: "base_link".to_string(),
            transform: Transform {
                translation: Vector3 { x: 1.0, y: 2.0, z: 0.0 },
                rotation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            },
        };
        let bytes = serialize(&transform).unwrap();
        let decoded: TransformStamped = deserialize(&bytes).unwrap();
        assert_eq!(transform, decoded);
    }
}
