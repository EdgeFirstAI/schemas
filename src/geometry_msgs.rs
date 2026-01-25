// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

use crate::std_msgs;
use serde_derive::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq, Clone, Copy, Debug)]
pub struct Accel {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct AccelStamped {
    pub header: std_msgs::Header,
    pub accel: Accel,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Copy, Debug)]
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

#[derive(Serialize, Deserialize, PartialEq, Clone, Copy, Debug)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Copy, Debug)]
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

#[derive(Serialize, Deserialize, PartialEq, Clone, Copy, Debug)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Copy, Debug)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Copy, Debug)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Copy, Debug)]
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

#[derive(Serialize, Deserialize, PartialEq, Clone, Copy, Debug)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct TwistStamped {
    pub header: std_msgs::Header,
    pub twist: Twist,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Copy, Debug)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Check if a type name is supported by this module.
pub fn is_type_supported(type_name: &str) -> bool {
    matches!(
        type_name,
        "Accel"
            | "AccelStamped"
            | "Inertia"
            | "InertiaStamped"
            | "Point"
            | "Point32"
            | "PointStamped"
            | "Pose"
            | "Pose2D"
            | "Quaternion"
            | "Transform"
            | "TransformStamped"
            | "Twist"
            | "TwistStamped"
            | "Vector3"
    )
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &[
        "geometry_msgs/msg/Accel",
        "geometry_msgs/msg/AccelStamped",
        "geometry_msgs/msg/Inertia",
        "geometry_msgs/msg/InertiaStamped",
        "geometry_msgs/msg/Point",
        "geometry_msgs/msg/Point32",
        "geometry_msgs/msg/PointStamped",
        "geometry_msgs/msg/Pose",
        "geometry_msgs/msg/Pose2D",
        "geometry_msgs/msg/Quaternion",
        "geometry_msgs/msg/Transform",
        "geometry_msgs/msg/TransformStamped",
        "geometry_msgs/msg/Twist",
        "geometry_msgs/msg/TwistStamped",
        "geometry_msgs/msg/Vector3",
    ]
}

// SchemaType implementations
use crate::schema_registry::SchemaType;

impl SchemaType for Accel {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Accel";
}

impl SchemaType for AccelStamped {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/AccelStamped";
}

impl SchemaType for Inertia {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Inertia";
}

impl SchemaType for InertiaStamped {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/InertiaStamped";
}

impl SchemaType for Point {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Point";
}

impl SchemaType for Point32 {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Point32";
}

impl SchemaType for PointStamped {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/PointStamped";
}

impl SchemaType for Pose {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Pose";
}

impl SchemaType for Pose2D {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Pose2D";
}

impl SchemaType for Quaternion {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Quaternion";
}

impl SchemaType for Transform {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Transform";
}

impl SchemaType for TransformStamped {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/TransformStamped";
}

impl SchemaType for Twist {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Twist";
}

impl SchemaType for TwistStamped {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/TwistStamped";
}

impl SchemaType for Vector3 {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Vector3";
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::serde_cdr::{deserialize, serialize};

    #[test]
    fn primitive_types_roundtrip() {
        // Vector3, Point, Point32, Quaternion - basic building blocks
        let vec = Vector3 {
            x: 1.5,
            y: -2.5,
            z: f64::MAX,
        };
        let bytes = serialize(&vec).unwrap();
        assert_eq!(vec, deserialize::<Vector3>(&bytes).unwrap());

        let point = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let bytes = serialize(&point).unwrap();
        assert_eq!(point, deserialize::<Point>(&bytes).unwrap());

        let point32 = Point32 {
            x: 1.0f32,
            y: 2.0,
            z: f32::MIN,
        };
        let bytes = serialize(&point32).unwrap();
        assert_eq!(point32, deserialize::<Point32>(&bytes).unwrap());

        // Identity quaternion
        let quat = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        let bytes = serialize(&quat).unwrap();
        assert_eq!(quat, deserialize::<Quaternion>(&bytes).unwrap());

        // 90-degree rotation around Z
        let quat_rot = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.707,
            w: 0.707,
        };
        let bytes = serialize(&quat_rot).unwrap();
        assert_eq!(quat_rot, deserialize::<Quaternion>(&bytes).unwrap());
    }

    #[test]
    fn composite_types_roundtrip() {
        // Pose, Pose2D, Transform, Twist, Accel
        let pose = Pose {
            position: Point {
                x: 1.0,
                y: 2.0,
                z: 0.5,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        };
        let bytes = serialize(&pose).unwrap();
        assert_eq!(pose, deserialize::<Pose>(&bytes).unwrap());

        let pose2d = Pose2D {
            x: 10.0,
            y: 20.0,
            theta: std::f64::consts::PI,
        };
        let bytes = serialize(&pose2d).unwrap();
        assert_eq!(pose2d, deserialize::<Pose2D>(&bytes).unwrap());

        let transform = Transform {
            translation: Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            rotation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.707,
                w: 0.707,
            },
        };
        let bytes = serialize(&transform).unwrap();
        assert_eq!(transform, deserialize::<Transform>(&bytes).unwrap());

        let twist = Twist {
            linear: Vector3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.5,
            },
        };
        let bytes = serialize(&twist).unwrap();
        assert_eq!(twist, deserialize::<Twist>(&bytes).unwrap());

        let accel = Accel {
            linear: Vector3 {
                x: 9.8,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        };
        let bytes = serialize(&accel).unwrap();
        assert_eq!(accel, deserialize::<Accel>(&bytes).unwrap());
    }

    #[test]
    fn inertia_roundtrip() {
        let inertia = Inertia {
            m: 10.0,
            com: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            ixx: 1.0,
            ixy: 0.0,
            ixz: 0.0,
            iyy: 1.0,
            iyz: 0.0,
            izz: 1.0,
        };
        let bytes = serialize(&inertia).unwrap();
        assert_eq!(inertia, deserialize::<Inertia>(&bytes).unwrap());
    }

    #[test]
    fn transform_stamped_roundtrip() {
        let ts = TransformStamped {
            header: crate::std_msgs::Header {
                stamp: Time::new(100, 0),
                frame_id: "map".to_string(),
            },
            child_frame_id: "base_link".to_string(),
            transform: Transform {
                translation: Vector3 {
                    x: 1.0,
                    y: 2.0,
                    z: 0.0,
                },
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        };
        let bytes = serialize(&ts).unwrap();
        assert_eq!(ts, deserialize::<TransformStamped>(&bytes).unwrap());
    }
}
