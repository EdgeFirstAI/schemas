// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! Schema Registry for EdgeFirst Schemas.
//!
//! This module provides functions to work with ROS2-style schema names.
//!
//! Schema names follow the ROS2 convention: `package/msg/TypeName`
//! (e.g., `sensor_msgs/msg/Image`, `geometry_msgs/msg/Pose`)
//!
//! # Example
//!
//! ```rust
//! use edgefirst_schemas::schema_registry::{SchemaType, is_supported};
//! use edgefirst_schemas::sensor_msgs::Image;
//!
//! // Get schema name from type
//! assert_eq!(Image::SCHEMA_NAME, "sensor_msgs/msg/Image");
//!
//! // Check if schema is supported
//! assert!(is_supported("sensor_msgs/msg/Image"));
//! assert!(!is_supported("unknown_msgs/msg/Foo"));
//! ```

use crate::{
    builtin_interfaces, edgefirst_msgs, foxglove_msgs, geometry_msgs, sensor_msgs, std_msgs,
};
use serde::{de::DeserializeOwned, Serialize};

/// Trait for types that have a schema name.
///
/// All message types implement this trait to provide their ROS2 schema name.
pub trait SchemaType: Serialize + DeserializeOwned {
    /// The ROS2 schema name (e.g., "sensor_msgs/msg/Image")
    const SCHEMA_NAME: &'static str;

    /// Returns the schema name for this type.
    fn schema_name() -> &'static str {
        Self::SCHEMA_NAME
    }
}

/// Parse a schema name into package and type components.
///
/// # Arguments
/// * `schema` - Schema name (e.g., "sensor_msgs/msg/Image")
///
/// # Returns
/// * `Some((package, type_name))` if valid format
/// * `None` if invalid format
///
/// # Example
/// ```rust
/// use edgefirst_schemas::schema_registry::parse_schema;
///
/// let (pkg, typ) = parse_schema("sensor_msgs/msg/Image").unwrap();
/// assert_eq!(pkg, "sensor_msgs");
/// assert_eq!(typ, "Image");
/// ```
pub fn parse_schema(schema: &str) -> Option<(&str, &str)> {
    let parts: Vec<&str> = schema.split('/').collect();
    if parts.len() == 3 && parts[1] == "msg" {
        Some((parts[0], parts[2]))
    } else {
        None
    }
}

/// Check if a schema name is supported by this library.
///
/// Uses hierarchical dispatch to the appropriate package module.
///
/// # Example
///
/// ```rust
/// use edgefirst_schemas::schema_registry::is_supported;
///
/// assert!(is_supported("sensor_msgs/msg/Image"));
/// assert!(!is_supported("unknown_msgs/msg/Foo"));
/// ```
pub fn is_supported(schema: &str) -> bool {
    let Some((package, type_name)) = parse_schema(schema) else {
        return false;
    };

    match package {
        "builtin_interfaces" => builtin_interfaces::is_type_supported(type_name),
        "std_msgs" => std_msgs::is_type_supported(type_name),
        "geometry_msgs" => geometry_msgs::is_type_supported(type_name),
        "sensor_msgs" => sensor_msgs::is_type_supported(type_name),
        "foxglove_msgs" => foxglove_msgs::is_type_supported(type_name),
        "edgefirst_msgs" => edgefirst_msgs::is_type_supported(type_name),
        _ => false,
    }
}

/// List all supported schema names.
///
/// Returns a vector of all schema names that this library supports.
pub fn list_schemas() -> Vec<&'static str> {
    let mut schemas = Vec::new();

    schemas.extend(builtin_interfaces::list_types().iter().copied());
    schemas.extend(std_msgs::list_types().iter().copied());
    schemas.extend(geometry_msgs::list_types().iter().copied());
    schemas.extend(sensor_msgs::list_types().iter().copied());
    schemas.extend(foxglove_msgs::list_types().iter().copied());
    schemas.extend(edgefirst_msgs::list_types().iter().copied());

    schemas
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_schema_valid() {
        let (pkg, typ) = parse_schema("sensor_msgs/msg/Image").unwrap();
        assert_eq!(pkg, "sensor_msgs");
        assert_eq!(typ, "Image");
    }

    #[test]
    fn test_parse_schema_invalid() {
        assert!(parse_schema("invalid").is_none());
        assert!(parse_schema("sensor_msgs/srv/Image").is_none());
        assert!(parse_schema("sensor_msgs/Image").is_none());
    }

    #[test]
    fn test_schema_name_trait() {
        assert_eq!(sensor_msgs::Image::SCHEMA_NAME, "sensor_msgs/msg/Image");
        assert_eq!(geometry_msgs::Pose::SCHEMA_NAME, "geometry_msgs/msg/Pose");
        assert_eq!(edgefirst_msgs::Box::SCHEMA_NAME, "edgefirst_msgs/msg/Box");
    }

    #[test]
    fn test_schema_name_method() {
        assert_eq!(sensor_msgs::Image::schema_name(), "sensor_msgs/msg/Image");
    }

    #[test]
    fn test_is_supported() {
        assert!(is_supported("sensor_msgs/msg/Image"));
        assert!(is_supported("geometry_msgs/msg/Pose"));
        assert!(is_supported("edgefirst_msgs/msg/Box"));
        assert!(is_supported("foxglove_msgs/msg/CompressedVideo"));
        assert!(!is_supported("unknown_msgs/msg/Foo"));
        assert!(!is_supported("sensor_msgs/Image")); // Wrong format
    }

    #[test]
    fn test_list_schemas() {
        let schemas = list_schemas();
        assert!(schemas.contains(&"sensor_msgs/msg/Image"));
        assert!(schemas.contains(&"geometry_msgs/msg/Pose"));
        assert!(schemas.contains(&"edgefirst_msgs/msg/Box"));
        assert!(!schemas.contains(&"unknown_msgs/msg/Foo"));
    }
}
