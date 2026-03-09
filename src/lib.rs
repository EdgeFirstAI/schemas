// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

#![allow(clippy::too_many_arguments)]

//! # EdgeFirst Middleware Schemas
//!
//! Zero-copy CDR1 Little-Endian message types for ROS 2 / Zenoh middleware.
//!
//! ## Architecture
//!
//! **CdrFixed types** — small, `Copy` structs with constant wire size (e.g.
//! `Time`, `Vector3`). Serialized with `cdr::encode_fixed` / `cdr::decode_fixed`.
//!
//! **Buffer-backed types** — generic `Type<B: AsRef<[u8]>>` wrappers that hold
//! a CDR byte buffer and a small offset table. Construction scans the buffer
//! once; field accessors are O(1). Use `Type::new(...)` to serialize and
//! `Type::from_cdr(buf)` to deserialize (zero-copy when `B = &[u8]`).
//!
//! ## Message sources
//!
//! * [common_interfaces](https://github.com/ros2/common_interfaces)
//! * [rcl_interfaces](https://github.com/ros2/rcl_interfaces)
//! * [foxglove schemas](https://github.com/foxglove/schemas/tree/main/ros_foxglove_msgs)
//! * [edgefirst schemas](https://github.com/EdgeFirstAI/schemas)

/// EdgeFirst custom perception messages.
pub mod edgefirst_msgs;

/// Foxglove visualization messages.
pub mod foxglove_msgs;

/// ROS 2 geometry message types.
pub mod geometry_msgs;
/// ROS 2 sensor message types.
pub mod sensor_msgs;
/// ROS 2 standard message types (Header, ColorRGBA).
pub mod std_msgs;

/// ROS 2 builtin interfaces (Time, Duration).
pub mod builtin_interfaces;
/// ROS 2 rosgraph messages (Clock).
pub mod rosgraph_msgs;

/// ROS 2 service header for Zenoh RPC.
pub mod service;

/// Zero-copy CDR serialization infrastructure.
pub mod cdr;

/// Schema registry for runtime schema name lookup.
pub mod schema_registry;

/// C FFI bindings.
mod ffi;
