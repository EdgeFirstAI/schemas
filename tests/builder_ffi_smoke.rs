// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Smoke tests for the `ros_<type>_builder_*` C FFI surface.
//!
//! Each test drives the C FFI through unsafe extern declarations and verifies
//! the bytes written by `..._builder_encode_into` match those produced by the
//! native Rust builder for the same field assignment. This pins the FFI
//! contract: any divergence from the Rust builder is a regression.

#![allow(non_camel_case_types)]

use edgefirst_schemas::builtin_interfaces::{Duration, Time};
use edgefirst_schemas::edgefirst_msgs::{self, Date, DetectBoxView, MaskView};
use edgefirst_schemas::foxglove_msgs::{
    self, FoxgloveCircleAnnotations, FoxgloveColor, FoxglovePoint2, FoxglovePointAnnotationView,
    FoxgloveTextAnnotationView,
};
use edgefirst_schemas::geometry_msgs::{Quaternion, Vector3};
use edgefirst_schemas::sensor_msgs::{self, NavSatStatus, PointFieldView, RegionOfInterest};
use edgefirst_schemas::std_msgs;
use std::ffi::CString;
use std::os::raw::c_char;

// Opaque handle aliases — the real Rust types live in the (private) ffi
// module, but the C FFI signatures only ever expose `*mut <handle>`, so void
// is sufficient for binding.
enum ros_header_builder_t {}
enum ros_image_builder_t {}
enum ros_fluid_pressure_builder_t {}
enum ros_temperature_builder_t {}
enum ros_compressed_image_builder_t {}
enum ros_imu_builder_t {}
enum ros_nav_sat_fix_builder_t {}
enum ros_point_field_builder_t {}
enum ros_point_cloud2_builder_t {}
enum ros_camera_info_builder_t {}
enum ros_magnetic_field_builder_t {}
enum ros_battery_state_builder_t {}
enum ros_mask_builder_t {}
enum ros_local_time_builder_t {}
enum ros_radar_cube_builder_t {}
enum ros_radar_info_builder_t {}
enum ros_track_builder_t {}
enum ros_detect_box_builder_t {}
enum ros_detect_builder_t {}
enum ros_camera_frame_builder_t {}
enum ros_model_builder_t {}
enum ros_model_info_builder_t {}
enum ros_vibration_builder_t {}
enum ros_foxglove_compressed_video_builder_t {}
enum ros_foxglove_text_annotation_builder_t {}
enum ros_foxglove_point_annotation_builder_t {}
enum ros_foxglove_image_annotation_builder_t {}

/// C-POD field descriptor for `ros_point_cloud2_builder_set_fields`.
#[repr(C)]
struct ros_point_field_elem_t {
    name: *const c_char,
    offset: u32,
    datatype: u8,
    count: u32,
}

/// C-POD descriptor for `ros_detect_builder_set_boxes` and
/// `ros_model_builder_set_boxes`.
#[repr(C)]
struct ros_detect_box_elem_t {
    center_x: f32,
    center_y: f32,
    width: f32,
    height: f32,
    label: *const c_char,
    score: f32,
    distance: f32,
    speed: f32,
    track_id: *const c_char,
    track_lifetime: i32,
    track_created_sec: i32,
    track_created_nanosec: u32,
}

/// C-POD descriptor for `ros_camera_frame_builder_set_planes`.
#[repr(C)]
struct ros_camera_plane_elem_t {
    fd: i32,
    offset: u32,
    stride: u32,
    size: u32,
    used: u32,
    data: *const u8,
    data_len: usize,
}

/// C-POD descriptor for `ros_model_builder_set_masks`.
#[repr(C)]
struct ros_mask_elem_t {
    height: u32,
    width: u32,
    length: u32,
    encoding: *const c_char,
    mask: *const u8,
    mask_len: usize,
    boxed: bool,
}

#[repr(C)]
#[derive(Copy, Clone)]
struct ros_foxglove_point2_elem_t {
    x: f64,
    y: f64,
}

#[repr(C)]
#[derive(Copy, Clone)]
struct ros_foxglove_color_elem_t {
    r: f64,
    g: f64,
    b: f64,
    a: f64,
}

#[repr(C)]
#[derive(Copy, Clone)]
struct ros_foxglove_circle_annotation_elem_t {
    timestamp_sec: i32,
    timestamp_nanosec: u32,
    position_x: f64,
    position_y: f64,
    diameter: f64,
    thickness: f64,
    fill_color_r: f64,
    fill_color_g: f64,
    fill_color_b: f64,
    fill_color_a: f64,
    outline_color_r: f64,
    outline_color_g: f64,
    outline_color_b: f64,
    outline_color_a: f64,
}

#[repr(C)]
struct ros_foxglove_point_annotation_elem_t {
    timestamp_sec: i32,
    timestamp_nanosec: u32,
    type_: u8,
    points: *const ros_foxglove_point2_elem_t,
    points_count: usize,
    outline_color_r: f64,
    outline_color_g: f64,
    outline_color_b: f64,
    outline_color_a: f64,
    outline_colors: *const ros_foxglove_color_elem_t,
    outline_colors_count: usize,
    fill_color_r: f64,
    fill_color_g: f64,
    fill_color_b: f64,
    fill_color_a: f64,
    thickness: f64,
}

#[repr(C)]
struct ros_foxglove_text_annotation_elem_t {
    timestamp_sec: i32,
    timestamp_nanosec: u32,
    position_x: f64,
    position_y: f64,
    text: *const c_char,
    font_size: f64,
    text_color_r: f64,
    text_color_g: f64,
    text_color_b: f64,
    text_color_a: f64,
    background_color_r: f64,
    background_color_g: f64,
    background_color_b: f64,
    background_color_a: f64,
}

extern "C" {
    // Header
    fn ros_header_builder_new() -> *mut ros_header_builder_t;
    fn ros_header_builder_free(b: *mut ros_header_builder_t);
    fn ros_header_builder_set_stamp(b: *mut ros_header_builder_t, sec: i32, nsec: u32);
    fn ros_header_builder_set_frame_id(b: *mut ros_header_builder_t, s: *const c_char) -> i32;
    fn ros_header_builder_encode_into(
        b: *mut ros_header_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // Image
    fn ros_image_builder_new() -> *mut ros_image_builder_t;
    fn ros_image_builder_free(b: *mut ros_image_builder_t);
    fn ros_image_builder_set_stamp(b: *mut ros_image_builder_t, sec: i32, nsec: u32);
    fn ros_image_builder_set_frame_id(b: *mut ros_image_builder_t, s: *const c_char) -> i32;
    fn ros_image_builder_set_height(b: *mut ros_image_builder_t, v: u32);
    fn ros_image_builder_set_width(b: *mut ros_image_builder_t, v: u32);
    fn ros_image_builder_set_encoding(b: *mut ros_image_builder_t, s: *const c_char) -> i32;
    fn ros_image_builder_set_is_bigendian(b: *mut ros_image_builder_t, v: u8);
    fn ros_image_builder_set_step(b: *mut ros_image_builder_t, v: u32);
    fn ros_image_builder_set_data(b: *mut ros_image_builder_t, data: *const u8, len: usize);
    fn ros_image_builder_encode_into(
        b: *mut ros_image_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // FluidPressure
    fn ros_fluid_pressure_builder_new() -> *mut ros_fluid_pressure_builder_t;
    fn ros_fluid_pressure_builder_free(b: *mut ros_fluid_pressure_builder_t);
    fn ros_fluid_pressure_builder_set_stamp(
        b: *mut ros_fluid_pressure_builder_t,
        sec: i32,
        nsec: u32,
    );
    fn ros_fluid_pressure_builder_set_frame_id(
        b: *mut ros_fluid_pressure_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_fluid_pressure_builder_set_fluid_pressure(b: *mut ros_fluid_pressure_builder_t, v: f64);
    fn ros_fluid_pressure_builder_set_variance(b: *mut ros_fluid_pressure_builder_t, v: f64);
    fn ros_fluid_pressure_builder_encode_into(
        b: *mut ros_fluid_pressure_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // Temperature
    fn ros_temperature_builder_new() -> *mut ros_temperature_builder_t;
    fn ros_temperature_builder_free(b: *mut ros_temperature_builder_t);
    fn ros_temperature_builder_set_stamp(b: *mut ros_temperature_builder_t, sec: i32, nsec: u32);
    fn ros_temperature_builder_set_frame_id(
        b: *mut ros_temperature_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_temperature_builder_set_temperature(b: *mut ros_temperature_builder_t, v: f64);
    fn ros_temperature_builder_set_variance(b: *mut ros_temperature_builder_t, v: f64);
    fn ros_temperature_builder_encode_into(
        b: *mut ros_temperature_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // CompressedImage
    fn ros_compressed_image_builder_new() -> *mut ros_compressed_image_builder_t;
    fn ros_compressed_image_builder_free(b: *mut ros_compressed_image_builder_t);
    fn ros_compressed_image_builder_set_stamp(
        b: *mut ros_compressed_image_builder_t,
        sec: i32,
        nsec: u32,
    );
    fn ros_compressed_image_builder_set_frame_id(
        b: *mut ros_compressed_image_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_compressed_image_builder_set_format(
        b: *mut ros_compressed_image_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_compressed_image_builder_set_data(
        b: *mut ros_compressed_image_builder_t,
        data: *const u8,
        len: usize,
    );
    fn ros_compressed_image_builder_encode_into(
        b: *mut ros_compressed_image_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // Imu
    fn ros_imu_builder_new() -> *mut ros_imu_builder_t;
    fn ros_imu_builder_free(b: *mut ros_imu_builder_t);
    fn ros_imu_builder_set_stamp(b: *mut ros_imu_builder_t, sec: i32, nsec: u32);
    fn ros_imu_builder_set_frame_id(b: *mut ros_imu_builder_t, s: *const c_char) -> i32;
    fn ros_imu_builder_set_orientation(b: *mut ros_imu_builder_t, x: f64, y: f64, z: f64, w: f64);
    fn ros_imu_builder_set_orientation_covariance(b: *mut ros_imu_builder_t, cov: *const f64);
    fn ros_imu_builder_set_angular_velocity(b: *mut ros_imu_builder_t, x: f64, y: f64, z: f64);
    fn ros_imu_builder_set_angular_velocity_covariance(b: *mut ros_imu_builder_t, cov: *const f64);
    fn ros_imu_builder_set_linear_acceleration(b: *mut ros_imu_builder_t, x: f64, y: f64, z: f64);
    fn ros_imu_builder_set_linear_acceleration_covariance(
        b: *mut ros_imu_builder_t,
        cov: *const f64,
    );
    fn ros_imu_builder_encode_into(
        b: *mut ros_imu_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // NavSatFix
    fn ros_nav_sat_fix_builder_new() -> *mut ros_nav_sat_fix_builder_t;
    fn ros_nav_sat_fix_builder_free(b: *mut ros_nav_sat_fix_builder_t);
    fn ros_nav_sat_fix_builder_set_stamp(b: *mut ros_nav_sat_fix_builder_t, sec: i32, nsec: u32);
    fn ros_nav_sat_fix_builder_set_frame_id(
        b: *mut ros_nav_sat_fix_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_nav_sat_fix_builder_set_status(
        b: *mut ros_nav_sat_fix_builder_t,
        status: i8,
        service: u16,
    );
    fn ros_nav_sat_fix_builder_set_latitude(b: *mut ros_nav_sat_fix_builder_t, v: f64);
    fn ros_nav_sat_fix_builder_set_longitude(b: *mut ros_nav_sat_fix_builder_t, v: f64);
    fn ros_nav_sat_fix_builder_set_altitude(b: *mut ros_nav_sat_fix_builder_t, v: f64);
    fn ros_nav_sat_fix_builder_set_position_covariance(
        b: *mut ros_nav_sat_fix_builder_t,
        cov: *const f64,
    );
    fn ros_nav_sat_fix_builder_set_position_covariance_type(
        b: *mut ros_nav_sat_fix_builder_t,
        v: u8,
    );
    fn ros_nav_sat_fix_builder_encode_into(
        b: *mut ros_nav_sat_fix_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // PointField
    fn ros_point_field_builder_new() -> *mut ros_point_field_builder_t;
    fn ros_point_field_builder_free(b: *mut ros_point_field_builder_t);
    fn ros_point_field_builder_set_name(b: *mut ros_point_field_builder_t, s: *const c_char)
        -> i32;
    fn ros_point_field_builder_set_offset(b: *mut ros_point_field_builder_t, v: u32);
    fn ros_point_field_builder_set_datatype(b: *mut ros_point_field_builder_t, v: u8);
    fn ros_point_field_builder_set_count(b: *mut ros_point_field_builder_t, v: u32);
    fn ros_point_field_builder_encode_into(
        b: *mut ros_point_field_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // PointCloud2
    fn ros_point_cloud2_builder_new() -> *mut ros_point_cloud2_builder_t;
    fn ros_point_cloud2_builder_free(b: *mut ros_point_cloud2_builder_t);
    fn ros_point_cloud2_builder_set_stamp(b: *mut ros_point_cloud2_builder_t, sec: i32, nsec: u32);
    fn ros_point_cloud2_builder_set_frame_id(
        b: *mut ros_point_cloud2_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_point_cloud2_builder_set_height(b: *mut ros_point_cloud2_builder_t, v: u32);
    fn ros_point_cloud2_builder_set_width(b: *mut ros_point_cloud2_builder_t, v: u32);
    fn ros_point_cloud2_builder_set_fields(
        b: *mut ros_point_cloud2_builder_t,
        fields: *const ros_point_field_elem_t,
        count: usize,
    );
    fn ros_point_cloud2_builder_set_is_bigendian(b: *mut ros_point_cloud2_builder_t, v: bool);
    fn ros_point_cloud2_builder_set_point_step(b: *mut ros_point_cloud2_builder_t, v: u32);
    fn ros_point_cloud2_builder_set_row_step(b: *mut ros_point_cloud2_builder_t, v: u32);
    fn ros_point_cloud2_builder_set_data(
        b: *mut ros_point_cloud2_builder_t,
        data: *const u8,
        len: usize,
    );
    fn ros_point_cloud2_builder_set_is_dense(b: *mut ros_point_cloud2_builder_t, v: bool);
    fn ros_point_cloud2_builder_encode_into(
        b: *mut ros_point_cloud2_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // CameraInfo
    fn ros_camera_info_builder_new() -> *mut ros_camera_info_builder_t;
    fn ros_camera_info_builder_free(b: *mut ros_camera_info_builder_t);
    fn ros_camera_info_builder_set_stamp(b: *mut ros_camera_info_builder_t, sec: i32, nsec: u32);
    fn ros_camera_info_builder_set_frame_id(
        b: *mut ros_camera_info_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_camera_info_builder_set_height(b: *mut ros_camera_info_builder_t, v: u32);
    fn ros_camera_info_builder_set_width(b: *mut ros_camera_info_builder_t, v: u32);
    fn ros_camera_info_builder_set_distortion_model(
        b: *mut ros_camera_info_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_camera_info_builder_set_d(
        b: *mut ros_camera_info_builder_t,
        data: *const f64,
        len: usize,
    );
    fn ros_camera_info_builder_set_k(b: *mut ros_camera_info_builder_t, k: *const f64);
    fn ros_camera_info_builder_set_r(b: *mut ros_camera_info_builder_t, r: *const f64);
    fn ros_camera_info_builder_set_p(b: *mut ros_camera_info_builder_t, p: *const f64);
    fn ros_camera_info_builder_set_binning_x(b: *mut ros_camera_info_builder_t, v: u32);
    fn ros_camera_info_builder_set_binning_y(b: *mut ros_camera_info_builder_t, v: u32);
    fn ros_camera_info_builder_set_roi(
        b: *mut ros_camera_info_builder_t,
        x_offset: u32,
        y_offset: u32,
        height: u32,
        width: u32,
        do_rectify: u8,
    );
    fn ros_camera_info_builder_encode_into(
        b: *mut ros_camera_info_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // MagneticField
    fn ros_magnetic_field_builder_new() -> *mut ros_magnetic_field_builder_t;
    fn ros_magnetic_field_builder_free(b: *mut ros_magnetic_field_builder_t);
    fn ros_magnetic_field_builder_set_stamp(
        b: *mut ros_magnetic_field_builder_t,
        sec: i32,
        nsec: u32,
    );
    fn ros_magnetic_field_builder_set_frame_id(
        b: *mut ros_magnetic_field_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_magnetic_field_builder_set_magnetic_field(
        b: *mut ros_magnetic_field_builder_t,
        x: f64,
        y: f64,
        z: f64,
    );
    fn ros_magnetic_field_builder_set_magnetic_field_covariance(
        b: *mut ros_magnetic_field_builder_t,
        cov: *const f64,
    );
    fn ros_magnetic_field_builder_encode_into(
        b: *mut ros_magnetic_field_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // BatteryState
    fn ros_battery_state_builder_new() -> *mut ros_battery_state_builder_t;
    fn ros_battery_state_builder_free(b: *mut ros_battery_state_builder_t);
    fn ros_battery_state_builder_set_stamp(
        b: *mut ros_battery_state_builder_t,
        sec: i32,
        nsec: u32,
    );
    fn ros_battery_state_builder_set_frame_id(
        b: *mut ros_battery_state_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_battery_state_builder_set_voltage(b: *mut ros_battery_state_builder_t, v: f32);
    fn ros_battery_state_builder_set_temperature(b: *mut ros_battery_state_builder_t, v: f32);
    fn ros_battery_state_builder_set_current(b: *mut ros_battery_state_builder_t, v: f32);
    fn ros_battery_state_builder_set_charge(b: *mut ros_battery_state_builder_t, v: f32);
    fn ros_battery_state_builder_set_capacity(b: *mut ros_battery_state_builder_t, v: f32);
    fn ros_battery_state_builder_set_design_capacity(b: *mut ros_battery_state_builder_t, v: f32);
    fn ros_battery_state_builder_set_percentage(b: *mut ros_battery_state_builder_t, v: f32);
    fn ros_battery_state_builder_set_power_supply_status(
        b: *mut ros_battery_state_builder_t,
        v: u8,
    );
    fn ros_battery_state_builder_set_power_supply_health(
        b: *mut ros_battery_state_builder_t,
        v: u8,
    );
    fn ros_battery_state_builder_set_power_supply_technology(
        b: *mut ros_battery_state_builder_t,
        v: u8,
    );
    fn ros_battery_state_builder_set_present(b: *mut ros_battery_state_builder_t, v: bool);
    fn ros_battery_state_builder_set_cell_voltage(
        b: *mut ros_battery_state_builder_t,
        data: *const f32,
        len: usize,
    );
    fn ros_battery_state_builder_set_cell_temperature(
        b: *mut ros_battery_state_builder_t,
        data: *const f32,
        len: usize,
    );
    fn ros_battery_state_builder_set_location(
        b: *mut ros_battery_state_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_battery_state_builder_set_serial_number(
        b: *mut ros_battery_state_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_battery_state_builder_encode_into(
        b: *mut ros_battery_state_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;
}

extern "C" {
    // Mask
    fn ros_mask_builder_new() -> *mut ros_mask_builder_t;
    fn ros_mask_builder_free(b: *mut ros_mask_builder_t);
    fn ros_mask_builder_set_height(b: *mut ros_mask_builder_t, v: u32);
    fn ros_mask_builder_set_width(b: *mut ros_mask_builder_t, v: u32);
    fn ros_mask_builder_set_length(b: *mut ros_mask_builder_t, v: u32);
    fn ros_mask_builder_set_encoding(b: *mut ros_mask_builder_t, s: *const c_char) -> i32;
    fn ros_mask_builder_set_mask(b: *mut ros_mask_builder_t, data: *const u8, len: usize);
    fn ros_mask_builder_set_boxed(b: *mut ros_mask_builder_t, v: bool);
    fn ros_mask_builder_encode_into(
        b: *mut ros_mask_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // LocalTime
    fn ros_local_time_builder_new() -> *mut ros_local_time_builder_t;
    fn ros_local_time_builder_free(b: *mut ros_local_time_builder_t);
    fn ros_local_time_builder_set_stamp(b: *mut ros_local_time_builder_t, sec: i32, nsec: u32);
    fn ros_local_time_builder_set_frame_id(
        b: *mut ros_local_time_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_local_time_builder_set_date(
        b: *mut ros_local_time_builder_t,
        year: u16,
        month: u8,
        day: u8,
    );
    fn ros_local_time_builder_set_time(b: *mut ros_local_time_builder_t, sec: i32, nsec: u32);
    fn ros_local_time_builder_set_timezone(b: *mut ros_local_time_builder_t, v: i16);
    fn ros_local_time_builder_encode_into(
        b: *mut ros_local_time_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // RadarCube
    fn ros_radar_cube_builder_new() -> *mut ros_radar_cube_builder_t;
    fn ros_radar_cube_builder_free(b: *mut ros_radar_cube_builder_t);
    fn ros_radar_cube_builder_set_stamp(b: *mut ros_radar_cube_builder_t, sec: i32, nsec: u32);
    fn ros_radar_cube_builder_set_frame_id(
        b: *mut ros_radar_cube_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_radar_cube_builder_set_timestamp(b: *mut ros_radar_cube_builder_t, v: u64);
    fn ros_radar_cube_builder_set_layout(
        b: *mut ros_radar_cube_builder_t,
        data: *const u8,
        len: usize,
    );
    fn ros_radar_cube_builder_set_shape(
        b: *mut ros_radar_cube_builder_t,
        data: *const u16,
        len: usize,
    );
    fn ros_radar_cube_builder_set_scales(
        b: *mut ros_radar_cube_builder_t,
        data: *const f32,
        len: usize,
    );
    fn ros_radar_cube_builder_set_cube(
        b: *mut ros_radar_cube_builder_t,
        data: *const i16,
        len: usize,
    );
    fn ros_radar_cube_builder_set_is_complex(b: *mut ros_radar_cube_builder_t, v: bool);
    fn ros_radar_cube_builder_encode_into(
        b: *mut ros_radar_cube_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // RadarInfo
    fn ros_radar_info_builder_new() -> *mut ros_radar_info_builder_t;
    fn ros_radar_info_builder_free(b: *mut ros_radar_info_builder_t);
    fn ros_radar_info_builder_set_stamp(b: *mut ros_radar_info_builder_t, sec: i32, nsec: u32);
    fn ros_radar_info_builder_set_frame_id(
        b: *mut ros_radar_info_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_radar_info_builder_set_center_frequency(
        b: *mut ros_radar_info_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_radar_info_builder_set_frequency_sweep(
        b: *mut ros_radar_info_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_radar_info_builder_set_range_toggle(
        b: *mut ros_radar_info_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_radar_info_builder_set_detection_sensitivity(
        b: *mut ros_radar_info_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_radar_info_builder_set_cube(b: *mut ros_radar_info_builder_t, v: bool);
    fn ros_radar_info_builder_encode_into(
        b: *mut ros_radar_info_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // Track
    fn ros_track_builder_new() -> *mut ros_track_builder_t;
    fn ros_track_builder_free(b: *mut ros_track_builder_t);
    fn ros_track_builder_set_id(b: *mut ros_track_builder_t, s: *const c_char) -> i32;
    fn ros_track_builder_set_lifetime(b: *mut ros_track_builder_t, v: i32);
    fn ros_track_builder_set_created(b: *mut ros_track_builder_t, sec: i32, nsec: u32);
    fn ros_track_builder_encode_into(
        b: *mut ros_track_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // DetectBox
    fn ros_detect_box_builder_new() -> *mut ros_detect_box_builder_t;
    fn ros_detect_box_builder_free(b: *mut ros_detect_box_builder_t);
    fn ros_detect_box_builder_set_center_x(b: *mut ros_detect_box_builder_t, v: f32);
    fn ros_detect_box_builder_set_center_y(b: *mut ros_detect_box_builder_t, v: f32);
    fn ros_detect_box_builder_set_width(b: *mut ros_detect_box_builder_t, v: f32);
    fn ros_detect_box_builder_set_height(b: *mut ros_detect_box_builder_t, v: f32);
    fn ros_detect_box_builder_set_label(b: *mut ros_detect_box_builder_t, s: *const c_char) -> i32;
    fn ros_detect_box_builder_set_score(b: *mut ros_detect_box_builder_t, v: f32);
    fn ros_detect_box_builder_set_distance(b: *mut ros_detect_box_builder_t, v: f32);
    fn ros_detect_box_builder_set_speed(b: *mut ros_detect_box_builder_t, v: f32);
    fn ros_detect_box_builder_set_track_id(
        b: *mut ros_detect_box_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_detect_box_builder_set_track_lifetime(b: *mut ros_detect_box_builder_t, v: i32);
    fn ros_detect_box_builder_set_track_created(
        b: *mut ros_detect_box_builder_t,
        sec: i32,
        nsec: u32,
    );
    fn ros_detect_box_builder_encode_into(
        b: *mut ros_detect_box_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // Detect
    fn ros_detect_builder_new() -> *mut ros_detect_builder_t;
    fn ros_detect_builder_free(b: *mut ros_detect_builder_t);
    fn ros_detect_builder_set_stamp(b: *mut ros_detect_builder_t, sec: i32, nsec: u32);
    fn ros_detect_builder_set_frame_id(b: *mut ros_detect_builder_t, s: *const c_char) -> i32;
    fn ros_detect_builder_set_input_timestamp(b: *mut ros_detect_builder_t, sec: i32, nsec: u32);
    fn ros_detect_builder_set_model_time(b: *mut ros_detect_builder_t, sec: i32, nsec: u32);
    fn ros_detect_builder_set_output_time(b: *mut ros_detect_builder_t, sec: i32, nsec: u32);
    fn ros_detect_builder_set_boxes(
        b: *mut ros_detect_builder_t,
        boxes: *const ros_detect_box_elem_t,
        count: usize,
    );
    fn ros_detect_builder_encode_into(
        b: *mut ros_detect_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // CameraFrame
    fn ros_camera_frame_builder_new() -> *mut ros_camera_frame_builder_t;
    fn ros_camera_frame_builder_free(b: *mut ros_camera_frame_builder_t);
    fn ros_camera_frame_builder_set_stamp(b: *mut ros_camera_frame_builder_t, sec: i32, nsec: u32);
    fn ros_camera_frame_builder_set_frame_id(
        b: *mut ros_camera_frame_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_camera_frame_builder_set_seq(b: *mut ros_camera_frame_builder_t, v: u64);
    fn ros_camera_frame_builder_set_pid(b: *mut ros_camera_frame_builder_t, v: u32);
    fn ros_camera_frame_builder_set_width(b: *mut ros_camera_frame_builder_t, v: u32);
    fn ros_camera_frame_builder_set_height(b: *mut ros_camera_frame_builder_t, v: u32);
    fn ros_camera_frame_builder_set_format(
        b: *mut ros_camera_frame_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_camera_frame_builder_set_color_space(
        b: *mut ros_camera_frame_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_camera_frame_builder_set_color_transfer(
        b: *mut ros_camera_frame_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_camera_frame_builder_set_color_encoding(
        b: *mut ros_camera_frame_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_camera_frame_builder_set_color_range(
        b: *mut ros_camera_frame_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_camera_frame_builder_set_fence_fd(b: *mut ros_camera_frame_builder_t, v: i32);
    fn ros_camera_frame_builder_set_planes(
        b: *mut ros_camera_frame_builder_t,
        planes: *const ros_camera_plane_elem_t,
        count: usize,
    );
    fn ros_camera_frame_builder_encode_into(
        b: *mut ros_camera_frame_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // Model
    fn ros_model_builder_new() -> *mut ros_model_builder_t;
    fn ros_model_builder_free(b: *mut ros_model_builder_t);
    fn ros_model_builder_set_stamp(b: *mut ros_model_builder_t, sec: i32, nsec: u32);
    fn ros_model_builder_set_frame_id(b: *mut ros_model_builder_t, s: *const c_char) -> i32;
    fn ros_model_builder_set_input_time(b: *mut ros_model_builder_t, sec: i32, nsec: u32);
    fn ros_model_builder_set_model_time(b: *mut ros_model_builder_t, sec: i32, nsec: u32);
    fn ros_model_builder_set_output_time(b: *mut ros_model_builder_t, sec: i32, nsec: u32);
    fn ros_model_builder_set_decode_time(b: *mut ros_model_builder_t, sec: i32, nsec: u32);
    fn ros_model_builder_set_boxes(
        b: *mut ros_model_builder_t,
        boxes: *const ros_detect_box_elem_t,
        count: usize,
    );
    fn ros_model_builder_set_masks(
        b: *mut ros_model_builder_t,
        masks: *const ros_mask_elem_t,
        count: usize,
    );
    fn ros_model_builder_encode_into(
        b: *mut ros_model_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // ModelInfo
    fn ros_model_info_builder_new() -> *mut ros_model_info_builder_t;
    fn ros_model_info_builder_free(b: *mut ros_model_info_builder_t);
    fn ros_model_info_builder_set_stamp(b: *mut ros_model_info_builder_t, sec: i32, nsec: u32);
    fn ros_model_info_builder_set_frame_id(
        b: *mut ros_model_info_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_model_info_builder_set_input_shape(
        b: *mut ros_model_info_builder_t,
        data: *const u32,
        len: usize,
    );
    fn ros_model_info_builder_set_input_type(b: *mut ros_model_info_builder_t, v: u8);
    fn ros_model_info_builder_set_output_shape(
        b: *mut ros_model_info_builder_t,
        data: *const u32,
        len: usize,
    );
    fn ros_model_info_builder_set_output_type(b: *mut ros_model_info_builder_t, v: u8);
    fn ros_model_info_builder_set_labels(
        b: *mut ros_model_info_builder_t,
        labels: *const *const c_char,
        count: usize,
    ) -> i32;
    fn ros_model_info_builder_set_model_type(
        b: *mut ros_model_info_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_model_info_builder_set_model_format(
        b: *mut ros_model_info_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_model_info_builder_set_model_name(
        b: *mut ros_model_info_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_model_info_builder_encode_into(
        b: *mut ros_model_info_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // Vibration
    fn ros_vibration_builder_new() -> *mut ros_vibration_builder_t;
    fn ros_vibration_builder_free(b: *mut ros_vibration_builder_t);
    fn ros_vibration_builder_set_stamp(b: *mut ros_vibration_builder_t, sec: i32, nsec: u32);
    fn ros_vibration_builder_set_frame_id(b: *mut ros_vibration_builder_t, s: *const c_char)
        -> i32;
    fn ros_vibration_builder_set_vibration(b: *mut ros_vibration_builder_t, x: f64, y: f64, z: f64);
    fn ros_vibration_builder_set_band_lower_hz(b: *mut ros_vibration_builder_t, v: f32);
    fn ros_vibration_builder_set_band_upper_hz(b: *mut ros_vibration_builder_t, v: f32);
    fn ros_vibration_builder_set_measurement_type(b: *mut ros_vibration_builder_t, v: u8);
    fn ros_vibration_builder_set_unit(b: *mut ros_vibration_builder_t, v: u8);
    fn ros_vibration_builder_set_clipping(
        b: *mut ros_vibration_builder_t,
        data: *const u32,
        len: usize,
    );
    fn ros_vibration_builder_encode_into(
        b: *mut ros_vibration_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // FoxgloveCompressedVideo
    fn ros_foxglove_compressed_video_builder_new() -> *mut ros_foxglove_compressed_video_builder_t;
    fn ros_foxglove_compressed_video_builder_free(b: *mut ros_foxglove_compressed_video_builder_t);
    fn ros_foxglove_compressed_video_builder_set_stamp(
        b: *mut ros_foxglove_compressed_video_builder_t,
        sec: i32,
        nsec: u32,
    );
    fn ros_foxglove_compressed_video_builder_set_frame_id(
        b: *mut ros_foxglove_compressed_video_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_foxglove_compressed_video_builder_set_data(
        b: *mut ros_foxglove_compressed_video_builder_t,
        data: *const u8,
        len: usize,
    );
    fn ros_foxglove_compressed_video_builder_set_format(
        b: *mut ros_foxglove_compressed_video_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_foxglove_compressed_video_builder_encode_into(
        b: *mut ros_foxglove_compressed_video_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // FoxgloveTextAnnotation
    fn ros_foxglove_text_annotation_builder_new() -> *mut ros_foxglove_text_annotation_builder_t;
    fn ros_foxglove_text_annotation_builder_free(b: *mut ros_foxglove_text_annotation_builder_t);
    fn ros_foxglove_text_annotation_builder_set_timestamp(
        b: *mut ros_foxglove_text_annotation_builder_t,
        sec: i32,
        nsec: u32,
    );
    fn ros_foxglove_text_annotation_builder_set_position(
        b: *mut ros_foxglove_text_annotation_builder_t,
        x: f64,
        y: f64,
    );
    fn ros_foxglove_text_annotation_builder_set_text(
        b: *mut ros_foxglove_text_annotation_builder_t,
        s: *const c_char,
    ) -> i32;
    fn ros_foxglove_text_annotation_builder_set_font_size(
        b: *mut ros_foxglove_text_annotation_builder_t,
        v: f64,
    );
    fn ros_foxglove_text_annotation_builder_set_text_color(
        b: *mut ros_foxglove_text_annotation_builder_t,
        r: f64,
        g: f64,
        bc: f64,
        a: f64,
    );
    fn ros_foxglove_text_annotation_builder_set_background_color(
        b: *mut ros_foxglove_text_annotation_builder_t,
        r: f64,
        g: f64,
        bc: f64,
        a: f64,
    );
    fn ros_foxglove_text_annotation_builder_encode_into(
        b: *mut ros_foxglove_text_annotation_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // FoxglovePointAnnotation
    fn ros_foxglove_point_annotation_builder_new() -> *mut ros_foxglove_point_annotation_builder_t;
    fn ros_foxglove_point_annotation_builder_free(b: *mut ros_foxglove_point_annotation_builder_t);
    fn ros_foxglove_point_annotation_builder_set_timestamp(
        b: *mut ros_foxglove_point_annotation_builder_t,
        sec: i32,
        nsec: u32,
    );
    fn ros_foxglove_point_annotation_builder_set_type(
        b: *mut ros_foxglove_point_annotation_builder_t,
        v: u8,
    );
    fn ros_foxglove_point_annotation_builder_set_points(
        b: *mut ros_foxglove_point_annotation_builder_t,
        points: *const ros_foxglove_point2_elem_t,
        count: usize,
    );
    fn ros_foxglove_point_annotation_builder_set_outline_color(
        b: *mut ros_foxglove_point_annotation_builder_t,
        r: f64,
        g: f64,
        bc: f64,
        a: f64,
    );
    fn ros_foxglove_point_annotation_builder_set_outline_colors(
        b: *mut ros_foxglove_point_annotation_builder_t,
        colors: *const ros_foxglove_color_elem_t,
        count: usize,
    );
    fn ros_foxglove_point_annotation_builder_set_fill_color(
        b: *mut ros_foxglove_point_annotation_builder_t,
        r: f64,
        g: f64,
        bc: f64,
        a: f64,
    );
    fn ros_foxglove_point_annotation_builder_set_thickness(
        b: *mut ros_foxglove_point_annotation_builder_t,
        v: f64,
    );
    fn ros_foxglove_point_annotation_builder_encode_into(
        b: *mut ros_foxglove_point_annotation_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;

    // FoxgloveImageAnnotation
    fn ros_foxglove_image_annotation_builder_new() -> *mut ros_foxglove_image_annotation_builder_t;
    fn ros_foxglove_image_annotation_builder_free(b: *mut ros_foxglove_image_annotation_builder_t);
    fn ros_foxglove_image_annotation_builder_set_circles(
        b: *mut ros_foxglove_image_annotation_builder_t,
        circles: *const ros_foxglove_circle_annotation_elem_t,
        count: usize,
    );
    fn ros_foxglove_image_annotation_builder_set_points(
        b: *mut ros_foxglove_image_annotation_builder_t,
        points: *const ros_foxglove_point_annotation_elem_t,
        count: usize,
    );
    fn ros_foxglove_image_annotation_builder_set_texts(
        b: *mut ros_foxglove_image_annotation_builder_t,
        texts: *const ros_foxglove_text_annotation_elem_t,
        count: usize,
    );
    fn ros_foxglove_image_annotation_builder_encode_into(
        b: *mut ros_foxglove_image_annotation_builder_t,
        buf: *mut u8,
        cap: usize,
        out_len: *mut usize,
    ) -> i32;
}

#[test]
fn ros_header_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_header_builder_new();
        assert!(!b.is_null(), "ros_header_builder_new returned NULL");
        ros_header_builder_set_stamp(b, 42, 7);
        let frame = CString::new("base_link").unwrap();
        assert_eq!(ros_header_builder_set_frame_id(b, frame.as_ptr()), 0);

        let mut buf = [0u8; 256];
        let mut out_len: usize = 0;
        let rc = ros_header_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0, "encode_into returned non-zero");
        assert!(out_len > 0);

        let via_rust = std_msgs::Header::builder()
            .stamp(Time::new(42, 7))
            .frame_id("base_link")
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_header_builder_free(b);
        // free should be NULL-safe
        ros_header_builder_free(std::ptr::null_mut());
    }
}

#[test]
fn ros_image_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_image_builder_new();
        assert!(!b.is_null());
        ros_image_builder_set_stamp(b, 1234, 567);
        let frame = CString::new("camera").unwrap();
        assert_eq!(ros_image_builder_set_frame_id(b, frame.as_ptr()), 0);
        ros_image_builder_set_height(b, 4);
        ros_image_builder_set_width(b, 3);
        let enc = CString::new("rgb8").unwrap();
        assert_eq!(ros_image_builder_set_encoding(b, enc.as_ptr()), 0);
        ros_image_builder_set_is_bigendian(b, 0);
        ros_image_builder_set_step(b, 9);
        let pixels: Vec<u8> = (0..36u8).collect();
        ros_image_builder_set_data(b, pixels.as_ptr(), pixels.len());

        let mut buf = [0u8; 1024];
        let mut out_len: usize = 0;
        let rc = ros_image_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = sensor_msgs::Image::builder()
            .stamp(Time::new(1234, 567))
            .frame_id("camera")
            .height(4)
            .width(3)
            .encoding("rgb8")
            .is_bigendian(0)
            .step(9)
            .data(&pixels)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        // Re-emit with empty data via NULL pointer + 0 len — the FFI must
        // treat it as an empty slice, matching `.data(&[])`.
        ros_image_builder_set_data(b, std::ptr::null(), 0);
        let mut buf2 = [0u8; 256];
        let mut out_len2: usize = 0;
        let rc2 = ros_image_builder_encode_into(b, buf2.as_mut_ptr(), buf2.len(), &mut out_len2);
        assert_eq!(rc2, 0);
        let via_rust_empty = sensor_msgs::Image::builder()
            .stamp(Time::new(1234, 567))
            .frame_id("camera")
            .height(4)
            .width(3)
            .encoding("rgb8")
            .is_bigendian(0)
            .step(9)
            .data(&[])
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf2[..out_len2], via_rust_empty.as_cdr());

        ros_image_builder_free(b);
    }
}

#[test]
fn ros_fluid_pressure_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_fluid_pressure_builder_new();
        assert!(!b.is_null());
        ros_fluid_pressure_builder_set_stamp(b, 11, 22);
        let frame = CString::new("baro_link").unwrap();
        assert_eq!(
            ros_fluid_pressure_builder_set_frame_id(b, frame.as_ptr()),
            0
        );
        ros_fluid_pressure_builder_set_fluid_pressure(b, 101_325.5);
        ros_fluid_pressure_builder_set_variance(b, 0.25);

        let mut buf = [0u8; 128];
        let mut out_len: usize = 0;
        let rc =
            ros_fluid_pressure_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = sensor_msgs::FluidPressure::builder()
            .stamp(Time::new(11, 22))
            .frame_id("baro_link")
            .fluid_pressure(101_325.5)
            .variance(0.25)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_fluid_pressure_builder_free(b);
    }
}

#[test]
fn ros_temperature_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_temperature_builder_new();
        assert!(!b.is_null());
        ros_temperature_builder_set_stamp(b, 99, 100);
        let frame = CString::new("thermo_link").unwrap();
        assert_eq!(ros_temperature_builder_set_frame_id(b, frame.as_ptr()), 0);
        ros_temperature_builder_set_temperature(b, 23.5);
        ros_temperature_builder_set_variance(b, 0.01);

        let mut buf = [0u8; 128];
        let mut out_len: usize = 0;
        let rc = ros_temperature_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = sensor_msgs::Temperature::builder()
            .stamp(Time::new(99, 100))
            .frame_id("thermo_link")
            .temperature(23.5)
            .variance(0.01)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_temperature_builder_free(b);
    }
}

#[test]
fn ros_compressed_image_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_compressed_image_builder_new();
        assert!(!b.is_null());
        ros_compressed_image_builder_set_stamp(b, 7, 8);
        let frame = CString::new("cam_color").unwrap();
        assert_eq!(
            ros_compressed_image_builder_set_frame_id(b, frame.as_ptr()),
            0
        );
        let fmt = CString::new("jpeg").unwrap();
        assert_eq!(ros_compressed_image_builder_set_format(b, fmt.as_ptr()), 0);
        let jpeg: Vec<u8> = (0..64u8).collect();
        ros_compressed_image_builder_set_data(b, jpeg.as_ptr(), jpeg.len());

        let mut buf = [0u8; 1024];
        let mut out_len: usize = 0;
        let rc =
            ros_compressed_image_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = sensor_msgs::CompressedImage::builder()
            .stamp(Time::new(7, 8))
            .frame_id("cam_color")
            .format("jpeg")
            .data(&jpeg)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_compressed_image_builder_free(b);
    }
}

#[test]
fn ros_imu_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_imu_builder_new();
        assert!(!b.is_null());
        ros_imu_builder_set_stamp(b, 100, 200);
        let frame = CString::new("imu_link").unwrap();
        assert_eq!(ros_imu_builder_set_frame_id(b, frame.as_ptr()), 0);
        ros_imu_builder_set_orientation(b, 0.1, 0.2, 0.3, 0.9);
        let ori_cov: [f64; 9] = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 2e-3];
        ros_imu_builder_set_orientation_covariance(b, ori_cov.as_ptr());
        ros_imu_builder_set_angular_velocity(b, 0.01, -0.02, 0.03);
        let av_cov: [f64; 9] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
        ros_imu_builder_set_angular_velocity_covariance(b, av_cov.as_ptr());
        ros_imu_builder_set_linear_acceleration(b, 0.0, 0.0, 9.81);
        let la_cov: [f64; 9] = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.2];
        ros_imu_builder_set_linear_acceleration_covariance(b, la_cov.as_ptr());

        let mut buf = [0u8; 512];
        let mut out_len: usize = 0;
        let rc = ros_imu_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = sensor_msgs::Imu::builder()
            .stamp(Time::new(100, 200))
            .frame_id("imu_link")
            .orientation(Quaternion {
                x: 0.1,
                y: 0.2,
                z: 0.3,
                w: 0.9,
            })
            .orientation_covariance(ori_cov)
            .angular_velocity(Vector3 {
                x: 0.01,
                y: -0.02,
                z: 0.03,
            })
            .angular_velocity_covariance(av_cov)
            .linear_acceleration(Vector3 {
                x: 0.0,
                y: 0.0,
                z: 9.81,
            })
            .linear_acceleration_covariance(la_cov)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_imu_builder_free(b);
    }
}

#[test]
fn ros_nav_sat_fix_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_nav_sat_fix_builder_new();
        assert!(!b.is_null());
        ros_nav_sat_fix_builder_set_stamp(b, 10, 20);
        let frame = CString::new("gps").unwrap();
        assert_eq!(ros_nav_sat_fix_builder_set_frame_id(b, frame.as_ptr()), 0);
        // STATUS_FIX = 0, SERVICE_GPS = 1
        ros_nav_sat_fix_builder_set_status(b, 0, 1);
        ros_nav_sat_fix_builder_set_latitude(b, 45.4215);
        ros_nav_sat_fix_builder_set_longitude(b, -75.6972);
        ros_nav_sat_fix_builder_set_altitude(b, 70.0);
        let pos_cov: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 4.0];
        ros_nav_sat_fix_builder_set_position_covariance(b, pos_cov.as_ptr());
        // COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
        ros_nav_sat_fix_builder_set_position_covariance_type(b, 2);

        let mut buf = [0u8; 512];
        let mut out_len: usize = 0;
        let rc = ros_nav_sat_fix_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = sensor_msgs::NavSatFix::builder()
            .stamp(Time::new(10, 20))
            .frame_id("gps")
            .status(NavSatStatus {
                status: 0,
                service: 1,
            })
            .latitude(45.4215)
            .longitude(-75.6972)
            .altitude(70.0)
            .position_covariance(pos_cov)
            .position_covariance_type(2)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_nav_sat_fix_builder_free(b);
    }
}

#[test]
fn ros_point_field_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_point_field_builder_new();
        assert!(!b.is_null());
        let name = CString::new("intensity").unwrap();
        assert_eq!(ros_point_field_builder_set_name(b, name.as_ptr()), 0);
        ros_point_field_builder_set_offset(b, 12);
        ros_point_field_builder_set_datatype(b, 7); // FLOAT32
        ros_point_field_builder_set_count(b, 1);

        let mut buf = [0u8; 128];
        let mut out_len: usize = 0;
        let rc = ros_point_field_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = sensor_msgs::PointField::builder()
            .name("intensity")
            .offset(12)
            .datatype(7)
            .count(1)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_point_field_builder_free(b);
    }
}

#[test]
fn ros_point_cloud2_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_point_cloud2_builder_new();
        assert!(!b.is_null());
        ros_point_cloud2_builder_set_stamp(b, 500, 0);
        let frame = CString::new("lidar").unwrap();
        assert_eq!(ros_point_cloud2_builder_set_frame_id(b, frame.as_ptr()), 0);
        ros_point_cloud2_builder_set_height(b, 1);
        ros_point_cloud2_builder_set_width(b, 2);

        // Construct 3 field descriptors: x (f32@0), y (f32@4), z (f32@8).
        let n_x = CString::new("x").unwrap();
        let n_y = CString::new("y").unwrap();
        let n_z = CString::new("z").unwrap();
        let descs = [
            ros_point_field_elem_t {
                name: n_x.as_ptr(),
                offset: 0,
                datatype: 7,
                count: 1,
            },
            ros_point_field_elem_t {
                name: n_y.as_ptr(),
                offset: 4,
                datatype: 7,
                count: 1,
            },
            ros_point_field_elem_t {
                name: n_z.as_ptr(),
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        ros_point_cloud2_builder_set_fields(b, descs.as_ptr(), descs.len());

        ros_point_cloud2_builder_set_is_bigendian(b, false);
        ros_point_cloud2_builder_set_point_step(b, 12);
        ros_point_cloud2_builder_set_row_step(b, 24);
        let cloud: Vec<u8> = (0..24u8).collect();
        ros_point_cloud2_builder_set_data(b, cloud.as_ptr(), cloud.len());
        ros_point_cloud2_builder_set_is_dense(b, true);

        let mut buf = [0u8; 1024];
        let mut out_len: usize = 0;
        let rc = ros_point_cloud2_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let fields_rust = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        let via_rust = sensor_msgs::PointCloud2::builder()
            .stamp(Time::new(500, 0))
            .frame_id("lidar")
            .height(1)
            .width(2)
            .fields(&fields_rust)
            .is_bigendian(false)
            .point_step(12)
            .row_step(24)
            .data(&cloud)
            .is_dense(true)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_point_cloud2_builder_free(b);
    }
}

#[test]
fn ros_camera_info_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_camera_info_builder_new();
        assert!(!b.is_null());
        ros_camera_info_builder_set_stamp(b, 1, 2);
        let frame = CString::new("cam_info").unwrap();
        assert_eq!(ros_camera_info_builder_set_frame_id(b, frame.as_ptr()), 0);
        ros_camera_info_builder_set_height(b, 480);
        ros_camera_info_builder_set_width(b, 640);
        let model = CString::new("plumb_bob").unwrap();
        assert_eq!(
            ros_camera_info_builder_set_distortion_model(b, model.as_ptr()),
            0
        );
        let d: [f64; 5] = [-0.32, 0.09, 0.001, -0.0005, 0.0];
        ros_camera_info_builder_set_d(b, d.as_ptr(), d.len());
        let k: [f64; 9] = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0];
        ros_camera_info_builder_set_k(b, k.as_ptr());
        let r: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        ros_camera_info_builder_set_r(b, r.as_ptr());
        let p: [f64; 12] = [
            525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0,
        ];
        ros_camera_info_builder_set_p(b, p.as_ptr());
        ros_camera_info_builder_set_binning_x(b, 1);
        ros_camera_info_builder_set_binning_y(b, 1);
        ros_camera_info_builder_set_roi(b, 10, 20, 100, 200, 1);

        let mut buf = [0u8; 1024];
        let mut out_len: usize = 0;
        let rc = ros_camera_info_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = sensor_msgs::CameraInfo::builder()
            .stamp(Time::new(1, 2))
            .frame_id("cam_info")
            .height(480)
            .width(640)
            .distortion_model("plumb_bob")
            .d(&d)
            .k(k)
            .r(r)
            .p(p)
            .binning_x(1)
            .binning_y(1)
            .roi(RegionOfInterest {
                x_offset: 10,
                y_offset: 20,
                height: 100,
                width: 200,
                do_rectify: true,
            })
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_camera_info_builder_free(b);
    }
}

#[test]
fn ros_magnetic_field_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_magnetic_field_builder_new();
        assert!(!b.is_null());
        ros_magnetic_field_builder_set_stamp(b, 44, 55);
        let frame = CString::new("mag_link").unwrap();
        assert_eq!(
            ros_magnetic_field_builder_set_frame_id(b, frame.as_ptr()),
            0
        );
        ros_magnetic_field_builder_set_magnetic_field(b, 1e-5, 2e-5, -4e-5);
        let cov: [f64; 9] = [1e-9, 0.0, 0.0, 0.0, 1e-9, 0.0, 0.0, 0.0, 2e-9];
        ros_magnetic_field_builder_set_magnetic_field_covariance(b, cov.as_ptr());

        let mut buf = [0u8; 256];
        let mut out_len: usize = 0;
        let rc =
            ros_magnetic_field_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = sensor_msgs::MagneticField::builder()
            .stamp(Time::new(44, 55))
            .frame_id("mag_link")
            .magnetic_field(Vector3 {
                x: 1e-5,
                y: 2e-5,
                z: -4e-5,
            })
            .magnetic_field_covariance(cov)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_magnetic_field_builder_free(b);
    }
}

#[test]
fn ros_battery_state_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_battery_state_builder_new();
        assert!(!b.is_null());
        ros_battery_state_builder_set_stamp(b, 1000, 250);
        let frame = CString::new("battery").unwrap();
        assert_eq!(ros_battery_state_builder_set_frame_id(b, frame.as_ptr()), 0);
        ros_battery_state_builder_set_voltage(b, 12.5);
        ros_battery_state_builder_set_temperature(b, 30.0);
        ros_battery_state_builder_set_current(b, -1.2);
        ros_battery_state_builder_set_charge(b, 2.1);
        ros_battery_state_builder_set_capacity(b, 2.5);
        ros_battery_state_builder_set_design_capacity(b, 3.0);
        ros_battery_state_builder_set_percentage(b, 0.84);
        ros_battery_state_builder_set_power_supply_status(b, 2); // DISCHARGING
        ros_battery_state_builder_set_power_supply_health(b, 1); // GOOD
        ros_battery_state_builder_set_power_supply_technology(b, 2); // LION
        ros_battery_state_builder_set_present(b, true);
        let cv: [f32; 3] = [4.1, 4.15, 4.08];
        ros_battery_state_builder_set_cell_voltage(b, cv.as_ptr(), cv.len());
        let ct: [f32; 3] = [29.5, 30.2, 30.0];
        ros_battery_state_builder_set_cell_temperature(b, ct.as_ptr(), ct.len());
        let location = CString::new("slot_A").unwrap();
        assert_eq!(
            ros_battery_state_builder_set_location(b, location.as_ptr()),
            0
        );
        let serial = CString::new("SN-42").unwrap();
        assert_eq!(
            ros_battery_state_builder_set_serial_number(b, serial.as_ptr()),
            0
        );

        let mut buf = [0u8; 1024];
        let mut out_len: usize = 0;
        let rc =
            ros_battery_state_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = sensor_msgs::BatteryState::builder()
            .stamp(Time::new(1000, 250))
            .frame_id("battery")
            .voltage(12.5)
            .temperature(30.0)
            .current(-1.2)
            .charge(2.1)
            .capacity(2.5)
            .design_capacity(3.0)
            .percentage(0.84)
            .power_supply_status(2)
            .power_supply_health(1)
            .power_supply_technology(2)
            .present(true)
            .cell_voltage(&cv)
            .cell_temperature(&ct)
            .location("slot_A")
            .serial_number("SN-42")
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_battery_state_builder_free(b);
    }
}

#[test]
fn ros_mask_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_mask_builder_new();
        assert!(!b.is_null());
        ros_mask_builder_set_height(b, 8);
        ros_mask_builder_set_width(b, 4);
        ros_mask_builder_set_length(b, 1);
        let enc = CString::new("rle").unwrap();
        assert_eq!(ros_mask_builder_set_encoding(b, enc.as_ptr()), 0);
        let data: Vec<u8> = (0..32u8).collect();
        ros_mask_builder_set_mask(b, data.as_ptr(), data.len());
        ros_mask_builder_set_boxed(b, true);

        let mut buf = [0u8; 256];
        let mut out_len: usize = 0;
        let rc = ros_mask_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = edgefirst_msgs::Mask::builder()
            .height(8)
            .width(4)
            .length(1)
            .encoding("rle")
            .mask(&data)
            .boxed(true)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_mask_builder_free(b);
    }
}

#[test]
fn ros_local_time_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_local_time_builder_new();
        assert!(!b.is_null());
        ros_local_time_builder_set_stamp(b, 1_700_000_000, 123);
        let frame = CString::new("clock").unwrap();
        assert_eq!(ros_local_time_builder_set_frame_id(b, frame.as_ptr()), 0);
        ros_local_time_builder_set_date(b, 2025, 12, 31);
        ros_local_time_builder_set_time(b, 86399, 999_000_000);
        ros_local_time_builder_set_timezone(b, -300);

        let mut buf = [0u8; 256];
        let mut out_len: usize = 0;
        let rc = ros_local_time_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = edgefirst_msgs::LocalTime::builder()
            .stamp(Time::new(1_700_000_000, 123))
            .frame_id("clock")
            .date(Date {
                year: 2025,
                month: 12,
                day: 31,
            })
            .time(Time::new(86399, 999_000_000))
            .timezone(-300)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_local_time_builder_free(b);
    }
}

#[test]
fn ros_radar_cube_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_radar_cube_builder_new();
        assert!(!b.is_null());
        ros_radar_cube_builder_set_stamp(b, 10, 20);
        let frame = CString::new("radar").unwrap();
        assert_eq!(ros_radar_cube_builder_set_frame_id(b, frame.as_ptr()), 0);
        ros_radar_cube_builder_set_timestamp(b, 0xdead_beef_cafe_u64);
        let layout: [u8; 3] = [1, 2, 3];
        ros_radar_cube_builder_set_layout(b, layout.as_ptr(), layout.len());
        let shape: [u16; 3] = [4, 5, 6];
        ros_radar_cube_builder_set_shape(b, shape.as_ptr(), shape.len());
        let scales: [f32; 2] = [0.5, 1.5];
        ros_radar_cube_builder_set_scales(b, scales.as_ptr(), scales.len());
        let cube: [i16; 4] = [-1, 2, -3, 4];
        ros_radar_cube_builder_set_cube(b, cube.as_ptr(), cube.len());
        ros_radar_cube_builder_set_is_complex(b, true);

        let mut buf = [0u8; 512];
        let mut out_len: usize = 0;
        let rc = ros_radar_cube_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = edgefirst_msgs::RadarCube::builder()
            .stamp(Time::new(10, 20))
            .frame_id("radar")
            .timestamp(0xdead_beef_cafe_u64)
            .layout(&layout)
            .shape(&shape)
            .scales(&scales)
            .cube(&cube)
            .is_complex(true)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_radar_cube_builder_free(b);
    }
}

#[test]
fn ros_radar_info_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_radar_info_builder_new();
        assert!(!b.is_null());
        ros_radar_info_builder_set_stamp(b, 7, 8);
        let frame = CString::new("radar").unwrap();
        assert_eq!(ros_radar_info_builder_set_frame_id(b, frame.as_ptr()), 0);
        let cf = CString::new("79GHz").unwrap();
        assert_eq!(
            ros_radar_info_builder_set_center_frequency(b, cf.as_ptr()),
            0
        );
        let fs = CString::new("UWB").unwrap();
        assert_eq!(
            ros_radar_info_builder_set_frequency_sweep(b, fs.as_ptr()),
            0
        );
        let rt = CString::new("short").unwrap();
        assert_eq!(ros_radar_info_builder_set_range_toggle(b, rt.as_ptr()), 0);
        let ds = CString::new("high").unwrap();
        assert_eq!(
            ros_radar_info_builder_set_detection_sensitivity(b, ds.as_ptr()),
            0
        );
        ros_radar_info_builder_set_cube(b, true);

        let mut buf = [0u8; 256];
        let mut out_len: usize = 0;
        let rc = ros_radar_info_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = edgefirst_msgs::RadarInfo::builder()
            .stamp(Time::new(7, 8))
            .frame_id("radar")
            .center_frequency("79GHz")
            .frequency_sweep("UWB")
            .range_toggle("short")
            .detection_sensitivity("high")
            .cube(true)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_radar_info_builder_free(b);
    }
}

#[test]
fn ros_track_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_track_builder_new();
        assert!(!b.is_null());
        let id = CString::new("track-42").unwrap();
        assert_eq!(ros_track_builder_set_id(b, id.as_ptr()), 0);
        ros_track_builder_set_lifetime(b, 7);
        ros_track_builder_set_created(b, 1234, 5678);

        let mut buf = [0u8; 128];
        let mut out_len: usize = 0;
        let rc = ros_track_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = edgefirst_msgs::Track::builder()
            .id("track-42")
            .lifetime(7)
            .created(Time::new(1234, 5678))
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_track_builder_free(b);
    }
}

#[test]
fn ros_detect_box_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_detect_box_builder_new();
        assert!(!b.is_null());
        ros_detect_box_builder_set_center_x(b, 0.5);
        ros_detect_box_builder_set_center_y(b, 0.25);
        ros_detect_box_builder_set_width(b, 0.3);
        ros_detect_box_builder_set_height(b, 0.4);
        let lab = CString::new("car").unwrap();
        assert_eq!(ros_detect_box_builder_set_label(b, lab.as_ptr()), 0);
        ros_detect_box_builder_set_score(b, 0.9);
        ros_detect_box_builder_set_distance(b, 12.0);
        ros_detect_box_builder_set_speed(b, 3.5);
        let tid = CString::new("t-7").unwrap();
        assert_eq!(ros_detect_box_builder_set_track_id(b, tid.as_ptr()), 0);
        ros_detect_box_builder_set_track_lifetime(b, 5);
        ros_detect_box_builder_set_track_created(b, 100, 200);

        let mut buf = [0u8; 256];
        let mut out_len: usize = 0;
        let rc = ros_detect_box_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = edgefirst_msgs::DetectBox::builder()
            .center_x(0.5)
            .center_y(0.25)
            .width(0.3)
            .height(0.4)
            .label("car")
            .score(0.9)
            .distance(12.0)
            .speed(3.5)
            .track_id("t-7")
            .track_lifetime(5)
            .track_created(Time::new(100, 200))
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_detect_box_builder_free(b);
    }
}

#[test]
fn ros_detect_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_detect_builder_new();
        assert!(!b.is_null());
        ros_detect_builder_set_stamp(b, 1, 2);
        let frame = CString::new("camera").unwrap();
        assert_eq!(ros_detect_builder_set_frame_id(b, frame.as_ptr()), 0);
        ros_detect_builder_set_input_timestamp(b, 3, 4);
        ros_detect_builder_set_model_time(b, 5, 6);
        ros_detect_builder_set_output_time(b, 7, 8);

        let lab0 = CString::new("person").unwrap();
        let tid0 = CString::new("t-1").unwrap();
        let lab1 = CString::new("bike").unwrap();
        let tid1 = CString::new("t-2").unwrap();
        let boxes = [
            ros_detect_box_elem_t {
                center_x: 0.1,
                center_y: 0.2,
                width: 0.3,
                height: 0.4,
                label: lab0.as_ptr(),
                score: 0.95,
                distance: 2.0,
                speed: 1.0,
                track_id: tid0.as_ptr(),
                track_lifetime: 10,
                track_created_sec: 11,
                track_created_nanosec: 12,
            },
            ros_detect_box_elem_t {
                center_x: 0.5,
                center_y: 0.6,
                width: 0.7,
                height: 0.8,
                label: lab1.as_ptr(),
                score: 0.4,
                distance: 7.0,
                speed: 0.5,
                track_id: tid1.as_ptr(),
                track_lifetime: 20,
                track_created_sec: 13,
                track_created_nanosec: 14,
            },
        ];
        ros_detect_builder_set_boxes(b, boxes.as_ptr(), boxes.len());

        let mut buf = [0u8; 1024];
        let mut out_len: usize = 0;
        let rc = ros_detect_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let rust_boxes = [
            DetectBoxView {
                center_x: 0.1,
                center_y: 0.2,
                width: 0.3,
                height: 0.4,
                label: "person",
                score: 0.95,
                distance: 2.0,
                speed: 1.0,
                track_id: "t-1",
                track_lifetime: 10,
                track_created: Time::new(11, 12),
            },
            DetectBoxView {
                center_x: 0.5,
                center_y: 0.6,
                width: 0.7,
                height: 0.8,
                label: "bike",
                score: 0.4,
                distance: 7.0,
                speed: 0.5,
                track_id: "t-2",
                track_lifetime: 20,
                track_created: Time::new(13, 14),
            },
        ];
        let via_rust = edgefirst_msgs::Detect::builder()
            .stamp(Time::new(1, 2))
            .frame_id("camera")
            .input_timestamp(Time::new(3, 4))
            .model_time(Time::new(5, 6))
            .output_time(Time::new(7, 8))
            .boxes(&rust_boxes)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_detect_builder_free(b);
    }
}

#[test]
fn ros_camera_frame_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_camera_frame_builder_new();
        assert!(!b.is_null());
        ros_camera_frame_builder_set_stamp(b, 50, 60);
        let frame = CString::new("cam0").unwrap();
        assert_eq!(ros_camera_frame_builder_set_frame_id(b, frame.as_ptr()), 0);
        ros_camera_frame_builder_set_seq(b, 42);
        ros_camera_frame_builder_set_pid(b, 1234);
        ros_camera_frame_builder_set_width(b, 640);
        ros_camera_frame_builder_set_height(b, 480);
        let fmt = CString::new("NV12").unwrap();
        assert_eq!(ros_camera_frame_builder_set_format(b, fmt.as_ptr()), 0);
        let cs = CString::new("bt709").unwrap();
        assert_eq!(ros_camera_frame_builder_set_color_space(b, cs.as_ptr()), 0);
        let ct = CString::new("srgb").unwrap();
        assert_eq!(
            ros_camera_frame_builder_set_color_transfer(b, ct.as_ptr()),
            0
        );
        let ce = CString::new("bt709").unwrap();
        assert_eq!(
            ros_camera_frame_builder_set_color_encoding(b, ce.as_ptr()),
            0
        );
        let cr = CString::new("full").unwrap();
        assert_eq!(ros_camera_frame_builder_set_color_range(b, cr.as_ptr()), 0);
        ros_camera_frame_builder_set_fence_fd(b, -1);

        let plane0: Vec<u8> = (0..32u8).collect();
        let plane1: Vec<u8> = (32..48u8).collect();
        let planes = [
            ros_camera_plane_elem_t {
                fd: -1,
                offset: 0,
                stride: 640,
                size: plane0.len() as u32,
                used: plane0.len() as u32,
                data: plane0.as_ptr(),
                data_len: plane0.len(),
            },
            ros_camera_plane_elem_t {
                fd: -1,
                offset: 0,
                stride: 320,
                size: plane1.len() as u32,
                used: plane1.len() as u32,
                data: plane1.as_ptr(),
                data_len: plane1.len(),
            },
        ];
        ros_camera_frame_builder_set_planes(b, planes.as_ptr(), planes.len());

        let mut buf = [0u8; 2048];
        let mut out_len: usize = 0;
        let rc = ros_camera_frame_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let rust_planes = [
            edgefirst_msgs::CameraPlaneView {
                fd: -1,
                offset: 0,
                stride: 640,
                size: plane0.len() as u32,
                used: plane0.len() as u32,
                data: &plane0,
            },
            edgefirst_msgs::CameraPlaneView {
                fd: -1,
                offset: 0,
                stride: 320,
                size: plane1.len() as u32,
                used: plane1.len() as u32,
                data: &plane1,
            },
        ];
        let via_rust = edgefirst_msgs::CameraFrame::builder()
            .stamp(Time::new(50, 60))
            .frame_id("cam0")
            .seq(42)
            .pid(1234)
            .width(640)
            .height(480)
            .format("NV12")
            .color_space("bt709")
            .color_transfer("srgb")
            .color_encoding("bt709")
            .color_range("full")
            .fence_fd(-1)
            .planes(&rust_planes)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_camera_frame_builder_free(b);
    }
}

#[test]
fn ros_model_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_model_builder_new();
        assert!(!b.is_null());
        ros_model_builder_set_stamp(b, 1, 2);
        let frame = CString::new("model").unwrap();
        assert_eq!(ros_model_builder_set_frame_id(b, frame.as_ptr()), 0);
        ros_model_builder_set_input_time(b, 0, 1_000_000);
        ros_model_builder_set_model_time(b, 0, 2_000_000);
        ros_model_builder_set_output_time(b, 0, 3_000_000);
        ros_model_builder_set_decode_time(b, 0, 4_000_000);

        let lab = CString::new("car").unwrap();
        let tid = CString::new("t-1").unwrap();
        let boxes = [ros_detect_box_elem_t {
            center_x: 0.5,
            center_y: 0.5,
            width: 0.2,
            height: 0.2,
            label: lab.as_ptr(),
            score: 0.8,
            distance: 5.0,
            speed: 1.0,
            track_id: tid.as_ptr(),
            track_lifetime: 3,
            track_created_sec: 10,
            track_created_nanosec: 20,
        }];
        ros_model_builder_set_boxes(b, boxes.as_ptr(), boxes.len());

        let enc0 = CString::new("raw").unwrap();
        let mask0: Vec<u8> = vec![1, 0, 1, 0];
        let masks = [ros_mask_elem_t {
            height: 2,
            width: 2,
            length: 1,
            encoding: enc0.as_ptr(),
            mask: mask0.as_ptr(),
            mask_len: mask0.len(),
            boxed: false,
        }];
        ros_model_builder_set_masks(b, masks.as_ptr(), masks.len());

        let mut buf = [0u8; 1024];
        let mut out_len: usize = 0;
        let rc = ros_model_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let rust_boxes = [DetectBoxView {
            center_x: 0.5,
            center_y: 0.5,
            width: 0.2,
            height: 0.2,
            label: "car",
            score: 0.8,
            distance: 5.0,
            speed: 1.0,
            track_id: "t-1",
            track_lifetime: 3,
            track_created: Time::new(10, 20),
        }];
        let rust_masks = [MaskView {
            height: 2,
            width: 2,
            length: 1,
            encoding: "raw",
            mask: &mask0,
            boxed: false,
        }];
        let via_rust = edgefirst_msgs::Model::builder()
            .stamp(Time::new(1, 2))
            .frame_id("model")
            .input_time(Duration {
                sec: 0,
                nanosec: 1_000_000,
            })
            .model_time(Duration {
                sec: 0,
                nanosec: 2_000_000,
            })
            .output_time(Duration {
                sec: 0,
                nanosec: 3_000_000,
            })
            .decode_time(Duration {
                sec: 0,
                nanosec: 4_000_000,
            })
            .boxes(&rust_boxes)
            .masks(&rust_masks)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_model_builder_free(b);
    }
}

#[test]
fn ros_model_info_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_model_info_builder_new();
        assert!(!b.is_null());
        ros_model_info_builder_set_stamp(b, 10, 20);
        let frame = CString::new("model").unwrap();
        assert_eq!(ros_model_info_builder_set_frame_id(b, frame.as_ptr()), 0);
        let ishape: [u32; 4] = [1, 3, 224, 224];
        ros_model_info_builder_set_input_shape(b, ishape.as_ptr(), ishape.len());
        ros_model_info_builder_set_input_type(b, 8); // FLOAT32
        let oshape: [u32; 2] = [1, 1000];
        ros_model_info_builder_set_output_shape(b, oshape.as_ptr(), oshape.len());
        ros_model_info_builder_set_output_type(b, 8);

        let l0 = CString::new("car").unwrap();
        let l1 = CString::new("bike").unwrap();
        let l2 = CString::new("person").unwrap();
        let label_ptrs: [*const c_char; 3] = [l0.as_ptr(), l1.as_ptr(), l2.as_ptr()];
        assert_eq!(
            ros_model_info_builder_set_labels(b, label_ptrs.as_ptr(), label_ptrs.len()),
            0
        );
        let mt = CString::new("classifier").unwrap();
        assert_eq!(ros_model_info_builder_set_model_type(b, mt.as_ptr()), 0);
        let mf = CString::new("tflite").unwrap();
        assert_eq!(ros_model_info_builder_set_model_format(b, mf.as_ptr()), 0);
        let mn = CString::new("resnet50").unwrap();
        assert_eq!(ros_model_info_builder_set_model_name(b, mn.as_ptr()), 0);

        let mut buf = [0u8; 512];
        let mut out_len: usize = 0;
        let rc = ros_model_info_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let labels: [&str; 3] = ["car", "bike", "person"];
        let via_rust = edgefirst_msgs::ModelInfo::builder()
            .stamp(Time::new(10, 20))
            .frame_id("model")
            .input_shape(&ishape)
            .input_type(8)
            .output_shape(&oshape)
            .output_type(8)
            .labels(&labels)
            .model_type("classifier")
            .model_format("tflite")
            .model_name("resnet50")
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_model_info_builder_free(b);
    }
}

#[test]
fn ros_vibration_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_vibration_builder_new();
        assert!(!b.is_null());
        ros_vibration_builder_set_stamp(b, 5, 6);
        let frame = CString::new("vib_link").unwrap();
        assert_eq!(ros_vibration_builder_set_frame_id(b, frame.as_ptr()), 0);
        ros_vibration_builder_set_vibration(b, 0.1, -0.2, 0.3);
        ros_vibration_builder_set_band_lower_hz(b, 10.0);
        ros_vibration_builder_set_band_upper_hz(b, 1000.0);
        ros_vibration_builder_set_measurement_type(b, 1); // RMS
        ros_vibration_builder_set_unit(b, 2); // ACCEL_G
        let clipping: [u32; 3] = [7, 13, 42];
        ros_vibration_builder_set_clipping(b, clipping.as_ptr(), clipping.len());

        let mut buf = [0u8; 256];
        let mut out_len: usize = 0;
        let rc = ros_vibration_builder_encode_into(b, buf.as_mut_ptr(), buf.len(), &mut out_len);
        assert_eq!(rc, 0);

        let via_rust = edgefirst_msgs::Vibration::builder()
            .stamp(Time::new(5, 6))
            .frame_id("vib_link")
            .vibration(Vector3 {
                x: 0.1,
                y: -0.2,
                z: 0.3,
            })
            .band_lower_hz(10.0)
            .band_upper_hz(1000.0)
            .measurement_type(1)
            .unit(2)
            .clipping(&clipping)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_vibration_builder_free(b);
    }
}

#[test]
fn ros_foxglove_compressed_video_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_foxglove_compressed_video_builder_new();
        assert!(!b.is_null());
        ros_foxglove_compressed_video_builder_set_stamp(b, 100, 500_000_000);
        let frame = CString::new("camera").unwrap();
        assert_eq!(
            ros_foxglove_compressed_video_builder_set_frame_id(b, frame.as_ptr()),
            0
        );
        let data: Vec<u8> = vec![0x00, 0x00, 0x00, 0x01, 0x67, 0x42];
        ros_foxglove_compressed_video_builder_set_data(b, data.as_ptr(), data.len());
        let fmt = CString::new("h264").unwrap();
        assert_eq!(
            ros_foxglove_compressed_video_builder_set_format(b, fmt.as_ptr()),
            0
        );

        let mut buf = [0u8; 256];
        let mut out_len: usize = 0;
        let rc = ros_foxglove_compressed_video_builder_encode_into(
            b,
            buf.as_mut_ptr(),
            buf.len(),
            &mut out_len,
        );
        assert_eq!(rc, 0);

        let via_rust = foxglove_msgs::FoxgloveCompressedVideo::builder()
            .stamp(Time::new(100, 500_000_000))
            .frame_id("camera")
            .data(&data)
            .format("h264")
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_foxglove_compressed_video_builder_free(b);
    }
}

#[test]
fn ros_foxglove_text_annotation_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_foxglove_text_annotation_builder_new();
        assert!(!b.is_null());
        ros_foxglove_text_annotation_builder_set_timestamp(b, 10, 20);
        ros_foxglove_text_annotation_builder_set_position(b, 50.0, 75.0);
        let text = CString::new("hello").unwrap();
        assert_eq!(
            ros_foxglove_text_annotation_builder_set_text(b, text.as_ptr()),
            0
        );
        ros_foxglove_text_annotation_builder_set_font_size(b, 14.0);
        ros_foxglove_text_annotation_builder_set_text_color(b, 1.0, 1.0, 1.0, 1.0);
        ros_foxglove_text_annotation_builder_set_background_color(b, 0.0, 0.0, 0.0, 0.5);

        let mut buf = [0u8; 256];
        let mut out_len: usize = 0;
        let rc = ros_foxglove_text_annotation_builder_encode_into(
            b,
            buf.as_mut_ptr(),
            buf.len(),
            &mut out_len,
        );
        assert_eq!(rc, 0);

        let via_rust = foxglove_msgs::FoxgloveTextAnnotation::builder()
            .timestamp(Time::new(10, 20))
            .position(FoxglovePoint2 { x: 50.0, y: 75.0 })
            .text("hello")
            .font_size(14.0)
            .text_color(FoxgloveColor {
                r: 1.0,
                g: 1.0,
                b: 1.0,
                a: 1.0,
            })
            .background_color(FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.5,
            })
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_foxglove_text_annotation_builder_free(b);
    }
}

#[test]
fn ros_foxglove_point_annotation_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_foxglove_point_annotation_builder_new();
        assert!(!b.is_null());
        ros_foxglove_point_annotation_builder_set_timestamp(b, 11, 22);
        ros_foxglove_point_annotation_builder_set_type(b, 2); // LINE_LOOP
        let points = [
            ros_foxglove_point2_elem_t { x: 0.0, y: 0.0 },
            ros_foxglove_point2_elem_t { x: 100.0, y: 0.0 },
            ros_foxglove_point2_elem_t { x: 100.0, y: 100.0 },
        ];
        ros_foxglove_point_annotation_builder_set_points(b, points.as_ptr(), points.len());
        ros_foxglove_point_annotation_builder_set_outline_color(b, 0.0, 1.0, 0.0, 1.0);
        let oc = [ros_foxglove_color_elem_t {
            r: 1.0,
            g: 0.5,
            b: 0.0,
            a: 0.8,
        }];
        ros_foxglove_point_annotation_builder_set_outline_colors(b, oc.as_ptr(), oc.len());
        ros_foxglove_point_annotation_builder_set_fill_color(b, 0.0, 0.5, 0.0, 0.3);
        ros_foxglove_point_annotation_builder_set_thickness(b, 3.0);

        let mut buf = [0u8; 512];
        let mut out_len: usize = 0;
        let rc = ros_foxglove_point_annotation_builder_encode_into(
            b,
            buf.as_mut_ptr(),
            buf.len(),
            &mut out_len,
        );
        assert_eq!(rc, 0);

        let pts_rust = [
            FoxglovePoint2 { x: 0.0, y: 0.0 },
            FoxglovePoint2 { x: 100.0, y: 0.0 },
            FoxglovePoint2 { x: 100.0, y: 100.0 },
        ];
        let oc_rust = [FoxgloveColor {
            r: 1.0,
            g: 0.5,
            b: 0.0,
            a: 0.8,
        }];
        let via_rust = foxglove_msgs::FoxglovePointAnnotation::builder()
            .timestamp(Time::new(11, 22))
            .type_(2)
            .points(&pts_rust)
            .outline_color(FoxgloveColor {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 1.0,
            })
            .outline_colors(&oc_rust)
            .fill_color(FoxgloveColor {
                r: 0.0,
                g: 0.5,
                b: 0.0,
                a: 0.3,
            })
            .thickness(3.0)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_foxglove_point_annotation_builder_free(b);
    }
}

#[test]
fn ros_foxglove_image_annotation_builder_encode_into_matches_rust_builder() {
    unsafe {
        let b = ros_foxglove_image_annotation_builder_new();
        assert!(!b.is_null());

        let circles = [ros_foxglove_circle_annotation_elem_t {
            timestamp_sec: 1,
            timestamp_nanosec: 2,
            position_x: 10.0,
            position_y: 20.0,
            diameter: 5.0,
            thickness: 1.0,
            fill_color_r: 1.0,
            fill_color_g: 0.0,
            fill_color_b: 0.0,
            fill_color_a: 0.5,
            outline_color_r: 0.0,
            outline_color_g: 1.0,
            outline_color_b: 0.0,
            outline_color_a: 1.0,
        }];
        ros_foxglove_image_annotation_builder_set_circles(b, circles.as_ptr(), circles.len());

        let pts = [
            ros_foxglove_point2_elem_t { x: 0.0, y: 0.0 },
            ros_foxglove_point2_elem_t { x: 1.0, y: 1.0 },
        ];
        let ocs: [ros_foxglove_color_elem_t; 0] = [];
        let point_annotations = [ros_foxglove_point_annotation_elem_t {
            timestamp_sec: 3,
            timestamp_nanosec: 4,
            type_: 1, // POINTS
            points: pts.as_ptr(),
            points_count: pts.len(),
            outline_color_r: 1.0,
            outline_color_g: 1.0,
            outline_color_b: 1.0,
            outline_color_a: 1.0,
            outline_colors: ocs.as_ptr(),
            outline_colors_count: ocs.len(),
            fill_color_r: 0.2,
            fill_color_g: 0.2,
            fill_color_b: 0.2,
            fill_color_a: 0.5,
            thickness: 2.0,
        }];
        ros_foxglove_image_annotation_builder_set_points(
            b,
            point_annotations.as_ptr(),
            point_annotations.len(),
        );

        let text = CString::new("label").unwrap();
        let texts = [ros_foxglove_text_annotation_elem_t {
            timestamp_sec: 5,
            timestamp_nanosec: 6,
            position_x: 40.0,
            position_y: 50.0,
            text: text.as_ptr(),
            font_size: 12.0,
            text_color_r: 1.0,
            text_color_g: 1.0,
            text_color_b: 1.0,
            text_color_a: 1.0,
            background_color_r: 0.0,
            background_color_g: 0.0,
            background_color_b: 0.0,
            background_color_a: 0.7,
        }];
        ros_foxglove_image_annotation_builder_set_texts(b, texts.as_ptr(), texts.len());

        let mut buf = [0u8; 1024];
        let mut out_len: usize = 0;
        let rc = ros_foxglove_image_annotation_builder_encode_into(
            b,
            buf.as_mut_ptr(),
            buf.len(),
            &mut out_len,
        );
        assert_eq!(rc, 0);

        let rust_circles = [FoxgloveCircleAnnotations {
            timestamp: Time::new(1, 2),
            position: FoxglovePoint2 { x: 10.0, y: 20.0 },
            diameter: 5.0,
            thickness: 1.0,
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
        }];
        let rust_points = [FoxglovePointAnnotationView {
            timestamp: Time::new(3, 4),
            type_: 1,
            points: vec![
                FoxglovePoint2 { x: 0.0, y: 0.0 },
                FoxglovePoint2 { x: 1.0, y: 1.0 },
            ],
            outline_color: FoxgloveColor {
                r: 1.0,
                g: 1.0,
                b: 1.0,
                a: 1.0,
            },
            outline_colors: vec![],
            fill_color: FoxgloveColor {
                r: 0.2,
                g: 0.2,
                b: 0.2,
                a: 0.5,
            },
            thickness: 2.0,
        }];
        let rust_texts = [FoxgloveTextAnnotationView {
            timestamp: Time::new(5, 6),
            position: FoxglovePoint2 { x: 40.0, y: 50.0 },
            text: "label",
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
                a: 0.7,
            },
        }];
        let via_rust = foxglove_msgs::FoxgloveImageAnnotation::builder()
            .circles(&rust_circles)
            .points(&rust_points)
            .texts(&rust_texts)
            .build()
            .expect("rust builder.build()");
        assert_eq!(&buf[..out_len], via_rust.as_cdr());

        ros_foxglove_image_annotation_builder_free(b);
    }
}
