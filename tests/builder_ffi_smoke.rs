// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Smoke tests for the `ros_<type>_builder_*` C FFI surface.
//!
//! Each test drives the C FFI through unsafe extern declarations and verifies
//! the bytes written by `..._builder_encode_into` match those produced by the
//! native Rust builder for the same field assignment. This pins the FFI
//! contract: any divergence from the Rust builder is a regression.

#![allow(non_camel_case_types)]

use edgefirst_schemas::builtin_interfaces::Time;
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

/// C-POD field descriptor for `ros_point_cloud2_builder_set_fields`.
#[repr(C)]
struct ros_point_field_elem_t {
    name: *const c_char,
    offset: u32,
    datatype: u8,
    count: u32,
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
