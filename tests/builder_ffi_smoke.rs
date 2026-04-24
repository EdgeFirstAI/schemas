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
use edgefirst_schemas::sensor_msgs;
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
