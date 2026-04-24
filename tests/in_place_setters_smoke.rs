// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Smoke tests for the in-place `ros_<type>_set_<field>(buf, len, ...)` FFI.
//!
//! Each test builds a representative message via the Rust builder, mutates a
//! fixed-size field through the C FFI setter, decodes the result, and asserts
//! the mutated field reflects the new value while an unmutated neighbour
//! field is unchanged. Error paths (NULL buf, malformed buffer) are also
//! covered.

#![allow(non_camel_case_types)]

use edgefirst_schemas::builtin_interfaces::{Duration, Time};
use edgefirst_schemas::edgefirst_msgs;
use edgefirst_schemas::foxglove_msgs;
use edgefirst_schemas::geometry_msgs::Quaternion;
use edgefirst_schemas::sensor_msgs;
use edgefirst_schemas::std_msgs;

extern "C" {
    // std_msgs
    fn ros_header_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32;

    // sensor_msgs::Image
    fn ros_image_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32;
    fn ros_image_set_height(buf: *mut u8, len: usize, v: u32) -> i32;
    fn ros_image_set_width(buf: *mut u8, len: usize, v: u32) -> i32;
    fn ros_image_set_is_bigendian(buf: *mut u8, len: usize, v: u8) -> i32;
    fn ros_image_set_step(buf: *mut u8, len: usize, v: u32) -> i32;

    // sensor_msgs::CompressedImage
    fn ros_compressed_image_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32;

    // sensor_msgs::Imu
    fn ros_imu_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32;
    fn ros_imu_set_orientation(buf: *mut u8, len: usize, x: f64, y: f64, z: f64, w: f64) -> i32;
    fn ros_imu_set_orientation_covariance(buf: *mut u8, len: usize, c: *const f64) -> i32;
    fn ros_imu_set_angular_velocity(buf: *mut u8, len: usize, x: f64, y: f64, z: f64) -> i32;
    fn ros_imu_set_linear_acceleration(buf: *mut u8, len: usize, x: f64, y: f64, z: f64) -> i32;

    // sensor_msgs::NavSatFix
    fn ros_nav_sat_fix_set_status(buf: *mut u8, len: usize, status: i8, service: u16) -> i32;
    fn ros_nav_sat_fix_set_latitude(buf: *mut u8, len: usize, v: f64) -> i32;
    fn ros_nav_sat_fix_set_altitude(buf: *mut u8, len: usize, v: f64) -> i32;

    // sensor_msgs::PointField
    fn ros_point_field_set_offset(buf: *mut u8, len: usize, v: u32) -> i32;
    fn ros_point_field_set_datatype(buf: *mut u8, len: usize, v: u8) -> i32;

    // sensor_msgs::PointCloud2
    fn ros_point_cloud2_set_height(buf: *mut u8, len: usize, v: u32) -> i32;
    fn ros_point_cloud2_set_is_bigendian(buf: *mut u8, len: usize, v: u8) -> i32;
    fn ros_point_cloud2_set_is_dense(buf: *mut u8, len: usize, v: u8) -> i32;

    // sensor_msgs::CameraInfo
    fn ros_camera_info_set_height(buf: *mut u8, len: usize, v: u32) -> i32;
    fn ros_camera_info_set_k(buf: *mut u8, len: usize, k: *const f64) -> i32;
    fn ros_camera_info_set_p(buf: *mut u8, len: usize, p: *const f64) -> i32;
    fn ros_camera_info_set_roi(
        buf: *mut u8,
        len: usize,
        x_offset: u32,
        y_offset: u32,
        height: u32,
        width: u32,
        do_rectify: u8,
    ) -> i32;

    // sensor_msgs::MagneticField
    fn ros_magnetic_field_set_magnetic_field(
        buf: *mut u8,
        len: usize,
        x: f64,
        y: f64,
        z: f64,
    ) -> i32;

    // sensor_msgs::FluidPressure
    fn ros_fluid_pressure_set_variance(buf: *mut u8, len: usize, v: f64) -> i32;

    // sensor_msgs::Temperature
    fn ros_temperature_set_temperature(buf: *mut u8, len: usize, v: f64) -> i32;

    // sensor_msgs::BatteryState
    fn ros_battery_state_set_voltage(buf: *mut u8, len: usize, v: f32) -> i32;
    fn ros_battery_state_set_present(buf: *mut u8, len: usize, v: u8) -> i32;

    // edgefirst_msgs::Mask
    fn ros_mask_set_height(buf: *mut u8, len: usize, v: u32) -> i32;
    fn ros_mask_set_boxed(buf: *mut u8, len: usize, v: u8) -> i32;

    // edgefirst_msgs::LocalTime
    fn ros_local_time_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32;
    fn ros_local_time_set_date(buf: *mut u8, len: usize, year: u16, month: u8, day: u8) -> i32;
    fn ros_local_time_set_time(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32;
    fn ros_local_time_set_timezone(buf: *mut u8, len: usize, v: i16) -> i32;

    // edgefirst_msgs::RadarCube
    fn ros_radar_cube_set_timestamp(buf: *mut u8, len: usize, v: u64) -> i32;

    // edgefirst_msgs::RadarInfo
    fn ros_radar_info_set_cube(buf: *mut u8, len: usize, v: u8) -> i32;

    // edgefirst_msgs::Track
    fn ros_track_set_lifetime(buf: *mut u8, len: usize, v: i32) -> i32;

    // edgefirst_msgs::DetectBox
    fn ros_detect_box_set_center_x(buf: *mut u8, len: usize, v: f32) -> i32;
    fn ros_detect_box_set_score(buf: *mut u8, len: usize, v: f32) -> i32;

    // edgefirst_msgs::Detect
    fn ros_detect_set_stamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32;
    fn ros_detect_set_input_timestamp(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32;

    // edgefirst_msgs::CameraFrame
    fn ros_camera_frame_set_seq(buf: *mut u8, len: usize, v: u64) -> i32;
    fn ros_camera_frame_set_fence_fd(buf: *mut u8, len: usize, v: i32) -> i32;

    // edgefirst_msgs::Model
    fn ros_model_set_input_time(buf: *mut u8, len: usize, sec: i32, nsec: u32) -> i32;

    // edgefirst_msgs::ModelInfo
    fn ros_model_info_set_input_type(buf: *mut u8, len: usize, v: u8) -> i32;

    // edgefirst_msgs::Vibration
    fn ros_vibration_set_band_lower_hz(buf: *mut u8, len: usize, v: f32) -> i32;

    // foxglove_msgs::FoxgloveCompressedVideo
    fn ros_foxglove_compressed_video_set_stamp(
        buf: *mut u8,
        len: usize,
        sec: i32,
        nsec: u32,
    ) -> i32;

    // foxglove_msgs::FoxgloveTextAnnotation
    fn ros_foxglove_text_annotation_set_position(buf: *mut u8, len: usize, x: f64, y: f64) -> i32;
    fn ros_foxglove_text_annotation_set_font_size(buf: *mut u8, len: usize, v: f64) -> i32;

    // foxglove_msgs::FoxglovePointAnnotation
    fn ros_foxglove_point_annotation_set_type(buf: *mut u8, len: usize, v: u8) -> i32;
    fn ros_foxglove_point_annotation_set_thickness(buf: *mut u8, len: usize, v: f64) -> i32;
}

fn errno() -> i32 {
    errno::errno().0
}

// ----- std_msgs::Header -----

#[test]
fn ros_header_in_place_set_stamp_mutates_only_stamp() {
    let hdr = std_msgs::Header::builder()
        .stamp(Time::new(100, 200))
        .frame_id("base_link")
        .build()
        .unwrap();
    let mut buf = hdr.to_cdr();
    let len = buf.len();

    let rc = unsafe { ros_header_set_stamp(buf.as_mut_ptr(), len, 999, 7) };
    assert_eq!(rc, 0);

    let decoded = std_msgs::Header::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.stamp(), Time::new(999, 7));
    assert_eq!(decoded.frame_id(), "base_link");
}

// ----- sensor_msgs::Image -----

#[test]
fn ros_image_in_place_setters_mutate_selected_fields() {
    let msg = sensor_msgs::Image::builder()
        .stamp(Time::new(1, 2))
        .frame_id("cam")
        .height(4)
        .width(3)
        .encoding("rgb8")
        .is_bigendian(0)
        .step(9)
        .data(&[0u8; 36])
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();

    assert_eq!(
        unsafe { ros_image_set_stamp(buf.as_mut_ptr(), len, 1234, 5678) },
        0
    );
    assert_eq!(
        unsafe { ros_image_set_height(buf.as_mut_ptr(), len, 40) },
        0
    );
    assert_eq!(unsafe { ros_image_set_width(buf.as_mut_ptr(), len, 30) }, 0);
    assert_eq!(
        unsafe { ros_image_set_is_bigendian(buf.as_mut_ptr(), len, 1) },
        0
    );
    assert_eq!(unsafe { ros_image_set_step(buf.as_mut_ptr(), len, 90) }, 0);

    let decoded = sensor_msgs::Image::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.stamp(), Time::new(1234, 5678));
    assert_eq!(decoded.height(), 40);
    assert_eq!(decoded.width(), 30);
    assert_eq!(decoded.is_bigendian(), 1);
    assert_eq!(decoded.step(), 90);
    // Unchanged variable fields still intact.
    assert_eq!(decoded.frame_id(), "cam");
    assert_eq!(decoded.encoding(), "rgb8");
}

// ----- sensor_msgs::CompressedImage -----

#[test]
fn ros_compressed_image_in_place_set_stamp() {
    let msg = sensor_msgs::CompressedImage::builder()
        .stamp(Time::new(1, 2))
        .frame_id("cam")
        .format("jpeg")
        .data(b"abc")
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_compressed_image_set_stamp(buf.as_mut_ptr(), len, 77, 88) },
        0
    );
    let decoded = sensor_msgs::CompressedImage::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.stamp(), Time::new(77, 88));
    assert_eq!(decoded.format(), "jpeg");
}

// ----- sensor_msgs::Imu -----

#[test]
fn ros_imu_in_place_setters_mutate_orientation_and_covariance() {
    let msg = sensor_msgs::Imu::builder()
        .stamp(Time::new(0, 0))
        .frame_id("imu")
        .orientation(Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        })
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();

    assert_eq!(
        unsafe { ros_imu_set_stamp(buf.as_mut_ptr(), len, 100, 200) },
        0
    );
    assert_eq!(
        unsafe { ros_imu_set_orientation(buf.as_mut_ptr(), len, 0.1, 0.2, 0.3, 0.9) },
        0
    );
    let cov = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
    assert_eq!(
        unsafe { ros_imu_set_orientation_covariance(buf.as_mut_ptr(), len, cov.as_ptr()) },
        0
    );
    assert_eq!(
        unsafe { ros_imu_set_angular_velocity(buf.as_mut_ptr(), len, 1.0, 2.0, 3.0) },
        0
    );
    assert_eq!(
        unsafe { ros_imu_set_linear_acceleration(buf.as_mut_ptr(), len, 4.0, 5.0, 6.0) },
        0
    );

    let decoded = sensor_msgs::Imu::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.stamp(), Time::new(100, 200));
    let q = decoded.orientation();
    assert!((q.x - 0.1).abs() < 1e-12);
    assert!((q.w - 0.9).abs() < 1e-12);
    assert_eq!(decoded.orientation_covariance(), cov);
}

// ----- sensor_msgs::NavSatFix -----

#[test]
fn ros_nav_sat_fix_in_place_setters() {
    let msg = sensor_msgs::NavSatFix::builder()
        .frame_id("gps")
        .latitude(12.0)
        .longitude(34.0)
        .altitude(56.0)
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();

    assert_eq!(
        unsafe { ros_nav_sat_fix_set_status(buf.as_mut_ptr(), len, 2, 5) },
        0
    );
    assert_eq!(
        unsafe { ros_nav_sat_fix_set_latitude(buf.as_mut_ptr(), len, 98.25) },
        0
    );
    assert_eq!(
        unsafe { ros_nav_sat_fix_set_altitude(buf.as_mut_ptr(), len, 123.0) },
        0
    );

    let decoded = sensor_msgs::NavSatFix::from_cdr(&buf[..]).unwrap();
    assert!((decoded.latitude() - 98.25).abs() < 1e-12);
    assert!((decoded.altitude() - 123.0).abs() < 1e-12);
    let s = decoded.status();
    assert_eq!(s.status, 2);
    assert_eq!(s.service, 5);
}

// ----- sensor_msgs::PointField -----

#[test]
fn ros_point_field_in_place_setters() {
    let msg = sensor_msgs::PointField::builder()
        .name("x")
        .offset(0)
        .datatype(sensor_msgs::point_field::FLOAT32)
        .count(1)
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();

    assert_eq!(
        unsafe { ros_point_field_set_offset(buf.as_mut_ptr(), len, 12) },
        0
    );
    assert_eq!(
        unsafe { ros_point_field_set_datatype(buf.as_mut_ptr(), len, 8) },
        0
    );
    let decoded = sensor_msgs::PointField::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.offset(), 12);
    assert_eq!(decoded.datatype(), 8);
    assert_eq!(decoded.name(), "x");
}

// ----- sensor_msgs::PointCloud2 -----

#[test]
fn ros_point_cloud2_in_place_setters() {
    let msg = sensor_msgs::PointCloud2::builder()
        .frame_id("lidar")
        .height(1)
        .width(2)
        .point_step(4)
        .row_step(8)
        .data(&[0u8; 8])
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();

    assert_eq!(
        unsafe { ros_point_cloud2_set_height(buf.as_mut_ptr(), len, 5) },
        0
    );
    assert_eq!(
        unsafe { ros_point_cloud2_set_is_bigendian(buf.as_mut_ptr(), len, 1) },
        0
    );
    assert_eq!(
        unsafe { ros_point_cloud2_set_is_dense(buf.as_mut_ptr(), len, 1) },
        0
    );
    let decoded = sensor_msgs::PointCloud2::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.height(), 5);
    assert!(decoded.is_bigendian());
    assert!(decoded.is_dense());
}

// ----- sensor_msgs::CameraInfo -----

#[test]
fn ros_camera_info_in_place_setters() {
    let msg = sensor_msgs::CameraInfo::builder()
        .frame_id("cam")
        .height(480)
        .width(640)
        .distortion_model("plumb_bob")
        .d(&[0.0, 0.0, 0.0, 0.0, 0.0])
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();

    assert_eq!(
        unsafe { ros_camera_info_set_height(buf.as_mut_ptr(), len, 1024) },
        0
    );
    let k = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];
    assert_eq!(
        unsafe { ros_camera_info_set_k(buf.as_mut_ptr(), len, k.as_ptr()) },
        0
    );
    let p = [
        10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0,
    ];
    assert_eq!(
        unsafe { ros_camera_info_set_p(buf.as_mut_ptr(), len, p.as_ptr()) },
        0
    );
    assert_eq!(
        unsafe { ros_camera_info_set_roi(buf.as_mut_ptr(), len, 1, 2, 3, 4, 1) },
        0
    );
    let decoded = sensor_msgs::CameraInfo::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.height(), 1024);
    assert_eq!(decoded.k(), k);
    assert_eq!(decoded.p(), p);
    let roi = decoded.roi();
    assert_eq!(roi.x_offset, 1);
    assert_eq!(roi.width, 4);
    assert!(roi.do_rectify);
}

// ----- sensor_msgs::MagneticField -----

#[test]
fn ros_magnetic_field_in_place_setters() {
    let msg = sensor_msgs::MagneticField::builder()
        .frame_id("mag")
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_magnetic_field_set_magnetic_field(buf.as_mut_ptr(), len, 1.0, 2.0, 3.0) },
        0
    );
    let decoded = sensor_msgs::MagneticField::from_cdr(&buf[..]).unwrap();
    let m = decoded.magnetic_field();
    assert_eq!(m.x, 1.0);
    assert_eq!(m.z, 3.0);
}

// ----- sensor_msgs::FluidPressure -----

#[test]
fn ros_fluid_pressure_in_place_setters() {
    let msg = sensor_msgs::FluidPressure::builder()
        .frame_id("fp")
        .fluid_pressure(100.0)
        .variance(1.0)
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_fluid_pressure_set_variance(buf.as_mut_ptr(), len, 42.0) },
        0
    );
    let decoded = sensor_msgs::FluidPressure::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.variance(), 42.0);
    assert_eq!(decoded.fluid_pressure(), 100.0);
}

// ----- sensor_msgs::Temperature -----

#[test]
fn ros_temperature_in_place_setters() {
    let msg = sensor_msgs::Temperature::builder()
        .frame_id("t")
        .temperature(25.0)
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_temperature_set_temperature(buf.as_mut_ptr(), len, -10.5) },
        0
    );
    let decoded = sensor_msgs::Temperature::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.temperature(), -10.5);
}

// ----- sensor_msgs::BatteryState -----

#[test]
fn ros_battery_state_in_place_setters() {
    let msg = sensor_msgs::BatteryState::builder()
        .frame_id("bat")
        .voltage(11.0)
        .present(false)
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_battery_state_set_voltage(buf.as_mut_ptr(), len, 12.5) },
        0
    );
    assert_eq!(
        unsafe { ros_battery_state_set_present(buf.as_mut_ptr(), len, 1) },
        0
    );
    let decoded = sensor_msgs::BatteryState::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.voltage(), 12.5);
    assert!(decoded.present());
}

// ----- edgefirst_msgs::Mask -----

#[test]
fn ros_mask_in_place_setters() {
    let msg = edgefirst_msgs::Mask::builder()
        .height(4)
        .width(4)
        .length(16)
        .encoding("u8")
        .mask(&[0u8; 16])
        .boxed(false)
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(unsafe { ros_mask_set_height(buf.as_mut_ptr(), len, 8) }, 0);
    assert_eq!(unsafe { ros_mask_set_boxed(buf.as_mut_ptr(), len, 1) }, 0);
    let decoded = edgefirst_msgs::Mask::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.height(), 8);
    assert!(decoded.boxed());
}

// ----- edgefirst_msgs::LocalTime -----

#[test]
fn ros_local_time_in_place_setters_round_trip() {
    // LocalTime's date / time / timezone fields live at CDR-dynamic offsets
    // (after the Header's variable-length frame_id string). The setter
    // must apply CDR alignment to locate those fields — u16 year needs
    // 2-align, i32 sec needs 4-align, i16 timezone needs 2-align — and
    // this alignment math depends on the frame_id length mod 4.
    // Exercise all four parities (0..=3) to catch off-by-alignment bugs.
    for frame_id in ["", "a", "ab", "abc"] {
        let msg = edgefirst_msgs::LocalTime::builder()
            .frame_id(frame_id)
            .date(edgefirst_msgs::Date {
                year: 2000,
                month: 1,
                day: 2,
            })
            .time(Time::new(3, 4))
            .timezone(0)
            .build()
            .unwrap();
        let mut buf = msg.into_cdr();
        let len = buf.len();

        assert_eq!(
            unsafe { ros_local_time_set_stamp(buf.as_mut_ptr(), len, 50, 60) },
            0,
            "frame_id={frame_id:?}",
        );
        assert_eq!(
            unsafe { ros_local_time_set_date(buf.as_mut_ptr(), len, 2026, 4, 23) },
            0,
            "frame_id={frame_id:?}",
        );
        assert_eq!(
            unsafe { ros_local_time_set_time(buf.as_mut_ptr(), len, 77, 88) },
            0,
            "frame_id={frame_id:?}",
        );
        assert_eq!(
            unsafe { ros_local_time_set_timezone(buf.as_mut_ptr(), len, -300) },
            0,
            "frame_id={frame_id:?}",
        );

        // Decode and verify every field was written at the correct offset.
        let decoded = edgefirst_msgs::LocalTime::from_cdr(&buf[..]).unwrap();
        assert_eq!(decoded.stamp(), Time::new(50, 60), "stamp for {frame_id:?}");
        assert_eq!(decoded.frame_id(), frame_id);
        let d = decoded.date();
        assert_eq!(
            (d.year, d.month, d.day),
            (2026, 4, 23),
            "date for {frame_id:?}",
        );
        assert_eq!(decoded.time(), Time::new(77, 88), "time for {frame_id:?}");
        assert_eq!(decoded.timezone(), -300, "timezone for {frame_id:?}");
    }
}

// ----- edgefirst_msgs::RadarCube -----

#[test]
fn ros_radar_cube_in_place_set_timestamp() {
    let msg = edgefirst_msgs::RadarCube::builder()
        .frame_id("radar")
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_radar_cube_set_timestamp(buf.as_mut_ptr(), len, 1_234_567_890_u64) },
        0
    );
    let decoded = edgefirst_msgs::RadarCube::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.timestamp(), 1_234_567_890_u64);
}

// ----- edgefirst_msgs::RadarInfo -----

#[test]
fn ros_radar_info_in_place_set_cube() {
    let msg = edgefirst_msgs::RadarInfo::builder()
        .frame_id("radar")
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_radar_info_set_cube(buf.as_mut_ptr(), len, 1) },
        0
    );
    let decoded = edgefirst_msgs::RadarInfo::from_cdr(&buf[..]).unwrap();
    assert!(decoded.cube());
}

// ----- edgefirst_msgs::Track -----

#[test]
fn ros_track_in_place_set_lifetime() {
    let msg = edgefirst_msgs::Track::builder()
        .id("tr-1")
        .lifetime(10)
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_track_set_lifetime(buf.as_mut_ptr(), len, 999) },
        0
    );
    let decoded = edgefirst_msgs::Track::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.lifetime(), 999);
    assert_eq!(decoded.id(), "tr-1");
}

// ----- edgefirst_msgs::DetectBox -----

#[test]
fn ros_detect_box_in_place_setters() {
    let msg = edgefirst_msgs::DetectBox::builder()
        .center_x(1.0)
        .center_y(2.0)
        .width(3.0)
        .height(4.0)
        .label("car")
        .score(0.5)
        .track_id("")
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_detect_box_set_center_x(buf.as_mut_ptr(), len, 99.0) },
        0
    );
    assert_eq!(
        unsafe { ros_detect_box_set_score(buf.as_mut_ptr(), len, 0.95) },
        0
    );
    let decoded = edgefirst_msgs::DetectBox::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.center_x(), 99.0);
    assert_eq!(decoded.score(), 0.95);
    assert_eq!(decoded.label(), "car");
}

// ----- edgefirst_msgs::Detect -----

#[test]
fn ros_detect_in_place_setters() {
    let msg = edgefirst_msgs::Detect::builder()
        .frame_id("det")
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_detect_set_stamp(buf.as_mut_ptr(), len, 10, 20) },
        0
    );
    assert_eq!(
        unsafe { ros_detect_set_input_timestamp(buf.as_mut_ptr(), len, 30, 40) },
        0
    );
    let decoded = edgefirst_msgs::Detect::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.stamp(), Time::new(10, 20));
    assert_eq!(decoded.input_timestamp(), Time::new(30, 40));
}

// ----- edgefirst_msgs::CameraFrame -----

#[test]
fn ros_camera_frame_in_place_setters() {
    let msg = edgefirst_msgs::CameraFrame::builder()
        .frame_id("cam")
        .seq(1)
        .pid(2)
        .width(320)
        .height(240)
        .format("nv12")
        .fence_fd(-1)
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_camera_frame_set_seq(buf.as_mut_ptr(), len, 42) },
        0
    );
    assert_eq!(
        unsafe { ros_camera_frame_set_fence_fd(buf.as_mut_ptr(), len, 7) },
        0
    );
    let decoded = edgefirst_msgs::CameraFrame::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.seq(), 42);
    assert_eq!(decoded.fence_fd(), 7);
}

// ----- edgefirst_msgs::Model -----

#[test]
fn ros_model_in_place_set_input_time() {
    let msg = edgefirst_msgs::Model::builder()
        .frame_id("m")
        .input_time(Duration::new(0, 0))
        .model_time(Duration::new(0, 0))
        .output_time(Duration::new(0, 0))
        .decode_time(Duration::new(0, 0))
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_model_set_input_time(buf.as_mut_ptr(), len, 5, 6) },
        0
    );
    let decoded = edgefirst_msgs::Model::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.input_time(), Duration::new(5, 6));
}

// ----- edgefirst_msgs::ModelInfo -----

#[test]
fn ros_model_info_in_place_set_input_type() {
    let msg = edgefirst_msgs::ModelInfo::builder()
        .frame_id("mi")
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_model_info_set_input_type(buf.as_mut_ptr(), len, 3) },
        0
    );
    let decoded = edgefirst_msgs::ModelInfo::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.input_type(), 3);
}

// ----- edgefirst_msgs::Vibration -----

#[test]
fn ros_vibration_in_place_set_band_lower_hz() {
    let msg = edgefirst_msgs::Vibration::builder()
        .frame_id("v")
        .band_lower_hz(10.0)
        .band_upper_hz(100.0)
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_vibration_set_band_lower_hz(buf.as_mut_ptr(), len, 20.0) },
        0
    );
    let decoded = edgefirst_msgs::Vibration::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.band_lower_hz(), 20.0);
    assert_eq!(decoded.band_upper_hz(), 100.0);
}

// ----- foxglove_msgs::FoxgloveCompressedVideo -----

#[test]
fn ros_foxglove_compressed_video_in_place_set_stamp() {
    let msg = foxglove_msgs::FoxgloveCompressedVideo::builder()
        .frame_id("cam")
        .format("h264")
        .data(b"xyz")
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_foxglove_compressed_video_set_stamp(buf.as_mut_ptr(), len, 50, 60) },
        0
    );
    let decoded = foxglove_msgs::FoxgloveCompressedVideo::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.stamp(), Time::new(50, 60));
}

// ----- foxglove_msgs::FoxgloveTextAnnotation -----

#[test]
fn ros_foxglove_text_annotation_in_place_setters() {
    let msg = foxglove_msgs::FoxgloveTextAnnotation::builder()
        .timestamp(Time::new(0, 0))
        .text("hi")
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_foxglove_text_annotation_set_position(buf.as_mut_ptr(), len, 1.5, 2.5) },
        0
    );
    assert_eq!(
        unsafe { ros_foxglove_text_annotation_set_font_size(buf.as_mut_ptr(), len, 14.0) },
        0
    );
    let decoded = foxglove_msgs::FoxgloveTextAnnotation::from_cdr(&buf[..]).unwrap();
    let p = decoded.position();
    assert_eq!(p.x, 1.5);
    assert_eq!(p.y, 2.5);
    assert_eq!(decoded.font_size(), 14.0);
    assert_eq!(decoded.text(), "hi");
}

// ----- foxglove_msgs::FoxglovePointAnnotation -----

#[test]
fn ros_foxglove_point_annotation_in_place_setters() {
    let msg = foxglove_msgs::FoxglovePointAnnotation::builder()
        .timestamp(Time::new(0, 0))
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    assert_eq!(
        unsafe { ros_foxglove_point_annotation_set_type(buf.as_mut_ptr(), len, 2) },
        0
    );
    assert_eq!(
        unsafe { ros_foxglove_point_annotation_set_thickness(buf.as_mut_ptr(), len, 3.5) },
        0
    );
    let decoded = foxglove_msgs::FoxglovePointAnnotation::from_cdr(&buf[..]).unwrap();
    assert_eq!(decoded.type_(), 2);
    assert_eq!(decoded.thickness(), 3.5);
}

// ----- Error paths -----

#[test]
fn ros_image_set_stamp_null_buf_returns_einval() {
    errno::set_errno(errno::Errno(0));
    let rc = unsafe { ros_image_set_stamp(std::ptr::null_mut(), 0, 0, 0) };
    assert_eq!(rc, -1);
    assert_eq!(errno(), libc::EINVAL);
}

#[test]
fn ros_image_set_stamp_malformed_buf_returns_ebadmsg() {
    errno::set_errno(errno::Errno(0));
    // Too-short buffer is not a valid CDR Image.
    let mut buf = [0u8; 4];
    let rc = unsafe { ros_image_set_stamp(buf.as_mut_ptr(), buf.len(), 0, 0) };
    assert_eq!(rc, -1);
    assert_eq!(errno(), libc::EBADMSG);
}

#[test]
fn ros_imu_set_orientation_covariance_null_array_returns_einval() {
    // Build a valid Imu buffer so the check precedes from_cdr.
    let msg = sensor_msgs::Imu::builder()
        .frame_id("imu")
        .orientation(Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        })
        .build()
        .unwrap();
    let mut buf = msg.into_cdr();
    let len = buf.len();
    errno::set_errno(errno::Errno(0));
    let rc = unsafe { ros_imu_set_orientation_covariance(buf.as_mut_ptr(), len, std::ptr::null()) };
    assert_eq!(rc, -1);
    assert_eq!(errno(), libc::EINVAL);
}
