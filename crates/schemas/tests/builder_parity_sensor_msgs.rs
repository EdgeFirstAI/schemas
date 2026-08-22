// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Byte parity tests for sensor_msgs builders vs legacy new() constructors.

#![allow(deprecated)]

use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::geometry_msgs::{Quaternion, Vector3};
use edgefirst_schemas::sensor_msgs::{
    BatteryState, CameraInfo, CompressedImage, FluidPressure, Imu, MagneticField, NavSatFix,
    NavSatStatus, PointField, RegionOfInterest, Temperature,
};

#[test]
fn compressed_image_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_000, 123_456_789);
    let frame_id = "camera_optical";
    let format = "jpeg";
    let data: Vec<u8> = (0..1024u16).map(|i| (i & 0xFF) as u8).collect();

    let via_new = CompressedImage::new(stamp, frame_id, format, &data).expect("new() succeeds");
    let via_builder = CompressedImage::builder()
        .stamp(stamp)
        .frame_id(frame_id)
        .format(format)
        .data(&data)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "CompressedImage builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn imu_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_001, 250_000_000);
    let frame_id = "imu_link";
    let orientation = Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.25,
        w: 0.9375,
    };
    let orientation_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01];
    let angular_velocity = Vector3 {
        x: 0.01,
        y: -0.02,
        z: 0.5,
    };
    let angular_velocity_cov = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001];
    let linear_acceleration = Vector3 {
        x: 0.1,
        y: 0.2,
        z: 9.81,
    };
    let linear_acceleration_cov = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05];

    let via_new = Imu::new(
        stamp,
        frame_id,
        orientation,
        orientation_cov,
        angular_velocity,
        angular_velocity_cov,
        linear_acceleration,
        linear_acceleration_cov,
    )
    .expect("new() succeeds");
    let via_builder = Imu::builder()
        .stamp(stamp)
        .frame_id(frame_id)
        .orientation(orientation)
        .orientation_covariance(orientation_cov)
        .angular_velocity(angular_velocity)
        .angular_velocity_covariance(angular_velocity_cov)
        .linear_acceleration(linear_acceleration)
        .linear_acceleration_covariance(linear_acceleration_cov)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "Imu builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn nav_sat_fix_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_002, 500_000_000);
    let frame_id = "gps_link";
    let status = NavSatStatus {
        status: 0,
        service: 1,
    };
    let latitude = 45.501_7_f64;
    let longitude = -73.567_3_f64;
    let altitude = 42.5_f64;
    let pos_cov = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 4.0];
    let pos_cov_type = 2_u8;

    let via_new = NavSatFix::new(
        stamp,
        frame_id,
        status,
        latitude,
        longitude,
        altitude,
        pos_cov,
        pos_cov_type,
    )
    .expect("new() succeeds");
    let via_builder = NavSatFix::builder()
        .stamp(stamp)
        .frame_id(frame_id)
        .status(status)
        .latitude(latitude)
        .longitude(longitude)
        .altitude(altitude)
        .position_covariance(pos_cov)
        .position_covariance_type(pos_cov_type)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "NavSatFix builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn point_field_builder_byte_parity_with_new() {
    let name = "intensity";
    let offset = 16_u32;
    let datatype = 7_u8; // FLOAT32
    let count = 1_u32;

    let via_new = PointField::new(name, offset, datatype, count).expect("new() succeeds");
    let via_builder = PointField::builder()
        .name(name)
        .offset(offset)
        .datatype(datatype)
        .count(count)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "PointField builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn camera_info_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_003, 750_000_000);
    let frame_id = "camera";
    let height = 480_u32;
    let width = 640_u32;
    let distortion_model = "plumb_bob";
    let d = [0.1_f64, -0.2, 0.001, -0.0005, 0.05];
    let k = [500.0_f64, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0];
    let r = [1.0_f64, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    let p = [
        500.0_f64, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    ];
    let binning_x = 1_u32;
    let binning_y = 1_u32;
    let roi = RegionOfInterest {
        x_offset: 0,
        y_offset: 0,
        height: 480,
        width: 640,
        do_rectify: false,
    };

    let via_new = CameraInfo::new(
        stamp,
        frame_id,
        height,
        width,
        distortion_model,
        &d,
        k,
        r,
        p,
        binning_x,
        binning_y,
        roi,
    )
    .expect("new() succeeds");
    let via_builder = CameraInfo::builder()
        .stamp(stamp)
        .frame_id(frame_id)
        .height(height)
        .width(width)
        .distortion_model(distortion_model)
        .d(&d)
        .k(k)
        .r(r)
        .p(p)
        .binning_x(binning_x)
        .binning_y(binning_y)
        .roi(roi)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "CameraInfo builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn magnetic_field_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_004, 100_000_000);
    let frame_id = "magnetometer";
    let field = Vector3 {
        x: 2.5e-5,
        y: -1.2e-5,
        z: 4.8e-5,
    };
    let cov = [1e-10, 0.0, 0.0, 0.0, 1e-10, 0.0, 0.0, 0.0, 1e-10];

    let via_new = MagneticField::new(stamp, frame_id, field, cov).expect("new() succeeds");
    let via_builder = MagneticField::builder()
        .stamp(stamp)
        .frame_id(frame_id)
        .magnetic_field(field)
        .magnetic_field_covariance(cov)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "MagneticField builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn fluid_pressure_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_005, 200_000_000);
    let frame_id = "barometer";
    let pressure = 101_325.0_f64;
    let variance = 2.5_f64;

    let via_new = FluidPressure::new(stamp, frame_id, pressure, variance).expect("new() succeeds");
    let via_builder = FluidPressure::builder()
        .stamp(stamp)
        .frame_id(frame_id)
        .fluid_pressure(pressure)
        .variance(variance)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "FluidPressure builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn temperature_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_006, 300_000_000);
    let frame_id = "thermocouple";
    let temperature = 23.75_f64;
    let variance = 0.05_f64;

    let via_new = Temperature::new(stamp, frame_id, temperature, variance).expect("new() succeeds");
    let via_builder = Temperature::builder()
        .stamp(stamp)
        .frame_id(frame_id)
        .temperature(temperature)
        .variance(variance)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "Temperature builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn battery_state_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_007, 400_000_000);
    let frame_id = "battery_0";
    let voltage = 12.6_f32;
    let temperature = 28.5_f32;
    let current = -1.25_f32;
    let charge = 3500.0_f32;
    let capacity = 5000.0_f32;
    let design_capacity = 5200.0_f32;
    let percentage = 0.7_f32;
    let power_supply_status = 2_u8; // DISCHARGING
    let power_supply_health = 1_u8; // GOOD
    let power_supply_technology = 2_u8; // LION
    let present = true;
    let cell_voltage = [3.15_f32, 3.16, 3.17, 3.18];
    let cell_temperature = [28.1_f32, 28.4, 28.8, 28.6];
    let location = "main_bay";
    let serial_number = "SN-ABC-00123";

    let via_new = BatteryState::new(
        stamp,
        frame_id,
        voltage,
        temperature,
        current,
        charge,
        capacity,
        design_capacity,
        percentage,
        power_supply_status,
        power_supply_health,
        power_supply_technology,
        present,
        &cell_voltage,
        &cell_temperature,
        location,
        serial_number,
    )
    .expect("new() succeeds");
    let via_builder = BatteryState::builder()
        .stamp(stamp)
        .frame_id(frame_id)
        .voltage(voltage)
        .temperature(temperature)
        .current(current)
        .charge(charge)
        .capacity(capacity)
        .design_capacity(design_capacity)
        .percentage(percentage)
        .power_supply_status(power_supply_status)
        .power_supply_health(power_supply_health)
        .power_supply_technology(power_supply_technology)
        .present(present)
        .cell_voltage(&cell_voltage)
        .cell_temperature(&cell_temperature)
        .location(location)
        .serial_number(serial_number)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "BatteryState builder and new() must produce identical CDR bytes",
    );
}
