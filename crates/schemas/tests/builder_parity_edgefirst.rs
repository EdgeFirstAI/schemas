// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Byte parity tests for edgefirst_msgs builders vs legacy new() constructors.

#![allow(deprecated)]

use edgefirst_schemas::builtin_interfaces::{Duration, Time};
use edgefirst_schemas::edgefirst_msgs::{
    Date, Detect, DetectBox, DetectBoxView, LocalTime, Mask, MaskView, Model, ModelInfo, RadarCube,
    RadarInfo, Track, Vibration,
};
use edgefirst_schemas::geometry_msgs::Vector3;

#[test]
fn mask_builder_byte_parity_with_new() {
    let mask_data: Vec<u8> = (0..1024u32).map(|i| (i & 0xff) as u8).collect();

    let via_new = Mask::new(32, 32, 1024, "raw", &mask_data, true).expect("new() succeeds");
    let via_builder = Mask::builder()
        .height(32)
        .width(32)
        .length(1024)
        .encoding("raw")
        .mask(&mask_data)
        .boxed(true)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "Mask builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn local_time_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_000, 123_456_789);
    let date = Date {
        year: 2026,
        month: 4,
        day: 23,
    };
    let time = Time::new(43_200, 500_000_000);

    let via_new = LocalTime::new(stamp, "clock", date, time, -300).expect("new() succeeds");
    let via_builder = LocalTime::builder()
        .stamp(stamp)
        .frame_id("clock")
        .date(date)
        .time(time)
        .timezone(-300)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "LocalTime builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn radar_cube_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_000, 987_654_321);
    let layout: &[u8] = &[6, 1, 5, 2];
    let shape: &[u16] = &[16, 256, 4, 64];
    let scales: &[f32] = &[1.0, 2.5, 1.0, 0.5];
    let cube: &[i16] = &[100, 200, -100, -200, 0, 1, -1, 42];

    let via_new = RadarCube::new(
        stamp,
        "radar_front",
        1_234_567_890_123_456u64,
        layout,
        shape,
        scales,
        cube,
        true,
    )
    .expect("new() succeeds");
    let via_builder = RadarCube::builder()
        .stamp(stamp)
        .frame_id("radar_front")
        .timestamp(1_234_567_890_123_456u64)
        .layout(layout)
        .shape(shape)
        .scales(scales)
        .cube(cube)
        .is_complex(true)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "RadarCube builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn radar_info_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_010, 0);

    let via_new = RadarInfo::new(stamp, "radar_front", "77GHz", "wide", "off", "high", true)
        .expect("new() succeeds");
    let via_builder = RadarInfo::builder()
        .stamp(stamp)
        .frame_id("radar_front")
        .center_frequency("77GHz")
        .frequency_sweep("wide")
        .range_toggle("off")
        .detection_sensitivity("high")
        .cube(true)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "RadarInfo builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn track_builder_byte_parity_with_new() {
    let created = Time::new(1_699_999_950, 250_000_000);

    let via_new = Track::new("track_007", 42, created).expect("new() succeeds");
    let via_builder = Track::builder()
        .id("track_007")
        .lifetime(42)
        .created(created)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "Track builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn detect_box_builder_byte_parity_with_new() {
    let track_created = Time::new(1_700_000_000, 0);

    let via_new = DetectBox::new(
        0.512,
        0.489,
        0.123,
        0.234,
        "car",
        0.981,
        12.5,
        3.2,
        "t_17",
        8,
        track_created,
    )
    .expect("new() succeeds");
    let via_builder = DetectBox::builder()
        .center_x(0.512)
        .center_y(0.489)
        .width(0.123)
        .height(0.234)
        .label("car")
        .score(0.981)
        .distance(12.5)
        .speed(3.2)
        .track_id("t_17")
        .track_lifetime(8)
        .track_created(track_created)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "DetectBox builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn detect_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_100, 500_000_000);
    let input_ts = Time::new(1_700_000_100, 400_000_000);
    let model_ts = Time::new(0, 50_000_000);
    let output_ts = Time::new(1_700_000_100, 500_000_000);

    let boxes = [
        DetectBoxView {
            center_x: 0.2,
            center_y: 0.3,
            width: 0.1,
            height: 0.2,
            label: "car",
            score: 0.95,
            distance: 10.0,
            speed: 2.0,
            track_id: "t1",
            track_lifetime: 5,
            track_created: Time::new(1_700_000_095, 0),
        },
        DetectBoxView {
            center_x: 0.7,
            center_y: 0.6,
            width: 0.05,
            height: 0.12,
            label: "pedestrian",
            score: 0.82,
            distance: 7.5,
            speed: 1.1,
            track_id: "track_longer_id",
            track_lifetime: 12,
            track_created: Time::new(1_700_000_080, 0),
        },
    ];

    let via_new = Detect::new(stamp, "camera_front", input_ts, model_ts, output_ts, &boxes)
        .expect("new() succeeds");
    let via_builder = Detect::builder()
        .stamp(stamp)
        .frame_id("camera_front")
        .input_timestamp(input_ts)
        .model_time(model_ts)
        .output_time(output_ts)
        .boxes(&boxes)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "Detect builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn model_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_200, 0);
    let input_time = Duration::new(0, 1_000_000);
    let model_time = Duration::new(0, 5_000_000);
    let output_time = Duration::new(0, 500_000);
    let decode_time = Duration::new(0, 2_000_000);

    let boxes = [DetectBoxView {
        center_x: 0.5,
        center_y: 0.5,
        width: 0.1,
        height: 0.2,
        label: "car",
        score: 0.9,
        distance: 15.0,
        speed: 4.5,
        track_id: "t_model",
        track_lifetime: 3,
        track_created: Time::new(1_700_000_197, 0),
    }];

    let mask_bytes: Vec<u8> = (0..64u32).map(|i| (i & 0xff) as u8).collect();
    let masks = [MaskView {
        height: 8,
        width: 8,
        length: 64,
        encoding: "raw",
        mask: &mask_bytes,
        boxed: false,
    }];

    let via_new = Model::new(
        stamp,
        "model_node",
        input_time,
        model_time,
        output_time,
        decode_time,
        &boxes,
        &masks,
    )
    .expect("new() succeeds");
    let via_builder = Model::builder()
        .stamp(stamp)
        .frame_id("model_node")
        .input_time(input_time)
        .model_time(model_time)
        .output_time(output_time)
        .decode_time(decode_time)
        .boxes(&boxes)
        .masks(&masks)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "Model builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn model_info_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_300, 0);
    let input_shape: &[u32] = &[1, 3, 640, 640];
    let output_shape: &[u32] = &[1, 84, 8400];
    let labels: &[&str] = &["person", "bicycle", "car", "motorcycle"];

    let via_new = ModelInfo::new(
        stamp,
        "cam0",
        input_shape,
        8,
        output_shape,
        8,
        labels,
        "object_detection",
        "DeepViewRT",
        "yolov8n",
    )
    .expect("new() succeeds");
    let via_builder = ModelInfo::builder()
        .stamp(stamp)
        .frame_id("cam0")
        .input_shape(input_shape)
        .input_type(8)
        .output_shape(output_shape)
        .output_type(8)
        .labels(labels)
        .model_type("object_detection")
        .model_format("DeepViewRT")
        .model_name("yolov8n")
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "ModelInfo builder and new() must produce identical CDR bytes",
    );
}

#[test]
fn vibration_builder_byte_parity_with_new() {
    let stamp = Time::new(1_700_000_400, 750_000_000);
    let vib = Vector3 {
        x: 0.125,
        y: -0.375,
        z: 1.5,
    };
    let clipping: &[u32] = &[17, 42, 99, 128, 256];

    let via_new = Vibration::new(stamp, "imu_link", 1, 2, 10.0, 1000.0, vib, clipping)
        .expect("new() succeeds");
    let via_builder = Vibration::builder()
        .stamp(stamp)
        .frame_id("imu_link")
        .measurement_type(1)
        .unit(2)
        .band_lower_hz(10.0)
        .band_upper_hz(1000.0)
        .vibration(vib)
        .clipping(clipping)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "Vibration builder and new() must produce identical CDR bytes",
    );
}
