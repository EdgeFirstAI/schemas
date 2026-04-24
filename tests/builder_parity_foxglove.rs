// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Byte parity tests for foxglove_msgs builders vs legacy new() constructors.

#![allow(deprecated)]

use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::foxglove_msgs::{
    point_annotation_type, FoxgloveCircleAnnotations, FoxgloveColor, FoxgloveCompressedVideo,
    FoxgloveImageAnnotation, FoxglovePoint2, FoxglovePointAnnotation, FoxglovePointAnnotationView,
    FoxgloveTextAnnotation, FoxgloveTextAnnotationView,
};

#[test]
fn foxglove_compressed_video_builder_byte_parity_with_new() {
    let stamp = Time::new(1_717_000_000, 123_456_789);
    let frame_id = "front_camera";
    // Non-trivial H.264 Annex-B start-code + NAL header plus a few payload bytes.
    let data: Vec<u8> = vec![
        0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0xC0, 0x1E, 0xDA, 0x02, 0xD0, 0x40, 0xAB, 0xCD, 0xEF,
        0x10, 0x20, 0x30,
    ];
    let format = "h264";

    let legacy = FoxgloveCompressedVideo::new(stamp, frame_id, &data, format).unwrap();

    let built = FoxgloveCompressedVideo::builder()
        .stamp(stamp)
        .frame_id(frame_id)
        .data(&data)
        .format(format)
        .build()
        .unwrap();

    assert_eq!(
        legacy.as_cdr(),
        built.as_cdr(),
        "FoxgloveCompressedVideo builder must produce byte-identical CDR",
    );

    // Also verify encode_into_vec parity.
    let mut reused = Vec::new();
    FoxgloveCompressedVideo::builder()
        .stamp(stamp)
        .frame_id(frame_id)
        .data(&data)
        .format(format)
        .encode_into_vec(&mut reused)
        .unwrap();
    assert_eq!(legacy.as_cdr(), &reused[..]);
}

#[test]
fn foxglove_text_annotation_builder_byte_parity_with_new() {
    let timestamp = Time::new(42, 250_000_000);
    let position = FoxglovePoint2 { x: 128.5, y: 72.25 };
    let text = "Pedestrian 87% @ frame 321";
    let font_size = 16.0;
    let text_color = FoxgloveColor {
        r: 1.0,
        g: 1.0,
        b: 1.0,
        a: 1.0,
    };
    let background_color = FoxgloveColor {
        r: 0.0,
        g: 0.0,
        b: 0.0,
        a: 0.65,
    };

    let legacy = FoxgloveTextAnnotation::new(
        timestamp,
        position,
        text,
        font_size,
        text_color,
        background_color,
    )
    .unwrap();

    let built = FoxgloveTextAnnotation::builder()
        .timestamp(timestamp)
        .position(position)
        .text(text)
        .font_size(font_size)
        .text_color(text_color)
        .background_color(background_color)
        .build()
        .unwrap();

    assert_eq!(
        legacy.as_cdr(),
        built.as_cdr(),
        "FoxgloveTextAnnotation builder must produce byte-identical CDR",
    );
}

#[test]
fn foxglove_point_annotation_builder_byte_parity_with_new() {
    let timestamp = Time::new(1234, 500_000_000);
    let type_ = point_annotation_type::LINE_STRIP;
    let points = [
        FoxglovePoint2 { x: 10.0, y: 20.0 },
        FoxglovePoint2 { x: 30.5, y: 40.25 },
        FoxglovePoint2 { x: 100.0, y: 200.0 },
        FoxglovePoint2 {
            x: -15.75,
            y: 95.125,
        },
    ];
    let outline_color = FoxgloveColor {
        r: 0.1,
        g: 0.2,
        b: 0.9,
        a: 1.0,
    };
    let outline_colors = [
        FoxgloveColor {
            r: 1.0,
            g: 0.0,
            b: 0.0,
            a: 1.0,
        },
        FoxgloveColor {
            r: 0.0,
            g: 1.0,
            b: 0.0,
            a: 1.0,
        },
        FoxgloveColor {
            r: 0.0,
            g: 0.0,
            b: 1.0,
            a: 1.0,
        },
    ];
    let fill_color = FoxgloveColor {
        r: 0.3,
        g: 0.3,
        b: 0.3,
        a: 0.5,
    };
    let thickness = 2.75;

    let legacy = FoxglovePointAnnotation::new(
        timestamp,
        type_,
        &points,
        outline_color,
        &outline_colors,
        fill_color,
        thickness,
    )
    .unwrap();

    let built = FoxglovePointAnnotation::builder()
        .timestamp(timestamp)
        .type_(type_)
        .points(&points)
        .outline_color(outline_color)
        .outline_colors(&outline_colors)
        .fill_color(fill_color)
        .thickness(thickness)
        .build()
        .unwrap();

    assert_eq!(
        legacy.as_cdr(),
        built.as_cdr(),
        "FoxglovePointAnnotation builder must produce byte-identical CDR",
    );
}

#[test]
fn foxglove_image_annotation_builder_byte_parity_with_new() {
    let circles = [
        FoxgloveCircleAnnotations {
            timestamp: Time::new(10, 0),
            position: FoxglovePoint2 { x: 160.0, y: 120.0 },
            diameter: 25.0,
            thickness: 1.5,
            fill_color: FoxgloveColor {
                r: 1.0,
                g: 0.0,
                b: 0.0,
                a: 0.5,
            },
            outline_color: FoxgloveColor {
                r: 1.0,
                g: 1.0,
                b: 1.0,
                a: 1.0,
            },
        },
        FoxgloveCircleAnnotations {
            timestamp: Time::new(10, 16_666_666),
            position: FoxglovePoint2 { x: 480.0, y: 320.0 },
            diameter: 40.0,
            thickness: 2.0,
            fill_color: FoxgloveColor {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 0.4,
            },
            outline_color: FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 1.0,
            },
        },
    ];

    let pts_ann = FoxglovePointAnnotationView {
        timestamp: Time::new(11, 0),
        type_: point_annotation_type::LINE_LOOP,
        points: vec![
            FoxglovePoint2 { x: 0.0, y: 0.0 },
            FoxglovePoint2 { x: 100.0, y: 0.0 },
            FoxglovePoint2 { x: 100.0, y: 100.0 },
            FoxglovePoint2 { x: 0.0, y: 100.0 },
        ],
        outline_color: FoxgloveColor {
            r: 0.0,
            g: 1.0,
            b: 0.0,
            a: 1.0,
        },
        outline_colors: vec![FoxgloveColor {
            r: 1.0,
            g: 1.0,
            b: 0.0,
            a: 1.0,
        }],
        fill_color: FoxgloveColor {
            r: 0.0,
            g: 0.5,
            b: 0.0,
            a: 0.3,
        },
        thickness: 3.0,
    };
    let points = [pts_ann];

    let text_ann = FoxgloveTextAnnotationView {
        timestamp: Time::new(12, 0),
        position: FoxglovePoint2 { x: 20.0, y: 20.0 },
        text: "vehicle 92%",
        font_size: 14.0,
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
    };
    let texts = [text_ann];

    let legacy = FoxgloveImageAnnotation::new(&circles, &points, &texts).unwrap();

    let built = FoxgloveImageAnnotation::builder()
        .circles(&circles)
        .points(&points)
        .texts(&texts)
        .build()
        .unwrap();

    assert_eq!(
        legacy.as_cdr(),
        built.as_cdr(),
        "FoxgloveImageAnnotation builder must produce byte-identical CDR",
    );

    // encode_into_slice round-trip parity.
    let need = legacy.as_cdr().len();
    let mut slot = vec![0u8; need];
    let written = FoxgloveImageAnnotation::builder()
        .circles(&circles)
        .points(&points)
        .texts(&texts)
        .encode_into_slice(&mut slot)
        .unwrap();
    assert_eq!(written, need);
    assert_eq!(legacy.as_cdr(), &slot[..]);
}
