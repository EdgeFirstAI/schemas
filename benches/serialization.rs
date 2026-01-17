// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! Comprehensive performance benchmarks for CDR serialization/deserialization.
//!
//! This benchmark suite measures serialization performance for EdgeFirst schemas,
//! focusing on heavy message types used in production perception pipelines.
//!
//! Run all benchmarks: `cargo bench`
//! Run specific group: `cargo bench -- "RadarCube"`

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use rand::Rng;
use std::hint::black_box;

use edgefirst_schemas::builtin_interfaces::{Duration, Time};
use edgefirst_schemas::edgefirst_msgs::{Mask, RadarCube};
use edgefirst_schemas::foxglove_msgs::FoxgloveCompressedVideo;
use edgefirst_schemas::geometry_msgs::{
    Point, Point32, Pose, Pose2D, Quaternion, Transform, Twist, Vector3,
};
use edgefirst_schemas::sensor_msgs::{point_field, Image, PointCloud2, PointField};
use edgefirst_schemas::serde_cdr::{deserialize, serialize};
use edgefirst_schemas::std_msgs::{ColorRGBA, Header};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

fn create_header() -> Header {
    Header {
        stamp: Time {
            sec: 1234567890,
            nanosec: 123456789,
        },
        frame_id: "sensor_frame".to_string(),
    }
}

/// Create a FoxgloveCompressedVideo with random data simulating H.264 NAL units.
fn create_compressed_video(data_size: usize) -> FoxgloveCompressedVideo {
    let mut rng = rand::rng();
    FoxgloveCompressedVideo {
        header: create_header(),
        data: (0..data_size).map(|_| rng.random()).collect(),
        format: "h264".to_string(),
    }
}

/// Create a RadarCube with random i16 data simulating real radar returns.
fn create_radar_cube(shape: &[u16; 4]) -> RadarCube {
    let mut rng = rand::rng();
    let total_elements: usize = shape.iter().map(|&x| x as usize).product();
    RadarCube {
        header: create_header(),
        timestamp: 1234567890123456,
        layout: vec![6, 1, 5, 2], // SEQUENCE, RANGE, RXCHANNEL, DOPPLER
        shape: shape.to_vec(),
        scales: vec![1.0, 2.5, 1.0, 0.5], // meters, meters, unitless, m/s
        cube: (0..total_elements).map(|_| rng.random()).collect(),
        is_complex: false,
    }
}

/// Create a PointCloud2 with standard LiDAR point format (x, y, z, intensity).
fn create_point_cloud(num_points: usize) -> PointCloud2 {
    let mut rng = rand::rng();
    let point_step = 16; // 4 fields * 4 bytes (FLOAT32)
    let fields = vec![
        PointField {
            name: "x".to_string(),
            offset: 0,
            datatype: point_field::FLOAT32,
            count: 1,
        },
        PointField {
            name: "y".to_string(),
            offset: 4,
            datatype: point_field::FLOAT32,
            count: 1,
        },
        PointField {
            name: "z".to_string(),
            offset: 8,
            datatype: point_field::FLOAT32,
            count: 1,
        },
        PointField {
            name: "intensity".to_string(),
            offset: 12,
            datatype: point_field::FLOAT32,
            count: 1,
        },
    ];

    PointCloud2 {
        header: create_header(),
        height: 1,
        width: num_points as u32,
        fields,
        is_bigendian: false,
        point_step,
        row_step: point_step * num_points as u32,
        data: (0..point_step as usize * num_points)
            .map(|_| rng.random())
            .collect(),
        is_dense: true,
    }
}

/// Create a Mask with random segmentation data.
fn create_mask(width: u32, height: u32, channels: u32) -> Mask {
    let mut rng = rand::rng();
    let data_size = (width * height * channels) as usize;
    Mask {
        height,
        width,
        length: channels,
        encoding: String::new(), // No compression
        mask: (0..data_size).map(|_| rng.random()).collect(),
        boxed: false,
    }
}

/// Create an Image with random pixel data.
fn create_image(width: u32, height: u32, encoding: &str, bytes_per_pixel: u32) -> Image {
    let mut rng = rand::rng();
    let step = width * bytes_per_pixel;
    let data_size = (step * height) as usize;
    Image {
        header: create_header(),
        height,
        width,
        encoding: encoding.to_string(),
        is_bigendian: 0,
        step,
        data: (0..data_size).map(|_| rng.random()).collect(),
    }
}

// ============================================================================
// BENCHMARK: builtin_interfaces
// ============================================================================

fn bench_builtin_interfaces(c: &mut Criterion) {
    let mut group = c.benchmark_group("builtin_interfaces");

    // Time
    let time = Time {
        sec: 1234567890,
        nanosec: 123456789,
    };
    let time_bytes = serialize(&time).unwrap();
    group.throughput(Throughput::Bytes(time_bytes.len() as u64));

    group.bench_function("Time/serialize", |b| b.iter(|| serialize(black_box(&time))));
    group.bench_function("Time/deserialize", |b| {
        b.iter(|| deserialize::<Time>(black_box(&time_bytes)))
    });

    // Duration
    let duration = Duration {
        sec: 60,
        nanosec: 500000000,
    };
    let duration_bytes = serialize(&duration).unwrap();

    group.bench_function("Duration/serialize", |b| {
        b.iter(|| serialize(black_box(&duration)))
    });
    group.bench_function("Duration/deserialize", |b| {
        b.iter(|| deserialize::<Duration>(black_box(&duration_bytes)))
    });

    group.finish();
}

// ============================================================================
// BENCHMARK: std_msgs
// ============================================================================

fn bench_std_msgs(c: &mut Criterion) {
    let mut group = c.benchmark_group("std_msgs");

    // Header
    let header = create_header();
    let header_bytes = serialize(&header).unwrap();
    group.throughput(Throughput::Bytes(header_bytes.len() as u64));

    group.bench_function("Header/serialize", |b| {
        b.iter(|| serialize(black_box(&header)))
    });
    group.bench_function("Header/deserialize", |b| {
        b.iter(|| deserialize::<Header>(black_box(&header_bytes)))
    });

    // ColorRGBA
    let color = ColorRGBA {
        r: 1.0,
        g: 0.5,
        b: 0.0,
        a: 1.0,
    };
    let color_bytes = serialize(&color).unwrap();

    group.bench_function("ColorRGBA/serialize", |b| {
        b.iter(|| serialize(black_box(&color)))
    });
    group.bench_function("ColorRGBA/deserialize", |b| {
        b.iter(|| deserialize::<ColorRGBA>(black_box(&color_bytes)))
    });

    group.finish();
}

// ============================================================================
// BENCHMARK: geometry_msgs
// ============================================================================

fn bench_geometry_msgs(c: &mut Criterion) {
    let mut group = c.benchmark_group("geometry_msgs");

    // Vector3
    let vec3 = Vector3 {
        x: 1.5,
        y: 2.5,
        z: 3.5,
    };
    let vec3_bytes = serialize(&vec3).unwrap();
    group.bench_function("Vector3/serialize", |b| {
        b.iter(|| serialize(black_box(&vec3)))
    });
    group.bench_function("Vector3/deserialize", |b| {
        b.iter(|| deserialize::<Vector3>(black_box(&vec3_bytes)))
    });

    // Point
    let point = Point {
        x: 10.0,
        y: 20.0,
        z: 30.0,
    };
    let point_bytes = serialize(&point).unwrap();
    group.bench_function("Point/serialize", |b| {
        b.iter(|| serialize(black_box(&point)))
    });
    group.bench_function("Point/deserialize", |b| {
        b.iter(|| deserialize::<Point>(black_box(&point_bytes)))
    });

    // Point32
    let point32 = Point32 {
        x: 1.0,
        y: 2.0,
        z: 3.0,
    };
    let point32_bytes = serialize(&point32).unwrap();
    group.bench_function("Point32/serialize", |b| {
        b.iter(|| serialize(black_box(&point32)))
    });
    group.bench_function("Point32/deserialize", |b| {
        b.iter(|| deserialize::<Point32>(black_box(&point32_bytes)))
    });

    // Quaternion
    let quat = Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.707,
        w: 0.707,
    };
    let quat_bytes = serialize(&quat).unwrap();
    group.bench_function("Quaternion/serialize", |b| {
        b.iter(|| serialize(black_box(&quat)))
    });
    group.bench_function("Quaternion/deserialize", |b| {
        b.iter(|| deserialize::<Quaternion>(black_box(&quat_bytes)))
    });

    // Pose
    let pose = Pose {
        position: point.clone(),
        orientation: quat.clone(),
    };
    let pose_bytes = serialize(&pose).unwrap();
    group.bench_function("Pose/serialize", |b| b.iter(|| serialize(black_box(&pose))));
    group.bench_function("Pose/deserialize", |b| {
        b.iter(|| deserialize::<Pose>(black_box(&pose_bytes)))
    });

    // Pose2D
    let pose2d = Pose2D {
        x: 10.0,
        y: 20.0,
        theta: 1.57,
    };
    let pose2d_bytes = serialize(&pose2d).unwrap();
    group.bench_function("Pose2D/serialize", |b| {
        b.iter(|| serialize(black_box(&pose2d)))
    });
    group.bench_function("Pose2D/deserialize", |b| {
        b.iter(|| deserialize::<Pose2D>(black_box(&pose2d_bytes)))
    });

    // Transform
    let transform = Transform {
        translation: vec3.clone(),
        rotation: quat.clone(),
    };
    let transform_bytes = serialize(&transform).unwrap();
    group.bench_function("Transform/serialize", |b| {
        b.iter(|| serialize(black_box(&transform)))
    });
    group.bench_function("Transform/deserialize", |b| {
        b.iter(|| deserialize::<Transform>(black_box(&transform_bytes)))
    });

    // Twist
    let twist = Twist {
        linear: vec3.clone(),
        angular: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.5,
        },
    };
    let twist_bytes = serialize(&twist).unwrap();
    group.bench_function("Twist/serialize", |b| {
        b.iter(|| serialize(black_box(&twist)))
    });
    group.bench_function("Twist/deserialize", |b| {
        b.iter(|| deserialize::<Twist>(black_box(&twist_bytes)))
    });

    group.finish();
}

// ============================================================================
// BENCHMARK: FoxgloveCompressedVideo (Heavy)
// ============================================================================

fn bench_compressed_video(c: &mut Criterion) {
    let mut group = c.benchmark_group("FoxgloveCompressedVideo");

    // Test sizes: 10KB, 100KB, 500KB, 1MB (simulating H.264 frame sizes)
    let sizes = [
        (10_000, "10KB"),
        (100_000, "100KB"),
        (500_000, "500KB"),
        (1_000_000, "1MB"),
    ];

    for (size, name) in sizes {
        let video = create_compressed_video(size);
        let bytes = serialize(&video).unwrap();

        group.throughput(Throughput::Bytes(size as u64));

        group.bench_with_input(BenchmarkId::new("serialize", name), &video, |b, v| {
            b.iter(|| serialize(black_box(v)))
        });

        group.bench_with_input(BenchmarkId::new("deserialize", name), &bytes, |b, data| {
            b.iter(|| deserialize::<FoxgloveCompressedVideo>(black_box(data)))
        });
    }

    group.finish();
}

// ============================================================================
// BENCHMARK: RadarCube (Heavy)
// ============================================================================

fn bench_radar_cube(c: &mut Criterion) {
    let mut group = c.benchmark_group("RadarCube");

    // Shapes from Raivin radar: [SEQ, RANGE, RXCHANNEL, DOPPLER]
    // Small: 65,536 elements (128 KB)
    // Medium: 1,048,576 elements (2 MB)
    // Large: 12,582,912 elements (24 MB)
    let shapes: [([u16; 4], &str); 3] = [
        ([8, 64, 4, 32], "small_128KB"),
        ([16, 128, 8, 64], "medium_2MB"),
        ([32, 256, 12, 128], "large_24MB"),
    ];

    for (shape, name) in shapes {
        let cube = create_radar_cube(&shape);
        let data_size = cube.cube.len() * 2; // i16 = 2 bytes
        let bytes = serialize(&cube).unwrap();

        group.throughput(Throughput::Bytes(data_size as u64));

        group.bench_with_input(BenchmarkId::new("serialize", name), &cube, |b, c| {
            b.iter(|| serialize(black_box(c)))
        });

        group.bench_with_input(BenchmarkId::new("deserialize", name), &bytes, |b, data| {
            b.iter(|| deserialize::<RadarCube>(black_box(data)))
        });
    }

    // Also test complex radar cubes (is_complex: true)
    let mut complex_cube = create_radar_cube(&[8, 64, 4, 32]);
    complex_cube.is_complex = true;
    let complex_bytes = serialize(&complex_cube).unwrap();

    group.bench_with_input(
        BenchmarkId::new("serialize", "small_complex"),
        &complex_cube,
        |b, c| b.iter(|| serialize(black_box(c))),
    );

    group.bench_with_input(
        BenchmarkId::new("deserialize", "small_complex"),
        &complex_bytes,
        |b, data| b.iter(|| deserialize::<RadarCube>(black_box(data))),
    );

    group.finish();
}

// ============================================================================
// BENCHMARK: PointCloud2 (Heavy)
// ============================================================================

fn bench_point_cloud(c: &mut Criterion) {
    let mut group = c.benchmark_group("PointCloud2");

    // Point counts typical for LiDAR/radar
    // Sparse: 1,000 points (16 KB)
    // Medium: 10,000 points (160 KB)
    // Dense: 65,536 points (1 MB)
    // Very Dense: 131,072 points (2 MB)
    let point_counts = [
        (1_000, "sparse_1K"),
        (10_000, "medium_10K"),
        (65_536, "dense_65K"),
        (131_072, "very_dense_131K"),
    ];

    for (num_points, name) in point_counts {
        let cloud = create_point_cloud(num_points);
        let data_size = cloud.data.len();
        let bytes = serialize(&cloud).unwrap();

        group.throughput(Throughput::Bytes(data_size as u64));

        group.bench_with_input(BenchmarkId::new("serialize", name), &cloud, |b, c| {
            b.iter(|| serialize(black_box(c)))
        });

        group.bench_with_input(BenchmarkId::new("deserialize", name), &bytes, |b, data| {
            b.iter(|| deserialize::<PointCloud2>(black_box(data)))
        });
    }

    group.finish();
}

// ============================================================================
// BENCHMARK: Mask (Heavy)
// ============================================================================

fn bench_mask(c: &mut Criterion) {
    let mut group = c.benchmark_group("Mask");

    // Segmentation mask sizes
    // Small: 256x256x1 (64 KB)
    // Medium: 640x480x1 (300 KB)
    // Large: 1920x1080x1 (2 MB)
    // Multi-class: 640x480x20 (6 MB)
    let mask_sizes = [
        ((256, 256, 1), "small_256x256"),
        ((640, 480, 1), "medium_640x480"),
        ((1920, 1080, 1), "large_1920x1080"),
        ((640, 480, 20), "multiclass_640x480x20"),
    ];

    for ((width, height, channels), name) in mask_sizes {
        let mask = create_mask(width, height, channels);
        let data_size = mask.mask.len();
        let bytes = serialize(&mask).unwrap();

        group.throughput(Throughput::Bytes(data_size as u64));

        group.bench_with_input(BenchmarkId::new("serialize", name), &mask, |b, m| {
            b.iter(|| serialize(black_box(m)))
        });

        group.bench_with_input(BenchmarkId::new("deserialize", name), &bytes, |b, data| {
            b.iter(|| deserialize::<Mask>(black_box(data)))
        });
    }

    group.finish();
}

// ============================================================================
// BENCHMARK: Image (Heavy)
// ============================================================================

fn bench_image(c: &mut Criterion) {
    let mut group = c.benchmark_group("Image");

    // Standard image resolutions
    // VGA RGB: 640x480x3 (900 KB)
    // HD RGB: 1280x720x3 (2.7 MB)
    // FHD RGB: 1920x1080x3 (6.2 MB)
    // YUV422: 1920x1080x2 (4.1 MB)
    let image_sizes = [
        ((640, 480, "rgb8", 3), "VGA_rgb8"),
        ((1280, 720, "rgb8", 3), "HD_rgb8"),
        ((1920, 1080, "rgb8", 3), "FHD_rgb8"),
        ((1920, 1080, "yuyv", 2), "FHD_yuyv"),
    ];

    for ((width, height, encoding, bpp), name) in image_sizes {
        let image = create_image(width, height, encoding, bpp);
        let data_size = image.data.len();
        let bytes = serialize(&image).unwrap();

        group.throughput(Throughput::Bytes(data_size as u64));

        group.bench_with_input(BenchmarkId::new("serialize", name), &image, |b, img| {
            b.iter(|| serialize(black_box(img)))
        });

        group.bench_with_input(BenchmarkId::new("deserialize", name), &bytes, |b, data| {
            b.iter(|| deserialize::<Image>(black_box(data)))
        });
    }

    group.finish();
}

// ============================================================================
// CRITERION GROUPS
// ============================================================================

criterion_group!(
    benches,
    bench_builtin_interfaces,
    bench_std_msgs,
    bench_geometry_msgs,
    bench_compressed_video,
    bench_radar_cube,
    bench_point_cloud,
    bench_mask,
    bench_image,
);

criterion_main!(benches);
