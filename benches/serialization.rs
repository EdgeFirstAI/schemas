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
use std::io::Cursor;

use edgefirst_schemas::builtin_interfaces::{Duration, Time};
use edgefirst_schemas::edgefirst_msgs::{DmaBuf, Mask, RadarCube};
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

/// Create a DmaBuf message representing a camera frame reference.
fn create_dmabuf(width: u32, height: u32, fourcc: u32) -> DmaBuf {
    let bytes_per_pixel = match fourcc {
        0x56595559 => 2, // YUYV
        0x3231564E => 1, // NV12 (1.5 bytes avg, but length is separate)
        _ => 3,          // RGB
    };
    DmaBuf {
        header: create_header(),
        pid: 12345,
        fd: 42,
        width,
        height,
        stride: width * bytes_per_pixel,
        fourcc,
        length: width * height * bytes_per_pixel,
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
/// Shape: [chirp_types, range_gates, rx_channels, doppler_bins]
fn create_radar_cube(shape: &[u16; 4], is_complex: bool) -> RadarCube {
    let mut rng = rand::rng();
    // For complex data, doppler dimension is doubled (real + imag interleaved)
    let total_elements: usize = shape.iter().map(|&x| x as usize).product();
    RadarCube {
        header: create_header(),
        timestamp: 1234567890123456,
        layout: vec![6, 1, 5, 2], // SEQUENCE, RANGE, RXCHANNEL, DOPPLER
        shape: shape.to_vec(),
        scales: vec![1.0, 0.117, 1.0, 0.156], // range: 0.117m/bin, speed: 0.156m/s/bin
        cube: (0..total_elements).map(|_| rng.random()).collect(),
        is_complex,
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

/// Create a compressed Mask with zstd-encoded data.
fn create_compressed_mask(width: u32, height: u32, channels: u32) -> Mask {
    let mut rng = rand::rng();
    let data_size = (width * height * channels) as usize;
    let raw_data: Vec<u8> = (0..data_size).map(|_| rng.random()).collect();

    // Compress with zstd (level 3 is a good balance)
    let compressed = zstd::stream::encode_all(Cursor::new(&raw_data), 3).unwrap();

    Mask {
        height,
        width,
        length: channels,
        encoding: "zstd".to_string(),
        mask: compressed,
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
        position: point,
        orientation: quat,
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
        translation: vec3,
        rotation: quat,
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
        linear: vec3,
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

    // SmartMicro DRVEGRD radar cube configurations
    // Shape: [chirp_types, range_gates, rx_channels, doppler_bins]
    // All cubes use complex i16 data (real+imag interleaved)
    //
    // DRVEGRD-169: 4D/UHD Corner Radar (77-81 GHz, 79 GHz typical)
    //   Designed for side/corner monitoring with wide FoV (±70° azimuth)
    //   - Ultra-Short: 0.1-9.5m, ≤0.15m resolution, ±60 km/h
    //   - Short: 0.2-19m, ≤0.3m resolution, ±340/+140 km/h
    //   - Medium: 0.6-56m, ≤0.6m resolution
    //   - Long: 1.3-130m, ≤1.3m resolution
    //
    // DRVEGRD-171: 4D/PXHD Front Radar (76-77 GHz)
    //   Designed for front long-range detection (±50° azimuth, 6TX/8RX)
    //   - Short: 0.2-40m, ≤0.4m resolution, ±400/+200 km/h
    //   - Medium: 0.5-100m, ≤1.0m resolution
    //   - Long: 1.2-240m, ≤2.4m resolution
    let shapes: [([u16; 4], &str, bool); 7] = [
        // DRVEGRD-169 Corner Radar configurations
        // Ultra-Short: high doppler for close-range maneuvering (parking, blind spot)
        ([4, 64, 12, 256], "DRVEGRD169_ultra_short", true),
        // Short: side monitoring, parking assist
        ([4, 128, 12, 128], "DRVEGRD169_short", true),
        // Medium: lane change assist, cross-traffic alert
        ([4, 192, 12, 96], "DRVEGRD169_medium", true),
        // Long: extended corner coverage
        ([4, 256, 12, 64], "DRVEGRD169_long", true),
        // DRVEGRD-171 Front Radar configurations
        // Short: emergency braking, pedestrian detection
        ([4, 160, 12, 128], "DRVEGRD171_short", true),
        // Medium: adaptive cruise control, typical front radar (~3 MB)
        ([4, 256, 12, 64], "DRVEGRD171_medium", true),
        // Long: highway ACC, long-range vehicle detection
        ([4, 512, 12, 32], "DRVEGRD171_long", true),
    ];

    for (shape, name, is_complex) in shapes {
        let cube = create_radar_cube(&shape, is_complex);
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

    // Segmentation mask sizes: square resolutions with 8 or 32 classes
    // 320x320: Common for lightweight models (MobileNet-based)
    // 640x640: Standard YOLO/detection input size
    // 1280x1280: High-resolution segmentation
    let mask_sizes = [
        ((320, 320, 8), "320x320_8class"),
        ((320, 320, 32), "320x320_32class"),
        ((640, 640, 8), "640x640_8class"),
        ((640, 640, 32), "640x640_32class"),
        ((1280, 1280, 8), "1280x1280_8class"),
        ((1280, 1280, 32), "1280x1280_32class"),
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
// BENCHMARK: CompressedMask (Heavy) - zstd compressed segmentation masks
// ============================================================================

fn bench_compressed_mask(c: &mut Criterion) {
    let mut group = c.benchmark_group("CompressedMask");

    // Same sizes as Mask but with zstd compression
    let mask_sizes = [
        ((320, 320, 8), "320x320_8class"),
        ((320, 320, 32), "320x320_32class"),
        ((640, 640, 8), "640x640_8class"),
        ((640, 640, 32), "640x640_32class"),
        ((1280, 1280, 8), "1280x1280_8class"),
        ((1280, 1280, 32), "1280x1280_32class"),
    ];

    for ((width, height, channels), name) in mask_sizes {
        let mask = create_compressed_mask(width, height, channels);
        let data_size = mask.mask.len(); // Compressed size
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

    // Standard image resolutions with various encodings
    // RGB8: 3 bytes per pixel
    // YUYV: 2 bytes per pixel (YUV422 packed)
    // NV12: 1.5 bytes per pixel (YUV420 semi-planar, stored as 1 byte luma + 0.5 chroma)
    let image_sizes = [
        // VGA resolution
        ((640, 480, "rgb8", 3), "VGA_rgb8"),
        ((640, 480, "yuyv", 2), "VGA_yuyv"),
        ((640, 480, "nv12", 3), "VGA_nv12"), // NV12 uses 1.5 bpp but we round to height*1.5
        // HD resolution
        ((1280, 720, "rgb8", 3), "HD_rgb8"),
        ((1280, 720, "yuyv", 2), "HD_yuyv"),
        ((1280, 720, "nv12", 3), "HD_nv12"),
        // FHD resolution
        ((1920, 1080, "rgb8", 3), "FHD_rgb8"),
        ((1920, 1080, "yuyv", 2), "FHD_yuyv"),
        ((1920, 1080, "nv12", 3), "FHD_nv12"),
    ];

    for ((width, height, encoding, bpp), name) in image_sizes {
        // NV12 is special: Y plane + UV plane at half resolution
        let data_height = if encoding == "nv12" {
            height + height / 2
        } else {
            height
        };
        let actual_bpp_for_step = if encoding == "nv12" { 1 } else { bpp };

        let image = create_image(width, data_height, encoding, actual_bpp_for_step);
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
// BENCHMARK: DmaBuf (Lightweight reference)
// ============================================================================

fn bench_dmabuf(c: &mut Criterion) {
    let mut group = c.benchmark_group("DmaBuf");

    // DmaBuf is a lightweight message - just metadata, no pixel data
    // Used as a reference to show overhead of heavy messages vs zero-copy
    let dmabuf_sizes = [
        ((640, 480, 0x56595559), "VGA_yuyv"),   // YUYV fourcc
        ((1280, 720, 0x56595559), "HD_yuyv"),   // YUYV fourcc
        ((1920, 1080, 0x56595559), "FHD_yuyv"), // YUYV fourcc
    ];

    for ((width, height, fourcc), name) in dmabuf_sizes {
        let dmabuf = create_dmabuf(width, height, fourcc);
        let bytes = serialize(&dmabuf).unwrap();

        // Throughput based on the referenced frame size, not message size
        let frame_size = (width * height * 2) as u64; // YUYV = 2 bpp
        group.throughput(Throughput::Bytes(frame_size));

        group.bench_with_input(BenchmarkId::new("serialize", name), &dmabuf, |b, d| {
            b.iter(|| serialize(black_box(d)))
        });

        group.bench_with_input(BenchmarkId::new("deserialize", name), &bytes, |b, data| {
            b.iter(|| deserialize::<DmaBuf>(black_box(data)))
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
    bench_dmabuf,
    bench_compressed_video,
    bench_radar_cube,
    bench_point_cloud,
    bench_mask,
    bench_compressed_mask,
    bench_image,
);

criterion_main!(benches);
