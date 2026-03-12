// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! Comprehensive performance benchmarks for CDR serialization/deserialization.
//!
//! This benchmark suite measures serialization performance for EdgeFirst schemas,
//! focusing on heavy message types used in production perception pipelines.
//!
//! Run all benchmarks: `cargo bench`
//! Run specific group: `cargo bench -- "RadarCube"`
//! Fast mode for CI: `BENCH_FAST=1 cargo bench`

use criterion::{criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use rand::Rng;
use std::hint::black_box;

use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::cdr;
use edgefirst_schemas::edgefirst_msgs::{DmaBuffer, Mask, RadarCube};
use edgefirst_schemas::foxglove_msgs::FoxgloveCompressedVideo;
use edgefirst_schemas::geometry_msgs::{Point, Pose, Quaternion, Vector3};
use edgefirst_schemas::sensor_msgs::Image;
use edgefirst_schemas::std_msgs::Header;

/// Check if fast benchmark mode is enabled via BENCH_FAST=1 environment variable.
/// Fast mode runs fewer benchmark variants for quicker CI feedback (~5-10 min vs ~20 min).
fn is_fast_mode() -> bool {
    std::env::var("BENCH_FAST").is_ok_and(|v| v == "1" || v.eq_ignore_ascii_case("true"))
}

// Image benchmark configuration type alias to avoid complex type warnings
type ImageConfig<'a> = ((u32, u32, &'a str, u32), &'a str);

// ============================================================================
// BENCHMARK: CdrFixed types (Time, Vector3, Pose)
// ============================================================================

fn bench_fixed_types(c: &mut Criterion) {
    let mut group = c.benchmark_group("CdrFixed");

    // Time
    let time = Time {
        sec: 1234567890,
        nanosec: 123456789,
    };
    let time_bytes = cdr::encode_fixed(&time).unwrap();
    group.bench_function("Time/encode", |b| {
        b.iter(|| cdr::encode_fixed(black_box(&time)).unwrap())
    });
    group.bench_function("Time/decode", |b| {
        b.iter(|| cdr::decode_fixed::<Time>(black_box(&time_bytes)))
    });

    // Vector3
    let vec3 = Vector3 {
        x: 1.5,
        y: 2.5,
        z: 3.5,
    };
    let vec3_bytes = cdr::encode_fixed(&vec3).unwrap();
    group.bench_function("Vector3/encode", |b| {
        b.iter(|| cdr::encode_fixed(black_box(&vec3)).unwrap())
    });
    group.bench_function("Vector3/decode", |b| {
        b.iter(|| cdr::decode_fixed::<Vector3>(black_box(&vec3_bytes)))
    });

    // Pose
    let pose = Pose {
        position: Point {
            x: 10.0,
            y: 20.0,
            z: 30.0,
        },
        orientation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.707,
            w: 0.707,
        },
    };
    let pose_bytes = cdr::encode_fixed(&pose).unwrap();
    group.bench_function("Pose/encode", |b| {
        b.iter(|| cdr::encode_fixed(black_box(&pose)).unwrap())
    });
    group.bench_function("Pose/decode", |b| {
        b.iter(|| cdr::decode_fixed::<Pose>(black_box(&pose_bytes)))
    });

    group.finish();
}

// ============================================================================
// BENCHMARK: Header (buffer-backed, 1 offset)
// ============================================================================

fn bench_header(c: &mut Criterion) {
    let mut group = c.benchmark_group("Header");

    let stamp = Time {
        sec: 1234567890,
        nanosec: 123456789,
    };
    let hdr = Header::new(stamp, "sensor_frame").unwrap();
    let bytes = hdr.to_cdr();
    group.throughput(Throughput::Bytes(bytes.len() as u64));

    group.bench_function("new", |b| {
        b.iter(|| Header::new(black_box(stamp), black_box("sensor_frame")).unwrap())
    });
    group.bench_function("from_cdr", |b| {
        b.iter(|| Header::from_cdr(black_box(bytes.clone())))
    });

    group.finish();
}

// ============================================================================
// BENCHMARK: Image (buffer-backed, 3 offsets)
// ============================================================================

fn bench_image(c: &mut Criterion) {
    let mut group = c.benchmark_group("Image");

    let stamp = Time {
        sec: 1234567890,
        nanosec: 123456789,
    };
    let fast_sizes: &[ImageConfig<'_>] = &[
        ((1280, 720, "rgb8", 3), "HD_rgb8"),
        ((1920, 1080, "yuyv", 2), "FHD_yuyv"),
    ];
    let all_sizes: &[ImageConfig<'_>] = &[
        ((640, 480, "rgb8", 3), "VGA_rgb8"),
        ((1280, 720, "rgb8", 3), "HD_rgb8"),
        ((1920, 1080, "yuyv", 2), "FHD_yuyv"),
    ];
    let sizes = if is_fast_mode() {
        fast_sizes
    } else {
        all_sizes
    };

    for &((width, height, encoding, bpp), name) in sizes {
        let mut rng = rand::rng();
        let actual_bpp = if encoding == "nv12" { 1 } else { bpp };
        let actual_h = if encoding == "nv12" {
            height + height / 2
        } else {
            height
        };
        let step = width * actual_bpp;
        let data: Vec<u8> = (0..(step * actual_h) as usize)
            .map(|_| rng.random())
            .collect();
        let data_size = data.len();

        let img = Image::new(
            stamp,
            "sensor_frame",
            actual_h,
            width,
            encoding,
            0,
            step,
            &data,
        )
        .unwrap();
        let bytes = img.to_cdr();

        group.throughput(Throughput::Bytes(data_size as u64));

        group.bench_with_input(BenchmarkId::new("new", name), &data, |b, d| {
            b.iter(|| {
                Image::new(
                    black_box(stamp),
                    "sensor_frame",
                    actual_h,
                    width,
                    encoding,
                    0,
                    step,
                    black_box(d),
                )
                .unwrap()
            })
        });

        group.bench_with_input(BenchmarkId::new("from_cdr", name), &bytes, |b, cdr| {
            b.iter(|| Image::from_cdr(black_box(cdr.clone())))
        });

        // from_cdr with borrow (zero-copy, no clone)
        group.bench_with_input(
            BenchmarkId::new("from_cdr_borrow", name),
            &bytes,
            |b, cdr| b.iter(|| Image::from_cdr(black_box(cdr.as_slice()))),
        );
    }

    group.finish();
}

// ============================================================================
// BENCHMARK: FoxgloveCompressedVideo (buffer-backed)
// ============================================================================

fn bench_compressed_video(c: &mut Criterion) {
    let mut group = c.benchmark_group("FoxgloveCompressedVideo");

    let stamp = Time {
        sec: 1234567890,
        nanosec: 123456789,
    };
    let sizes: &[(usize, &str)] = &[(100_000, "100KB"), (500_000, "500KB")];

    for &(size, name) in sizes {
        let mut rng = rand::rng();
        let data: Vec<u8> = (0..size).map(|_| rng.random()).collect();

        let video = FoxgloveCompressedVideo::new(stamp, "sensor_frame", &data, "h264").unwrap();
        let bytes = video.to_cdr();

        group.throughput(Throughput::Bytes(size as u64));

        group.bench_with_input(BenchmarkId::new("new", name), &data, |b, d| {
            b.iter(|| {
                FoxgloveCompressedVideo::new(black_box(stamp), "sensor_frame", black_box(d), "h264")
                    .unwrap()
            })
        });

        group.bench_with_input(
            BenchmarkId::new("from_cdr_borrow", name),
            &bytes,
            |b, cdr| b.iter(|| FoxgloveCompressedVideo::from_cdr(black_box(cdr.as_slice()))),
        );
    }

    group.finish();
}

// ============================================================================
// BENCHMARK: RadarCube (buffer-backed, 5 offsets)
// ============================================================================

fn bench_radar_cube(c: &mut Criterion) {
    let mut group = c.benchmark_group("RadarCube");

    let stamp = Time {
        sec: 1234567890,
        nanosec: 123456789,
    };
    let shapes: &[([u16; 4], &str)] = &[
        ([4, 128, 12, 128], "DRVEGRD169_short"),
        ([4, 512, 12, 32], "DRVEGRD171_long"),
    ];

    for &(ref shape, name) in shapes {
        let mut rng = rand::rng();
        let total_elements: usize = shape.iter().map(|&x| x as usize).product();
        let cube_data: Vec<i16> = (0..total_elements).map(|_| rng.random()).collect();
        let layout: Vec<u8> = vec![6, 1, 5, 2];
        let shape_vec: Vec<u16> = shape.to_vec();
        let scales: Vec<f32> = vec![1.0, 0.117, 1.0, 0.156];
        let data_size = cube_data.len() * 2;

        let cube = RadarCube::new(
            stamp,
            "radar_frame",
            1234567890123456u64,
            &layout,
            &shape_vec,
            &scales,
            &cube_data,
            true,
        )
        .unwrap();
        let bytes = cube.to_cdr();

        group.throughput(Throughput::Bytes(data_size as u64));

        group.bench_with_input(BenchmarkId::new("new", name), &cube_data, |b, cd| {
            b.iter(|| {
                RadarCube::new(
                    black_box(stamp),
                    "radar_frame",
                    1234567890123456u64,
                    &layout,
                    &shape_vec,
                    &scales,
                    black_box(cd),
                    true,
                )
                .unwrap()
            })
        });

        group.bench_with_input(
            BenchmarkId::new("from_cdr_borrow", name),
            &bytes,
            |b, cdr| b.iter(|| RadarCube::from_cdr(black_box(cdr.as_slice()))),
        );
    }

    group.finish();
}

// ============================================================================
// BENCHMARK: Mask (buffer-backed, 2 offsets)
// ============================================================================

fn bench_mask(c: &mut Criterion) {
    let mut group = c.benchmark_group("Mask");

    let sizes: &[((u32, u32, u32), &str)] = &[
        ((640, 640, 8), "640x640_8class"),
        ((1280, 1280, 32), "1280x1280_32class"),
    ];

    for &((width, height, channels), name) in sizes {
        let mut rng = rand::rng();
        let data_size = (width * height * channels) as usize;
        let mask_data: Vec<u8> = (0..data_size).map(|_| rng.random()).collect();

        let mask = Mask::new(height, width, channels, "", &mask_data, false).unwrap();
        let bytes = mask.to_cdr();

        group.throughput(Throughput::Bytes(data_size as u64));

        group.bench_with_input(BenchmarkId::new("new", name), &mask_data, |b, d| {
            b.iter(|| Mask::new(height, width, channels, "", black_box(d), false).unwrap())
        });

        group.bench_with_input(
            BenchmarkId::new("from_cdr_borrow", name),
            &bytes,
            |b, cdr| b.iter(|| Mask::from_cdr(black_box(cdr.as_slice()))),
        );
    }

    group.finish();
}

// ============================================================================
// BENCHMARK: DmaBuffer (buffer-backed, 1 offset, lightweight)
// ============================================================================

fn bench_dmabuf(c: &mut Criterion) {
    let mut group = c.benchmark_group("DmaBuffer");

    let stamp = Time {
        sec: 1234567890,
        nanosec: 123456789,
    };
    let dmabuf = DmaBuffer::new(
        stamp,
        "sensor_frame",
        12345,
        42,
        1280,
        720,
        2560,
        0x56595559,
        1843200,
    )
    .unwrap();
    let bytes = dmabuf.to_cdr();
    let frame_size = 1280u64 * 720 * 2;
    group.throughput(Throughput::Bytes(frame_size));

    group.bench_function("new", |b| {
        b.iter(|| {
            DmaBuffer::new(
                black_box(stamp),
                "sensor_frame",
                12345,
                42,
                1280,
                720,
                2560,
                0x56595559,
                1843200,
            )
            .unwrap()
        })
    });
    group.bench_function("from_cdr_borrow", |b| {
        b.iter(|| DmaBuffer::from_cdr(black_box(bytes.as_slice())))
    });

    group.finish();
}

// ============================================================================
// POINTCLOUD ACCESS BENCHMARKS
// ============================================================================

use edgefirst_schemas::define_point;
use edgefirst_schemas::sensor_msgs::pointcloud::{DynPointCloud, PointCloud};
use edgefirst_schemas::sensor_msgs::{PointCloud2, PointFieldView};

define_point! {
    struct BenchXyz {
        x: f32 => 0,
        y: f32 => 4,
        z: f32 => 8,
    }
}

/// Build a 1024-point xyz cloud for benchmarking.
fn make_bench_cloud() -> Vec<u8> {
    let fields = [
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
    let n = 1024u32;
    let point_step = 12u32;
    let mut data = vec![0u8; (point_step * n) as usize];
    for i in 0..n {
        let base = (i * point_step) as usize;
        data[base..base + 4].copy_from_slice(&(i as f32).to_le_bytes());
        data[base + 4..base + 8].copy_from_slice(&(i as f32 * 0.5).to_le_bytes());
        data[base + 8..base + 12].copy_from_slice(&(i as f32 * 0.1).to_le_bytes());
    }
    let pc = PointCloud2::new(
        Time::new(0, 0),
        "lidar",
        1,
        n,
        &fields,
        false,
        point_step,
        point_step * n,
        &data,
        true,
    )
    .unwrap();
    pc.to_cdr()
}

fn bench_pointcloud(c: &mut Criterion) {
    let cdr = make_bench_cloud();
    let decoded = PointCloud2::from_cdr(&cdr).unwrap();

    let mut group = c.benchmark_group("PointCloud");
    group.throughput(Throughput::Elements(1024));

    group.bench_function("dyn_gather_xyz_1024", |b| {
        b.iter(|| {
            let cloud = DynPointCloud::from_pointcloud2(black_box(&decoded)).unwrap();
            let _x = cloud.gather_f32("x");
            let _y = cloud.gather_f32("y");
            let _z = cloud.gather_f32("z");
        })
    });

    group.bench_function("static_iter_xyz_1024", |b| {
        b.iter(|| {
            let cloud = PointCloud::<BenchXyz>::from_pointcloud2(black_box(&decoded)).unwrap();
            let mut sum = 0.0f32;
            for p in cloud.iter() {
                sum += p.x + p.y + p.z;
            }
            black_box(sum);
        })
    });

    group.bench_function("dyn_iter_xyz_1024", |b| {
        b.iter(|| {
            let cloud = DynPointCloud::from_pointcloud2(black_box(&decoded)).unwrap();
            let x_desc = cloud.field("x").unwrap();
            let y_desc = cloud.field("y").unwrap();
            let z_desc = cloud.field("z").unwrap();
            let mut sum = 0.0f32;
            for p in cloud.iter() {
                sum += p.read_f32_at(x_desc) + p.read_f32_at(y_desc) + p.read_f32_at(z_desc);
            }
            black_box(sum);
        })
    });

    group.finish();
}

// ============================================================================
// CRITERION GROUPS
// ============================================================================

criterion_group! {
    name = benches;
    // Reduce iterations for faster CI runs (~5-10 min instead of ~20 min)
    // - sample_size: 10 (default 100) - fewer iterations per benchmark
    // - measurement_time: 1s (default 5s) - shorter measurement window
    // - warm_up_time: 500ms (default 3s) - shorter warm-up
    // For even faster runs, set BENCH_FAST=1 to also reduce benchmark variants
    config = Criterion::default()
        .sample_size(10)
        .measurement_time(std::time::Duration::from_secs(1))
        .warm_up_time(std::time::Duration::from_millis(500));
    targets =
        bench_fixed_types,
        bench_header,
        bench_image,
        bench_compressed_video,
        bench_radar_cube,
        bench_mask,
        bench_dmabuf,
        bench_pointcloud,
}

criterion_main!(benches);
