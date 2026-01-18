# TODO Phase A: Rust Benchmarks

**Phase:** A  
**Status:** ✅ Complete  
**Estimate:** 6-8 hours  
**Actual:** ~4 hours  
**Dependencies:** None  
**Blocks:** Phase F (Cross-Language Validation)

---

## Objective

Implement comprehensive performance benchmarks for Rust CDR serialization/deserialization, focusing on heavy message types used in production (`~/Software/EdgeFirst/samples`).

---

## Completion Summary

All deliverables have been implemented:

| Deliverable | Status | Notes |
|-------------|--------|-------|
| A.1 Cargo Configuration | ✅ | criterion 0.5 + rand 0.8 added |
| A.2 Benchmark Implementation | ✅ | `benches/serialization.rs` (~450 lines) |
| A.3.1 FoxgloveCompressedVideo | ✅ | 4 sizes (10KB, 100KB, 500KB, 1MB) |
| A.3.2 RadarCube | ✅ | 3 sizes + complex variant |
| A.3.3 PointCloud2 | ✅ | 4 sizes (1K-131K points) |
| A.3.4 Mask | ✅ | 4 sizes (64KB-6MB) |
| A.3.5 Image | ✅ | 4 sizes (VGA-FHD, rgb8/yuyv) |
| A.4 Helper functions | ✅ | Random data generation |
| A.5 Baseline benchmarks | ✅ | Time, Duration, Header, geometry_msgs |
| Documentation | ✅ | TESTING.md updated |

### Files Modified/Created

- `Cargo.toml` - Added dev-dependencies and bench configuration
- `benches/serialization.rs` - New benchmark file (~450 lines)
- `TESTING.md` - Updated benchmark documentation

### Verification

```bash
# All tests pass
cargo test  # 49 tests pass

# Benchmarks compile and run
cargo bench --no-run  # Compiles successfully
cargo bench -- "builtin_interfaces"  # Runs with throughput metrics
```

### Deferred Items

The following items will be completed before submitting changes:

- [ ] Run full benchmark suite and capture baseline metrics
- [ ] Document baseline performance numbers

---

## Background

The EdgeFirst samples application processes high-throughput sensor data:
- **CompressedVideo**: H.264 encoded camera frames (~50-500KB per frame at 30fps)
- **RadarCube**: 4D radar tensors (shape: [SEQ, RANGE, RX, DOPPLER], int16 data)
- **PointCloud2**: LiDAR/radar point clouds (10K-100K points typical)
- **Mask**: Segmentation masks (often zstd compressed, full-frame resolution)

Understanding serialization performance is critical for optimizing the perception pipeline.

---

## Deliverables

### A.1 Cargo Configuration

**File:** `Cargo.toml`

Add benchmark dependencies:
```toml
[dev-dependencies]
criterion = { version = "0.5", features = ["html_reports"] }
rand = "0.8"

[[bench]]
name = "serialization"
harness = false
```

**Acceptance:**
- [ ] `cargo bench` compiles without errors
- [ ] Criterion generates HTML reports in `target/criterion/`

---

### A.2 Benchmark Implementation

**File:** `benches/serialization_benchmarks.rs`

Create comprehensive benchmarks organized by message category:

```rust
use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId, Throughput};
use edgefirst_schemas::*;

// ============================================================================
// BENCHMARK GROUPS
// ============================================================================

fn bench_builtin_interfaces(c: &mut Criterion) {
    // Time, Duration - lightweight baseline
}

fn bench_std_msgs(c: &mut Criterion) {
    // Header, ColorRGBA
}

fn bench_geometry_msgs(c: &mut Criterion) {
    // Vector3, Point, Quaternion, Pose, Transform, Twist
}

fn bench_heavy_messages(c: &mut Criterion) {
    // CompressedVideo, RadarCube, PointCloud2, Mask, Image
    // Multiple sizes per type
}

criterion_group!(
    benches,
    bench_builtin_interfaces,
    bench_std_msgs,
    bench_geometry_msgs,
    bench_heavy_messages,
);
criterion_main!(benches);
```

**Acceptance:**
- [ ] All benchmark functions compile and run
- [ ] Benchmarks produce reproducible results (low variance)
- [ ] HTML reports generated with charts

---

### A.3 Heavy Message Benchmarks (Priority)

#### A.3.1 FoxgloveCompressedVideo

Test with realistic H.264 payload sizes:

| Scenario | Data Size | Description |
|----------|-----------|-------------|
| Small frame | 10 KB | Low-motion scene |
| Medium frame | 100 KB | Typical scene |
| Large frame | 500 KB | High-detail scene |
| Keyframe | 1 MB | I-frame |

```rust
fn bench_compressed_video(c: &mut Criterion) {
    let mut group = c.benchmark_group("FoxgloveCompressedVideo");
    
    for size in [10_000, 100_000, 500_000, 1_000_000] {
        group.throughput(Throughput::Bytes(size as u64));
        
        let video = create_compressed_video(size);
        
        group.bench_with_input(
            BenchmarkId::new("serialize", size),
            &video,
            |b, v| b.iter(|| serde_cdr::serialize(black_box(v))),
        );
        
        let bytes = serde_cdr::serialize(&video).unwrap();
        group.bench_with_input(
            BenchmarkId::new("deserialize", size),
            &bytes,
            |b, data| b.iter(|| serde_cdr::deserialize::<FoxgloveCompressedVideo>(black_box(data))),
        );
    }
    group.finish();
}
```

**Acceptance:**
- [ ] Benchmarks for 4 payload sizes
- [ ] Both serialize and deserialize measured
- [ ] Throughput reported in bytes/sec

#### A.3.2 RadarCube

Test with Raivin radar dimensions (from samples):

| Scenario | Shape | Elements | Data Size |
|----------|-------|----------|-----------|
| Small cube | [8, 64, 4, 32] | 65,536 | 128 KB |
| Medium cube | [16, 128, 8, 64] | 1,048,576 | 2 MB |
| Large cube | [32, 256, 12, 128] | 12,582,912 | 24 MB |

```rust
fn bench_radar_cube(c: &mut Criterion) {
    let mut group = c.benchmark_group("RadarCube");
    
    let shapes = [
        ([8, 64, 4, 32], "small"),
        ([16, 128, 8, 64], "medium"),
        ([32, 256, 12, 128], "large"),
    ];
    
    for (shape, name) in shapes {
        let cube = create_radar_cube(&shape);
        let data_size = cube.cube.len() * 2; // i16 = 2 bytes
        
        group.throughput(Throughput::Bytes(data_size as u64));
        
        group.bench_with_input(
            BenchmarkId::new("serialize", name),
            &cube,
            |b, c| b.iter(|| serde_cdr::serialize(black_box(c))),
        );
        
        let bytes = serde_cdr::serialize(&cube).unwrap();
        group.bench_with_input(
            BenchmarkId::new("deserialize", name),
            &bytes,
            |b, data| b.iter(|| serde_cdr::deserialize::<RadarCube>(black_box(data))),
        );
    }
    group.finish();
}
```

**Acceptance:**
- [ ] Benchmarks for 3 cube sizes
- [ ] Complex number handling tested (is_complex: true)
- [ ] Memory allocation patterns analyzed

#### A.3.3 PointCloud2

Test with typical LiDAR/radar point counts:

| Scenario | Points | Point Step | Data Size |
|----------|--------|------------|-----------|
| Sparse | 1,000 | 32 bytes | 32 KB |
| Medium | 10,000 | 32 bytes | 320 KB |
| Dense | 65,536 | 32 bytes | 2 MB |
| Very Dense | 131,072 | 32 bytes | 4 MB |

Standard point format (x, y, z, intensity, ring, time):
```rust
fn create_point_cloud(num_points: usize) -> PointCloud2 {
    let point_step = 32; // 4 fields * 4 bytes + padding
    let fields = vec![
        PointField { name: "x".into(), offset: 0, datatype: 7, count: 1 },
        PointField { name: "y".into(), offset: 4, datatype: 7, count: 1 },
        PointField { name: "z".into(), offset: 8, datatype: 7, count: 1 },
        PointField { name: "intensity".into(), offset: 12, datatype: 7, count: 1 },
    ];
    
    PointCloud2 {
        header: create_header(),
        height: 1,
        width: num_points as u32,
        fields,
        is_bigendian: false,
        point_step,
        row_step: point_step * num_points as u32,
        data: vec![0u8; point_step as usize * num_points],
        is_dense: true,
    }
}
```

**Acceptance:**
- [ ] Benchmarks for 4 point cloud sizes
- [ ] PointField vector handling tested
- [ ] Memory efficiency analyzed

#### A.3.4 Mask (Segmentation)

Test with typical segmentation output sizes:

| Scenario | Dimensions | Data Size |
|----------|------------|-----------|
| Small | 256×256×1 | 64 KB |
| Medium | 640×480×1 | 300 KB |
| Large | 1920×1080×1 | 2 MB |
| Multi-class | 640×480×20 | 6 MB |

**Acceptance:**
- [ ] Benchmarks for uncompressed masks
- [ ] Note: zstd compression benchmarks are out of scope (handled by application)

#### A.3.5 Image (Raw)

Test with standard image resolutions:

| Scenario | Resolution | Encoding | Data Size |
|----------|------------|----------|-----------|
| VGA RGB | 640×480×3 | rgb8 | 900 KB |
| HD RGB | 1280×720×3 | rgb8 | 2.7 MB |
| FHD RGB | 1920×1080×3 | rgb8 | 6.2 MB |
| YUV422 | 1920×1080×2 | yuyv | 4.1 MB |

**Acceptance:**
- [ ] Benchmarks for 4 image sizes
- [ ] Different encodings tested

---

### A.4 Benchmark Helpers

**File:** `benches/serialization_benchmarks.rs` (helper functions section)

```rust
// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

fn create_header() -> std_msgs::Header {
    std_msgs::Header {
        stamp: builtin_interfaces::Time { sec: 1234567890, nanosec: 123456789 },
        frame_id: "sensor_frame".to_string(),
    }
}

fn create_compressed_video(data_size: usize) -> FoxgloveCompressedVideo {
    FoxgloveCompressedVideo {
        header: create_header(),
        data: vec![0u8; data_size], // Simulated H.264 NAL units
        format: "h264".to_string(),
    }
}

fn create_radar_cube(shape: &[u32; 4]) -> RadarCube {
    let total_elements: usize = shape.iter().map(|&x| x as usize).product();
    RadarCube {
        header: create_header(),
        timestamp: 1234567890123456,
        layout: vec![6, 1, 5, 2], // SEQUENCE, RANGE, RXCHANNEL, DOPPLER
        shape: shape.iter().map(|&x| x as u16).collect(),
        scales: vec![1.0, 2.5, 1.0, 0.5], // meters, meters, unitless, m/s
        cube: vec![0i16; total_elements],
        is_complex: false,
    }
}

fn create_point_cloud(num_points: usize) -> PointCloud2 { /* ... */ }

fn create_mask(width: u32, height: u32, channels: u32) -> Mask { /* ... */ }

fn create_image(width: u32, height: u32, encoding: &str) -> Image { /* ... */ }
```

---

### A.5 Baseline Benchmarks (Lightweight Messages)

Include lightweight messages for baseline comparison:

```rust
fn bench_builtin_interfaces(c: &mut Criterion) {
    let mut group = c.benchmark_group("builtin_interfaces");
    
    // Time
    let time = Time { sec: 1234567890, nanosec: 123456789 };
    group.bench_function("Time/serialize", |b| {
        b.iter(|| serde_cdr::serialize(black_box(&time)))
    });
    
    // Duration  
    let duration = Duration { sec: 60, nanosec: 500000000 };
    group.bench_function("Duration/serialize", |b| {
        b.iter(|| serde_cdr::serialize(black_box(&duration)))
    });
    
    group.finish();
}

fn bench_geometry_msgs(c: &mut Criterion) {
    let mut group = c.benchmark_group("geometry_msgs");
    
    // Vector3, Point, Quaternion, Pose, Transform, Twist
    // Each with serialize + deserialize
    
    group.finish();
}
```

---

## Test Data Considerations

### Realistic Data Generation

For accurate benchmarks, consider:

1. **CompressedVideo**: Use random bytes (simulates entropy of H.264)
2. **RadarCube**: Use random i16 values (simulates real radar returns)
3. **PointCloud2**: Generate structured point data (x,y,z in realistic ranges)
4. **Mask**: Use random u8 for class IDs (0-255)

### Memory Allocation

Track allocations using:
- `#[global_allocator]` with a tracking allocator
- Compare `Vec::with_capacity` pre-allocation vs dynamic growth

---

## Validation Checklist

### Before Merging

- [ ] All benchmarks compile: `cargo bench --no-run`
- [ ] Benchmarks complete in reasonable time (<5 min total)
- [ ] Results are reproducible (variance <10%)
- [ ] HTML reports generate correctly
- [ ] No regressions in existing tests: `cargo test`
- [ ] Documentation updated in TESTING.md

### Performance Baselines (Expected Ranges)

| Message Type | Size | Serialize | Deserialize |
|--------------|------|-----------|-------------|
| Time | 8 bytes | <100 ns | <100 ns |
| Header | ~50 bytes | <500 ns | <500 ns |
| PointCloud2 (10K pts) | 320 KB | <1 ms | <1 ms |
| RadarCube (medium) | 2 MB | <5 ms | <5 ms |
| CompressedVideo (100KB) | 100 KB | <500 μs | <500 μs |

*Note: These are rough estimates. Actual baselines will be established on target hardware.*

---

## File Structure After Completion

```
benches/
└── serialization_benchmarks.rs    # All benchmarks (~500 lines)

target/criterion/
├── FoxgloveCompressedVideo/
│   ├── serialize/
│   │   └── 10000/ (through 1000000)
│   └── deserialize/
├── RadarCube/
├── PointCloud2/
├── Mask/
├── Image/
├── builtin_interfaces/
├── std_msgs/
└── geometry_msgs/
```

---

## Commands

```bash
# Run all benchmarks
cargo bench

# Run specific benchmark group
cargo bench -- "RadarCube"
cargo bench -- "PointCloud2"

# Run with verbose output
cargo bench -- --verbose

# Generate only (no run) - useful for CI
cargo bench --no-run

# Compare against baseline
cargo bench -- --save-baseline main
cargo bench -- --baseline main
```

---

## Success Criteria

- [x] `Cargo.toml` updated with criterion dependency
- [x] `benches/serialization.rs` implemented (~450 lines)
- [x] CompressedVideo benchmarks (4 sizes) complete
- [x] RadarCube benchmarks (3 sizes + complex) complete  
- [x] PointCloud2 benchmarks (4 sizes) complete
- [x] Mask benchmarks (4 sizes) complete
- [x] Image benchmarks (4 sizes) complete
- [x] Baseline benchmarks for lightweight messages complete
- [ ] HTML reports generated and reviewed (deferred)
- [ ] Performance baselines documented (deferred)
- [x] TESTING.md updated with benchmark instructions

---

**Next Phase:** [TODO_B.md](./TODO_B.md) - C API Expansion (can proceed in parallel)
