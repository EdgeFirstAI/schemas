# CDR Serialization Benchmarks

## Target Hardware

- Board: NXP i.MX 8M Plus EVK (imx8mpevk-06)
- CPU: Cortex-A53 @ 1.8 GHz (4 cores)
- Architecture: aarch64

## Benchmark Configuration

- Framework: Criterion 0.8.1
- Sample size: 10, Measurement time: 1s, Warm-up: 500ms
- Mode: BENCH_FAST=1

## Libraries

| Library | Version | Serde | CDR Format | ROS2 Compatible |
|---------|---------|-------|------------|-----------------|
| cdr | 0.2.4 | Yes | CDR1 LE | Yes |
| cdr-encoding | 0.10.2 | Yes | CDR1 LE | Yes |

## Libraries Ruled Out

| Library | Version | Reason |
|---------|---------|--------|
| ros2_message | 0.0.5 | Dynamic message parser, no serde/typed serialization support |
| hdds-micro | N/A | CDR2 format (incompatible with ROS2 CDR1), no serde, manual API only, not on crates.io |

## Results

### Basic Types

| Message | cdr ser | cdr deser | cdr-enc ser | cdr-enc deser | ser delta | deser delta |
|---------|---------|-----------|-------------|---------------|-----------|-------------|
| Time | 100.78 ns | 37.86 ns | 230.62 ns | 15.03 ns | +129% | **-60%** |
| Duration | 100.74 ns | 37.88 ns | 230.46 ns | 15.03 ns | +129% | **-60%** |
| Header | 162.65 ns | 248.64 ns | 280.72 ns | 212.95 ns | +73% | **-14%** |
| ColorRGBA | 161.42 ns | 46.20 ns | 265.91 ns | 25.05 ns | +65% | **-46%** |
| Vector3 | 165.42 ns | 54.54 ns | 257.79 ns | 25.61 ns | +56% | **-53%** |
| Point | 165.34 ns | 54.55 ns | 257.77 ns | 25.60 ns | +56% | **-53%** |
| Point32 | 140.37 ns | 42.32 ns | 246.12 ns | 24.50 ns | +75% | **-42%** |
| Quaternion | 233.85 ns | 59.57 ns | 288.47 ns | 35.65 ns | +23% | **-40%** |
| Pose | 328.61 ns | 92.52 ns | 361.57 ns | 54.56 ns | +10% | **-41%** |
| Pose2D | 165.31 ns | 43.41 ns | 257.91 ns | 25.60 ns | +56% | **-41%** |
| Transform | 328.55 ns | 92.51 ns | 361.53 ns | 54.56 ns | +10% | **-41%** |
| Twist | 203.79 ns | 87.98 ns | 328.97 ns | 25.05 ns | +61% | **-72%** |

### DmaBuffer (Zero-Copy Reference)

| Variant | cdr ser | cdr deser | cdr-enc ser | cdr-enc deser | ser delta | deser delta |
|---------|---------|-----------|-------------|---------------|-----------|-------------|
| HD_yuyv | 334.16 ns | 340.20 ns | 448.40 ns | 251.96 ns | +34% | **-26%** |

### FoxgloveCompressedVideo

| Size | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt | ser delta | deser delta |
|------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|-----------|-------------|
| 100KB | 496.56 µs | 192.06 MiB/s | 560.07 µs | 170.28 MiB/s | 532.47 µs | 179.10 MiB/s | 627.58 µs | 151.96 MiB/s | -7% | +12% |
| 500KB | 2.4926 ms | 191.30 MiB/s | 2.9062 ms | 164.07 MiB/s | 2.9641 ms | 160.87 MiB/s | 3.2811 ms | 145.33 MiB/s | -16% | +13% |

### CompressedImage

| Variant | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt | ser delta | deser delta |
|---------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|-----------|-------------|
| HD_jpeg (150KB) | 743.38 µs | 192.43 MiB/s | 841.75 µs | 169.94 MiB/s | 798.24 µs | 179.21 MiB/s | 961.72 µs | 148.74 MiB/s | -7% | +14% |
| FHD_jpeg (400KB) | 1.9997 ms | 190.76 MiB/s | 2.3270 ms | 163.93 MiB/s | 2.2515 ms | 169.43 MiB/s | 2.5659 ms | 148.67 MiB/s | -11% | +10% |

### RadarCube

| Config | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt | ser delta | deser delta |
|--------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|-----------|-------------|
| DRVEGRD169_short | 6.3935 ms | 234.61 MiB/s | 10.113 ms | 148.33 MiB/s | 5.7587 ms | 260.48 MiB/s | 7.7865 ms | 192.64 MiB/s | **-10%** | **-23%** |
| DRVEGRD171_medium | 6.4689 ms | 231.88 MiB/s | 10.131 ms | 148.06 MiB/s | 5.7578 ms | 260.51 MiB/s | 7.7730 ms | 192.98 MiB/s | **-11%** | **-23%** |
| DRVEGRD171_long | 6.4827 ms | 231.39 MiB/s | 10.156 ms | 147.70 MiB/s | 5.7614 ms | 260.35 MiB/s | 7.7799 ms | 192.81 MiB/s | **-11%** | **-23%** |

### PointCloud2

| Config | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt | ser delta | deser delta |
|--------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|-----------|-------------|
| medium_10K (160KB) | 795.44 µs | 191.83 MiB/s | 1.0373 ms | 147.10 MiB/s | 850.54 µs | 179.40 MiB/s | 935.82 µs | 163.05 MiB/s | -6% | **-10%** |
| dense_65K (1MB) | 5.1174 ms | 195.41 MiB/s | 6.8220 ms | 146.58 MiB/s | 6.2179 ms | 160.83 MiB/s | 6.4266 ms | 155.60 MiB/s | -18% | **-6%** |

### Image

| Variant | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt | ser delta | deser delta |
|---------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|-----------|-------------|
| HD_rgb8 (2.6MB) | 13.329 ms | 197.82 MiB/s | 16.141 ms | 163.36 MiB/s | 15.906 ms | 165.77 MiB/s | 18.958 ms | 139.08 MiB/s | -16% | +17% |
| HD_nv12 (1.3MB) | 6.7308 ms | 195.87 MiB/s | 8.0683 ms | 163.40 MiB/s | 8.0325 ms | 164.13 MiB/s | 9.6112 ms | 137.17 MiB/s | -16% | +19% |
| FHD_yuyv (3.9MB) | 19.927 ms | 198.48 MiB/s | 24.059 ms | 164.39 MiB/s | 23.703 ms | 166.86 MiB/s | 26.718 ms | 148.03 MiB/s | -16% | +11% |

### Mask

| Config | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt | ser delta | deser delta |
|--------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|-----------|-------------|
| 640x640_8class (3.1MB) | 15.890 ms | 196.66 MiB/s | 19.049 ms | 164.05 MiB/s | 18.901 ms | 165.34 MiB/s | 19.561 ms | 159.76 MiB/s | -16% | -3% |
| 1280x1280_32class (50MB) | 256.74 ms | 194.75 MiB/s | 322.64 ms | 154.97 MiB/s | 324.04 ms | 154.30 MiB/s | 321.54 ms | 155.50 MiB/s | -21% | ~0% |

### CompressedMask (zstd)

| Config | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt | ser delta | deser delta |
|--------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|-----------|-------------|
| 640x640_8class | 15.700 ms | 199.05 MiB/s | 20.121 ms | 155.32 MiB/s | 18.777 ms | 166.43 MiB/s | 19.389 ms | 161.18 MiB/s | -16% | **-4%** |
| 1280x1280_32class | 257.84 ms | 193.93 MiB/s | 321.69 ms | 155.43 MiB/s | 325.99 ms | 153.38 MiB/s | 321.58 ms | 155.49 MiB/s | -21% | ~0% |

### Model

| Config | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt | ser delta | deser delta |
|--------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|-----------|-------------|
| 20_boxes (1.3KB) | 8.5955 µs | 158.88 MiB/s | 18.926 µs | 72.16 MiB/s | 8.6113 µs | 158.59 MiB/s | 18.121 µs | 75.36 MiB/s | ~0% | **-4%** |
| 20_boxes_mask_VGA (300KB) | 1.5644 ms | 188.16 MiB/s | 1.8113 ms | 162.51 MiB/s | 1.7661 ms | 166.67 MiB/s | 2.0270 ms | 145.22 MiB/s | -11% | +12% |

## Wire Compatibility

Byte-level comparison confirms **identical CDR wire format** between `cdr` 0.2.4 and `cdr-encoding` 0.10.2 for all tested message types:

- **Time** (fixed-size struct): identical
- **Header** (String field): identical
- **CompressedImage** (Vec<u8>): identical
- **Model** (nested structs, Vec<Box>, Vec<Mask>): identical

Note: `cdr` includes a 4-byte CDR encapsulation header (`[0x00, 0x01, 0x00, 0x00]` for CDR1 LE) that `cdr-encoding` does not emit natively. The `serde_cdr` wrapper adds/strips this header to maintain full wire compatibility.

## Summary

### Serialization (writing)

`cdr-encoding` is **slower** for serialization across the board:

- **Small types**: 10-129% slower (CDR encapsulation header overhead dominates)
- **Vec<u8> payloads** (Image, CompressedImage, Mask): 7-21% slower
- **Vec<i16> payloads** (RadarCube): **10-11% faster** (only exception)
- **Structured data** (Model without mask): ~0% (negligible difference)

### Deserialization (reading)

Mixed results for deserialization:

- **Small types**: 14-72% **faster** (significant wins)
- **RadarCube** (Vec<i16>): 23% **faster**
- **PointCloud2**: 6-10% **faster**
- **Vec<u8> payloads** (Image, CompressedImage, CompressedVideo): 10-19% **slower**
- **Large Mask**: ~0% (negligible)
- **Model**: 4% faster (small), 12% slower (with mask)

### Key Observations

1. **RadarCube is the clear winner for cdr-encoding**: Both serialize and deserialize improve significantly (+11%/+30% throughput). This is likely due to more efficient handling of `Vec<i16>` data.

2. **Vec<u8> heavy types favor cdr**: Image, CompressedImage, CompressedVideo, and large Masks serialize 7-21% faster with the current `cdr` crate.

3. **Small message deserialization strongly favors cdr-encoding**: 2-3.5x faster for fixed-size types. However, these messages are already sub-microsecond and not a bottleneck.

4. **Small message serialization strongly favors cdr**: The CDR encapsulation header overhead (4-byte prefix + Vec allocation) significantly impacts small messages. This could be mitigated by pre-allocating or writing directly.

## Recommendations

**Stay with `cdr` 0.2.4** as the primary CDR library. Rationale:

1. The production-critical workloads (Image, CompressedImage, CompressedVideo, PointCloud2) are all Vec<u8>-heavy and perform better with `cdr`.

2. RadarCube benefits from `cdr-encoding`, but this single type doesn't justify switching the entire library.

3. Small type deserialization improvements from `cdr-encoding` are impressive in relative terms but the absolute times (15-55 ns) are not bottlenecks.

4. The CDR encapsulation header compatibility layer adds complexity and overhead to `cdr-encoding`.

**Future consideration**: If RadarCube serialization becomes a bottleneck, consider a specialized fast path using `cdr-encoding` for that type only, or investigate zero-copy approaches.
