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

| Message | cdr ser | cdr deser | cdr-encoding ser | cdr-encoding deser |
|---------|---------|-----------|-------------------|--------------------|
| Time | 100.78 ns | 37.86 ns | | |
| Duration | 100.74 ns | 37.88 ns | | |
| Header | 162.65 ns | 248.64 ns | | |
| ColorRGBA | 161.42 ns | 46.20 ns | | |
| Vector3 | 165.42 ns | 54.54 ns | | |
| Point | 165.34 ns | 54.55 ns | | |
| Point32 | 140.37 ns | 42.32 ns | | |
| Quaternion | 233.85 ns | 59.57 ns | | |
| Pose | 328.61 ns | 92.52 ns | | |
| Pose2D | 165.31 ns | 43.41 ns | | |
| Transform | 328.55 ns | 92.51 ns | | |
| Twist | 203.79 ns | 87.98 ns | | |

### DmaBuffer (Zero-Copy Reference)

| Variant | cdr ser | cdr deser | cdr-encoding ser | cdr-encoding deser |
|---------|---------|-----------|-------------------|--------------------|
| HD_yuyv | 334.16 ns | 340.20 ns | | |

### FoxgloveCompressedVideo

| Size | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt |
|------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|
| 100KB | 496.56 µs | 192.06 MiB/s | 560.07 µs | 170.28 MiB/s | | | | |
| 500KB | 2.4926 ms | 191.30 MiB/s | 2.9062 ms | 164.07 MiB/s | | | | |

### CompressedImage

| Variant | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt |
|---------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|
| HD_jpeg (150KB) | 743.38 µs | 192.43 MiB/s | 841.75 µs | 169.94 MiB/s | | | | |
| FHD_jpeg (400KB) | 1.9997 ms | 190.76 MiB/s | 2.3270 ms | 163.93 MiB/s | | | | |

### RadarCube

| Config | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt |
|--------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|
| DRVEGRD169_short | 6.3935 ms | 234.61 MiB/s | 10.113 ms | 148.33 MiB/s | | | | |
| DRVEGRD171_medium | 6.4689 ms | 231.88 MiB/s | 10.131 ms | 148.06 MiB/s | | | | |
| DRVEGRD171_long | 6.4827 ms | 231.39 MiB/s | 10.156 ms | 147.70 MiB/s | | | | |

### PointCloud2

| Config | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt |
|--------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|
| medium_10K (160KB) | 795.44 µs | 191.83 MiB/s | 1.0373 ms | 147.10 MiB/s | | | | |
| dense_65K (1MB) | 5.1174 ms | 195.41 MiB/s | 6.8220 ms | 146.58 MiB/s | | | | |

### Image

| Variant | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt |
|---------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|
| HD_rgb8 (2.6MB) | 13.329 ms | 197.82 MiB/s | 16.141 ms | 163.36 MiB/s | | | | |
| HD_nv12 (1.3MB) | 6.7308 ms | 195.87 MiB/s | 8.0683 ms | 163.40 MiB/s | | | | |
| FHD_yuyv (3.9MB) | 19.927 ms | 198.48 MiB/s | 24.059 ms | 164.39 MiB/s | | | | |

### Mask

| Config | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt |
|--------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|
| 640x640_8class (3.1MB) | 15.890 ms | 196.66 MiB/s | 19.049 ms | 164.05 MiB/s | | | | |
| 1280x1280_32class (50MB) | 256.74 ms | 194.75 MiB/s | 322.64 ms | 154.97 MiB/s | | | | |

### CompressedMask (zstd)

| Config | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt |
|--------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|
| 640x640_8class | 15.700 ms | 199.05 MiB/s | 20.121 ms | 155.32 MiB/s | | | | |
| 1280x1280_32class | 257.84 ms | 193.93 MiB/s | 321.69 ms | 155.43 MiB/s | | | | |

### Model

| Config | cdr ser | cdr ser thrpt | cdr deser | cdr deser thrpt | cdr-enc ser | cdr-enc ser thrpt | cdr-enc deser | cdr-enc deser thrpt |
|--------|---------|---------------|-----------|-----------------|-------------|-------------------|---------------|---------------------|
| 20_boxes (1.3KB) | 8.5955 µs | 158.88 MiB/s | 18.926 µs | 72.16 MiB/s | | | | |
| 20_boxes_mask_VGA (300KB) | 1.5644 ms | 188.16 MiB/s | 1.8113 ms | 162.51 MiB/s | | | | |

## Wire Compatibility

_To be verified after cdr-encoding integration._

## Recommendations

_Pending cdr-encoding benchmark results._
