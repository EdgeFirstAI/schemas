# CDR Serialization Benchmarks

## Overview

This crate provides in-place CDR handling with zero-copy deserialization and
near-zero-cost access to message fields. Buffer-backed types wrap a raw CDR
byte buffer directly — `from_cdr(&[u8])` borrows the underlying memory with
no allocation, no copy, and no per-element decoding. Typed array fields
(`&[i16]`, `&[f32]`, `&[f64]`, etc.) are exposed as native slices over the
wire buffer on little-endian targets.

## Target Hardware

- Board: NXP i.MX 8M Plus EVK (imx8mpevk-06)
- CPU: Cortex-A53 @ 1.8 GHz (4 cores)
- Architecture: aarch64 (little-endian)

## Benchmark Configuration

| Language | Framework | Config |
|----------|-----------|--------|
| Rust | Criterion 0.8.1 | 10 samples, 1s measurement, 500ms warm-up |
| Python | pytest-benchmark 5.2.3 | min 5 rounds, warm-up enabled, GC disabled |

## CdrFixed Types

Small fixed-size types use direct `encode_fixed` / `decode_fixed` without serde.

| Type | Rust encode | Rust decode | Python ser | Python deser | Py/Rust encode | Py/Rust decode |
|------|------------|------------|------------|-------------|----------------|----------------|
| Time | 109.20 ns | 25.05 ns | 57.72 µs | 50.59 µs | 529x | 2,020x |
| Vector3 | 149.18 ns | 31.19 ns | 63.59 µs | 57.16 µs | 426x | 1,833x |
| Pose | 218.41 ns | 71.26 ns | 98.73 µs | 101.42 µs | 452x | 1,423x |

## Buffer-backed Types

Buffer-backed types provide three modes of operation:

- **`new()`** — serialize fields into a new CDR buffer (single memcpy for bulk data)
- **`from_cdr(vec)`** — take ownership of a buffer, scan once to build offset table
- **`from_cdr(&[u8])`** — borrow existing buffer in-place, zero allocation

### Small Messages

| Type | Rust encode | Rust decode (borrow) | Python ser | Python deser | Py/Rust encode | Py/Rust decode |
|------|------------|---------------------|------------|-------------|----------------|----------------|
| Header | 159.27 ns | 165.70 ns | 80.24 µs | 77.93 µs | 504x | 470x |
| DmaBuffer | 247.91 ns | 97.18 ns | 130.38 µs | 136.57 µs | 526x | 1,405x |

### Image & Video Payloads

Payloads are `&[u8]` blobs — serialization is a single memcpy, zero-copy
borrow provides direct access to the underlying bytes.

| Type | Size | Rust encode | Rust decode (borrow) | Python ser | Python deser |
|------|------|------------|---------------------|------------|-------------|
| Image | VGA rgb8 (900 KB) | 451 µs | 144 ns | 148.9 ms | 34.8 ms |
| Image | HD rgb8 (2.6 MB) | 1.44 ms | 144 ns | 463.9 ms | 103.0 ms |
| Image | FHD yuyv (3.9 MB) | 2.17 ms | 144 ns | 698.9 ms | 165.4 ms |
| CompressedVideo | 100 KB | 25.1 µs | 126 ns | 21.5 ms | 5.97 ms |
| CompressedVideo | 500 KB | 154.3 µs | 126 ns | 94.5 ms | 23.7 ms |

| Type | Size | Py/Rust encode | Py/Rust decode |
|------|------|----------------|----------------|
| Image | VGA rgb8 (900 KB) | 330x | 241,700,000x |
| Image | HD rgb8 (2.6 MB) | 322x | 715,300,000x |
| Image | FHD yuyv (3.9 MB) | 322x | 1,150,700,000x |
| CompressedVideo | 100 KB | 857x | 47,380,000x |
| CompressedVideo | 500 KB | 613x | 188,100,000x |

> Zero-copy borrow at ~144 ns is independent of payload size — the ratio
> diverges toward infinity as message size grows.

### Typed Array Payloads

On little-endian targets, CDR1-LE typed arrays (`i16`, `u16`, `f32`, `f64`)
share native byte order. Serialization uses bulk memcpy; deserialization
exposes a zero-copy `&[T]` slice view directly over the wire buffer.

| Type | Size | Rust encode | Rust decode (borrow) | Python ser | Python deser |
|------|------|------------|---------------------|------------|-------------|
| RadarCube | DRVEGRD169 (1.5 MB) | 734 µs | 129 ns | 358.2 ms | 230.4 ms |
| RadarCube | DRVEGRD171 (1.5 MB) | 887 µs | 129 ns | 371.2 ms | 232.4 ms |

| Type | Size | Py/Rust encode | Py/Rust decode |
|------|------|----------------|----------------|
| RadarCube | DRVEGRD169 (1.5 MB) | 488x | 1,786,000,000x |
| RadarCube | DRVEGRD171 (1.5 MB) | 418x | 1,802,000,000x |

### Segmentation Masks

Mask data is a raw `&[u8]` blob. Zero-copy borrow cost is independent of
payload size.

| Config | Size | Rust encode | Rust decode (borrow) | Python ser | Python deser |
|--------|------|------------|---------------------|------------|-------------|
| 640x640 8-class | 3.1 MB | 1.81 ms | 38 ns | 561.7 ms | 131.0 ms |
| 1280x1280 32-class | 50 MB | 32.4 ms | 38 ns | 8,819 ms | 2,046 ms |

| Config | Size | Py/Rust encode | Py/Rust decode |
|--------|------|----------------|----------------|
| 640x640 8-class | 3.1 MB | 310x | 3,447,400,000x |
| 1280x1280 32-class | 50 MB | 272x | 53,840,000,000x |

## Summary

### Encoding (Serialization)

Rust `new()` uses `CdrSizer` + single-allocation write with bulk memcpy.
Python pycdr2 uses serde-style per-field serialization.

| Category | Rust | Python | Speedup |
|----------|------|--------|---------|
| Fixed types (Time, Vector3, Pose) | 109–218 ns | 58–99 µs | **430–530x** |
| Small messages (Header, DmaBuffer) | 159–248 ns | 80–130 µs | **504–526x** |
| Images (VGA–FHD) | 0.45–2.17 ms | 149–699 ms | **322–330x** |
| Radar cubes (1.5 MB) | 0.73–0.89 ms | 358–371 ms | **418–488x** |
| Masks (3.1–50 MB) | 1.8–32.4 ms | 562–8,819 ms | **272–310x** |

### Decoding (Deserialization)

Rust zero-copy borrow records a small offset table in constant time.
Python pycdr2 must deserialize every field and allocate Python objects.

| Category | Rust borrow | Python deser | Speedup |
|----------|------------|-------------|---------|
| Fixed types (decode_fixed) | 25–71 ns | 51–101 µs | **1,400–2,000x** |
| Small messages | 97–166 ns | 78–137 µs | **470–1,400x** |
| Images (VGA–FHD) | 144 ns | 35–165 ms | **241M–1.2Bx** |
| Radar cubes (1.5 MB) | 129 ns | 230–232 ms | **1.8Bx** |
| Masks (3.1–50 MB) | 38 ns | 131–2,046 ms | **3.4B–53.8Bx** |

> Borrow cost is O(1) with respect to payload size. For buffer-backed types
> carrying large payloads, the Rust zero-copy advantage is effectively
> unbounded — it grows proportionally with message size.

## Design

### How zero-copy borrow works

`from_cdr(&[u8])` scans the buffer once to record a small offset table
(typically 1-7 `usize` values). Field accessors then read at known offsets
with no further validation. The scan itself is O(number of variable-length
fields) — fixed-size regions and typed arrays are skipped in constant time.

The buffer is never copied or reallocated. String fields return `&str` slices
into the buffer. Byte payloads return `&[u8]`. Typed numeric arrays return
`&[i16]`, `&[f32]`, etc. directly reinterpreted from the wire bytes.

### Serialization with `new()`

`new()` computes the exact buffer size in a dry-run pass (`CdrSizer`), then
writes all fields in a single allocation. Bulk data (byte blobs, typed arrays)
is written via `memcpy` rather than element-by-element.

## Previous Work

Full results from the `cdr` 0.2.4 vs `cdr-encoding` 0.10.2 evaluation are
available on branch `feature/EDGEAI-1092-cdr-encoding`.
