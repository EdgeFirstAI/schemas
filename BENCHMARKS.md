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

- Framework: Criterion 0.8.1
- Sample size: 10, Measurement time: 1s, Warm-up: 500ms

## Baseline

| Implementation | Version | Approach |
|----------------|---------|----------|
| cdr + serde (baseline) | cdr 0.2.4 | serde serialize/deserialize with CDR encapsulation |
| zero-copy (this crate) | — | CdrFixed encode/decode + buffer-backed in-place types |

## CdrFixed Types

Small fixed-size types use direct `encode_fixed` / `decode_fixed` without serde.

| Type | Baseline ser | `encode_fixed` | Baseline deser | `decode_fixed` |
|------|-------------|----------------|---------------|----------------|
| Time | 100.78 ns | 109.12 ns | 37.86 ns | 25.06 ns |
| Vector3 | 165.42 ns | 149.19 ns | 54.54 ns | 31.19 ns |
| Pose | 328.61 ns | 219.36 ns | 92.52 ns | 71.25 ns |

## Buffer-backed Types

Buffer-backed types provide three modes of operation:

- **`new()`** — serialize fields into a new CDR buffer (single memcpy for bulk data)
- **`from_cdr(vec)`** — take ownership of a buffer, scan once to build offset table
- **`from_cdr(&[u8])`** — borrow existing buffer in-place, zero allocation

### Small Messages

| Type | Baseline ser | `new()` | Baseline deser | `from_cdr` (borrow) |
|------|-------------|---------|---------------|---------------------|
| Header | 162.65 ns | 158.20 ns | 248.64 ns | 165.10 ns |
| DmaBuffer | 334.16 ns | 247.55 ns | 340.20 ns | 97.15 ns |

### Image & Video Payloads

Payloads are `&[u8]` blobs — serialization is a single memcpy, zero-copy
borrow provides direct access to the underlying bytes.

| Type | Size | Baseline ser | `new()` | Baseline deser | Borrow |
|------|------|-------------|---------|---------------|--------|
| Image | VGA rgb8 (900 KB) | — | 444 µs | — | 145 ns |
| Image | HD rgb8 (2.6 MB) | 13.33 ms | 1.44 ms | 16.14 ms | 145 ns |
| Image | FHD yuyv (3.9 MB) | 19.93 ms | 2.07 ms | 24.06 ms | 145 ns |
| CompressedVideo | 100 KB | 496.6 µs | 25.1 µs | 560.1 µs | 126 ns |
| CompressedVideo | 500 KB | 2.49 ms | 154.4 µs | 2.91 ms | 126 ns |

### Typed Array Payloads

On little-endian targets, CDR1-LE typed arrays (`i16`, `u16`, `f32`, `f64`)
share native byte order. Serialization uses bulk memcpy; deserialization
exposes a zero-copy `&[T]` slice view directly over the wire buffer.

| Type | Size | Baseline ser | `new()` | Baseline deser | Borrow |
|------|------|-------------|---------|---------------|--------|
| RadarCube | DRVEGRD169 (1.5 MB) | 6.39 ms | 845 µs | 10.11 ms | 129 ns |
| RadarCube | DRVEGRD171 (1.5 MB) | 6.48 ms | 842 µs | 10.13 ms | 129 ns |

### Segmentation Masks

Mask data is a raw `&[u8]` blob. Zero-copy borrow cost is independent of
payload size.

| Config | Size | Baseline ser | `new()` | Baseline deser | Borrow |
|--------|------|-------------|---------|---------------|--------|
| 640x640 8-class | 3.1 MB | 15.89 ms | 1.94 ms | 19.05 ms | 38 ns |
| 1280x1280 32-class | 50 MB | 256.7 ms | 32.6 ms | 322.6 ms | 38 ns |

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
