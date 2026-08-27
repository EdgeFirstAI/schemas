# Testing and Benchmarks

**Version:** 1.0  
**Last Updated:** January 2026

This document describes the comprehensive testing strategy for EdgeFirst Perception Schemas, covering unit tests, integration tests, and performance benchmarks across Rust, C, and Python implementations.

---

## Table of Contents

- [Quick Start](#quick-start)
- [Testing Philosophy](#testing-philosophy)
- [Coverage Targets](#coverage-targets)
- [Test Architecture](#test-architecture)
- [Rust Testing](#rust-testing)
- [C Testing](#c-testing)
- [Python Testing](#python-testing)
- [MCAP Testing](#mcap-testing)
- [Benchmarks](#benchmarks)
- [Coverage and CI/CD](#coverage-and-cicd)
- [Test Data](#test-data)

---

## Quick Start

### Prerequisites

**Install testing tools:**

```bash
# Rust coverage and benchmarking
cargo install cargo-llvm-cov

# C testing framework (Criterion)
# Ubuntu/Debian:
sudo apt-get install -y libcriterion-dev

# macOS:
brew install criterion

# Python testing tools
pip install pytest pytest-cov pytest-benchmark mypy hypothesis
```

### Run All Tests

```bash
# Rust tests with coverage
cargo llvm-cov --all-features --workspace --html
open target/llvm-cov/html/index.html

# C tests with coverage
cd tests/c
make test-coverage
open coverage-html/index.html

# Python tests with coverage
pytest tests/python/ --cov=edgefirst --cov-report=html
open htmlcov/index.html
```

### Run Benchmarks

```bash
# Rust benchmarks (optimized + debug symbols)
cargo bench

# C benchmarks
cd tests/c
make bench

# Python benchmarks
pytest benches/python/bench_native.py --benchmark-only
```

For complete command reference, see the [Commands Cheat Sheet](#commands-cheat-sheet) section.

---

## Testing Philosophy

### Goals

1. **High Coverage**: Minimum 70% code coverage overall, 80%+ for critical paths
2. **Cross-Language Validation**: Verify serialization compatibility between Rust, C, and Python
3. **Performance Visibility**: Benchmark serialization/deserialization for all message types
4. **Memory Safety**: Validate C API memory management and leak detection
5. **CI/CD Integration**: Automated testing in GitHub Actions with SonarCloud reporting

### Testing Pyramid

```
        ┌──────────────────┐
        │  E2E / Integration│  5%   - Cross-language, full workflows
        ├──────────────────┤
        │  Integration      │  25%  - Multi-module interactions
        ├──────────────────┤
        │  Unit Tests       │  70%  - Individual functions/types
        └──────────────────┘
```

---

## Coverage Targets

| Component | Minimum Coverage | Notes |
|-----------|-----------------|-------|
| **Overall** | 70% | Project-wide minimum |
| **builtin_interfaces** | 90% | Critical time primitives |
| **std_msgs** | 90% | Fundamental ROS2 types |
| **geometry_msgs** | 85% | Spatial primitives |
| **sensor_msgs** | 85% | Camera, LiDAR, IMU |
| **edgefirst_msgs** | 80% | Custom detection types |
| **foxglove_msgs** | 80% | Visualization |
| **FFI (crates/capi/src/lib.rs)** | 70% | Tested via C tests |
| **CDR encode/decode** | 95% | Serialization critical |

---

## Test Architecture

### Test Organization

```
schemas/
├── crates/
│   ├── schemas/                  # Pure-Rust rlib (`edgefirst-schemas`)
│   │   ├── src/                  # #[cfg(test)] mod tests inline
│   │   ├── benches/serialization.rs  # Criterion benchmarks
│   │   └── tests/                # Rust integration tests
│   ├── capi/                     # C library (`edgefirst-schemas-capi`)
│   │   ├── src/lib.rs            # #[no_mangle] FFI surface (tested via C)
│   │   └── tests/
│   │       ├── c/                # Criterion C tests
│   │       │   ├── test_builtin_interfaces.c
│   │       │   ├── test_std_msgs.c
│   │       │   ├── test_geometry_msgs.c
│   │       │   ├── test_sensor_msgs.c
│   │       │   └── test_edgefirst_msgs.c
│   │       └── cpp/              # Criterion C++ tests
│   │           ├── test_image_view.cpp
│   │           └── ...
│   └── python/                   # PyO3 wheel (`edgefirst-schemas-python`)
│       ├── src/                  # Rust binding code
│       └── python/               # .pyi type stubs
│
└── tests/python/                 # Python integration tests (top-level)
    ├── test_std_msgs.py
    ├── test_sensor_msgs.py
    ├── test_geometry_msgs.py
    ├── test_edgefirst_msgs.py
    ├── test_cross_language.py    # Rust ↔ Python validation
    └── benchmarks/               # pytest-benchmark
        ├── test_serialize.py
        ├── test_deserialize.py
        └── test_round_trip.py
```

### Test Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    Test Execution Flow                       │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │  Rust Tests  │  │   C Tests    │  │Python Tests  │     │
│  │              │  │              │  │              │     │
│  │ • Unit       │  │ • Criterion  │  │ • pytest     │     │
│  │ • Integration│  │ • Valgrind   │  │ • hypothesis │     │
│  │ • Proptest   │  │ • Round-trip │  │ • mypy       │     │
│  │ • Criterion  │  │ • Benchmarks │  │ • benchmarks │     │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘     │
│         │                  │                  │             │
│         ▼                  ▼                  ▼             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │cargo-llvm-cov│  │  gcov/lcov   │  │ pytest-cov   │     │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘     │
│         │                  │                  │             │
│         └──────────────────┼──────────────────┘             │
│                            ▼                                │
│                  ┌──────────────────┐                       │
│                  │  Coverage Merge  │                       │
│                  │   (LCOV format)  │                       │
│                  └─────────┬────────┘                       │
│                            ▼                                │
│         ┌──────────────────────────────────┐               │
│         │        SonarCloud Upload         │               │
│         │  • Quality Gate Check            │               │
│         │  • Coverage Visualization        │               │
│         │  • Code Smell Detection          │               │
│         └──────────────────────────────────┘               │
└─────────────────────────────────────────────────────────────┘
```

---

## Rust Testing

### Unit Tests

**Location**: Co-located with implementation using `#[cfg(test)]` modules

**Example (src/std_msgs.rs):**

```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_header_default() {
        let header = Header::default();
        assert_eq!(header.stamp.sec, 0);
        assert_eq!(header.stamp.nanosec, 0);
        assert_eq!(header.frame_id, "");
    }
    
    #[test]
    fn test_header_round_trip_cdr() {
        let header = Header {
            stamp: Time { sec: 1234567890, nanosec: 123456789 },
            frame_id: "camera".to_string(),
        };
        
        // Serialize
        let encoded = cdr::serialize(&header, cdr::Infinite).unwrap();
        
        // Deserialize
        let decoded: Header = cdr::deserialize(&encoded).unwrap();
        
        assert_eq!(header, decoded);
    }
}
```

### Integration Tests

**Location**: `tests/` directory (separate from src/)

**Example (tests/cross_language.rs):**

```rust
/// Verify that Rust and Python produce identical CDR encoding
#[test]
fn test_rust_python_interop() {
    use edgefirst_schemas::std_msgs::Header;
    use edgefirst_schemas::builtin_interfaces::Time;
    
    let header = Header {
        stamp: Time { sec: 1234567890, nanosec: 123456789 },
        frame_id: "test_frame".to_string(),
    };
    
    // Serialize in Rust
    let rust_bytes = cdr::serialize(&header, cdr::Infinite).unwrap();
    
    // Python script deserializes the same bytes
    // (Integration test would spawn Python subprocess)
    
    // Verify round-trip
    let decoded: Header = cdr::deserialize(&rust_bytes).unwrap();
    assert_eq!(header, decoded);
}
```

### Property-Based Tests

**Using proptest for fuzz testing:**

```rust
use proptest::prelude::*;

proptest! {
    #[test]
    fn test_header_serialization_always_succeeds(
        sec in any::<i32>(),
        nanosec in any::<u32>(),
        frame_id in ".*"
    ) {
        let header = Header {
            stamp: Time { sec, nanosec },
            frame_id: frame_id.clone(),
        };
        
        let encoded = cdr::serialize(&header, cdr::Infinite).unwrap();
        let decoded: Header = cdr::deserialize(&encoded).unwrap();
        
        assert_eq!(header.stamp.sec, decoded.stamp.sec);
        assert_eq!(header.stamp.nanosec, decoded.stamp.nanosec);
        assert_eq!(header.frame_id, decoded.frame_id);
    }
}
```

### Running Rust Tests

```bash
# All unit tests
cargo test --all-features --workspace

# With coverage (HTML report)
cargo llvm-cov --all-features --workspace --html
open target/llvm-cov/html/index.html

# Generate LCOV for SonarCloud
cargo llvm-cov --all-features --workspace --lcov --output-path lcov-rust.info

# Property tests (longer runtime)
cargo test --release -- --ignored

# Specific test
cargo test test_header_round_trip_cdr

# Show test output
cargo test -- --nocapture
```

---

## C Testing

### Framework: Criterion

EdgeFirst Schemas uses [Criterion](https://github.com/Snaipe/Criterion) for C testing:

- **Automatic test registration** - No manual suite setup
- **Native output formats** - JUnit XML, JSON, TAP for CI/CD
- **Parallel execution** - Faster test runs
- **Rich assertions** - Better error messages
- **Parameterized tests** - Data-driven testing

### Test Structure

**Example (`crates/capi/tests/c/test_builtin_interfaces.c`):**

```c
#include <criterion/criterion.h>
#include <errno.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

Test(builtin_interfaces, time_encode_decode_roundtrip) {
    uint8_t buf[64];
    size_t written = 0;

    int ret = builtin_interfaces_time_encode(buf, sizeof(buf), &written, 12345, 67890);
    cr_assert_eq(ret, 0, "Encode should succeed");
    cr_assert_gt(written, 0, "Written bytes should be > 0");

    int32_t sec = 0;
    uint32_t nanosec = 0;
    ret = builtin_interfaces_time_decode(buf, written, &sec, &nanosec);
    cr_assert_eq(ret, 0, "Decode should succeed");
    cr_assert_eq(sec, 12345);
    cr_assert_eq(nanosec, 67890);
}

Test(builtin_interfaces, time_encode_null_buf_returns_einval) {
    size_t written = 0;
    int ret = builtin_interfaces_time_encode(NULL, 64, &written, 1, 2);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}
```

### Memory Leak Detection with Valgrind

```bash
# Build debug C tests, then run under Valgrind (Linux)
RELEASE=0 make test-c
valgrind --leak-check=full ./build/test_builtin_interfaces
```

Valgrind checks for:
- Memory leaks (unfreed allocations)
- Invalid memory access
- Use of uninitialized values
- Double-free errors

### Running C Tests

```bash
# From repository root (builds the Rust library, then Criterion binaries)
make test-c

# JUnit XML output (for CI/CD)
make test-c-xml

# Debug library (for coverage)
RELEASE=0 make test-c
```

### Build System (Makefile)

```makefile
# Makefile (repo root) / crates/capi/tests/c/
CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -I../../include
LDFLAGS = -L../../target/release -ledgefirst_schemas -lcriterion

test: build/test_runner
	./build/test_runner

test-coverage: CFLAGS += --coverage -fprofile-arcs -ftest-coverage
test-coverage: test
	lcov --capture --directory . --output-file coverage.info
	genhtml coverage.info --output-directory coverage-html

test-valgrind: build/test_runner
	valgrind --leak-check=full --error-exitcode=1 ./build/test_runner

clean:
	rm -rf build/ *.gcda *.gcno coverage.info coverage-html/
```

---

## Python Testing

### Framework: pytest

**Test Structure:**

```python
# tests/python/test_std_msgs.py
import pytest
from edgefirst.schemas.std_msgs import Header
from edgefirst.schemas.builtin_interfaces import Time

def test_header_default():
    """Test default header construction."""
    header = Header()
    assert header.stamp.sec == 0
    assert header.stamp.nanosec == 0
    assert header.frame_id == ""

def test_header_round_trip_cdr():
    """Test CDR serialization round-trip."""
    header = Header(
        stamp=Time(sec=1234567890, nanosec=123456789),
        frame_id="camera"
    )
    
    # Encode / decode CDR (PyO3 bindings)
    encoded = header.to_bytes()
    
    # Decode from CDR
    decoded = Header.from_cdr(encoded)
    
    assert decoded.stamp.sec == header.stamp.sec
    assert decoded.stamp.nanosec == header.stamp.nanosec
    assert decoded.frame_id == header.frame_id

@pytest.mark.parametrize("sec,nanosec,frame_id", [
    (0, 0, ""),
    (1234567890, 123456789, "camera"),
    (-1, 4294967295, "lidar"),
    (2147483647, 999999999, "world"),
])
def test_header_parameterized(sec, nanosec, frame_id):
    """Test various header configurations."""
    header = Header(
        stamp=Time(sec=sec, nanosec=nanosec),
        frame_id=frame_id
    )
    encoded = header.to_bytes()
    decoded = Header.from_cdr(encoded)
    assert decoded.stamp.sec == sec
    assert decoded.stamp.nanosec == nanosec
    assert decoded.frame_id == frame_id
```

### Property-Based Testing with Hypothesis

```python
from hypothesis import given, strategies as st
from edgefirst.schemas.geometry_msgs import Vector3

@given(
    x=st.floats(allow_nan=False, allow_infinity=False),
    y=st.floats(allow_nan=False, allow_infinity=False),
    z=st.floats(allow_nan=False, allow_infinity=False)
)
def test_vector3_round_trip(x, y, z):
    """Property test: all finite floats round-trip correctly."""
    vec = Vector3(x=x, y=y, z=z)
    encoded = vec.to_bytes()
    decoded = Vector3.from_cdr(encoded)
    
    # Allow small floating point errors
    assert abs(decoded.x - x) < 1e-6
    assert abs(decoded.y - y) < 1e-6
    assert abs(decoded.z - z) < 1e-6
```

### Type Checking with mypy

```bash
# Type check all Python code
mypy edgefirst/

# Strict mode
mypy --strict edgefirst/schemas/
```

### Running Python Tests

```bash
# All tests (excluding benchmarks)
pytest tests/python/ -m "not benchmark"

# With coverage
pytest tests/python/ -m "not benchmark" \
    --cov=edgefirst --cov-report=html
open htmlcov/index.html

# Generate coverage.xml for SonarCloud
pytest tests/python/ -m "not benchmark" \
    --cov=edgefirst --cov-report=xml

# Specific test file
pytest tests/python/test_std_msgs.py -v

# Specific test
pytest tests/python/test_std_msgs.py::test_header_default -v

# Show print output
pytest tests/python/ -s

# Type checking
mypy edgefirst/
```

---

## MCAP Testing

### Overview

MCAP tests validate that EdgeFirst Schemas can correctly parse real-world CDR-encoded
messages from MCAP recordings captured on hardware devices. This ensures the schemas
work correctly with actual production data from Maivin, Raivin, and other EdgeFirst
platforms.

### How It Works

1. **Discovery**: Tests scan `testdata/` for all `.mcap` files
2. **Schema Validation**: Verifies all message types in each MCAP are supported
3. **Deserialization**: Parses every message using the schemas library
4. **Round-Trip**: Re-serializes each message and verifies identical CDR bytes

### Test Data Location

Place MCAP files in the `testdata/` directory at the repository root:

```
schemas/
└── testdata/
    ├── device1_recording.mcap
    └── device2_recording.mcap
```

MCAP files are not committed to git. They should be:
- Recorded on EdgeFirst hardware with real sensor data
- Contain CDR-encoded messages (standard Zenoh/ROS2 encoding)
- Managed externally (Git LFS, cloud storage, etc.)

### Running MCAP Tests

```bash
# Run all MCAP tests
pytest tests/python/test_mcap.py -v

# Run with detailed message counts
pytest tests/python/test_mcap.py -v -s

# Run specific test class
pytest tests/python/test_mcap.py::TestMcapDeserialization -v -s
```

### Test Classes

| Class | Purpose |
|-------|---------|
| `TestMcapSchemaSupport` | Fails if MCAP contains unsupported schema types |
| `TestMcapDeserialization` | Deserializes every message, fails on any error |
| `TestMcapRoundTrip` | Verifies serialize/deserialize produces identical bytes |
| `TestMcapFieldValidation` | Validates timestamps and dimensions are reasonable |

### Supported Schema Types

Tests require all schemas in an MCAP file to be supported. The table below shows
cross-language support status:

| Schema | Python | Rust | Notes |
|--------|--------|------|-------|
| **sensor_msgs** |
| CameraInfo | ✅ | ✅ | |
| CompressedImage | ✅ | ✅ | |
| Image | ✅ | ✅ | |
| Imu | ✅ | ✅ | |
| NavSatFix | ✅ | ✅ | |
| PointCloud2 | ✅ | ✅ | |
| PointField | ✅ | ✅ | Nested struct in PointCloud2 |
| **geometry_msgs** |
| Transform | ✅ | ✅ | |
| TransformStamped | ✅ | ✅ | |
| Vector3 | ✅ | ✅ | |
| Quaternion | ✅ | ✅ | |
| Pose | ✅ | ✅ | |
| PoseStamped | ✅ | - | Not in Rust |
| Point | ✅ | ✅ | |
| Twist | ✅ | ✅ | |
| TwistStamped | ✅ | ✅ | |
| **foxglove_msgs** |
| CompressedVideo | ✅ | ✅ | |
| CompressedImage | ✅ | - | Not in Rust |
| FrameTransform | ✅ | - | Not in Rust |
| LocationFix | ✅ | - | Not in Rust |
| Log | ✅ | - | Not in Rust |
| PointCloud | ✅ | - | Not in Rust |
| RawImage | ✅ | - | Not in Rust |
| **edgefirst_msgs** |
| Box | ✅ | ✅ | |
| Detect | ✅ | ✅ | |
| Mask | ✅ | ✅ | |
| ModelInfo | ✅ | ✅ | |
| RadarCube | ✅ | ✅ | |
| RadarInfo | ✅ | ✅ | |
| Track | ✅ | ✅ | |

### Adding Schema Support

If tests fail due to unsupported schemas, add the mapping in **both** files:

**Python** (`tests/python/test_mcap.py`):
```python
SCHEMA_MAP: dict[str, type] = {
    "sensor_msgs/msg/NewType": sensor_msgs.NewType,
    # ...
}
```

**Rust** (`tests/mcap_test.rs`):
```rust
fn deserialize_message(schema_name: &str, data: &[u8]) -> Result<Vec<u8>, String> {
    match schema_name {
        "sensor_msgs/msg/NewType" => {
            let msg: sensor_msgs::NewType = cdr::deserialize(data)?;
            cdr::serialize::<_, _, cdr::CdrLe>(&msg, cdr::Infinite)
        }
        // ...
    }
}

fn is_schema_supported(schema_name: &str) -> bool {
    matches!(schema_name, "sensor_msgs/msg/NewType" | /* ... */)
}
```

### Multi-Language Support

MCAP tests should be implemented for each language that provides its own CDR parser:

- **Python**: `tests/python/test_mcap.py` ✅
- **Rust**: `tests/mcap_test.rs` ✅
- **C**: Not needed (uses Rust parser via FFI)

### Running Rust MCAP Tests

```bash
# Run all MCAP tests
cargo test --test mcap_test

# Run with output showing message counts
cargo test --test mcap_test -- --nocapture
```

Future language bindings should implement equivalent MCAP tests to validate their
parsers produce identical results.

---

## Benchmarks

### Overview

All message types have **THREE** benchmark variants:

1. **Serialization**: Message → CDR bytes
2. **Deserialization**: CDR bytes → Message
3. **Round-trip**: Message → CDR bytes → Message (validates correctness)

### Critical Requirements

**1. Coverage Instrumentation**

All benchmarks MUST run with coverage enabled:

- **Rust**: `RUSTFLAGS="-C instrument-coverage" cargo bench --profile bench`
- **C**: Compiled with `--coverage -fprofile-arcs -ftest-coverage -O2 -g`
- **Python**: `pytest --benchmark-only --cov=edgefirst`

**2. Profiling Mode**

Benchmarks built with optimization + debug symbols:

- **Rust**: `[profile.bench] opt-level = 3, debug = true`
- **C**: `-O2 -g` compiler flags
- **Python**: Default pytest-benchmark configuration

### Rust Benchmarks (Criterion)

**Configuration (Cargo.toml):**

```toml
[dev-dependencies]
criterion = { version = "0.5", features = ["html_reports"] }

[[bench]]
name = "serialization"
harness = false

[profile.bench]
opt-level = 3
debug = true  # Enable debug symbols for profiling
lto = "thin"
codegen-units = 1
```

**Example (benches/serialization.rs):**

```rust
use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::std_msgs::Header;

fn bench_std_msgs(c: &mut Criterion) {
    let mut group = c.benchmark_group("std_msgs");

    let stamp = Time { sec: 1234567890, nanosec: 123456789 };
    let hdr = Header::builder()
        .stamp(stamp)
        .frame_id("camera")
        .build()
        .unwrap();
    let bytes = hdr.to_cdr();
    group.throughput(Throughput::Bytes(bytes.len() as u64));

    group.bench_function("Header/builder", |b| {
        b.iter(|| {
            Header::builder()
                .stamp(black_box(stamp))
                .frame_id("camera")
                .build()
                .unwrap()
        })
    });
    group.bench_function("Header/from_cdr", |b| {
        b.iter(|| Header::from_cdr(black_box(bytes.clone())))
    });

    group.finish();
}

criterion_group!(benches, bench_std_msgs);
criterion_main!(benches);
```

**Running Rust Benchmarks:**

```bash
# Run all benchmarks
cargo bench

# Run specific benchmark group
cargo bench -- "RadarCube"
cargo bench -- "PointCloud2"
cargo bench -- "Image"
cargo bench -- "builtin_interfaces"

# Run with verbose output
cargo bench -- --verbose

# Specific benchmark
cargo bench -- "Header/serialize"

# HTML report
cargo bench
open target/criterion/report/index.html

# Save baseline for comparison
cargo bench -- --save-baseline main

# Compare against baseline
cargo bench -- --baseline main
```

### C Benchmarks (Custom Harness)

C uses a **custom benchmark harness** with `clock_gettime(CLOCK_MONOTONIC)`:

**Example (builder encode_into — preferred for hot paths):**

```c
#include <edgefirst/schemas.h>
#include <stdint.h>
#include <stdio.h>

void encode_header_into_stack_buf(void) {
    std_msgs_header_builder_t *b = std_msgs_header_builder_new();
    std_msgs_header_builder_set_stamp(b, 1234567890, 123456789);
    std_msgs_header_builder_set_frame_id(b, "camera");

    uint8_t buf[256];
    size_t out_len = 0;
    int rc = std_msgs_header_builder_encode_into(b, buf, sizeof(buf), &out_len);
    if (rc != 0) {
        /* errno set — ENOBUFS if buf too small */
    }
    std_msgs_header_builder_free(b);
    printf("encoded %zu bytes\n", out_len);
}
```

**Running C / C++ Benchmarks:** see [`benches/cpp/`](benches/cpp/) and
[`BENCHMARKS.md`](BENCHMARKS.md) (Google Benchmark competitive suite).

### Python Benchmarks (pytest-benchmark)

**Example (`benches/python/bench_native.py`):**

```python
import pytest
from edgefirst.schemas.std_msgs import Header
from edgefirst.schemas.builtin_interfaces import Time

def test_header_serialize(benchmark):
    """Benchmark header encoding."""
    header = Header(
        stamp=Time(sec=1234567890, nanosec=123456789),
        frame_id="camera"
    )

    result = benchmark(header.to_bytes)
    assert len(result) > 0
```

**Running Python Benchmarks:**

```bash
# Native EdgeFirst bindings
pytest benches/python/bench_native.py --benchmark-only

# With JSON output
pytest benches/python/bench_native.py --benchmark-only \
    --benchmark-json=benchmark_results.json
```

### Performance Targets

| Message Type | Rust | C | Python | Notes |
|-------------|------|---|--------|-------|
| **Time** (12 bytes) | <20ns | <30ns | <2µs | Primitive |
| **Header** (~50 bytes) | <100ns | <150ns | <5µs | String alloc |
| **Image 640x480** (~900KB) | <10µs | <15µs | <500µs | Memcpy heavy |
| **Image 1080p** (~6MB) | <50µs | <75µs | <2ms | Large buffer |
| **Image 4K** (~24MB) | <200µs | <300µs | <8ms | Very large |
| **PointCloud2 1K pts** | <5µs | <8µs | <100µs | Small cloud |
| **PointCloud2 100K pts** | <500µs | <800µs | <10ms | Typical LiDAR |

---

## Coverage and CI/CD

### Local Coverage

**Rust:**
```bash
# HTML report
cargo llvm-cov --all-features --workspace --html
open target/llvm-cov/html/index.html

# LCOV format (for SonarCloud)
cargo llvm-cov --all-features --workspace --lcov --output-path lcov-rust.info
```

**C:**
```bash
cd tests/c
make test-coverage
open coverage-html/index.html

# Generate LCOV
make coverage.info
```

**Python:**
```bash
# HTML report
pytest tests/python/ --cov=edgefirst --cov-report=html
open htmlcov/index.html

# XML for SonarCloud
pytest tests/python/ --cov=edgefirst --cov-report=xml
```

### GitHub Actions Workflow

**Workflow: `.github/workflows/test-and-coverage.yml`**

```yaml
name: Test and Coverage

on: [push, pull_request]

jobs:
  test-rust:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Install Rust
        uses: actions-rust-lang/setup-rust-toolchain@v1
        
      - name: Install cargo-llvm-cov
        uses: taiki-e/install-action@cargo-llvm-cov
        
      - name: Run tests with coverage
        run: |
          cargo llvm-cov --all-features --workspace \
            --lcov --output-path lcov-rust.info
          
      - name: Upload to Codecov
        uses: codecov/codecov-action@v3
        with:
          files: lcov-rust.info
          flags: rust

  test-c:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Install Criterion
        run: sudo apt-get install -y libcriterion-dev lcov
        
      - name: Build Rust library
        run: cargo build --release
        
      - name: Run C tests with coverage
        run: |
          cd tests/c
          make test-coverage
          
      - name: Upload to Codecov
        uses: codecov/codecov-action@v3
        with:
          files: lcov.info  # combined Rust+C via cargo-llvm-cov
          flags: c

  test-python:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Setup Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'
          
      - name: Install dependencies
        run: |
          pip install -e .[test]
          
      - name: Run tests with coverage
        run: |
          pytest tests/python/ -m "not benchmark" \
            --cov=edgefirst --cov-report=xml
            
      - name: Upload to Codecov
        uses: codecov/codecov-action@v3
        with:
          files: coverage.xml
          flags: python

  sonarcloud:
    runs-on: ubuntu-latest
    needs: [test-rust, test-c, test-python]
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
          
      - name: Download coverage reports
        uses: actions/download-artifact@v3
        
      - name: SonarCloud Scan
        uses: SonarSource/sonarcloud-github-action@master
        env:
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
          SONAR_TOKEN: ${{ secrets.SONAR_TOKEN }}
```

### SonarCloud Configuration

**File: `sonar-project.properties`**

```properties
sonar.organization=edgefirst
sonar.projectKey=edgefirst_schemas

# Coverage paths
sonar.coverage.exclusions=**/tests/**,**/benches/**,**/examples/**
sonar.rust.lcov.reportPaths=lcov-rust.info
sonar.coverage.reportPaths=lcov.info
sonar.python.coverage.reportPaths=coverage.xml

# Test exclusions
sonar.test.inclusions=**/tests/**,**/test_*.rs,**/test_*.c,**/test_*.py

# Rust-specific
sonar.sources=src/,include/
sonar.tests=tests/,benches/
```

---

## Test Data

### Synthetic Data Generation

For initial implementation, tests use **synthetic data**:

```rust
// Rust test data generation
fn create_test_image_640x480() -> Image {
    Image {
        header: Header::default(),
        height: 480,
        width: 640,
        encoding: "rgb8".to_string(),
        is_bigendian: 0,
        step: 640 * 3,
        data: vec![128u8; 640 * 480 * 3], // Gray image
    }
}

fn create_test_pointcloud_1k() -> PointCloud2 {
    let num_points = 1000;
    let point_step = 16; // x, y, z, intensity (4 floats)
    
    PointCloud2 {
        header: Header::default(),
        height: 1,
        width: num_points,
        fields: vec![
            PointField { name: "x".into(), offset: 0, datatype: 7, count: 1 },
            PointField { name: "y".into(), offset: 4, datatype: 7, count: 1 },
            PointField { name: "z".into(), offset: 8, datatype: 7, count: 1 },
            PointField { name: "intensity".into(), offset: 12, datatype: 7, count: 1 },
        ],
        is_bigendian: false,
        point_step,
        row_step: num_points * point_step,
        data: vec![0u8; num_points * point_step as usize],
        is_dense: true,
    }
}
```

### Future: Real Sensor Data

**Planned fixtures** (to be added in Phase 2):

- **Camera**: Real captured images from Maivin (JPEG, H.264, NV12)
- **LiDAR**: Real Ouster point cloud captures (.pcap files)
- **Radar**: Real radar cube data from smartmicro sensors
- **IMU**: Real IMU data from MPU6050/BMI160
- **GPS**: Real NavSatFix data from u-blox receivers

**Fixture location**: `tests/fixtures/`

```
tests/fixtures/
├── camera/
│   ├── 640x480_rgb8.jpg
│   ├── 1920x1080_h264.mp4
│   └── 3840x2160_nv12.raw
├── lidar/
│   ├── ouster_1k_points.pcap
│   ├── ouster_100k_points.pcap
│   └── velodyne_vlp16.pcap
├── radar/
│   └── smartmicro_cube.bin
├── imu/
│   └── mpu6050_100hz.csv
└── gps/
    └── ublox_nav_pvt.ubx
```

---

## Commands Cheat Sheet

### Rust

```bash
# Quick test
cargo test

# All tests with output
cargo test -- --nocapture

# Coverage HTML
cargo llvm-cov --all-features --workspace --html

# Coverage LCOV
cargo llvm-cov --all-features --workspace --lcov --output-path lcov-rust.info

# Benchmarks
cargo bench

# Benchmarks with coverage
RUSTFLAGS="-C instrument-coverage" cargo bench --profile bench
```

### C

```bash
cd tests/c

# Quick test
make test

# With coverage
make test-coverage

# Memory check
make test-valgrind

# Benchmarks
make bench

# Clean
make clean
```

### Python

```bash
# Quick test
pytest tests/python/ -m "not benchmark"

# With coverage
pytest tests/python/ -m "not benchmark" --cov=edgefirst --cov-report=html

# Type check
mypy edgefirst/

# Benchmarks
pytest benches/python/bench_native.py --benchmark-only

# Benchmarks with JSON
pytest benches/python/bench_native.py --benchmark-only --benchmark-json=results.json
```

### Full CI/CD Simulation

```bash
# Run everything locally before pushing
./scripts/run_all_tests.sh

# Or manually:
cargo llvm-cov --all-features --workspace --lcov --output-path lcov-rust.info
cd tests/c && make test-coverage && cd ../..
pytest tests/python/ --cov=edgefirst --cov-report=xml
```

---

## Additional Resources

- **[EdgeFirst Samples](https://github.com/EdgeFirstAI/samples)**: Real-world usage examples
- **[Criterion Documentation](https://criterion.readthedocs.io/)**: C test framework
- **[pytest Documentation](https://docs.pytest.org/)**: Python testing
- **[cargo-llvm-cov](https://github.com/taiki-e/cargo-llvm-cov)**: Rust coverage tool
- **[SonarCloud Docs](https://docs.sonarcloud.io/)**: Code quality platform

---

**For questions or issues:** support@au-zone.com
