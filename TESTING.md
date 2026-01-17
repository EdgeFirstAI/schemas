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
pytest tests/python/benchmarks/ --benchmark-only
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
| **FFI (src/ffi.rs)** | 70% | Tested via C tests |
| **CDR encode/decode** | 95% | Serialization critical |

---

## Test Architecture

### Test Organization

```
schemas/
├── src/                          # Rust library
│   ├── lib.rs                    # Unit tests inline
│   ├── std_msgs.rs               # #[cfg(test)] mod tests
│   ├── sensor_msgs.rs
│   └── ffi.rs                    # Tested via C
│
├── benches/                      # Rust benchmarks (Criterion)
│   └── serialization.rs          # All benchmarks in single file
│
├── tests/c/                      # C tests and benchmarks
│   ├── Makefile                  # Build system
│   ├── test_builtin_interfaces.c # Criterion tests
│   ├── test_std_msgs.c
│   ├── test_geometry_msgs.c
│   ├── test_sensor_msgs.c
│   ├── test_edgefirst_msgs.c
│   ├── bench_helpers.h           # Benchmark utilities
│   ├── bench_runner.c            # Benchmark orchestration
│   ├── bench_serialization.c     # Serialize benchmarks
│   └── bench_deserialization.c   # Deserialize benchmarks
│
└── tests/python/                 # Python tests
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

**Example (tests/c/test_builtin_interfaces.c):**

```c
#include <criterion/criterion.h>
#include <edgefirst/schemas.h>

Test(builtin_interfaces, time_default) {
    RosTime* time = ros_time_new();
    
    cr_assert_not_null(time);
    cr_assert_eq(ros_time_get_sec(time), 0);
    cr_assert_eq(ros_time_get_nanosec(time), 0);
    
    ros_time_free(time);
}

Test(builtin_interfaces, time_round_trip_cdr) {
    RosTime* time = ros_time_new();
    ros_time_set_sec(time, 1234567890);
    ros_time_set_nanosec(time, 123456789);
    
    // Serialize
    uint8_t* bytes = NULL;
    size_t len = 0;
    EdgeFirstResult result = ros_time_serialize(time, &bytes, &len);
    cr_assert_eq(result, EDGEFIRST_OK);
    cr_assert_not_null(bytes);
    cr_assert_gt(len, 0);
    
    // Deserialize
    RosTime* decoded = ros_time_deserialize(bytes, len, &result);
    cr_assert_eq(result, EDGEFIRST_OK);
    cr_assert_not_null(decoded);
    cr_assert_eq(ros_time_get_sec(decoded), 1234567890);
    cr_assert_eq(ros_time_get_nanosec(decoded), 123456789);
    
    // Cleanup
    free(bytes);
    ros_time_free(time);
    ros_time_free(decoded);
}

Test(builtin_interfaces, time_serialize_null_checks) {
    uint8_t* bytes = NULL;
    size_t len = 0;
    
    EdgeFirstResult result = ros_time_serialize(NULL, &bytes, &len);
    cr_assert_eq(result, EDGEFIRST_ERROR_NULL_POINTER);
}
```

### Memory Leak Detection with Valgrind

```bash
cd tests/c
make test-valgrind
```

Valgrind checks for:
- Memory leaks (unfreed allocations)
- Invalid memory access
- Use of uninitialized values
- Double-free errors

### Running C Tests

```bash
cd tests/c

# Build and run all tests
make test

# With coverage
make test-coverage
open coverage-html/index.html

# JUnit XML output (for CI/CD)
make test-xml

# JSON output
make test-json

# Valgrind memory check
make test-valgrind

# Specific test suite
./build/test_builtin_interfaces

# Clean build artifacts
make clean
```

### Build System (Makefile)

```makefile
# tests/c/Makefile
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
    
    # Serialize to CDR
    encoded = header.serialize()
    
    # Deserialize from CDR
    decoded = Header.deserialize(encoded)
    
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
    encoded = header.serialize()
    decoded = Header.deserialize(encoded)
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
    """Property test: all finite floats serialize correctly."""
    vec = Vector3(x=x, y=y, z=z)
    encoded = vec.serialize()
    decoded = Vector3.deserialize(encoded)
    
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
use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId, Throughput};
use edgefirst_schemas::std_msgs::Header;
use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::serde_cdr::{serialize, deserialize};

fn bench_std_msgs(c: &mut Criterion) {
    let mut group = c.benchmark_group("std_msgs");
    
    // Header
    let header = Header {
        stamp: Time { sec: 1234567890, nanosec: 123456789 },
        frame_id: "camera".to_string(),
    };
    let header_bytes = serialize(&header).unwrap();
    group.throughput(Throughput::Bytes(header_bytes.len() as u64));
    
    group.bench_function("Header/serialize", |b| {
        b.iter(|| serialize(black_box(&header)))
    });
    group.bench_function("Header/deserialize", |b| {
        b.iter(|| deserialize::<Header>(black_box(&header_bytes)))
    });
    
    group.finish();
}

fn bench_heavy_messages(c: &mut Criterion) {
    use edgefirst_schemas::sensor_msgs::Image;
    
    let mut group = c.benchmark_group("Image");
    
    // VGA RGB
    let image = Image {
        header: Header::default(),
        height: 480,
        width: 640,
        encoding: "rgb8".to_string(),
        is_bigendian: 0,
        step: 640 * 3,
        data: vec![0u8; 640 * 480 * 3], // ~900KB
    };
    
    group.throughput(Throughput::Bytes(image.data.len() as u64));
    group.bench_function("serialize/VGA_rgb8", |b| {
        b.iter(|| serialize(black_box(&image)))
    });
    
    group.finish();
}

criterion_group!(benches, bench_std_msgs, bench_heavy_messages);
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

**Example (tests/c/bench_serialization.c):**

```c
#include "bench_helpers.h"
#include <edgefirst/schemas.h>

void bench_header_serialize(BenchmarkResults* results) {
    RosHeader* header = ros_header_new();
    ros_header_set_frame_id(header, "camera");
    
    RosTime* stamp = ros_header_get_stamp_mut(header);
    ros_time_set_sec(stamp, 1234567890);
    ros_time_set_nanosec(stamp, 123456789);
    
    uint8_t* bytes = NULL;
    size_t len = 0;
    
    const int iterations = 10000;
    struct timespec start, end;
    
    clock_gettime(CLOCK_MONOTONIC, &start);
    for (int i = 0; i < iterations; i++) {
        ros_header_serialize(header, &bytes, &len);
        free(bytes);
        bytes = NULL;
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    
    results->total_ns = timespec_diff_ns(&start, &end);
    results->iterations = iterations;
    results->avg_ns = results->total_ns / iterations;
    
    ros_header_free(header);
}
```

**Running C Benchmarks:**

```bash
cd tests/c

# Run all benchmarks
make bench

# JSON output for CI/CD
make bench-json

# With coverage
make bench-coverage
```

### Python Benchmarks (pytest-benchmark)

**Example (tests/python/benchmarks/test_serialize.py):**

```python
import pytest
from edgefirst.schemas.std_msgs import Header
from edgefirst.schemas.builtin_interfaces import Time

def test_header_serialize(benchmark):
    """Benchmark header serialization."""
    header = Header(
        stamp=Time(sec=1234567890, nanosec=123456789),
        frame_id="camera"
    )
    
    result = benchmark(header.serialize)
    assert len(result) > 0

@pytest.mark.slow
def test_image_serialize_640x480(benchmark):
    """Benchmark 640x480 image serialization."""
    from edgefirst.schemas.sensor_msgs import Image
    
    image = Image(
        header=Header(),
        height=480,
        width=640,
        encoding="rgb8",
        is_bigendian=0,
        step=640 * 3,
        data=bytes(640 * 480 * 3)  # ~900KB
    )
    
    result = benchmark(image.serialize)
    assert len(result) > 0
```

**Running Python Benchmarks:**

```bash
# All benchmarks
pytest tests/python/benchmarks/ --benchmark-only

# With JSON output
pytest tests/python/benchmarks/ --benchmark-only \
    --benchmark-json=benchmark_results.json

# Exclude slow tests
pytest tests/python/benchmarks/ --benchmark-only -m "not slow"

# With coverage
pytest tests/python/benchmarks/ --benchmark-only \
    --cov=edgefirst --cov-report=html

# Compare results
pytest-benchmark compare 0001 0002 --group-by=func
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
          files: tests/c/coverage.info
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
sonar.c.coverage.reportPaths=tests/c/coverage.info
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
pytest tests/python/benchmarks/ --benchmark-only

# Benchmarks with JSON
pytest tests/python/benchmarks/ --benchmark-only --benchmark-json=results.json
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
