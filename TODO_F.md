# TODO Phase F: Cross-Language Validation

**Phase:** F  
**Status:** Not Started  
**Estimate:** 8-10 hours  
**Dependencies:** Phase A (Rust Benchmarks), Phase B (C API), Phase D (Python Tests), Phase E (Python Benchmarks)  
**Blocks:** None (final validation phase)

---

## Objective

Validate that CDR serialization is byte-compatible across all three language implementations (Rust, C, Python), ensuring messages serialized in one language can be correctly deserialized in another.

---

## Background

### Why Cross-Language Validation Matters

EdgeFirst Perception uses multiple languages in production:
- **Rust**: Core library, high-performance processing
- **Python**: Prototyping, ML integration, EdgeFirst Studio
- **C**: Embedded systems, legacy integration

Messages are exchanged via Zenoh, which transmits raw CDR bytes. If serialization differs between languages, messages become unreadable across language boundaries.

### Potential Compatibility Issues

| Issue | Cause | Impact |
|-------|-------|--------|
| Byte order | BE vs LE CDR encoding | Complete data corruption |
| Padding | Alignment differences | Field offset errors |
| String encoding | UTF-8 vs ASCII | String corruption |
| Array length | Length prefix format | Array truncation |
| Nested messages | Recursive serialization | Nested field corruption |

---

## Deliverables

### F.1 Cross-Language Test Infrastructure

#### F.1.1 Test Data Generator (Rust)

Generate canonical test data from Rust:

**File:** `tests/cross_language/generate_test_data.rs`

```rust
//! Generate canonical CDR test data for cross-language validation.
//! 
//! Run with: cargo run --example generate_test_data

use edgefirst_schemas::*;
use std::fs;
use std::path::Path;

fn main() {
    let output_dir = Path::new("tests/cross_language/data");
    fs::create_dir_all(output_dir).expect("Failed to create output directory");

    // Generate test data for each message type
    generate_time(output_dir);
    generate_duration(output_dir);
    generate_header(output_dir);
    generate_vector3(output_dir);
    generate_point(output_dir);
    generate_quaternion(output_dir);
    generate_pose(output_dir);
    generate_point_cloud2(output_dir);
    generate_radar_cube(output_dir);
    generate_compressed_video(output_dir);
    generate_dmabuf(output_dir);
    generate_detect(output_dir);

    println!("Generated test data in {:?}", output_dir);
}

fn generate_time(dir: &Path) {
    let msg = builtin_interfaces::Time {
        sec: 1234567890,
        nanosec: 123456789,
    };
    let bytes = serde_cdr::serialize(&msg).unwrap();
    
    // Save binary CDR
    fs::write(dir.join("time.cdr"), &bytes).unwrap();
    
    // Save JSON metadata for verification
    let metadata = serde_json::json!({
        "type": "builtin_interfaces/Time",
        "sec": msg.sec,
        "nanosec": msg.nanosec,
        "cdr_length": bytes.len(),
    });
    fs::write(
        dir.join("time.json"),
        serde_json::to_string_pretty(&metadata).unwrap()
    ).unwrap();
}

fn generate_header(dir: &Path) {
    let msg = std_msgs::Header {
        stamp: builtin_interfaces::Time {
            sec: 1234567890,
            nanosec: 123456789,
        },
        frame_id: "test_frame".to_string(),
    };
    let bytes = serde_cdr::serialize(&msg).unwrap();
    
    fs::write(dir.join("header.cdr"), &bytes).unwrap();
    
    let metadata = serde_json::json!({
        "type": "std_msgs/Header",
        "stamp": { "sec": msg.stamp.sec, "nanosec": msg.stamp.nanosec },
        "frame_id": msg.frame_id,
        "cdr_length": bytes.len(),
    });
    fs::write(
        dir.join("header.json"),
        serde_json::to_string_pretty(&metadata).unwrap()
    ).unwrap();
}

fn generate_point_cloud2(dir: &Path) {
    let msg = sensor_msgs::PointCloud2 {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 100, nanosec: 0 },
            frame_id: "lidar".to_string(),
        },
        height: 1,
        width: 10,
        fields: vec![
            sensor_msgs::PointField {
                name: "x".to_string(),
                offset: 0,
                datatype: 7,
                count: 1,
            },
            sensor_msgs::PointField {
                name: "y".to_string(),
                offset: 4,
                datatype: 7,
                count: 1,
            },
            sensor_msgs::PointField {
                name: "z".to_string(),
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ],
        is_bigendian: false,
        point_step: 12,
        row_step: 120,
        data: vec![0u8; 120],
        is_dense: true,
    };
    let bytes = serde_cdr::serialize(&msg).unwrap();
    
    fs::write(dir.join("point_cloud2.cdr"), &bytes).unwrap();
    
    let metadata = serde_json::json!({
        "type": "sensor_msgs/PointCloud2",
        "width": msg.width,
        "height": msg.height,
        "point_step": msg.point_step,
        "fields_count": msg.fields.len(),
        "data_length": msg.data.len(),
        "cdr_length": bytes.len(),
    });
    fs::write(
        dir.join("point_cloud2.json"),
        serde_json::to_string_pretty(&metadata).unwrap()
    ).unwrap();
}

fn generate_radar_cube(dir: &Path) {
    let shape = vec![4u16, 16, 4, 8];  // Small cube for testing
    let total: usize = shape.iter().map(|&x| x as usize).product();
    
    let msg = edgefirst_msgs::RadarCube {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 200, nanosec: 0 },
            frame_id: "radar".to_string(),
        },
        timestamp: 1234567890123456u64,
        layout: vec![6, 1, 5, 2],  // SEQUENCE, RANGE, RXCHANNEL, DOPPLER
        shape: shape.clone(),
        scales: vec![1.0f32, 2.5, 1.0, 0.5],
        cube: vec![42i16; total],  // Fill with recognizable value
        is_complex: false,
    };
    let bytes = serde_cdr::serialize(&msg).unwrap();
    
    fs::write(dir.join("radar_cube.cdr"), &bytes).unwrap();
    
    let metadata = serde_json::json!({
        "type": "edgefirst_msgs/RadarCube",
        "timestamp": msg.timestamp,
        "shape": msg.shape,
        "layout": msg.layout,
        "scales": msg.scales,
        "cube_length": msg.cube.len(),
        "is_complex": msg.is_complex,
        "cdr_length": bytes.len(),
    });
    fs::write(
        dir.join("radar_cube.json"),
        serde_json::to_string_pretty(&metadata).unwrap()
    ).unwrap();
}

fn generate_compressed_video(dir: &Path) {
    let msg = foxglove_msgs::FoxgloveCompressedVideo {
        header: std_msgs::Header {
            stamp: builtin_interfaces::Time { sec: 300, nanosec: 0 },
            frame_id: "camera".to_string(),
        },
        data: vec![0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE],  // Recognizable pattern
        format: "h264".to_string(),
    };
    let bytes = serde_cdr::serialize(&msg).unwrap();
    
    fs::write(dir.join("compressed_video.cdr"), &bytes).unwrap();
    
    let metadata = serde_json::json!({
        "type": "foxglove_msgs/CompressedVideo",
        "format": msg.format,
        "data_length": msg.data.len(),
        "data_hex": msg.data.iter().map(|b| format!("{:02x}", b)).collect::<Vec<_>>().join(""),
        "cdr_length": bytes.len(),
    });
    fs::write(
        dir.join("compressed_video.json"),
        serde_json::to_string_pretty(&metadata).unwrap()
    ).unwrap();
}

// Add similar functions for other message types...
fn generate_duration(dir: &Path) { /* ... */ }
fn generate_vector3(dir: &Path) { /* ... */ }
fn generate_point(dir: &Path) { /* ... */ }
fn generate_quaternion(dir: &Path) { /* ... */ }
fn generate_pose(dir: &Path) { /* ... */ }
fn generate_dmabuf(dir: &Path) { /* ... */ }
fn generate_detect(dir: &Path) { /* ... */ }
```

---

#### F.1.2 Python Validation Tests

**File:** `tests/python/test_cross_language.py`

```python
"""Cross-language CDR validation tests.

These tests verify that CDR data serialized by Rust can be correctly
deserialized by Python, and vice versa.
"""

import json
import pytest
from pathlib import Path
from edgefirst.schemas import (
    builtin_interfaces,
    std_msgs,
    geometry_msgs,
    sensor_msgs,
    edgefirst_msgs,
    foxglove_msgs,
)


# Path to test data generated by Rust
TEST_DATA_DIR = Path(__file__).parent.parent.parent / "cross_language" / "data"


def load_test_data(name: str) -> tuple[bytes, dict]:
    """Load CDR bytes and metadata for a test case."""
    cdr_path = TEST_DATA_DIR / f"{name}.cdr"
    json_path = TEST_DATA_DIR / f"{name}.json"
    
    if not cdr_path.exists():
        pytest.skip(f"Test data not found: {cdr_path}")
    
    cdr_data = cdr_path.read_bytes()
    metadata = json.loads(json_path.read_text())
    
    return cdr_data, metadata


class TestCrossLanguageBuiltinInterfaces:
    """Cross-language tests for builtin_interfaces."""

    def test_time_rust_to_python(self):
        """Deserialize Rust-serialized Time in Python."""
        cdr_data, metadata = load_test_data("time")
        
        time = builtin_interfaces.Time.deserialize(cdr_data)
        
        assert time.sec == metadata["sec"]
        assert time.nanosec == metadata["nanosec"]

    def test_time_python_to_rust_compatible(self):
        """Verify Python serialization matches Rust format."""
        cdr_data, metadata = load_test_data("time")
        
        # Create equivalent message in Python
        time = builtin_interfaces.Time(
            sec=metadata["sec"],
            nanosec=metadata["nanosec"],
        )
        
        # Serialize with Python
        python_cdr = time.serialize()
        
        # Should produce identical bytes
        assert python_cdr == cdr_data, (
            f"CDR mismatch: Python={python_cdr.hex()}, Rust={cdr_data.hex()}"
        )

    def test_duration_rust_to_python(self):
        """Deserialize Rust-serialized Duration in Python."""
        cdr_data, metadata = load_test_data("duration")
        
        duration = builtin_interfaces.Duration.deserialize(cdr_data)
        
        assert duration.sec == metadata["sec"]
        assert duration.nanosec == metadata["nanosec"]


class TestCrossLanguageStdMsgs:
    """Cross-language tests for std_msgs."""

    def test_header_rust_to_python(self):
        """Deserialize Rust-serialized Header in Python."""
        cdr_data, metadata = load_test_data("header")
        
        header = std_msgs.Header.deserialize(cdr_data)
        
        assert header.frame_id == metadata["frame_id"]
        assert header.stamp.sec == metadata["stamp"]["sec"]
        assert header.stamp.nanosec == metadata["stamp"]["nanosec"]

    def test_header_python_to_rust_compatible(self):
        """Verify Python Header serialization matches Rust format."""
        cdr_data, metadata = load_test_data("header")
        
        header = std_msgs.Header(
            stamp=builtin_interfaces.Time(
                sec=metadata["stamp"]["sec"],
                nanosec=metadata["stamp"]["nanosec"],
            ),
            frame_id=metadata["frame_id"],
        )
        
        python_cdr = header.serialize()
        assert python_cdr == cdr_data


class TestCrossLanguageSensorMsgs:
    """Cross-language tests for sensor_msgs."""

    def test_point_cloud2_rust_to_python(self):
        """Deserialize Rust-serialized PointCloud2 in Python."""
        cdr_data, metadata = load_test_data("point_cloud2")
        
        cloud = sensor_msgs.PointCloud2.deserialize(cdr_data)
        
        assert cloud.width == metadata["width"]
        assert cloud.height == metadata["height"]
        assert cloud.point_step == metadata["point_step"]
        assert len(cloud.fields) == metadata["fields_count"]
        assert len(cloud.data) == metadata["data_length"]

    def test_point_cloud2_roundtrip(self):
        """Verify PointCloud2 survives Python serialize/deserialize."""
        cdr_data, _ = load_test_data("point_cloud2")
        
        # Deserialize Rust data
        cloud1 = sensor_msgs.PointCloud2.deserialize(cdr_data)
        
        # Serialize with Python
        python_cdr = cloud1.serialize()
        
        # Deserialize again
        cloud2 = sensor_msgs.PointCloud2.deserialize(python_cdr)
        
        # Verify equivalence
        assert cloud1.width == cloud2.width
        assert cloud1.height == cloud2.height
        assert cloud1.data == cloud2.data


class TestCrossLanguageEdgeFirstMsgs:
    """Cross-language tests for edgefirst_msgs."""

    def test_radar_cube_rust_to_python(self):
        """Deserialize Rust-serialized RadarCube in Python."""
        cdr_data, metadata = load_test_data("radar_cube")
        
        cube = edgefirst_msgs.RadarCube.deserialize(cdr_data)
        
        assert cube.timestamp == metadata["timestamp"]
        assert list(cube.shape) == metadata["shape"]
        assert list(cube.layout) == metadata["layout"]
        assert len(cube.cube) == metadata["cube_length"]
        assert cube.is_complex == metadata["is_complex"]
        # Verify scale values with tolerance
        for actual, expected in zip(cube.scales, metadata["scales"]):
            assert actual == pytest.approx(expected)

    def test_radar_cube_python_to_rust_compatible(self):
        """Verify Python RadarCube serialization matches Rust format."""
        cdr_data, metadata = load_test_data("radar_cube")
        
        cube = edgefirst_msgs.RadarCube(
            header=std_msgs.Header(
                stamp=builtin_interfaces.Time(sec=200, nanosec=0),
                frame_id="radar",
            ),
            timestamp=metadata["timestamp"],
            layout=metadata["layout"],
            shape=metadata["shape"],
            scales=metadata["scales"],
            cube=[42] * metadata["cube_length"],
            is_complex=metadata["is_complex"],
        )
        
        python_cdr = cube.serialize()
        assert python_cdr == cdr_data


class TestCrossLanguageFoxgloveMsgs:
    """Cross-language tests for foxglove_msgs."""

    def test_compressed_video_rust_to_python(self):
        """Deserialize Rust-serialized CompressedVideo in Python."""
        cdr_data, metadata = load_test_data("compressed_video")
        
        video = foxglove_msgs.FoxgloveCompressedVideo.deserialize(cdr_data)
        
        assert video.format == metadata["format"]
        assert len(video.data) == metadata["data_length"]
        
        # Verify exact data bytes
        expected_data = bytes.fromhex(metadata["data_hex"])
        assert video.data == expected_data

    def test_compressed_video_python_to_rust_compatible(self):
        """Verify Python CompressedVideo serialization matches Rust format."""
        cdr_data, metadata = load_test_data("compressed_video")
        
        video = foxglove_msgs.FoxgloveCompressedVideo(
            header=std_msgs.Header(
                stamp=builtin_interfaces.Time(sec=300, nanosec=0),
                frame_id="camera",
            ),
            data=bytes.fromhex(metadata["data_hex"]),
            format=metadata["format"],
        )
        
        python_cdr = video.serialize()
        assert python_cdr == cdr_data
```

---

#### F.1.3 C Validation Tests

**File:** `tests/c/test_cross_language.c`

```c
/**
 * Cross-language CDR validation tests for C.
 * 
 * Tests that CDR data serialized by Rust can be correctly deserialized by C.
 */

#include <criterion/criterion.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "edgefirst/schemas.h"

/* Helper to load test data from file */
static uint8_t* load_test_data(const char* name, size_t* out_len) {
    char path[256];
    snprintf(path, sizeof(path), "../cross_language/data/%s.cdr", name);
    
    FILE* f = fopen(path, "rb");
    if (!f) {
        cr_log_warn("Test data not found: %s", path);
        return NULL;
    }
    
    fseek(f, 0, SEEK_END);
    *out_len = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    uint8_t* data = malloc(*out_len);
    fread(data, 1, *out_len, f);
    fclose(f);
    
    return data;
}

/* builtin_interfaces tests */

Test(cross_language, time_rust_to_c) {
    size_t len;
    uint8_t* cdr_data = load_test_data("time", &len);
    cr_skip_if(!cdr_data, "Test data not available");
    
    RosTime* time = ros_time_deserialize(cdr_data, len);
    cr_assert_not_null(time);
    
    cr_assert_eq(ros_time_get_sec(time), 1234567890);
    cr_assert_eq(ros_time_get_nanosec(time), 123456789);
    
    ros_time_free(time);
    free(cdr_data);
}

Test(cross_language, time_c_to_rust_compatible) {
    size_t len;
    uint8_t* rust_cdr = load_test_data("time", &len);
    cr_skip_if(!rust_cdr, "Test data not available");
    
    /* Create equivalent message in C */
    RosTime* time = ros_time_new();
    ros_time_set_sec(time, 1234567890);
    ros_time_set_nanosec(time, 123456789);
    
    /* Serialize with C */
    size_t c_len;
    uint8_t* c_cdr = ros_time_serialize(time, &c_len);
    cr_assert_not_null(c_cdr);
    
    /* Should produce identical bytes */
    cr_assert_eq(c_len, len, "CDR length mismatch: C=%zu, Rust=%zu", c_len, len);
    cr_assert_arr_eq(c_cdr, rust_cdr, len, "CDR bytes differ");
    
    edgefirst_free_bytes(c_cdr);
    ros_time_free(time);
    free(rust_cdr);
}

/* std_msgs tests */

Test(cross_language, header_rust_to_c) {
    size_t len;
    uint8_t* cdr_data = load_test_data("header", &len);
    cr_skip_if(!cdr_data, "Test data not available");
    
    RosHeader* header = ros_header_deserialize(cdr_data, len);
    cr_assert_not_null(header);
    
    char* frame_id = ros_header_get_frame_id(header);
    cr_assert_str_eq(frame_id, "test_frame");
    
    RosTime* stamp = ros_header_get_stamp(header);
    cr_assert_eq(ros_time_get_sec(stamp), 1234567890);
    
    free(frame_id);
    ros_header_free(header);
    free(cdr_data);
}

/* sensor_msgs tests */

Test(cross_language, point_cloud2_rust_to_c) {
    size_t len;
    uint8_t* cdr_data = load_test_data("point_cloud2", &len);
    cr_skip_if(!cdr_data, "Test data not available");
    
    RosPointCloud2* cloud = ros_point_cloud2_deserialize(cdr_data, len);
    cr_assert_not_null(cloud);
    
    cr_assert_eq(ros_point_cloud2_get_width(cloud), 10);
    cr_assert_eq(ros_point_cloud2_get_height(cloud), 1);
    cr_assert_eq(ros_point_cloud2_get_point_step(cloud), 12);
    cr_assert_eq(ros_point_cloud2_get_fields_len(cloud), 3);
    
    ros_point_cloud2_free(cloud);
    free(cdr_data);
}

/* edgefirst_msgs tests */

Test(cross_language, radar_cube_rust_to_c) {
    size_t len;
    uint8_t* cdr_data = load_test_data("radar_cube", &len);
    cr_skip_if(!cdr_data, "Test data not available");
    
    EdgeFirstRadarCube* cube = edgefirst_radar_cube_deserialize(cdr_data, len);
    cr_assert_not_null(cube);
    
    cr_assert_eq(edgefirst_radar_cube_get_timestamp(cube), 1234567890123456ULL);
    cr_assert_eq(edgefirst_radar_cube_get_is_complex(cube), false);
    
    /* Verify shape */
    size_t shape_len = edgefirst_radar_cube_get_shape_len(cube);
    cr_assert_eq(shape_len, 4);
    
    const uint16_t* shape = edgefirst_radar_cube_get_shape(cube);
    cr_assert_eq(shape[0], 4);
    cr_assert_eq(shape[1], 16);
    cr_assert_eq(shape[2], 4);
    cr_assert_eq(shape[3], 8);
    
    edgefirst_radar_cube_free(cube);
    free(cdr_data);
}

/* foxglove_msgs tests */

Test(cross_language, compressed_video_rust_to_c) {
    size_t len;
    uint8_t* cdr_data = load_test_data("compressed_video", &len);
    cr_skip_if(!cdr_data, "Test data not available");
    
    FoxgloveCompressedVideo* video = foxglove_compressed_video_deserialize(cdr_data, len);
    cr_assert_not_null(video);
    
    char* format = foxglove_compressed_video_get_format(video);
    cr_assert_str_eq(format, "h264");
    
    size_t data_len = foxglove_compressed_video_get_data_len(video);
    cr_assert_eq(data_len, 6);
    
    const uint8_t* data = foxglove_compressed_video_get_data(video);
    cr_assert_eq(data[0], 0xDE);
    cr_assert_eq(data[1], 0xAD);
    cr_assert_eq(data[2], 0xBE);
    cr_assert_eq(data[3], 0xEF);
    
    free(format);
    foxglove_compressed_video_free(video);
    free(cdr_data);
}
```

---

### F.2 Automated Validation Script

**File:** `scripts/run_cross_language_tests.sh`

```bash
#!/bin/bash
# Run cross-language validation tests
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."

echo "=== EdgeFirst Schemas Cross-Language Validation ==="

# Step 1: Generate test data from Rust
echo ""
echo "Step 1: Generating test data from Rust..."
cd "$PROJECT_ROOT"

# Create example binary to generate test data
if [ ! -f "examples/generate_test_data.rs" ]; then
    echo "Creating test data generator..."
    mkdir -p examples
    # The generator code would be here or in tests/cross_language/
fi

cargo build --example generate_test_data 2>/dev/null || {
    echo "Note: Test data generator not built. Using existing test data if available."
}

if [ -f "target/debug/examples/generate_test_data" ]; then
    ./target/debug/examples/generate_test_data
fi

# Verify test data exists
if [ ! -d "tests/cross_language/data" ]; then
    echo "ERROR: Test data directory not found. Run Rust generator first."
    exit 1
fi

TEST_FILES=$(find tests/cross_language/data -name "*.cdr" 2>/dev/null | wc -l)
echo "Found $TEST_FILES CDR test files"

# Step 2: Run Python validation
echo ""
echo "Step 2: Running Python cross-language tests..."
cd "$PROJECT_ROOT"
pytest tests/python/test_cross_language.py -v --tb=short

# Step 3: Run C validation
echo ""
echo "Step 3: Running C cross-language tests..."
cd "$PROJECT_ROOT/tests/c"
make test_cross_language 2>/dev/null || {
    echo "Building C cross-language tests..."
    make all
}
./test_cross_language --verbose

# Step 4: Summary
echo ""
echo "=== Cross-Language Validation Complete ==="
echo "All tests passed! CDR serialization is compatible across Rust, Python, and C."
```

---

### F.3 Performance Comparison Report

#### F.3.1 Benchmark Comparison Script

**File:** `scripts/compare_benchmarks.py`

```python
#!/usr/bin/env python3
"""Compare Rust and Python benchmark results.

Usage:
    python scripts/compare_benchmarks.py \
        --rust target/criterion/report/index.html \
        --python benchmark_results.json
"""

import argparse
import json
import re
from pathlib import Path


def parse_criterion_results(criterion_dir: Path) -> dict:
    """Parse Criterion benchmark results."""
    results = {}
    
    # Criterion stores results in JSON files
    for bench_dir in criterion_dir.glob("*/new"):
        if not bench_dir.is_dir():
            continue
        
        estimates_file = bench_dir / "estimates.json"
        if estimates_file.exists():
            with open(estimates_file) as f:
                data = json.load(f)
                bench_name = bench_dir.parent.name
                results[bench_name] = {
                    "mean_ns": data["mean"]["point_estimate"],
                    "std_dev_ns": data["std_dev"]["point_estimate"],
                }
    
    return results


def parse_pytest_benchmark(json_file: Path) -> dict:
    """Parse pytest-benchmark results."""
    with open(json_file) as f:
        data = json.load(f)
    
    results = {}
    for bench in data.get("benchmarks", []):
        name = bench["name"]
        stats = bench["stats"]
        results[name] = {
            "mean_ns": stats["mean"] * 1e9,  # Convert to nanoseconds
            "std_dev_ns": stats["stddev"] * 1e9,
        }
    
    return results


def generate_comparison_report(rust_results: dict, python_results: dict) -> str:
    """Generate markdown comparison report."""
    lines = [
        "# EdgeFirst Schemas Performance Comparison",
        "",
        "## Rust vs Python CDR Serialization",
        "",
        "| Message Type | Rust (μs) | Python (μs) | Ratio (Python/Rust) |",
        "|--------------|-----------|-------------|---------------------|",
    ]
    
    # Match up benchmarks by name (approximate matching)
    for py_name, py_stats in sorted(python_results.items()):
        # Try to find corresponding Rust benchmark
        rust_match = None
        for rust_name in rust_results:
            if similar_name(rust_name, py_name):
                rust_match = rust_name
                break
        
        if rust_match:
            rust_mean = rust_results[rust_match]["mean_ns"] / 1000  # μs
            py_mean = py_stats["mean_ns"] / 1000  # μs
            ratio = py_mean / rust_mean if rust_mean > 0 else float('inf')
            
            lines.append(
                f"| {py_name} | {rust_mean:.2f} | {py_mean:.2f} | {ratio:.1f}x |"
            )
        else:
            py_mean = py_stats["mean_ns"] / 1000
            lines.append(f"| {py_name} | N/A | {py_mean:.2f} | - |")
    
    lines.extend([
        "",
        "## Analysis",
        "",
        "**Expected Performance Characteristics:**",
        "- Python is typically 5-20x slower than Rust for CDR operations",
        "- Large messages (MB+) may show smaller ratio due to memory copy dominance",
        "- Python 3.11+ shows improved performance over 3.8",
        "",
        "**Recommendations:**",
        "- Use Rust for high-throughput production deployments",
        "- Python is acceptable for development and low-frequency messages",
        "- Consider Rust FFI for Python if performance is critical",
    ])
    
    return "\n".join(lines)


def similar_name(rust_name: str, python_name: str) -> bool:
    """Check if benchmark names refer to same operation."""
    # Normalize names
    rust_norm = rust_name.lower().replace("_", "").replace("-", "")
    python_norm = python_name.lower().replace("_", "").replace("-", "")
    
    # Check for common patterns
    return (
        rust_norm in python_norm or
        python_norm in rust_norm or
        # Handle specific mappings
        ("radarcube" in rust_norm and "radarcube" in python_norm) or
        ("pointcloud" in rust_norm and "pointcloud" in python_norm)
    )


def main():
    parser = argparse.ArgumentParser(description="Compare Rust and Python benchmarks")
    parser.add_argument("--rust", type=Path, help="Criterion results directory")
    parser.add_argument("--python", type=Path, required=True, help="pytest-benchmark JSON")
    parser.add_argument("--output", type=Path, default=Path("benchmark_comparison.md"))
    args = parser.parse_args()
    
    rust_results = {}
    if args.rust and args.rust.exists():
        rust_results = parse_criterion_results(args.rust)
    
    python_results = parse_pytest_benchmark(args.python)
    
    report = generate_comparison_report(rust_results, python_results)
    
    args.output.write_text(report)
    print(f"Report written to: {args.output}")
    print(report)


if __name__ == "__main__":
    main()
```

---

### F.4 Test Data Directory Structure

```
tests/
└── cross_language/
    ├── data/                          # Generated test data
    │   ├── time.cdr                   # CDR binary
    │   ├── time.json                  # Metadata
    │   ├── duration.cdr
    │   ├── duration.json
    │   ├── header.cdr
    │   ├── header.json
    │   ├── point_cloud2.cdr
    │   ├── point_cloud2.json
    │   ├── radar_cube.cdr
    │   ├── radar_cube.json
    │   ├── compressed_video.cdr
    │   └── compressed_video.json
    ├── generate_test_data.rs          # Rust generator (example)
    └── README.md                      # Documentation
```

---

## Validation Checklist

### Test Data Generation

- [ ] Rust test data generator compiles
- [ ] All message types have `.cdr` and `.json` files
- [ ] Metadata includes field values for verification

### Python Validation

- [ ] `test_cross_language.py` runs without skips
- [ ] All `rust_to_python` tests pass
- [ ] All `python_to_rust_compatible` tests pass
- [ ] Byte-for-byte CDR match verified

### C Validation

- [ ] `test_cross_language.c` compiles
- [ ] All `rust_to_c` tests pass
- [ ] All `c_to_rust_compatible` tests pass
- [ ] Byte-for-byte CDR match verified

### Performance Comparison

- [ ] Rust benchmarks available (Phase A)
- [ ] Python benchmarks available (Phase E)
- [ ] Comparison script generates report
- [ ] Performance ratios documented

---

## Success Criteria

- [ ] Test data generated for all priority message types
- [ ] Python deserializes all Rust CDR correctly
- [ ] C deserializes all Rust CDR correctly
- [ ] Python CDR matches Rust CDR byte-for-byte
- [ ] C CDR matches Rust CDR byte-for-byte
- [ ] Cross-language validation script runs end-to-end
- [ ] Performance comparison report generated
- [ ] Documentation complete

---

## Final Project Completion Checklist

After completing Phase F, verify all requirements are met:

### Requirement 1: Rust CDR Encoding/Decoding ✅
- [ ] All message types serialize/deserialize (existing)
- [ ] Cross-language validation confirms correctness

### Requirement 2: Rust Tests + Benchmarks
- [ ] Rust unit tests pass (existing)
- [ ] Criterion benchmarks implemented (Phase A)
- [ ] Heavy message benchmarks complete

### Requirement 3: Complete C API
- [ ] All message types have FFI (Phase B)
- [ ] C tests pass (Phase B)
- [ ] Cross-language validation confirms (Phase F)

### Requirement 4: C Tests with Rust Coverage
- [ ] Coverage instrumentation working (Phase C)
- [ ] Combined coverage report generated

### Requirement 5: Python Tests + Coverage
- [ ] Python tests implemented (Phase D)
- [ ] Coverage ≥ 70%
- [ ] Cross-language validation confirms (Phase F)

### Requirement 6: Python Benchmarks
- [ ] pytest-benchmark configured (Phase E)
- [ ] Heavy message benchmarks complete
- [ ] Comparison with Rust available (Phase F)

---

**This completes the comprehensive TODO plan for EdgeFirst Schemas.**
