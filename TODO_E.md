# TODO Phase E: Python Benchmarks

**Phase:** E  
**Status:** Not Started  
**Estimate:** 6-8 hours  
**Dependencies:** Phase D (Python Test Suite) - need pytest infrastructure  
**Blocks:** Phase F (Cross-Language Validation)

---

## Objective

Implement comprehensive performance benchmarks for Python CDR serialization/deserialization using `pytest-benchmark`, focusing on the same heavy message types benchmarked in Rust (Phase A) to enable cross-language comparison.

---

## Background

### Python CDR Implementation

Python uses `pycdr2` for CDR serialization (not the Rust library). This means:
- Serialization is **pure Python** (with some C extensions in pycdr2)
- Performance characteristics differ significantly from Rust
- Benchmarks help identify if Python performance is acceptable for production use

### Target Message Types

Same as Rust benchmarks (Phase A):
- **FoxgloveCompressedVideo**: H.264 video frames (10KB - 1MB)
- **RadarCube**: 4D radar tensors (128KB - 24MB)
- **PointCloud2**: Point clouds (32KB - 4MB)
- **Image**: Raw images (900KB - 6.2MB)
- **Mask**: Segmentation masks (64KB - 6MB)

---

## Deliverables

### E.1 Benchmark Configuration

#### E.1.1 Update pyproject.toml

Ensure pytest-benchmark is configured:

```toml
[tool.pytest.ini_options]
# ... existing config ...
markers = [
    "slow: marks tests as slow",
    "benchmark: marks tests as benchmarks (use -m benchmark to run)",
]

# Benchmark settings
[tool.pytest.benchmark]
min_rounds = 10
disable_gc = true
warmup = true
```

**Note:** pytest-benchmark should already be in `[project.optional-dependencies].test` from Phase D.

---

### E.2 Benchmark Implementation

#### E.2.1 Benchmark Directory Structure

```
tests/python/benchmarks/
├── __init__.py
├── conftest.py                    # Benchmark fixtures
├── bench_builtin_interfaces.py    # Baseline (lightweight)
├── bench_heavy_messages.py        # Main benchmarks
└── bench_comparison.py            # Cross-language comparison data
```

#### E.2.2 Benchmark Fixtures

**File:** `tests/python/benchmarks/conftest.py`

```python
"""Fixtures for benchmark tests."""

import pytest
from edgefirst.schemas import (
    builtin_interfaces,
    std_msgs,
    geometry_msgs,
    sensor_msgs,
    edgefirst_msgs,
    foxglove_msgs,
)


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def create_header():
    """Create a standard header for benchmarks."""
    return std_msgs.Header(
        stamp=builtin_interfaces.Time(sec=1234567890, nanosec=123456789),
        frame_id="benchmark_frame",
    )


# =============================================================================
# COMPRESSED VIDEO FIXTURES (Various sizes)
# =============================================================================

@pytest.fixture(params=[10_000, 100_000, 500_000, 1_000_000], ids=["10KB", "100KB", "500KB", "1MB"])
def compressed_video_data(request):
    """Create CompressedVideo messages of various sizes."""
    size = request.param
    return foxglove_msgs.FoxgloveCompressedVideo(
        header=create_header(),
        data=bytes(size),
        format="h264",
    )


@pytest.fixture
def compressed_video_100kb():
    """Single 100KB video for standalone benchmarks."""
    return foxglove_msgs.FoxgloveCompressedVideo(
        header=create_header(),
        data=bytes(100_000),
        format="h264",
    )


# =============================================================================
# RADAR CUBE FIXTURES (Various sizes)
# =============================================================================

def create_radar_cube(shape):
    """Helper to create RadarCube with given shape."""
    total_elements = 1
    for dim in shape:
        total_elements *= dim
    return edgefirst_msgs.RadarCube(
        header=create_header(),
        timestamp=1234567890123456,
        layout=[6, 1, 5, 2],  # SEQUENCE, RANGE, RXCHANNEL, DOPPLER
        shape=list(shape),
        scales=[1.0, 2.5, 1.0, 0.5],
        cube=[0] * total_elements,
        is_complex=False,
    )


@pytest.fixture(params=[
    ((8, 64, 4, 32), "small_128KB"),
    ((16, 128, 8, 64), "medium_2MB"),
    ((32, 256, 12, 128), "large_24MB"),
], ids=lambda x: x[1])
def radar_cube_data(request):
    """Create RadarCube messages of various sizes."""
    shape, _ = request.param
    return create_radar_cube(shape)


@pytest.fixture
def radar_cube_medium():
    """Single medium RadarCube for standalone benchmarks."""
    return create_radar_cube((16, 128, 8, 64))


# =============================================================================
# POINT CLOUD FIXTURES (Various sizes)
# =============================================================================

def create_point_cloud(num_points, point_step=32):
    """Helper to create PointCloud2 with given point count."""
    fields = [
        sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
        sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
        sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
        sensor_msgs.PointField(name="intensity", offset=12, datatype=7, count=1),
        sensor_msgs.PointField(name="ring", offset=16, datatype=4, count=1),
        sensor_msgs.PointField(name="time", offset=20, datatype=8, count=1),
    ]
    return sensor_msgs.PointCloud2(
        header=create_header(),
        height=1,
        width=num_points,
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * num_points,
        data=bytes(point_step * num_points),
        is_dense=True,
    )


@pytest.fixture(params=[1_000, 10_000, 65_536, 131_072], ids=["1K", "10K", "65K", "131K"])
def point_cloud_data(request):
    """Create PointCloud2 messages of various sizes."""
    num_points = request.param
    return create_point_cloud(num_points)


@pytest.fixture
def point_cloud_10k():
    """Single 10K point cloud for standalone benchmarks."""
    return create_point_cloud(10_000)


# =============================================================================
# IMAGE FIXTURES (Various resolutions)
# =============================================================================

def create_image(width, height, encoding="rgb8", bytes_per_pixel=3):
    """Helper to create Image with given dimensions."""
    return sensor_msgs.Image(
        header=create_header(),
        height=height,
        width=width,
        encoding=encoding,
        is_bigendian=0,
        step=width * bytes_per_pixel,
        data=bytes(width * height * bytes_per_pixel),
    )


@pytest.fixture(params=[
    ((640, 480, 3), "VGA_RGB"),
    ((1280, 720, 3), "HD_RGB"),
    ((1920, 1080, 3), "FHD_RGB"),
    ((1920, 1080, 2), "FHD_YUV"),
], ids=lambda x: x[1])
def image_data(request):
    """Create Image messages of various sizes."""
    (width, height, bpp), _ = request.param
    encoding = "rgb8" if bpp == 3 else "yuyv"
    return create_image(width, height, encoding, bpp)


@pytest.fixture
def image_hd():
    """Single HD image for standalone benchmarks."""
    return create_image(1280, 720)


# =============================================================================
# MASK FIXTURES (Various sizes)
# =============================================================================

def create_mask(width, height, channels=1):
    """Helper to create Mask with given dimensions."""
    return edgefirst_msgs.Mask(
        height=height,
        width=width,
        length=channels,
        encoding="",
        mask=bytes(width * height * channels),
        boxed=False,
    )


@pytest.fixture(params=[
    ((256, 256, 1), "small"),
    ((640, 480, 1), "medium"),
    ((1920, 1080, 1), "large"),
    ((640, 480, 20), "multiclass"),
], ids=lambda x: x[1])
def mask_data(request):
    """Create Mask messages of various sizes."""
    (width, height, channels), _ = request.param
    return create_mask(width, height, channels)


@pytest.fixture
def mask_medium():
    """Single medium mask for standalone benchmarks."""
    return create_mask(640, 480, 1)
```

---

#### E.2.3 Heavy Message Benchmarks

**File:** `tests/python/benchmarks/bench_heavy_messages.py`

```python
"""Benchmarks for heavy message types.

Run with: pytest tests/python/benchmarks/ -m benchmark --benchmark-only
"""

import pytest
from edgefirst.schemas import (
    foxglove_msgs,
    edgefirst_msgs,
    sensor_msgs,
)


# =============================================================================
# COMPRESSED VIDEO BENCHMARKS
# =============================================================================

class TestCompressedVideoBenchmarks:
    """Benchmarks for FoxgloveCompressedVideo."""

    @pytest.mark.benchmark(group="compressed_video_serialize")
    def test_serialize(self, benchmark, compressed_video_data):
        """Benchmark serialization of CompressedVideo."""
        result = benchmark(compressed_video_data.serialize)
        assert isinstance(result, bytes)

    @pytest.mark.benchmark(group="compressed_video_deserialize")
    def test_deserialize(self, benchmark, compressed_video_data):
        """Benchmark deserialization of CompressedVideo."""
        data = compressed_video_data.serialize()
        result = benchmark(foxglove_msgs.FoxgloveCompressedVideo.deserialize, data)
        assert result.format == "h264"

    @pytest.mark.benchmark(group="compressed_video_roundtrip")
    def test_roundtrip(self, benchmark, compressed_video_100kb):
        """Benchmark full serialize/deserialize roundtrip."""
        def roundtrip():
            data = compressed_video_100kb.serialize()
            return foxglove_msgs.FoxgloveCompressedVideo.deserialize(data)
        
        result = benchmark(roundtrip)
        assert result.format == "h264"


# =============================================================================
# RADAR CUBE BENCHMARKS
# =============================================================================

class TestRadarCubeBenchmarks:
    """Benchmarks for RadarCube."""

    @pytest.mark.benchmark(group="radar_cube_serialize")
    def test_serialize(self, benchmark, radar_cube_data):
        """Benchmark serialization of RadarCube."""
        result = benchmark(radar_cube_data.serialize)
        assert isinstance(result, bytes)

    @pytest.mark.benchmark(group="radar_cube_deserialize")
    def test_deserialize(self, benchmark, radar_cube_data):
        """Benchmark deserialization of RadarCube."""
        data = radar_cube_data.serialize()
        result = benchmark(edgefirst_msgs.RadarCube.deserialize, data)
        assert len(result.cube) == len(radar_cube_data.cube)

    @pytest.mark.benchmark(group="radar_cube_roundtrip")
    def test_roundtrip(self, benchmark, radar_cube_medium):
        """Benchmark full serialize/deserialize roundtrip."""
        def roundtrip():
            data = radar_cube_medium.serialize()
            return edgefirst_msgs.RadarCube.deserialize(data)
        
        result = benchmark(roundtrip)
        assert len(result.cube) > 0


# =============================================================================
# POINT CLOUD BENCHMARKS
# =============================================================================

class TestPointCloud2Benchmarks:
    """Benchmarks for PointCloud2."""

    @pytest.mark.benchmark(group="point_cloud_serialize")
    def test_serialize(self, benchmark, point_cloud_data):
        """Benchmark serialization of PointCloud2."""
        result = benchmark(point_cloud_data.serialize)
        assert isinstance(result, bytes)

    @pytest.mark.benchmark(group="point_cloud_deserialize")
    def test_deserialize(self, benchmark, point_cloud_data):
        """Benchmark deserialization of PointCloud2."""
        data = point_cloud_data.serialize()
        result = benchmark(sensor_msgs.PointCloud2.deserialize, data)
        assert result.width == point_cloud_data.width

    @pytest.mark.benchmark(group="point_cloud_roundtrip")
    def test_roundtrip(self, benchmark, point_cloud_10k):
        """Benchmark full serialize/deserialize roundtrip."""
        def roundtrip():
            data = point_cloud_10k.serialize()
            return sensor_msgs.PointCloud2.deserialize(data)
        
        result = benchmark(roundtrip)
        assert result.width == 10_000


# =============================================================================
# IMAGE BENCHMARKS
# =============================================================================

class TestImageBenchmarks:
    """Benchmarks for Image."""

    @pytest.mark.benchmark(group="image_serialize")
    def test_serialize(self, benchmark, image_data):
        """Benchmark serialization of Image."""
        result = benchmark(image_data.serialize)
        assert isinstance(result, bytes)

    @pytest.mark.benchmark(group="image_deserialize")
    def test_deserialize(self, benchmark, image_data):
        """Benchmark deserialization of Image."""
        data = image_data.serialize()
        result = benchmark(sensor_msgs.Image.deserialize, data)
        assert result.width == image_data.width

    @pytest.mark.benchmark(group="image_roundtrip")
    def test_roundtrip(self, benchmark, image_hd):
        """Benchmark full serialize/deserialize roundtrip."""
        def roundtrip():
            data = image_hd.serialize()
            return sensor_msgs.Image.deserialize(data)
        
        result = benchmark(roundtrip)
        assert result.width == 1280


# =============================================================================
# MASK BENCHMARKS
# =============================================================================

class TestMaskBenchmarks:
    """Benchmarks for Mask."""

    @pytest.mark.benchmark(group="mask_serialize")
    def test_serialize(self, benchmark, mask_data):
        """Benchmark serialization of Mask."""
        result = benchmark(mask_data.serialize)
        assert isinstance(result, bytes)

    @pytest.mark.benchmark(group="mask_deserialize")
    def test_deserialize(self, benchmark, mask_data):
        """Benchmark deserialization of Mask."""
        data = mask_data.serialize()
        result = benchmark(edgefirst_msgs.Mask.deserialize, data)
        assert result.width == mask_data.width

    @pytest.mark.benchmark(group="mask_roundtrip")
    def test_roundtrip(self, benchmark, mask_medium):
        """Benchmark full serialize/deserialize roundtrip."""
        def roundtrip():
            data = mask_medium.serialize()
            return edgefirst_msgs.Mask.deserialize(data)
        
        result = benchmark(roundtrip)
        assert result.width == 640
```

---

#### E.2.4 Baseline Benchmarks

**File:** `tests/python/benchmarks/bench_builtin_interfaces.py`

```python
"""Baseline benchmarks for lightweight message types.

These establish a performance floor for comparison with heavy messages.
"""

import pytest
from edgefirst.schemas import (
    builtin_interfaces,
    std_msgs,
    geometry_msgs,
)


class TestBuiltinInterfacesBenchmarks:
    """Baseline benchmarks for builtin_interfaces."""

    @pytest.mark.benchmark(group="baseline")
    def test_time_serialize(self, benchmark):
        """Benchmark Time serialization."""
        time = builtin_interfaces.Time(sec=1234567890, nanosec=123456789)
        result = benchmark(time.serialize)
        assert isinstance(result, bytes)

    @pytest.mark.benchmark(group="baseline")
    def test_time_deserialize(self, benchmark):
        """Benchmark Time deserialization."""
        time = builtin_interfaces.Time(sec=1234567890, nanosec=123456789)
        data = time.serialize()
        result = benchmark(builtin_interfaces.Time.deserialize, data)
        assert result.sec == 1234567890


class TestStdMsgsBenchmarks:
    """Baseline benchmarks for std_msgs."""

    @pytest.mark.benchmark(group="baseline")
    def test_header_serialize(self, benchmark):
        """Benchmark Header serialization."""
        header = std_msgs.Header(
            stamp=builtin_interfaces.Time(sec=100, nanosec=200),
            frame_id="base_link",
        )
        result = benchmark(header.serialize)
        assert isinstance(result, bytes)

    @pytest.mark.benchmark(group="baseline")
    def test_header_deserialize(self, benchmark):
        """Benchmark Header deserialization."""
        header = std_msgs.Header(
            stamp=builtin_interfaces.Time(sec=100, nanosec=200),
            frame_id="base_link",
        )
        data = header.serialize()
        result = benchmark(std_msgs.Header.deserialize, data)
        assert result.frame_id == "base_link"


class TestGeometryMsgsBenchmarks:
    """Baseline benchmarks for geometry_msgs."""

    @pytest.mark.benchmark(group="baseline")
    def test_vector3_serialize(self, benchmark):
        """Benchmark Vector3 serialization."""
        vec = geometry_msgs.Vector3(x=1.0, y=2.0, z=3.0)
        result = benchmark(vec.serialize)
        assert isinstance(result, bytes)

    @pytest.mark.benchmark(group="baseline")
    def test_pose_serialize(self, benchmark):
        """Benchmark Pose serialization."""
        pose = geometry_msgs.Pose(
            position=geometry_msgs.Point(x=1.0, y=2.0, z=3.0),
            orientation=geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        result = benchmark(pose.serialize)
        assert isinstance(result, bytes)
```

---

### E.3 Benchmark Commands

#### E.3.1 Running Benchmarks

```bash
# Run all benchmarks
pytest tests/python/benchmarks/ --benchmark-only

# Run with detailed output
pytest tests/python/benchmarks/ --benchmark-only -v

# Run specific benchmark group
pytest tests/python/benchmarks/ --benchmark-only -k "radar_cube"

# Save benchmark results to JSON
pytest tests/python/benchmarks/ --benchmark-only --benchmark-json=benchmark_results.json

# Compare against saved baseline
pytest tests/python/benchmarks/ --benchmark-only --benchmark-compare=baseline.json

# Save current run as baseline
pytest tests/python/benchmarks/ --benchmark-only --benchmark-save=baseline

# Generate histogram (requires terminal support)
pytest tests/python/benchmarks/ --benchmark-only --benchmark-histogram

# Disable garbage collection (more accurate, already in config)
pytest tests/python/benchmarks/ --benchmark-only --benchmark-disable-gc

# Increase minimum rounds for stability
pytest tests/python/benchmarks/ --benchmark-only --benchmark-min-rounds=50
```

#### E.3.2 Benchmark Output Example

```
----------------------------- benchmark: 12 tests -----------------------------
Name (time in us)                    Min        Max       Mean    StdDev     Median
----------------------------------------------------------------------------------
test_time_serialize               2.1234     5.4321     2.5678    0.1234     2.4567
test_header_serialize             5.1234    12.3456     6.7890    0.4567     6.5432
test_compressed_video_100KB     234.567   567.890   345.678   45.678   334.567
test_radar_cube_medium        1234.567  2345.678  1567.890  234.567  1456.789
test_point_cloud_10K           456.789   890.123   567.890   56.789   543.210
----------------------------------------------------------------------------------
```

---

### E.4 Cross-Language Comparison

#### E.4.1 Comparison Data Collector

**File:** `tests/python/benchmarks/bench_comparison.py`

```python
"""Generate comparison data for Python vs Rust benchmarks.

After running both Rust and Python benchmarks, this module helps
generate a comparison report.

Usage:
    1. Run Rust benchmarks: cargo bench
    2. Run Python benchmarks: pytest tests/python/benchmarks/ --benchmark-json=python_bench.json
    3. Run comparison: python tests/python/benchmarks/bench_comparison.py
"""

import json
import sys
from pathlib import Path


def load_python_benchmarks(filepath: str) -> dict:
    """Load Python benchmark results from JSON."""
    with open(filepath) as f:
        data = json.load(f)
    
    results = {}
    for bench in data.get("benchmarks", []):
        name = bench["name"]
        stats = bench["stats"]
        results[name] = {
            "mean_us": stats["mean"] * 1_000_000,  # Convert to microseconds
            "min_us": stats["min"] * 1_000_000,
            "max_us": stats["max"] * 1_000_000,
            "stddev_us": stats["stddev"] * 1_000_000,
        }
    return results


def generate_comparison_table(python_results: dict) -> str:
    """Generate markdown table comparing Python results."""
    lines = [
        "# Python CDR Serialization Benchmarks",
        "",
        "| Message Type | Operation | Mean (μs) | Min (μs) | Max (μs) |",
        "|--------------|-----------|-----------|----------|----------|",
    ]
    
    for name, stats in sorted(python_results.items()):
        lines.append(
            f"| {name} | serialize | {stats['mean_us']:.2f} | "
            f"{stats['min_us']:.2f} | {stats['max_us']:.2f} |"
        )
    
    lines.extend([
        "",
        "## Comparison Notes",
        "",
        "- Python uses pycdr2 for CDR serialization (pure Python with C extensions)",
        "- Rust uses the `cdr` crate with native code",
        "- Expected Python/Rust ratio: 5-20x slower for Python",
        "- Large messages may show smaller ratio due to memory copy dominance",
    ])
    
    return "\n".join(lines)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python bench_comparison.py <python_bench.json>")
        sys.exit(1)
    
    python_file = sys.argv[1]
    python_results = load_python_benchmarks(python_file)
    
    report = generate_comparison_table(python_results)
    print(report)
    
    # Save to file
    output_path = Path(python_file).parent / "benchmark_report.md"
    output_path.write_text(report)
    print(f"\nReport saved to: {output_path}")
```

---

### E.5 Expected Performance Baselines

Based on pycdr2 characteristics, rough expected ranges:

| Message Type | Size | Python Serialize | Python Deserialize |
|--------------|------|------------------|-------------------|
| Time | 8 bytes | ~2-5 μs | ~2-5 μs |
| Header | ~50 bytes | ~10-20 μs | ~10-20 μs |
| CompressedVideo (100KB) | 100 KB | ~500-1000 μs | ~500-1000 μs |
| RadarCube (2MB) | 2 MB | ~10-30 ms | ~10-30 ms |
| PointCloud2 (10K) | 320 KB | ~2-5 ms | ~2-5 ms |
| Image (HD) | 2.7 MB | ~15-40 ms | ~15-40 ms |

**Note:** These are rough estimates. Actual performance depends on:
- Python version (3.8 vs 3.11+ has significant improvements)
- pycdr2 version and optimization level
- System memory bandwidth
- CPU cache effects

---

### E.6 CI Integration

#### E.6.1 Add to GitHub Actions

Add benchmark job to existing workflow:

```yaml
# In .github/workflows/test.yml or similar

  python-benchmarks:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: "3.11"

      - name: Install dependencies
        run: pip install -e ".[test]"

      - name: Run benchmarks
        run: |
          pytest tests/python/benchmarks/ \
            --benchmark-only \
            --benchmark-json=benchmark_results.json \
            --benchmark-min-rounds=20

      - name: Upload benchmark results
        uses: actions/upload-artifact@v4
        with:
          name: python-benchmarks
          path: benchmark_results.json

      - name: Check for performance regression
        run: |
          # Compare against stored baseline (if exists)
          if [ -f .benchmarks/baseline.json ]; then
            pytest tests/python/benchmarks/ \
              --benchmark-only \
              --benchmark-compare=.benchmarks/baseline.json \
              --benchmark-compare-fail=mean:10%
          fi
```

---

## Validation Checklist

### Installation

- [ ] `pip install -e ".[test]"` includes pytest-benchmark
- [ ] `pytest --benchmark-only --help` shows benchmark options

### Benchmark Execution

- [ ] `pytest tests/python/benchmarks/ --benchmark-only` runs all benchmarks
- [ ] All benchmarks complete without timeout
- [ ] Results are reproducible (low variance)

### Output

- [ ] JSON output generated with `--benchmark-json`
- [ ] Benchmark groups organized logically
- [ ] Statistics include mean, min, max, stddev

### Cross-Language

- [ ] Python results can be compared with Rust results
- [ ] Comparison report generated

---

## Success Criteria

- [ ] `tests/python/benchmarks/conftest.py` with parameterized fixtures
- [ ] `bench_builtin_interfaces.py` with baseline benchmarks
- [ ] `bench_heavy_messages.py` with all 5 message types
- [ ] CompressedVideo benchmarks (4 sizes)
- [ ] RadarCube benchmarks (3 sizes)
- [ ] PointCloud2 benchmarks (4 sizes)
- [ ] Image benchmarks (4 resolutions)
- [ ] Mask benchmarks (4 sizes)
- [ ] Benchmarks complete in < 5 minutes total
- [ ] JSON results exportable
- [ ] Comparison script works
- [ ] CI job configured (optional)

---

**Next Phase:** [TODO_F.md](./TODO_F.md) - Cross-Language Validation
