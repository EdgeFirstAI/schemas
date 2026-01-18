# TODO Phase D: Python Test Suite

**Phase:** D  
**Status:** Not Started  
**Estimate:** 10-12 hours  
**Dependencies:** None (can start immediately)  
**Blocks:** Phase E (Python Benchmarks), Phase F (Cross-Language Validation)

---

## Objective

Create a comprehensive Python test suite with pytest, pytest-cov for coverage reporting, and achieve ≥70% test coverage across all Python schema modules.

---

## Current State

### Python Package Structure

```
edgefirst/
└── schemas/
    ├── __init__.py              # Package init, decode_pcd() helper
    ├── builtin_interfaces.py    # Time, Duration
    ├── std_msgs.py              # Header, ColorRGBA
    ├── geometry_msgs.py         # Vector3, Point, Pose, etc.
    ├── sensor_msgs.py           # Image, PointCloud2, IMU, etc.
    ├── edgefirst_msgs.py        # DmaBuf, RadarCube, Detect, etc.
    ├── foxglove_msgs.py         # CompressedVideo, Annotations
    └── nav_msgs.py              # Odometry, Path
```

### CDR Serialization

All classes inherit from `pycdr2.IdlStruct`, which provides:
- `serialize()` → `bytes`
- `deserialize(data: bytes)` → instance

### Current Test State

- **No test files exist** in `tests/python/`
- `tests/python/benchmarks/` directory exists but is empty
- No pytest configuration in `pyproject.toml`

---

## Deliverables

### D.1 Update pyproject.toml

**File:** `pyproject.toml`

Add test dependencies and pytest configuration:

```toml
[project]
name = "edgefirst-schemas"
# ... existing config ...

[project.optional-dependencies]
test = [
    "pytest>=7.0",
    "pytest-cov>=4.0",
    "pytest-benchmark>=4.0",
    "hypothesis>=6.0",
    "mypy>=1.0",
]
dev = [
    "edgefirst-schemas[test]",
    "black",
    "ruff",
]

[tool.pytest.ini_options]
testpaths = ["tests/python"]
python_files = ["test_*.py"]
python_classes = ["Test*"]
python_functions = ["test_*"]
addopts = [
    "-v",
    "--tb=short",
    "--strict-markers",
]
markers = [
    "slow: marks tests as slow (deselect with '-m \"not slow\"')",
    "benchmark: marks tests as benchmarks",
]

[tool.coverage.run]
source = ["edgefirst"]
branch = true
omit = [
    "*/tests/*",
    "*/__pycache__/*",
]

[tool.coverage.report]
exclude_lines = [
    "pragma: no cover",
    "def __repr__",
    "raise NotImplementedError",
    "if TYPE_CHECKING:",
]
fail_under = 70
show_missing = true

[tool.mypy]
python_version = "3.8"
warn_return_any = true
warn_unused_ignores = true
disallow_untyped_defs = true
```

**Acceptance:**
- [ ] `pip install -e ".[test]"` installs all test dependencies
- [ ] `pytest` runs with configured options
- [ ] Coverage threshold set to 70%

---

### D.2 Test Directory Structure

Create the following structure:

```
tests/
└── python/
    ├── __init__.py
    ├── conftest.py                    # Shared fixtures
    ├── test_builtin_interfaces.py     # Time, Duration tests
    ├── test_std_msgs.py               # Header, ColorRGBA tests
    ├── test_geometry_msgs.py          # Vector3, Point, Pose, etc.
    ├── test_sensor_msgs.py            # Image, PointCloud2, IMU, etc.
    ├── test_edgefirst_msgs.py         # DmaBuf, RadarCube, Detect, etc.
    ├── test_foxglove_msgs.py          # CompressedVideo, Annotations
    ├── test_nav_msgs.py               # Odometry, Path
    ├── test_decode_pcd.py             # decode_pcd() helper function
    └── benchmarks/
        └── (Phase E)
```

---

### D.3 Shared Test Fixtures

**File:** `tests/python/conftest.py`

```python
"""Shared pytest fixtures for EdgeFirst Schemas tests."""

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
# BUILTIN INTERFACES FIXTURES
# =============================================================================

@pytest.fixture
def sample_time():
    """Create a sample Time message."""
    return builtin_interfaces.Time(sec=1234567890, nanosec=123456789)


@pytest.fixture
def sample_duration():
    """Create a sample Duration message."""
    return builtin_interfaces.Duration(sec=60, nanosec=500000000)


# =============================================================================
# STD_MSGS FIXTURES
# =============================================================================

@pytest.fixture
def sample_header(sample_time):
    """Create a sample Header message."""
    return std_msgs.Header(stamp=sample_time, frame_id="test_frame")


@pytest.fixture
def sample_color_rgba():
    """Create a sample ColorRGBA message."""
    return std_msgs.ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)


# =============================================================================
# GEOMETRY_MSGS FIXTURES
# =============================================================================

@pytest.fixture
def sample_vector3():
    """Create a sample Vector3 message."""
    return geometry_msgs.Vector3(x=1.0, y=2.0, z=3.0)


@pytest.fixture
def sample_point():
    """Create a sample Point message."""
    return geometry_msgs.Point(x=1.0, y=2.0, z=3.0)


@pytest.fixture
def sample_quaternion():
    """Create a sample Quaternion (identity)."""
    return geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


@pytest.fixture
def sample_pose(sample_point, sample_quaternion):
    """Create a sample Pose message."""
    return geometry_msgs.Pose(position=sample_point, orientation=sample_quaternion)


# =============================================================================
# SENSOR_MSGS FIXTURES
# =============================================================================

@pytest.fixture
def sample_point_field():
    """Create a sample PointField for x coordinate."""
    return sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1)


@pytest.fixture
def sample_point_cloud2(sample_header):
    """Create a sample PointCloud2 with 100 points."""
    fields = [
        sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
        sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
        sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
    ]
    num_points = 100
    point_step = 12
    return sensor_msgs.PointCloud2(
        header=sample_header,
        height=1,
        width=num_points,
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * num_points,
        data=bytes(point_step * num_points),
        is_dense=True,
    )


@pytest.fixture
def sample_image(sample_header):
    """Create a sample 640x480 RGB8 Image."""
    width, height = 640, 480
    return sensor_msgs.Image(
        header=sample_header,
        height=height,
        width=width,
        encoding="rgb8",
        is_bigendian=0,
        step=width * 3,
        data=bytes(width * height * 3),
    )


# =============================================================================
# EDGEFIRST_MSGS FIXTURES
# =============================================================================

@pytest.fixture
def sample_radar_cube(sample_header):
    """Create a sample RadarCube."""
    shape = [8, 64, 4, 32]  # seq, range, rx, doppler
    total_elements = 8 * 64 * 4 * 32
    return edgefirst_msgs.RadarCube(
        header=sample_header,
        timestamp=1234567890123456,
        layout=[6, 1, 5, 2],  # SEQUENCE, RANGE, RXCHANNEL, DOPPLER
        shape=shape,
        scales=[1.0, 2.5, 1.0, 0.5],
        cube=[0] * total_elements,
        is_complex=False,
    )


@pytest.fixture
def sample_dmabuf(sample_header):
    """Create a sample DmaBuf message."""
    return edgefirst_msgs.DmaBuf(
        header=sample_header,
        pid=12345,
        fd=10,
        width=1920,
        height=1080,
        stride=3840,
        fourcc=0x56595559,  # YUYV
        length=1920 * 1080 * 2,
    )


# =============================================================================
# FOXGLOVE_MSGS FIXTURES
# =============================================================================

@pytest.fixture
def sample_compressed_video(sample_header):
    """Create a sample FoxgloveCompressedVideo."""
    return foxglove_msgs.FoxgloveCompressedVideo(
        header=sample_header,
        data=bytes(10000),  # 10KB H.264 data
        format="h264",
    )
```

**Acceptance:**
- [ ] All fixtures provide valid message instances
- [ ] Fixtures are reusable across test modules
- [ ] Nested fixtures work correctly (e.g., `sample_pose` uses `sample_point`)

---

### D.4 Test Modules

#### D.4.1 test_builtin_interfaces.py

**File:** `tests/python/test_builtin_interfaces.py`

```python
"""Tests for builtin_interfaces module."""

import pytest
from edgefirst.schemas import builtin_interfaces


class TestTime:
    """Tests for Time message."""

    def test_time_creation(self):
        """Test creating a Time message with default values."""
        time = builtin_interfaces.Time()
        assert time.sec == 0
        assert time.nanosec == 0

    def test_time_with_values(self):
        """Test creating a Time message with specific values."""
        time = builtin_interfaces.Time(sec=100, nanosec=500000000)
        assert time.sec == 100
        assert time.nanosec == 500000000

    def test_time_serialize_deserialize(self, sample_time):
        """Test CDR serialization roundtrip."""
        data = sample_time.serialize()
        assert isinstance(data, bytes)
        assert len(data) > 0

        restored = builtin_interfaces.Time.deserialize(data)
        assert restored.sec == sample_time.sec
        assert restored.nanosec == sample_time.nanosec

    def test_time_equality(self):
        """Test Time equality comparison."""
        t1 = builtin_interfaces.Time(sec=100, nanosec=200)
        t2 = builtin_interfaces.Time(sec=100, nanosec=200)
        t3 = builtin_interfaces.Time(sec=100, nanosec=300)
        assert t1 == t2
        assert t1 != t3

    def test_time_max_nanosec(self):
        """Test Time with maximum nanosecond value."""
        time = builtin_interfaces.Time(sec=0, nanosec=999999999)
        data = time.serialize()
        restored = builtin_interfaces.Time.deserialize(data)
        assert restored.nanosec == 999999999


class TestDuration:
    """Tests for Duration message."""

    def test_duration_creation(self):
        """Test creating a Duration message with default values."""
        duration = builtin_interfaces.Duration()
        assert duration.sec == 0
        assert duration.nanosec == 0

    def test_duration_serialize_deserialize(self, sample_duration):
        """Test CDR serialization roundtrip."""
        data = sample_duration.serialize()
        restored = builtin_interfaces.Duration.deserialize(data)
        assert restored.sec == sample_duration.sec
        assert restored.nanosec == sample_duration.nanosec

    def test_duration_negative(self):
        """Test Duration with negative seconds (valid for durations)."""
        duration = builtin_interfaces.Duration(sec=-10, nanosec=500000000)
        data = duration.serialize()
        restored = builtin_interfaces.Duration.deserialize(data)
        assert restored.sec == -10
```

#### D.4.2 test_std_msgs.py

**File:** `tests/python/test_std_msgs.py`

```python
"""Tests for std_msgs module."""

import pytest
from edgefirst.schemas import std_msgs, builtin_interfaces


class TestHeader:
    """Tests for Header message."""

    def test_header_creation(self):
        """Test creating a Header with default values."""
        header = std_msgs.Header()
        assert header.frame_id == ""
        assert header.stamp is not None

    def test_header_with_values(self, sample_time):
        """Test creating a Header with specific values."""
        header = std_msgs.Header(stamp=sample_time, frame_id="base_link")
        assert header.frame_id == "base_link"
        assert header.stamp.sec == sample_time.sec

    def test_header_serialize_deserialize(self, sample_header):
        """Test CDR serialization roundtrip."""
        data = sample_header.serialize()
        restored = std_msgs.Header.deserialize(data)
        assert restored.frame_id == sample_header.frame_id
        assert restored.stamp.sec == sample_header.stamp.sec

    def test_header_empty_frame_id(self, sample_time):
        """Test Header with empty frame_id."""
        header = std_msgs.Header(stamp=sample_time, frame_id="")
        data = header.serialize()
        restored = std_msgs.Header.deserialize(data)
        assert restored.frame_id == ""

    def test_header_unicode_frame_id(self, sample_time):
        """Test Header with unicode frame_id."""
        header = std_msgs.Header(stamp=sample_time, frame_id="sensor_日本語")
        data = header.serialize()
        restored = std_msgs.Header.deserialize(data)
        assert restored.frame_id == "sensor_日本語"


class TestColorRGBA:
    """Tests for ColorRGBA message."""

    def test_color_rgba_creation(self):
        """Test creating a ColorRGBA with default values."""
        color = std_msgs.ColorRGBA()
        assert color.r == 0.0
        assert color.g == 0.0
        assert color.b == 0.0
        assert color.a == 0.0

    def test_color_rgba_serialize_deserialize(self, sample_color_rgba):
        """Test CDR serialization roundtrip."""
        data = sample_color_rgba.serialize()
        restored = std_msgs.ColorRGBA.deserialize(data)
        assert restored.r == pytest.approx(sample_color_rgba.r)
        assert restored.g == pytest.approx(sample_color_rgba.g)
        assert restored.b == pytest.approx(sample_color_rgba.b)
        assert restored.a == pytest.approx(sample_color_rgba.a)

    def test_color_rgba_full_white(self):
        """Test fully white color."""
        color = std_msgs.ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        data = color.serialize()
        restored = std_msgs.ColorRGBA.deserialize(data)
        assert all(v == pytest.approx(1.0) for v in [restored.r, restored.g, restored.b, restored.a])
```

#### D.4.3 test_sensor_msgs.py (Heavy Messages)

**File:** `tests/python/test_sensor_msgs.py`

```python
"""Tests for sensor_msgs module."""

import pytest
import struct
from edgefirst.schemas import sensor_msgs, std_msgs, builtin_interfaces


class TestPointField:
    """Tests for PointField message."""

    def test_point_field_creation(self):
        """Test creating a PointField."""
        field = sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1)
        assert field.name == "x"
        assert field.offset == 0
        assert field.datatype == 7  # FLOAT32
        assert field.count == 1

    def test_point_field_serialize_deserialize(self, sample_point_field):
        """Test CDR serialization roundtrip."""
        data = sample_point_field.serialize()
        restored = sensor_msgs.PointField.deserialize(data)
        assert restored.name == sample_point_field.name
        assert restored.offset == sample_point_field.offset
        assert restored.datatype == sample_point_field.datatype


class TestPointCloud2:
    """Tests for PointCloud2 message."""

    def test_point_cloud2_creation(self, sample_point_cloud2):
        """Test PointCloud2 structure."""
        assert sample_point_cloud2.height == 1
        assert sample_point_cloud2.width == 100
        assert len(sample_point_cloud2.fields) == 3
        assert sample_point_cloud2.is_dense is True

    def test_point_cloud2_serialize_deserialize(self, sample_point_cloud2):
        """Test CDR serialization roundtrip."""
        data = sample_point_cloud2.serialize()
        restored = sensor_msgs.PointCloud2.deserialize(data)
        assert restored.width == sample_point_cloud2.width
        assert restored.height == sample_point_cloud2.height
        assert len(restored.fields) == len(sample_point_cloud2.fields)
        assert len(restored.data) == len(sample_point_cloud2.data)

    @pytest.mark.slow
    def test_point_cloud2_large(self, sample_header):
        """Test PointCloud2 with large point count (65536 points)."""
        fields = [
            sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
            sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
            sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
            sensor_msgs.PointField(name="intensity", offset=12, datatype=7, count=1),
        ]
        num_points = 65536
        point_step = 16
        cloud = sensor_msgs.PointCloud2(
            header=sample_header,
            height=1,
            width=num_points,
            fields=fields,
            is_bigendian=False,
            point_step=point_step,
            row_step=point_step * num_points,
            data=bytes(point_step * num_points),
            is_dense=True,
        )

        data = cloud.serialize()
        restored = sensor_msgs.PointCloud2.deserialize(data)
        assert restored.width == num_points
        assert len(restored.data) == point_step * num_points


class TestImage:
    """Tests for Image message."""

    def test_image_creation(self, sample_image):
        """Test Image structure."""
        assert sample_image.width == 640
        assert sample_image.height == 480
        assert sample_image.encoding == "rgb8"

    def test_image_serialize_deserialize(self, sample_image):
        """Test CDR serialization roundtrip."""
        data = sample_image.serialize()
        restored = sensor_msgs.Image.deserialize(data)
        assert restored.width == sample_image.width
        assert restored.height == sample_image.height
        assert restored.encoding == sample_image.encoding
        assert len(restored.data) == len(sample_image.data)

    @pytest.mark.slow
    def test_image_full_hd(self, sample_header):
        """Test Image with Full HD resolution."""
        width, height = 1920, 1080
        image = sensor_msgs.Image(
            header=sample_header,
            height=height,
            width=width,
            encoding="rgb8",
            is_bigendian=0,
            step=width * 3,
            data=bytes(width * height * 3),
        )

        data = image.serialize()
        assert len(data) > 6_000_000  # ~6.2 MB for FHD RGB
        restored = sensor_msgs.Image.deserialize(data)
        assert restored.width == width
        assert restored.height == height
```

#### D.4.4 test_edgefirst_msgs.py

**File:** `tests/python/test_edgefirst_msgs.py`

```python
"""Tests for edgefirst_msgs module."""

import pytest
from edgefirst.schemas import edgefirst_msgs, std_msgs, builtin_interfaces


class TestRadarCube:
    """Tests for RadarCube message."""

    def test_radar_cube_creation(self, sample_radar_cube):
        """Test RadarCube structure."""
        assert sample_radar_cube.timestamp == 1234567890123456
        assert len(sample_radar_cube.layout) == 4
        assert len(sample_radar_cube.shape) == 4
        assert sample_radar_cube.is_complex is False

    def test_radar_cube_serialize_deserialize(self, sample_radar_cube):
        """Test CDR serialization roundtrip."""
        data = sample_radar_cube.serialize()
        restored = edgefirst_msgs.RadarCube.deserialize(data)
        assert restored.timestamp == sample_radar_cube.timestamp
        assert restored.layout == sample_radar_cube.layout
        assert restored.shape == sample_radar_cube.shape
        assert restored.is_complex == sample_radar_cube.is_complex
        assert len(restored.cube) == len(sample_radar_cube.cube)

    @pytest.mark.slow
    def test_radar_cube_large(self, sample_header):
        """Test RadarCube with large dimensions."""
        shape = [16, 128, 8, 64]  # 1M elements
        total_elements = 16 * 128 * 8 * 64
        cube = edgefirst_msgs.RadarCube(
            header=sample_header,
            timestamp=0,
            layout=[6, 1, 5, 2],
            shape=shape,
            scales=[1.0, 2.5, 1.0, 0.5],
            cube=[0] * total_elements,
            is_complex=True,
        )

        data = cube.serialize()
        restored = edgefirst_msgs.RadarCube.deserialize(data)
        assert len(restored.cube) == total_elements


class TestDmaBuf:
    """Tests for DmaBuf message."""

    def test_dmabuf_creation(self, sample_dmabuf):
        """Test DmaBuf structure."""
        assert sample_dmabuf.pid == 12345
        assert sample_dmabuf.fd == 10
        assert sample_dmabuf.width == 1920
        assert sample_dmabuf.height == 1080

    def test_dmabuf_serialize_deserialize(self, sample_dmabuf):
        """Test CDR serialization roundtrip."""
        data = sample_dmabuf.serialize()
        restored = edgefirst_msgs.DmaBuf.deserialize(data)
        assert restored.pid == sample_dmabuf.pid
        assert restored.fd == sample_dmabuf.fd
        assert restored.width == sample_dmabuf.width
        assert restored.height == sample_dmabuf.height
        assert restored.fourcc == sample_dmabuf.fourcc


class TestDetect:
    """Tests for Detect message."""

    def test_detect_with_boxes(self, sample_header):
        """Test Detect with detection boxes."""
        box = edgefirst_msgs.DetectBox2D(
            center_x=320.0,
            center_y=240.0,
            width=100.0,
            height=80.0,
            label="person",
            score=0.95,
            distance=5.0,
            speed=0.0,
            track=edgefirst_msgs.DetectTrack(
                id="track_001",
                lifetime=10,
                created=builtin_interfaces.Time(sec=100, nanosec=0),
            ),
        )
        detect = edgefirst_msgs.Detect(
            header=sample_header,
            input_timestamp=builtin_interfaces.Time(sec=100, nanosec=0),
            model_time=builtin_interfaces.Time(sec=0, nanosec=5000000),
            output_time=builtin_interfaces.Time(sec=0, nanosec=1000000),
            boxes=[box],
        )

        data = detect.serialize()
        restored = edgefirst_msgs.Detect.deserialize(data)
        assert len(restored.boxes) == 1
        assert restored.boxes[0].label == "person"
        assert restored.boxes[0].score == pytest.approx(0.95)
```

#### D.4.5 test_foxglove_msgs.py

**File:** `tests/python/test_foxglove_msgs.py`

```python
"""Tests for foxglove_msgs module."""

import pytest
from edgefirst.schemas import foxglove_msgs, std_msgs, builtin_interfaces


class TestFoxgloveCompressedVideo:
    """Tests for FoxgloveCompressedVideo message."""

    def test_compressed_video_creation(self, sample_compressed_video):
        """Test FoxgloveCompressedVideo structure."""
        assert sample_compressed_video.format == "h264"
        assert len(sample_compressed_video.data) == 10000

    def test_compressed_video_serialize_deserialize(self, sample_compressed_video):
        """Test CDR serialization roundtrip."""
        data = sample_compressed_video.serialize()
        restored = foxglove_msgs.FoxgloveCompressedVideo.deserialize(data)
        assert restored.format == sample_compressed_video.format
        assert len(restored.data) == len(sample_compressed_video.data)

    @pytest.mark.slow
    def test_compressed_video_large(self, sample_header):
        """Test with large video payload (1MB)."""
        video = foxglove_msgs.FoxgloveCompressedVideo(
            header=sample_header,
            data=bytes(1_000_000),
            format="h264",
        )

        data = video.serialize()
        restored = foxglove_msgs.FoxgloveCompressedVideo.deserialize(data)
        assert len(restored.data) == 1_000_000
```

---

### D.5 decode_pcd Helper Tests

**File:** `tests/python/test_decode_pcd.py`

```python
"""Tests for decode_pcd helper function."""

import pytest
import struct
from edgefirst.schemas import decode_pcd, sensor_msgs, std_msgs, builtin_interfaces


class TestDecodePcd:
    """Tests for decode_pcd() function."""

    def test_decode_pcd_xyz(self, sample_header):
        """Test decoding XYZ point cloud."""
        fields = [
            sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
            sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
            sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
        ]
        
        # Create test point data
        points = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        data = b"".join(struct.pack("<fff", *p) for p in points)
        
        cloud = sensor_msgs.PointCloud2(
            header=sample_header,
            height=1,
            width=2,
            fields=fields,
            is_bigendian=False,
            point_step=12,
            row_step=24,
            data=data,
            is_dense=True,
        )

        decoded = decode_pcd(cloud)
        assert len(decoded) == 2
        assert decoded[0] == pytest.approx((1.0, 2.0, 3.0))
        assert decoded[1] == pytest.approx((4.0, 5.0, 6.0))

    def test_decode_pcd_empty(self, sample_header):
        """Test decoding empty point cloud."""
        fields = [
            sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
            sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
            sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
        ]
        cloud = sensor_msgs.PointCloud2(
            header=sample_header,
            height=1,
            width=0,
            fields=fields,
            is_bigendian=False,
            point_step=12,
            row_step=0,
            data=b"",
            is_dense=True,
        )

        decoded = decode_pcd(cloud)
        assert len(decoded) == 0
```

---

### D.6 Test Commands

#### Quick Reference

Add to `TESTING.md`:

```bash
# === Python Test Commands ===

# Install test dependencies
pip install -e ".[test]"

# Run all tests
pytest tests/python/

# Run with coverage
pytest tests/python/ --cov=edgefirst --cov-report=html --cov-report=term

# Run excluding slow tests
pytest tests/python/ -m "not slow"

# Run specific module
pytest tests/python/test_sensor_msgs.py -v

# Run with verbose output
pytest tests/python/ -v --tb=long

# Type checking
mypy edgefirst/

# Generate coverage report
pytest tests/python/ --cov=edgefirst --cov-report=xml --cov-report=html
# Output: htmlcov/index.html, coverage.xml
```

---

## Validation Checklist

### Installation

- [ ] `pip install -e ".[test]"` succeeds
- [ ] `pytest --version` shows pytest installed
- [ ] `pytest --collect-only` finds all test files

### Test Execution

- [ ] `pytest tests/python/` runs all tests
- [ ] All tests pass (green)
- [ ] `pytest tests/python/ -m "not slow"` skips slow tests

### Coverage

- [ ] `pytest --cov=edgefirst` generates coverage report
- [ ] Coverage ≥ 70% for all modules
- [ ] `htmlcov/index.html` viewable

### Type Checking

- [ ] `mypy edgefirst/` passes with no errors

---

## Success Criteria

- [ ] `pyproject.toml` updated with test dependencies
- [ ] `tests/python/conftest.py` with shared fixtures
- [ ] `test_builtin_interfaces.py` with ≥10 tests
- [ ] `test_std_msgs.py` with ≥10 tests
- [ ] `test_geometry_msgs.py` with ≥15 tests
- [ ] `test_sensor_msgs.py` with ≥15 tests
- [ ] `test_edgefirst_msgs.py` with ≥15 tests
- [ ] `test_foxglove_msgs.py` with ≥10 tests
- [ ] `test_decode_pcd.py` with ≥5 tests
- [ ] All tests pass: `pytest tests/python/`
- [ ] Coverage ≥ 70%: `pytest --cov=edgefirst --cov-fail-under=70`
- [ ] Type checking passes: `mypy edgefirst/`

---

**Next Phase:** [TODO_E.md](./TODO_E.md) - Python Benchmarks
