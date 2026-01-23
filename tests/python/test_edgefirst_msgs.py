"""Tests for edgefirst_msgs module."""

import pytest
from edgefirst.schemas import (
    edgefirst_msgs,
    std_msgs,
    builtin_interfaces,
)


class TestDate:
    """Tests for Date message."""

    def test_date_creation(self):
        """Test Date structure."""
        date = edgefirst_msgs.Date(year=2025, month=1, day=15)
        assert date.year == 2025
        assert date.month == 1
        assert date.day == 15

    def test_date_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        date = edgefirst_msgs.Date(year=2024, month=12, day=31)
        data = date.serialize()
        restored = edgefirst_msgs.Date.deserialize(data)
        assert restored.year == 2024
        assert restored.month == 12
        assert restored.day == 31


class TestLocalTime:
    """Tests for LocalTime message."""

    def test_local_time_creation(self, sample_header):
        """Test LocalTime structure."""
        lt = edgefirst_msgs.LocalTime(
            header=sample_header,
            date=edgefirst_msgs.Date(year=2025, month=1, day=15),
            time=builtin_interfaces.Time(sec=43200, nanosec=0),  # Noon
            timezone=-300,  # EST (UTC-5)
        )
        assert lt.timezone == -300
        assert lt.date.year == 2025

    def test_local_time_serialize_deserialize(self, sample_header):
        """Test CDR serialization roundtrip."""
        lt = edgefirst_msgs.LocalTime(
            header=sample_header,
            date=edgefirst_msgs.Date(year=2025, month=6, day=21),
            time=builtin_interfaces.Time(sec=36000, nanosec=500000000),
            timezone=120,  # UTC+2
        )
        data = lt.serialize()
        restored = edgefirst_msgs.LocalTime.deserialize(data)
        assert restored.date.month == 6
        assert restored.timezone == 120


class TestTrack:
    """Tests for Track message (object tracking)."""

    def test_track_creation(self):
        """Test Track structure."""
        track = edgefirst_msgs.Track(
            id="track_42",
            lifetime=100,
            created=builtin_interfaces.Time(sec=1000, nanosec=0),
        )
        assert track.id == "track_42"
        assert track.lifetime == 100

    def test_track_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        track = edgefirst_msgs.Track(
            id="person_1",
            lifetime=50,
            created=builtin_interfaces.Time(sec=500, nanosec=123456),
        )
        data = track.serialize()
        restored = edgefirst_msgs.Track.deserialize(data)
        assert restored.id == "person_1"
        assert restored.lifetime == 50


class TestBox:
    """Tests for Box message (detection bounding box)."""

    def test_box_creation(self, sample_detect_box2d):
        """Test Box structure (matches Rust normalized coords)."""
        assert sample_detect_box2d.center_x == pytest.approx(0.5)
        assert sample_detect_box2d.center_y == pytest.approx(0.5)
        assert sample_detect_box2d.width == pytest.approx(0.1)
        assert sample_detect_box2d.height == pytest.approx(0.2)
        assert sample_detect_box2d.label == "car"
        assert sample_detect_box2d.score == pytest.approx(0.98)
        assert sample_detect_box2d.distance == pytest.approx(10.0)
        assert sample_detect_box2d.speed == pytest.approx(5.0)
        assert sample_detect_box2d.track.id == "t1"
        assert sample_detect_box2d.track.lifetime == 5

    def test_box_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        box = edgefirst_msgs.Box(
            center_x=0.5,
            center_y=0.5,
            width=0.3,
            height=0.4,
            label="car",
            score=0.88,
            distance=15.0,
            speed=5.5,
            track=edgefirst_msgs.Track(id="car_1", lifetime=10),
        )
        data = box.serialize()
        restored = edgefirst_msgs.Box.deserialize(data)
        assert restored.label == "car"
        assert restored.score == pytest.approx(0.88)
        assert restored.distance == pytest.approx(15.0)
        assert restored.track.id == "car_1"

    def test_box_normalized_coords(self):
        """Test Box with normalized coordinates (0-1)."""
        box = edgefirst_msgs.Box(
            center_x=0.25,
            center_y=0.75,
            width=0.1,
            height=0.2,
            label="bicycle",
            score=0.72,
            track=edgefirst_msgs.Track(),
        )
        data = box.serialize()
        restored = edgefirst_msgs.Box.deserialize(data)
        assert restored.center_x == pytest.approx(0.25)
        assert restored.center_y == pytest.approx(0.75)


class TestMask:
    """Tests for Mask message."""

    def test_mask_creation(self, sample_mask):
        """Test Mask structure (matches Rust VGA uncompressed)."""
        assert sample_mask.width == 640
        assert sample_mask.height == 480
        assert sample_mask.length == 0  # Uncompressed
        assert sample_mask.boxed is False

    def test_mask_serialize_deserialize(self, sample_mask):
        """Test CDR serialization roundtrip."""
        data = sample_mask.serialize()
        restored = edgefirst_msgs.Mask.deserialize(data)
        assert restored.width == sample_mask.width
        assert restored.height == sample_mask.height
        assert len(restored.mask) == len(sample_mask.mask)

    def test_mask_compressed(self):
        """Test Mask FHD compressed (matches Rust compressed test)."""
        mask = edgefirst_msgs.Mask(
            width=1920,
            height=1080,
            length=5,  # Compressed data length
            encoding="zstd",  # Matches Rust
            mask=[1, 2, 3, 4, 5],  # Matches Rust compressed data
            boxed=True,  # Matches Rust
        )
        data = mask.serialize()
        restored = edgefirst_msgs.Mask.deserialize(data)
        assert restored.width == 1920
        assert restored.height == 1080
        assert restored.length == 5
        assert restored.encoding == "zstd"
        assert list(restored.mask) == [1, 2, 3, 4, 5]
        assert restored.boxed is True

    def test_mask_binary_pattern(self):
        """Test Mask with binary pattern."""
        width, height = 8, 8
        mask_data = [
            0xFF if (i + j) % 2 == 0 else 0x00
            for j in range(height) for i in range(width)
        ]
        mask = edgefirst_msgs.Mask(
            width=width,
            height=height,
            length=0,  # Uncompressed
            mask=mask_data,
            boxed=True,
        )
        serialized = mask.serialize()
        restored = edgefirst_msgs.Mask.deserialize(serialized)
        assert list(restored.mask) == mask_data


class TestModel:
    """Tests for Model message (inference results)."""

    def test_model_creation(self, sample_header):
        """Test Model structure."""
        model = edgefirst_msgs.Model(
            header=sample_header,
            input_time=builtin_interfaces.Duration(sec=0, nanosec=1000000),
            model_time=builtin_interfaces.Duration(sec=0, nanosec=5000000),
            output_time=builtin_interfaces.Duration(sec=0, nanosec=500000),
            decode_time=builtin_interfaces.Duration(sec=0, nanosec=200000),
            boxes=[
                edgefirst_msgs.Box(
                    center_x=0.5, center_y=0.5, width=0.2, height=0.3,
                    label="person", score=0.95
                )
            ],
        )
        assert len(model.boxes) == 1
        assert model.boxes[0].label == "person"

    def test_model_serialize_deserialize(self, sample_header):
        """Test CDR serialization roundtrip."""
        model = edgefirst_msgs.Model(
            header=sample_header,
            input_time=builtin_interfaces.Duration(sec=0, nanosec=1000000),
            model_time=builtin_interfaces.Duration(sec=0, nanosec=8000000),
            output_time=builtin_interfaces.Duration(sec=0, nanosec=300000),
            decode_time=builtin_interfaces.Duration(sec=0, nanosec=100000),
            boxes=[],
        )
        data = model.serialize()
        restored = edgefirst_msgs.Model.deserialize(data)
        assert restored.model_time.nanosec == 8000000


class TestModelInfo:
    """Tests for ModelInfo message."""

    def test_model_info_creation(self, sample_header):
        """Test ModelInfo structure."""
        info = edgefirst_msgs.ModelInfo(
            header=sample_header,
            input_shape=[1, 640, 640, 3],
            input_type=edgefirst_msgs.model_info.UINT8.value,
            output_shape=[1, 8400, 84],
            output_type=edgefirst_msgs.model_info.FLOAT32.value,
            labels=["person", "car", "bicycle"],
            model_type="object_detection",
            model_format="TFLite",
            model_name="yolov8n",
        )
        assert info.model_name == "yolov8n"
        assert len(info.labels) == 3

    def test_model_info_serialize_deserialize(self, sample_header):
        """Test CDR serialization roundtrip."""
        info = edgefirst_msgs.ModelInfo(
            header=sample_header,
            input_shape=[1, 224, 224, 3],
            input_type=edgefirst_msgs.model_info.FLOAT32.value,
            output_shape=[1, 1000],
            output_type=edgefirst_msgs.model_info.FLOAT32.value,
            labels=[],
            model_type="classification",
            model_format="ONNX",
            model_name="resnet50",
        )
        data = info.serialize()
        restored = edgefirst_msgs.ModelInfo.deserialize(data)
        assert restored.model_type == "classification"
        assert restored.model_format == "ONNX"


class TestDetect:
    """Tests for Detect message."""

    def test_detect_creation(self, sample_detect):
        """Test Detect structure."""
        assert sample_detect.boxes is not None
        assert len(sample_detect.boxes) > 0

    def test_detect_serialize_deserialize(self, sample_detect):
        """Test CDR serialization roundtrip."""
        data = sample_detect.serialize()
        restored = edgefirst_msgs.Detect.deserialize(data)
        assert len(restored.boxes) == len(sample_detect.boxes)

    def test_detect_multiple_boxes(self, sample_header, sample_time):
        """Test Detect with multiple detections."""
        boxes = [
            edgefirst_msgs.Box(
                center_x=0.2 * i, center_y=0.3,
                width=0.1, height=0.15,
                label=f"obj_{i}", score=0.9 - 0.1 * i,
                track=edgefirst_msgs.Track(),
            )
            for i in range(5)
        ]
        detect = edgefirst_msgs.Detect(
            header=sample_header,
            input_timestamp=sample_time,
            boxes=boxes,
        )
        data = detect.serialize()
        restored = edgefirst_msgs.Detect.deserialize(data)
        assert len(restored.boxes) == 5
        assert restored.boxes[0].label == "obj_0"


class TestDmaBuffer:
    """Tests for DmaBuffer message."""

    def test_dma_buffer_creation(self, sample_dmabuf):
        """Test DmaBuffer structure (matches Rust FHD RG24)."""
        assert sample_dmabuf.fd == 42  # Matches Rust
        assert sample_dmabuf.width == 1920
        assert sample_dmabuf.height == 1080
        assert sample_dmabuf.stride == 5760  # 1920 * 3 (matches Rust)
        assert sample_dmabuf.fourcc == 0x34325247  # RG24 (matches Rust)
        assert sample_dmabuf.length == 1920 * 1080 * 3

    def test_dma_buffer_serialize_deserialize(self, sample_dmabuf):
        """Test CDR serialization roundtrip."""
        data = sample_dmabuf.serialize()
        restored = edgefirst_msgs.DmaBuffer.deserialize(data)
        assert restored.fd == sample_dmabuf.fd
        assert restored.width == sample_dmabuf.width
        assert restored.height == sample_dmabuf.height
        assert restored.fourcc == sample_dmabuf.fourcc

    def test_dma_buffer_formats(self, sample_header):
        """Test DmaBuffer with different pixel formats."""
        formats = [
            (0x56595559, "YUYV"),  # YUYV
            (0x32315559, "YU12"),  # YU12 (I420)
            (0x3231564E, "NV12"),  # NV12
        ]
        for fourcc, _ in formats:
            buf = edgefirst_msgs.DmaBuffer(
                header=sample_header,
                pid=1000,
                fd=5,
                width=640,
                height=480,
                stride=640 * 2,
                fourcc=fourcc,
                length=640 * 480 * 2,
            )
            data = buf.serialize()
            restored = edgefirst_msgs.DmaBuffer.deserialize(data)
            assert restored.fourcc == fourcc


class TestRadarCube:
    """Tests for RadarCube message."""

    def test_radar_cube_creation(self, sample_radar_cube):
        """Test RadarCube structure (matches Rust dimensions)."""
        assert len(sample_radar_cube.shape) == 4
        assert list(sample_radar_cube.shape) == [16, 256, 4, 64]
        assert sample_radar_cube.is_complex is False
        assert len(sample_radar_cube.cube) == 16 * 256 * 4 * 64

    def test_radar_cube_serialize_deserialize(self, sample_radar_cube):
        """Test CDR serialization roundtrip (matches Rust)."""
        data = sample_radar_cube.serialize()
        restored = edgefirst_msgs.RadarCube.deserialize(data)
        assert list(restored.shape) == list(sample_radar_cube.shape)
        assert list(restored.layout) == list(sample_radar_cube.layout)
        assert len(restored.cube) == len(sample_radar_cube.cube)
        # Verify some values survived roundtrip
        assert restored.cube[0] == sample_radar_cube.cube[0]
        assert restored.cube[1000] == sample_radar_cube.cube[1000]

    def test_radar_cube_layout(self, sample_header):
        """Test RadarCube with Range-Doppler layout (matches Rust complex)."""
        # Range-Doppler cube (matches Rust complex_cube shape)
        cube = edgefirst_msgs.RadarCube(
            header=sample_header,
            timestamp=0,
            layout=[
                edgefirst_msgs.RadarChannel.RANGE.value,
                edgefirst_msgs.RadarChannel.DOPPLER.value,
            ],
            shape=[64, 32],
            scales=[1.0, 0.1],  # Matches Rust
            cube=[0] * (64 * 32),
            is_complex=False,
        )
        data = cube.serialize()
        restored = edgefirst_msgs.RadarCube.deserialize(data)
        assert restored.layout[0] == edgefirst_msgs.RadarChannel.RANGE.value
        assert restored.layout[1] == edgefirst_msgs.RadarChannel.DOPPLER.value

    def test_radar_cube_complex(self, sample_header):
        """Test RadarCube with complex data (matches Rust complex_cube)."""
        cube = edgefirst_msgs.RadarCube(
            header=sample_header,
            timestamp=0,
            layout=[
                edgefirst_msgs.RadarChannel.RANGE.value,
                edgefirst_msgs.RadarChannel.DOPPLER.value,
            ],
            shape=[64, 32],
            scales=[1.0, 0.1],
            cube=[100, 200, -100, -200],  # Matches Rust test values
            is_complex=True,
        )
        data = cube.serialize()
        restored = edgefirst_msgs.RadarCube.deserialize(data)
        assert restored.is_complex is True
        assert list(restored.cube) == [100, 200, -100, -200]


class TestRadarInfo:
    """Tests for RadarInfo message."""

    def test_radar_info_creation(self, sample_header):
        """Test RadarInfo structure."""
        info = edgefirst_msgs.RadarInfo(
            header=sample_header,
            center_frequency="77GHz",
            frequency_sweep="1GHz",
            range_toggle="off",
            detection_sensitivity="high",
            cube=True,
        )
        assert info.center_frequency == "77GHz"
        assert info.cube is True

    def test_radar_info_serialize_deserialize(self, sample_header):
        """Test CDR serialization roundtrip."""
        info = edgefirst_msgs.RadarInfo(
            header=sample_header,
            center_frequency="79GHz",
            frequency_sweep="2GHz",
            range_toggle="short-long",
            detection_sensitivity="medium",
            cube=False,
        )
        data = info.serialize()
        restored = edgefirst_msgs.RadarInfo.deserialize(data)
        assert restored.center_frequency == "79GHz"
        assert restored.range_toggle == "short-long"
        assert restored.cube is False
