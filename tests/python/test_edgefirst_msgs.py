# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for `edgefirst.schemas.edgefirst_msgs`.

Exercises the four wrapped types:
- ``Date`` — CdrFixed (4-byte payload, year/month/day).
- ``DmaBuffer`` — buffer-backed metadata-only message (deprecated upstream).
- ``Mask`` — buffer-backed segmentation mask with bulk byte payload.
- ``RadarCube`` — buffer-backed with multiple typed array fields
  (layout u8, shape u16, scales f32, cube i16).
"""

import numpy as np
import pytest

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.edgefirst_msgs import Date, DmaBuffer, LocalTime, Mask, RadarCube, Track
from edgefirst.schemas.std_msgs import Header


# ── Date ───────────────────────────────────────────────────────────


class TestDate:
    def test_defaults(self):
        d = Date()
        assert (d.year, d.month, d.day) == (1970, 1, 1)

    @pytest.mark.parametrize(
        "year,month,day",
        [
            (2026, 4, 27),
            (1970, 1, 1),
            (2099, 12, 31),
            (2000, 2, 29),  # leap day
        ],
    )
    def test_round_trip(self, year, month, day):
        d = Date(year=year, month=month, day=day)
        restored = Date.from_cdr(d.to_bytes())
        assert (restored.year, restored.month, restored.day) == (year, month, day)


# ── DmaBuffer (deprecated upstream, kept for bench parity) ─────────


class TestDmaBuffer:
    @pytest.mark.filterwarnings("ignore::DeprecationWarning")
    def test_round_trip(self, sample_header):
        # YUYV fourcc = 'YUYV' = 0x56595559
        db = DmaBuffer(
            header=sample_header,
            pid=12345, fd=42,
            width=1920, height=1080,
            stride=1920 * 2, fourcc=0x56595559,
            length=1920 * 1080 * 2,
        )
        restored = DmaBuffer.from_cdr(db.to_bytes())
        assert restored.pid == 12345
        assert restored.fd == 42
        assert restored.width == 1920
        assert restored.height == 1080
        assert restored.stride == 1920 * 2
        assert restored.fourcc == 0x56595559
        assert restored.length == 1920 * 1080 * 2

    @pytest.mark.filterwarnings("ignore::DeprecationWarning")
    def test_negative_fd_round_trip(self, sample_header):
        # fd is i32 — sentinel values may be negative in producer code.
        db = DmaBuffer(
            header=sample_header,
            pid=1, fd=-1,
            width=1, height=1, stride=1, fourcc=0, length=0,
        )
        assert DmaBuffer.from_cdr(db.to_bytes()).fd == -1


# ── Mask ───────────────────────────────────────────────────────────


class TestMask:
    @pytest.mark.parametrize(
        "h,w,channels",
        [
            (320, 320, 1),    # ModelPack semantic HD
            (480, 480, 1),    # ModelPack semantic FHD
            (640, 640, 1),    # YOLO instance mask
            (1280, 720, 1),   # retina-scaled HD
        ],
    )
    def test_uncompressed_round_trip(self, h, w, channels):
        # encoding="" is the raw uint8 path.
        rng = np.random.default_rng(seed=h * w + channels)
        payload = rng.integers(0, 256, size=h * w * channels, dtype=np.uint8)
        m = Mask(
            height=h, width=w, length=channels,
            encoding="", mask=payload, boxed=False,
        )
        restored = Mask.from_cdr(m.to_bytes())
        assert restored.height == h
        assert restored.width == w
        assert restored.length == channels
        assert restored.encoding == ""
        assert restored.boxed is False
        # Bulk payload — verify byte-for-byte preservation.
        assert restored.mask.tobytes() == bytes(payload)

    def test_zstd_encoding_marker(self):
        # `encoding` is just a string — payload bytes are opaque to the
        # codec, so we don't actually compress here, just check the
        # marker round-trips.
        m = Mask(
            height=1, width=1, length=1,
            encoding="zstd", mask=b"\x01\x02\x03",
            boxed=True,
        )
        restored = Mask.from_cdr(m.to_bytes())
        assert restored.encoding == "zstd"
        assert restored.boxed is True
        assert restored.mask.tobytes() == b"\x01\x02\x03"


# ── RadarCube ──────────────────────────────────────────────────────


class TestRadarCube:
    def test_round_trip_typed_arrays(self, sample_header):
        # DRVEGRD-169 short-range shape: (chirps, ranges, rx, doppler×2).
        shape = np.array([2, 128, 12, 128], dtype=np.uint16)
        scales = np.array([1.0, 0.117, 1.0, 0.156], dtype=np.float32)
        layout = np.array([6, 1, 5, 2], dtype=np.uint8)
        cube_len = int(np.prod(shape))
        rng = np.random.default_rng(seed=42)
        cube = rng.integers(-32768, 32767, size=cube_len, dtype=np.int16)

        # `.tobytes()` is the portable input form — abi3-py38 wheels
        # don't expose Py_buffer in the limited API, so typed numpy
        # arrays go through bytes. On Py 3.11+ / non-abi3 builds the
        # typed arrays themselves work directly.
        rc = RadarCube(
            header=sample_header,
            timestamp=1234567890123456,
            layout=layout.tobytes(),
            shape=shape.tobytes(),
            scales=scales.tobytes(),
            cube=cube.tobytes(),
            is_complex=True,
        )

        restored = RadarCube.from_cdr(rc.to_bytes())
        assert restored.timestamp == 1234567890123456
        assert restored.is_complex is True

        # Each typed array reinterprets through the documented dtype.
        layout_back = np.frombuffer(restored.layout.view(), dtype=np.uint8)
        shape_back = np.frombuffer(restored.shape.view(), dtype=np.uint16)
        scales_back = np.frombuffer(restored.scales.view(), dtype=np.float32)
        cube_back = np.frombuffer(restored.cube.view(), dtype=np.int16)

        assert np.array_equal(layout_back, layout)
        assert np.array_equal(shape_back, shape)
        assert np.allclose(scales_back, scales)
        assert np.array_equal(cube_back, cube)

    def test_accepts_bytes_for_layout(self, sample_header):
        # `layout` is a uint8 sequence — bytes objects work as input
        # without needing `.tobytes()` because they're already byte-typed.
        rc = RadarCube(
            header=sample_header,
            timestamp=0,
            layout=b"\x06\x01\x05\x02",
            shape=np.array([1, 1, 1, 1], dtype=np.uint16).tobytes(),
            scales=np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32).tobytes(),
            cube=np.array([0], dtype=np.int16).tobytes(),
            is_complex=False,
        )
        assert rc.layout.tobytes() == b"\x06\x01\x05\x02"

    def test_invalid_shape_byte_count_raises(self, sample_header):
        # shape is uint16 — the buffer length must be a multiple of 2.
        with pytest.raises(ValueError, match="shape"):
            RadarCube(
                header=sample_header,
                timestamp=0,
                layout=b"",
                shape=b"\x01\x02\x03",  # 3 bytes — invalid
                scales=b"",
                cube=b"",
                is_complex=False,
            )

    def test_invalid_scales_byte_count_raises(self, sample_header):
        # scales is f32 — buffer must be a multiple of 4.
        with pytest.raises(ValueError, match="scales"):
            RadarCube(
                header=sample_header,
                timestamp=0,
                layout=b"",
                shape=b"",
                scales=b"\x01\x02\x03",  # 3 bytes — invalid
                cube=b"",
                is_complex=False,
            )


# ── LocalTime ──────────────────────────────────────────────────────


class TestLocalTime:
    def test_round_trip(self, sample_header):
        lt = LocalTime(
            header=sample_header,
            date=Date(year=2026, month=4, day=27),
            time=Time(sec=3600, nanosec=500_000_000),  # 01:00:00.5
            timezone=-300,  # UTC-5 in minutes
        )
        restored = LocalTime.from_cdr(lt.to_bytes())
        assert restored.frame_id == sample_header.frame_id
        assert restored.date.year == 2026
        assert restored.date.month == 4
        assert restored.date.day == 27
        assert restored.time.sec == 3600
        assert restored.time.nanosec == 500_000_000
        assert restored.timezone == -300

    @pytest.mark.parametrize(
        "tz",
        [-720, -300, 0, 60, 330, 720],  # UTC-12, UTC-5, UTC, UTC+1, UTC+5:30, UTC+12
    )
    def test_timezone_range(self, sample_header, tz):
        # Timezones span ±12 hours in minutes; +05:30 (India) is the
        # canonical 30-minute offset case.
        lt = LocalTime(header=sample_header, timezone=tz)
        assert LocalTime.from_cdr(lt.to_bytes()).timezone == tz


# ── Track ──────────────────────────────────────────────────────────


class TestTrack:
    def test_round_trip(self):
        # Track has no Header — it's a standalone tracking record.
        t = Track(id="object-42", lifetime=120, created=Time(sec=1234567890, nanosec=0))
        restored = Track.from_cdr(t.to_bytes())
        assert restored.id == "object-42"
        assert restored.lifetime == 120
        assert restored.created.sec == 1234567890

    def test_empty_id_means_untracked(self):
        # The id is empty when the object isn't being tracked.
        t = Track(id="", lifetime=0, created=Time(0, 0))
        assert Track.from_cdr(t.to_bytes()).id == ""


# ── DetectBox / Detect ─────────────────────────────────────────────

from edgefirst.schemas.edgefirst_msgs import Detect, DetectBox


class TestDetect:
    def test_round_trip_with_boxes(self, sample_header):
        box1 = DetectBox(label="cat", score=0.95, center_x=0.5, center_y=0.5, width=0.2, height=0.3)
        box2 = DetectBox(label="dog", score=0.80, center_x=0.1, center_y=0.9, width=0.15, height=0.25)
        det = Detect(header=sample_header, input_timestamp=Time(0, 1000), boxes=[box1, box2])
        restored = Detect.from_cdr(det.to_bytes())
        assert len(restored.boxes) == 2
        assert restored.boxes[0].label == "cat"
        assert abs(restored.boxes[0].score - 0.95) < 0.001
        assert restored.boxes[1].label == "dog"
        assert restored.input_timestamp.nanosec == 1000

    def test_empty_boxes(self, sample_header):
        det = Detect(header=sample_header)
        restored = Detect.from_cdr(det.to_bytes())
        assert len(restored.boxes) == 0


# ── CameraFrame / CameraPlane ─────────────────────────────────────

from edgefirst.schemas.edgefirst_msgs import CameraFrame, CameraPlane


class TestCameraFrame:
    def test_round_trip(self, sample_header):
        plane = CameraPlane(fd=5, offset=0, stride=1920, size=1920 * 1080)
        cf = CameraFrame(
            header=sample_header,
            width=1920,
            height=1080,
            format="NV12",
            planes=[plane],
        )
        restored = CameraFrame.from_cdr(cf.to_bytes())
        assert restored.width == 1920
        assert restored.height == 1080
        assert restored.format == "NV12"
        assert len(restored.planes) == 1
        assert restored.planes[0].fd == 5
        assert restored.planes[0].stride == 1920

    def test_no_planes(self, sample_header):
        cf = CameraFrame(header=sample_header, width=640, height=480)
        restored = CameraFrame.from_cdr(cf.to_bytes())
        assert len(restored.planes) == 0

    def test_inline_data_plane(self, sample_header):
        pixel_data = bytes([0xFF] * 64)
        plane = CameraPlane(fd=-1, offset=0, stride=8, size=64, data=pixel_data)
        assert plane.fd == -1
        assert plane.data == pixel_data
        cf = CameraFrame(
            header=sample_header, width=8, height=8, format="GRAY8",
            planes=[plane],
        )
        restored = CameraFrame.from_cdr(cf.to_bytes())
        assert restored.planes[0].fd == -1
        assert restored.planes[0].data == pixel_data

    def test_inline_data_requires_fd_minus_one(self):
        import pytest
        with pytest.raises(ValueError, match="inline data.*fd == -1"):
            CameraPlane(fd=3, data=b"\x00\x01\x02")


# ── Model / MaskBox ───────────────────────────────────────────────

from edgefirst.schemas.edgefirst_msgs import Model, MaskBox


class TestModel:
    def test_round_trip_with_boxes(self, sample_header):
        from edgefirst.schemas.builtin_interfaces import Duration
        box = DetectBox(label="person", score=0.92, center_x=0.5, center_y=0.5, width=0.3, height=0.6)
        m = Model(header=sample_header, model_time=Duration(0, 500), boxes=[box])
        restored = Model.from_cdr(m.to_bytes())
        assert len(restored.boxes) == 1
        assert restored.boxes[0].label == "person"
        assert restored.model_time.nanosec == 500

    def test_empty_model(self, sample_header):
        m = Model(header=sample_header)
        restored = Model.from_cdr(m.to_bytes())
        assert len(restored.boxes) == 0
        assert len(restored.masks) == 0

    def test_round_trip_with_masks(self, sample_header):
        mask_data = bytes(range(24))  # 2x3x4 mask
        mask = MaskBox(height=2, width=3, length=4, encoding="", mask=mask_data)
        m = Model(header=sample_header, masks=[mask])
        restored = Model.from_cdr(m.to_bytes())
        assert len(restored.masks) == 1
        assert restored.masks[0].height == 2
        assert restored.masks[0].width == 3
        assert restored.masks[0].length == 4
        assert restored.masks[0].mask == mask_data


class TestMaskBox:
    def test_defaults(self):
        mb = MaskBox()
        assert mb.height == 0
        assert mb.width == 0
        assert mb.length == 0
        assert mb.encoding == ""
        assert mb.mask == b""
        assert mb.boxed is False

    def test_explicit_values(self):
        data = bytes([1, 2, 3, 4])
        mb = MaskBox(height=10, width=20, length=1, encoding="zstd", mask=data, boxed=True)
        assert mb.height == 10
        assert mb.width == 20
        assert mb.length == 1
        assert mb.encoding == "zstd"
        assert mb.mask == data
        assert mb.boxed is True


# ── ModelInfo ─────────────────────────────────────────────────────

from edgefirst.schemas.edgefirst_msgs import ModelInfo


class TestModelInfo:
    def test_round_trip(self, sample_header):
        mi = ModelInfo(
            header=sample_header,
            model_name="yolov8n",
            model_type="detection",
            model_format="tflite",
            input_shape=[1, 320, 320, 3],
            labels=["cat", "dog", "person"],
        )
        restored = ModelInfo.from_cdr(mi.to_bytes())
        assert restored.model_name == "yolov8n"
        assert restored.model_type == "detection"
        assert restored.model_format == "tflite"
        assert restored.input_shape == [1, 320, 320, 3]
        assert restored.labels == ["cat", "dog", "person"]


# ── RadarInfo ─────────────────────────────────────────────────────

from edgefirst.schemas.edgefirst_msgs import RadarInfo


class TestRadarInfo:
    def test_round_trip(self, sample_header):
        ri = RadarInfo(
            header=sample_header,
            center_frequency="79GHz",
            frequency_sweep="1GHz",
            cube=True,
        )
        restored = RadarInfo.from_cdr(ri.to_bytes())
        assert restored.center_frequency == "79GHz"
        assert restored.frequency_sweep == "1GHz"
        assert restored.cube is True

    def test_defaults(self, sample_header):
        ri = RadarInfo(header=sample_header)
        assert ri.center_frequency == ""
        assert ri.cube is False


# ── Vibration ─────────────────────────────────────────────────────

from edgefirst.schemas.edgefirst_msgs import Vibration
from edgefirst.schemas.geometry_msgs import Vector3


class TestVibration:
    def test_round_trip(self, sample_header):
        v = Vibration(
            header=sample_header,
            vibration=Vector3(0.1, 0.2, 0.3),
            band_lower_hz=10.0,
            band_upper_hz=1000.0,
        )
        restored = Vibration.from_cdr(v.to_bytes())
        assert abs(restored.vibration.x - 0.1) < 1e-9
        assert abs(restored.band_lower_hz - 10.0) < 1e-9
        assert abs(restored.band_upper_hz - 1000.0) < 1e-9
