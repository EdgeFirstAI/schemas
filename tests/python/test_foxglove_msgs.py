# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for `edgefirst.schemas.foxglove_msgs`."""

import numpy as np
import pytest

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.foxglove_msgs import (
    Color,
    CompressedImage,
    CompressedVideo,
    Point2,
)


class TestCompressedVideo:
    def test_round_trip(self):
        rng = np.random.default_rng(seed=0xC0DEC)
        payload = rng.bytes(50_000)
        cv = CompressedVideo(
            timestamp=Time(sec=1234567890, nanosec=123456789),
            frame_id="camera",
            data=payload,
            format="h264",
        )
        restored = CompressedVideo.from_cdr(cv.to_bytes())
        assert restored.timestamp.sec == 1234567890
        assert restored.frame_id == "camera"
        assert restored.format == "h264"
        assert restored.data.tobytes() == payload

    @pytest.mark.parametrize("format", ["h264", "h265", "vp9", "av1"])
    def test_format_round_trip(self, format):
        cv = CompressedVideo(
            timestamp=Time(0, 0),
            frame_id="cam",
            data=b"\x00\x00\x00\x01",  # NAL start code
            format=format,
        )
        assert CompressedVideo.from_cdr(cv.to_bytes()).format == format

    def test_data_view_zero_copy(self):
        payload = bytes(100_000)
        cv = CompressedVideo(
            timestamp=Time(0, 0), frame_id="cam", data=payload, format="h264"
        )
        # Two views must share storage — zero-copy parent ref pattern.
        v1 = cv.data.view()
        v2 = cv.data.view()
        a1 = np.frombuffer(v1, dtype=np.uint8)
        a2 = np.frombuffer(v2, dtype=np.uint8)
        assert a1.ctypes.data == a2.ctypes.data


class TestCompressedImage:
    def test_round_trip(self):
        rng = np.random.default_rng(seed=0x1A6E)
        payload = rng.bytes(50_000)
        ci = CompressedImage(
            timestamp=Time(sec=1234567890, nanosec=123456789),
            frame_id="camera",
            data=payload,
            format="jpeg",
        )
        restored = CompressedImage.from_cdr(ci.to_bytes())
        assert restored.timestamp.sec == 1234567890
        assert restored.frame_id == "camera"
        assert restored.format == "jpeg"
        assert restored.data.tobytes() == payload

    @pytest.mark.parametrize("format", ["jpeg", "png", "webp", "avif"])
    def test_format_round_trip(self, format):
        ci = CompressedImage(
            timestamp=Time(0, 0),
            frame_id="cam",
            data=b"\xff\xd8\xff\xe0",  # JPEG SOI + APP0
            format=format,
        )
        assert CompressedImage.from_cdr(ci.to_bytes()).format == format

    def test_data_view_zero_copy(self):
        payload = bytes(100_000)
        ci = CompressedImage(
            timestamp=Time(0, 0), frame_id="cam", data=payload, format="jpeg"
        )
        v1 = ci.data.view()
        v2 = ci.data.view()
        a1 = np.frombuffer(v1, dtype=np.uint8)
        a2 = np.frombuffer(v2, dtype=np.uint8)
        assert a1.ctypes.data == a2.ctypes.data

    def test_wire_identical_to_compressed_video(self):
        # CompressedImage and CompressedVideo share an identical CDR layout;
        # only the typename and the documented `format` semantics differ.
        kwargs = dict(
            timestamp=Time(sec=42, nanosec=7),
            frame_id="cam0",
            data=b"\x01\x02\x03\x04",
            format="h264",
        )
        assert (
            CompressedImage(**kwargs).to_bytes()
            == CompressedVideo(**kwargs).to_bytes()
        )


class TestPoint2:
    def test_round_trip(self):
        p = Point2(x=1.5, y=-2.5)
        restored = Point2.from_cdr(p.to_bytes())
        assert restored.x == 1.5
        assert restored.y == -2.5


class TestColor:
    def test_round_trip(self):
        # Foxglove Color uses f64 components (vs std_msgs.ColorRGBA's f32).
        c = Color(r=1.0, g=0.5, b=0.25, a=1.0)
        restored = Color.from_cdr(c.to_bytes())
        assert restored.r == 1.0
        assert restored.g == 0.5
        assert restored.b == 0.25
        assert restored.a == 1.0

    @pytest.mark.parametrize(
        "rgba",
        [
            (0.0, 0.0, 0.0, 0.0),
            (1.0, 1.0, 1.0, 1.0),
            (0.123456789, 0.987654321, 0.5, 0.75),
        ],
    )
    def test_f64_precision(self, rgba):
        # Foxglove's f64 colour components preserve full double precision —
        # unlike std_msgs.ColorRGBA which is f32.
        c = Color(*rgba)
        restored = Color.from_cdr(c.to_bytes())
        assert (restored.r, restored.g, restored.b, restored.a) == rgba


# ── CircleAnnotations ──────────────────────────────────────────────

from edgefirst.schemas.foxglove_msgs import CircleAnnotations


class TestCircleAnnotations:
    def test_round_trip(self):
        ca = CircleAnnotations(
            timestamp=Time(1, 0),
            position=Point2(100.0, 200.0),
            diameter=50.0,
            thickness=2.0,
            fill_color=Color(1.0, 0.0, 0.0, 1.0),
            outline_color=Color(0.0, 1.0, 0.0, 1.0),
        )
        restored = CircleAnnotations.from_cdr(ca.to_bytes())
        assert restored.timestamp.sec == 1
        assert restored.position.x == 100.0
        assert restored.diameter == 50.0
        assert restored.fill_color.r == 1.0

    def test_defaults(self):
        ca = CircleAnnotations()
        assert ca.diameter == 0.0


# ── TextAnnotation ─────────────────────────────────────────────────

from edgefirst.schemas.foxglove_msgs import TextAnnotation


class TestTextAnnotation:
    def test_round_trip(self):
        ta = TextAnnotation(
            timestamp=Time(1, 0),
            position=Point2(10.0, 20.0),
            text="Hello, world!",
            font_size=14.0,
            text_color=Color(1.0, 1.0, 1.0, 1.0),
        )
        restored = TextAnnotation.from_cdr(ta.to_bytes())
        assert restored.text == "Hello, world!"
        assert restored.font_size == 14.0
        assert restored.timestamp.sec == 1

    def test_empty_text(self):
        ta = TextAnnotation()
        restored = TextAnnotation.from_cdr(ta.to_bytes())
        assert restored.text == ""


# ── PointAnnotation ────────────────────────────────────────────────

from edgefirst.schemas.foxglove_msgs import PointAnnotation


class TestPointAnnotation:
    def test_round_trip(self):
        pa = PointAnnotation(
            timestamp=Time(2, 0),
            type_=1,
            points=[Point2(0.0, 0.0), Point2(100.0, 100.0)],
            outline_color=Color(0.0, 1.0, 0.0, 1.0),
            thickness=3.0,
        )
        restored = PointAnnotation.from_cdr(pa.to_bytes())
        assert restored.type_ == 1
        assert len(restored.points) == 2
        assert restored.points[1].x == 100.0
        assert restored.thickness == 3.0


# ── ImageAnnotation ────────────────────────────────────────────────

from edgefirst.schemas.foxglove_msgs import ImageAnnotation


class TestImageAnnotation:
    def test_round_trip_empty(self):
        ia = ImageAnnotation()
        restored = ImageAnnotation.from_cdr(ia.to_bytes())
        assert len(restored.circles) == 0
        assert len(restored.points) == 0
        assert len(restored.texts) == 0

    def test_round_trip_with_annotations(self):
        ca = CircleAnnotations(
            timestamp=Time(1, 0),
            position=Point2(50.0, 50.0),
            diameter=20.0,
        )
        ta = TextAnnotation(
            timestamp=Time(1, 0),
            text="label",
            font_size=12.0,
        )
        ia = ImageAnnotation(circles=[ca], texts=[ta])
        restored = ImageAnnotation.from_cdr(ia.to_bytes())
        assert len(restored.circles) == 1
        assert restored.circles[0].diameter == 20.0
        assert len(restored.texts) == 1
        assert restored.texts[0].text == "label"


# ── Legacy name aliases (pre-3.2.0 pure-Python / Foxglove typenames) ─

from edgefirst.schemas.foxglove_msgs import (
    CircleAnnotation,
    ImageAnnotations,
    PointsAnnotation,
)


class TestFoxgloveLegacyAliases:
    def test_circle_annotation_alias(self):
        assert CircleAnnotation is CircleAnnotations

    def test_points_annotation_alias(self):
        assert PointsAnnotation is PointAnnotation

    def test_image_annotations_alias(self):
        assert ImageAnnotations is ImageAnnotation

    def test_image_annotations_round_trip_via_alias(self):
        ia = ImageAnnotations(circles=[CircleAnnotation()])
        restored = ImageAnnotations.from_cdr(ia.to_bytes())
        assert len(restored.circles) == 1
