# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for `edgefirst.schemas.foxglove_msgs`.

The pyo3 module currently wraps `CompressedVideo` from the Foxglove
schema set. Other Foxglove types (TextAnnotation, PointAnnotation,
ImageAnnotation, …) are present in the Rust crate but not yet exposed
to Python.
"""

import numpy as np
import pytest

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.foxglove_msgs import Color, CompressedVideo, Point2


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
