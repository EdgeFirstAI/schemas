# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for `edgefirst.schemas.std_msgs`.

`Header` is buffer-backed (owned `Vec<u8>` plus offset table). `ColorRGBA`
is CdrFixed (16 bytes payload). The pyo3 wrappers expose the same surface
the Rust crate documents: read-only properties + `to_bytes()` /
`from_cdr()`.
"""

import pytest

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.std_msgs import ColorRGBA, Header


class TestHeader:
    def test_defaults(self):
        h = Header()
        assert h.stamp == Time(0, 0)
        assert h.frame_id == ""

    def test_explicit_values(self, sample_header):
        assert sample_header.stamp.sec == 1234567890
        assert sample_header.stamp.nanosec == 123456789
        assert sample_header.frame_id == "sensor_frame"

    def test_round_trip(self, sample_header):
        restored = Header.from_cdr(sample_header.to_bytes())
        assert restored.stamp == sample_header.stamp
        assert restored.frame_id == sample_header.frame_id

    @pytest.mark.parametrize(
        "frame_id",
        [
            "",
            "camera",
            "camera/optical_frame",
            "/odom",
            # Frame IDs commonly include numbers and underscores.
            "imu_link_2",
            # UTF-8 codepoints round-trip through CDR strings.
            "caméra",
        ],
    )
    def test_frame_id_round_trip(self, frame_id):
        h = Header(stamp=Time(1, 0), frame_id=frame_id)
        restored = Header.from_cdr(h.to_bytes())
        assert restored.frame_id == frame_id

    def test_cdr_size_reports_buffer_length(self, sample_header):
        # `cdr_size` is the underlying buffer's byte length, must match
        # the bytes returned by `to_bytes()` since they're the same buffer.
        assert sample_header.cdr_size == len(sample_header.to_bytes())


class TestColorRGBA:
    def test_defaults(self):
        c = ColorRGBA()
        assert c.r == 0.0 and c.g == 0.0 and c.b == 0.0 and c.a == 0.0

    def test_explicit_values(self, sample_color):
        assert sample_color.r == 1.0
        assert sample_color.g == 0.5
        assert sample_color.b == 0.25
        assert sample_color.a == 1.0

    def test_round_trip(self, sample_color):
        data = sample_color.to_bytes()
        # CDR header (4) + 4 × f32 (16) = 20 bytes.
        assert len(data) == 20
        restored = ColorRGBA.from_cdr(data)
        # f32 values exact-equal because we picked dyadic fractions.
        assert restored.r == sample_color.r
        assert restored.g == sample_color.g
        assert restored.b == sample_color.b
        assert restored.a == sample_color.a

    @pytest.mark.parametrize(
        "rgba",
        [
            (0.0, 0.0, 0.0, 0.0),
            (1.0, 1.0, 1.0, 1.0),
            (0.5, 0.25, 0.125, 0.0625),
        ],
    )
    def test_round_trip_values(self, rgba):
        c = ColorRGBA(*rgba)
        restored = ColorRGBA.from_cdr(c.to_bytes())
        assert (restored.r, restored.g, restored.b, restored.a) == rgba
