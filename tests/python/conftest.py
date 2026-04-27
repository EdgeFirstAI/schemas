# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Shared pytest fixtures for the pyo3 `edgefirst.schemas` test suite.

The old pycdr2-era fixtures relied on dataclass-style construction
(field-by-field after instantiation); the new pyo3 API takes the same
fields as constructor kwargs, so most fixtures collapse to one-liners.
"""

import numpy as np
import pytest

from edgefirst.schemas.builtin_interfaces import Duration, Time
from edgefirst.schemas.std_msgs import ColorRGBA, Header


# ── builtin_interfaces ─────────────────────────────────────────────


@pytest.fixture
def sample_time() -> Time:
    return Time(sec=1234567890, nanosec=123456789)


@pytest.fixture
def sample_duration() -> Duration:
    return Duration(sec=60, nanosec=500_000_000)


# ── std_msgs ───────────────────────────────────────────────────────


@pytest.fixture
def sample_header(sample_time: Time) -> Header:
    return Header(stamp=sample_time, frame_id="sensor_frame")


@pytest.fixture
def sample_color() -> ColorRGBA:
    return ColorRGBA(r=1.0, g=0.5, b=0.25, a=1.0)


# ── shared bulk-payload helpers ────────────────────────────────────


@pytest.fixture
def zero_pixels_hd_rgb8() -> np.ndarray:
    """720x1280x3 zeroed RGB8 frame, common bulk-payload fixture."""
    return np.zeros(720 * 1280 * 3, dtype=np.uint8)


@pytest.fixture
def random_bytes_100kb() -> bytes:
    """100 KiB of pseudo-random bytes for compressed-payload tests."""
    rng = np.random.default_rng(seed=0xC0FFEE)
    return rng.bytes(100 * 1024)
