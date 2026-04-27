# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for `edgefirst.schemas.builtin_interfaces`.

`Time` and `Duration` are CdrFixed types — small, fixed wire size, with
`to_bytes()` / `from_cdr()` for round-trip and ordinary equality
semantics. The pyo3 wrapper exposes immutable properties only; mutation
would require rebuilding the value, matching ROS 2 IDL semantics.
"""

import pytest

from edgefirst.schemas.builtin_interfaces import Duration, Time


class TestTime:
    def test_defaults(self):
        t = Time()
        assert t.sec == 0
        assert t.nanosec == 0

    def test_explicit_values(self):
        t = Time(sec=100, nanosec=500_000_000)
        assert t.sec == 100
        assert t.nanosec == 500_000_000

    def test_round_trip(self, sample_time):
        data = sample_time.to_bytes()
        assert isinstance(data, bytes)
        # CDR header (4) + i32 sec (4) + u32 nanosec (4) = 12 bytes.
        assert len(data) == 12

        restored = Time.from_cdr(data)
        assert restored.sec == sample_time.sec
        assert restored.nanosec == sample_time.nanosec

    def test_equality_and_hash(self):
        a = Time(100, 200)
        b = Time(100, 200)
        c = Time(100, 300)
        assert a == b
        assert a != c
        assert hash(a) == hash(b)

    def test_max_nanosec(self):
        t = Time(sec=0, nanosec=999_999_999)
        restored = Time.from_cdr(t.to_bytes())
        assert restored.nanosec == 999_999_999

    def test_max_sec(self):
        t = Time(sec=2**31 - 1, nanosec=0)
        restored = Time.from_cdr(t.to_bytes())
        assert restored.sec == 2**31 - 1

    def test_from_nanos_round_trip(self):
        t = Time.from_nanos(1_234_567_890_123_456_789)
        assert t.to_nanos() == 1_234_567_890_123_456_789

    def test_to_nanos_negative_returns_none(self):
        # Pre-epoch timestamps can't fit in u64; the API returns None
        # rather than wrapping or raising.
        assert Time(sec=-1, nanosec=0).to_nanos() is None

    def test_repr_contains_field_values(self):
        # Repr should make field values visible (round-trip through
        # parsing isn't promised).
        r = repr(Time(42, 100))
        assert "42" in r and "100" in r


class TestDuration:
    def test_defaults(self):
        d = Duration()
        assert d.sec == 0
        assert d.nanosec == 0

    def test_explicit_values(self):
        d = Duration(sec=60, nanosec=500_000_000)
        assert d.sec == 60
        assert d.nanosec == 500_000_000

    def test_negative_sec_round_trip(self):
        # Durations are signed — negative values are valid.
        d = Duration(sec=-10, nanosec=500_000_000)
        restored = Duration.from_cdr(d.to_bytes())
        assert restored.sec == -10
        assert restored.nanosec == 500_000_000

    def test_equality(self):
        assert Duration(10, 200) == Duration(10, 200)
        assert Duration(10, 200) != Duration(10, 300)

    def test_round_trip(self, sample_duration):
        restored = Duration.from_cdr(sample_duration.to_bytes())
        assert restored.sec == sample_duration.sec
        assert restored.nanosec == sample_duration.nanosec


@pytest.mark.parametrize(
    "sec,nanosec",
    [
        (0, 0),
        (1, 0),
        (0, 1),
        (1234567890, 123456789),
        (2**31 - 1, 999_999_999),
        (-1, 999_999_999),
    ],
)
def test_time_duration_round_trip_values(sec, nanosec):
    """Sweep edge values across both types — same wire format."""
    if sec >= 0:
        t = Time(sec, nanosec)
        restored = Time.from_cdr(t.to_bytes())
        assert (restored.sec, restored.nanosec) == (sec, nanosec)

    d = Duration(sec, nanosec)
    restored_d = Duration.from_cdr(d.to_bytes())
    assert (restored_d.sec, restored_d.nanosec) == (sec, nanosec)
