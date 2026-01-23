"""Tests for builtin_interfaces module."""

import pytest
from edgefirst.schemas import builtin_interfaces


class TestTime:
    """Tests for Time message."""

    def test_time_creation_defaults(self):
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

    def test_time_large_sec(self):
        """Test Time with large second value."""
        time = builtin_interfaces.Time(sec=2147483647, nanosec=0)
        data = time.serialize()
        restored = builtin_interfaces.Time.deserialize(data)
        assert restored.sec == 2147483647

    def test_time_zero(self):
        """Test Time with zero values (epoch)."""
        time = builtin_interfaces.Time(sec=0, nanosec=0)
        data = time.serialize()
        restored = builtin_interfaces.Time.deserialize(data)
        assert restored.sec == 0
        assert restored.nanosec == 0


class TestDuration:
    """Tests for Duration message."""

    def test_duration_creation_defaults(self):
        """Test creating a Duration message with default values."""
        duration = builtin_interfaces.Duration()
        assert duration.sec == 0
        assert duration.nanosec == 0

    def test_duration_with_values(self):
        """Test creating a Duration with specific values."""
        duration = builtin_interfaces.Duration(sec=60, nanosec=500000000)
        assert duration.sec == 60
        assert duration.nanosec == 500000000

    def test_duration_serialize_deserialize(self, sample_duration):
        """Test CDR serialization roundtrip."""
        data = sample_duration.serialize()
        assert isinstance(data, bytes)

        restored = builtin_interfaces.Duration.deserialize(data)
        assert restored.sec == sample_duration.sec
        assert restored.nanosec == sample_duration.nanosec

    def test_duration_negative_sec(self):
        """Test Duration with negative seconds (valid for durations)."""
        duration = builtin_interfaces.Duration(sec=-10, nanosec=500000000)
        data = duration.serialize()
        restored = builtin_interfaces.Duration.deserialize(data)
        assert restored.sec == -10
        assert restored.nanosec == 500000000

    def test_duration_equality(self):
        """Test Duration equality comparison."""
        d1 = builtin_interfaces.Duration(sec=10, nanosec=200)
        d2 = builtin_interfaces.Duration(sec=10, nanosec=200)
        d3 = builtin_interfaces.Duration(sec=10, nanosec=300)
        assert d1 == d2
        assert d1 != d3

    def test_duration_zero(self):
        """Test Duration with zero values."""
        duration = builtin_interfaces.Duration(sec=0, nanosec=0)
        data = duration.serialize()
        restored = builtin_interfaces.Duration.deserialize(data)
        assert restored.sec == 0
        assert restored.nanosec == 0
