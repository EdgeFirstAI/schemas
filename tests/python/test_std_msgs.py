"""Tests for std_msgs module."""

import pytest
from edgefirst.schemas import std_msgs, builtin_interfaces


class TestHeader:
    """Tests for Header message."""

    def test_header_creation_defaults(self):
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
        assert isinstance(data, bytes)

        restored = std_msgs.Header.deserialize(data)
        assert restored.frame_id == sample_header.frame_id
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.stamp.nanosec == sample_header.stamp.nanosec

    def test_header_empty_frame_id(self, sample_time):
        """Test Header with empty frame_id."""
        header = std_msgs.Header(stamp=sample_time, frame_id="")
        data = header.serialize()
        restored = std_msgs.Header.deserialize(data)
        assert restored.frame_id == ""

    def test_header_long_frame_id(self, sample_time):
        """Test Header with long frame_id."""
        long_id = "sensor_" + "a" * 100
        header = std_msgs.Header(stamp=sample_time, frame_id=long_id)
        data = header.serialize()
        restored = std_msgs.Header.deserialize(data)
        assert restored.frame_id == long_id

    def test_header_special_chars_frame_id(self, sample_time):
        """Test Header with special characters in frame_id."""
        special_id = "sensor/camera_01/rgb"
        header = std_msgs.Header(stamp=sample_time, frame_id=special_id)
        data = header.serialize()
        restored = std_msgs.Header.deserialize(data)
        assert restored.frame_id == special_id


class TestColorRGBA:
    """Tests for ColorRGBA message."""

    def test_color_rgba_creation_defaults(self):
        """Test creating a ColorRGBA with default values."""
        color = std_msgs.ColorRGBA()
        assert color.r == 0.0
        assert color.g == 0.0
        assert color.b == 0.0
        assert color.a == 0.0

    def test_color_rgba_with_values(self):
        """Test creating ColorRGBA with specific values."""
        color = std_msgs.ColorRGBA(r=1.0, g=0.5, b=0.25, a=0.8)
        assert color.r == pytest.approx(1.0)
        assert color.g == pytest.approx(0.5)
        assert color.b == pytest.approx(0.25)
        assert color.a == pytest.approx(0.8)

    def test_color_rgba_serialize_deserialize(self, sample_color_rgba):
        """Test CDR serialization roundtrip."""
        data = sample_color_rgba.serialize()
        assert isinstance(data, bytes)

        restored = std_msgs.ColorRGBA.deserialize(data)
        assert restored.r == pytest.approx(sample_color_rgba.r)
        assert restored.g == pytest.approx(sample_color_rgba.g)
        assert restored.b == pytest.approx(sample_color_rgba.b)
        assert restored.a == pytest.approx(sample_color_rgba.a)

    def test_color_rgba_white(self):
        """Test fully white color."""
        color = std_msgs.ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        data = color.serialize()
        restored = std_msgs.ColorRGBA.deserialize(data)
        for val in [restored.r, restored.g, restored.b, restored.a]:
            assert val == pytest.approx(1.0)

    def test_color_rgba_black_transparent(self):
        """Test black transparent color."""
        color = std_msgs.ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)
        data = color.serialize()
        restored = std_msgs.ColorRGBA.deserialize(data)
        for val in [restored.r, restored.g, restored.b, restored.a]:
            assert val == pytest.approx(0.0)


class TestBool:
    """Tests for Bool message."""

    def test_bool_true(self):
        """Test Bool with True value."""
        msg = std_msgs.Bool(data=True)
        data = msg.serialize()
        restored = std_msgs.Bool.deserialize(data)
        assert restored.data is True

    def test_bool_false(self):
        """Test Bool with False value."""
        msg = std_msgs.Bool(data=False)
        data = msg.serialize()
        restored = std_msgs.Bool.deserialize(data)
        assert restored.data is False


class TestString:
    """Tests for String message."""

    def test_string_empty(self):
        """Test String with empty value."""
        msg = std_msgs.String(data="")
        data = msg.serialize()
        restored = std_msgs.String.deserialize(data)
        assert restored.data == ""

    def test_string_value(self):
        """Test String with value."""
        msg = std_msgs.String(data="Hello, World!")
        data = msg.serialize()
        restored = std_msgs.String.deserialize(data)
        assert restored.data == "Hello, World!"

    def test_string_long(self):
        """Test String with long value."""
        long_str = "x" * 10000
        msg = std_msgs.String(data=long_str)
        data = msg.serialize()
        restored = std_msgs.String.deserialize(data)
        assert restored.data == long_str


class TestNumericTypes:
    """Tests for numeric wrapper messages."""

    def test_int8(self):
        """Test Int8 message."""
        msg = std_msgs.Int8(data=-128)
        data = msg.serialize()
        restored = std_msgs.Int8.deserialize(data)
        assert restored.data == -128

    def test_uint8(self):
        """Test Uint8 message."""
        msg = std_msgs.Uint8(data=255)
        data = msg.serialize()
        restored = std_msgs.Uint8.deserialize(data)
        assert restored.data == 255

    def test_int16(self):
        """Test Int16 message."""
        msg = std_msgs.Int16(data=-32768)
        data = msg.serialize()
        restored = std_msgs.Int16.deserialize(data)
        assert restored.data == -32768

    def test_uint16(self):
        """Test Uint16 message."""
        msg = std_msgs.Uint16(data=65535)
        data = msg.serialize()
        restored = std_msgs.Uint16.deserialize(data)
        assert restored.data == 65535

    def test_int32(self):
        """Test Int32 message."""
        msg = std_msgs.Int32(data=-2147483648)
        data = msg.serialize()
        restored = std_msgs.Int32.deserialize(data)
        assert restored.data == -2147483648

    def test_uint32(self):
        """Test Uint32 message."""
        msg = std_msgs.Uint32(data=4294967295)
        data = msg.serialize()
        restored = std_msgs.Uint32.deserialize(data)
        assert restored.data == 4294967295

    def test_int64(self):
        """Test Int64 message."""
        msg = std_msgs.Int64(data=-9223372036854775808)
        data = msg.serialize()
        restored = std_msgs.Int64.deserialize(data)
        assert restored.data == -9223372036854775808

    def test_uint64(self):
        """Test Uint64 message."""
        msg = std_msgs.Uint64(data=18446744073709551615)
        data = msg.serialize()
        restored = std_msgs.Uint64.deserialize(data)
        assert restored.data == 18446744073709551615

    def test_float32(self):
        """Test Float32 message."""
        msg = std_msgs.Float32(data=3.14159)
        data = msg.serialize()
        restored = std_msgs.Float32.deserialize(data)
        assert restored.data == pytest.approx(3.14159, rel=1e-5)

    def test_float64(self):
        """Test Float64 message."""
        msg = std_msgs.Float64(data=3.141592653589793)
        data = msg.serialize()
        restored = std_msgs.Float64.deserialize(data)
        assert restored.data == pytest.approx(3.141592653589793)
