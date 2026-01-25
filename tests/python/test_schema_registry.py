# SPDX-License-Identifier: Apache-2.0
# Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

"""Tests for schema registry functions."""

import pytest

from edgefirst.schemas import (
    from_schema,
    is_supported,
    list_schemas,
    schema_name,
)
from edgefirst.schemas import (
    edgefirst_msgs,
    foxglove_msgs,
    geometry_msgs,
    sensor_msgs,
    std_msgs,
)


class TestFromSchema:
    """Tests for from_schema() function."""

    def test_sensor_msgs_image(self):
        """Test getting sensor_msgs.Image from schema name."""
        cls = from_schema("sensor_msgs/msg/Image")
        assert cls is sensor_msgs.Image

    def test_geometry_msgs_pose(self):
        """Test getting geometry_msgs.Pose from schema name."""
        cls = from_schema("geometry_msgs/msg/Pose")
        assert cls is geometry_msgs.Pose

    def test_edgefirst_msgs_box(self):
        """Test getting edgefirst_msgs.Box from schema name."""
        cls = from_schema("edgefirst_msgs/msg/Box")
        assert cls is edgefirst_msgs.Box

    def test_foxglove_msgs_compressed_video(self):
        """Test getting foxglove_msgs.CompressedVideo from schema name."""
        cls = from_schema("foxglove_msgs/msg/CompressedVideo")
        assert cls is foxglove_msgs.CompressedVideo

    def test_std_msgs_header(self):
        """Test getting std_msgs.Header from schema name."""
        cls = from_schema("std_msgs/msg/Header")
        assert cls is std_msgs.Header

    def test_short_format(self):
        """Test short format (package/TypeName) also works."""
        cls = from_schema("sensor_msgs/Image")
        assert cls is sensor_msgs.Image

    def test_invalid_format_raises(self):
        """Test invalid schema format raises ValueError."""
        with pytest.raises(ValueError, match="Invalid schema format"):
            from_schema("invalid")

    def test_wrong_msg_marker_raises(self):
        """Test wrong message marker raises ValueError."""
        with pytest.raises(ValueError, match="expected 'msg'"):
            from_schema("sensor_msgs/srv/Image")

    def test_unknown_package_raises(self):
        """Test unknown package raises KeyError."""
        with pytest.raises(KeyError, match="Unknown package"):
            from_schema("unknown_msgs/msg/Foo")

    def test_unknown_type_raises(self):
        """Test unknown type in known package raises KeyError."""
        with pytest.raises(KeyError, match="Unknown type"):
            from_schema("sensor_msgs/msg/NonExistent")

    def test_can_instantiate_class(self):
        """Test that returned class can be instantiated."""
        cls = from_schema("sensor_msgs/msg/Image")
        instance = cls()
        assert isinstance(instance, sensor_msgs.Image)


class TestSchemaName:
    """Tests for schema_name() function."""

    def test_sensor_msgs_image_class(self):
        """Test getting schema name from sensor_msgs.Image class."""
        name = schema_name(sensor_msgs.Image)
        assert name == "sensor_msgs/msg/Image"

    def test_geometry_msgs_pose_class(self):
        """Test getting schema name from geometry_msgs.Pose class."""
        name = schema_name(geometry_msgs.Pose)
        assert name == "geometry_msgs/msg/Pose"

    def test_edgefirst_msgs_box_class(self):
        """Test getting schema name from edgefirst_msgs.Box class."""
        name = schema_name(edgefirst_msgs.Box)
        assert name == "edgefirst_msgs/msg/Box"

    def test_from_instance(self):
        """Test getting schema name from instance."""
        img = sensor_msgs.Image()
        name = schema_name(img)
        assert name == "sensor_msgs/msg/Image"

    def test_invalid_type_raises(self):
        """Test non-schema type raises ValueError."""
        with pytest.raises(ValueError, match="does not have a typename"):
            schema_name(str)

    def test_roundtrip(self):
        """Test from_schema and schema_name are inverses."""
        original = "sensor_msgs/msg/CameraInfo"
        cls = from_schema(original)
        name = schema_name(cls)
        assert name == original


class TestListSchemas:
    """Tests for list_schemas() function."""

    def test_returns_list(self):
        """Test that list_schemas returns a list."""
        schemas = list_schemas()
        assert isinstance(schemas, list)

    def test_contains_common_schemas(self):
        """Test that common schemas are in the list."""
        schemas = list_schemas()
        assert "sensor_msgs/msg/Image" in schemas
        assert "geometry_msgs/msg/Pose" in schemas
        assert "edgefirst_msgs/msg/Box" in schemas

    def test_filter_by_package(self):
        """Test filtering schemas by package."""
        sensor_schemas = list_schemas("sensor_msgs")
        assert all(s.startswith("sensor_msgs/") for s in sensor_schemas)
        assert "sensor_msgs/msg/Image" in sensor_schemas

    def test_unknown_package_returns_empty(self):
        """Test unknown package returns empty list."""
        schemas = list_schemas("unknown_pkg")
        assert schemas == []

    def test_sorted(self):
        """Test that schemas are sorted."""
        schemas = list_schemas()
        assert schemas == sorted(schemas)


class TestIsSupported:
    """Tests for is_supported() function."""

    def test_supported_schema(self):
        """Test supported schema returns True."""
        assert is_supported("sensor_msgs/msg/Image")
        assert is_supported("geometry_msgs/msg/Pose")
        assert is_supported("edgefirst_msgs/msg/Box")

    def test_unsupported_schema(self):
        """Test unsupported schema returns False."""
        assert not is_supported("unknown_msgs/msg/Foo")
        assert not is_supported("sensor_msgs/msg/NonExistent")

    def test_invalid_format(self):
        """Test invalid format returns False (not raises)."""
        assert not is_supported("invalid")


class TestCrossLanguageCompatibility:
    """Tests ensuring Python schema names match Rust conventions."""

    # List of schemas that must match between Python and Rust
    COMMON_SCHEMAS = [
        "sensor_msgs/msg/CameraInfo",
        "sensor_msgs/msg/CompressedImage",
        "sensor_msgs/msg/Image",
        "sensor_msgs/msg/Imu",
        "sensor_msgs/msg/NavSatFix",
        "sensor_msgs/msg/PointCloud2",
        "geometry_msgs/msg/Pose",
        "geometry_msgs/msg/Transform",
        "geometry_msgs/msg/TransformStamped",
        "geometry_msgs/msg/Twist",
        "geometry_msgs/msg/Vector3",
        "geometry_msgs/msg/Quaternion",
        "foxglove_msgs/msg/CompressedVideo",
        "edgefirst_msgs/msg/Box",
        "edgefirst_msgs/msg/Detect",
        "edgefirst_msgs/msg/DmaBuffer",
        "edgefirst_msgs/msg/Mask",
        "edgefirst_msgs/msg/ModelInfo",
        "edgefirst_msgs/msg/RadarCube",
        "edgefirst_msgs/msg/RadarInfo",
        "edgefirst_msgs/msg/Track",
    ]

    @pytest.mark.parametrize("schema", COMMON_SCHEMAS)
    def test_schema_supported(self, schema: str):
        """Test all common schemas are supported."""
        assert is_supported(schema), f"Schema {schema} should be supported"

    @pytest.mark.parametrize("schema", COMMON_SCHEMAS)
    def test_schema_roundtrip(self, schema: str):
        """Test from_schema and schema_name roundtrip for all schemas."""
        cls = from_schema(schema)
        name = schema_name(cls)
        assert name == schema, f"Roundtrip failed for {schema}: got {name}"
