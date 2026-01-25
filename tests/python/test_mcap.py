"""MCAP decode tests using real sensor data.

These tests validate that EdgeFirst Schemas can correctly deserialize
CDR-encoded messages from real MCAP recordings captured on hardware devices.

Test data should be placed in the testdata/ directory. All .mcap files
found there will be automatically tested.

Tests FAIL (not skip) for:
- Unsupported schema types
- Deserialization errors
- Round-trip serialization mismatches
"""

from collections import defaultdict
from pathlib import Path
from typing import Any

from mcap.reader import make_reader

from edgefirst.schemas import (
    edgefirst_msgs,
    foxglove_msgs,
    geometry_msgs,
    sensor_msgs,
)

# Schema name to Python class mapping
# This must include ALL schema types that may appear in test MCAP files.
#
# Cross-language status:
# - Types marked [Rust: ✓] have equivalent Rust implementations
# - Types marked [Rust: -] are Python-only (Rust support pending)
# - Types marked [Rust: name] have Rust equivalent with different name
#
# Note: sensor_msgs/PointField is a nested struct used within PointCloud2,
# not typically a standalone top-level schema in MCAP recordings.
SCHEMA_MAP: dict[str, type] = {
    # sensor_msgs [Rust: ✓ all]
    "sensor_msgs/msg/CameraInfo": sensor_msgs.CameraInfo,
    "sensor_msgs/msg/CompressedImage": sensor_msgs.CompressedImage,
    "sensor_msgs/msg/Image": sensor_msgs.Image,
    "sensor_msgs/msg/Imu": sensor_msgs.Imu,
    "sensor_msgs/msg/NavSatFix": sensor_msgs.NavSatFix,
    "sensor_msgs/msg/PointCloud2": sensor_msgs.PointCloud2,
    "sensor_msgs/msg/PointField": sensor_msgs.PointField,
    # geometry_msgs [Rust: ✓ except PoseStamped]
    "geometry_msgs/msg/Transform": geometry_msgs.Transform,
    "geometry_msgs/msg/TransformStamped": geometry_msgs.TransformStamped,
    "geometry_msgs/msg/Vector3": geometry_msgs.Vector3,
    "geometry_msgs/msg/Quaternion": geometry_msgs.Quaternion,
    "geometry_msgs/msg/Pose": geometry_msgs.Pose,
    "geometry_msgs/msg/PoseStamped": geometry_msgs.PoseStamped,  # [Rust: -]
    "geometry_msgs/msg/Point": geometry_msgs.Point,
    "geometry_msgs/msg/Twist": geometry_msgs.Twist,
    "geometry_msgs/msg/TwistStamped": geometry_msgs.TwistStamped,
    # foxglove_msgs [Rust: only CompressedVideo]
    "foxglove_msgs/msg/CompressedVideo": foxglove_msgs.CompressedVideo,
    "foxglove_msgs/msg/CompressedImage": foxglove_msgs.CompressedImage,  # [Rust: -]
    "foxglove_msgs/msg/FrameTransform": foxglove_msgs.FrameTransform,  # [Rust: -]
    "foxglove_msgs/msg/LocationFix": foxglove_msgs.LocationFix,  # [Rust: -]
    "foxglove_msgs/msg/Log": foxglove_msgs.Log,  # [Rust: -]
    "foxglove_msgs/msg/PointCloud": foxglove_msgs.PointCloud,  # [Rust: -]
    "foxglove_msgs/msg/RawImage": foxglove_msgs.RawImage,  # [Rust: -]
    # edgefirst_msgs [Rust: ✓ all]
    "edgefirst_msgs/msg/Box": edgefirst_msgs.Box,
    "edgefirst_msgs/msg/Detect": edgefirst_msgs.Detect,
    "edgefirst_msgs/msg/DmaBuffer": edgefirst_msgs.DmaBuffer,
    "edgefirst_msgs/msg/Mask": edgefirst_msgs.Mask,
    "edgefirst_msgs/msg/ModelInfo": edgefirst_msgs.ModelInfo,
    "edgefirst_msgs/msg/RadarCube": edgefirst_msgs.RadarCube,
    "edgefirst_msgs/msg/RadarInfo": edgefirst_msgs.RadarInfo,
    "edgefirst_msgs/msg/Track": edgefirst_msgs.Track,
}


def get_mcap_summary(mcap_path: Path) -> dict[str, Any]:
    """Get summary information from an MCAP file."""
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()

        if not summary:
            return {"schemas": {}, "channels": {}, "statistics": None}

        return {
            "schemas": {s.name: s for s in summary.schemas.values()},
            "channels": {c.topic: c for c in summary.channels.values()},
            "statistics": summary.statistics,
        }


def iter_mcap_messages(mcap_path: Path):
    """Iterate over all messages in an MCAP file."""
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages():
            yield schema, channel, message


class TestMcapSchemaSupport:
    """Test that all schemas in MCAP files are supported."""

    def test_all_schemas_supported(self, mcap_file: Path):
        """Verify all schema types in the MCAP are implemented."""
        summary = get_mcap_summary(mcap_file)
        unsupported = []

        for schema_name in summary["schemas"]:
            if schema_name not in SCHEMA_MAP:
                unsupported.append(schema_name)

        assert not unsupported, (
            f"Unsupported schema types in {mcap_file.name}: {unsupported}\n"
            f"Add these to SCHEMA_MAP in test_mcap.py"
        )


class TestMcapDeserialization:
    """Test deserialization of all messages in MCAP files."""

    def test_deserialize_all_messages(self, mcap_file: Path):
        """Deserialize every message in the MCAP file."""
        errors = []
        message_counts = defaultdict(int)

        for schema, channel, message in iter_mcap_messages(mcap_file):
            schema_name = schema.name if schema else "unknown"
            message_counts[schema_name] += 1

            if schema_name not in SCHEMA_MAP:
                errors.append(
                    f"Unsupported schema: {schema_name} (topic: {channel.topic})"
                )
                continue

            cls = SCHEMA_MAP[schema_name]
            try:
                msg = cls.deserialize(message.data)
                assert msg is not None, (
                    f"Deserialization returned None for {schema_name}"
                )
            except Exception as e:
                errors.append(
                    f"Failed to deserialize {schema_name} (topic: {channel.topic}): {e}"
                )

        # Report stats
        total = sum(message_counts.values())
        print(f"\nDeserialized {total} messages from {mcap_file.name}:")
        for schema_name, count in sorted(message_counts.items()):
            print(f"  {schema_name}: {count}")

        assert not errors, (
            f"Deserialization errors in {mcap_file.name}:\n"
            + "\n".join(f"  - {e}" for e in errors[:10])
            + (f"\n  ... and {len(errors) - 10} more" if len(errors) > 10 else "")
        )


class TestMcapRoundTrip:
    """Test round-trip serialization of MCAP messages."""

    def test_roundtrip_all_messages(self, mcap_file: Path):
        """Verify messages survive serialize/deserialize round-trip.

        This test validates byte-perfect round-trip serialization:
        1. Deserialize original CDR bytes from MCAP
        2. Serialize back to CDR
        3. Compare serialized bytes to original (must be identical)
        4. Deserialize again and verify field equality
        """
        errors = []
        success_count = 0

        for schema, channel, message in iter_mcap_messages(mcap_file):
            schema_name = schema.name if schema else "unknown"

            if schema_name not in SCHEMA_MAP:
                continue  # Already tested in test_all_schemas_supported

            cls = SCHEMA_MAP[schema_name]
            try:
                # Deserialize original CDR bytes
                msg1 = cls.deserialize(message.data)

                # Serialize back to CDR
                cdr_bytes = msg1.serialize()

                # Byte-level comparison (like Rust test does)
                if cdr_bytes != message.data:
                    errors.append(
                        f"Byte mismatch for {schema_name} "
                        f"(topic: {channel.topic}): "
                        f"original {len(message.data)} bytes, "
                        f"reserialized {len(cdr_bytes)} bytes"
                    )
                    continue

                # Deserialize again and verify field equality
                msg2 = cls.deserialize(cdr_bytes)
                self._compare_messages(msg1, msg2, schema_name, channel.topic)
                success_count += 1

            except Exception as e:
                errors.append(
                    f"Round-trip failed for {schema_name} (topic: {channel.topic}): {e}"
                )

        print(f"\nRound-trip validated {success_count} messages")

        assert not errors, (
            f"Round-trip errors in {mcap_file.name}:\n"
            + "\n".join(f"  - {e}" for e in errors[:10])
            + (f"\n  ... and {len(errors) - 10} more" if len(errors) > 10 else "")
        )

    def _compare_messages(
        self,
        msg1: Any,
        msg2: Any,
        schema_name: str,
        topic: str,
    ):
        """Compare two deserialized messages for equality."""
        # Compare header if present
        if hasattr(msg1, "header") and msg1.header is not None:
            assert msg1.header.stamp.sec == msg2.header.stamp.sec, (
                f"Header stamp.sec mismatch: {msg1.header.stamp.sec} "
                f"!= {msg2.header.stamp.sec}"
            )
            assert msg1.header.stamp.nanosec == msg2.header.stamp.nanosec
            assert msg1.header.frame_id == msg2.header.frame_id

        # Compare dimensions for image-like messages
        if hasattr(msg1, "width") and hasattr(msg1, "height"):
            assert msg1.width == msg2.width
            assert msg1.height == msg2.height

        # Compare data lengths for messages with data arrays
        if hasattr(msg1, "data"):
            if isinstance(msg1.data, (bytes, list)):
                assert len(msg1.data) == len(msg2.data), (
                    f"Data length mismatch: {len(msg1.data)} != {len(msg2.data)}"
                )

        # Compare specific fields for known types
        if "RadarCube" in schema_name:
            assert list(msg1.shape) == list(msg2.shape)
            assert list(msg1.layout) == list(msg2.layout)
            assert msg1.is_complex == msg2.is_complex

        if "NavSatFix" in schema_name:
            assert msg1.latitude == msg2.latitude
            assert msg1.longitude == msg2.longitude
            assert msg1.altitude == msg2.altitude

        if "Imu" in schema_name:
            assert msg1.angular_velocity.x == msg2.angular_velocity.x
            assert msg1.linear_acceleration.x == msg2.linear_acceleration.x


class TestMcapFieldValidation:
    """Validate field values are reasonable."""

    def test_validate_timestamps(self, mcap_file: Path):
        """Verify timestamp fields have reasonable values."""
        errors = []

        for schema, channel, message in iter_mcap_messages(mcap_file):
            schema_name = schema.name if schema else "unknown"

            if schema_name not in SCHEMA_MAP:
                continue

            cls = SCHEMA_MAP[schema_name]
            try:
                msg = cls.deserialize(message.data)

                # Check header timestamp
                if hasattr(msg, "header") and msg.header is not None:
                    stamp = msg.header.stamp
                    # Timestamp should be reasonable (after year 2000)
                    if stamp.sec > 0 and stamp.sec < 946684800:
                        # Small sec values are relative timestamps, OK
                        pass
                    elif stamp.sec < 0:
                        errors.append(
                            f"Negative timestamp in {schema_name} "
                            f"(topic: {channel.topic}): sec={stamp.sec}"
                        )
                    # nanosec should be < 1 billion
                    if stamp.nanosec >= 1_000_000_000:
                        errors.append(
                            f"Invalid nanosec in {schema_name} "
                            f"(topic: {channel.topic}): "
                            f"nanosec={stamp.nanosec}"
                        )

            except Exception as e:
                errors.append(f"Validation error for {schema_name}: {e}")

        assert not errors, (
            f"Timestamp validation errors in {mcap_file.name}:\n"
            + "\n".join(f"  - {e}" for e in errors)
        )

    def test_validate_dimensions(self, mcap_file: Path):
        """Verify dimension fields are non-negative."""
        errors = []

        for schema, channel, message in iter_mcap_messages(mcap_file):
            schema_name = schema.name if schema else "unknown"

            if schema_name not in SCHEMA_MAP:
                continue

            cls = SCHEMA_MAP[schema_name]
            try:
                msg = cls.deserialize(message.data)

                # Check image dimensions
                if hasattr(msg, "width"):
                    if msg.width < 0:
                        errors.append(f"Negative width in {schema_name}: {msg.width}")
                if hasattr(msg, "height"):
                    if msg.height < 0:
                        errors.append(f"Negative height in {schema_name}: {msg.height}")

                # Check PointCloud2 dimensions
                if hasattr(msg, "point_step"):
                    if msg.point_step <= 0:
                        errors.append(
                            f"Invalid point_step in {schema_name}: {msg.point_step}"
                        )

            except Exception as e:
                errors.append(f"Validation error for {schema_name}: {e}")

        assert not errors, (
            f"Dimension validation errors in {mcap_file.name}:\n"
            + "\n".join(f"  - {e}" for e in errors)
        )
