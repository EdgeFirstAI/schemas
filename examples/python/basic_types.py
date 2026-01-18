#!/usr/bin/env python3
"""
Basic Types Example

SPDX-License-Identifier: Apache-2.0
Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

Demonstrates fundamental ROS2 message types including:
- builtin_interfaces (Time, Duration)
- std_msgs (Header, ColorRGBA)
- geometry_msgs (Vector3, Point, Quaternion, Pose)
"""

from edgefirst.schemas.builtin_interfaces import Duration, Time
from edgefirst.schemas.geometry_msgs import Point, Pose, Quaternion, Vector3
from edgefirst.schemas.std_msgs import ColorRGBA, Header


def example_time() -> None:
    """Demonstrate Time message."""
    print("=== Example: Time ===")

    # Create a timestamp
    time = Time(sec=1234567890, nanosec=123456789)

    print(f"Time: {time.sec}.{time.nanosec:09d} seconds")
    print(f"Time (repr): {time!r}\n")


def example_duration() -> None:
    """Demonstrate Duration message."""
    print("=== Example: Duration ===")

    # Create a duration (5.5 seconds)
    duration = Duration(sec=5, nanosec=500_000_000)

    total_ns = duration.sec * 1_000_000_000 + duration.nanosec
    print(f"Duration: {total_ns / 1e9} seconds ({total_ns} nanoseconds)\n")


def example_header() -> None:
    """Demonstrate Header message."""
    print("=== Example: Header ===")

    # Create a header with timestamp and frame ID
    header = Header(
        stamp=Time(sec=1234567890, nanosec=123456789),
        frame_id="camera_optical_frame"
    )

    print("Header:")
    print(f"  timestamp: {header.stamp.sec}.{header.stamp.nanosec:09d}")
    print(f"  frame_id: {header.frame_id}\n")

    # Default initialization
    default_header = Header()
    print(f"Default header: {default_header!r}\n")


def example_color() -> None:
    """Demonstrate ColorRGBA message."""
    print("=== Example: ColorRGBA ===")

    # Create colors
    red = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
    transparent_blue = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5)

    print(f"Red: {red!r}")
    print(f"Transparent blue: {transparent_blue!r}\n")


def example_vector3() -> None:
    """Demonstrate Vector3 message."""
    print("=== Example: Vector3 ===")

    velocity = Vector3(x=1.5, y=2.0, z=0.5)

    # Calculate magnitude
    magnitude = (velocity.x**2 + velocity.y**2 + velocity.z**2) ** 0.5

    print(f"Velocity vector: ({velocity.x}, {velocity.y}, {velocity.z})")
    print(f"Magnitude: {magnitude:.3f}\n")


def example_point() -> None:
    """Demonstrate Point message."""
    print("=== Example: Point ===")

    origin = Point(x=0.0, y=0.0, z=0.0)
    target = Point(x=10.0, y=5.0, z=2.0)

    # Calculate distance
    dx = target.x - origin.x
    dy = target.y - origin.y
    dz = target.z - origin.z
    distance = (dx * dx + dy * dy + dz * dz) ** 0.5

    print(f"Origin: {origin!r}")
    print(f"Target: {target!r}")
    print(f"Distance: {distance:.3f}\n")


def example_quaternion() -> None:
    """Demonstrate Quaternion message."""
    print("=== Example: Quaternion ===")

    # Identity quaternion (no rotation)
    identity = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    # 90 degree rotation around Z axis
    rotation_z_90 = Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)

    print(f"Identity rotation: {identity!r}")
    print(f"90° Z rotation: {rotation_z_90!r}\n")


def example_pose() -> None:
    """Demonstrate Pose message."""
    print("=== Example: Pose ===")

    pose = Pose(
        position=Point(x=1.0, y=2.0, z=0.5),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )

    print("Pose:")
    print(f"  position: ({pose.position.x}, {pose.position.y}, {pose.position.z})")
    print(f"  orientation: ({pose.orientation.x}, {pose.orientation.y}, "
          f"{pose.orientation.z}, {pose.orientation.w})\n")


def example_serialization() -> None:
    """Demonstrate CDR serialization."""
    print("=== Example: Serialization ===")

    header = Header(
        stamp=Time(sec=1234567890, nanosec=123456789),
        frame_id="test_frame"
    )

    # Serialize to CDR
    bytes_data = header.serialize()
    print(f"Serialized header to {len(bytes_data)} bytes")

    # Deserialize from CDR
    decoded = Header.deserialize(bytes_data)

    # Verify round-trip
    assert decoded.stamp.sec == header.stamp.sec
    assert decoded.stamp.nanosec == header.stamp.nanosec
    assert decoded.frame_id == header.frame_id

    print("Round-trip successful!")
    print(f"  Original:     {header!r}")
    print(f"  Deserialized: {decoded!r}\n")


def main() -> None:
    """Run all examples."""
    print("EdgeFirst Schemas - Basic Types Examples")
    print("=" * 50 + "\n")

    example_time()
    example_duration()
    example_header()
    example_color()
    example_vector3()
    example_point()
    example_quaternion()
    example_pose()
    example_serialization()

    print("=" * 50)
    print("All examples completed successfully!")


if __name__ == "__main__":
    main()
