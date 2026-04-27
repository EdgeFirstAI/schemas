# SPDX-License-Identifier: Apache-2.0
# Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

from dataclasses import dataclass

from pycdr2 import IdlStruct
from pycdr2.types import int32, uint32


@dataclass
class Duration(IdlStruct, typename='builtin_interfaces/Duration'):
    """
    Duration defines a period between two time points.
    Messages of this datatype are of ROS Time following this design:
    https://design.ros2.org/articles/clock_and_time.html
    """
    sec: int32 = 0
    """
    The seconds component, valid over all int32 values.
    """
    nanosec: uint32 = 0
    """
    The nanoseconds component, valid in the range [0, 10e9).
    """


@dataclass
class Time(IdlStruct, typename='builtin_interfaces/Time'):
    sec: int32 = 0
    """
    The seconds component, valid over all int32 values.
    """
    nanosec: uint32 = 0
    """
    The nanoseconds component, valid in the range [0, 10e9).
    """


# Schema registry support
_TYPES = {
    "Duration": Duration,
    "Time": Time,
}


def is_type_supported(type_name: str) -> bool:
    """Check if a type name is supported by this module."""
    return type_name in _TYPES


def list_types() -> list[str]:
    """List all type schema names in this module."""
    return [
        "builtin_interfaces/msg/Duration",
        "builtin_interfaces/msg/Time",
    ]


def get_type(type_name: str) -> type:
    """Get the type class by name. Returns None if not found."""
    return _TYPES.get(type_name)
