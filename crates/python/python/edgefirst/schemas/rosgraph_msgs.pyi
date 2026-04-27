# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.rosgraph_msgs``."""

from __future__ import annotations
from typing import Optional

from . import BufferLike
from .builtin_interfaces import Time

__all__ = ["Clock"]


class Clock:
    """``rosgraph_msgs.Clock`` — single :class:`Time` value for the
    ROS 2 ``/clock`` topic. CdrFixed (8-byte payload).
    """

    def __init__(self, clock: Optional[Time] = None) -> None: ...

    @property
    def clock(self) -> Time: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Clock: ...
    def __repr__(self) -> str: ...
