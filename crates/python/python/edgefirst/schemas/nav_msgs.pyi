# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.nav_msgs``."""

from __future__ import annotations
from typing import Optional

from . import BufferLike
from .builtin_interfaces import Time
from .geometry_msgs import PoseWithCovariance, TwistWithCovariance
from .std_msgs import Header

__all__ = ["Odometry"]


class Odometry:
    """``nav_msgs.Odometry`` — robot pose + velocity with covariances and
    a `child_frame_id` linking the body frame to the odom frame.
    """

    def __init__(
        self,
        header: Header,
        child_frame_id: str = "",
        pose: Optional[PoseWithCovariance] = None,
        twist: Optional[TwistWithCovariance] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def child_frame_id(self) -> str: ...
    @property
    def pose(self) -> PoseWithCovariance: ...
    @property
    def twist(self) -> TwistWithCovariance: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Odometry: ...
    def __repr__(self) -> str: ...
