# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.nav_msgs``."""

from __future__ import annotations
from typing import List, Optional, Tuple

from . import BufferLike
from .builtin_interfaces import Time
from .geometry_msgs import Pose, Point, PoseWithCovariance, TwistWithCovariance
from .std_msgs import Header

__all__ = [
    "GridCells",
    "MapMetaData",
    "OccupancyGrid",
    "Odometry",
    "Path",
]


class MapMetaData:
    """``nav_msgs.MapMetaData`` — occupancy-grid map metadata.

    Stores the load time, resolution (metres/cell), grid dimensions, and
    origin pose of the map.
    """

    def __init__(
        self,
        map_load_time: Optional[Time] = None,
        resolution: float = 0.0,
        width: int = 0,
        height: int = 0,
        origin: Optional[Pose] = None,
    ) -> None: ...

    @property
    def map_load_time(self) -> Time: ...
    @property
    def resolution(self) -> float: ...
    @property
    def width(self) -> int: ...
    @property
    def height(self) -> int: ...
    @property
    def origin(self) -> Pose: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> MapMetaData: ...
    def __repr__(self) -> str: ...


class GridCells:
    """``nav_msgs.GridCells`` — list of cell centres in a costmap grid."""

    def __init__(
        self,
        header: Header,
        cell_width: float = 0.0,
        cell_height: float = 0.0,
        cells: Optional[List[Point]] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def cell_width(self) -> float: ...
    @property
    def cell_height(self) -> float: ...
    @property
    def cells(self) -> List[Point]: ...
    @property
    def cdr_size(self) -> int: ...

    def __len__(self) -> int: ...
    def __getitem__(self, idx: int) -> Point: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> GridCells: ...


class OccupancyGrid:
    """``nav_msgs.OccupancyGrid`` — 2-D occupancy grid map.

    Each cell value is an ``int8`` in range -1..100 (-1 = unknown,
    0..100 = probability occupied).  The raw bytes are accessible via
    ``data`` (zero-copy).
    """

    def __init__(
        self,
        header: Header,
        info: Optional[MapMetaData] = None,
        data: Optional[List[int]] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def info(self) -> MapMetaData: ...
    @property
    def data(self) -> bytes:
        """Zero-copy ``bytes`` view of the raw int8 occupancy data."""
        ...
    @property
    def data_len(self) -> int: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> OccupancyGrid: ...


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


class Path:
    """``nav_msgs.Path`` — an ordered sequence of PoseStamped waypoints.

    Construct from a ``Header`` and an optional list of ``Pose`` objects
    (all sharing the header's stamp/frame_id).  For round-trip or
    variable-stamp usage prefer ``from_cdr``.
    """

    def __init__(
        self,
        header: Header,
        poses: Optional[List[Pose]] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def cdr_size(self) -> int: ...

    def __len__(self) -> int: ...

    def poses(self) -> List[Tuple[Time, str, Pose]]:
        """Return all poses as a list of ``(stamp, frame_id, pose)`` tuples."""
        ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Path: ...
