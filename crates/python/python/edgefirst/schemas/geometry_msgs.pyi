# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.geometry_msgs``.

Fixed-size types (``CdrFixed``) use ``to_bytes()`` / ``from_cdr()``.
Stamped types are buffer-backed with header + inner CdrFixed payload.
"""

from __future__ import annotations
from typing import Optional

from typing import List, Sequence

from . import BufferLike
from .builtin_interfaces import Time
from .std_msgs import Header

__all__ = [
    "Accel",
    "AccelStamped",
    "AccelWithCovariance",
    "AccelWithCovarianceStamped",
    "Inertia",
    "InertiaStamped",
    "Point",
    "Point32",
    "PointStamped",
    "Polygon",
    "PolygonStamped",
    "Pose",
    "Pose2D",
    "PoseArray",
    "PoseStamped",
    "PoseWithCovariance",
    "PoseWithCovarianceStamped",
    "Quaternion",
    "QuaternionStamped",
    "Transform",
    "TransformStamped",
    "Twist",
    "TwistStamped",
    "TwistWithCovariance",
    "TwistWithCovarianceStamped",
    "Vector3",
    "Vector3Stamped",
    "Wrench",
    "WrenchStamped",
]


class Vector3:
    """``geometry_msgs.Vector3`` — 3D vector (f64 ``x``, ``y``, ``z``)."""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None: ...

    @property
    def x(self) -> float: ...
    @property
    def y(self) -> float: ...
    @property
    def z(self) -> float: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Vector3: ...
    def __repr__(self) -> str: ...


class Point:
    """``geometry_msgs.Point`` — 3D point (f64 ``x``, ``y``, ``z``)."""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None: ...
    @property
    def x(self) -> float: ...
    @property
    def y(self) -> float: ...
    @property
    def z(self) -> float: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Point: ...
    def __repr__(self) -> str: ...


class Point32:
    """``geometry_msgs.Point32`` — 3D point (f32 ``x``, ``y``, ``z``)."""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None: ...
    @property
    def x(self) -> float: ...
    @property
    def y(self) -> float: ...
    @property
    def z(self) -> float: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Point32: ...
    def __repr__(self) -> str: ...


class Quaternion:
    """``geometry_msgs.Quaternion`` — orientation (f64 ``x``, ``y``, ``z``, ``w``).

    Defaults to the identity quaternion (``w = 1``).
    """

    def __init__(
        self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0
    ) -> None: ...

    @property
    def x(self) -> float: ...
    @property
    def y(self) -> float: ...
    @property
    def z(self) -> float: ...
    @property
    def w(self) -> float: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Quaternion: ...
    def __repr__(self) -> str: ...


class Pose:
    """``geometry_msgs.Pose`` — :class:`Point` position + :class:`Quaternion`
    orientation. CdrFixed (56-byte payload).
    """

    def __init__(
        self,
        position: Optional[Point] = None,
        orientation: Optional[Quaternion] = None,
    ) -> None: ...

    @property
    def position(self) -> Point: ...
    @property
    def orientation(self) -> Quaternion: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Pose: ...
    def __repr__(self) -> str: ...


class Pose2D:
    """``geometry_msgs.Pose2D`` — 2D pose (f64 ``x``, ``y``, ``theta``)."""

    def __init__(
        self, x: float = 0.0, y: float = 0.0, theta: float = 0.0
    ) -> None: ...

    @property
    def x(self) -> float: ...
    @property
    def y(self) -> float: ...
    @property
    def theta(self) -> float: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Pose2D: ...
    def __repr__(self) -> str: ...


class Transform:
    """``geometry_msgs.Transform`` — :class:`Vector3` translation +
    :class:`Quaternion` rotation. CdrFixed (56-byte payload).
    """

    def __init__(
        self,
        translation: Optional[Vector3] = None,
        rotation: Optional[Quaternion] = None,
    ) -> None: ...

    @property
    def translation(self) -> Vector3: ...
    @property
    def rotation(self) -> Quaternion: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Transform: ...


class Twist:
    """``geometry_msgs.Twist`` — linear + angular :class:`Vector3` velocity.
    CdrFixed (48-byte payload).
    """

    def __init__(
        self,
        linear: Optional[Vector3] = None,
        angular: Optional[Vector3] = None,
    ) -> None: ...

    @property
    def linear(self) -> Vector3: ...
    @property
    def angular(self) -> Vector3: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Twist: ...


class Accel:
    """``geometry_msgs.Accel`` — linear + angular :class:`Vector3`
    acceleration. CdrFixed (48-byte payload).
    """

    def __init__(
        self,
        linear: Optional[Vector3] = None,
        angular: Optional[Vector3] = None,
    ) -> None: ...

    @property
    def linear(self) -> Vector3: ...
    @property
    def angular(self) -> Vector3: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Accel: ...


class Inertia:
    """``geometry_msgs.Inertia`` — rigid-body mass + inertia tensor.
    CdrFixed.
    """

    def __init__(
        self,
        m: float = 0.0,
        com: Optional[Vector3] = None,
        ixx: float = 0.0, ixy: float = 0.0, ixz: float = 0.0,
        iyy: float = 0.0, iyz: float = 0.0, izz: float = 0.0,
    ) -> None: ...

    @property
    def m(self) -> float: ...
    @property
    def com(self) -> Vector3: ...
    @property
    def ixx(self) -> float: ...
    @property
    def ixy(self) -> float: ...
    @property
    def ixz(self) -> float: ...
    @property
    def iyy(self) -> float: ...
    @property
    def iyz(self) -> float: ...
    @property
    def izz(self) -> float: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Inertia: ...


class PoseWithCovariance:
    """``geometry_msgs.PoseWithCovariance`` — :class:`Pose` + 6×6 row-major
    covariance of (x, y, z, rotX, rotY, rotZ). CdrFixed (344-byte payload).

    The 36-element covariance is exposed as ``list[float]``; pass any
    36-element sequence (list/tuple/numpy 1D float array) at construction.
    """

    def __init__(
        self,
        pose: Optional[Pose] = None,
        covariance: Optional[Sequence[float]] = None,
    ) -> None: ...

    @property
    def pose(self) -> Pose: ...
    @property
    def covariance(self) -> List[float]: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> PoseWithCovariance: ...


class TwistWithCovariance:
    """``geometry_msgs.TwistWithCovariance`` — :class:`Twist` + 6×6
    row-major covariance. CdrFixed.
    """

    def __init__(
        self,
        twist: Optional[Twist] = None,
        covariance: Optional[Sequence[float]] = None,
    ) -> None: ...

    @property
    def twist(self) -> Twist: ...
    @property
    def covariance(self) -> List[float]: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> TwistWithCovariance: ...


class TwistStamped:
    """``geometry_msgs.TwistStamped`` — header + :class:`Twist`.

    Buffer-backed; ``from_cdr`` deserializes via offset table.
    """

    def __init__(
        self,
        header: Header,
        twist: Optional[Twist] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def twist(self) -> Twist: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> TwistStamped: ...


class AccelStamped:
    """``geometry_msgs.AccelStamped`` — header + :class:`Accel`.

    Buffer-backed; ``from_cdr`` deserializes via offset table.
    """

    def __init__(
        self,
        header: Header,
        accel: Optional[Accel] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def accel(self) -> Accel: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> AccelStamped: ...


class InertiaStamped:
    """``geometry_msgs.InertiaStamped`` — header + :class:`Inertia`.

    Buffer-backed; ``from_cdr`` deserializes via offset table.
    """

    def __init__(
        self,
        header: Header,
        inertia: Optional[Inertia] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def inertia(self) -> Inertia: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> InertiaStamped: ...


class PointStamped:
    """``geometry_msgs.PointStamped`` — header + :class:`Point`.

    Buffer-backed; ``from_cdr`` deserializes via offset table.
    """

    def __init__(
        self,
        header: Header,
        point: Optional[Point] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def point(self) -> Point: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> PointStamped: ...


class TransformStamped:
    """``geometry_msgs.TransformStamped`` — header + child_frame_id + :class:`Transform`.

    Buffer-backed; ``from_cdr`` deserializes via offset table.
    """

    def __init__(
        self,
        header: Header,
        child_frame_id: str = "",
        transform: Optional[Transform] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def child_frame_id(self) -> str: ...
    @property
    def transform(self) -> Transform: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> TransformStamped: ...


class Wrench:
    """``geometry_msgs.Wrench`` — force + torque as :class:`Vector3` pair.

    CdrFixed (48 bytes). Use ``to_bytes()`` / ``from_cdr()``.
    """

    def __init__(
        self,
        force: Optional[Vector3] = None,
        torque: Optional[Vector3] = None,
    ) -> None: ...

    @property
    def force(self) -> Vector3: ...
    @property
    def torque(self) -> Vector3: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Wrench: ...


class AccelWithCovariance:
    """``geometry_msgs.AccelWithCovariance`` — :class:`Accel` + 6×6 covariance.

    CdrFixed (336 bytes). Use ``to_bytes()`` / ``from_cdr()``.
    """

    def __init__(
        self,
        accel: Optional[Accel] = None,
        covariance: Optional[Sequence[float]] = None,
    ) -> None: ...

    @property
    def accel(self) -> Accel: ...
    @property
    def covariance(self) -> List[float]: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> AccelWithCovariance: ...


class Vector3Stamped:
    """``geometry_msgs.Vector3Stamped`` — header + :class:`Vector3`.

    Buffer-backed; ``from_cdr`` deserializes via offset table.
    """

    def __init__(
        self,
        header: Header,
        vector: Optional[Vector3] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def vector(self) -> Vector3: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Vector3Stamped: ...


class PoseStamped:
    """``geometry_msgs.PoseStamped`` — header + :class:`Pose`.

    Buffer-backed; ``from_cdr`` deserializes via offset table.
    """

    def __init__(
        self,
        header: Header,
        pose: Optional[Pose] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def pose(self) -> Pose: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> PoseStamped: ...


class QuaternionStamped:
    """``geometry_msgs.QuaternionStamped`` — header + :class:`Quaternion`.

    Buffer-backed; ``from_cdr`` deserializes via offset table.
    """

    def __init__(
        self,
        header: Header,
        quaternion: Optional[Quaternion] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def quaternion(self) -> Quaternion: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> QuaternionStamped: ...


class WrenchStamped:
    """``geometry_msgs.WrenchStamped`` — header + :class:`Wrench`.

    Buffer-backed; ``from_cdr`` deserializes via offset table.
    """

    def __init__(
        self,
        header: Header,
        wrench: Optional[Wrench] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def wrench(self) -> Wrench: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> WrenchStamped: ...


class PoseWithCovarianceStamped:
    """``geometry_msgs.PoseWithCovarianceStamped`` — header + :class:`PoseWithCovariance`.

    Buffer-backed; ``from_cdr`` deserializes via offset table.
    """

    def __init__(
        self,
        header: Header,
        pose_with_covariance: Optional[PoseWithCovariance] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def pose_with_covariance(self) -> PoseWithCovariance: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> PoseWithCovarianceStamped: ...


class TwistWithCovarianceStamped:
    """``geometry_msgs.TwistWithCovarianceStamped`` — header + :class:`TwistWithCovariance`.

    Buffer-backed; ``from_cdr`` deserializes via offset table.
    """

    def __init__(
        self,
        header: Header,
        twist_with_covariance: Optional[TwistWithCovariance] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def twist_with_covariance(self) -> TwistWithCovariance: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> TwistWithCovarianceStamped: ...


class AccelWithCovarianceStamped:
    """``geometry_msgs.AccelWithCovarianceStamped`` — header + :class:`AccelWithCovariance`.

    Buffer-backed; ``from_cdr`` deserializes via offset table.
    """

    def __init__(
        self,
        header: Header,
        accel_with_covariance: Optional[AccelWithCovariance] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def accel_with_covariance(self) -> AccelWithCovariance: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> AccelWithCovarianceStamped: ...


class Polygon:
    """``geometry_msgs.Polygon`` — variable-length sequence of :class:`Point32`.

    Buffer-backed; supports ``len()`` and indexing.
    """

    def __init__(
        self,
        points: Optional[Sequence[Point32]] = None,
    ) -> None: ...

    @property
    def cdr_size(self) -> int: ...
    @property
    def points(self) -> List[Point32]: ...

    def __len__(self) -> int: ...
    def __getitem__(self, idx: int) -> Point32: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Polygon: ...


class PolygonStamped:
    """``geometry_msgs.PolygonStamped`` — header + :class:`Polygon`.

    Buffer-backed; supports ``len()`` and indexing for points.
    """

    def __init__(
        self,
        header: Header,
        points: Optional[Sequence[Point32]] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def cdr_size(self) -> int: ...
    @property
    def points(self) -> List[Point32]: ...

    def __len__(self) -> int: ...
    def __getitem__(self, idx: int) -> Point32: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> PolygonStamped: ...


class PoseArray:
    """``geometry_msgs.PoseArray`` — header + variable-length sequence of :class:`Pose`.

    Buffer-backed; supports ``len()`` and indexing.
    """

    def __init__(
        self,
        header: Header,
        poses: Optional[Sequence[Pose]] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def cdr_size(self) -> int: ...
    @property
    def poses(self) -> List[Pose]: ...

    def __len__(self) -> int: ...
    def __getitem__(self, idx: int) -> Pose: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> PoseArray: ...
