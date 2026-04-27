# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.geometry_msgs``.

All types here are ``CdrFixed`` — small, ``Copy``-style structs with a
fixed wire size. Encode/decode via ``to_bytes()`` / ``from_cdr()``.
"""

from __future__ import annotations
from typing import Optional

from typing import List, Sequence

from . import BufferLike

__all__ = [
    "Accel",
    "Inertia",
    "Point",
    "Point32",
    "Pose",
    "Pose2D",
    "PoseWithCovariance",
    "Quaternion",
    "Transform",
    "Twist",
    "TwistWithCovariance",
    "Vector3",
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
