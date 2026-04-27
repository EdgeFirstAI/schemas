# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for `edgefirst.schemas.geometry_msgs`.

All eight types are CdrFixed. The composites (Pose, Transform, Twist)
embed their components by value — no offset tables, no buffer-backed
storage. Round-trip tests confirm wire compatibility with the Rust
implementation.
"""

import pytest

from edgefirst.schemas.geometry_msgs import (
    Accel,
    Inertia,
    Point,
    Point32,
    Pose,
    Pose2D,
    PoseWithCovariance,
    Quaternion,
    Transform,
    Twist,
    TwistWithCovariance,
    Vector3,
)


class TestVector3:
    def test_defaults(self):
        v = Vector3()
        assert v.x == 0.0 and v.y == 0.0 and v.z == 0.0

    def test_round_trip(self):
        v = Vector3(x=1.5, y=2.5, z=3.5)
        restored = Vector3.from_cdr(v.to_bytes())
        assert (restored.x, restored.y, restored.z) == (1.5, 2.5, 3.5)


class TestPoint:
    def test_defaults(self):
        p = Point()
        assert (p.x, p.y, p.z) == (0.0, 0.0, 0.0)

    def test_round_trip(self):
        p = Point(x=10.0, y=20.0, z=30.0)
        restored = Point.from_cdr(p.to_bytes())
        assert (restored.x, restored.y, restored.z) == (10.0, 20.0, 30.0)


class TestPoint32:
    def test_defaults(self):
        p = Point32()
        assert (p.x, p.y, p.z) == (0.0, 0.0, 0.0)

    def test_round_trip(self):
        # Use dyadic fractions so the f32 round-trip is exact.
        p = Point32(x=1.5, y=2.25, z=-3.125)
        restored = Point32.from_cdr(p.to_bytes())
        assert (restored.x, restored.y, restored.z) == (1.5, 2.25, -3.125)


class TestQuaternion:
    def test_default_is_identity(self):
        # Identity quaternion (w=1) is the natural rotation default.
        q = Quaternion()
        assert (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 1.0)

    def test_round_trip(self):
        q = Quaternion(x=0.0, y=0.0, z=0.7071067811865476, w=0.7071067811865476)
        restored = Quaternion.from_cdr(q.to_bytes())
        assert restored.z == q.z
        assert restored.w == q.w


class TestPose2D:
    def test_round_trip(self):
        p = Pose2D(x=10.0, y=20.0, theta=1.57)
        restored = Pose2D.from_cdr(p.to_bytes())
        assert (restored.x, restored.y, restored.theta) == (10.0, 20.0, 1.57)


class TestPose:
    def test_default_components(self):
        # Default pose: zero position + identity orientation.
        p = Pose()
        assert (p.position.x, p.position.y, p.position.z) == (0.0, 0.0, 0.0)
        assert (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w) == (
            0.0, 0.0, 0.0, 1.0,
        )

    def test_round_trip(self):
        p = Pose(
            position=Point(10.0, 20.0, 30.0),
            orientation=Quaternion(0.0, 0.0, 0.7071, 0.7071),
        )
        data = p.to_bytes()
        # CDR header (4) + Point (24) + Quaternion (32) = 60 bytes.
        assert len(data) == 60
        restored = Pose.from_cdr(data)
        assert restored.position.x == 10.0
        assert restored.orientation.w == 0.7071


class TestTransform:
    def test_round_trip(self):
        t = Transform(
            translation=Vector3(1.5, 2.5, 3.5),
            rotation=Quaternion(0.0, 0.0, 0.7071, 0.7071),
        )
        restored = Transform.from_cdr(t.to_bytes())
        assert restored.translation.y == 2.5
        assert restored.rotation.z == 0.7071


class TestTwist:
    def test_round_trip(self):
        t = Twist(linear=Vector3(1.0, 2.0, 3.0), angular=Vector3(0.0, 0.0, 0.5))
        restored = Twist.from_cdr(t.to_bytes())
        assert (restored.linear.x, restored.linear.y, restored.linear.z) == (1.0, 2.0, 3.0)
        assert restored.angular.z == 0.5


@pytest.mark.parametrize(
    "cls,kwargs",
    [
        (Vector3, dict(x=1.5, y=2.5, z=3.5)),
        (Point, dict(x=10.0, y=20.0, z=30.0)),
        (Point32, dict(x=1.5, y=2.5, z=3.5)),
        (Quaternion, dict(x=0.0, y=0.0, z=0.707, w=0.707)),
        (Pose2D, dict(x=10.0, y=20.0, theta=1.57)),
    ],
)
def test_simple_round_trip(cls, kwargs):
    v = cls(**kwargs)
    restored = cls.from_cdr(v.to_bytes())
    for k, want in kwargs.items():
        assert getattr(restored, k) == want, f"{cls.__name__}.{k}"


# ── Composite CdrFixed types ───────────────────────────────────────


class TestAccel:
    def test_round_trip(self):
        a = Accel(linear=Vector3(1.0, 2.0, 3.0), angular=Vector3(0.1, 0.2, 0.3))
        restored = Accel.from_cdr(a.to_bytes())
        assert restored.linear.x == 1.0
        assert restored.angular.z == 0.3


class TestInertia:
    def test_round_trip(self):
        i = Inertia(
            m=10.0,
            com=Vector3(0.1, 0.2, 0.3),
            ixx=1.0, ixy=0.0, ixz=0.0,
            iyy=2.0, iyz=0.0, izz=3.0,
        )
        restored = Inertia.from_cdr(i.to_bytes())
        assert restored.m == 10.0
        assert restored.com.x == 0.1
        assert restored.ixx == 1.0
        assert restored.iyy == 2.0
        assert restored.izz == 3.0


class TestPoseWithCovariance:
    def test_default_covariance(self):
        p = PoseWithCovariance()
        assert p.covariance == [0.0] * 36

    def test_round_trip_with_covariance(self):
        cov = [float(i) * 0.1 for i in range(36)]
        p = PoseWithCovariance(
            pose=Pose(position=Point(1.0, 2.0, 3.0), orientation=Quaternion(0, 0, 0, 1)),
            covariance=cov,
        )
        restored = PoseWithCovariance.from_cdr(p.to_bytes())
        assert restored.pose.position.x == 1.0
        assert restored.covariance == cov

    def test_invalid_covariance_length_raises(self):
        with pytest.raises(ValueError, match="36-element"):
            PoseWithCovariance(covariance=[0.0, 1.0, 2.0])


class TestTwistWithCovariance:
    def test_round_trip(self):
        cov = list(range(36))
        t = TwistWithCovariance(
            twist=Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, 0.5)),
            covariance=[float(c) for c in cov],
        )
        restored = TwistWithCovariance.from_cdr(t.to_bytes())
        assert restored.twist.angular.z == 0.5
        assert restored.covariance == [float(c) for c in cov]
