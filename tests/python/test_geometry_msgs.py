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
    AccelStamped,
    AccelWithCovariance,
    AccelWithCovarianceStamped,
    Inertia,
    InertiaStamped,
    Point,
    Point32,
    PointStamped,
    Polygon,
    PolygonStamped,
    Pose,
    Pose2D,
    PoseArray,
    PoseStamped,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
    QuaternionStamped,
    Transform,
    TransformStamped,
    Twist,
    TwistStamped,
    TwistWithCovariance,
    TwistWithCovarianceStamped,
    Vector3,
    Vector3Stamped,
    Wrench,
    WrenchStamped,
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


# ── Stamped types (buffer-backed) ──────────────────────────────────

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.std_msgs import Header


class TestTwistStamped:
    def test_round_trip(self, sample_header):
        tw = Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.5))
        ts = TwistStamped(header=sample_header, twist=tw)
        restored = TwistStamped.from_cdr(ts.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.frame_id == "sensor_frame"
        assert restored.twist.linear.x == 1.0
        assert restored.twist.angular.z == 0.5

    def test_defaults(self, sample_header):
        ts = TwistStamped(header=sample_header)
        assert ts.twist.linear.x == 0.0


class TestAccelStamped:
    def test_round_trip(self, sample_header):
        ac = Accel(linear=Vector3(9.81, 0.0, 0.0), angular=Vector3(0.0, 0.0, 1.0))
        a = AccelStamped(header=sample_header, accel=ac)
        restored = AccelStamped.from_cdr(a.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.frame_id == "sensor_frame"
        assert restored.accel.linear.x == 9.81
        assert restored.accel.angular.z == 1.0


class TestPointStamped:
    def test_round_trip(self, sample_header):
        pt = Point(x=1.0, y=2.0, z=3.0)
        ps = PointStamped(header=sample_header, point=pt)
        restored = PointStamped.from_cdr(ps.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.frame_id == "sensor_frame"
        assert (restored.point.x, restored.point.y, restored.point.z) == (1.0, 2.0, 3.0)


class TestInertiaStamped:
    def test_round_trip(self, sample_header):
        inertia = Inertia(m=10.0, com=Vector3(0.1, 0.2, 0.3), ixx=1.0, iyy=2.0, izz=3.0)
        i = InertiaStamped(header=sample_header, inertia=inertia)
        restored = InertiaStamped.from_cdr(i.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.inertia.m == 10.0
        assert restored.inertia.izz == 3.0


class TestTransformStamped:
    def test_round_trip(self, sample_header):
        tf = Transform(translation=Vector3(1.0, 2.0, 3.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0))
        ts = TransformStamped(header=sample_header, child_frame_id="child", transform=tf)
        restored = TransformStamped.from_cdr(ts.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.child_frame_id == "child"
        assert restored.transform.translation.x == 1.0

    def test_empty_child_frame_id(self, sample_header):
        ts = TransformStamped(header=sample_header)
        assert ts.child_frame_id == ""


class TestWrench:
    def test_defaults(self):
        w = Wrench()
        assert w.force.x == 0.0 and w.torque.x == 0.0

    def test_round_trip(self):
        w = Wrench(force=Vector3(1.5, -2.5, 3.0), torque=Vector3(0.1, 0.2, 0.3))
        restored = Wrench.from_cdr(w.to_bytes())
        assert (restored.force.x, restored.force.y, restored.force.z) == (1.5, -2.5, 3.0)
        assert (restored.torque.x, restored.torque.y, restored.torque.z) == (0.1, 0.2, 0.3)


class TestAccelWithCovariance:
    def test_defaults(self):
        a = AccelWithCovariance()
        assert a.accel.linear.x == 0.0
        assert len(a.covariance) == 36
        assert all(c == 0.0 for c in a.covariance)

    def test_round_trip(self):
        cov = [float(i) for i in range(36)]
        ac = Accel(linear=Vector3(1.0, 2.0, 3.0), angular=Vector3(4.0, 5.0, 6.0))
        a = AccelWithCovariance(accel=ac, covariance=cov)
        restored = AccelWithCovariance.from_cdr(a.to_bytes())
        assert restored.accel.linear.x == 1.0
        assert restored.covariance == cov


class TestVector3Stamped:
    def test_round_trip(self, sample_header):
        v = Vector3(1.5, -2.5, 3.0)
        vs = Vector3Stamped(header=sample_header, vector=v)
        restored = Vector3Stamped.from_cdr(vs.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.frame_id == "sensor_frame"
        assert (restored.vector.x, restored.vector.y, restored.vector.z) == (1.5, -2.5, 3.0)

    def test_defaults(self, sample_header):
        vs = Vector3Stamped(header=sample_header)
        assert vs.vector.x == 0.0


class TestPoseStamped:
    def test_round_trip(self, sample_header):
        p = Pose(position=Point(1.0, 2.0, 3.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))
        ps = PoseStamped(header=sample_header, pose=p)
        restored = PoseStamped.from_cdr(ps.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.frame_id == "sensor_frame"
        assert restored.pose.position.x == 1.0
        assert restored.pose.orientation.w == 1.0


class TestQuaternionStamped:
    def test_round_trip(self, sample_header):
        q = Quaternion(0.0, 0.0, 0.707, 0.707)
        qs = QuaternionStamped(header=sample_header, quaternion=q)
        restored = QuaternionStamped.from_cdr(qs.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.frame_id == "sensor_frame"
        assert abs(restored.quaternion.z - 0.707) < 1e-10
        assert abs(restored.quaternion.w - 0.707) < 1e-10


class TestWrenchStamped:
    def test_round_trip(self, sample_header):
        w = Wrench(force=Vector3(1.0, 2.0, 3.0), torque=Vector3(0.1, 0.2, 0.3))
        ws = WrenchStamped(header=sample_header, wrench=w)
        restored = WrenchStamped.from_cdr(ws.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.frame_id == "sensor_frame"
        assert restored.wrench.force.x == 1.0
        assert restored.wrench.torque.z == 0.3


class TestPoseWithCovarianceStamped:
    def test_round_trip(self, sample_header):
        cov = [0.0] * 36
        cov[0] = 1.0
        pwc = PoseWithCovariance(
            pose=Pose(position=Point(1.0, 2.0, 3.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
            covariance=cov,
        )
        msg = PoseWithCovarianceStamped(header=sample_header, pose_with_covariance=pwc)
        restored = PoseWithCovarianceStamped.from_cdr(msg.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.frame_id == "sensor_frame"
        assert restored.pose_with_covariance.pose.position.x == 1.0
        assert restored.pose_with_covariance.covariance[0] == 1.0


class TestTwistWithCovarianceStamped:
    def test_round_trip(self, sample_header):
        cov = [0.0] * 36
        cov[0] = 2.0
        twc = TwistWithCovariance(
            twist=Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.5)),
            covariance=cov,
        )
        msg = TwistWithCovarianceStamped(header=sample_header, twist_with_covariance=twc)
        restored = TwistWithCovarianceStamped.from_cdr(msg.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.twist_with_covariance.twist.linear.x == 1.0
        assert restored.twist_with_covariance.covariance[0] == 2.0


class TestAccelWithCovarianceStamped:
    def test_round_trip(self, sample_header):
        cov = [0.0] * 36
        cov[0] = 3.0
        awc = AccelWithCovariance(
            accel=Accel(linear=Vector3(9.81, 0.0, 0.0), angular=Vector3(0.0, 0.0, 1.0)),
            covariance=cov,
        )
        msg = AccelWithCovarianceStamped(header=sample_header, accel_with_covariance=awc)
        restored = AccelWithCovarianceStamped.from_cdr(msg.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.accel_with_covariance.accel.linear.x == 9.81
        assert restored.accel_with_covariance.covariance[0] == 3.0


class TestPolygon:
    def test_round_trip(self):
        pts = [Point32(1.0, 2.0, 0.0), Point32(3.0, 4.0, 0.0), Point32(5.0, 6.0, 0.0)]
        poly = Polygon(points=pts)
        restored = Polygon.from_cdr(poly.to_bytes())
        assert len(restored) == 3
        assert restored[0].x == 1.0
        assert restored[2].y == 6.0

    def test_empty(self):
        poly = Polygon()
        assert len(poly) == 0

    def test_negative_index(self):
        pts = [Point32(1.0, 2.0, 0.0), Point32(3.0, 4.0, 0.0)]
        poly = Polygon(points=pts)
        assert poly[-1].x == 3.0

    def test_index_out_of_range(self):
        poly = Polygon(points=[Point32(1.0, 2.0, 0.0)])
        with pytest.raises(IndexError):
            _ = poly[5]

    def test_points_property(self):
        pts = [Point32(1.0, 2.0, 0.0), Point32(3.0, 4.0, 0.0)]
        poly = Polygon(points=pts)
        all_pts = poly.points
        assert len(all_pts) == 2
        assert all_pts[0].x == 1.0


class TestPolygonStamped:
    def test_round_trip(self, sample_header):
        pts = [Point32(1.0, 2.0, 0.0), Point32(3.0, 4.0, 0.0)]
        ps = PolygonStamped(header=sample_header, points=pts)
        restored = PolygonStamped.from_cdr(ps.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.frame_id == "sensor_frame"
        assert len(restored) == 2
        assert restored[0].x == 1.0

    def test_empty_polygon(self, sample_header):
        ps = PolygonStamped(header=sample_header)
        assert len(ps) == 0


class TestPoseArray:
    def test_round_trip(self, sample_header):
        poses = [
            Pose(position=Point(1.0, 2.0, 3.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
            Pose(position=Point(4.0, 5.0, 6.0), orientation=Quaternion(0.0, 0.0, 1.0, 0.0)),
        ]
        pa = PoseArray(header=sample_header, poses=poses)
        restored = PoseArray.from_cdr(pa.to_bytes())
        assert restored.stamp.sec == sample_header.stamp.sec
        assert restored.frame_id == "sensor_frame"
        assert len(restored) == 2
        assert restored[0].position.x == 1.0
        assert restored[1].position.x == 4.0
        assert restored[1].orientation.z == 1.0

    def test_empty(self, sample_header):
        pa = PoseArray(header=sample_header)
        assert len(pa) == 0

    def test_poses_property(self, sample_header):
        poses = [
            Pose(position=Point(1.0, 2.0, 3.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0)),
        ]
        pa = PoseArray(header=sample_header, poses=poses)
        all_poses = pa.poses
        assert len(all_poses) == 1
        assert all_poses[0].position.x == 1.0
