# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for `edgefirst.schemas.nav_msgs`."""

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.geometry_msgs import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from edgefirst.schemas.nav_msgs import Odometry
from edgefirst.schemas.std_msgs import Header


class TestOdometry:
    def test_round_trip(self):
        h = Header(stamp=Time(1, 2), frame_id="odom")
        pose = PoseWithCovariance(
            pose=Pose(position=Point(1.0, 2.0, 3.0), orientation=Quaternion(0, 0, 0, 1)),
            covariance=[0.01] * 36,
        )
        twist = TwistWithCovariance(
            twist=Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0, 0, 0.1)),
            covariance=[0.001] * 36,
        )
        odo = Odometry(header=h, child_frame_id="base_link", pose=pose, twist=twist)

        restored = Odometry.from_cdr(odo.to_bytes())
        assert restored.frame_id == "odom"
        assert restored.child_frame_id == "base_link"
        # Composite drill-down — pose/twist round-trip end-to-end.
        assert restored.pose.pose.position.x == 1.0
        assert restored.pose.covariance == [0.01] * 36
        assert restored.twist.twist.linear.x == 0.5
        assert restored.twist.twist.angular.z == 0.1

    def test_default_pose_is_identity(self):
        h = Header(stamp=Time(0, 0), frame_id="odom")
        odo = Odometry(header=h, child_frame_id="base")
        restored = Odometry.from_cdr(odo.to_bytes())
        # Identity quaternion (w=1) is the natural default for Pose.
        assert restored.pose.pose.orientation.w == 1.0
        assert restored.pose.pose.position.x == 0.0
