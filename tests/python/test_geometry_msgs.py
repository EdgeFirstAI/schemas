"""Tests for geometry_msgs module."""

import pytest

from edgefirst.schemas import geometry_msgs


class TestVector3:
    """Tests for Vector3 message."""

    def test_vector3_creation_defaults(self):
        """Test creating a Vector3 with default values."""
        vec = geometry_msgs.Vector3()
        assert vec.x == 0.0
        assert vec.y == 0.0
        assert vec.z == 0.0

    def test_vector3_with_values(self):
        """Test creating a Vector3 with specific values."""
        vec = geometry_msgs.Vector3(x=1.0, y=2.0, z=3.0)
        assert vec.x == pytest.approx(1.0)
        assert vec.y == pytest.approx(2.0)
        assert vec.z == pytest.approx(3.0)

    def test_vector3_serialize_deserialize(self, sample_vector3):
        """Test CDR serialization roundtrip."""
        data = sample_vector3.serialize()
        assert isinstance(data, bytes)

        restored = geometry_msgs.Vector3.deserialize(data)
        assert restored.x == pytest.approx(sample_vector3.x)
        assert restored.y == pytest.approx(sample_vector3.y)
        assert restored.z == pytest.approx(sample_vector3.z)

    def test_vector3_negative_values(self):
        """Test Vector3 with negative values."""
        vec = geometry_msgs.Vector3(x=-1.5, y=-2.5, z=-3.5)
        data = vec.serialize()
        restored = geometry_msgs.Vector3.deserialize(data)
        assert restored.x == pytest.approx(-1.5)
        assert restored.y == pytest.approx(-2.5)
        assert restored.z == pytest.approx(-3.5)


class TestPoint:
    """Tests for Point message."""

    def test_point_creation_defaults(self):
        """Test creating a Point with default values."""
        point = geometry_msgs.Point()
        assert point.x == 0.0
        assert point.y == 0.0
        assert point.z == 0.0

    def test_point_serialize_deserialize(self, sample_point):
        """Test CDR serialization roundtrip."""
        data = sample_point.serialize()
        restored = geometry_msgs.Point.deserialize(data)
        assert restored.x == pytest.approx(sample_point.x)
        assert restored.y == pytest.approx(sample_point.y)
        assert restored.z == pytest.approx(sample_point.z)


class TestQuaternion:
    """Tests for Quaternion message."""

    def test_quaternion_creation_defaults(self):
        """Test creating a Quaternion with default values (identity)."""
        quat = geometry_msgs.Quaternion()
        assert quat.x == 0.0
        assert quat.y == 0.0
        assert quat.z == 0.0
        assert quat.w == 1.0  # Default is identity quaternion

    def test_quaternion_identity(self, sample_quaternion):
        """Test identity quaternion."""
        assert sample_quaternion.x == 0.0
        assert sample_quaternion.y == 0.0
        assert sample_quaternion.z == 0.0
        assert sample_quaternion.w == 1.0

    def test_quaternion_serialize_deserialize(self, sample_quaternion):
        """Test CDR serialization roundtrip."""
        data = sample_quaternion.serialize()
        restored = geometry_msgs.Quaternion.deserialize(data)
        assert restored.x == pytest.approx(sample_quaternion.x)
        assert restored.y == pytest.approx(sample_quaternion.y)
        assert restored.z == pytest.approx(sample_quaternion.z)
        assert restored.w == pytest.approx(sample_quaternion.w)

    def test_quaternion_rotation(self):
        """Test quaternion with rotation values."""
        # 90 degree rotation around Z axis
        quat = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.7071068, w=0.7071068)
        data = quat.serialize()
        restored = geometry_msgs.Quaternion.deserialize(data)
        assert restored.z == pytest.approx(0.7071068, rel=1e-6)
        assert restored.w == pytest.approx(0.7071068, rel=1e-6)


class TestPose:
    """Tests for Pose message."""

    def test_pose_creation_defaults(self):
        """Test creating a Pose with default values."""
        pose = geometry_msgs.Pose()
        assert pose.position is not None
        assert pose.orientation is not None

    def test_pose_serialize_deserialize(self, sample_pose):
        """Test CDR serialization roundtrip."""
        data = sample_pose.serialize()
        restored = geometry_msgs.Pose.deserialize(data)
        assert restored.position.x == pytest.approx(sample_pose.position.x)
        assert restored.position.y == pytest.approx(sample_pose.position.y)
        assert restored.position.z == pytest.approx(sample_pose.position.z)
        assert restored.orientation.w == pytest.approx(
            sample_pose.orientation.w
        )


class TestPoseStamped:
    """Tests for PoseStamped message."""

    def test_pose_stamped_serialize_deserialize(
        self, sample_header, sample_pose
    ):
        """Test CDR serialization roundtrip."""
        msg = geometry_msgs.PoseStamped(header=sample_header, pose=sample_pose)
        data = msg.serialize()
        restored = geometry_msgs.PoseStamped.deserialize(data)
        assert restored.header.frame_id == sample_header.frame_id
        assert restored.pose.position.x == pytest.approx(
            sample_pose.position.x
        )


class TestPose2D:
    """Tests for Pose2D message."""

    def test_pose2d_creation(self):
        """Test creating a Pose2D."""
        pose = geometry_msgs.Pose2D(x=1.0, y=2.0, theta=1.57)
        assert pose.x == pytest.approx(1.0)
        assert pose.y == pytest.approx(2.0)
        assert pose.theta == pytest.approx(1.57)

    def test_pose2d_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        pose = geometry_msgs.Pose2D(x=5.0, y=10.0, theta=3.14159)
        data = pose.serialize()
        restored = geometry_msgs.Pose2D.deserialize(data)
        assert restored.x == pytest.approx(5.0)
        assert restored.y == pytest.approx(10.0)
        assert restored.theta == pytest.approx(3.14159)


class TestTransform:
    """Tests for Transform message."""

    def test_transform_creation_defaults(self):
        """Test creating a Transform with default values."""
        transform = geometry_msgs.Transform()
        assert transform.translation is not None
        assert transform.rotation is not None

    def test_transform_serialize_deserialize(self, sample_transform):
        """Test CDR serialization roundtrip."""
        data = sample_transform.serialize()
        restored = geometry_msgs.Transform.deserialize(data)
        assert restored.translation.x == pytest.approx(
            sample_transform.translation.x
        )
        assert restored.rotation.w == pytest.approx(
            sample_transform.rotation.w
        )


class TestTransformStamped:
    """Tests for TransformStamped message."""

    def test_transform_stamped_serialize_deserialize(
        self, sample_header, sample_transform
    ):
        """Test CDR serialization roundtrip."""
        msg = geometry_msgs.TransformStamped(
            header=sample_header,
            child_frame_id="child_frame",
            transform=sample_transform,
        )
        data = msg.serialize()
        restored = geometry_msgs.TransformStamped.deserialize(data)
        assert restored.header.frame_id == sample_header.frame_id
        assert restored.child_frame_id == "child_frame"


class TestTwist:
    """Tests for Twist message."""

    def test_twist_creation_defaults(self):
        """Test creating a Twist with default values."""
        twist = geometry_msgs.Twist()
        assert twist.linear is not None
        assert twist.angular is not None

    def test_twist_serialize_deserialize(self, sample_twist):
        """Test CDR serialization roundtrip."""
        data = sample_twist.serialize()
        restored = geometry_msgs.Twist.deserialize(data)
        assert restored.linear.x == pytest.approx(sample_twist.linear.x)
        assert restored.angular.z == pytest.approx(sample_twist.angular.z)


class TestTwistStamped:
    """Tests for TwistStamped message."""

    def test_twist_stamped_serialize_deserialize(
        self, sample_header, sample_twist
    ):
        """Test CDR serialization roundtrip."""
        msg = geometry_msgs.TwistStamped(
            header=sample_header, twist=sample_twist
        )
        data = msg.serialize()
        restored = geometry_msgs.TwistStamped.deserialize(data)
        assert restored.header.frame_id == sample_header.frame_id
        assert restored.twist.linear.x == pytest.approx(sample_twist.linear.x)


class TestAccel:
    """Tests for Accel message."""

    def test_accel_creation_defaults(self):
        """Test creating an Accel with default values."""
        accel = geometry_msgs.Accel()
        assert accel.linear is not None
        assert accel.angular is not None

    def test_accel_serialize_deserialize(self, sample_vector3):
        """Test CDR serialization roundtrip."""
        accel = geometry_msgs.Accel(
            linear=sample_vector3,
            angular=geometry_msgs.Vector3(x=0.1, y=0.2, z=0.3),
        )
        data = accel.serialize()
        restored = geometry_msgs.Accel.deserialize(data)
        assert restored.linear.x == pytest.approx(sample_vector3.x)


class TestWrench:
    """Tests for Wrench message."""

    def test_wrench_creation_defaults(self):
        """Test creating a Wrench with default values."""
        wrench = geometry_msgs.Wrench()
        assert wrench.force is not None
        assert wrench.torque is not None

    def test_wrench_serialize_deserialize(self, sample_vector3):
        """Test CDR serialization roundtrip."""
        wrench = geometry_msgs.Wrench(
            force=sample_vector3,
            torque=geometry_msgs.Vector3(x=0.1, y=0.2, z=0.3),
        )
        data = wrench.serialize()
        restored = geometry_msgs.Wrench.deserialize(data)
        assert restored.force.x == pytest.approx(sample_vector3.x)


class TestInertia:
    """Tests for Inertia message."""

    def test_inertia_creation_defaults(self):
        """Test creating an Inertia with default values."""
        inertia = geometry_msgs.Inertia()
        assert inertia.m == 0.0
        assert inertia.com is not None

    def test_inertia_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        inertia = geometry_msgs.Inertia(
            m=10.0,
            com=geometry_msgs.Vector3(x=0.0, y=0.0, z=0.5),
            ixx=1.0,
            ixy=0.0,
            ixz=0.0,
            iyy=1.0,
            iyz=0.0,
            izz=1.0,
        )
        data = inertia.serialize()
        restored = geometry_msgs.Inertia.deserialize(data)
        assert restored.m == pytest.approx(10.0)
        assert restored.ixx == pytest.approx(1.0)


class TestPoseWithCovariance:
    """Tests for PoseWithCovariance message."""

    def test_pose_with_covariance_serialize_deserialize(self, sample_pose):
        """Test CDR serialization roundtrip."""
        msg = geometry_msgs.PoseWithCovariance(
            pose=sample_pose, covariance=[0.1 * i for i in range(36)]
        )
        data = msg.serialize()
        restored = geometry_msgs.PoseWithCovariance.deserialize(data)
        assert restored.pose.position.x == pytest.approx(
            sample_pose.position.x
        )
        assert len(restored.covariance) == 36
        assert restored.covariance[0] == pytest.approx(0.0)
        assert restored.covariance[1] == pytest.approx(0.1)


class TestTwistWithCovariance:
    """Tests for TwistWithCovariance message."""

    def test_twist_with_covariance_serialize_deserialize(self, sample_twist):
        """Test CDR serialization roundtrip."""
        msg = geometry_msgs.TwistWithCovariance(
            twist=sample_twist, covariance=[0.0] * 36
        )
        data = msg.serialize()
        restored = geometry_msgs.TwistWithCovariance.deserialize(data)
        assert restored.twist.linear.x == pytest.approx(sample_twist.linear.x)
        assert len(restored.covariance) == 36
