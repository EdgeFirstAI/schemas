"""Shared pytest fixtures for EdgeFirst Schemas tests."""

import pytest
from edgefirst.schemas import (
    builtin_interfaces,
    std_msgs,
    geometry_msgs,
    sensor_msgs,
    edgefirst_msgs,
    foxglove_msgs,
    nav_msgs,
)


# ============================================================================
# BUILTIN INTERFACES FIXTURES
# ============================================================================


@pytest.fixture
def sample_time():
    """Create a sample Time message."""
    return builtin_interfaces.Time(sec=1234567890, nanosec=123456789)


@pytest.fixture
def sample_duration():
    """Create a sample Duration message."""
    return builtin_interfaces.Duration(sec=60, nanosec=500000000)


# ============================================================================
# STD_MSGS FIXTURES
# ============================================================================


@pytest.fixture
def sample_header(sample_time):
    """Create a sample Header message."""
    return std_msgs.Header(stamp=sample_time, frame_id="test_frame")


@pytest.fixture
def sample_color_rgba():
    """Create a sample ColorRGBA message."""
    return std_msgs.ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)


# ============================================================================
# GEOMETRY_MSGS FIXTURES
# ============================================================================


@pytest.fixture
def sample_vector3():
    """Create a sample Vector3 message."""
    return geometry_msgs.Vector3(x=1.0, y=2.0, z=3.0)


@pytest.fixture
def sample_point():
    """Create a sample Point message."""
    return geometry_msgs.Point(x=1.0, y=2.0, z=3.0)


@pytest.fixture
def sample_quaternion():
    """Create a sample Quaternion (identity)."""
    return geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


@pytest.fixture
def sample_pose(sample_point, sample_quaternion):
    """Create a sample Pose message."""
    return geometry_msgs.Pose(
        position=sample_point,
        orientation=sample_quaternion
    )


@pytest.fixture
def sample_transform(sample_vector3, sample_quaternion):
    """Create a sample Transform message."""
    return geometry_msgs.Transform(
        translation=sample_vector3,
        rotation=sample_quaternion
    )


@pytest.fixture
def sample_twist(sample_vector3):
    """Create a sample Twist message."""
    return geometry_msgs.Twist(
        linear=sample_vector3,
        angular=geometry_msgs.Vector3(x=0.1, y=0.2, z=0.3)
    )


# ============================================================================
# SENSOR_MSGS FIXTURES
# ============================================================================


@pytest.fixture
def sample_point_field():
    """Create a sample PointField for x coordinate."""
    return sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1)


@pytest.fixture
def sample_point_cloud2(sample_header):
    """Create a sample PointCloud2 with 100 points."""
    fields = [
        sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
        sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
        sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
    ]
    num_points = 100
    point_step = 12
    return sensor_msgs.PointCloud2(
        header=sample_header,
        height=1,
        width=num_points,
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * num_points,
        data=bytes(point_step * num_points),
        is_dense=True,
    )


@pytest.fixture
def sample_image(sample_header):
    """Create a sample 640x480 RGB8 Image."""
    width, height = 640, 480
    return sensor_msgs.Image(
        header=sample_header,
        height=height,
        width=width,
        encoding="rgb8",
        is_bigendian=0,
        step=width * 3,
        data=bytes(width * height * 3),
    )


@pytest.fixture
def sample_imu(sample_header, sample_quaternion, sample_vector3):
    """Create a sample IMU message."""
    return sensor_msgs.Imu(
        header=sample_header,
        orientation=sample_quaternion,
        orientation_covariance=[0.0] * 9,
        angular_velocity=sample_vector3,
        angular_velocity_covariance=[0.0] * 9,
        linear_acceleration=geometry_msgs.Vector3(x=0.0, y=0.0, z=9.81),
        linear_acceleration_covariance=[0.0] * 9,
    )


@pytest.fixture
def sample_navsatfix(sample_header):
    """Create a sample NavSatFix message."""
    return sensor_msgs.NavSatFix(
        header=sample_header,
        status=sensor_msgs.NavSatStatus(
            status=0,
            service=1
        ),
        latitude=45.5017,
        longitude=-73.5673,
        altitude=50.0,
        position_covariance=[0.0] * 9,
        position_covariance_type=0,
    )


# ============================================================================
# EDGEFIRST_MSGS FIXTURES
# ============================================================================


@pytest.fixture
def sample_radar_cube(sample_header):
    """Create a sample RadarCube."""
    shape = [8, 64, 4, 32]  # seq, range, rx, doppler
    total_elements = 8 * 64 * 4 * 32
    return edgefirst_msgs.RadarCube(
        header=sample_header,
        timestamp=1234567890123456,
        layout=[6, 1, 5, 2],  # SEQUENCE, RANGE, RXCHANNEL, DOPPLER
        shape=shape,
        scales=[1.0, 2.5, 1.0, 0.5],
        cube=[0] * total_elements,
        is_complex=False,
    )


@pytest.fixture
def sample_dmabuf(sample_header):
    """Create a sample DmaBuffer message."""
    return edgefirst_msgs.DmaBuffer(
        header=sample_header,
        pid=12345,
        fd=10,
        width=1920,
        height=1080,
        stride=3840,
        fourcc=0x56595559,  # YUYV
        length=1920 * 1080 * 2,
    )


@pytest.fixture
def sample_detect_box2d():
    """Create a sample detection box."""
    return edgefirst_msgs.Box(
        center_x=320.0,
        center_y=240.0,
        width=100.0,
        height=80.0,
        label="person",
        score=0.95,
        distance=5.0,
        speed=0.0,
        track=edgefirst_msgs.Track(
            id="track_1", lifetime=10,
            created=builtin_interfaces.Time(sec=0, nanosec=0),
        ),
    )


@pytest.fixture
def sample_detect(sample_header, sample_time, sample_detect_box2d):
    """Create a sample Detect message."""
    return edgefirst_msgs.Detect(
        header=sample_header,
        input_timestamp=sample_time,
        boxes=[sample_detect_box2d],
    )


@pytest.fixture
def sample_mask():
    """Create a sample Mask message."""
    width, height = 640, 480
    return edgefirst_msgs.Mask(
        height=height,
        width=width,
        length=1,
        encoding="",
        mask=bytes(width * height),
        boxed=False,
    )


# ============================================================================
# FOXGLOVE_MSGS FIXTURES
# ============================================================================


@pytest.fixture
def sample_compressed_video(sample_time):
    """Create a sample FoxgloveCompressedVideo."""
    return foxglove_msgs.CompressedVideo(
        timestamp=sample_time,
        data=bytes(10000),  # 10KB H.264 data
        format="h264",
        frame_id="camera",
    )


@pytest.fixture
def sample_foxglove_color():
    """Create a sample Foxglove Color."""
    return foxglove_msgs.Color(r=1.0, g=0.5, b=0.0, a=1.0)


# ============================================================================
# NAV_MSGS FIXTURES
# ============================================================================


@pytest.fixture
def sample_odometry(sample_header, sample_pose, sample_twist):
    """Create a sample Odometry message."""
    pose_cov = geometry_msgs.PoseWithCovariance(
        pose=sample_pose,
        covariance=[0.0] * 36
    )
    twist_cov = geometry_msgs.TwistWithCovariance(
        twist=sample_twist,
        covariance=[0.0] * 36
    )
    return nav_msgs.Odometry(
        header=sample_header,
        child_frame_id="base_link",
        pose=pose_cov,
        twist=twist_cov,
    )


@pytest.fixture
def sample_path(sample_header, sample_pose):
    """Create a sample Path message."""
    poses = [
        geometry_msgs.PoseStamped(header=sample_header, pose=sample_pose)
        for _ in range(5)
    ]
    return nav_msgs.Path(header=sample_header, poses=poses)


@pytest.fixture
def sample_occupancy_grid(sample_header):
    """Create a sample OccupancyGrid message."""
    width, height = 100, 100
    return nav_msgs.OccupancyGrid(
        header=sample_header,
        info=nav_msgs.MapMetaData(
            map_load_time=builtin_interfaces.Time(sec=0, nanosec=0),
            resolution=0.05,
            width=width,
            height=height,
            origin=geometry_msgs.Pose(
                position=geometry_msgs.Point(x=0.0, y=0.0, z=0.0),
                orientation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
            ),
        ),
        data=[0] * (width * height),
    )
