"""Shared pytest fixtures for EdgeFirst Schemas tests."""

from pathlib import Path

import pytest

from edgefirst.schemas import (
    builtin_interfaces,
    edgefirst_msgs,
    foxglove_msgs,
    geometry_msgs,
    nav_msgs,
    sensor_msgs,
    std_msgs,
)

# Path to test data directory
TESTDATA_DIR = Path(__file__).parent.parent.parent / "testdata"


def pytest_generate_tests(metafunc):
    """Dynamically generate test parameters for all MCAP files."""
    if "mcap_file" in metafunc.fixturenames:
        mcap_files = sorted(TESTDATA_DIR.glob("*.mcap"))
        if not mcap_files:
            pytest.skip("No MCAP files found in testdata/")
        metafunc.parametrize(
            "mcap_file",
            mcap_files,
            ids=[f.name for f in mcap_files],
        )


@pytest.fixture
def testdata_dir():
    """Return the testdata directory path."""
    return TESTDATA_DIR


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
        position=sample_point, orientation=sample_quaternion
    )


@pytest.fixture
def sample_transform(sample_vector3, sample_quaternion):
    """Create a sample Transform message."""
    return geometry_msgs.Transform(
        translation=sample_vector3, rotation=sample_quaternion
    )


@pytest.fixture
def sample_twist(sample_vector3):
    """Create a sample Twist message."""
    return geometry_msgs.Twist(
        linear=sample_vector3,
        angular=geometry_msgs.Vector3(x=0.1, y=0.2, z=0.3),
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
    """Create a sample PointCloud2 with 1024 points (matches Rust test)."""
    fields = [
        sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
        sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
        sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
    ]
    num_points = 1024
    point_step = 12
    return sensor_msgs.PointCloud2(
        header=sample_header,
        height=1,
        width=num_points,
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=12288,  # 12 * 1024 (matches Rust)
        data=bytes(12288),
        is_dense=True,
    )


@pytest.fixture
def sample_image(sample_header):
    """Create a sample VGA RGB8 Image (matches Rust test)."""
    width, height = 640, 480
    return sensor_msgs.Image(
        header=sample_header,
        height=height,
        width=width,
        encoding="rgb8",
        is_bigendian=0,
        step=1920,  # 640 * 3 (matches Rust)
        data=bytes([128] * (1920 * 480)),  # Gray image (128u8)
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
    """Create a sample NavSatFix (Montreal coords - matches Rust test)."""
    return sensor_msgs.NavSatFix(
        header=sample_header,
        status=sensor_msgs.NavSatStatus(status=0, service=1),
        latitude=45.5017,
        longitude=-73.5673,
        altitude=100.0,  # Matches Rust
        position_covariance=[
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ],  # Matches Rust identity matrix
        position_covariance_type=2,  # Matches Rust
    )


# ============================================================================
# EDGEFIRST_MSGS FIXTURES
# ============================================================================


@pytest.fixture
def sample_radar_cube(sample_header):
    """Create a sample RadarCube (matches Rust test dimensions)."""
    shape = [16, 256, 4, 64]  # seq, range, rx, doppler (matches Rust)
    total_elements = 16 * 256 * 4 * 64
    return edgefirst_msgs.RadarCube(
        header=sample_header,
        timestamp=1234567890123456,
        layout=[6, 1, 5, 2],  # SEQUENCE, RANGE, RXCHANNEL, DOPPLER
        shape=shape,
        scales=[1.0, 2.5, 1.0, 0.5],
        cube=[i % 32768 for i in range(total_elements)],  # i16 range
        is_complex=False,
    )


@pytest.fixture
def sample_dmabuf(sample_header):
    """Create a sample DmaBuffer message (matches Rust FHD)."""
    return edgefirst_msgs.DmaBuffer(
        header=sample_header,
        pid=12345,
        fd=42,  # Matches Rust
        width=1920,
        height=1080,
        stride=5760,  # 1920 * 3 (matches Rust RG24)
        fourcc=0x34325247,  # RG24 (matches Rust)
        length=1920 * 1080 * 3,
    )


@pytest.fixture
def sample_detect_box2d():
    """Create a sample detection box (matches Rust normalized coords)."""
    return edgefirst_msgs.Box(
        center_x=0.5,  # Normalized (matches Rust)
        center_y=0.5,
        width=0.1,
        height=0.2,
        label="car",  # Matches Rust
        score=0.98,  # Matches Rust
        distance=10.0,  # Matches Rust
        speed=5.0,  # Matches Rust
        track=edgefirst_msgs.Track(
            id="t1",  # Matches Rust
            lifetime=5,  # Matches Rust
            created=builtin_interfaces.Time(sec=95, nanosec=0),  # Matches Rust
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
    """Create a sample Mask message (matches Rust VGA)."""
    width, height = 640, 480
    return edgefirst_msgs.Mask(
        height=height,
        width=width,
        length=0,  # Uncompressed (matches Rust)
        encoding="",
        mask=bytes(width * height),  # Matches Rust vec![0u8; 480*640]
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
        pose=sample_pose, covariance=[0.0] * 36
    )
    twist_cov = geometry_msgs.TwistWithCovariance(
        twist=sample_twist, covariance=[0.0] * 36
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
