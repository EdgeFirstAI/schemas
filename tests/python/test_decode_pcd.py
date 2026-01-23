"""Tests for decode_pcd utility function.

These tests verify real-world usage patterns from the samples project.
"""

import struct
import pytest
from edgefirst.schemas import decode_pcd
from edgefirst.schemas import sensor_msgs, std_msgs, builtin_interfaces


@pytest.fixture
def sample_header():
    """Create a sample Header."""
    return std_msgs.Header(
        stamp=builtin_interfaces.Time(sec=1234567890, nanosec=0),
        frame_id="lidar_frame"
    )


@pytest.fixture
def xyz_point_cloud(sample_header):
    """Create a PointCloud2 with x, y, z fields (common LiDAR format)."""
    fields = [
        sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
        sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
        sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
    ]
    point_step = 12
    num_points = 5

    # Create point data: (1,2,3), (4,5,6), (7,8,9), (10,11,12), (13,14,15)
    data = bytearray()
    for i in range(num_points):
        data.extend(struct.pack("<fff", float(i*3+1), float(i*3+2), float(i*3+3)))

    return sensor_msgs.PointCloud2(
        header=sample_header,
        height=1,
        width=num_points,
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * num_points,
        data=bytes(data),
        is_dense=True,
    )


@pytest.fixture
def radar_point_cloud(sample_header):
    """Create PointCloud2 with radar target fields (x, y, z, cluster_id)."""
    fields = [
        sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
        sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
        sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
        sensor_msgs.PointField(
            name="cluster_id", offset=12, datatype=5, count=1
        ),  # INT32
    ]
    point_step = 16
    num_points = 3

    data = bytearray()
    # Point 1: (1.0, 2.0, 0.5) cluster 1
    data.extend(struct.pack("<fffi", 1.0, 2.0, 0.5, 1))
    # Point 2: (3.0, 4.0, 0.3) cluster 1
    data.extend(struct.pack("<fffi", 3.0, 4.0, 0.3, 1))
    # Point 3: (10.0, 5.0, 1.0) cluster 2
    data.extend(struct.pack("<fffi", 10.0, 5.0, 1.0, 2))

    return sensor_msgs.PointCloud2(
        header=sample_header,
        height=1,
        width=num_points,
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * num_points,
        data=bytes(data),
        is_dense=True,
    )


@pytest.fixture
def fusion_point_cloud(sample_header):
    """Create PointCloud2 with fusion fields (x, y, z, vision_class)."""
    fields = [
        sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
        sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
        sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
        sensor_msgs.PointField(
            name="vision_class", offset=12, datatype=5, count=1
        ),  # INT32
    ]
    point_step = 16
    num_points = 4

    data = bytearray()
    # Occupancy grid points with vision classes
    data.extend(struct.pack("<fffi", 1.0, 1.0, 0.0, 0))  # background
    data.extend(struct.pack("<fffi", 2.0, 2.0, 0.0, 1))  # person
    data.extend(struct.pack("<fffi", 3.0, 1.5, 0.0, 2))  # vehicle
    data.extend(struct.pack("<fffi", 4.0, 3.0, 0.0, 1))  # person

    return sensor_msgs.PointCloud2(
        header=sample_header,
        height=1,
        width=num_points,
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * num_points,
        data=bytes(data),
        is_dense=True,
    )


class TestDecodePcd:
    """Tests for decode_pcd function."""

    def test_decode_xyz_points(self, xyz_point_cloud):
        """Test decoding basic XYZ point cloud (LiDAR pattern)."""
        points = decode_pcd(xyz_point_cloud)
        assert len(points) == 5

        # Check first point
        assert points[0].x == pytest.approx(1.0)
        assert points[0].y == pytest.approx(2.0)
        assert points[0].z == pytest.approx(3.0)

        # Check last point
        assert points[4].x == pytest.approx(13.0)
        assert points[4].y == pytest.approx(14.0)
        assert points[4].z == pytest.approx(15.0)

    def test_decode_radar_targets(self, radar_point_cloud):
        """Test decoding radar targets with cluster_id (targets.py pattern)."""
        points = decode_pcd(radar_point_cloud)
        assert len(points) == 3

        # Access pattern from samples/python/radar/targets.py
        pos = [[p.x, p.y, p.z] for p in points]
        assert pos[0] == [pytest.approx(1.0), pytest.approx(2.0), pytest.approx(0.5)]

        # Filter by cluster_id (used in mega_sample.py)
        clusters = [p for p in points if p.cluster_id > 0]
        assert len(clusters) == 3

        max_id = max(p.cluster_id for p in clusters)
        assert max_id == 2

    def test_decode_fusion_occupancy(self, fusion_point_cloud):
        """Test decoding fusion output with vision_class (occupancy.py)."""
        points = decode_pcd(fusion_point_cloud)
        assert len(points) == 4

        # Access pattern from samples/python/fusion/occupancy.py
        max_class = max(max([p.vision_class for p in points]), 1)
        assert max_class == 2

        pos = [[p.x, p.y, p.z] for p in points]
        assert len(pos) == 4

    def test_decode_with_serialization_roundtrip(self, xyz_point_cloud):
        """Test decode_pcd after serialize/deserialize roundtrip."""
        # This mirrors the actual usage in samples
        serialized = xyz_point_cloud.serialize()
        restored = sensor_msgs.PointCloud2.deserialize(serialized)
        points = decode_pcd(restored)

        assert len(points) == 5
        assert points[0].x == pytest.approx(1.0)
        assert points[2].z == pytest.approx(9.0)

    def test_empty_point_cloud(self, sample_header):
        """Test decode_pcd with empty point cloud."""
        fields = [
            sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
        ]
        cloud = sensor_msgs.PointCloud2(
            header=sample_header,
            height=1,
            width=0,
            fields=fields,
            is_bigendian=False,
            point_step=4,
            row_step=0,
            data=bytes(),
            is_dense=True,
        )
        points = decode_pcd(cloud)
        assert len(points) == 0


class TestDecodePcdFieldTypes:
    """Test decode_pcd with various field data types."""

    def test_int8_field(self, sample_header):
        """Test INT8 field type."""
        fields = [
            sensor_msgs.PointField(
                name="intensity", offset=0, datatype=1, count=1
            ),  # INT8
        ]
        data = struct.pack("<b", -100)
        cloud = sensor_msgs.PointCloud2(
            header=sample_header,
            height=1,
            width=1,
            fields=fields,
            is_bigendian=False,
            point_step=1,
            row_step=1,
            data=bytes(data),
            is_dense=True,
        )
        points = decode_pcd(cloud)
        assert points[0].intensity == -100

    def test_uint8_field(self, sample_header):
        """Test UINT8 field type."""
        fields = [
            sensor_msgs.PointField(
                name="ring", offset=0, datatype=2, count=1
            ),  # UINT8
        ]
        data = struct.pack("<B", 255)
        cloud = sensor_msgs.PointCloud2(
            header=sample_header,
            height=1,
            width=1,
            fields=fields,
            is_bigendian=False,
            point_step=1,
            row_step=1,
            data=bytes(data),
            is_dense=True,
        )
        points = decode_pcd(cloud)
        assert points[0].ring == 255

    def test_float64_field(self, sample_header):
        """Test FLOAT64 field type."""
        fields = [
            sensor_msgs.PointField(
                name="time", offset=0, datatype=8, count=1
            ),  # FLOAT64
        ]
        data = struct.pack("<d", 1234567890.123456)
        cloud = sensor_msgs.PointCloud2(
            header=sample_header,
            height=1,
            width=1,
            fields=fields,
            is_bigendian=False,
            point_step=8,
            row_step=8,
            data=bytes(data),
            is_dense=True,
        )
        points = decode_pcd(cloud)
        assert points[0].time == pytest.approx(1234567890.123456)

    def test_multi_count_field(self, sample_header):
        """Test field with count > 1 (e.g., normal vector)."""
        fields = [
            sensor_msgs.PointField(
                name="normal", offset=0, datatype=7, count=3
            ),  # 3 floats
        ]
        data = struct.pack("<fff", 0.0, 0.0, 1.0)
        cloud = sensor_msgs.PointCloud2(
            header=sample_header,
            height=1,
            width=1,
            fields=fields,
            is_bigendian=False,
            point_step=12,
            row_step=12,
            data=bytes(data),
            is_dense=True,
        )
        points = decode_pcd(cloud)
        # Multi-count fields are accessed as normal, normal1, normal2
        assert points[0].normal == pytest.approx(0.0)
        assert points[0].normal1 == pytest.approx(0.0)
        assert points[0].normal2 == pytest.approx(1.0)


class TestDecodePcdEdgeCases:
    """Edge case tests for decode_pcd."""

    def test_big_endian(self, sample_header):
        """Test big-endian point cloud."""
        fields = [
            sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
        ]
        data = struct.pack(">f", 42.0)  # Big-endian
        cloud = sensor_msgs.PointCloud2(
            header=sample_header,
            height=1,
            width=1,
            fields=fields,
            is_bigendian=True,
            point_step=4,
            row_step=4,
            data=bytes(data),
            is_dense=True,
        )
        points = decode_pcd(cloud)
        assert points[0].x == pytest.approx(42.0)

    def test_organized_point_cloud(self, sample_header):
        """Test organized point cloud (height > 1)."""
        fields = [
            sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
            sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
            sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
        ]
        height, width = 2, 3
        point_step = 12

        data = bytearray()
        for i in range(height * width):
            data.extend(struct.pack("<fff", float(i), float(i), float(i)))

        cloud = sensor_msgs.PointCloud2(
            header=sample_header,
            height=height,
            width=width,
            fields=fields,
            is_bigendian=False,
            point_step=point_step,
            row_step=point_step * width,
            data=bytes(data),
            is_dense=True,
        )
        points = decode_pcd(cloud)
        assert len(points) == 6
        assert points[5].x == pytest.approx(5.0)

    def test_fields_with_padding(self, sample_header):
        """Test fields with gaps/padding between them."""
        fields = [
            sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
            # 4-byte gap
            sensor_msgs.PointField(name="y", offset=8, datatype=7, count=1),
        ]
        point_step = 12

        # Pack with padding
        data = struct.pack("<f", 1.0) + b'\x00\x00\x00\x00' + struct.pack("<f", 2.0)
        cloud = sensor_msgs.PointCloud2(
            header=sample_header,
            height=1,
            width=1,
            fields=fields,
            is_bigendian=False,
            point_step=point_step,
            row_step=point_step,
            data=bytes(data),
            is_dense=True,
        )
        points = decode_pcd(cloud)
        assert points[0].x == pytest.approx(1.0)
        assert points[0].y == pytest.approx(2.0)
