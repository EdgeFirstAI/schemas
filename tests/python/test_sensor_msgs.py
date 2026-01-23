"""Tests for sensor_msgs module."""

import pytest

from edgefirst.schemas import geometry_msgs, sensor_msgs


class TestPointField:
    """Tests for PointField message."""

    def test_point_field_creation(self):
        """Test creating a PointField."""
        field = sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1)
        assert field.name == "x"
        assert field.offset == 0
        assert field.datatype == 7  # FLOAT32
        assert field.count == 1

    def test_point_field_serialize_deserialize(self, sample_point_field):
        """Test CDR serialization roundtrip."""
        data = sample_point_field.serialize()
        restored = sensor_msgs.PointField.deserialize(data)
        assert restored.name == sample_point_field.name
        assert restored.offset == sample_point_field.offset
        assert restored.datatype == sample_point_field.datatype
        assert restored.count == sample_point_field.count


class TestPointCloud2:
    """Tests for PointCloud2 message."""

    def test_point_cloud2_creation(self, sample_point_cloud2):
        """Test PointCloud2 structure (matches Rust 1024 points)."""
        assert sample_point_cloud2.height == 1
        assert sample_point_cloud2.width == 1024
        assert len(sample_point_cloud2.fields) == 3
        assert sample_point_cloud2.is_dense is True
        assert sample_point_cloud2.row_step == 12288

    def test_point_cloud2_serialize_deserialize(self, sample_point_cloud2):
        """Test CDR serialization roundtrip."""
        data = sample_point_cloud2.serialize()
        restored = sensor_msgs.PointCloud2.deserialize(data)
        assert restored.width == sample_point_cloud2.width
        assert restored.height == sample_point_cloud2.height
        assert len(restored.fields) == len(sample_point_cloud2.fields)
        assert len(restored.data) == len(sample_point_cloud2.data)
        assert restored.is_dense == sample_point_cloud2.is_dense

    def test_point_cloud2_fields_preserved(self, sample_point_cloud2):
        """Test that field metadata is preserved."""
        data = sample_point_cloud2.serialize()
        restored = sensor_msgs.PointCloud2.deserialize(data)
        for orig, rest in zip(sample_point_cloud2.fields, restored.fields):
            assert orig.name == rest.name
            assert orig.offset == rest.offset
            assert orig.datatype == rest.datatype

    @pytest.mark.slow
    def test_point_cloud2_large(self, sample_header):
        """Test PointCloud2 with large point count (65536 points)."""
        fields = [
            sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
            sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
            sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
            sensor_msgs.PointField(
                name="intensity", offset=12, datatype=7, count=1
            ),
        ]
        num_points = 65536
        point_step = 16
        cloud = sensor_msgs.PointCloud2(
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

        data = cloud.serialize()
        restored = sensor_msgs.PointCloud2.deserialize(data)
        assert restored.width == num_points
        assert len(restored.data) == point_step * num_points


class TestImage:
    """Tests for Image message."""

    def test_image_creation(self, sample_image):
        """Test Image structure."""
        assert sample_image.width == 640
        assert sample_image.height == 480
        assert sample_image.encoding == "rgb8"

    def test_image_serialize_deserialize(self, sample_image):
        """Test CDR serialization roundtrip."""
        data = sample_image.serialize()
        restored = sensor_msgs.Image.deserialize(data)
        assert restored.width == sample_image.width
        assert restored.height == sample_image.height
        assert restored.encoding == sample_image.encoding
        assert len(restored.data) == len(sample_image.data)
        assert restored.step == sample_image.step

    def test_image_encodings(self, sample_header):
        """Test Image with different encodings."""
        encodings = ["rgb8", "bgr8", "mono8", "mono16", "yuv422"]
        for enc in encodings:
            image = sensor_msgs.Image(
                header=sample_header,
                height=100,
                width=100,
                encoding=enc,
                is_bigendian=0,
                step=100,
                data=bytes(100 * 100),
            )
            data = image.serialize()
            restored = sensor_msgs.Image.deserialize(data)
            assert restored.encoding == enc

    @pytest.mark.slow
    def test_image_full_hd(self, sample_header):
        """Test Image with Full HD resolution."""
        width, height = 1920, 1080
        image = sensor_msgs.Image(
            header=sample_header,
            height=height,
            width=width,
            encoding="rgb8",
            is_bigendian=0,
            step=width * 3,
            data=bytes(width * height * 3),
        )

        data = image.serialize()
        assert len(data) > 6_000_000  # ~6.2 MB for FHD RGB
        restored = sensor_msgs.Image.deserialize(data)
        assert restored.width == width
        assert restored.height == height


class TestCompressedImage:
    """Tests for CompressedImage message."""

    def test_compressed_image_creation(self, sample_header):
        """Test CompressedImage structure."""
        img = sensor_msgs.CompressedImage(
            header=sample_header,
            format="jpeg",
            data=bytes(1000),
        )
        assert img.format == "jpeg"
        assert len(img.data) == 1000

    def test_compressed_image_serialize_deserialize(self, sample_header):
        """Test CDR serialization roundtrip."""
        img = sensor_msgs.CompressedImage(
            header=sample_header,
            format="png",
            data=bytes([0x89, 0x50, 0x4E, 0x47]),  # PNG header
        )
        data = img.serialize()
        restored = sensor_msgs.CompressedImage.deserialize(data)
        assert restored.format == "png"
        assert list(restored.data) == [0x89, 0x50, 0x4E, 0x47]


class TestImu:
    """Tests for IMU message."""

    def test_imu_creation(self, sample_imu):
        """Test IMU structure."""
        assert sample_imu.orientation is not None
        assert len(sample_imu.orientation_covariance) == 9
        assert sample_imu.angular_velocity is not None
        assert sample_imu.linear_acceleration is not None

    def test_imu_serialize_deserialize(self, sample_imu):
        """Test CDR serialization roundtrip."""
        data = sample_imu.serialize()
        restored = sensor_msgs.Imu.deserialize(data)
        assert restored.orientation.w == pytest.approx(
            sample_imu.orientation.w
        )
        assert restored.linear_acceleration.z == pytest.approx(9.81)
        assert len(restored.orientation_covariance) == 9

    def test_imu_covariance_values(self, sample_header):
        """Test IMU with specific covariance values."""
        imu = sensor_msgs.Imu(
            header=sample_header,
            orientation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
            orientation_covariance=[0.01 * i for i in range(9)],
            angular_velocity=geometry_msgs.Vector3(x=0, y=0, z=0),
            angular_velocity_covariance=[0.02 * i for i in range(9)],
            linear_acceleration=geometry_msgs.Vector3(x=0, y=0, z=9.81),
            linear_acceleration_covariance=[0.03 * i for i in range(9)],
        )
        data = imu.serialize()
        restored = sensor_msgs.Imu.deserialize(data)
        assert restored.orientation_covariance[1] == pytest.approx(0.01)
        assert restored.angular_velocity_covariance[1] == pytest.approx(0.02)
        assert restored.linear_acceleration_covariance[1] == pytest.approx(
            0.03
        )


class TestNavSatFix:
    """Tests for NavSatFix message."""

    def test_navsatfix_creation(self, sample_navsatfix):
        """Test NavSatFix structure (matches Rust Montreal coords)."""
        assert sample_navsatfix.latitude == pytest.approx(45.5017)
        assert sample_navsatfix.longitude == pytest.approx(-73.5673)
        assert sample_navsatfix.altitude == pytest.approx(
            100.0
        )  # Matches Rust
        assert sample_navsatfix.position_covariance_type == 2  # Matches Rust

    def test_navsatfix_serialize_deserialize(self, sample_navsatfix):
        """Test CDR serialization roundtrip."""
        data = sample_navsatfix.serialize()
        restored = sensor_msgs.NavSatFix.deserialize(data)
        assert restored.latitude == pytest.approx(sample_navsatfix.latitude)
        assert restored.longitude == pytest.approx(sample_navsatfix.longitude)
        assert restored.altitude == pytest.approx(sample_navsatfix.altitude)
        assert len(restored.position_covariance) == 9
        # Verify identity matrix covariance (matches Rust)
        assert restored.position_covariance[0] == pytest.approx(1.0)
        assert restored.position_covariance[4] == pytest.approx(1.0)
        assert restored.position_covariance[8] == pytest.approx(1.0)


class TestNavSatStatus:
    """Tests for NavSatStatus message."""

    def test_navsatstatus_creation(self):
        """Test NavSatStatus structure."""
        status = sensor_msgs.NavSatStatus(status=0, service=1)
        assert status.status == 0
        assert status.service == 1

    def test_navsatstatus_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        status = sensor_msgs.NavSatStatus(status=1, service=3)
        data = status.serialize()
        restored = sensor_msgs.NavSatStatus.deserialize(data)
        assert restored.status == 1
        assert restored.service == 3


class TestCameraInfo:
    """Tests for CameraInfo message."""

    def test_camera_info_creation(self, sample_header):
        """Test CameraInfo structure."""
        info = sensor_msgs.CameraInfo(
            header=sample_header,
            height=480,
            width=640,
            distortion_model="plumb_bob",
            d=[0.0, 0.0, 0.0, 0.0, 0.0],
            k=[500.0, 0, 320.0, 0, 500.0, 240.0, 0, 0, 1.0],
            r=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
            p=[500.0, 0, 320.0, 0, 0, 500.0, 240.0, 0, 0, 0, 1.0, 0],
            binning_x=0,
            binning_y=0,
            roi=sensor_msgs.RegionOfInterest(
                x_offset=0, y_offset=0, height=0, width=0, do_rectify=False
            ),
        )
        assert info.width == 640
        assert info.height == 480

    def test_camera_info_serialize_deserialize(self, sample_header):
        """Test CDR serialization roundtrip."""
        info = sensor_msgs.CameraInfo(
            header=sample_header,
            height=480,
            width=640,
            distortion_model="plumb_bob",
            d=[0.1, 0.2, 0.0, 0.0, 0.0],
            k=[500.0, 0, 320.0, 0, 500.0, 240.0, 0, 0, 1.0],
            r=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
            p=[500.0, 0, 320.0, 0, 0, 500.0, 240.0, 0, 0, 0, 1.0, 0],
            binning_x=0,
            binning_y=0,
            roi=sensor_msgs.RegionOfInterest(
                x_offset=0, y_offset=0, height=0, width=0, do_rectify=False
            ),
        )
        data = info.serialize()
        restored = sensor_msgs.CameraInfo.deserialize(data)
        assert restored.width == 640
        assert restored.height == 480
        assert restored.distortion_model == "plumb_bob"
        assert len(restored.k) == 9
        assert restored.k[0] == pytest.approx(500.0)


class TestRegionOfInterest:
    """Tests for RegionOfInterest message."""

    def test_roi_creation(self):
        """Test RegionOfInterest structure."""
        roi = sensor_msgs.RegionOfInterest(
            x_offset=100,
            y_offset=50,
            height=200,
            width=300,
            do_rectify=True,
        )
        assert roi.x_offset == 100
        assert roi.y_offset == 50
        assert roi.height == 200
        assert roi.width == 300
        assert roi.do_rectify is True

    def test_roi_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        roi = sensor_msgs.RegionOfInterest(
            x_offset=10,
            y_offset=20,
            height=100,
            width=150,
            do_rectify=False,
        )
        data = roi.serialize()
        restored = sensor_msgs.RegionOfInterest.deserialize(data)
        assert restored.x_offset == 10
        assert restored.y_offset == 20
        assert restored.height == 100
        assert restored.width == 150
        assert restored.do_rectify is False


class TestLaserScan:
    """Tests for LaserScan message."""

    def test_laser_scan_creation(self, sample_header):
        """Test LaserScan structure."""
        scan = sensor_msgs.LaserScan(
            header=sample_header,
            angle_min=-1.57,
            angle_max=1.57,
            angle_increment=0.01,
            time_increment=0.0001,
            scan_time=0.1,
            range_min=0.1,
            range_max=30.0,
            ranges=[1.0] * 314,
            intensities=[100.0] * 314,
        )
        assert len(scan.ranges) == 314
        assert scan.angle_min == pytest.approx(-1.57)

    def test_laser_scan_serialize_deserialize(self, sample_header):
        """Test CDR serialization roundtrip."""
        scan = sensor_msgs.LaserScan(
            header=sample_header,
            angle_min=-1.57,
            angle_max=1.57,
            angle_increment=0.01,
            time_increment=0.0001,
            scan_time=0.1,
            range_min=0.1,
            range_max=30.0,
            ranges=[i * 0.1 for i in range(100)],
            intensities=[50.0] * 100,
        )
        data = scan.serialize()
        restored = sensor_msgs.LaserScan.deserialize(data)
        assert len(restored.ranges) == 100
        assert restored.ranges[10] == pytest.approx(1.0)


class TestRange:
    """Tests for Range message."""

    def test_range_creation(self, sample_header):
        """Test Range structure."""
        rng = sensor_msgs.Range(
            header=sample_header,
            radiation_type=0,  # ULTRASOUND
            field_of_view=0.5,
            min_range=0.1,
            max_range=5.0,
            range=2.5,
        )
        assert rng.range == pytest.approx(2.5)

    def test_range_serialize_deserialize(self, sample_header):
        """Test CDR serialization roundtrip."""
        rng = sensor_msgs.Range(
            header=sample_header,
            radiation_type=1,  # INFRARED
            field_of_view=0.3,
            min_range=0.05,
            max_range=2.0,
            range=1.5,
        )
        data = rng.serialize()
        restored = sensor_msgs.Range.deserialize(data)
        assert restored.radiation_type == 1
        assert restored.range == pytest.approx(1.5)


class TestJointState:
    """Tests for JointState message."""

    def test_joint_state_creation(self, sample_header):
        """Test JointState structure."""
        state = sensor_msgs.JointState(
            header=sample_header,
            name=["joint1", "joint2", "joint3"],
            position=[0.0, 1.57, 3.14],
            velocity=[0.1, 0.2, 0.3],
            effort=[1.0, 2.0, 3.0],
        )
        assert len(state.name) == 3
        assert state.position[1] == pytest.approx(1.57)

    def test_joint_state_serialize_deserialize(self, sample_header):
        """Test CDR serialization roundtrip."""
        state = sensor_msgs.JointState(
            header=sample_header,
            name=["shoulder", "elbow"],
            position=[0.5, 1.0],
            velocity=[0.0, 0.0],
            effort=[10.0, 5.0],
        )
        data = state.serialize()
        restored = sensor_msgs.JointState.deserialize(data)
        assert restored.name == ["shoulder", "elbow"]
        assert restored.position[0] == pytest.approx(0.5)
