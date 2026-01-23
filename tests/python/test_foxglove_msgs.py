"""Tests for foxglove_msgs module."""

import pytest

from edgefirst.schemas import foxglove_msgs, geometry_msgs


class TestVector2:
    """Tests for Vector2 message."""

    def test_vector2_creation(self):
        """Test Vector2 structure."""
        v = foxglove_msgs.Vector2(x=1.5, y=2.5)
        assert v.x == pytest.approx(1.5)
        assert v.y == pytest.approx(2.5)

    def test_vector2_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        v = foxglove_msgs.Vector2(x=3.14, y=2.71)
        data = v.serialize()
        restored = foxglove_msgs.Vector2.deserialize(data)
        assert restored.x == pytest.approx(3.14)
        assert restored.y == pytest.approx(2.71)


class TestColor:
    """Tests for Color message."""

    def test_color_creation(self, sample_foxglove_color):
        """Test Color structure."""
        assert sample_foxglove_color.r == pytest.approx(1.0)
        assert sample_foxglove_color.g == pytest.approx(0.5)
        assert sample_foxglove_color.a == pytest.approx(1.0)

    def test_color_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        color = foxglove_msgs.Color(r=0.2, g=0.4, b=0.6, a=0.8)
        data = color.serialize()
        restored = foxglove_msgs.Color.deserialize(data)
        assert restored.r == pytest.approx(0.2)
        assert restored.g == pytest.approx(0.4)
        assert restored.b == pytest.approx(0.6)
        assert restored.a == pytest.approx(0.8)


class TestPoint2:
    """Tests for Point2 message."""

    def test_point2_creation(self):
        """Test Point2 structure."""
        p = foxglove_msgs.Point2(x=100.0, y=200.0)
        assert p.x == pytest.approx(100.0)
        assert p.y == pytest.approx(200.0)

    def test_point2_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        p = foxglove_msgs.Point2(x=50.5, y=75.5)
        data = p.serialize()
        restored = foxglove_msgs.Point2.deserialize(data)
        assert restored.x == pytest.approx(50.5)
        assert restored.y == pytest.approx(75.5)


class TestKeyValuePair:
    """Tests for KeyValuePair message."""

    def test_key_value_pair_creation(self):
        """Test KeyValuePair structure."""
        kv = foxglove_msgs.KeyValuePair(key="name", value="test_object")
        assert kv.key == "name"
        assert kv.value == "test_object"

    def test_key_value_pair_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        kv = foxglove_msgs.KeyValuePair(key="sensor_id", value="camera_front")
        data = kv.serialize()
        restored = foxglove_msgs.KeyValuePair.deserialize(data)
        assert restored.key == "sensor_id"
        assert restored.value == "camera_front"


class TestPackedElementField:
    """Tests for PackedElementField message."""

    def test_packed_element_field_creation(self):
        """Test PackedElementField structure."""
        field = foxglove_msgs.PackedElementField(
            name="x",
            offset=0,
            type=7,  # FLOAT32
        )
        assert field.name == "x"
        assert field.offset == 0
        assert field.type == 7

    def test_packed_element_field_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        field = foxglove_msgs.PackedElementField(
            name="intensity",
            offset=12,
            type=1,  # UINT8
        )
        data = field.serialize()
        restored = foxglove_msgs.PackedElementField.deserialize(data)
        assert restored.name == "intensity"
        assert restored.offset == 12


class TestCompressedVideo:
    """Tests for CompressedVideo message."""

    def test_compressed_video_creation(self, sample_compressed_video):
        """Test CompressedVideo structure."""
        assert sample_compressed_video.format == "h264"
        assert len(sample_compressed_video.data) == 10000

    def test_compressed_video_serialize_deserialize(self, sample_time):
        """Test CDR serialization roundtrip."""
        video = foxglove_msgs.CompressedVideo(
            timestamp=sample_time,
            frame_id="camera_front",
            data=bytes([0x00, 0x00, 0x00, 0x01]),  # NAL unit start
            format="h264",
        )
        data = video.serialize()
        restored = foxglove_msgs.CompressedVideo.deserialize(data)
        assert restored.format == "h264"
        assert restored.frame_id == "camera_front"
        assert list(restored.data) == [0x00, 0x00, 0x00, 0x01]


class TestCompressedImage:
    """Tests for Foxglove CompressedImage message."""

    def test_compressed_image_creation(self, sample_time):
        """Test CompressedImage structure."""
        img = foxglove_msgs.CompressedImage(
            timestamp=sample_time,
            frame_id="camera",
            data=bytes(5000),
            format="jpeg",
        )
        assert img.format == "jpeg"
        assert len(img.data) == 5000

    def test_compressed_image_serialize_deserialize(self, sample_time):
        """Test CDR serialization roundtrip."""
        img = foxglove_msgs.CompressedImage(
            timestamp=sample_time,
            frame_id="rgb_camera",
            data=bytes([0xFF, 0xD8, 0xFF, 0xE0]),  # JPEG header
            format="jpeg",
        )
        data = img.serialize()
        restored = foxglove_msgs.CompressedImage.deserialize(data)
        assert restored.format == "jpeg"
        assert list(restored.data)[:4] == [0xFF, 0xD8, 0xFF, 0xE0]


class TestCameraCalibration:
    """Tests for CameraCalibration message."""

    def test_camera_calibration_creation(self, sample_time):
        """Test CameraCalibration structure."""
        calib = foxglove_msgs.CameraCalibration(
            timestamp=sample_time,
            frame_id="camera_optical",
            width=1920,
            height=1080,
            distortion_model="plumb_bob",
            d=[0.0, 0.0, 0.0, 0.0, 0.0],
            k=[960.0, 0, 960.0, 0, 960.0, 540.0, 0, 0, 1.0],
            r=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
            p=[960.0, 0, 960.0, 0, 0, 960.0, 540.0, 0, 0, 0, 1.0, 0],
        )
        assert calib.width == 1920
        assert calib.height == 1080

    def test_camera_calibration_serialize_deserialize(self, sample_time):
        """Test CDR serialization roundtrip."""
        calib = foxglove_msgs.CameraCalibration(
            timestamp=sample_time,
            frame_id="camera",
            width=640,
            height=480,
            distortion_model="plumb_bob",
            d=[0.1, -0.2, 0.0, 0.0, 0.0],
            k=[500.0, 0, 320.0, 0, 500.0, 240.0, 0, 0, 1.0],
            r=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],
            p=[500.0, 0, 320.0, 0, 0, 500.0, 240.0, 0, 0, 0, 1.0, 0],
        )
        data = calib.serialize()
        restored = foxglove_msgs.CameraCalibration.deserialize(data)
        assert restored.width == 640
        assert restored.distortion_model == "plumb_bob"
        assert restored.k[0] == pytest.approx(500.0)


class TestFrameTransform:
    """Tests for FrameTransform message."""

    def test_frame_transform_creation(self, sample_time):
        """Test FrameTransform structure."""
        tf = foxglove_msgs.FrameTransform(
            timestamp=sample_time,
            parent_frame_id="world",
            child_frame_id="base_link",
            translation=geometry_msgs.Vector3(x=1.0, y=2.0, z=0.0),
            rotation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
        )
        assert tf.parent_frame_id == "world"
        assert tf.child_frame_id == "base_link"

    def test_frame_transform_serialize_deserialize(self, sample_time):
        """Test CDR serialization roundtrip."""
        tf = foxglove_msgs.FrameTransform(
            timestamp=sample_time,
            parent_frame_id="odom",
            child_frame_id="base_footprint",
            translation=geometry_msgs.Vector3(x=5.0, y=3.0, z=0.1),
            rotation=geometry_msgs.Quaternion(x=0, y=0, z=0.707, w=0.707),
        )
        data = tf.serialize()
        restored = foxglove_msgs.FrameTransform.deserialize(data)
        assert restored.parent_frame_id == "odom"
        assert restored.translation.x == pytest.approx(5.0)
        assert restored.rotation.z == pytest.approx(0.707)


class TestCircleAnnotation:
    """Tests for CircleAnnotation message."""

    def test_circle_annotation_creation(self, sample_time):
        """Test CircleAnnotation structure."""
        circle = foxglove_msgs.CircleAnnotation(
            timestamp=sample_time,
            position=foxglove_msgs.Point2(x=320.0, y=240.0),
            diameter=50.0,
            thickness=2.0,
            fill_color=foxglove_msgs.Color(r=1.0, g=0, b=0, a=0.5),
            outline_color=foxglove_msgs.Color(r=1.0, g=0, b=0, a=1.0),
        )
        assert circle.diameter == pytest.approx(50.0)

    def test_circle_annotation_serialize_deserialize(self, sample_time):
        """Test CDR serialization roundtrip."""
        circle = foxglove_msgs.CircleAnnotation(
            timestamp=sample_time,
            position=foxglove_msgs.Point2(x=100.0, y=100.0),
            diameter=30.0,
            thickness=1.0,
            fill_color=foxglove_msgs.Color(r=0, g=1.0, b=0, a=0.3),
            outline_color=foxglove_msgs.Color(r=0, g=1.0, b=0, a=1.0),
        )
        data = circle.serialize()
        restored = foxglove_msgs.CircleAnnotation.deserialize(data)
        assert restored.position.x == pytest.approx(100.0)
        assert restored.diameter == pytest.approx(30.0)


class TestTextAnnotation:
    """Tests for TextAnnotation message."""

    def test_text_annotation_creation(self, sample_time):
        """Test TextAnnotation structure."""
        text = foxglove_msgs.TextAnnotation(
            timestamp=sample_time,
            position=foxglove_msgs.Point2(x=50.0, y=50.0),
            text="Hello World",
            font_size=12.0,
            text_color=foxglove_msgs.Color(r=1.0, g=1.0, b=1.0, a=1.0),
            background_color=foxglove_msgs.Color(r=0, g=0, b=0, a=0.5),
        )
        assert text.text == "Hello World"
        assert text.font_size == pytest.approx(12.0)

    def test_text_annotation_serialize_deserialize(self, sample_time):
        """Test CDR serialization roundtrip."""
        text = foxglove_msgs.TextAnnotation(
            timestamp=sample_time,
            position=foxglove_msgs.Point2(x=200.0, y=300.0),
            text="Detection: person 95%",
            font_size=16.0,
            text_color=foxglove_msgs.Color(r=1.0, g=1.0, b=0, a=1.0),
            background_color=foxglove_msgs.Color(r=0, g=0, b=0, a=0.7),
        )
        data = text.serialize()
        restored = foxglove_msgs.TextAnnotation.deserialize(data)
        assert restored.text == "Detection: person 95%"


class TestLog:
    """Tests for Log message."""

    def test_log_creation(self, sample_time):
        """Test Log structure."""
        log = foxglove_msgs.Log(
            timestamp=sample_time,
            level=foxglove_msgs.LogLevel.INFO.value,
            message="System initialized",
            name="main",
            file="main.cpp",
            line=42,
        )
        assert log.message == "System initialized"
        assert log.level == foxglove_msgs.LogLevel.INFO.value

    def test_log_serialize_deserialize(self, sample_time):
        """Test CDR serialization roundtrip."""
        log = foxglove_msgs.Log(
            timestamp=sample_time,
            level=foxglove_msgs.LogLevel.WARNING.value,
            message="Low memory warning",
            name="memory_monitor",
            file="monitor.py",
            line=100,
        )
        data = log.serialize()
        restored = foxglove_msgs.Log.deserialize(data)
        assert restored.message == "Low memory warning"
        assert restored.level == foxglove_msgs.LogLevel.WARNING.value


class TestLocationFix:
    """Tests for LocationFix message."""

    def test_location_fix_creation(self, sample_time):
        """Test LocationFix structure."""
        fix = foxglove_msgs.LocationFix(
            timestamp=sample_time,
            frame_id="gps",
            latitude=45.5017,
            longitude=-73.5673,
            altitude=50.0,
            position_covariance=[1.0, 0, 0, 0, 1.0, 0, 0, 0, 4.0],
            position_covariance_type=2,
        )
        assert fix.latitude == pytest.approx(45.5017)
        assert fix.longitude == pytest.approx(-73.5673)

    def test_location_fix_serialize_deserialize(self, sample_time):
        """Test CDR serialization roundtrip."""
        fix = foxglove_msgs.LocationFix(
            timestamp=sample_time,
            frame_id="gnss",
            latitude=40.7128,
            longitude=-74.0060,
            altitude=10.0,
            position_covariance=[0.5] * 9,
            position_covariance_type=1,
        )
        data = fix.serialize()
        restored = foxglove_msgs.LocationFix.deserialize(data)
        assert restored.latitude == pytest.approx(40.7128)
        assert restored.longitude == pytest.approx(-74.0060)


class TestGeoJSON:
    """Tests for GeoJSON message."""

    def test_geojson_creation(self):
        """Test GeoJSON structure."""
        geojson = foxglove_msgs.GeoJSON(
            geojson='{"type": "Point", "coordinates": [-73.5673, 45.5017]}'
        )
        assert "Point" in geojson.geojson

    def test_geojson_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        geojson = foxglove_msgs.GeoJSON(
            geojson='{"type": "LineString", "coordinates": [[0,0], [1,1]]}'
        )
        data = geojson.serialize()
        restored = foxglove_msgs.GeoJSON.deserialize(data)
        assert "LineString" in restored.geojson


class TestLaserScan:
    """Tests for Foxglove LaserScan message."""

    def test_laser_scan_creation(self, sample_time):
        """Test LaserScan structure."""
        scan = foxglove_msgs.LaserScan(
            timestamp=sample_time,
            frame_id="laser",
            pose=geometry_msgs.Pose(
                position=geometry_msgs.Point(x=0, y=0, z=0.1),
                orientation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
            ),
            start_angle=-1.57,
            end_angle=1.57,
            ranges=[1.0] * 100,
            intensities=[50.0] * 100,
        )
        assert len(scan.ranges) == 100

    def test_laser_scan_serialize_deserialize(self, sample_time):
        """Test CDR serialization roundtrip."""
        scan = foxglove_msgs.LaserScan(
            timestamp=sample_time,
            frame_id="lidar",
            pose=geometry_msgs.Pose(
                position=geometry_msgs.Point(x=0, y=0, z=0),
                orientation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
            ),
            start_angle=0.0,
            end_angle=6.28,
            ranges=[i * 0.1 for i in range(50)],
            intensities=[100.0] * 50,
        )
        data = scan.serialize()
        restored = foxglove_msgs.LaserScan.deserialize(data)
        assert len(restored.ranges) == 50
        assert restored.start_angle == pytest.approx(0.0)


class TestGrid:
    """Tests for Grid message (occupancy grid for visualization)."""

    def test_grid_creation(self, sample_time):
        """Test Grid structure."""
        grid = foxglove_msgs.Grid(
            timestamp=sample_time,
            frame_id="map",
            pose=geometry_msgs.Pose(
                position=geometry_msgs.Point(x=0, y=0, z=0),
                orientation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
            ),
            column_count=10,
            cell_size=foxglove_msgs.Vector2(x=0.1, y=0.1),
            row_stride=10,
            cell_stride=1,
            fields=[
                foxglove_msgs.PackedElementField(
                    name="occupancy", offset=0, type=1
                )
            ],
            data=0,  # data is uint8, not sequence
        )
        assert grid.column_count == 10

    def test_grid_serialize_deserialize(self, sample_time):
        """Test CDR serialization roundtrip."""
        grid = foxglove_msgs.Grid(
            timestamp=sample_time,
            frame_id="base_link",
            pose=geometry_msgs.Pose(
                position=geometry_msgs.Point(x=-5, y=-5, z=0),
                orientation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
            ),
            column_count=100,
            cell_size=foxglove_msgs.Vector2(x=0.05, y=0.05),
            row_stride=100,
            cell_stride=1,
            fields=[
                foxglove_msgs.PackedElementField(name="cost", offset=0, type=1)
            ],
            data=255,  # data is uint8, not sequence
        )
        data = grid.serialize()
        restored = foxglove_msgs.Grid.deserialize(data)
        assert restored.column_count == 100
        assert restored.data == 255
