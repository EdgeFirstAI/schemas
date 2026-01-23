"""Tests verifying real-world usage patterns from samples project.

These tests mirror the exact usage patterns from EdgeFirst/samples/python
to ensure the schemas work correctly in production scenarios.
"""

import pytest
import numpy as np
from edgefirst.schemas import (
    decode_pcd,
    builtin_interfaces,
    std_msgs,
    sensor_msgs,
    geometry_msgs,
    edgefirst_msgs,
)


@pytest.fixture
def sample_header():
    """Create a sample Header."""
    return std_msgs.Header(
        stamp=builtin_interfaces.Time(sec=1234567890, nanosec=123456789),
        frame_id="camera_frame"
    )


class TestDetectBoxesUsage:
    """Test Detect/Box usage patterns from boxes2d.py and mega_sample.py."""

    def test_boxes2d_access_pattern(self, sample_header):
        """Test access pattern from samples/python/model/boxes2d.py."""
        # Create detection with boxes
        boxes = [
            edgefirst_msgs.Box(
                center_x=0.5, center_y=0.3,
                width=0.2, height=0.3,
                label="person", score=0.95,
                track=edgefirst_msgs.Track(id="", lifetime=0),
            ),
            edgefirst_msgs.Box(
                center_x=0.7, center_y=0.6,
                width=0.15, height=0.25,
                label="car", score=0.88,
                track=edgefirst_msgs.Track(id="", lifetime=0),
            ),
        ]
        detect = edgefirst_msgs.Detect(
            header=sample_header,
            input_timestamp=builtin_interfaces.Time(sec=100, nanosec=0),
            boxes=boxes,
        )

        # Serialize and deserialize (as done via Zenoh)
        data = detect.serialize()
        detection = edgefirst_msgs.Detect.deserialize(data)

        # Pattern from boxes2d.py lines 34-42
        centers = []
        sizes = []
        labels = []
        for box in detection.boxes:
            centers.append((box.center_x, box.center_y))
            sizes.append((box.width, box.height))
            labels.append(box.label)

        assert len(centers) == 2
        assert centers[0] == (pytest.approx(0.5), pytest.approx(0.3))
        assert labels == ["person", "car"]

    def test_boxes2d_tracked_access_pattern(self, sample_header):
        """Test tracked boxes pattern from boxes2d_tracked.py."""
        boxes = [
            edgefirst_msgs.Box(
                center_x=0.5, center_y=0.3,
                width=0.2, height=0.3,
                label="person", score=0.95,
                track=edgefirst_msgs.Track(id="abc123def456", lifetime=10),
            ),
            edgefirst_msgs.Box(
                center_x=0.7, center_y=0.6,
                width=0.15, height=0.25,
                label="car", score=0.88,
                track=edgefirst_msgs.Track(id="", lifetime=0),  # untracked
            ),
        ]
        detect = edgefirst_msgs.Detect(
            header=sample_header,
            input_timestamp=builtin_interfaces.Time(sec=100, nanosec=0),
            boxes=boxes,
        )

        data = detect.serialize()
        detection = edgefirst_msgs.Detect.deserialize(data)

        # Pattern from boxes2d_tracked.py lines 34-52
        boxes_tracked = {}
        for box in detection.boxes:
            if box.track.id and box.track.id not in boxes_tracked:
                boxes_tracked[box.track.id] = [
                    box.label + ": " + box.track.id[:6],
                    [0, 255, 0],  # color
                ]

        assert "abc123def456" in boxes_tracked
        assert boxes_tracked["abc123def456"][0] == "person: abc123"

    def test_detect_model_time_access(self, sample_header):
        """Test model_time access from mega_sample.py line 197."""
        detect = edgefirst_msgs.Detect(
            header=sample_header,
            input_timestamp=builtin_interfaces.Time(sec=100, nanosec=0),
            model_time=builtin_interfaces.Duration(sec=0, nanosec=5000000),
            boxes=[],
        )

        data = detect.serialize()
        detection = edgefirst_msgs.Detect.deserialize(data)

        # Pattern from mega_sample.py lines 194-199
        inference_time = (
            float(detection.model_time.sec) +
            float(detection.model_time.nanosec / 1e9)
        )
        assert inference_time == pytest.approx(0.005)


class TestMaskUsage:
    """Test Mask usage patterns from mask.py and mega_sample.py."""

    def test_mask_numpy_reshape(self, sample_header):
        """Test mask numpy reshape pattern from mask.py."""
        width, height, classes = 64, 48, 2
        mask_data = [i % 256 for i in range(width * height * classes)]

        mask = edgefirst_msgs.Mask(
            width=width,
            height=height,
            length=classes,
            mask=mask_data,
            boxed=False,
        )

        data = mask.serialize()
        mask = edgefirst_msgs.Mask.deserialize(data)

        # Pattern from mask.py lines 36-40
        np_arr = np.asarray(mask.mask, dtype=np.uint8)
        np_arr = np.reshape(np_arr, [mask.height, mask.width, -1])
        np_arr = np.argmax(np_arr, axis=2)

        assert np_arr.shape == (height, width)


class TestRadarCubeUsage:
    """Test RadarCube usage patterns from cube.py."""

    def test_radar_cube_numpy_reshape(self, sample_header):
        """Test radar cube reshape pattern from cube.py."""
        shape = [16, 64, 4, 32]  # SEQ, RANGE, RX, DOPPLER
        total_size = 16 * 64 * 4 * 32
        # RadarCube.cube is sequence[int16], not float
        cube_data = [i % 100 for i in range(total_size)]

        radar_cube = edgefirst_msgs.RadarCube(
            header=sample_header,
            timestamp=123456789,
            layout=[
                edgefirst_msgs.RadarChannel.SEQUENCE.value,
                edgefirst_msgs.RadarChannel.RANGE.value,
                edgefirst_msgs.RadarChannel.RXCHANNEL.value,
                edgefirst_msgs.RadarChannel.DOPPLER.value,
            ],
            shape=shape,
            scales=[1.0, 0.1, 1.0, 0.5],
            cube=cube_data,
            is_complex=False,
        )

        data = radar_cube.serialize()
        radar_cube = edgefirst_msgs.RadarCube.deserialize(data)

        # Pattern from cube.py lines 34-39
        data_array = np.array(radar_cube.cube).reshape(radar_cube.shape)
        data_array = np.abs(data_array)

        assert data_array.shape == tuple(shape)


class TestImuUsage:
    """Test Imu usage patterns from imu.py."""

    def test_imu_orientation_access(self, sample_header):
        """Test IMU orientation access pattern from imu.py."""
        imu = sensor_msgs.Imu(
            header=sample_header,
            orientation=geometry_msgs.Quaternion(
                x=0.1, y=0.2, z=0.3, w=0.9
            ),
            angular_velocity=geometry_msgs.Vector3(x=0.0, y=0.0, z=0.0),
            linear_acceleration=geometry_msgs.Vector3(x=0.0, y=0.0, z=9.8),
        )

        data = imu.serialize()
        imu = sensor_msgs.Imu.deserialize(data)

        # Pattern from imu.py lines 34-41
        x = imu.orientation.x
        y = imu.orientation.y
        z = imu.orientation.z
        w = imu.orientation.w

        assert x == pytest.approx(0.1)
        assert y == pytest.approx(0.2)
        assert z == pytest.approx(0.3)
        assert w == pytest.approx(0.9)


class TestNavSatFixUsage:
    """Test NavSatFix usage patterns from gps.py."""

    def test_gps_lat_lon_access(self, sample_header):
        """Test GPS lat/lon access pattern from gps.py."""
        gps = sensor_msgs.NavSatFix(
            header=sample_header,
            latitude=45.4215,
            longitude=-75.6972,
            altitude=100.0,
        )

        data = gps.serialize()
        gps = sensor_msgs.NavSatFix.deserialize(data)

        # Pattern from gps.py lines 33-35
        lat_lon = [gps.latitude, gps.longitude]

        assert lat_lon[0] == pytest.approx(45.4215)
        assert lat_lon[1] == pytest.approx(-75.6972)


class TestCompressedImageUsage:
    """Test CompressedImage usage patterns from jpeg.py."""

    def test_jpeg_data_access(self, sample_header):
        """Test JPEG data access pattern from jpeg.py."""
        # Fake JPEG data (just bytes, not real JPEG)
        jpeg_data = bytes([0xFF, 0xD8, 0xFF, 0xE0] + [0x00] * 100)

        image = sensor_msgs.CompressedImage(
            header=sample_header,
            format="jpeg",
            data=jpeg_data,
        )

        data = image.serialize()
        image = sensor_msgs.CompressedImage.deserialize(data)

        # Pattern from jpeg.py line 41
        np_arr = np.frombuffer(bytearray(image.data), np.uint8)

        assert len(np_arr) == 104
        assert np_arr[0] == 0xFF  # JPEG marker


class TestCameraInfoUsage:
    """Test CameraInfo usage patterns from camera_info.py."""

    def test_camera_info_dimensions(self, sample_header):
        """Test CameraInfo width/height access from camera_info.py."""
        info = sensor_msgs.CameraInfo(
            header=sample_header,
            width=1920,
            height=1080,
            distortion_model="plumb_bob",
            d=[0.0, 0.0, 0.0, 0.0, 0.0],
            k=[1000.0, 0.0, 960.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 1.0],
            r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            p=[1000.0, 0.0, 960.0, 0.0, 0.0, 1000.0, 540.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        )

        data = info.serialize()
        info = sensor_msgs.CameraInfo.deserialize(data)

        # Pattern from camera_info.py lines 36-40
        width = info.width
        height = info.height

        assert width == 1920
        assert height == 1080


class TestModelInfoUsage:
    """Test ModelInfo usage patterns from model_info.py."""

    def test_model_info_access(self, sample_header):
        """Test ModelInfo type/name access from model_info.py."""
        info = edgefirst_msgs.ModelInfo(
            header=sample_header,
            input_shape=[1, 640, 640, 3],
            input_type=edgefirst_msgs.model_info.UINT8.value,
            output_shape=[1, 8400, 84],
            output_type=edgefirst_msgs.model_info.FLOAT32.value,
            labels=["person", "car", "bicycle"],
            model_type="object_detection",
            model_format="TFLite",
            model_name="yolov8n",
        )

        data = info.serialize()
        info = edgefirst_msgs.ModelInfo.deserialize(data)

        # Pattern from model_info.py lines 36-39
        m_type = info.model_type
        m_name = info.model_name

        assert m_type == "object_detection"
        assert m_name == "yolov8n"


class TestDmaBufferUsage:
    """Test DmaBuffer usage patterns from mega_sample.py."""

    def test_dma_buffer_access(self, sample_header):
        """Test DmaBuffer field access from mega_sample.py."""
        dma_buf = edgefirst_msgs.DmaBuffer(
            header=sample_header,
            pid=12345,
            fd=10,
            width=1920,
            height=1080,
            stride=1920 * 2,
            fourcc=0x56595559,  # YUYV
            length=1920 * 1080 * 2,
        )

        data = dma_buf.serialize()
        dma_buf = edgefirst_msgs.DmaBuffer.deserialize(data)

        # Pattern from mega_sample.py lines 107-120
        pid = dma_buf.pid
        fd = dma_buf.fd
        width = dma_buf.width
        height = dma_buf.height
        length = dma_buf.length

        assert pid == 12345
        assert fd == 10
        assert width == 1920
        assert height == 1080
        assert length == 1920 * 1080 * 2


class TestDetect3DBoxesUsage:
    """Test 3D boxes usage patterns from mega_sample.py."""

    def test_boxes3d_conversion(self, sample_header):
        """Test 3D box coordinate conversion from mega_sample.py."""
        boxes = [
            edgefirst_msgs.Box(
                center_x=1.0, center_y=0.5,
                width=2.0, height=1.5,
                label="person", score=0.9,
                distance=5.0,
                track=edgefirst_msgs.Track(),
            ),
        ]
        detect = edgefirst_msgs.Detect(
            header=sample_header,
            input_timestamp=builtin_interfaces.Time(sec=100, nanosec=0),
            boxes=boxes,
        )

        data = detect.serialize()
        detection = edgefirst_msgs.Detect.deserialize(data)

        # Pattern from mega_sample.py lines 278-282
        # Convert from optical frame to normal frame
        centers = [
            (x.distance, -x.center_x, -x.center_y)
            for x in detection.boxes
        ]
        sizes = [
            (x.width, x.width, x.height)
            for x in detection.boxes
        ]

        assert centers[0] == (
            pytest.approx(5.0),
            pytest.approx(-1.0),
            pytest.approx(-0.5)
        )
        assert sizes[0] == (
            pytest.approx(2.0),
            pytest.approx(2.0),
            pytest.approx(1.5)
        )
