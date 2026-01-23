# SPDX-License-Identifier: Apache-2.0
# Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

"""
Comprehensive performance benchmarks for CDR serialization/deserialization.

This benchmark suite measures serialization performance for EdgeFirst schemas,
focusing on heavy message types used in production perception pipelines.

Run all benchmarks: `pytest benches/python/ --benchmark-only`
Run specific group: `pytest benches/python/ -k "radar_cube" --benchmark-only`
Fast mode for CI: `BENCH_FAST=1 pytest benches/python/ --benchmark-only`

Note: This mirrors the Rust benchmark suite in benches/serialization.rs
"""

import os
import random

import pytest

from edgefirst.schemas.builtin_interfaces import (
    Duration,
    Time,
)
from edgefirst.schemas.std_msgs import (
    ColorRGBA,
    Header,
)
from edgefirst.schemas.geometry_msgs import (
    Point,
    Point32,
    Pose,
    Pose2D,
    Quaternion,
    Transform,
    Twist,
    Vector3,
)
from edgefirst.schemas.sensor_msgs import (
    Image,
    PointCloud2,
    PointField,
)
from edgefirst.schemas.edgefirst_msgs import (
    DmaBuffer,
    Mask,
    RadarCube,
)
from edgefirst.schemas.foxglove_msgs import (
    CompressedVideo as FoxgloveCompressedVideo,
)


def is_fast_mode() -> bool:
    """Check if fast benchmark mode is enabled via BENCH_FAST=1 environment variable.

    Fast mode runs fewer benchmark variants for quicker CI feedback (~5-10 min vs ~20 min).
    """
    value = os.environ.get("BENCH_FAST", "")
    return value == "1" or value.lower() == "true"


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================


def create_header() -> Header:
    """Create a standard header for benchmarking."""
    return Header(
        stamp=Time(sec=1234567890, nanosec=123456789),
        frame_id="sensor_frame",
    )


def create_dmabuf(width: int, height: int, fourcc: int) -> DmaBuffer:
    """Create a DmaBuffer message representing a camera frame reference."""
    # Bytes per pixel based on fourcc
    if fourcc == 0x56595559:  # YUYV
        bytes_per_pixel = 2
    elif fourcc == 0x3231564E:  # NV12
        bytes_per_pixel = 1
    else:  # RGB
        bytes_per_pixel = 3

    return DmaBuffer(
        header=create_header(),
        pid=12345,
        fd=42,
        width=width,
        height=height,
        stride=width * bytes_per_pixel,
        fourcc=fourcc,
        length=width * height * bytes_per_pixel,
    )


def create_compressed_video(data_size: int) -> FoxgloveCompressedVideo:
    """Create a FoxgloveCompressedVideo with random data simulating H.264 NAL units."""
    return FoxgloveCompressedVideo(
        timestamp=Time(sec=1234567890, nanosec=123456789),
        frame_id="sensor_frame",
        data=bytes(random.getrandbits(8) for _ in range(data_size)),
        format="h264",
    )


def create_radar_cube(shape: tuple[int, int, int, int], is_complex: bool) -> RadarCube:
    """Create a RadarCube with random i16 data simulating real radar returns.

    Shape: [chirp_types, range_gates, rx_channels, doppler_bins]
    """
    total_elements = shape[0] * shape[1] * shape[2] * shape[3]
    return RadarCube(
        header=create_header(),
        timestamp=1234567890123456,
        layout=[6, 1, 5, 2],  # SEQUENCE, RANGE, RXCHANNEL, DOPPLER
        shape=list(shape),
        scales=[1.0, 0.117, 1.0, 0.156],  # range: 0.117m/bin, speed: 0.156m/s/bin
        cube=[random.randint(-32768, 32767) for _ in range(total_elements)],
        is_complex=is_complex,
    )


def create_point_cloud(num_points: int) -> PointCloud2:
    """Create a PointCloud2 with standard LiDAR point format (x, y, z, intensity)."""
    point_step = 16  # 4 fields * 4 bytes (FLOAT32)
    fields = [
        PointField(name="x", offset=0, datatype=7, count=1),  # FLOAT32
        PointField(name="y", offset=4, datatype=7, count=1),
        PointField(name="z", offset=8, datatype=7, count=1),
        PointField(name="intensity", offset=12, datatype=7, count=1),
    ]

    return PointCloud2(
        header=create_header(),
        height=1,
        width=num_points,
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * num_points,
        data=bytes(random.getrandbits(8) for _ in range(point_step * num_points)),
        is_dense=True,
    )


def create_mask(width: int, height: int, channels: int) -> Mask:
    """Create a Mask with random segmentation data."""
    data_size = width * height * channels
    return Mask(
        height=height,
        width=width,
        length=channels,
        encoding="",  # No compression
        mask=bytes(random.getrandbits(8) for _ in range(data_size)),
        boxed=False,
    )


def create_compressed_mask(width: int, height: int, channels: int) -> Mask:
    """Create a compressed Mask with zstd-encoded data."""
    try:
        import zstd
    except ImportError:
        # Fall back to uncompressed if zstd not available
        return create_mask(width, height, channels)

    data_size = width * height * channels
    raw_data = bytes(random.getrandbits(8) for _ in range(data_size))
    compressed = zstd.compress(raw_data, 3)

    return Mask(
        height=height,
        width=width,
        length=channels,
        encoding="zstd",
        mask=compressed,
        boxed=False,
    )


def create_image(width: int, height: int, encoding: str, bytes_per_pixel: int) -> Image:
    """Create an Image with random pixel data."""
    step = width * bytes_per_pixel
    data_size = step * height
    return Image(
        header=create_header(),
        height=height,
        width=width,
        encoding=encoding,
        is_bigendian=0,
        step=step,
        data=bytes(random.getrandbits(8) for _ in range(data_size)),
    )


# ============================================================================
# BENCHMARK: builtin_interfaces
# ============================================================================


class TestBuiltinInterfacesBenchmarks:
    """Benchmarks for builtin_interfaces messages."""

    def test_time_serialize(self, benchmark):
        """Benchmark Time serialization."""
        time = Time(sec=1234567890, nanosec=123456789)
        benchmark(time.serialize)

    def test_time_deserialize(self, benchmark):
        """Benchmark Time deserialization."""
        time = Time(sec=1234567890, nanosec=123456789)
        data = time.serialize()
        benchmark(Time.deserialize, data)

    def test_duration_serialize(self, benchmark):
        """Benchmark Duration serialization."""
        duration = Duration(sec=60, nanosec=500000000)
        benchmark(duration.serialize)

    def test_duration_deserialize(self, benchmark):
        """Benchmark Duration deserialization."""
        duration = Duration(sec=60, nanosec=500000000)
        data = duration.serialize()
        benchmark(Duration.deserialize, data)


# ============================================================================
# BENCHMARK: std_msgs
# ============================================================================


class TestStdMsgsBenchmarks:
    """Benchmarks for std_msgs messages."""

    def test_header_serialize(self, benchmark):
        """Benchmark Header serialization."""
        header = create_header()
        benchmark(header.serialize)

    def test_header_deserialize(self, benchmark):
        """Benchmark Header deserialization."""
        header = create_header()
        data = header.serialize()
        benchmark(Header.deserialize, data)

    def test_colorrgba_serialize(self, benchmark):
        """Benchmark ColorRGBA serialization."""
        color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        benchmark(color.serialize)

    def test_colorrgba_deserialize(self, benchmark):
        """Benchmark ColorRGBA deserialization."""
        color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        data = color.serialize()
        benchmark(ColorRGBA.deserialize, data)


# ============================================================================
# BENCHMARK: geometry_msgs
# ============================================================================


class TestGeometryMsgsBenchmarks:
    """Benchmarks for geometry_msgs messages."""

    def test_vector3_serialize(self, benchmark):
        """Benchmark Vector3 serialization."""
        vec3 = Vector3(x=1.5, y=2.5, z=3.5)
        benchmark(vec3.serialize)

    def test_vector3_deserialize(self, benchmark):
        """Benchmark Vector3 deserialization."""
        vec3 = Vector3(x=1.5, y=2.5, z=3.5)
        data = vec3.serialize()
        benchmark(Vector3.deserialize, data)

    def test_point_serialize(self, benchmark):
        """Benchmark Point serialization."""
        point = Point(x=10.0, y=20.0, z=30.0)
        benchmark(point.serialize)

    def test_point_deserialize(self, benchmark):
        """Benchmark Point deserialization."""
        point = Point(x=10.0, y=20.0, z=30.0)
        data = point.serialize()
        benchmark(Point.deserialize, data)

    def test_point32_serialize(self, benchmark):
        """Benchmark Point32 serialization."""
        point32 = Point32(x=1.0, y=2.0, z=3.0)
        benchmark(point32.serialize)

    def test_point32_deserialize(self, benchmark):
        """Benchmark Point32 deserialization."""
        point32 = Point32(x=1.0, y=2.0, z=3.0)
        data = point32.serialize()
        benchmark(Point32.deserialize, data)

    def test_quaternion_serialize(self, benchmark):
        """Benchmark Quaternion serialization."""
        quat = Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)
        benchmark(quat.serialize)

    def test_quaternion_deserialize(self, benchmark):
        """Benchmark Quaternion deserialization."""
        quat = Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)
        data = quat.serialize()
        benchmark(Quaternion.deserialize, data)

    def test_pose_serialize(self, benchmark):
        """Benchmark Pose serialization."""
        pose = Pose(
            position=Point(x=10.0, y=20.0, z=30.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707),
        )
        benchmark(pose.serialize)

    def test_pose_deserialize(self, benchmark):
        """Benchmark Pose deserialization."""
        pose = Pose(
            position=Point(x=10.0, y=20.0, z=30.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707),
        )
        data = pose.serialize()
        benchmark(Pose.deserialize, data)

    def test_pose2d_serialize(self, benchmark):
        """Benchmark Pose2D serialization."""
        pose2d = Pose2D(x=10.0, y=20.0, theta=1.57)
        benchmark(pose2d.serialize)

    def test_pose2d_deserialize(self, benchmark):
        """Benchmark Pose2D deserialization."""
        pose2d = Pose2D(x=10.0, y=20.0, theta=1.57)
        data = pose2d.serialize()
        benchmark(Pose2D.deserialize, data)

    def test_transform_serialize(self, benchmark):
        """Benchmark Transform serialization."""
        transform = Transform(
            translation=Vector3(x=1.5, y=2.5, z=3.5),
            rotation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707),
        )
        benchmark(transform.serialize)

    def test_transform_deserialize(self, benchmark):
        """Benchmark Transform deserialization."""
        transform = Transform(
            translation=Vector3(x=1.5, y=2.5, z=3.5),
            rotation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707),
        )
        data = transform.serialize()
        benchmark(Transform.deserialize, data)

    def test_twist_serialize(self, benchmark):
        """Benchmark Twist serialization."""
        twist = Twist(
            linear=Vector3(x=1.5, y=2.5, z=3.5),
            angular=Vector3(x=0.0, y=0.0, z=0.5),
        )
        benchmark(twist.serialize)

    def test_twist_deserialize(self, benchmark):
        """Benchmark Twist deserialization."""
        twist = Twist(
            linear=Vector3(x=1.5, y=2.5, z=3.5),
            angular=Vector3(x=0.0, y=0.0, z=0.5),
        )
        data = twist.serialize()
        benchmark(Twist.deserialize, data)


# ============================================================================
# BENCHMARK: DmaBuffer (Lightweight reference)
# ============================================================================


class TestDmaBufferBenchmarks:
    """Benchmarks for DmaBuffer messages.

    DmaBuffer is a lightweight message - just metadata, no pixel data.
    Used as a reference to show overhead of heavy messages vs zero-copy.
    """

    @pytest.fixture(autouse=True)
    def setup_sizes(self):
        """Set up benchmark sizes based on fast mode."""
        if is_fast_mode():
            self.sizes = [((1280, 720, 0x56595559), "HD_yuyv")]
        else:
            self.sizes = [
                ((640, 480, 0x56595559), "VGA_yuyv"),
                ((1280, 720, 0x56595559), "HD_yuyv"),
                ((1920, 1080, 0x56595559), "FHD_yuyv"),
            ]

    def test_dmabuf_vga_serialize(self, benchmark):
        """Benchmark DmaBuffer VGA serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        dmabuf = create_dmabuf(640, 480, 0x56595559)
        benchmark(dmabuf.serialize)

    def test_dmabuf_vga_deserialize(self, benchmark):
        """Benchmark DmaBuffer VGA deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        dmabuf = create_dmabuf(640, 480, 0x56595559)
        data = dmabuf.serialize()
        benchmark(DmaBuffer.deserialize, data)

    def test_dmabuf_hd_serialize(self, benchmark):
        """Benchmark DmaBuffer HD serialization."""
        dmabuf = create_dmabuf(1280, 720, 0x56595559)
        benchmark(dmabuf.serialize)

    def test_dmabuf_hd_deserialize(self, benchmark):
        """Benchmark DmaBuffer HD deserialization."""
        dmabuf = create_dmabuf(1280, 720, 0x56595559)
        data = dmabuf.serialize()
        benchmark(DmaBuffer.deserialize, data)

    def test_dmabuf_fhd_serialize(self, benchmark):
        """Benchmark DmaBuffer FHD serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        dmabuf = create_dmabuf(1920, 1080, 0x56595559)
        benchmark(dmabuf.serialize)

    def test_dmabuf_fhd_deserialize(self, benchmark):
        """Benchmark DmaBuffer FHD deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        dmabuf = create_dmabuf(1920, 1080, 0x56595559)
        data = dmabuf.serialize()
        benchmark(DmaBuffer.deserialize, data)


# ============================================================================
# BENCHMARK: FoxgloveCompressedVideo (Heavy)
# ============================================================================


class TestCompressedVideoBenchmarks:
    """Benchmarks for FoxgloveCompressedVideo messages.

    Test sizes: 10KB, 100KB, 500KB, 1MB (simulating H.264 frame sizes).
    Fast mode: 100KB and 500KB only (representative sizes).
    """

    def test_compressed_video_10kb_serialize(self, benchmark):
        """Benchmark CompressedVideo 10KB serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        video = create_compressed_video(10_000)
        benchmark(video.serialize)

    def test_compressed_video_10kb_deserialize(self, benchmark):
        """Benchmark CompressedVideo 10KB deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        video = create_compressed_video(10_000)
        data = video.serialize()
        benchmark(FoxgloveCompressedVideo.deserialize, data)

    def test_compressed_video_100kb_serialize(self, benchmark):
        """Benchmark CompressedVideo 100KB serialization."""
        video = create_compressed_video(100_000)
        benchmark(video.serialize)

    def test_compressed_video_100kb_deserialize(self, benchmark):
        """Benchmark CompressedVideo 100KB deserialization."""
        video = create_compressed_video(100_000)
        data = video.serialize()
        benchmark(FoxgloveCompressedVideo.deserialize, data)

    def test_compressed_video_500kb_serialize(self, benchmark):
        """Benchmark CompressedVideo 500KB serialization."""
        video = create_compressed_video(500_000)
        benchmark(video.serialize)

    def test_compressed_video_500kb_deserialize(self, benchmark):
        """Benchmark CompressedVideo 500KB deserialization."""
        video = create_compressed_video(500_000)
        data = video.serialize()
        benchmark(FoxgloveCompressedVideo.deserialize, data)

    def test_compressed_video_1mb_serialize(self, benchmark):
        """Benchmark CompressedVideo 1MB serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        video = create_compressed_video(1_000_000)
        benchmark(video.serialize)

    def test_compressed_video_1mb_deserialize(self, benchmark):
        """Benchmark CompressedVideo 1MB deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        video = create_compressed_video(1_000_000)
        data = video.serialize()
        benchmark(FoxgloveCompressedVideo.deserialize, data)


# ============================================================================
# BENCHMARK: RadarCube (Heavy)
# ============================================================================


class TestRadarCubeBenchmarks:
    """Benchmarks for RadarCube messages.

    SmartMicro DRVEGRD radar cube configurations:
    - DRVEGRD-169: 4D/UHD Corner Radar (77-81 GHz)
    - DRVEGRD-171: 4D/PXHD Front Radar (76-77 GHz)

    Fast mode: 3 representative configurations (short, medium, long).
    """

    def test_radar_cube_drvegrd169_ultra_short_serialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-169 Ultra-Short serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        cube = create_radar_cube((4, 64, 12, 256), is_complex=True)
        benchmark(cube.serialize)

    def test_radar_cube_drvegrd169_ultra_short_deserialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-169 Ultra-Short deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        cube = create_radar_cube((4, 64, 12, 256), is_complex=True)
        data = cube.serialize()
        benchmark(RadarCube.deserialize, data)

    def test_radar_cube_drvegrd169_short_serialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-169 Short serialization."""
        cube = create_radar_cube((4, 128, 12, 128), is_complex=True)
        benchmark(cube.serialize)

    def test_radar_cube_drvegrd169_short_deserialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-169 Short deserialization."""
        cube = create_radar_cube((4, 128, 12, 128), is_complex=True)
        data = cube.serialize()
        benchmark(RadarCube.deserialize, data)

    def test_radar_cube_drvegrd169_medium_serialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-169 Medium serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        cube = create_radar_cube((4, 192, 12, 96), is_complex=True)
        benchmark(cube.serialize)

    def test_radar_cube_drvegrd169_medium_deserialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-169 Medium deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        cube = create_radar_cube((4, 192, 12, 96), is_complex=True)
        data = cube.serialize()
        benchmark(RadarCube.deserialize, data)

    def test_radar_cube_drvegrd169_long_serialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-169 Long serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        cube = create_radar_cube((4, 256, 12, 64), is_complex=True)
        benchmark(cube.serialize)

    def test_radar_cube_drvegrd169_long_deserialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-169 Long deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        cube = create_radar_cube((4, 256, 12, 64), is_complex=True)
        data = cube.serialize()
        benchmark(RadarCube.deserialize, data)

    def test_radar_cube_drvegrd171_short_serialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-171 Short serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        cube = create_radar_cube((4, 160, 12, 128), is_complex=True)
        benchmark(cube.serialize)

    def test_radar_cube_drvegrd171_short_deserialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-171 Short deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        cube = create_radar_cube((4, 160, 12, 128), is_complex=True)
        data = cube.serialize()
        benchmark(RadarCube.deserialize, data)

    def test_radar_cube_drvegrd171_medium_serialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-171 Medium serialization."""
        cube = create_radar_cube((4, 256, 12, 64), is_complex=True)
        benchmark(cube.serialize)

    def test_radar_cube_drvegrd171_medium_deserialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-171 Medium deserialization."""
        cube = create_radar_cube((4, 256, 12, 64), is_complex=True)
        data = cube.serialize()
        benchmark(RadarCube.deserialize, data)

    def test_radar_cube_drvegrd171_long_serialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-171 Long serialization."""
        cube = create_radar_cube((4, 512, 12, 32), is_complex=True)
        benchmark(cube.serialize)

    def test_radar_cube_drvegrd171_long_deserialize(self, benchmark):
        """Benchmark RadarCube DRVEGRD-171 Long deserialization."""
        cube = create_radar_cube((4, 512, 12, 32), is_complex=True)
        data = cube.serialize()
        benchmark(RadarCube.deserialize, data)


# ============================================================================
# BENCHMARK: PointCloud2 (Heavy)
# ============================================================================


class TestPointCloud2Benchmarks:
    """Benchmarks for PointCloud2 messages.

    Point counts typical for LiDAR/radar:
    - Sparse: 1,000 points (16 KB)
    - Medium: 10,000 points (160 KB)
    - Dense: 65,536 points (1 MB)
    - Very Dense: 131,072 points (2 MB)

    Fast mode: medium and dense only.
    """

    def test_point_cloud_1k_serialize(self, benchmark):
        """Benchmark PointCloud2 1K points serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        cloud = create_point_cloud(1_000)
        benchmark(cloud.serialize)

    def test_point_cloud_1k_deserialize(self, benchmark):
        """Benchmark PointCloud2 1K points deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        cloud = create_point_cloud(1_000)
        data = cloud.serialize()
        benchmark(PointCloud2.deserialize, data)

    def test_point_cloud_10k_serialize(self, benchmark):
        """Benchmark PointCloud2 10K points serialization."""
        cloud = create_point_cloud(10_000)
        benchmark(cloud.serialize)

    def test_point_cloud_10k_deserialize(self, benchmark):
        """Benchmark PointCloud2 10K points deserialization."""
        cloud = create_point_cloud(10_000)
        data = cloud.serialize()
        benchmark(PointCloud2.deserialize, data)

    def test_point_cloud_65k_serialize(self, benchmark):
        """Benchmark PointCloud2 65K points serialization."""
        cloud = create_point_cloud(65_536)
        benchmark(cloud.serialize)

    def test_point_cloud_65k_deserialize(self, benchmark):
        """Benchmark PointCloud2 65K points deserialization."""
        cloud = create_point_cloud(65_536)
        data = cloud.serialize()
        benchmark(PointCloud2.deserialize, data)

    def test_point_cloud_131k_serialize(self, benchmark):
        """Benchmark PointCloud2 131K points serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        cloud = create_point_cloud(131_072)
        benchmark(cloud.serialize)

    def test_point_cloud_131k_deserialize(self, benchmark):
        """Benchmark PointCloud2 131K points deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        cloud = create_point_cloud(131_072)
        data = cloud.serialize()
        benchmark(PointCloud2.deserialize, data)


# ============================================================================
# BENCHMARK: Mask (Heavy)
# ============================================================================


class TestMaskBenchmarks:
    """Benchmarks for Mask messages (uncompressed).

    Segmentation mask sizes: square resolutions with 8 or 32 classes:
    - 320x320: Common for lightweight models (MobileNet-based)
    - 640x640: Standard YOLO/detection input size
    - 1280x1280: High-resolution segmentation

    Fast mode: 640x640 8class and 1280x1280 32class only.
    """

    def test_mask_320x320_8class_serialize(self, benchmark):
        """Benchmark Mask 320x320 8 class serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        mask = create_mask(320, 320, 8)
        benchmark(mask.serialize)

    def test_mask_320x320_8class_deserialize(self, benchmark):
        """Benchmark Mask 320x320 8 class deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        mask = create_mask(320, 320, 8)
        data = mask.serialize()
        benchmark(Mask.deserialize, data)

    def test_mask_320x320_32class_serialize(self, benchmark):
        """Benchmark Mask 320x320 32 class serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        mask = create_mask(320, 320, 32)
        benchmark(mask.serialize)

    def test_mask_320x320_32class_deserialize(self, benchmark):
        """Benchmark Mask 320x320 32 class deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        mask = create_mask(320, 320, 32)
        data = mask.serialize()
        benchmark(Mask.deserialize, data)

    def test_mask_640x640_8class_serialize(self, benchmark):
        """Benchmark Mask 640x640 8 class serialization."""
        mask = create_mask(640, 640, 8)
        benchmark(mask.serialize)

    def test_mask_640x640_8class_deserialize(self, benchmark):
        """Benchmark Mask 640x640 8 class deserialization."""
        mask = create_mask(640, 640, 8)
        data = mask.serialize()
        benchmark(Mask.deserialize, data)

    def test_mask_640x640_32class_serialize(self, benchmark):
        """Benchmark Mask 640x640 32 class serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        mask = create_mask(640, 640, 32)
        benchmark(mask.serialize)

    def test_mask_640x640_32class_deserialize(self, benchmark):
        """Benchmark Mask 640x640 32 class deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        mask = create_mask(640, 640, 32)
        data = mask.serialize()
        benchmark(Mask.deserialize, data)

    def test_mask_1280x1280_8class_serialize(self, benchmark):
        """Benchmark Mask 1280x1280 8 class serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        mask = create_mask(1280, 1280, 8)
        benchmark(mask.serialize)

    def test_mask_1280x1280_8class_deserialize(self, benchmark):
        """Benchmark Mask 1280x1280 8 class deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        mask = create_mask(1280, 1280, 8)
        data = mask.serialize()
        benchmark(Mask.deserialize, data)

    def test_mask_1280x1280_32class_serialize(self, benchmark):
        """Benchmark Mask 1280x1280 32 class serialization."""
        mask = create_mask(1280, 1280, 32)
        benchmark(mask.serialize)

    def test_mask_1280x1280_32class_deserialize(self, benchmark):
        """Benchmark Mask 1280x1280 32 class deserialization."""
        mask = create_mask(1280, 1280, 32)
        data = mask.serialize()
        benchmark(Mask.deserialize, data)


# ============================================================================
# BENCHMARK: CompressedMask (Heavy) - zstd compressed segmentation masks
# ============================================================================


class TestCompressedMaskBenchmarks:
    """Benchmarks for Mask messages (zstd compressed).

    Same sizes as Mask but with zstd compression.
    Fast mode: 640x640 8class and 1280x1280 32class only.

    Note: These benchmarks require the 'zstd' package.
    """

    @pytest.fixture(autouse=True)
    def check_zstd(self):
        """Skip tests if zstd is not available."""
        try:
            import zstd  # noqa: F401
        except ImportError:
            pytest.skip("zstd package not available")

    def test_compressed_mask_320x320_8class_serialize(self, benchmark):
        """Benchmark CompressedMask 320x320 8 class serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        mask = create_compressed_mask(320, 320, 8)
        benchmark(mask.serialize)

    def test_compressed_mask_320x320_8class_deserialize(self, benchmark):
        """Benchmark CompressedMask 320x320 8 class deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        mask = create_compressed_mask(320, 320, 8)
        data = mask.serialize()
        benchmark(Mask.deserialize, data)

    def test_compressed_mask_640x640_8class_serialize(self, benchmark):
        """Benchmark CompressedMask 640x640 8 class serialization."""
        mask = create_compressed_mask(640, 640, 8)
        benchmark(mask.serialize)

    def test_compressed_mask_640x640_8class_deserialize(self, benchmark):
        """Benchmark CompressedMask 640x640 8 class deserialization."""
        mask = create_compressed_mask(640, 640, 8)
        data = mask.serialize()
        benchmark(Mask.deserialize, data)

    def test_compressed_mask_1280x1280_32class_serialize(self, benchmark):
        """Benchmark CompressedMask 1280x1280 32 class serialization."""
        mask = create_compressed_mask(1280, 1280, 32)
        benchmark(mask.serialize)

    def test_compressed_mask_1280x1280_32class_deserialize(self, benchmark):
        """Benchmark CompressedMask 1280x1280 32 class deserialization."""
        mask = create_compressed_mask(1280, 1280, 32)
        data = mask.serialize()
        benchmark(Mask.deserialize, data)


# ============================================================================
# BENCHMARK: Image (Heavy)
# ============================================================================


class TestImageBenchmarks:
    """Benchmarks for Image messages.

    Standard image resolutions with various encodings:
    - RGB8: 3 bytes per pixel
    - YUYV: 2 bytes per pixel (YUV422 packed)
    - NV12: 1.5 bytes per pixel (YUV420 semi-planar)

    Fast mode: HD resolution only with one encoding per type.
    """

    def test_image_vga_rgb8_serialize(self, benchmark):
        """Benchmark Image VGA RGB8 serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        image = create_image(640, 480, "rgb8", 3)
        benchmark(image.serialize)

    def test_image_vga_rgb8_deserialize(self, benchmark):
        """Benchmark Image VGA RGB8 deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        image = create_image(640, 480, "rgb8", 3)
        data = image.serialize()
        benchmark(Image.deserialize, data)

    def test_image_vga_yuyv_serialize(self, benchmark):
        """Benchmark Image VGA YUYV serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        image = create_image(640, 480, "yuyv", 2)
        benchmark(image.serialize)

    def test_image_vga_yuyv_deserialize(self, benchmark):
        """Benchmark Image VGA YUYV deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        image = create_image(640, 480, "yuyv", 2)
        data = image.serialize()
        benchmark(Image.deserialize, data)

    def test_image_vga_nv12_serialize(self, benchmark):
        """Benchmark Image VGA NV12 serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        # NV12: Y plane (640x480) + UV plane (640x240)
        image = create_image(640, 720, "nv12", 1)  # 480 + 240 = 720 effective height
        benchmark(image.serialize)

    def test_image_vga_nv12_deserialize(self, benchmark):
        """Benchmark Image VGA NV12 deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        image = create_image(640, 720, "nv12", 1)
        data = image.serialize()
        benchmark(Image.deserialize, data)

    def test_image_hd_rgb8_serialize(self, benchmark):
        """Benchmark Image HD RGB8 serialization."""
        image = create_image(1280, 720, "rgb8", 3)
        benchmark(image.serialize)

    def test_image_hd_rgb8_deserialize(self, benchmark):
        """Benchmark Image HD RGB8 deserialization."""
        image = create_image(1280, 720, "rgb8", 3)
        data = image.serialize()
        benchmark(Image.deserialize, data)

    def test_image_hd_yuyv_serialize(self, benchmark):
        """Benchmark Image HD YUYV serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        image = create_image(1280, 720, "yuyv", 2)
        benchmark(image.serialize)

    def test_image_hd_yuyv_deserialize(self, benchmark):
        """Benchmark Image HD YUYV deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        image = create_image(1280, 720, "yuyv", 2)
        data = image.serialize()
        benchmark(Image.deserialize, data)

    def test_image_hd_nv12_serialize(self, benchmark):
        """Benchmark Image HD NV12 serialization."""
        # NV12: Y plane (1280x720) + UV plane (1280x360)
        image = create_image(1280, 1080, "nv12", 1)  # 720 + 360 = 1080 effective height
        benchmark(image.serialize)

    def test_image_hd_nv12_deserialize(self, benchmark):
        """Benchmark Image HD NV12 deserialization."""
        image = create_image(1280, 1080, "nv12", 1)
        data = image.serialize()
        benchmark(Image.deserialize, data)

    def test_image_fhd_rgb8_serialize(self, benchmark):
        """Benchmark Image FHD RGB8 serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        image = create_image(1920, 1080, "rgb8", 3)
        benchmark(image.serialize)

    def test_image_fhd_rgb8_deserialize(self, benchmark):
        """Benchmark Image FHD RGB8 deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        image = create_image(1920, 1080, "rgb8", 3)
        data = image.serialize()
        benchmark(Image.deserialize, data)

    def test_image_fhd_yuyv_serialize(self, benchmark):
        """Benchmark Image FHD YUYV serialization."""
        image = create_image(1920, 1080, "yuyv", 2)
        benchmark(image.serialize)

    def test_image_fhd_yuyv_deserialize(self, benchmark):
        """Benchmark Image FHD YUYV deserialization."""
        image = create_image(1920, 1080, "yuyv", 2)
        data = image.serialize()
        benchmark(Image.deserialize, data)

    def test_image_fhd_nv12_serialize(self, benchmark):
        """Benchmark Image FHD NV12 serialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        # NV12: Y plane (1920x1080) + UV plane (1920x540)
        image = create_image(1920, 1620, "nv12", 1)  # 1080 + 540 = 1620 effective height
        benchmark(image.serialize)

    def test_image_fhd_nv12_deserialize(self, benchmark):
        """Benchmark Image FHD NV12 deserialization."""
        if is_fast_mode():
            pytest.skip("Skipped in fast mode")
        image = create_image(1920, 1620, "nv12", 1)
        data = image.serialize()
        benchmark(Image.deserialize, data)
