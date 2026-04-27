# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""
pycdr2 (legacy pure-Python) CDR benchmark suite.

Same fixture shapes as `bench_native.py` and `bench_cyclonedds.py`
(sourced from `shapes.py`, mirroring `benches/cpp/common.hpp`). Test names
line up across the three implementations so the renderer can pair fixtures.

The pycdr2 module retired here (`legacy/`) is the pre-pyo3 pure-Python
implementation, kept only for performance comparison.

Run: `pytest benches/python/bench_pycdr2.py --benchmark-only`
"""

import os

import pytest

from legacy.builtin_interfaces import Duration, Time
from legacy.edgefirst_msgs import (
    DmaBuffer,
    Mask,
    RadarCube,
)
from legacy.foxglove_msgs import CompressedVideo as FoxgloveCompressedVideo
from legacy.geometry_msgs import (
    Point,
    Point32,
    Pose,
    Pose2D,
    Quaternion,
    Transform,
    Twist,
    Vector3,
)
from legacy.sensor_msgs import Image, PointCloud2, PointField
from legacy.std_msgs import ColorRGBA, Header

from shapes import (
    COMPRESSED_VIDEO_VARIANTS,
    DMA_BUFFER_VARIANTS,
    IMAGE_VARIANTS,
    MASK_VARIANTS,
    POINT_CLOUD_VARIANTS,
    RADAR_CUBE_VARIANTS,
)


def is_fast_mode() -> bool:
    return os.environ.get("BENCH_FAST", "").lower() in ("1", "true")


def make_header() -> Header:
    return Header(stamp=Time(sec=1234567890, nanosec=123456789), frame_id="sensor_frame")


def make_image(v) -> Image:
    return Image(
        header=make_header(),
        height=v.height, width=v.width, encoding=v.encoding,
        is_bigendian=0, step=v.step,
        data=bytes(v.payload_bytes),
    )


def make_radar_cube(v) -> RadarCube:
    return RadarCube(
        header=make_header(),
        timestamp=1234567890123456,
        layout=[6, 1, 5, 2],
        shape=list(v.shape),
        scales=[1.0, 0.117, 1.0, 0.156],
        cube=[0] * v.cube_elements,
        is_complex=True,
    )


def make_mask(v) -> Mask:
    return Mask(
        height=v.height, width=v.width, length=1, encoding="",
        mask=bytes(v.payload_bytes),
        boxed=False,
    )


def make_point_cloud(v) -> PointCloud2:
    fields = [PointField(name=n, offset=o, datatype=d, count=c) for n, o, d, c in v.fields()]
    return PointCloud2(
        header=make_header(), height=1, width=v.num_points,
        fields=fields, is_bigendian=False,
        point_step=v.point_step, row_step=v.point_step * v.num_points,
        data=bytes(v.payload_bytes),
        is_dense=True,
    )


def make_compressed_video(v) -> FoxgloveCompressedVideo:
    return FoxgloveCompressedVideo(
        timestamp=Time(sec=1234567890, nanosec=123456789),
        frame_id="camera",
        data=bytes(v.payload_bytes),
        format="h264",
    )


def make_dmabuf(v) -> DmaBuffer:
    return DmaBuffer(
        header=make_header(),
        pid=12345, fd=7,
        width=v.width, height=v.height,
        stride=v.stride, fourcc=v.fourcc, length=v.length,
    )


# ── Heavy types ────────────────────────────────────────────────────


@pytest.mark.parametrize("v", IMAGE_VARIANTS, ids=lambda v: v.name)
def test_image_serialize(benchmark, v):
    img = make_image(v)
    benchmark(img.serialize)


@pytest.mark.parametrize("v", IMAGE_VARIANTS, ids=lambda v: v.name)
def test_image_deserialize(benchmark, v):
    buf = make_image(v).serialize()
    benchmark(Image.deserialize, buf)


@pytest.mark.parametrize("v", RADAR_CUBE_VARIANTS, ids=lambda v: v.name)
def test_radar_cube_serialize(benchmark, v):
    rc = make_radar_cube(v)
    benchmark(rc.serialize)


@pytest.mark.parametrize("v", RADAR_CUBE_VARIANTS, ids=lambda v: v.name)
def test_radar_cube_deserialize(benchmark, v):
    buf = make_radar_cube(v).serialize()
    benchmark(RadarCube.deserialize, buf)


@pytest.mark.parametrize("v", MASK_VARIANTS, ids=lambda v: v.name)
def test_mask_serialize(benchmark, v):
    m = make_mask(v)
    benchmark(m.serialize)


@pytest.mark.parametrize("v", MASK_VARIANTS, ids=lambda v: v.name)
def test_mask_deserialize(benchmark, v):
    buf = make_mask(v).serialize()
    benchmark(Mask.deserialize, buf)


@pytest.mark.parametrize("v", POINT_CLOUD_VARIANTS, ids=lambda v: v.name)
def test_point_cloud_serialize(benchmark, v):
    pc = make_point_cloud(v)
    benchmark(pc.serialize)


@pytest.mark.parametrize("v", POINT_CLOUD_VARIANTS, ids=lambda v: v.name)
def test_point_cloud_deserialize(benchmark, v):
    buf = make_point_cloud(v).serialize()
    benchmark(PointCloud2.deserialize, buf)


@pytest.mark.parametrize("v", COMPRESSED_VIDEO_VARIANTS, ids=lambda v: v.name)
def test_compressed_video_serialize(benchmark, v):
    cv = make_compressed_video(v)
    benchmark(cv.serialize)


@pytest.mark.parametrize("v", COMPRESSED_VIDEO_VARIANTS, ids=lambda v: v.name)
def test_compressed_video_deserialize(benchmark, v):
    buf = make_compressed_video(v).serialize()
    benchmark(FoxgloveCompressedVideo.deserialize, buf)


@pytest.mark.parametrize("v", DMA_BUFFER_VARIANTS, ids=lambda v: v.name)
def test_dmabuf_serialize(benchmark, v):
    d = make_dmabuf(v)
    benchmark(d.serialize)


@pytest.mark.parametrize("v", DMA_BUFFER_VARIANTS, ids=lambda v: v.name)
def test_dmabuf_deserialize(benchmark, v):
    buf = make_dmabuf(v).serialize()
    benchmark(DmaBuffer.deserialize, buf)


# ── Light types ────────────────────────────────────────────────────


def test_time_serialize(benchmark):
    benchmark(Time(sec=1234567890, nanosec=123456789).serialize)


def test_time_deserialize(benchmark):
    data = Time(sec=1234567890, nanosec=123456789).serialize()
    benchmark(Time.deserialize, data)


def test_duration_serialize(benchmark):
    benchmark(Duration(sec=60, nanosec=500_000_000).serialize)


def test_duration_deserialize(benchmark):
    data = Duration(sec=60, nanosec=500_000_000).serialize()
    benchmark(Duration.deserialize, data)


def test_header_serialize(benchmark):
    benchmark(make_header().serialize)


def test_header_deserialize(benchmark):
    data = make_header().serialize()
    benchmark(Header.deserialize, data)


def test_colorrgba_serialize(benchmark):
    benchmark(ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0).serialize)


def test_colorrgba_deserialize(benchmark):
    data = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0).serialize()
    benchmark(ColorRGBA.deserialize, data)


def test_vector3_serialize(benchmark):
    benchmark(Vector3(x=1.5, y=2.5, z=3.5).serialize)


def test_vector3_deserialize(benchmark):
    data = Vector3(x=1.5, y=2.5, z=3.5).serialize()
    benchmark(Vector3.deserialize, data)


def test_point_serialize(benchmark):
    benchmark(Point(x=10.0, y=20.0, z=30.0).serialize)


def test_point_deserialize(benchmark):
    data = Point(x=10.0, y=20.0, z=30.0).serialize()
    benchmark(Point.deserialize, data)


def test_point32_serialize(benchmark):
    benchmark(Point32(x=1.0, y=2.0, z=3.0).serialize)


def test_point32_deserialize(benchmark):
    data = Point32(x=1.0, y=2.0, z=3.0).serialize()
    benchmark(Point32.deserialize, data)


def test_quaternion_serialize(benchmark):
    benchmark(Quaternion(x=0.0, y=0.0, z=0.707, w=0.707).serialize)


def test_quaternion_deserialize(benchmark):
    data = Quaternion(x=0.0, y=0.0, z=0.707, w=0.707).serialize()
    benchmark(Quaternion.deserialize, data)


def test_pose_serialize(benchmark):
    p = Pose(position=Point(x=10.0, y=20.0, z=30.0),
             orientation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707))
    benchmark(p.serialize)


def test_pose_deserialize(benchmark):
    data = Pose(position=Point(x=10.0, y=20.0, z=30.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)).serialize()
    benchmark(Pose.deserialize, data)


def test_pose2d_serialize(benchmark):
    benchmark(Pose2D(x=10.0, y=20.0, theta=1.57).serialize)


def test_pose2d_deserialize(benchmark):
    data = Pose2D(x=10.0, y=20.0, theta=1.57).serialize()
    benchmark(Pose2D.deserialize, data)


def test_transform_serialize(benchmark):
    t = Transform(translation=Vector3(x=1.5, y=2.5, z=3.5),
                  rotation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707))
    benchmark(t.serialize)


def test_transform_deserialize(benchmark):
    data = Transform(translation=Vector3(x=1.5, y=2.5, z=3.5),
                     rotation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707)).serialize()
    benchmark(Transform.deserialize, data)


def test_twist_serialize(benchmark):
    t = Twist(linear=Vector3(x=1.5, y=2.5, z=3.5),
              angular=Vector3(x=0.0, y=0.0, z=0.5))
    benchmark(t.serialize)


def test_twist_deserialize(benchmark):
    data = Twist(linear=Vector3(x=1.5, y=2.5, z=3.5),
                 angular=Vector3(x=0.0, y=0.0, z=0.5)).serialize()
    benchmark(Twist.deserialize, data)
