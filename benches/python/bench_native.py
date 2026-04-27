# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""
Native pyo3 (`edgefirst.schemas`) CDR benchmark suite.

Fixture shapes come from `shapes.py`, which mirrors `benches/cpp/common.hpp`
exactly — every Python implementation (this file, bench_cyclonedds.py,
bench_pycdr2.py) benches the same wire shapes the C++ benches use.

Run: `pytest benches/python/bench_native.py --benchmark-only`
"""

import os

import numpy as np
import pytest

from edgefirst.schemas.builtin_interfaces import Duration, Time
from edgefirst.schemas.edgefirst_msgs import DmaBuffer, Mask, RadarCube
from edgefirst.schemas.foxglove_msgs import CompressedVideo as FoxgloveCompressedVideo
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
from edgefirst.schemas.sensor_msgs import Image, PointCloud2, PointField
from edgefirst.schemas.std_msgs import ColorRGBA, Header

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


# ── Fixture builders (one per message type) ────────────────────────


def make_image(v) -> Image:
    return Image(
        header=make_header(),
        height=v.height, width=v.width, encoding=v.encoding,
        is_bigendian=0, step=v.step,
        data=np.zeros(v.payload_bytes, dtype=np.uint8),
    )


def make_radar_cube(v) -> RadarCube:
    # Typed numpy arrays go through `.tobytes()` for portability:
    # abi3-py38 wheels don't expose Py_buffer in the limited API, so the
    # constructor accepts only byte-typed inputs there. On Py 3.11+ /
    # non-abi3 builds passing the typed arrays directly also works, but
    # the bench harness picks the portable form.
    return RadarCube(
        header=make_header(),
        timestamp=1234567890123456,
        layout=np.array([6, 1, 5, 2], dtype=np.uint8).tobytes(),
        shape=np.array(v.shape, dtype=np.uint16).tobytes(),
        scales=np.array([1.0, 0.117, 1.0, 0.156], dtype=np.float32).tobytes(),
        cube=np.zeros(v.cube_elements, dtype=np.int16).tobytes(),
        is_complex=True,
    )


def make_mask(v) -> Mask:
    return Mask(
        height=v.height, width=v.width, length=1, encoding="",
        mask=np.zeros(v.payload_bytes, dtype=np.uint8),
        boxed=False,
    )


def make_point_cloud(v) -> PointCloud2:
    fields = [PointField(name=n, offset=o, datatype=d, count=c) for n, o, d, c in v.fields()]
    return PointCloud2(
        header=make_header(), height=1, width=v.num_points,
        fields=fields, is_bigendian=False,
        point_step=v.point_step, row_step=v.point_step * v.num_points,
        data=np.zeros(v.payload_bytes, dtype=np.uint8),
        is_dense=True,
    )


def make_compressed_video(v) -> FoxgloveCompressedVideo:
    return FoxgloveCompressedVideo(
        timestamp=Time(sec=1234567890, nanosec=123456789),
        frame_id="camera",
        data=np.zeros(v.payload_bytes, dtype=np.uint8),
        format="h264",
    )


def make_dmabuf(v) -> DmaBuffer:
    return DmaBuffer(
        header=make_header(),
        pid=12345, fd=7,
        width=v.width, height=v.height,
        stride=v.stride, fourcc=v.fourcc, length=v.length,
    )


# ── Heavy types: parametrized across all variants ──────────────────


@pytest.mark.parametrize("v", IMAGE_VARIANTS, ids=lambda v: v.name)
def test_image_serialize(benchmark, v):
    img = make_image(v)
    benchmark(img.to_bytes)


@pytest.mark.parametrize("v", IMAGE_VARIANTS, ids=lambda v: v.name)
def test_image_deserialize(benchmark, v):
    buf = make_image(v).to_bytes()
    benchmark(Image.from_cdr, buf)


@pytest.mark.parametrize("v", RADAR_CUBE_VARIANTS, ids=lambda v: v.name)
def test_radar_cube_serialize(benchmark, v):
    rc = make_radar_cube(v)
    benchmark(rc.to_bytes)


@pytest.mark.parametrize("v", RADAR_CUBE_VARIANTS, ids=lambda v: v.name)
def test_radar_cube_deserialize(benchmark, v):
    buf = make_radar_cube(v).to_bytes()
    benchmark(RadarCube.from_cdr, buf)


@pytest.mark.parametrize("v", MASK_VARIANTS, ids=lambda v: v.name)
def test_mask_serialize(benchmark, v):
    m = make_mask(v)
    benchmark(m.to_bytes)


@pytest.mark.parametrize("v", MASK_VARIANTS, ids=lambda v: v.name)
def test_mask_deserialize(benchmark, v):
    buf = make_mask(v).to_bytes()
    benchmark(Mask.from_cdr, buf)


@pytest.mark.parametrize("v", POINT_CLOUD_VARIANTS, ids=lambda v: v.name)
def test_point_cloud_serialize(benchmark, v):
    pc = make_point_cloud(v)
    benchmark(pc.to_bytes)


@pytest.mark.parametrize("v", POINT_CLOUD_VARIANTS, ids=lambda v: v.name)
def test_point_cloud_deserialize(benchmark, v):
    buf = make_point_cloud(v).to_bytes()
    benchmark(PointCloud2.from_cdr, buf)


@pytest.mark.parametrize("v", COMPRESSED_VIDEO_VARIANTS, ids=lambda v: v.name)
def test_compressed_video_serialize(benchmark, v):
    cv = make_compressed_video(v)
    benchmark(cv.to_bytes)


@pytest.mark.parametrize("v", COMPRESSED_VIDEO_VARIANTS, ids=lambda v: v.name)
def test_compressed_video_deserialize(benchmark, v):
    buf = make_compressed_video(v).to_bytes()
    benchmark(FoxgloveCompressedVideo.from_cdr, buf)


@pytest.mark.parametrize("v", DMA_BUFFER_VARIANTS, ids=lambda v: v.name)
def test_dmabuf_serialize(benchmark, v):
    d = make_dmabuf(v)
    benchmark(d.to_bytes)


@pytest.mark.parametrize("v", DMA_BUFFER_VARIANTS, ids=lambda v: v.name)
def test_dmabuf_deserialize(benchmark, v):
    buf = make_dmabuf(v).to_bytes()
    benchmark(DmaBuffer.from_cdr, buf)


# ── Light CdrFixed types ──────────────────────────────────────────


def test_time_serialize(benchmark):
    t = Time(sec=1234567890, nanosec=123456789)
    benchmark(t.to_bytes)


def test_time_deserialize(benchmark):
    data = Time(sec=1234567890, nanosec=123456789).to_bytes()
    benchmark(Time.from_cdr, data)


def test_duration_serialize(benchmark):
    d = Duration(sec=60, nanosec=500_000_000)
    benchmark(d.to_bytes)


def test_duration_deserialize(benchmark):
    data = Duration(sec=60, nanosec=500_000_000).to_bytes()
    benchmark(Duration.from_cdr, data)


def test_header_serialize(benchmark):
    benchmark(make_header().to_bytes)


def test_header_deserialize(benchmark):
    data = make_header().to_bytes()
    benchmark(Header.from_cdr, data)


def test_colorrgba_serialize(benchmark):
    benchmark(ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0).to_bytes)


def test_colorrgba_deserialize(benchmark):
    data = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0).to_bytes()
    benchmark(ColorRGBA.from_cdr, data)


def test_vector3_serialize(benchmark):
    benchmark(Vector3(x=1.5, y=2.5, z=3.5).to_bytes)


def test_vector3_deserialize(benchmark):
    data = Vector3(x=1.5, y=2.5, z=3.5).to_bytes()
    benchmark(Vector3.from_cdr, data)


def test_point_serialize(benchmark):
    benchmark(Point(x=10.0, y=20.0, z=30.0).to_bytes)


def test_point_deserialize(benchmark):
    data = Point(x=10.0, y=20.0, z=30.0).to_bytes()
    benchmark(Point.from_cdr, data)


def test_point32_serialize(benchmark):
    benchmark(Point32(x=1.0, y=2.0, z=3.0).to_bytes)


def test_point32_deserialize(benchmark):
    data = Point32(x=1.0, y=2.0, z=3.0).to_bytes()
    benchmark(Point32.from_cdr, data)


def test_quaternion_serialize(benchmark):
    benchmark(Quaternion(x=0.0, y=0.0, z=0.707, w=0.707).to_bytes)


def test_quaternion_deserialize(benchmark):
    data = Quaternion(x=0.0, y=0.0, z=0.707, w=0.707).to_bytes()
    benchmark(Quaternion.from_cdr, data)


def test_pose_serialize(benchmark):
    p = Pose(position=Point(10, 20, 30), orientation=Quaternion(0, 0, 0.707, 0.707))
    benchmark(p.to_bytes)


def test_pose_deserialize(benchmark):
    data = Pose(position=Point(10, 20, 30), orientation=Quaternion(0, 0, 0.707, 0.707)).to_bytes()
    benchmark(Pose.from_cdr, data)


def test_pose2d_serialize(benchmark):
    benchmark(Pose2D(x=10.0, y=20.0, theta=1.57).to_bytes)


def test_pose2d_deserialize(benchmark):
    data = Pose2D(x=10.0, y=20.0, theta=1.57).to_bytes()
    benchmark(Pose2D.from_cdr, data)


def test_transform_serialize(benchmark):
    t = Transform(translation=Vector3(1.5, 2.5, 3.5), rotation=Quaternion(0, 0, 0.707, 0.707))
    benchmark(t.to_bytes)


def test_transform_deserialize(benchmark):
    data = Transform(translation=Vector3(1.5, 2.5, 3.5), rotation=Quaternion(0, 0, 0.707, 0.707)).to_bytes()
    benchmark(Transform.from_cdr, data)


def test_twist_serialize(benchmark):
    t = Twist(linear=Vector3(1.5, 2.5, 3.5), angular=Vector3(0, 0, 0.5))
    benchmark(t.to_bytes)


def test_twist_deserialize(benchmark):
    data = Twist(linear=Vector3(1.5, 2.5, 3.5), angular=Vector3(0, 0, 0.5)).to_bytes()
    benchmark(Twist.from_cdr, data)
