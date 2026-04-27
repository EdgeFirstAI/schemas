# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""
cyclonedds-python CDR benchmark suite.

Same fixture shapes as `bench_native.py` and `bench_pycdr2.py` (sourced
from `shapes.py`, mirroring `benches/cpp/common.hpp`). Test names line
up across the three implementations so the renderer can pair fixtures.

Install: `pip install cyclonedds`
Run: `pytest benches/python/bench_cyclonedds.py --benchmark-only`
"""

import os
from dataclasses import dataclass, field

import pytest

cyclonedds = pytest.importorskip("cyclonedds.idl")
from cyclonedds.idl import IdlStruct  # noqa: E402
from cyclonedds.idl.types import (  # noqa: E402
    float32,
    float64,
    int16,
    int32,
    sequence,
    uint8,
    uint16,
    uint32,
    uint64,
)

from shapes import (  # noqa: E402
    COMPRESSED_VIDEO_VARIANTS,
    DMA_BUFFER_VARIANTS,
    IMAGE_VARIANTS,
    MASK_VARIANTS,
    POINT_CLOUD_VARIANTS,
    RADAR_CUBE_VARIANTS,
)

cdr_bool = bool


def is_fast_mode() -> bool:
    return os.environ.get("BENCH_FAST", "").lower() in ("1", "true")


# ── IDL definitions ────────────────────────────────────────────────


@dataclass
class Time(IdlStruct, typename="builtin_interfaces/Time"):
    sec: int32 = 0
    nanosec: uint32 = 0


@dataclass
class Duration(IdlStruct, typename="builtin_interfaces/Duration"):
    sec: int32 = 0
    nanosec: uint32 = 0


@dataclass
class Header(IdlStruct, typename="std_msgs/Header"):
    stamp: Time = field(default_factory=Time)
    frame_id: str = ""


@dataclass
class ColorRGBA(IdlStruct, typename="std_msgs/ColorRGBA"):
    r: float32 = 0.0
    g: float32 = 0.0
    b: float32 = 0.0
    a: float32 = 0.0


@dataclass
class Vector3(IdlStruct, typename="geometry_msgs/Vector3"):
    x: float64 = 0.0
    y: float64 = 0.0
    z: float64 = 0.0


@dataclass
class Point(IdlStruct, typename="geometry_msgs/Point"):
    x: float64 = 0.0
    y: float64 = 0.0
    z: float64 = 0.0


@dataclass
class Point32(IdlStruct, typename="geometry_msgs/Point32"):
    x: float32 = 0.0
    y: float32 = 0.0
    z: float32 = 0.0


@dataclass
class Quaternion(IdlStruct, typename="geometry_msgs/Quaternion"):
    x: float64 = 0.0
    y: float64 = 0.0
    z: float64 = 0.0
    w: float64 = 1.0


@dataclass
class Pose(IdlStruct, typename="geometry_msgs/Pose"):
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class Pose2D(IdlStruct, typename="geometry_msgs/Pose2D"):
    x: float64 = 0.0
    y: float64 = 0.0
    theta: float64 = 0.0


@dataclass
class Transform(IdlStruct, typename="geometry_msgs/Transform"):
    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)


@dataclass
class Twist(IdlStruct, typename="geometry_msgs/Twist"):
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)


@dataclass
class DmaBuffer(IdlStruct, typename="edgefirst_msgs/DmaBuffer"):
    header: Header = field(default_factory=Header)
    pid: uint32 = 0
    fd: int32 = 0
    width: uint32 = 0
    height: uint32 = 0
    stride: uint32 = 0
    fourcc: uint32 = 0
    length: uint32 = 0


@dataclass
class CompressedVideo(IdlStruct, typename="foxglove_msgs/CompressedVideo"):
    timestamp: Time = field(default_factory=Time)
    frame_id: str = ""
    data: sequence[uint8] = field(default_factory=list)
    format: str = ""


@dataclass
class RadarCube(IdlStruct, typename="edgefirst_msgs/RadarCube"):
    header: Header = field(default_factory=Header)
    timestamp: uint64 = 0
    layout: sequence[uint8] = field(default_factory=list)
    shape: sequence[uint16] = field(default_factory=list)
    scales: sequence[float32] = field(default_factory=list)
    cube: sequence[int16] = field(default_factory=list)
    is_complex: cdr_bool = False


@dataclass
class PointFieldStruct(IdlStruct, typename="sensor_msgs/PointField"):
    name: str = ""
    offset: uint32 = 0
    datatype: uint8 = 0
    count: uint32 = 0


@dataclass
class PointCloud2(IdlStruct, typename="sensor_msgs/PointCloud2"):
    header: Header = field(default_factory=Header)
    height: uint32 = 0
    width: uint32 = 0
    fields: sequence[PointFieldStruct] = field(default_factory=list)
    is_bigendian: cdr_bool = False
    point_step: uint32 = 0
    row_step: uint32 = 0
    data: sequence[uint8] = field(default_factory=list)
    is_dense: cdr_bool = True


@dataclass
class Mask(IdlStruct, typename="edgefirst_msgs/Mask"):
    height: uint32 = 0
    width: uint32 = 0
    length: uint32 = 0
    encoding: str = ""
    mask: sequence[uint8] = field(default_factory=list)
    boxed: cdr_bool = False


@dataclass
class Image(IdlStruct, typename="sensor_msgs/Image"):
    header: Header = field(default_factory=Header)
    height: uint32 = 0
    width: uint32 = 0
    encoding: str = ""
    is_bigendian: uint8 = 0
    step: uint32 = 0
    data: sequence[uint8] = field(default_factory=list)


# ── Fixture builders ───────────────────────────────────────────────


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
    fields = [
        PointFieldStruct(name=n, offset=o, datatype=d, count=c)
        for n, o, d, c in v.fields()
    ]
    return PointCloud2(
        header=make_header(), height=1, width=v.num_points,
        fields=fields, is_bigendian=False,
        point_step=v.point_step, row_step=v.point_step * v.num_points,
        data=bytes(v.payload_bytes),
        is_dense=True,
    )


def make_compressed_video(v) -> CompressedVideo:
    return CompressedVideo(
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


# ── Heavy types: parametrized across all variants ──────────────────


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
    benchmark(CompressedVideo.deserialize, buf)


@pytest.mark.parametrize("v", DMA_BUFFER_VARIANTS, ids=lambda v: v.name)
def test_dmabuf_serialize(benchmark, v):
    d = make_dmabuf(v)
    benchmark(d.serialize)


@pytest.mark.parametrize("v", DMA_BUFFER_VARIANTS, ids=lambda v: v.name)
def test_dmabuf_deserialize(benchmark, v):
    buf = make_dmabuf(v).serialize()
    benchmark(DmaBuffer.deserialize, buf)


# ── Light CdrFixed types ──────────────────────────────────────────


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
    p = Pose(position=Point(10, 20, 30), orientation=Quaternion(0, 0, 0.707, 0.707))
    benchmark(p.serialize)


def test_pose_deserialize(benchmark):
    data = Pose(position=Point(10, 20, 30), orientation=Quaternion(0, 0, 0.707, 0.707)).serialize()
    benchmark(Pose.deserialize, data)


def test_pose2d_serialize(benchmark):
    benchmark(Pose2D(x=10.0, y=20.0, theta=1.57).serialize)


def test_pose2d_deserialize(benchmark):
    data = Pose2D(x=10.0, y=20.0, theta=1.57).serialize()
    benchmark(Pose2D.deserialize, data)


def test_transform_serialize(benchmark):
    t = Transform(translation=Vector3(1.5, 2.5, 3.5), rotation=Quaternion(0, 0, 0.707, 0.707))
    benchmark(t.serialize)


def test_transform_deserialize(benchmark):
    data = Transform(translation=Vector3(1.5, 2.5, 3.5), rotation=Quaternion(0, 0, 0.707, 0.707)).serialize()
    benchmark(Transform.deserialize, data)


def test_twist_serialize(benchmark):
    t = Twist(linear=Vector3(1.5, 2.5, 3.5), angular=Vector3(0, 0, 0.5))
    benchmark(t.serialize)


def test_twist_deserialize(benchmark):
    data = Twist(linear=Vector3(1.5, 2.5, 3.5), angular=Vector3(0, 0, 0.5)).serialize()
    benchmark(Twist.deserialize, data)
