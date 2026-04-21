#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

"""Generate golden CDR test files for cross-language validation.

Each message type is serialized with pycdr2 using hardcoded, non-trivial field
values.  The resulting .cdr files live under testdata/cdr/{namespace}/ and are
consumed by Rust integration tests in tests/cdr_golden.rs.

Usage:
    source venv/bin/activate
    python scripts/generate_cdr_testdata.py           # write fixtures
    python scripts/generate_cdr_testdata.py --verify  # compare, exit 1 on diff

`--verify` re-runs every generator in-memory and compares the result to the
on-disk fixture. A silent pycdr2 upgrade that changes padding or alignment
would otherwise overwrite the fixtures on the next `generate` invocation
with no warning; --verify is the CI guard against that drift.
"""

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from pycdr2 import IdlStruct
from pycdr2.types import float64, int16, int32, uint32

from edgefirst.schemas import (
    builtin_interfaces,
    default_field,
    edgefirst_msgs,
    foxglove_msgs,
    geometry_msgs,
    sensor_msgs,
    std_msgs,
)

# ---------------------------------------------------------------------------
# Output root
# ---------------------------------------------------------------------------
TESTDATA_CDR = Path(__file__).resolve().parent.parent / "testdata" / "cdr"

# ---------------------------------------------------------------------------
# Shared constants — must match Rust test values exactly
# ---------------------------------------------------------------------------
STAMP = builtin_interfaces.Time(sec=1234567890, nanosec=123456789)
FRAME_ID = "test_frame"


_VERIFY_MODE = False
_DIFFS: list[str] = []


def write_cdr(namespace: str, type_name: str, msg) -> None:
    """Serialize *msg* to testdata/cdr/{namespace}/{type_name}.cdr.

    In verify mode, compare against the on-disk fixture instead of writing
    and record any mismatch in `_DIFFS`.
    """
    out_dir = TESTDATA_CDR / namespace
    path = out_dir / f"{type_name}.cdr"
    fresh = msg.serialize()
    rel = path.relative_to(TESTDATA_CDR.parent.parent)
    if _VERIFY_MODE:
        if not path.exists():
            _DIFFS.append(f"MISSING  {rel}")
        else:
            disk = path.read_bytes()
            if disk != fresh:
                _DIFFS.append(
                    f"DIFFER   {rel} (disk={len(disk)}B, fresh={len(fresh)}B)"
                )
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    path.write_bytes(fresh)
    print(f"  {rel}")


# ═══════════════════════════════════════════════════════════════════════════
# rosgraph_msgs — not in the Python package, define locally
# ═══════════════════════════════════════════════════════════════════════════

@dataclass
class Clock(IdlStruct, typename="rosgraph_msgs/Clock"):
    clock: builtin_interfaces.Time = default_field(builtin_interfaces.Time)


# ═══════════════════════════════════════════════════════════════════════════
# GENERATORS
# ═══════════════════════════════════════════════════════════════════════════

def gen_builtin_interfaces():
    write_cdr("builtin_interfaces", "Time",
              builtin_interfaces.Time(sec=1234567890, nanosec=123456789))
    write_cdr("builtin_interfaces", "Duration",
              builtin_interfaces.Duration(sec=60, nanosec=500000000))


def gen_geometry_msgs():
    # CdrFixed types
    write_cdr("geometry_msgs", "Vector3",
              geometry_msgs.Vector3(x=1.5, y=-2.5, z=3.0))
    write_cdr("geometry_msgs", "Point",
              geometry_msgs.Point(x=1.5, y=-2.5, z=3.0))
    write_cdr("geometry_msgs", "Point32",
              geometry_msgs.Point32(x=1.5, y=-2.5, z=3.0))
    write_cdr("geometry_msgs", "Quaternion",
              geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

    pos = geometry_msgs.Point(x=1.5, y=-2.5, z=3.0)
    quat = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    write_cdr("geometry_msgs", "Pose",
              geometry_msgs.Pose(position=pos, orientation=quat))

    write_cdr("geometry_msgs", "Pose2D",
              geometry_msgs.Pose2D(x=1.5, y=-2.5, theta=0.785398163))

    vec_t = geometry_msgs.Vector3(x=1.5, y=-2.5, z=3.0)
    write_cdr("geometry_msgs", "Transform",
              geometry_msgs.Transform(translation=vec_t, rotation=quat))

    lin = geometry_msgs.Vector3(x=1.0, y=2.0, z=3.0)
    ang = geometry_msgs.Vector3(x=0.1, y=0.2, z=0.3)
    write_cdr("geometry_msgs", "Accel",
              geometry_msgs.Accel(linear=lin, angular=ang))
    write_cdr("geometry_msgs", "Twist",
              geometry_msgs.Twist(linear=lin, angular=ang))

    com = geometry_msgs.Vector3(x=0.1, y=0.2, z=0.3)
    write_cdr("geometry_msgs", "Inertia",
              geometry_msgs.Inertia(
                  m=10.0, com=com,
                  ixx=1.0, ixy=0.1, ixz=0.2,
                  iyy=2.0, iyz=0.3, izz=3.0))

    # Buffer-backed stamped types
    header = std_msgs.Header(stamp=STAMP, frame_id=FRAME_ID)

    accel = geometry_msgs.Accel(linear=lin, angular=ang)
    write_cdr("geometry_msgs", "AccelStamped",
              geometry_msgs.AccelStamped(header=header, accel=accel))

    twist = geometry_msgs.Twist(linear=lin, angular=ang)
    write_cdr("geometry_msgs", "TwistStamped",
              geometry_msgs.TwistStamped(header=header, twist=twist))

    inertia = geometry_msgs.Inertia(
        m=10.0, com=com, ixx=1.0, ixy=0.1, ixz=0.2,
        iyy=2.0, iyz=0.3, izz=3.0)
    write_cdr("geometry_msgs", "InertiaStamped",
              geometry_msgs.InertiaStamped(header=header, inertia=inertia))

    point = geometry_msgs.Point(x=1.5, y=-2.5, z=3.0)
    write_cdr("geometry_msgs", "PointStamped",
              geometry_msgs.PointStamped(header=header, point=point))

    tf = geometry_msgs.Transform(translation=vec_t, rotation=quat)
    write_cdr("geometry_msgs", "TransformStamped",
              geometry_msgs.TransformStamped(
                  header=header, child_frame_id="child_frame", transform=tf))


def gen_std_msgs():
    write_cdr("std_msgs", "ColorRGBA",
              std_msgs.ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0))

    write_cdr("std_msgs", "Header",
              std_msgs.Header(stamp=STAMP, frame_id=FRAME_ID))


def gen_sensor_msgs():
    # CdrFixed types
    write_cdr("sensor_msgs", "NavSatStatus",
              sensor_msgs.NavSatStatus(status=0, service=1))
    write_cdr("sensor_msgs", "RegionOfInterest",
              sensor_msgs.RegionOfInterest(
                  x_offset=10, y_offset=20, height=100, width=200,
                  do_rectify=True))

    # Buffer-backed types
    header = std_msgs.Header(stamp=STAMP, frame_id=FRAME_ID)

    # CompressedImage — small 16-byte JPEG-like payload
    write_cdr("sensor_msgs", "CompressedImage",
              sensor_msgs.CompressedImage(
                  header=header, format="jpeg",
                  data=list(range(16))))

    # Image — 4x2 RGB8 (24 bytes of pixel data)
    pixels = list(range(24))
    write_cdr("sensor_msgs", "Image",
              sensor_msgs.Image(
                  header=header, height=2, width=4, encoding="rgb8",
                  is_bigendian=0, step=12, data=pixels))

    # Imu
    quat = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    ang_vel = geometry_msgs.Vector3(x=0.01, y=0.02, z=0.03)
    lin_acc = geometry_msgs.Vector3(x=0.0, y=0.0, z=9.81)
    write_cdr("sensor_msgs", "Imu",
              sensor_msgs.Imu(
                  header=header, orientation=quat,
                  orientation_covariance=[0.0]*9,
                  angular_velocity=ang_vel,
                  angular_velocity_covariance=[0.0]*9,
                  linear_acceleration=lin_acc,
                  linear_acceleration_covariance=[0.0]*9))

    # NavSatFix
    write_cdr("sensor_msgs", "NavSatFix",
              sensor_msgs.NavSatFix(
                  header=header,
                  status=sensor_msgs.NavSatStatus(status=0, service=1),
                  latitude=45.5017, longitude=-73.5673, altitude=100.0,
                  position_covariance=[
                      1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0],
                  position_covariance_type=2))

    # PointCloud2 — 4 points with xyz float32 fields
    fields = [
        sensor_msgs.PointField(name="x", offset=0, datatype=7, count=1),
        sensor_msgs.PointField(name="y", offset=4, datatype=7, count=1),
        sensor_msgs.PointField(name="z", offset=8, datatype=7, count=1),
    ]
    import struct
    pc_data = b""
    for i in range(4):
        pc_data += struct.pack("<fff", float(i), float(i + 1), float(i + 2))
    write_cdr("sensor_msgs", "PointCloud2",
              sensor_msgs.PointCloud2(
                  header=header, height=1, width=4, fields=fields,
                  is_bigendian=False, point_step=12, row_step=48,
                  data=list(pc_data), is_dense=True))

    # CameraInfo — minimal with plumb_bob distortion
    d_coeffs = [0.1, -0.2, 0.001, 0.002, 0.0]
    k_matrix = [500.0, 0.0, 320.0,
                0.0, 500.0, 240.0,
                0.0, 0.0, 1.0]
    r_matrix = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
    p_matrix = [500.0, 0.0, 320.0, 0.0,
                0.0, 500.0, 240.0, 0.0,
                0.0, 0.0, 1.0, 0.0]
    roi = sensor_msgs.RegionOfInterest(
        x_offset=0, y_offset=0, height=480, width=640, do_rectify=False)
    write_cdr("sensor_msgs", "CameraInfo",
              sensor_msgs.CameraInfo(
                  header=header, height=480, width=640,
                  distortion_model="plumb_bob", d=d_coeffs,
                  k=k_matrix, r=r_matrix, p=p_matrix,
                  binning_x=1, binning_y=1, roi=roi))


def gen_edgefirst_msgs():
    # CdrFixed
    write_cdr("edgefirst_msgs", "Date",
              edgefirst_msgs.Date(year=2025, month=6, day=15))

    header = std_msgs.Header(stamp=STAMP, frame_id=FRAME_ID)

    # Mask — 4x2 (8 bytes)
    write_cdr("edgefirst_msgs", "Mask",
              edgefirst_msgs.Mask(
                  height=2, width=4, length=0, encoding="",
                  mask=list(range(8)), boxed=False))

    # CameraFrame — 9 variants covering raw, compressed, multi-plane, inlined.
    def _plane(fd=0, offset=0, stride=0, size=0, used=None, data=None):
        return edgefirst_msgs.CameraPlane(
            fd=fd, offset=offset, stride=stride, size=size,
            used=used if used is not None else size,
            data=data if data is not None else [])

    # Single-plane RGB8 1920x1080
    write_cdr("edgefirst_msgs", "CameraFrame",
              edgefirst_msgs.CameraFrame(
                  header=header, seq=1, pid=1234, width=1920, height=1080,
                  format="rgb8",
                  color_space="srgb", color_transfer="srgb",
                  color_encoding="", color_range="full", fence_fd=-1,
                  planes=[_plane(fd=42, stride=5760, size=6_220_800)]))

    # NV12 two-plane, shared fd
    write_cdr("edgefirst_msgs", "CameraFrame_nv12",
              edgefirst_msgs.CameraFrame(
                  header=header, seq=2, pid=1234, width=1920, height=1080,
                  format="NV12",
                  color_space="bt709", color_transfer="bt709",
                  color_encoding="bt709", color_range="limited", fence_fd=-1,
                  planes=[
                      _plane(fd=42, stride=1920, size=2_073_600),
                      _plane(fd=42, offset=2_073_600, stride=1920, size=1_036_800),
                  ]))

    # I420 three-plane, shared fd
    w, h = 1920, 1080
    y_sz = w * h
    uv_sz = (w // 2) * (h // 2)
    write_cdr("edgefirst_msgs", "CameraFrame_i420",
              edgefirst_msgs.CameraFrame(
                  header=header, seq=3, pid=1234, width=w, height=h,
                  format="I420",
                  color_space="bt709", color_transfer="bt709",
                  color_encoding="bt709", color_range="limited", fence_fd=-1,
                  planes=[
                      _plane(fd=42, stride=w, size=y_sz),
                      _plane(fd=42, offset=y_sz, stride=w // 2, size=uv_sz),
                      _plane(fd=42, offset=y_sz + uv_sz, stride=w // 2, size=uv_sz),
                  ]))

    # Planar RGB8 NCHW model input
    n = 640 * 640
    write_cdr("edgefirst_msgs", "CameraFrame_planar_nchw",
              edgefirst_msgs.CameraFrame(
                  header=header, seq=4, pid=1234, width=640, height=640,
                  format="rgb8_planar_nchw",
                  color_space="srgb", color_transfer="srgb",
                  color_encoding="", color_range="full", fence_fd=-1,
                  planes=[_plane(fd=50, offset=i * n, stride=640, size=n)
                          for i in range(3)]))

    # True V4L2 MPLANE — distinct fd per plane, plus a GPU fence
    write_cdr("edgefirst_msgs", "CameraFrame_split_fd",
              edgefirst_msgs.CameraFrame(
                  header=header, seq=5, pid=1234, width=1920, height=1080,
                  format="NV12",
                  color_space="bt709", color_transfer="bt709",
                  color_encoding="bt709", color_range="limited", fence_fd=77,
                  planes=[
                      _plane(fd=70, stride=1920, size=2_073_600),
                      _plane(fd=71, stride=1920, size=1_036_800),
                  ]))

    # H.264 bitstream — oversized buffer, used << size
    write_cdr("edgefirst_msgs", "CameraFrame_h264",
              edgefirst_msgs.CameraFrame(
                  header=header, seq=6, pid=1234, width=1920, height=1080,
                  format="h264",
                  color_space="bt709", color_transfer="bt709",
                  color_encoding="bt709", color_range="limited", fence_fd=-1,
                  planes=[_plane(fd=90, size=4_194_304, used=187_392)]))

    # Off-device bridge — inlined data, no fd, pid=0
    write_cdr("edgefirst_msgs", "CameraFrame_inlined",
              edgefirst_msgs.CameraFrame(
                  header=header, seq=7, pid=0, width=16, height=16,
                  format="rgb8",
                  color_space="srgb", color_transfer="srgb",
                  color_encoding="", color_range="full", fence_fd=-1,
                  planes=[_plane(fd=-1, stride=64, size=1024,
                                 data=[i & 0xFF for i in range(1024)])]))

    # Zero-plane metadata-only frame — exercises planes.len()==0 edge case
    # which is distinct from the populated-planes path (empty seq-count walk
    # in CDR, no scan_plane_element calls).
    write_cdr("edgefirst_msgs", "CameraFrame_empty",
              edgefirst_msgs.CameraFrame(
                  header=header, seq=8, pid=0, width=1, height=1,
                  format="", color_space="", color_transfer="",
                  color_encoding="", color_range="", fence_fd=-1,
                  planes=[]))

    # DmaBuffer
    write_cdr("edgefirst_msgs", "DmaBuffer",
              edgefirst_msgs.DmaBuffer(
                  header=header, pid=12345, fd=42,
                  width=1920, height=1080, stride=5760,
                  fourcc=0x34325247, length=1920*1080*3))

    # LocalTime
    write_cdr("edgefirst_msgs", "LocalTime",
              edgefirst_msgs.LocalTime(
                  header=header,
                  date=edgefirst_msgs.Date(year=2025, month=6, day=15),
                  time=builtin_interfaces.Time(sec=43200, nanosec=0),
                  timezone=-300))

    # RadarCube — shape [2,4,2,2] = 32 i16 values
    shape = [2, 4, 2, 2]
    total = 2 * 4 * 2 * 2
    cube_data = [i * 100 for i in range(total)]
    write_cdr("edgefirst_msgs", "RadarCube",
              edgefirst_msgs.RadarCube(
                  header=header, timestamp=1234567890123456,
                  layout=[6, 1, 5, 2],  # SEQUENCE, RANGE, RXCHANNEL, DOPPLER
                  shape=shape, scales=[1.0, 2.5, 1.0, 0.5],
                  cube=cube_data, is_complex=False))

    # RadarInfo
    write_cdr("edgefirst_msgs", "RadarInfo",
              edgefirst_msgs.RadarInfo(
                  header=header, center_frequency="77GHz",
                  frequency_sweep="wide", range_toggle="off",
                  detection_sensitivity="high", cube=True))

    # Track
    write_cdr("edgefirst_msgs", "Track",
              edgefirst_msgs.Track(
                  id="t1", lifetime=5,
                  created=builtin_interfaces.Time(sec=95, nanosec=0)))

    # DetectBox (Box)
    track = edgefirst_msgs.Track(
        id="t1", lifetime=5,
        created=builtin_interfaces.Time(sec=95, nanosec=0))
    write_cdr("edgefirst_msgs", "Box",
              edgefirst_msgs.Box(
                  center_x=0.5, center_y=0.5, width=0.1, height=0.2,
                  label="car", score=0.98, distance=10.0, speed=5.0,
                  track=track))

    # Detect
    box_msg = edgefirst_msgs.Box(
        center_x=0.5, center_y=0.5, width=0.1, height=0.2,
        label="car", score=0.98, distance=10.0, speed=5.0,
        track=track)
    write_cdr("edgefirst_msgs", "Detect",
              edgefirst_msgs.Detect(
                  header=header,
                  input_timestamp=STAMP,
                  model_time=builtin_interfaces.Time(sec=0, nanosec=1000000),
                  output_time=builtin_interfaces.Time(sec=0, nanosec=2000000),
                  boxes=[box_msg]))

    # Detect with multiple boxes (varying string lengths for alignment testing)
    boxes_multi = [
        edgefirst_msgs.Box(
            center_x=0.1, center_y=0.2, width=0.5, height=0.6,
            label="a", score=0.95, distance=5.0, speed=1.0,
            track=edgefirst_msgs.Track(
                id="t", lifetime=1,
                created=builtin_interfaces.Time(sec=1, nanosec=0))),
        edgefirst_msgs.Box(
            center_x=0.3, center_y=0.4, width=0.2, height=0.3,
            label="person", score=0.87, distance=12.0, speed=3.0,
            track=edgefirst_msgs.Track(
                id="track_long_id", lifetime=10,
                created=builtin_interfaces.Time(sec=2, nanosec=0))),
        edgefirst_msgs.Box(
            center_x=0.7, center_y=0.8, width=0.1, height=0.1,
            label="ab", score=0.50, distance=0.0, speed=0.0,
            track=edgefirst_msgs.Track(
                id="abc", lifetime=0,
                created=builtin_interfaces.Time(sec=0, nanosec=0))),
    ]
    write_cdr("edgefirst_msgs", "Detect_multi",
              edgefirst_msgs.Detect(
                  header=header,
                  input_timestamp=STAMP,
                  model_time=builtin_interfaces.Time(sec=0, nanosec=1000000),
                  output_time=builtin_interfaces.Time(sec=0, nanosec=2000000),
                  boxes=boxes_multi))

    # Model
    write_cdr("edgefirst_msgs", "Model",
              edgefirst_msgs.Model(
                  header=header,
                  input_time=builtin_interfaces.Duration(sec=0, nanosec=1000000),
                  model_time=builtin_interfaces.Duration(sec=0, nanosec=5000000),
                  output_time=builtin_interfaces.Duration(sec=0, nanosec=500000),
                  decode_time=builtin_interfaces.Duration(sec=0, nanosec=200000),
                  boxes=[box_msg],
                  mask=[edgefirst_msgs.Mask(
                      height=2, width=4, length=0, encoding="",
                      mask=list(range(8)), boxed=True)]))

    # ModelInfo
    write_cdr("edgefirst_msgs", "ModelInfo",
              edgefirst_msgs.ModelInfo(
                  header=header,
                  input_shape=[1, 3, 640, 640],
                  input_type=8,   # FLOAT32
                  output_shape=[1, 84, 8400],
                  output_type=8,  # FLOAT32
                  labels=["person", "car", "bicycle"],
                  model_type="object_detection",
                  model_format="DeepViewRT",
                  model_name="yolov8n"))

    # ModelInfo with alignment-stressing labels
    write_cdr("edgefirst_msgs", "ModelInfo_labels",
              edgefirst_msgs.ModelInfo(
                  header=header,
                  input_shape=[1, 3, 320, 320],
                  input_type=8,
                  output_shape=[1, 100, 6],
                  output_type=8,
                  labels=["a", "ab", "abc", "abcd", "abcde"],
                  model_type="object_detection",
                  model_format="DeepViewRT",
                  model_name="yolov8n"))

    # ModelInfo with empty labels array
    write_cdr("edgefirst_msgs", "ModelInfo_empty",
              edgefirst_msgs.ModelInfo(
                  header=header,
                  input_shape=[1, 3, 224, 224],
                  input_type=8,
                  output_shape=[1, 10],
                  output_type=8,
                  labels=[],
                  model_type="classifier",
                  model_format="onnx",
                  model_name="mobilenet"))


def gen_foxglove_msgs():
    # CdrFixed types
    write_cdr("foxglove_msgs", "Point2",
              foxglove_msgs.Point2(x=10.5, y=20.5))
    write_cdr("foxglove_msgs", "Color",
              foxglove_msgs.Color(r=1.0, g=0.5, b=0.0, a=1.0))

    fill = foxglove_msgs.Color(r=1.0, g=0.0, b=0.0, a=0.5)
    outline = foxglove_msgs.Color(r=0.0, g=1.0, b=0.0, a=1.0)
    pos = foxglove_msgs.Point2(x=100.0, y=200.0)
    write_cdr("foxglove_msgs", "CircleAnnotation",
              foxglove_msgs.CircleAnnotation(
                  timestamp=STAMP, position=pos, diameter=50.0,
                  thickness=2.0, fill_color=fill, outline_color=outline))

    # Buffer-backed types
    write_cdr("foxglove_msgs", "CompressedVideo",
              foxglove_msgs.CompressedVideo(
                  timestamp=STAMP, frame_id="camera",
                  data=list(range(32)), format="h264"))

    # TextAnnotation
    text_color = foxglove_msgs.Color(r=1.0, g=1.0, b=1.0, a=1.0)
    bg_color = foxglove_msgs.Color(r=0.0, g=0.0, b=0.0, a=0.5)
    text_pos = foxglove_msgs.Point2(x=50.0, y=100.0)
    write_cdr("foxglove_msgs", "TextAnnotation",
              foxglove_msgs.TextAnnotation(
                  timestamp=STAMP, position=text_pos, text="hello",
                  font_size=14.0, text_color=text_color,
                  background_color=bg_color))

    # PointsAnnotation
    pts = [foxglove_msgs.Point2(x=float(i * 10), y=float(i * 20))
           for i in range(3)]
    pt_outline = foxglove_msgs.Color(r=0.0, g=1.0, b=0.0, a=1.0)
    pt_fill = foxglove_msgs.Color(r=0.0, g=0.0, b=1.0, a=0.5)
    write_cdr("foxglove_msgs", "PointsAnnotation",
              foxglove_msgs.PointsAnnotation(
                  timestamp=STAMP, type=1,  # POINTS
                  points=pts, outline_color=pt_outline,
                  outline_colors=[], fill_color=pt_fill, thickness=3.0))

    # ImageAnnotations — one of each sub-type
    circle = foxglove_msgs.CircleAnnotation(
        timestamp=STAMP, position=pos, diameter=50.0, thickness=2.0,
        fill_color=fill, outline_color=outline)
    text_ann = foxglove_msgs.TextAnnotation(
        timestamp=STAMP, position=text_pos, text="hello",
        font_size=14.0, text_color=text_color, background_color=bg_color)
    pts_ann = foxglove_msgs.PointsAnnotation(
        timestamp=STAMP, type=1, points=pts,
        outline_color=pt_outline, outline_colors=[],
        fill_color=pt_fill, thickness=3.0)
    write_cdr("foxglove_msgs", "ImageAnnotations",
              foxglove_msgs.ImageAnnotations(
                  circles=[circle], points=[pts_ann], texts=[text_ann]))


def gen_rosgraph_msgs():
    write_cdr("rosgraph_msgs", "Clock",
              Clock(clock=builtin_interfaces.Time(sec=1234567890, nanosec=123456789)))


# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    global _VERIFY_MODE
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--verify", action="store_true",
        help="Re-encode in memory and compare to on-disk fixtures; "
             "exit 1 on any mismatch without modifying files.",
    )
    args = parser.parse_args()
    _VERIFY_MODE = args.verify

    if _VERIFY_MODE:
        print("Verifying golden CDR fixtures against fresh encodings …")
    else:
        print("Generating golden CDR test files …")
    gen_builtin_interfaces()
    gen_geometry_msgs()
    gen_std_msgs()
    gen_sensor_msgs()
    gen_edgefirst_msgs()
    gen_foxglove_msgs()
    gen_rosgraph_msgs()

    if _VERIFY_MODE:
        if _DIFFS:
            print(f"\n{len(_DIFFS)} fixture(s) out of date:", file=sys.stderr)
            for d in _DIFFS:
                print(f"  {d}", file=sys.stderr)
            print(
                "\nRun `python scripts/generate_cdr_testdata.py` without "
                "--verify to regenerate, then review the diffs in git.",
                file=sys.stderr,
            )
            sys.exit(1)
        print("All fixtures match.")
    else:
        print("Done.")


if __name__ == "__main__":
    main()
