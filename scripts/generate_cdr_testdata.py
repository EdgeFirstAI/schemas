#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

"""Generate golden CDR test files for cross-language validation.

Each message type is serialized with pycdr2 using hardcoded, non-trivial field
values.  The resulting .cdr files live under testdata/cdr/{namespace}/ and are
consumed by Rust integration tests in crates/schemas/tests/cdr_golden.rs.

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
from typing import List

# Legacy pycdr2 message definitions used for new types not yet in the
# edgefirst-schemas Python extension (nav_msgs: GridCells, MapMetaData,
# OccupancyGrid, Path; sensor_msgs: RelativeHumidity, TimeReference).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "benches" / "python"))
from legacy import builtin_interfaces as _legacy_builtin_interfaces  # noqa: E402
from legacy import nav_msgs as _legacy_nav_msgs  # noqa: E402
from legacy import sensor_msgs as _legacy_sensor_msgs  # noqa: E402
from legacy import geometry_msgs as _legacy_geometry_msgs  # noqa: E402
from legacy import edgefirst_msgs as _legacy_edgefirst_msgs  # noqa: E402
from legacy import default_field  # noqa: E402  (removed from edgefirst.schemas in 3.2.0)

from pycdr2 import IdlStruct
from pycdr2.types import float64, int16, int32, uint32

from edgefirst.schemas import (
    builtin_interfaces,
    edgefirst_msgs,
    foxglove_msgs,
    geometry_msgs,
    mavros_msgs,
    nav_msgs,
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
_DIFFS: List[str] = []


def write_cdr(namespace: str, type_name: str, msg) -> None:
    """Serialize *msg* to testdata/cdr/{namespace}/{type_name}.cdr.

    In verify mode, compare against the on-disk fixture instead of writing
    and record any mismatch in `_DIFFS`.

    Accepts both pycdr2 objects (``msg.serialize()``) and edgefirst.schemas
    PyO3 objects (``msg.to_bytes()``), which replaced pycdr2 in release 3.2.0.
    """
    out_dir = TESTDATA_CDR / namespace
    path = out_dir / f"{type_name}.cdr"
    # pycdr2 objects expose .serialize(); PyO3 objects expose .to_bytes()
    fresh = msg.serialize() if hasattr(msg, "serialize") else msg.to_bytes()
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
    clock: _legacy_builtin_interfaces.Time = default_field(_legacy_builtin_interfaces.Time)


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

    # PoseWithCovariance — Pose + 36 covariance slots; non-zero diag + off-diag
    pose_cov = [0.0] * 36
    for i in range(6):
        pose_cov[i * 6 + i] = 0.1 * (i + 1)  # diagonal 0.1..0.6
    pose_cov[1] = 0.01   # pose cov(x,y) cross-term
    pose_cov[6] = 0.01
    write_cdr("geometry_msgs", "PoseWithCovariance",
              geometry_msgs.PoseWithCovariance(
                  pose=geometry_msgs.Pose(position=pos, orientation=quat),
                  covariance=pose_cov))

    twist_cov = [0.0] * 36
    for i in range(6):
        twist_cov[i * 6 + i] = 0.02 * (i + 1)
    twist_cov[7] = 0.001
    write_cdr("geometry_msgs", "TwistWithCovariance",
              geometry_msgs.TwistWithCovariance(
                  twist=geometry_msgs.Twist(linear=lin, angular=ang),
                  covariance=twist_cov))

    write_cdr("geometry_msgs", "Vector3Stamped",
              geometry_msgs.Vector3Stamped(header=header, vector=lin))
    write_cdr("geometry_msgs", "PoseStamped",
              geometry_msgs.PoseStamped(
                  header=header,
                  pose=geometry_msgs.Pose(position=pos, orientation=quat)))
    write_cdr("geometry_msgs", "QuaternionStamped",
              geometry_msgs.QuaternionStamped(header=header, quaternion=quat))
    write_cdr("geometry_msgs", "WrenchStamped",
              geometry_msgs.WrenchStamped(
                  header=header,
                  wrench=geometry_msgs.Wrench(force=lin, torque=ang)))
    write_cdr("geometry_msgs", "PoseWithCovarianceStamped",
              geometry_msgs.PoseWithCovarianceStamped(
                  header=header,
                  pose_with_covariance=geometry_msgs.PoseWithCovariance(
                      pose=geometry_msgs.Pose(position=pos, orientation=quat),
                      covariance=pose_cov)))
    write_cdr("geometry_msgs", "TwistWithCovarianceStamped",
              geometry_msgs.TwistWithCovarianceStamped(
                  header=header,
                  twist_with_covariance=geometry_msgs.TwistWithCovariance(
                      twist=geometry_msgs.Twist(linear=lin, angular=ang),
                      covariance=twist_cov)))
    accel_cov = list(pose_cov)
    write_cdr("geometry_msgs", "AccelWithCovarianceStamped",
              geometry_msgs.AccelWithCovarianceStamped(
                  header=header,
                  accel_with_covariance=geometry_msgs.AccelWithCovariance(
                      accel=geometry_msgs.Accel(linear=lin, angular=ang),
                      covariance=accel_cov)))

    poly_pts = [
        geometry_msgs.Point32(x=1.0, y=2.0, z=3.0),
        geometry_msgs.Point32(x=4.0, y=5.0, z=6.0),
        geometry_msgs.Point32(x=7.0, y=8.0, z=9.0),
    ]
    write_cdr("geometry_msgs", "Polygon",
              geometry_msgs.Polygon(points=poly_pts))
    write_cdr("geometry_msgs", "PolygonStamped",
              geometry_msgs.PolygonStamped(header=header, points=poly_pts))
    write_cdr("geometry_msgs", "PoseArray",
              geometry_msgs.PoseArray(
                  header=header,
                  poses=[
                      geometry_msgs.Pose(position=pos, orientation=quat),
                      geometry_msgs.Pose(
                          position=geometry_msgs.Point(x=10.0, y=20.0, z=30.0),
                          orientation=quat),
                  ]))


def gen_mavros_msgs():
    header = std_msgs.Header(stamp=STAMP, frame_id=FRAME_ID)
    write_cdr("mavros_msgs", "Altitude",
              mavros_msgs.Altitude(
                  header=header, monotonic=100.0, amsl=50.0, local=10.0,
                  relative=5.0, terrain=2.0, bottom_clearance=1.5))
    write_cdr("mavros_msgs", "VfrHud",
              mavros_msgs.VfrHud(
                  header=header, airspeed=12.5, groundspeed=11.0, heading=90,
                  throttle=0.75, altitude=120.0, climb=0.5))
    write_cdr("mavros_msgs", "EstimatorStatus",
              mavros_msgs.EstimatorStatus(
                  header=header,
                  attitude_status_flag=True,
                  velocity_horiz_status_flag=True,
                  velocity_vert_status_flag=True,
                  pos_horiz_rel_status_flag=True,
                  pos_horiz_abs_status_flag=False,
                  pos_vert_abs_status_flag=True,
                  pos_vert_agl_status_flag=False,
                  const_pos_mode_status_flag=False,
                  pred_pos_horiz_rel_status_flag=True,
                  pred_pos_horiz_abs_status_flag=False,
                  gps_glitch_status_flag=False,
                  accel_error_status_flag=False))
    write_cdr("mavros_msgs", "ExtendedState",
              mavros_msgs.ExtendedState(
                  header=header, vtol_state=3, landed_state=2))
    write_cdr("mavros_msgs", "SysStatus",
              mavros_msgs.SysStatus(
                  header=header, sensors_present=0xFFFF, sensors_enabled=0x00FF,
                  sensors_health=0x00FE, load=500, voltage_battery=12600,
                  current_battery=-150, battery_remaining=85, drop_rate_comm=12,
                  errors_comm=3, errors_count1=1, errors_count2=2,
                  errors_count3=3, errors_count4=4))
    write_cdr("mavros_msgs", "State",
              mavros_msgs.State(
                  header=header, connected=True, armed=True, guided=True,
                  manual_input=False, mode="OFFBOARD", system_status=4))
    write_cdr("mavros_msgs", "StatusText",
              mavros_msgs.StatusText(
                  header=header, severity=4,
                  text="prearm: compass not calibrated"))
    write_cdr("mavros_msgs", "GpsRaw",
              mavros_msgs.GpsRaw(
                  header=header, fix_type=3, lat=473762200, lon=85453900,
                  alt=488000, eph=120, epv=180, vel=250, cog=9000,
                  satellites_visible=12, alt_ellipsoid=500000, h_acc=150,
                  v_acc=200, vel_acc=50, hdg_acc=100, yaw=9000,
                  dgps_numch=8, dgps_age=120))
    write_cdr("mavros_msgs", "TimesyncStatus",
              mavros_msgs.TimesyncStatus(
                  header=header, remote_timestamp_ns=1234567890123,
                  observed_offset_ns=-1500, estimated_offset_ns=-1200,
                  round_trip_time_ms=2.5))


def gen_nav_msgs():
    header = std_msgs.Header(stamp=STAMP, frame_id=FRAME_ID)
    pos = geometry_msgs.Point(x=1.5, y=-2.5, z=3.0)
    quat = geometry_msgs.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    lin = geometry_msgs.Vector3(x=1.0, y=2.0, z=3.0)
    ang = geometry_msgs.Vector3(x=0.1, y=0.2, z=0.3)
    pose_cov = [0.0] * 36
    twist_cov = [0.0] * 36
    for i in range(6):
        pose_cov[i * 6 + i] = 0.1 * (i + 1)
        twist_cov[i * 6 + i] = 0.02 * (i + 1)
    pose_cov[1] = 0.01
    pose_cov[6] = 0.01
    twist_cov[7] = 0.001
    write_cdr("nav_msgs", "Odometry",
              nav_msgs.Odometry(
                  header=header,
                  child_frame_id="base_link",
                  pose=geometry_msgs.PoseWithCovariance(
                      pose=geometry_msgs.Pose(position=pos, orientation=quat),
                      covariance=pose_cov),
                  twist=geometry_msgs.TwistWithCovariance(
                      twist=geometry_msgs.Twist(linear=lin, angular=ang),
                      covariance=twist_cov)))

    # Legacy types (GridCells, MapMetaData, OccupancyGrid, Path) ─────────
    _lg = _legacy_nav_msgs
    _lg_geo = _legacy_geometry_msgs

    _lg_stamp = _lg.Time(sec=STAMP.sec, nanosec=STAMP.nanosec)
    _lg_header = _lg.Header(stamp=_lg_stamp, frame_id=FRAME_ID)

    # MapMetaData — standalone fixed-size encoding
    _origin = _lg_geo.Pose(
        position=_lg_geo.Point(x=-5.0, y=-5.0, z=0.0),
        orientation=_lg_geo.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    write_cdr("nav_msgs", "MapMetaData",
              _lg.MapMetaData(
                  map_load_time=_lg_stamp,
                  resolution=0.05,
                  width=200,
                  height=200,
                  origin=_origin))

    # GridCells — 3 cells
    _cells = [
        _lg_geo.Point(x=1.0, y=2.0, z=0.0),
        _lg_geo.Point(x=3.0, y=4.0, z=0.0),
        _lg_geo.Point(x=5.0, y=6.0, z=0.0),
    ]
    write_cdr("nav_msgs", "GridCells",
              _lg.GridCells(
                  header=_lg_header,
                  cell_width=0.5,
                  cell_height=0.5,
                  cells=_cells))

    # OccupancyGrid — 4×2 grid with non-trivial values
    _info = _lg.MapMetaData(
        map_load_time=_lg_stamp,
        resolution=0.1,
        width=4,
        height=2,
        origin=_lg_geo.Pose(
            position=_lg_geo.Point(x=0.0, y=0.0, z=0.0),
            orientation=_lg_geo.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))
    write_cdr("nav_msgs", "OccupancyGrid",
              _lg.OccupancyGrid(
                  header=_lg_header,
                  info=_info,
                  data=[0, 50, 100, -1, 25, 75, 0, 0]))

    # Path — 3 poses with different frame_ids
    _pose_a = _lg_geo.Pose(
        position=_lg_geo.Point(x=1.0, y=0.0, z=0.0),
        orientation=_lg_geo.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    _pose_b = _lg_geo.Pose(
        position=_lg_geo.Point(x=2.0, y=1.0, z=0.0),
        orientation=_lg_geo.Quaternion(x=0.0, y=0.0, z=0.707, w=0.707))
    _pose_c = _lg_geo.Pose(
        position=_lg_geo.Point(x=3.0, y=2.0, z=0.0),
        orientation=_lg_geo.Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
    write_cdr("nav_msgs", "Path",
              _lg.Path(
                  header=_lg_header,
                  poses=[
                      _lg_geo.PoseStamped(
                          header=_lg.Header(
                              stamp=_lg.Time(sec=1, nanosec=0),
                              frame_id="map"),
                          pose=_pose_a),
                      _lg_geo.PoseStamped(
                          header=_lg.Header(
                              stamp=_lg.Time(sec=2, nanosec=0),
                              frame_id="map"),
                          pose=_pose_b),
                      _lg_geo.PoseStamped(
                          header=_lg.Header(
                              stamp=_lg.Time(sec=3, nanosec=0),
                              frame_id="map"),
                          pose=_pose_c),
                  ]))


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
                  data=bytes(range(16))))

    # Image — 4x2 RGB8 (24 bytes of pixel data)
    pixels = bytes(range(24))
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
                  data=pc_data, is_dense=True))

    # MagneticField — Earth-field-magnitude sample
    write_cdr("sensor_msgs", "MagneticField",
              sensor_msgs.MagneticField(
                  header=header,
                  magnetic_field=geometry_msgs.Vector3(
                      x=2.5e-5, y=-1.2e-5, z=4.1e-5),
                  magnetic_field_covariance=[
                      1e-10, 0.0, 0.0,
                      0.0, 1e-10, 0.0,
                      0.0, 0.0, 1e-10]))

    # FluidPressure — standard atmospheric pressure in Pa
    write_cdr("sensor_msgs", "FluidPressure",
              sensor_msgs.FluidPressure(
                  header=header, fluid_pressure=101325.0, variance=25.0))

    # Temperature — 22.5 C with 0.01 C^2 variance
    write_cdr("sensor_msgs", "Temperature",
              sensor_msgs.Temperature(
                  header=header, temperature=22.5, variance=0.01))

    # BatteryState — 3S1P LiPo pack with full metadata
    write_cdr("sensor_msgs", "BatteryState",
              sensor_msgs.BatteryState(
                  header=header,
                  voltage=12.34, temperature=27.5, current=-2.1,
                  charge=4.2, capacity=5.0, design_capacity=5.0,
                  percentage=0.84,
                  power_supply_status=2,       # DISCHARGING
                  power_supply_health=1,       # GOOD
                  power_supply_technology=3,   # LIPO
                  present=True,
                  cell_voltage=[4.11, 4.12, 4.10],
                  cell_temperature=[27.1, 27.3, 27.4],
                  location="battery0",
                  serial_number="SN0123456"))

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

    # Legacy types (RelativeHumidity, TimeReference) ─────────────────────
    _ls = _legacy_sensor_msgs
    _lg_stamp = _ls.Time(sec=STAMP.sec, nanosec=STAMP.nanosec)
    _lg_header = _ls.Header(stamp=_lg_stamp, frame_id=FRAME_ID)

    # RelativeHumidity — 65 % RH with 0.001 variance
    write_cdr("sensor_msgs", "RelativeHumidity",
              _ls.RelativeHumidity(
                  header=_lg_header,
                  relative_humidity=0.65,
                  variance=0.001))

    # TimeReference — GPS UTC source
    write_cdr("sensor_msgs", "TimeReference",
              _ls.TimeReference(
                  header=_lg_header,
                  time_ref=_ls.Time(sec=1234567890, nanosec=987654321),
                  source="GPS_UTC"))


def gen_edgefirst_msgs():
    # CdrFixed
    write_cdr("edgefirst_msgs", "Date",
              edgefirst_msgs.Date(year=2025, month=6, day=15))

    header = std_msgs.Header(stamp=STAMP, frame_id=FRAME_ID)

    # Mask — 4x2 (8 bytes)
    write_cdr("edgefirst_msgs", "Mask",
              edgefirst_msgs.Mask(
                  height=2, width=4, length=0, encoding="",
                  mask=bytes(range(8)), boxed=False))

    # LocalTime
    write_cdr("edgefirst_msgs", "LocalTime",
              edgefirst_msgs.LocalTime(
                  header=header,
                  date=edgefirst_msgs.Date(year=2025, month=6, day=15),
                  time=builtin_interfaces.Time(sec=43200, nanosec=0),
                  timezone=-300))

    # RadarCube — shape [2,4,2,2] = 32 i16 values
    # Use legacy (pycdr2) type so that typed-array sequence encoding
    # (sequence<uint16>, sequence<float32>, sequence<int16>) is byte-identical
    # to the on-disk golden fixture, which was generated by pycdr2.
    _le = _legacy_edgefirst_msgs
    _le_stamp = _le.Time(sec=STAMP.sec, nanosec=STAMP.nanosec)
    _le_header = _le.Header(stamp=_le_stamp, frame_id=FRAME_ID)
    shape = [2, 4, 2, 2]
    total = 2 * 4 * 2 * 2
    cube_data = [i * 100 for i in range(total)]
    write_cdr("edgefirst_msgs", "RadarCube",
              _le.RadarCube(
                  header=_le_header, timestamp=1234567890123456,
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

    # DetectBox (Box) — edgefirst_msgs.Box is a value type (no to_bytes()); use
    # the legacy pycdr2 type for the standalone CDR golden file.
    _le = _legacy_edgefirst_msgs
    _le_track = _le.Track(
        id="t1", lifetime=5,
        created=_le.Time(sec=95, nanosec=0))
    write_cdr("edgefirst_msgs", "Box",
              _le.Box(
                  center_x=0.5, center_y=0.5, width=0.1, height=0.2,
                  label="car", score=0.98, distance=10.0, speed=5.0,
                  track=_le_track))

    # Detect — PyO3 Box (value type) is compatible with the legacy CDR encoding;
    # verified byte-identical against the pycdr2 golden in tests.
    box_msg = edgefirst_msgs.Box(
        center_x=0.5, center_y=0.5, width=0.1, height=0.2,
        label="car", score=0.98, distance=10.0, speed=5.0,
        track_id="t1", track_lifetime=5,
        track_created=builtin_interfaces.Time(sec=95, nanosec=0))
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
            track_id="t", track_lifetime=1,
            track_created=builtin_interfaces.Time(sec=1, nanosec=0)),
        edgefirst_msgs.Box(
            center_x=0.3, center_y=0.4, width=0.2, height=0.3,
            label="person", score=0.87, distance=12.0, speed=3.0,
            track_id="track_long_id", track_lifetime=10,
            track_created=builtin_interfaces.Time(sec=2, nanosec=0)),
        edgefirst_msgs.Box(
            center_x=0.7, center_y=0.8, width=0.1, height=0.1,
            label="ab", score=0.50, distance=0.0, speed=0.0,
            track_id="abc", track_lifetime=0,
            track_created=builtin_interfaces.Time(sec=0, nanosec=0)),
    ]
    write_cdr("edgefirst_msgs", "Detect_multi",
              edgefirst_msgs.Detect(
                  header=header,
                  input_timestamp=STAMP,
                  model_time=builtin_interfaces.Time(sec=0, nanosec=1000000),
                  output_time=builtin_interfaces.Time(sec=0, nanosec=2000000),
                  boxes=boxes_multi))

    # Model — uses MaskBox (value type) in masks=[]; Mask (standalone) differs.
    write_cdr("edgefirst_msgs", "Model",
              edgefirst_msgs.Model(
                  header=header,
                  input_time=builtin_interfaces.Duration(sec=0, nanosec=1000000),
                  model_time=builtin_interfaces.Duration(sec=0, nanosec=5000000),
                  output_time=builtin_interfaces.Duration(sec=0, nanosec=500000),
                  decode_time=builtin_interfaces.Duration(sec=0, nanosec=200000),
                  boxes=[box_msg],
                  masks=[edgefirst_msgs.MaskBox(
                      height=2, width=4, length=0, encoding="",
                      mask=bytes(range(8)), boxed=True)]))

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

    # Vibration — MAVLink-style RMS in m/s^2 with 3 clipping counters
    write_cdr("edgefirst_msgs", "Vibration",
              edgefirst_msgs.Vibration(
                  header=header,
                  measurement_type=1,  # MEASUREMENT_RMS
                  unit=1,              # UNIT_ACCEL_M_PER_S2
                  band_lower_hz=10.0,
                  band_upper_hz=1000.0,
                  vibration=geometry_msgs.Vector3(x=0.42, y=0.51, z=0.37),
                  clipping=[3, 1, 0]))

    # ── Tensor family ────────────────────────────────────────────────────
    #
    # Generated from the legacy pycdr2 dataclasses, NOT from the PyO3
    # bindings. That is the whole point: goldens produced by the same code
    # they validate would only prove self-consistency. pycdr2 is an
    # independent implementation of the same .msg contract, so a mismatch
    # means one of the two encoders is wrong.
    _le = _legacy_edgefirst_msgs
    _le_hdr = _le.Header(
        stamp=_le.Time(sec=STAMP.sec, nanosec=STAMP.nanosec), frame_id=FRAME_ID)

    # Referenced NV12: one allocation, Y at 0 and UV at w*h, unquantized.
    # shape is [h, w] with a U8 dtype against an h*w*3/2 allocation — the
    # addressing grid, deliberately not the byte layout.
    nv12_planes = [
        _le.TensorPlane(handle=7, offset=0, stride=640,
                        size=640 * 480, used=640 * 480, modifier=0,
                        handle_bytes=[0xDE, 0xAD, 0xBE, 0xEF], data=[]),
        _le.TensorPlane(handle=7, offset=640 * 480, stride=640,
                        size=640 * 480 // 2, used=640 * 480 // 2, modifier=0,
                        handle_bytes=[], data=[]),
    ]
    nv12 = _le.Tensor(
        storage_kind=2, pid=4242, fence_fd=-1, dtype=1, quant_axis=-2,
        shape=[480, 640], strides=[640, 1],
        quant_scales=[], quant_zero_points=[],
        format="NV12", color_space="bt709", color_transfer="bt709",
        color_encoding="bt709", color_range="limited",
        planes=nv12_planes)
    write_cdr("edgefirst_msgs", "Tensor", nv12)

    # Inline single plane — the off-device bridging path, where bytes travel
    # in the message instead of behind a handle.
    write_cdr("edgefirst_msgs", "Tensor_inline",
              _le.Tensor(
                  storage_kind=0, pid=0, fence_fd=-1, dtype=1, quant_axis=-2,
                  shape=[2, 4], strides=[4, 1],
                  quant_scales=[], quant_zero_points=[],
                  format="mono8", color_space="", color_transfer="",
                  color_encoding="", color_range="",
                  planes=[_le.TensorPlane(
                      handle=-1, offset=0, stride=4, size=8, used=8,
                      modifier=0, handle_bytes=[], data=list(range(8)))]))

    # Per-axis quantization: exactly shape[quant_axis] scales.
    write_cdr("edgefirst_msgs", "Tensor_quantized",
              _le.Tensor(
                  storage_kind=0, pid=0, fence_fd=-1, dtype=3, quant_axis=0,
                  shape=[3, 8], strides=[],
                  quant_scales=[0.5, 0.25, 0.125],
                  quant_zero_points=[128, 0, -128],
                  format="", color_space="", color_transfer="",
                  color_encoding="", color_range="",
                  planes=[]))

    # Both wrappers, same tensor and same header — the encoded bytes must be
    # identical to each other, which crates/schemas/tests/cdr_golden.rs asserts.
    write_cdr("edgefirst_msgs", "TensorStamped",
              _le.TensorStamped(header=_le_hdr, seq=99, tensor=nv12))
    write_cdr("edgefirst_msgs", "CameraFrame",
              _le.CameraFrame(header=_le_hdr, seq=99, tensor=nv12))

    # A long frame_id shifts every header byte but must NOT shift the embedded
    # tensor: `seq` (uint64) forces it to the same 8-aligned offset either way.
    write_cdr("edgefirst_msgs", "CameraFrame_long_frame_id",
              _le.CameraFrame(
                  header=_le.Header(
                      stamp=_le.Time(sec=STAMP.sec, nanosec=STAMP.nanosec),
                      frame_id="a_very_long_frame_identifier_x"),
                  seq=99, tensor=nv12))

    # I420 three-plane, shared handle (legacy CameraFrame_i420 scenario).
    w, h = 1920, 1080
    y_sz = w * h
    uv_sz = (w // 2) * (h // 2)
    i420_planes = [
        _le.TensorPlane(handle=42, offset=0, stride=w,
                        size=y_sz, used=y_sz, modifier=0,
                        handle_bytes=[], data=[]),
        _le.TensorPlane(handle=42, offset=y_sz, stride=w // 2,
                        size=uv_sz, used=uv_sz, modifier=0,
                        handle_bytes=[], data=[]),
        _le.TensorPlane(handle=42, offset=y_sz + uv_sz, stride=w // 2,
                        size=uv_sz, used=uv_sz, modifier=0,
                        handle_bytes=[], data=[]),
    ]
    i420 = _le.Tensor(
        storage_kind=2, pid=1234, fence_fd=-1, dtype=1, quant_axis=-2,
        shape=[h, w], strides=[w, 1],
        quant_scales=[], quant_zero_points=[],
        format="I420", color_space="bt709", color_transfer="bt709",
        color_encoding="bt709", color_range="limited",
        planes=i420_planes)
    write_cdr("edgefirst_msgs", "Tensor_i420", i420)
    write_cdr("edgefirst_msgs", "CameraFrame_i420",
              _le.CameraFrame(header=_le_hdr, seq=3, tensor=i420))

    # Split-fd MPLANE — distinct handle per plane plus a GPU fence.
    split_planes = [
        _le.TensorPlane(handle=70, offset=0, stride=1920,
                        size=2_073_600, used=2_073_600, modifier=0,
                        handle_bytes=[], data=[]),
        _le.TensorPlane(handle=71, offset=0, stride=1920,
                        size=1_036_800, used=1_036_800, modifier=0,
                        handle_bytes=[], data=[]),
    ]
    split_nv12 = _le.Tensor(
        storage_kind=2, pid=1234, fence_fd=77, dtype=1, quant_axis=-2,
        shape=[1080, 1920], strides=[1920, 1],
        quant_scales=[], quant_zero_points=[],
        format="NV12", color_space="bt709", color_transfer="bt709",
        color_encoding="bt709", color_range="limited",
        planes=split_planes)
    write_cdr("edgefirst_msgs", "Tensor_split_fd", split_nv12)
    write_cdr("edgefirst_msgs", "CameraFrame_split_fd",
              _le.CameraFrame(header=_le_hdr, seq=5, tensor=split_nv12))

    # H264 bitstream — oversized buffer where used << size.
    h264_planes = [
        _le.TensorPlane(handle=90, offset=0, stride=0,
                        size=4_194_304, used=187_392, modifier=0,
                        handle_bytes=[], data=[]),
    ]
    h264 = _le.Tensor(
        storage_kind=2, pid=1234, fence_fd=-1, dtype=1, quant_axis=-2,
        shape=[1080, 1920], strides=[1920, 1],
        quant_scales=[], quant_zero_points=[],
        format="h264", color_space="bt709", color_transfer="bt709",
        color_encoding="bt709", color_range="limited",
        planes=h264_planes)
    write_cdr("edgefirst_msgs", "Tensor_h264", h264)
    write_cdr("edgefirst_msgs", "CameraFrame_h264",
              _le.CameraFrame(header=_le_hdr, seq=6, tensor=h264))

    # Metadata-only frame — zero planes exercises the empty planes path.
    empty_tensor = _le.Tensor(
        storage_kind=2, pid=0, fence_fd=-1, dtype=1, quant_axis=-2,
        shape=[1, 1], strides=[1, 1],
        quant_scales=[], quant_zero_points=[],
        format="", color_space="", color_transfer="",
        color_encoding="", color_range="",
        planes=[])
    write_cdr("edgefirst_msgs", "Tensor_empty", empty_tensor)
    write_cdr("edgefirst_msgs", "CameraFrame_empty",
              _le.CameraFrame(header=_le_hdr, seq=8, tensor=empty_tensor))


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
                  data=bytes(range(32)), format="h264"))

    # CompressedImage — wire-identical layout to CompressedVideo; the format
    # carries an image media type instead of a video codec.
    write_cdr("foxglove_msgs", "CompressedImage",
              foxglove_msgs.CompressedImage(
                  timestamp=STAMP, frame_id="camera",
                  data=bytes(range(32)), format="jpeg"))

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
                  timestamp=STAMP, type_=1,  # POINTS
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
        timestamp=STAMP, type_=1, points=pts,
        outline_color=pt_outline, outline_colors=[],
        fill_color=pt_fill, thickness=3.0)
    write_cdr("foxglove_msgs", "ImageAnnotations",
              foxglove_msgs.ImageAnnotations(
                  circles=[circle], points=[pts_ann], texts=[text_ann]))


def gen_rosgraph_msgs():
    write_cdr("rosgraph_msgs", "Clock",
              Clock(clock=_legacy_builtin_interfaces.Time(sec=1234567890, nanosec=123456789)))


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
    gen_mavros_msgs()
    gen_nav_msgs()
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
