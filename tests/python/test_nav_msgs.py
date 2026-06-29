# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for `edgefirst.schemas.nav_msgs`."""

import pathlib
import struct

import pytest

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.geometry_msgs import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from edgefirst.schemas.nav_msgs import GridCells, MapMetaData, OccupancyGrid, Odometry, Path
from edgefirst.schemas.std_msgs import Header

_CDR_DIR = pathlib.Path(__file__).parent.parent.parent / "testdata" / "cdr"


def _load(ns: str, msg: str) -> bytes:
    return (_CDR_DIR / ns / f"{msg}.cdr").read_bytes()


def _build_path_cdr(header_stamp, header_frame, elements):
    """Hand-encode a ``nav_msgs/Path`` CDR buffer (XCDR1, little-endian).

    The Python ``Path`` constructor forces every PoseStamped element to share
    the header's stamp/frame_id, so it cannot produce *distinct* per-element
    frame_ids.  This minimal encoder writes them explicitly, letting the tests
    prove that ``__getitem__``/``__iter__`` read each element's own frame_id
    rather than the header's.  Layout matches ``Path::new`` in src/nav_msgs.rs.

    ``elements`` is a list of ``(sec, nanosec, frame_id, (x, y, z), (qx, qy, qz, qw))``.
    """
    body = bytearray()

    def align(n: int) -> None:
        # CDR alignment origin is the start of the body (after the 4-byte
        # encapsulation header), which is exactly ``len(body)`` here.
        while len(body) % n != 0:
            body.append(0)

    def u32(v: int) -> None:
        align(4)
        body.extend(struct.pack("<I", v))

    def i32(v: int) -> None:
        align(4)
        body.extend(struct.pack("<i", v))

    def f64(v: float) -> None:
        align(8)
        body.extend(struct.pack("<d", v))

    def cdr_string(s: str) -> None:
        b = s.encode("utf-8")
        u32(len(b) + 1)  # length includes the NUL terminator
        body.extend(b)
        body.append(0)

    def time(sec: int, nanosec: int) -> None:
        i32(sec)
        u32(nanosec)

    def pose(xyz, quat) -> None:
        for v in xyz:  # Point (first f64 triggers the align-to-8)
            f64(v)
        for v in quat:  # Quaternion
            f64(v)

    # Header: stamp + frame_id
    time(*header_stamp)
    cdr_string(header_frame)
    # Sequence of PoseStamped
    u32(len(elements))
    for sec, nanosec, fid, xyz, quat in elements:
        time(sec, nanosec)
        cdr_string(fid)
        pose(xyz, quat)

    return b"\x00\x01\x00\x00" + bytes(body)


class TestOdometry:
    def test_round_trip(self):
        h = Header(stamp=Time(1, 2), frame_id="odom")
        pose = PoseWithCovariance(
            pose=Pose(position=Point(1.0, 2.0, 3.0), orientation=Quaternion(0, 0, 0, 1)),
            covariance=[0.01] * 36,
        )
        twist = TwistWithCovariance(
            twist=Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0, 0, 0.1)),
            covariance=[0.001] * 36,
        )
        odo = Odometry(header=h, child_frame_id="base_link", pose=pose, twist=twist)

        restored = Odometry.from_cdr(odo.to_bytes())
        assert restored.frame_id == "odom"
        assert restored.child_frame_id == "base_link"
        # Composite drill-down — pose/twist round-trip end-to-end.
        assert restored.pose.pose.position.x == 1.0
        assert restored.pose.covariance == [0.01] * 36
        assert restored.twist.twist.linear.x == 0.5
        assert restored.twist.twist.angular.z == 0.1

    def test_default_pose_is_identity(self):
        h = Header(stamp=Time(0, 0), frame_id="odom")
        odo = Odometry(header=h, child_frame_id="base")
        restored = Odometry.from_cdr(odo.to_bytes())
        # Identity quaternion (w=1) is the natural default for Pose.
        assert restored.pose.pose.orientation.w == 1.0
        assert restored.pose.pose.position.x == 0.0


# ── MapMetaData ─────────────────────────────────────────────────────


class TestMapMetaData:
    def test_round_trip(self):
        md = MapMetaData(
            map_load_time=Time(1, 2),
            resolution=0.05,
            width=100,
            height=100,
            origin=Pose(position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
        )
        restored = MapMetaData.from_cdr(md.to_bytes())
        assert abs(restored.resolution - 0.05) < 1e-6
        assert restored.width == 100
        assert restored.height == 100
        assert restored.map_load_time.sec == 1
        assert restored.origin.orientation.w == 1.0

    def test_from_golden(self):
        golden = _load("nav_msgs", "MapMetaData")
        md = MapMetaData.from_cdr(golden)
        assert abs(md.resolution - 0.05) < 1e-6
        assert md.width == 200
        assert md.height == 200
        assert abs(md.origin.position.x - (-5.0)) < 1e-10
        assert abs(md.origin.position.y - (-5.0)) < 1e-10
        assert md.origin.orientation.w == 1.0
        # round-trip
        assert MapMetaData.from_cdr(md.to_bytes()).to_bytes() == golden

    def test_repr(self):
        md = MapMetaData(resolution=0.1, width=50, height=50)
        r = repr(md)
        assert "MapMetaData" in r
        assert "50" in r


# ── GridCells ────────────────────────────────────────────────────────


class TestGridCells:
    def test_round_trip(self):
        h = Header(stamp=Time(1, 0), frame_id="map")
        cells = [Point(1.0, 2.0, 0.0), Point(3.0, 4.0, 0.0)]
        gc = GridCells(header=h, cell_width=0.5, cell_height=0.5, cells=cells)
        restored = GridCells.from_cdr(gc.to_bytes())
        assert restored.frame_id == "map"
        assert abs(restored.cell_width - 0.5) < 1e-6
        assert abs(restored.cell_height - 0.5) < 1e-6
        assert len(restored) == 2
        assert abs(restored[0].x - 1.0) < 1e-10
        assert abs(restored[1].y - 4.0) < 1e-10

    def test_from_golden(self):
        golden = _load("nav_msgs", "GridCells")
        gc = GridCells.from_cdr(golden)
        assert gc.frame_id == "test_frame"
        assert abs(gc.cell_width - 0.5) < 1e-6
        assert len(gc) == 3
        assert abs(gc[0].x - 1.0) < 1e-10
        assert abs(gc[2].y - 6.0) < 1e-10
        all_cells = gc.cells
        assert len(all_cells) == 3
        assert GridCells.from_cdr(golden).to_bytes() == golden

    def test_empty(self):
        h = Header(stamp=Time(0, 0), frame_id="map")
        gc = GridCells(header=h)
        assert len(gc) == 0
        restored = GridCells.from_cdr(gc.to_bytes())
        assert len(restored) == 0

    def test_index_error(self):
        h = Header(stamp=Time(0, 0), frame_id="map")
        gc = GridCells(header=h, cells=[Point(0, 0, 0)])
        with pytest.raises(IndexError):
            _ = gc[5]


# ── OccupancyGrid ────────────────────────────────────────────────────


class TestOccupancyGrid:
    def test_round_trip(self):
        h = Header(stamp=Time(1, 0), frame_id="map")
        meta = MapMetaData(
            resolution=0.1,
            width=4,
            height=2,
            origin=Pose(position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
        )
        og = OccupancyGrid(header=h, info=meta, data=[0, 50, 100, -1, 25, 75, 0, 0])
        restored = OccupancyGrid.from_cdr(og.to_bytes())
        assert restored.frame_id == "map"
        assert abs(restored.info.resolution - 0.1) < 1e-6
        assert restored.info.width == 4
        assert restored.info.height == 2
        assert restored.data_len == 8
        raw = bytes(restored.data)
        import struct
        vals = list(struct.unpack_from("8b", raw))
        assert vals[1] == 50
        assert vals[2] == 100
        assert vals[3] == -1

    def test_from_golden(self):
        golden = _load("nav_msgs", "OccupancyGrid")
        og = OccupancyGrid.from_cdr(golden)
        assert og.frame_id == "test_frame"
        info = og.info
        assert abs(info.resolution - 0.1) < 1e-6
        assert info.width == 4
        assert info.height == 2
        assert og.data_len == 8
        vals = list(struct.unpack_from("8b", bytes(og.data)))
        assert vals == [0, 50, 100, -1, 25, 75, 0, 0]
        assert OccupancyGrid.from_cdr(golden).to_bytes() == golden

    def test_empty_data(self):
        h = Header(stamp=Time(0, 0), frame_id="map")
        og = OccupancyGrid(header=h)
        assert og.data_len == 0
        restored = OccupancyGrid.from_cdr(og.to_bytes())
        assert restored.data_len == 0


# ── Path ─────────────────────────────────────────────────────────────


class TestPath:
    def test_round_trip(self):
        h = Header(stamp=Time(0, 0), frame_id="odom")
        poses = [
            Pose(position=Point(1.0, 0.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
            Pose(position=Point(2.0, 1.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
        ]
        path = Path(header=h, poses=poses)
        restored = Path.from_cdr(path.to_bytes())
        assert restored.frame_id == "odom"
        assert len(restored) == 2
        all_poses = restored.poses()
        assert len(all_poses) == 2
        assert abs(all_poses[0][2].position.x - 1.0) < 1e-10
        assert abs(all_poses[1][2].position.x - 2.0) < 1e-10

    def test_from_golden(self):
        golden = _load("nav_msgs", "Path")
        path = Path.from_cdr(golden)
        assert path.frame_id == "test_frame"
        assert len(path) == 3
        all_poses = path.poses()
        assert all_poses[0][0].sec == 1
        assert all_poses[1][0].sec == 2
        assert all_poses[2][0].sec == 3
        assert abs(all_poses[0][2].position.x - 1.0) < 1e-10
        assert Path.from_cdr(golden).to_bytes() == golden

    def test_empty(self):
        h = Header(stamp=Time(0, 0), frame_id="map")
        path = Path(header=h)
        assert len(path) == 0
        restored = Path.from_cdr(path.to_bytes())
        assert len(restored) == 0
        assert restored.poses() == []

    def test_getitem_and_iter_distinct_frame_ids(self):
        # Hand-built buffer so each element carries its OWN stamp/frame_id,
        # distinct from each other and from the header — this is what the
        # uniform Python constructor cannot express.
        elements = [
            (1, 100, "frame_a", (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            (2, 200, "frame_b", (2.0, 1.0, 0.0), (0.0, 0.0, 0.707, 0.707)),
            (3, 300, "frame_c", (3.0, 2.0, 0.0), (0.0, 0.0, 1.0, 0.0)),
        ]
        buf = _build_path_cdr((10, 0), "world", elements)
        path = Path.from_cdr(buf)
        assert len(path) == 3
        # Header frame_id is "world"; per-element frame_ids must NOT be it.
        assert path.frame_id == "world"

        # __getitem__ returns each element's own (stamp, frame_id, pose).
        for i, (sec, nanosec, fid, xyz, quat) in enumerate(elements):
            stamp, frame_id, pose = path[i]
            assert stamp.sec == sec
            assert stamp.nanosec == nanosec
            assert frame_id == fid
            assert frame_id != path.frame_id
            assert abs(pose.position.x - xyz[0]) < 1e-10
            assert abs(pose.orientation.w - quat[3]) < 1e-10

        # __iter__ is a single pass yielding the same per-element tuples.
        iterated = list(path)
        assert len(iterated) == 3
        assert [e[1] for e in iterated] == ["frame_a", "frame_b", "frame_c"]
        # frame_ids genuinely differ per element (not the header's repeated).
        assert len(set(e[1] for e in iterated)) == 3
        for i, (stamp, frame_id, pose) in enumerate(iterated):
            g_stamp, g_frame, g_pose = path[i]
            assert (stamp.sec, stamp.nanosec) == (g_stamp.sec, g_stamp.nanosec)
            assert frame_id == g_frame
            assert pose.position.x == g_pose.position.x

        # __iter__ agrees with poses() element-for-element.
        listed = path.poses()
        assert [e[1] for e in iterated] == [e[1] for e in listed]
        # Negative indexing mirrors the other sequence types.
        assert path[-1][1] == "frame_c"
        assert path[-3][1] == "frame_a"

    def test_iter_matches_constructor(self):
        # for-loop iteration over a constructor-built Path yields one tuple
        # per pose without relying on repeated __getitem__ calls.
        h = Header(stamp=Time(7, 8), frame_id="odom")
        poses = [
            Pose(position=Point(1.0, 0.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
            Pose(position=Point(2.0, 1.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
        ]
        path = Path(header=h, poses=poses)
        collected = [pose.position.x for (_stamp, _fid, pose) in path]
        assert collected == [1.0, 2.0]

    def test_index_error(self):
        h = Header(stamp=Time(0, 0), frame_id="odom")
        path = Path(
            header=h,
            poses=[Pose(position=Point(0, 0, 0), orientation=Quaternion(0, 0, 0, 1))],
        )
        with pytest.raises(IndexError):
            _ = path[5]
        with pytest.raises(IndexError):
            _ = path[-5]
