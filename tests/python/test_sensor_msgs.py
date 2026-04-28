# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for `edgefirst.schemas.sensor_msgs`.

Exercises all six wrapped types:

- ``Image`` — buffer-backed, with the bulk pixel payload exposed via a
  zero-copy ``BorrowedBuf`` (buffer protocol on Py 3.11+ / non-abi3,
  ``view()`` fallback on abi3-py38).
- ``CompressedImage`` — same buffer-backed pattern with a small `format`
  field instead of full `Image` metadata.
- ``PointCloud2`` — buffer-backed plus a list of ``PointField``
  descriptors. Round-trip preserves field name/offset/datatype/count.
- ``PointField`` — value-type descriptor used by PointCloud2.
- ``NavSatStatus`` — CdrFixed (4 bytes payload, position-dependent
  alignment within composite messages).
- ``RegionOfInterest`` — CdrFixed (17 bytes payload).

`Image` covers both the abi3-py38 path (`img.data.view()` returning a
``memoryview``) and the buffer-protocol path (``np.frombuffer(img.data)``
on builds where the protocol is available).
"""

import numpy as np
import pytest

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.geometry_msgs import Quaternion, Vector3
from edgefirst.schemas.sensor_msgs import (
    CompressedImage,
    FluidPressure,
    Image,
    Imu,
    MagneticField,
    NavSatFix,
    NavSatStatus,
    PointCloud2,
    PointField,
    RegionOfInterest,
    Temperature,
)
from edgefirst.schemas.std_msgs import Header


# ── Image ──────────────────────────────────────────────────────────


class TestImage:
    def test_construct_from_numpy(self, sample_header, zero_pixels_hd_rgb8):
        img = Image(
            header=sample_header,
            height=720,
            width=1280,
            encoding="rgb8",
            is_bigendian=0,
            step=1280 * 3,
            data=zero_pixels_hd_rgb8,
        )
        assert img.height == 720
        assert img.width == 1280
        assert img.encoding == "rgb8"
        assert img.is_bigendian == 0
        assert img.step == 1280 * 3
        assert img.frame_id == "sensor_frame"

    def test_construct_from_bytes(self, sample_header):
        # Accepts plain bytes too.
        data = bytes(640 * 480 * 3)
        img = Image(
            header=sample_header,
            height=480, width=640, encoding="rgb8",
            is_bigendian=0, step=640 * 3, data=data,
        )
        assert img.height == 480

    def test_round_trip_preserves_payload(self, sample_header):
        # Random payload — verify byte-for-byte preservation through CDR.
        rng = np.random.default_rng(seed=42)
        pixels = rng.integers(0, 256, size=720 * 1280 * 3, dtype=np.uint8)
        img = Image(
            header=sample_header, height=720, width=1280, encoding="rgb8",
            is_bigendian=0, step=1280 * 3, data=pixels,
        )

        restored = Image.from_cdr(img.to_bytes())
        assert restored.width == 1280
        assert restored.encoding == "rgb8"
        # tobytes() always works regardless of buffer protocol availability.
        assert restored.data.tobytes() == bytes(pixels)

    def test_data_view_is_memoryview(self, sample_header, zero_pixels_hd_rgb8):
        # `view()` is the abi3-py38 fallback; always returns a memoryview.
        img = Image(
            header=sample_header, height=720, width=1280, encoding="rgb8",
            is_bigendian=0, step=1280 * 3, data=zero_pixels_hd_rgb8,
        )
        mv = img.data.view()
        assert isinstance(mv, memoryview)
        assert len(mv) == 720 * 1280 * 3

    def test_data_view_is_zero_copy(self, sample_header, zero_pixels_hd_rgb8):
        # Two views of the same Image's payload must share storage —
        # i.e. the BorrowedBuf isn't copying on each `data` access.
        img = Image(
            header=sample_header, height=720, width=1280, encoding="rgb8",
            is_bigendian=0, step=1280 * 3, data=zero_pixels_hd_rgb8,
        )
        mv1 = img.data.view()
        mv2 = img.data.view()
        # memoryview.obj is the underlying object; cast to int is the
        # buffer pointer, which must match between views.
        # Easier: read via numpy and confirm pointer equivalence.
        a1 = np.frombuffer(mv1, dtype=np.uint8)
        a2 = np.frombuffer(mv2, dtype=np.uint8)
        assert a1.ctypes.data == a2.ctypes.data

    def test_borrowed_buf_lifetime_holds_parent(self, sample_header):
        # The view must keep the parent alive even after `img` goes
        # out of scope — that's the whole point of the parent ref slot.
        def make_view():
            local_img = Image(
                header=sample_header, height=10, width=10, encoding="rgb8",
                is_bigendian=0, step=30, data=bytes(10 * 10 * 3),
            )
            return local_img.data

        view = make_view()
        # The Image is no longer reachable directly, but the parent ref
        # inside `view` holds it.
        assert view.tobytes() == bytes(10 * 10 * 3)

    def test_cdr_size_matches_to_bytes(self, sample_header, zero_pixels_hd_rgb8):
        img = Image(
            header=sample_header, height=720, width=1280, encoding="rgb8",
            is_bigendian=0, step=1280 * 3, data=zero_pixels_hd_rgb8,
        )
        assert img.cdr_size == len(img.to_bytes())


# ── CompressedImage ────────────────────────────────────────────────


class TestCompressedImage:
    def test_round_trip(self, sample_header, random_bytes_100kb):
        img = CompressedImage(
            header=sample_header, format="jpeg", data=random_bytes_100kb
        )
        restored = CompressedImage.from_cdr(img.to_bytes())
        assert restored.format == "jpeg"
        assert restored.data.tobytes() == random_bytes_100kb
        assert restored.frame_id == sample_header.frame_id

    @pytest.mark.parametrize("format", ["jpeg", "png", "webp", "avif"])
    def test_format_round_trip(self, sample_header, format):
        img = CompressedImage(
            header=sample_header, format=format, data=b"\x00\x01\x02\x03"
        )
        assert CompressedImage.from_cdr(img.to_bytes()).format == format


# ── PointCloud2 ────────────────────────────────────────────────────


@pytest.fixture
def xyz_intensity_fields():
    """Standard (x, y, z, intensity) layout — 16 bytes per point, all f32."""
    return [
        PointField(name="x", offset=0, datatype=7, count=1),
        PointField(name="y", offset=4, datatype=7, count=1),
        PointField(name="z", offset=8, datatype=7, count=1),
        PointField(name="intensity", offset=12, datatype=7, count=1),
    ]


class TestPointCloud2:
    def test_round_trip(self, sample_header, xyz_intensity_fields):
        n_points = 10_000
        data = bytes(n_points * 16)
        pc = PointCloud2(
            header=sample_header, height=1, width=n_points,
            fields=xyz_intensity_fields, is_bigendian=False,
            point_step=16, row_step=16 * n_points, data=data, is_dense=True,
        )

        restored = PointCloud2.from_cdr(pc.to_bytes())
        assert restored.width == n_points
        assert restored.height == 1
        assert restored.point_step == 16
        assert restored.row_step == 16 * n_points
        assert restored.is_dense is True
        assert len(restored.fields) == 4

        # Field descriptors should round-trip exactly.
        names = [f.name for f in restored.fields]
        assert names == ["x", "y", "z", "intensity"]
        offsets = [f.offset for f in restored.fields]
        assert offsets == [0, 4, 8, 12]

    def test_data_view_zero_copy(self, sample_header, xyz_intensity_fields):
        rng = np.random.default_rng(seed=0)
        # Random f32 point payload — confirm byte-for-byte recovery.
        payload = rng.bytes(100 * 16)
        pc = PointCloud2(
            header=sample_header, height=1, width=100,
            fields=xyz_intensity_fields, is_bigendian=False,
            point_step=16, row_step=100 * 16, data=payload, is_dense=True,
        )
        assert pc.data.tobytes() == payload


# ── PointField ─────────────────────────────────────────────────────


class TestPointField:
    def test_defaults(self):
        f = PointField()
        assert f.name == "" and f.offset == 0 and f.datatype == 0 and f.count == 1

    def test_explicit_values(self):
        f = PointField(name="x", offset=4, datatype=7, count=1)
        assert f.name == "x"
        assert f.offset == 4
        assert f.datatype == 7
        assert f.count == 1


# ── NavSatStatus ───────────────────────────────────────────────────


class TestNavSatStatus:
    def test_defaults(self):
        s = NavSatStatus()
        assert s.status == 0 and s.service == 0

    @pytest.mark.parametrize(
        "status,service",
        [
            (-1, 0),       # NO_FIX
            (0, 0x01),     # FIX, GPS only
            (1, 0x05),     # SBAS_FIX, GPS + COMPASS
            (2, 0x0F),     # GBAS_FIX, all four constellations
        ],
    )
    def test_round_trip(self, status, service):
        s = NavSatStatus(status=status, service=service)
        restored = NavSatStatus.from_cdr(s.to_bytes())
        assert restored.status == status
        assert restored.service == service


# ── RegionOfInterest ───────────────────────────────────────────────


class TestRegionOfInterest:
    def test_defaults(self):
        r = RegionOfInterest()
        assert (r.x_offset, r.y_offset, r.height, r.width) == (0, 0, 0, 0)
        assert r.do_rectify is False

    def test_round_trip(self):
        r = RegionOfInterest(
            x_offset=10, y_offset=20, height=480, width=640, do_rectify=True
        )
        restored = RegionOfInterest.from_cdr(r.to_bytes())
        assert restored.x_offset == 10
        assert restored.y_offset == 20
        assert restored.height == 480
        assert restored.width == 640
        assert restored.do_rectify is True


# ── Imu ────────────────────────────────────────────────────────────


class TestImu:
    def test_round_trip(self, sample_header):
        cov = list(range(9))  # 0..8
        imu = Imu(
            header=sample_header,
            orientation=Quaternion(x=0, y=0, z=0.7071, w=0.7071),
            orientation_covariance=cov,
            angular_velocity=Vector3(x=0.1, y=0.2, z=0.3),
            angular_velocity_covariance=[0.001] * 9,
            linear_acceleration=Vector3(x=9.81, y=0.0, z=0.0),
            linear_acceleration_covariance=[0.01] * 9,
        )
        restored = Imu.from_cdr(imu.to_bytes())
        assert restored.frame_id == sample_header.frame_id
        assert restored.orientation.w == 0.7071
        assert restored.orientation_covariance == [float(i) for i in range(9)]
        assert restored.angular_velocity.z == 0.3
        assert restored.linear_acceleration.x == 9.81

    def test_invalid_covariance_length(self, sample_header):
        with pytest.raises(ValueError, match="9-element"):
            Imu(
                header=sample_header,
                orientation_covariance=[0.0, 1.0],  # wrong length
            )


# ── NavSatFix ──────────────────────────────────────────────────────


class TestNavSatFix:
    def test_round_trip(self, sample_header):
        # Au-Zone HQ coords (Montréal area) — confirms negative longitude
        # round-trips correctly through f64 wire format.
        nf = NavSatFix(
            header=sample_header,
            status=NavSatStatus(status=0, service=0x01),
            latitude=45.5017,
            longitude=-73.5673,
            altitude=36.0,
            position_covariance=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            position_covariance_type=2,
        )
        restored = NavSatFix.from_cdr(nf.to_bytes())
        assert restored.latitude == 45.5017
        assert restored.longitude == -73.5673
        assert restored.altitude == 36.0
        assert restored.status.status == 0
        assert restored.status.service == 0x01
        assert restored.position_covariance_type == 2


# ── MagneticField ──────────────────────────────────────────────────


class TestMagneticField:
    def test_round_trip(self, sample_header):
        mf = MagneticField(
            header=sample_header,
            magnetic_field=Vector3(x=20e-6, y=5e-6, z=-45e-6),
            magnetic_field_covariance=[1e-12] * 9,
        )
        restored = MagneticField.from_cdr(mf.to_bytes())
        assert restored.magnetic_field.x == 20e-6
        assert restored.magnetic_field.z == -45e-6


# ── FluidPressure ──────────────────────────────────────────────────


class TestFluidPressure:
    def test_round_trip(self, sample_header):
        fp = FluidPressure(
            header=sample_header, fluid_pressure=101_325.0, variance=10.0
        )
        restored = FluidPressure.from_cdr(fp.to_bytes())
        assert restored.fluid_pressure == 101_325.0
        assert restored.variance == 10.0


# ── Temperature ────────────────────────────────────────────────────


class TestTemperature:
    def test_round_trip(self, sample_header):
        t = Temperature(header=sample_header, temperature=23.5, variance=0.1)
        restored = Temperature.from_cdr(t.to_bytes())
        assert restored.temperature == 23.5
        assert restored.variance == 0.1

    @pytest.mark.parametrize(
        "value", [-273.15, 0.0, 25.0, 100.0, 1500.0]
    )
    def test_temperature_range(self, sample_header, value):
        # Temperature is f64 — sweep typical sensor ranges.
        t = Temperature(header=sample_header, temperature=value, variance=0.0)
        assert Temperature.from_cdr(t.to_bytes()).temperature == value


# ── CameraInfo ─────────────────────────────────────────────────────

from edgefirst.schemas.sensor_msgs import CameraInfo


class TestCameraInfo:
    def test_round_trip(self, sample_header):
        ci = CameraInfo(
            header=sample_header,
            height=480,
            width=640,
            distortion_model="plumb_bob",
            d=[0.1, 0.2, 0.3, 0.4, 0.5],
            k=[1.0] * 9,
            r=[0.0] * 9,
            p=[0.0] * 12,
            binning_x=1,
            binning_y=1,
        )
        restored = CameraInfo.from_cdr(ci.to_bytes())
        assert restored.height == 480
        assert restored.width == 640
        assert restored.distortion_model == "plumb_bob"
        assert len(restored.d) == 5
        assert len(restored.k) == 9
        assert len(restored.p) == 12
        assert restored.binning_x == 1

    def test_defaults(self, sample_header):
        ci = CameraInfo(header=sample_header)
        assert ci.height == 0
        assert ci.width == 0
        assert ci.distortion_model == ""


# ── BatteryState ───────────────────────────────────────────────────

from edgefirst.schemas.sensor_msgs import BatteryState


class TestBatteryState:
    def test_round_trip(self, sample_header):
        bs = BatteryState(
            header=sample_header,
            voltage=12.5,
            current=1.5,
            percentage=0.75,
            present=True,
            location="main",
            serial_number="SN001",
            cell_voltage=[3.7, 3.7, 3.7],
        )
        restored = BatteryState.from_cdr(bs.to_bytes())
        assert abs(restored.voltage - 12.5) < 0.01
        assert abs(restored.current - 1.5) < 0.01
        assert abs(restored.percentage - 0.75) < 0.01
        assert restored.present is True
        assert restored.location == "main"
        assert restored.serial_number == "SN001"
        assert len(restored.cell_voltage) == 3
