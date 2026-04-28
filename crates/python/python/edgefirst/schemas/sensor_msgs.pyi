# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.sensor_msgs``."""

from __future__ import annotations
from typing import List, Optional, Sequence

from . import BorrowedBuf, BufferLike
from .builtin_interfaces import Time
from .geometry_msgs import Quaternion, Vector3
from .std_msgs import Header

__all__ = [
    "BatteryState",
    "CameraInfo",
    "CompressedImage",
    "FluidPressure",
    "Image",
    "Imu",
    "MagneticField",
    "NavSatFix",
    "NavSatStatus",
    "PointCloud2",
    "PointField",
    "RegionOfInterest",
    "Temperature",
]


class NavSatStatus:
    """``sensor_msgs.NavSatStatus`` — GNSS fix status + service bitmask.

    Status (i8): ``-1`` NO_FIX, ``0`` FIX, ``1`` SBAS_FIX, ``2`` GBAS_FIX.

    Service bitmask (u16):
        ``0x01`` GPS, ``0x02`` GLONASS, ``0x04`` COMPASS, ``0x08`` GALILEO.

    CdrFixed — 4 bytes payload (i8 + pad + u16, position-dependent
    alignment within composite messages).
    """

    def __init__(self, status: int = 0, service: int = 0) -> None: ...

    @property
    def status(self) -> int: ...
    @property
    def service(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> NavSatStatus: ...
    def __repr__(self) -> str: ...


class RegionOfInterest:
    """``sensor_msgs.RegionOfInterest`` — sub-window of a camera image.

    CdrFixed — 17 bytes payload (4×u32 + bool).
    """

    def __init__(
        self,
        x_offset: int = 0,
        y_offset: int = 0,
        height: int = 0,
        width: int = 0,
        do_rectify: bool = False,
    ) -> None: ...

    @property
    def x_offset(self) -> int: ...
    @property
    def y_offset(self) -> int: ...
    @property
    def height(self) -> int: ...
    @property
    def width(self) -> int: ...
    @property
    def do_rectify(self) -> bool: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> RegionOfInterest: ...
    def __repr__(self) -> str: ...


class PointField:
    """``sensor_msgs.PointField`` — descriptor of one field within a
    :class:`PointCloud2`'s packed point layout.

    ``datatype`` follows the ROS 2 spec:
        1 INT8, 2 UINT8, 3 INT16, 4 UINT16, 5 INT32, 6 UINT32,
        7 FLOAT32, 8 FLOAT64.
    """

    def __init__(
        self,
        name: str = "",
        offset: int = 0,
        datatype: int = 0,
        count: int = 1,
    ) -> None: ...

    @property
    def name(self) -> str: ...
    @property
    def offset(self) -> int: ...
    @property
    def datatype(self) -> int: ...
    @property
    def count(self) -> int: ...
    def __repr__(self) -> str: ...


class Image:
    """``sensor_msgs.Image`` — uncompressed raster image.

    Construction copies the bulk ``data`` payload once into the CDR
    buffer (GIL released during the copy). Read-side ``data`` returns a
    :class:`BorrowedBuf` exposing the in-buffer payload via the buffer
    protocol — zero-copy for ``np.frombuffer``, ``memoryview``, etc.

    Example
    -------
    ::

        import numpy as np
        pixels = np.zeros((720, 1280, 3), dtype=np.uint8)
        img = Image(
            header=Header(stamp=Time(1, 0), frame_id="cam"),
            height=720, width=1280, encoding="rgb8",
            is_bigendian=0, step=1280 * 3,
            data=pixels,
        )

        # Zero-copy view (Py 3.11+ / non-abi3):
        arr = np.frombuffer(img.data, dtype=np.uint8).reshape(720, 1280, 3)
        # On abi3-py38: arr = np.frombuffer(img.data.view(), dtype=np.uint8).reshape(720, 1280, 3)
    """

    def __init__(
        self,
        header: Header,
        height: int,
        width: int,
        encoding: str,
        is_bigendian: int,
        step: int,
        data: BufferLike,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def height(self) -> int: ...
    @property
    def width(self) -> int: ...
    @property
    def encoding(self) -> str: ...
    @property
    def is_bigendian(self) -> int: ...
    @property
    def step(self) -> int:
        """Row stride in bytes."""

    @property
    def cdr_size(self) -> int: ...

    @property
    def data(self) -> BorrowedBuf:
        """Zero-copy view of the pixel data."""

    def cdr_view(self) -> BorrowedBuf:
        """Zero-copy view of the full CDR buffer (header + payload)."""

    def to_bytes(self) -> bytes:
        """Return the full CDR buffer as ``bytes`` (one copy)."""

    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Image: ...

    def __repr__(self) -> str: ...


class CompressedImage:
    """``sensor_msgs.CompressedImage`` — JPEG / PNG / etc. encoded image.

    Same construction model as :class:`Image`: bulk ``data`` is copied
    once at build time, then exposed read-only via :class:`BorrowedBuf`.
    """

    def __init__(
        self,
        header: Header,
        format: str,
        data: BufferLike,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def format(self) -> str:
        """Codec identifier — e.g. ``"jpeg"``, ``"png"``."""

    @property
    def cdr_size(self) -> int: ...

    @property
    def data(self) -> BorrowedBuf:
        """Zero-copy view of the encoded image bytes."""

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> CompressedImage: ...
    def __repr__(self) -> str: ...


class PointCloud2:
    """``sensor_msgs.PointCloud2`` — packed point cloud with
    :class:`PointField` descriptors.

    ``data`` is a packed byte array of ``width × height`` points, each
    ``point_step`` bytes wide. Field offsets/types come from the
    ``fields`` list (set at construction or returned by the property).

    Example
    -------
    ::

        fields = [
            PointField(name="x", offset=0,  datatype=7, count=1),  # float32
            PointField(name="y", offset=4,  datatype=7, count=1),
            PointField(name="z", offset=8,  datatype=7, count=1),
            PointField(name="i", offset=12, datatype=2, count=1),  # uint8
        ]
        pc = PointCloud2(
            header=Header(stamp=Time(1, 0), frame_id="lidar"),
            height=1, width=131072, fields=fields,
            is_bigendian=False, point_step=13, row_step=13 * 131072,
            data=byte_buffer, is_dense=True,
        )
    """

    def __init__(
        self,
        header: Header,
        height: int,
        width: int,
        fields: Sequence[PointField],
        is_bigendian: bool,
        point_step: int,
        row_step: int,
        data: BufferLike,
        is_dense: bool = True,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def height(self) -> int: ...
    @property
    def width(self) -> int: ...
    @property
    def is_bigendian(self) -> bool: ...
    @property
    def point_step(self) -> int: ...
    @property
    def row_step(self) -> int: ...
    @property
    def is_dense(self) -> bool: ...
    @property
    def cdr_size(self) -> int: ...
    @property
    def fields(self) -> List[PointField]: ...

    @property
    def data(self) -> BorrowedBuf:
        """Zero-copy view of the packed point data."""

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> PointCloud2: ...
    def __repr__(self) -> str: ...


class Imu:
    """``sensor_msgs.Imu`` — orientation, angular velocity and linear
    acceleration each with a 3×3 row-major covariance matrix.

    Covariance matrices are exposed as 9-element ``list[float]`` (not
    numpy) since they're small fixed-size and Python list semantics are
    cleaner for this scale.
    """

    def __init__(
        self,
        header: Header,
        orientation: Optional[Quaternion] = None,
        orientation_covariance: Optional[Sequence[float]] = None,
        angular_velocity: Optional[Vector3] = None,
        angular_velocity_covariance: Optional[Sequence[float]] = None,
        linear_acceleration: Optional[Vector3] = None,
        linear_acceleration_covariance: Optional[Sequence[float]] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def orientation(self) -> Quaternion: ...
    @property
    def orientation_covariance(self) -> List[float]: ...
    @property
    def angular_velocity(self) -> Vector3: ...
    @property
    def angular_velocity_covariance(self) -> List[float]: ...
    @property
    def linear_acceleration(self) -> Vector3: ...
    @property
    def linear_acceleration_covariance(self) -> List[float]: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Imu: ...


class NavSatFix:
    """``sensor_msgs.NavSatFix`` — GNSS position fix with covariance.

    ``position_covariance_type`` constants:
        0 = UNKNOWN, 1 = APPROXIMATED, 2 = DIAGONAL_KNOWN, 3 = KNOWN.
    """

    def __init__(
        self,
        header: Header,
        status: Optional[NavSatStatus] = None,
        latitude: float = 0.0,
        longitude: float = 0.0,
        altitude: float = 0.0,
        position_covariance: Optional[Sequence[float]] = None,
        position_covariance_type: int = 0,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def status(self) -> NavSatStatus: ...
    @property
    def latitude(self) -> float: ...
    @property
    def longitude(self) -> float: ...
    @property
    def altitude(self) -> float: ...
    @property
    def position_covariance(self) -> List[float]: ...
    @property
    def position_covariance_type(self) -> int: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> NavSatFix: ...


class MagneticField:
    """``sensor_msgs.MagneticField`` — 3-axis magnetic field vector with
    3×3 covariance. Units are typically Tesla.
    """

    def __init__(
        self,
        header: Header,
        magnetic_field: Optional[Vector3] = None,
        magnetic_field_covariance: Optional[Sequence[float]] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def magnetic_field(self) -> Vector3: ...
    @property
    def magnetic_field_covariance(self) -> List[float]: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> MagneticField: ...


class FluidPressure:
    """``sensor_msgs.FluidPressure`` — atmospheric / fluid pressure
    (typically Pascals) with scalar variance.
    """

    def __init__(
        self, header: Header, fluid_pressure: float = 0.0, variance: float = 0.0
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def fluid_pressure(self) -> float: ...
    @property
    def variance(self) -> float: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> FluidPressure: ...


class Temperature:
    """``sensor_msgs.Temperature`` — temperature reading (typically
    Celsius) with scalar variance.
    """

    def __init__(
        self, header: Header, temperature: float = 0.0, variance: float = 0.0
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def temperature(self) -> float: ...
    @property
    def variance(self) -> float: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Temperature: ...


class CameraInfo:
    """``sensor_msgs.CameraInfo`` — camera calibration and metadata.

    Intrinsic matrix ``k`` (3×3, 9 elements), rectification ``r`` (3×3),
    projection ``p`` (3×4, 12 elements), distortion ``d`` (variable length).
    """

    def __init__(
        self,
        header: Header,
        height: int = 0,
        width: int = 0,
        distortion_model: str = "",
        d: Optional[Sequence[float]] = None,
        k: Optional[Sequence[float]] = None,
        r: Optional[Sequence[float]] = None,
        p: Optional[Sequence[float]] = None,
        binning_x: int = 0,
        binning_y: int = 0,
        roi: Optional[RegionOfInterest] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def height(self) -> int: ...
    @property
    def width(self) -> int: ...
    @property
    def distortion_model(self) -> str: ...
    @property
    def d(self) -> List[float]: ...
    @property
    def k(self) -> List[float]: ...
    @property
    def r(self) -> List[float]: ...
    @property
    def p(self) -> List[float]: ...
    @property
    def binning_x(self) -> int: ...
    @property
    def binning_y(self) -> int: ...
    @property
    def roi(self) -> RegionOfInterest: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> CameraInfo: ...


class BatteryState:
    """``sensor_msgs.BatteryState`` — battery charge status and health.

    Voltage, current, charge, capacity in SI units (V, A, Ah).
    """

    def __init__(
        self,
        header: Header,
        voltage: float = 0.0,
        temperature: float = 0.0,
        current: float = 0.0,
        charge: float = 0.0,
        capacity: float = 0.0,
        design_capacity: float = 0.0,
        percentage: float = 0.0,
        power_supply_status: int = 0,
        power_supply_health: int = 0,
        power_supply_technology: int = 0,
        present: bool = False,
        cell_voltage: Optional[Sequence[float]] = None,
        cell_temperature: Optional[Sequence[float]] = None,
        location: str = "",
        serial_number: str = "",
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def voltage(self) -> float: ...
    @property
    def temperature(self) -> float: ...
    @property
    def current(self) -> float: ...
    @property
    def charge(self) -> float: ...
    @property
    def capacity(self) -> float: ...
    @property
    def design_capacity(self) -> float: ...
    @property
    def percentage(self) -> float: ...
    @property
    def power_supply_status(self) -> int: ...
    @property
    def power_supply_health(self) -> int: ...
    @property
    def power_supply_technology(self) -> int: ...
    @property
    def present(self) -> bool: ...
    @property
    def cell_voltage(self) -> List[float]: ...
    @property
    def cell_temperature(self) -> List[float]: ...
    @property
    def location(self) -> str: ...
    @property
    def serial_number(self) -> str: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> BatteryState: ...
