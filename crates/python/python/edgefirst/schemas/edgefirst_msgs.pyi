# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.edgefirst_msgs``."""

from __future__ import annotations

from typing import List, Optional, Sequence

from . import BorrowedBuf, BufferLike
from .builtin_interfaces import Time
from .std_msgs import Header
from .geometry_msgs import Vector3

__all__ = [
    "CameraFrame",
    "CameraPlane",
    "Date",
    "Detect",
    "DetectBox",
    "DmaBuffer",
    "LocalTime",
    "Mask",
    "MaskBox",
    "Model",
    "ModelInfo",
    "RadarCube",
    "RadarInfo",
    "Track",
    "Vibration",
]


class Date:
    """``edgefirst_msgs.Date`` — calendar date (year/month/day) used by
    ``LocalTime``. CdrFixed (4-byte payload).
    """

    def __init__(self, year: int = 1970, month: int = 1, day: int = 1) -> None: ...

    @property
    def year(self) -> int: ...
    @property
    def month(self) -> int: ...
    @property
    def day(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Date: ...
    def __repr__(self) -> str: ...


class DmaBuffer:
    """``edgefirst_msgs.DmaBuffer`` — DMA-buf reference (header + 7 small
    u32/i32 metadata fields).

    .. deprecated:: 3.1.0
        Use :class:`CameraFrame` for multi-plane support, colorimetry,
        GPU fences, and off-device bridging. Kept here for bench parity
        with pycdr2.
    """

    def __init__(
        self,
        header: Header,
        pid: int,
        fd: int,
        width: int,
        height: int,
        stride: int,
        fourcc: int,
        length: int,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def pid(self) -> int: ...
    @property
    def fd(self) -> int:
        """Linux file-descriptor number for the DMA-buf."""

    @property
    def width(self) -> int: ...
    @property
    def height(self) -> int: ...
    @property
    def stride(self) -> int: ...
    @property
    def fourcc(self) -> int:
        """Pixel format in V4L2 fourcc encoding (e.g. 0x56595559 = 'YUYV')."""

    @property
    def length(self) -> int:
        """Total payload size in bytes."""

    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> DmaBuffer: ...
    def __repr__(self) -> str: ...


class Mask:
    """``edgefirst_msgs.Mask`` — segmentation mask (H × W × L bytes).

    ``encoding`` is ``""`` for raw uint8 or ``"zstd"`` for zstd-compressed
    payloads. ``mask`` exposes the bytes via :class:`BorrowedBuf` for
    zero-copy numpy access::

        arr = np.frombuffer(mask.mask, dtype=np.uint8).reshape(L, H, W)

    On ``abi3-py38`` use ``mask.mask.view()`` instead of ``np.frombuffer``.
    """

    def __init__(
        self,
        height: int,
        width: int,
        length: int,
        encoding: str,
        mask: BufferLike,
        boxed: bool = False,
    ) -> None: ...

    @property
    def height(self) -> int: ...
    @property
    def width(self) -> int: ...
    @property
    def length(self) -> int:
        """Number of channels (depth dimension of the mask tensor)."""

    @property
    def encoding(self) -> str: ...
    @property
    def boxed(self) -> bool: ...
    @property
    def cdr_size(self) -> int: ...

    @property
    def mask(self) -> BorrowedBuf:
        """Zero-copy view of the mask bytes (H × W × L uint8)."""

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Mask: ...
    def __repr__(self) -> str: ...


class RadarCube:
    """``edgefirst_msgs.RadarCube`` — radar tensor with typed metadata
    arrays and an int16 cube payload.

    Bulk-array accessors return :class:`BorrowedBuf`; reinterpret with
    the documented dtype:

    - ``layout``  → bytes               (1 byte per axis index)
    - ``shape``   → ``np.uint16``        (number of bins per axis)
    - ``scales``  → ``np.float32``       (real-world scale per axis)
    - ``cube``    → ``np.int16``         (interleaved I/Q samples)

    Example
    -------
    ::

        cube = RadarCube(
            header=Header(stamp=Time(1, 0), frame_id="radar"),
            timestamp=1234567890123456,
            layout=np.array([6, 1, 5, 2], dtype=np.uint8),
            shape=np.array([2, 128, 12, 128], dtype=np.uint16),
            scales=np.array([1.0, 0.117, 1.0, 0.156], dtype=np.float32),
            cube=np.zeros(2 * 128 * 12 * 128, dtype=np.int16),
            is_complex=True,
        )

        cube_view = np.frombuffer(cube.cube, dtype=np.int16)
        shape    = np.frombuffer(cube.shape, dtype=np.uint16)
        cube_arr = cube_view.reshape(*shape)
    """

    def __init__(
        self,
        header: Header,
        timestamp: int,
        layout: BufferLike,
        shape: BufferLike,
        scales: BufferLike,
        cube: BufferLike,
        is_complex: bool = False,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def timestamp(self) -> int:
        """Sensor-supplied microsecond timestamp (radar ASIC clock)."""

    @property
    def is_complex(self) -> bool: ...
    @property
    def cdr_size(self) -> int: ...

    @property
    def layout(self) -> BorrowedBuf:
        """Layout codes — uint8 sequence; one entry per axis identifying
        SEQUENCE / RANGE / RX_CHANNEL / DOPPLER (see radarpub docs).
        """

    @property
    def shape(self) -> BorrowedBuf:
        """Shape vector — `np.frombuffer(..., dtype=np.uint16)`."""

    @property
    def scales(self) -> BorrowedBuf:
        """Per-axis scales — `np.frombuffer(..., dtype=np.float32)`."""

    @property
    def cube(self) -> BorrowedBuf:
        """Cube data — `np.frombuffer(..., dtype=np.int16)`."""

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> RadarCube: ...
    def __repr__(self) -> str: ...


class LocalTime:
    """``edgefirst_msgs.LocalTime`` — calendar time anchor used by the
    EdgeFirst Publisher to align multiple MCAP recordings.

    `time` is the offset from `date` (typically wall-clock time of day).
    `timezone` is the offset from UTC in minutes (e.g. -300 for UTC-5,
    +330 for India Standard Time).
    """

    def __init__(
        self,
        header: Header,
        date: Optional["Date"] = None,
        time: Optional[Time] = None,
        timezone: int = 0,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def date(self) -> "Date": ...
    @property
    def time(self) -> Time: ...
    @property
    def timezone(self) -> int: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> LocalTime: ...


class Track:
    """``edgefirst_msgs.Track`` — object-tracking record (no header).

    `id` is empty when the object isn't being tracked. `lifetime` counts
    consecutive frames the track has been seen. `created` is the
    timestamp of the first sighting.
    """

    def __init__(
        self,
        id: str = "",
        lifetime: int = 0,
        created: Optional[Time] = None,
    ) -> None: ...

    @property
    def id(self) -> str: ...
    @property
    def lifetime(self) -> int: ...
    @property
    def created(self) -> Time: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Track: ...


class DetectBox:
    """``edgefirst_msgs.DetectBox`` — single bounding box within a
    :class:`Detect` message. Not CDR-serializable on its own.
    """

    def __init__(
        self,
        label: str = "",
        score: float = 0.0,
        center_x: float = 0.0,
        center_y: float = 0.0,
        width: float = 0.0,
        height: float = 0.0,
        distance: float = 0.0,
        speed: float = 0.0,
        track_id: str = "",
        track_lifetime: int = 0,
        track_created: Optional[Time] = None,
    ) -> None: ...

    @property
    def label(self) -> str: ...
    @property
    def score(self) -> float: ...
    @property
    def center_x(self) -> float: ...
    @property
    def center_y(self) -> float: ...
    @property
    def width(self) -> float: ...
    @property
    def height(self) -> float: ...
    @property
    def distance(self) -> float: ...
    @property
    def speed(self) -> float: ...
    @property
    def track_id(self) -> str: ...
    @property
    def track_lifetime(self) -> int: ...
    @property
    def track_created(self) -> Time: ...


class Detect:
    """``edgefirst_msgs.Detect`` — detection result with header + boxes.

    Carries a sequence of :class:`DetectBox` results plus timing metadata.
    """

    def __init__(
        self,
        header: Header,
        input_timestamp: int = 0,
        model_time: int = 0,
        output_time: int = 0,
        boxes: Optional[Sequence[DetectBox]] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def input_timestamp(self) -> int: ...
    @property
    def model_time(self) -> int: ...
    @property
    def output_time(self) -> int: ...
    @property
    def boxes(self) -> List[DetectBox]: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Detect: ...


class MaskBox:
    """``edgefirst_msgs.MaskBox`` — single segmentation mask entry within
    a :class:`Model` message.
    """

    @property
    def height(self) -> int: ...
    @property
    def width(self) -> int: ...
    @property
    def length(self) -> int: ...
    @property
    def encoding(self) -> str: ...
    @property
    def mask(self) -> bytes: ...
    @property
    def boxed(self) -> bool: ...


class CameraPlane:
    """``edgefirst_msgs.CameraPlane`` — single DMA plane within a
    :class:`CameraFrame`.
    """

    def __init__(
        self,
        fd: int = -1,
        offset: int = 0,
        stride: int = 0,
        size: int = 0,
        used: int = 0,
        data: Optional[bytes] = None,
    ) -> None: ...

    @property
    def fd(self) -> int: ...
    @property
    def offset(self) -> int: ...
    @property
    def stride(self) -> int: ...
    @property
    def size(self) -> int: ...
    @property
    def used(self) -> int: ...
    @property
    def data(self) -> bytes: ...


class CameraFrame:
    """``edgefirst_msgs.CameraFrame`` — multi-plane camera frame with
    colorimetry and GPU fence metadata.
    """

    def __init__(
        self,
        header: Header,
        seq: int = 0,
        pid: int = 0,
        width: int = 0,
        height: int = 0,
        format: str = "",
        color_space: str = "",
        color_transfer: str = "",
        color_encoding: str = "",
        color_range: str = "",
        fence_fd: int = -1,
        planes: Optional[Sequence[CameraPlane]] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def seq(self) -> int: ...
    @property
    def pid(self) -> int: ...
    @property
    def width(self) -> int: ...
    @property
    def height(self) -> int: ...
    @property
    def format(self) -> str: ...
    @property
    def color_space(self) -> str: ...
    @property
    def color_transfer(self) -> str: ...
    @property
    def color_encoding(self) -> str: ...
    @property
    def color_range(self) -> str: ...
    @property
    def fence_fd(self) -> int: ...
    @property
    def num_planes(self) -> int: ...
    @property
    def planes(self) -> List[CameraPlane]: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> CameraFrame: ...


class Model:
    """``edgefirst_msgs.Model`` — full model inference result with boxes
    and segmentation masks.
    """

    def __init__(
        self,
        header: Header,
        input_time: int = 0,
        model_time: int = 0,
        output_time: int = 0,
        decode_time: int = 0,
        boxes: Optional[Sequence[DetectBox]] = None,
        masks: Optional[Sequence[MaskBox]] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def input_time(self) -> int: ...
    @property
    def model_time(self) -> int: ...
    @property
    def output_time(self) -> int: ...
    @property
    def decode_time(self) -> int: ...
    @property
    def boxes(self) -> List[DetectBox]: ...
    @property
    def masks(self) -> List[MaskBox]: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Model: ...


class ModelInfo:
    """``edgefirst_msgs.ModelInfo`` — model metadata (name, type, format,
    shape, labels).
    """

    def __init__(
        self,
        header: Header,
        input_shape: Optional[Sequence[int]] = None,
        input_type: int = 0,
        output_shape: Optional[Sequence[int]] = None,
        output_type: int = 0,
        labels: Optional[Sequence[str]] = None,
        model_type: str = "",
        model_format: str = "",
        model_name: str = "",
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def input_shape(self) -> List[int]: ...
    @property
    def input_type(self) -> int: ...
    @property
    def output_shape(self) -> List[int]: ...
    @property
    def output_type(self) -> int: ...
    @property
    def labels(self) -> List[str]: ...
    @property
    def model_type(self) -> str: ...
    @property
    def model_format(self) -> str: ...
    @property
    def model_name(self) -> str: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> ModelInfo: ...


class RadarInfo:
    """``edgefirst_msgs.RadarInfo`` — radar configuration parameters."""

    def __init__(
        self,
        header: Header,
        center_frequency: str = "",
        frequency_sweep: str = "",
        range_toggle: str = "",
        detection_sensitivity: str = "",
        cube: bool = False,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def center_frequency(self) -> str: ...
    @property
    def frequency_sweep(self) -> str: ...
    @property
    def range_toggle(self) -> str: ...
    @property
    def detection_sensitivity(self) -> str: ...
    @property
    def cube(self) -> bool: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> RadarInfo: ...


class Vibration:
    """``edgefirst_msgs.Vibration`` — vibration measurement with frequency band."""

    def __init__(
        self,
        header: Header,
        vibration: Optional[Vector3] = None,
        band_lower_hz: float = 0.0,
        band_upper_hz: float = 0.0,
        measurement_type: int = 0,
        unit: int = 0,
        clipping: Optional[Vector3] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def vibration(self) -> Vector3: ...
    @property
    def band_lower_hz(self) -> float: ...
    @property
    def band_upper_hz(self) -> float: ...
    @property
    def measurement_type(self) -> int: ...
    @property
    def unit(self) -> int: ...
    @property
    def clipping(self) -> Vector3: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Vibration: ...
