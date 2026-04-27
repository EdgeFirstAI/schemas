# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.edgefirst_msgs``."""

from __future__ import annotations

from typing import Optional

from . import BorrowedBuf, BufferLike
from .builtin_interfaces import Time
from .std_msgs import Header

__all__ = ["Date", "DmaBuffer", "LocalTime", "Mask", "RadarCube", "Track"]


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
