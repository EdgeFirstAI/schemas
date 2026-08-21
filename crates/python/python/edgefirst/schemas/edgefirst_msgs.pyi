# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.edgefirst_msgs``."""

from __future__ import annotations

from typing import List, Optional, Sequence

from . import BorrowedBuf, BufferLike
from .builtin_interfaces import Duration, Time
from .std_msgs import Header
from .geometry_msgs import Vector3

__all__ = [
    "Box",
    "CameraFrame",
    "Date",
    "Detect",
    "DetectBox",
    "LocalTime",
    "Mask",
    "MaskBox",
    "Model",
    "ModelInfo",
    "RadarCube",
    "RadarInfo",
    "Tensor",
    "TensorPlane",
    "TensorStamped",
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
        center_x: float = 0.0,
        center_y: float = 0.0,
        width: float = 0.0,
        height: float = 0.0,
        label: str = "",
        score: float = 0.0,
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


# Backwards-compatibility alias for the pre-3.2.0 name. `Box` was renamed
# `DetectBox` when the module became a pyo3 binding (avoids shadowing Rust's
# `std::boxed::Box`); the old name remains a resolvable alias of the same class.
Box = DetectBox


class Detect:
    """``edgefirst_msgs.Detect`` — detection result with header + boxes.

    Carries a sequence of :class:`DetectBox` results plus timing metadata.
    """

    def __init__(
        self,
        header: Header,
        input_timestamp: Optional[Time] = None,
        model_time: Optional[Time] = None,
        output_time: Optional[Time] = None,
        boxes: Optional[Sequence[DetectBox]] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def input_timestamp(self) -> Time: ...
    @property
    def model_time(self) -> Time: ...
    @property
    def output_time(self) -> Time: ...
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

    def __init__(
        self,
        height: int = 0,
        width: int = 0,
        length: int = 0,
        encoding: str = "",
        mask: Optional[bytes] = None,
        boxed: bool = False,
    ) -> None: ...

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


class Model:
    """``edgefirst_msgs.Model`` — full model inference result with boxes
    and segmentation masks.
    """

    def __init__(
        self,
        header: Header,
        input_time: Optional[Duration] = None,
        model_time: Optional[Duration] = None,
        output_time: Optional[Duration] = None,
        decode_time: Optional[Duration] = None,
        boxes: Optional[Sequence[DetectBox]] = None,
        masks: Optional[Sequence[MaskBox]] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def input_time(self) -> Duration: ...
    @property
    def model_time(self) -> Duration: ...
    @property
    def output_time(self) -> Duration: ...
    @property
    def decode_time(self) -> Duration: ...
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
        clipping: Optional[Sequence[int]] = None,
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
    def clipping(self) -> List[int]: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Vibration: ...


class TensorPlane:
    """``edgefirst_msgs.TensorPlane`` — one plane of a :class:`Tensor`.

    Two mutually exclusive transport modes:

    - ``handle >= 0`` — the bytes live behind the platform handle and
      ``data`` is empty. This is the dma-buf / shared-memory path.
    - ``handle == -1`` — the bytes are inline in ``data``; ``is_inline`` is
      True, ``size == len(data)``, ``modifier == 0`` and ``handle_bytes`` is
      empty.

    A frame must not mix modes: all planes inline, or none. The tensor
    carries a single ``storage_kind``, ``pid`` and ``fence_fd`` covering
    every plane, so a mixed set has no coherent meaning and is rejected.

    Read back from a :class:`Tensor` this is a value copy, ``data``
    included. For a large inline payload prefer :meth:`Tensor.plane_data`,
    which returns a zero-copy view.
    """

    def __init__(
        self,
        handle: int = -1,
        offset: int = 0,
        stride: int = 0,
        size: int = 0,
        used: int = 0,
        modifier: int = 0,
        handle_bytes: Optional[bytes] = None,
        data: Optional[bytes] = None,
    ) -> None: ...

    @property
    def handle(self) -> int: ...
    @property
    def offset(self) -> int: ...
    @property
    def stride(self) -> int:
        """Row stride in BYTES."""
        ...
    @property
    def size(self) -> int:
        """Allocated size of the plane, in bytes."""
        ...
    @property
    def used(self) -> int:
        """Bytes actually populated; always ``<= size``."""
        ...
    @property
    def modifier(self) -> int:
        """Format modifier (tiling / compression); 0 for linear."""
        ...
    @property
    def handle_bytes(self) -> bytes: ...
    @property
    def data(self) -> bytes: ...
    @property
    def is_inline(self) -> bool: ...

    def __repr__(self) -> str: ...


class Tensor:
    """``edgefirst_msgs.Tensor`` — the unstamped tensor payload.

    Carries the element type, the addressing grid, optional quantization
    parameters, optional colorimetry, and one or more planes.

    ``shape`` is the addressing grid, NOT the byte layout: an NV12 frame
    carries ``shape == [h, w]`` with a U8 ``dtype`` against an ``h*w*3/2``
    allocation. It is deliberately never validated against any buffer size.
    ``strides`` is in BYTES, and is either empty or exactly as long as
    ``shape``.

    ``quant_axis`` selects which shape the quantization parameters take,
    and the encoder enforces the match:

    - ``-2`` unquantized — ``quant_scales`` must be empty
    - ``-1`` per-tensor — exactly one scale
    - ``>= 0`` per-axis — exactly ``shape[quant_axis]`` scales

    ``quant_zero_points`` is either empty or the same length as
    ``quant_scales``. Colorimetry may only be set when ``format`` is.

    Note that ``fence_fd`` and ``quant_axis`` default to ``-1`` and ``-2``
    respectively — the schema's "absent" values, not zero.

    Example
    -------
    ::

        t = Tensor(
            storage_kind=2,
            pid=os.getpid(),
            dtype=1,
            shape=[480, 640],
            strides=[640, 1],
            format="NV12",
            color_space="bt709",
            color_range="limited",
            planes=[
                TensorPlane(handle=fd, offset=0, stride=640,
                            size=640 * 480, used=640 * 480),
                TensorPlane(handle=fd, offset=640 * 480, stride=640,
                            size=640 * 480 // 2, used=640 * 480 // 2),
            ],
        )
    """

    def __init__(
        self,
        storage_kind: int = 0,
        pid: int = 0,
        fence_fd: int = -1,
        dtype: int = 0,
        quant_axis: int = -2,
        shape: Optional[Sequence[int]] = None,
        strides: Optional[Sequence[int]] = None,
        quant_scales: Optional[Sequence[float]] = None,
        quant_zero_points: Optional[Sequence[int]] = None,
        format: str = "",
        color_space: str = "",
        color_transfer: str = "",
        color_encoding: str = "",
        color_range: str = "",
        planes: Optional[Sequence[TensorPlane]] = None,
    ) -> None: ...

    @property
    def storage_kind(self) -> int:
        """Storage class shared by every plane (HAL ``storage_kind`` codes)."""
        ...
    @property
    def pid(self) -> int:
        """Producer PID, for handle resolution; 0 when not applicable."""
        ...
    @property
    def fence_fd(self) -> int:
        """ACQUIRE fence fd; ``-1`` when there is no fence."""
        ...
    @property
    def dtype(self) -> int:
        """Element type (HAL ``dtype`` codes)."""
        ...
    @property
    def quant_axis(self) -> int: ...
    @property
    def shape(self) -> List[int]: ...
    @property
    def strides(self) -> List[int]: ...
    @property
    def quant_scales(self) -> List[float]: ...
    @property
    def quant_zero_points(self) -> List[int]: ...
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
    def num_planes(self) -> int: ...
    @property
    def planes(self) -> List[TensorPlane]: ...
    @property
    def cdr_size(self) -> int: ...

    def plane_data(self, index: int) -> BorrowedBuf:
        """Zero-copy view of one plane's inline bytes.

        :attr:`planes` copies each plane's ``data``; this does not::

            arr = np.frombuffer(t.plane_data(0), dtype=np.uint8)

        Returns an empty view for a plane whose bytes travel behind a
        handle. Raises :class:`ValueError` if ``index`` is out of range.
        """
        ...

    def to_standalone_cdr(self) -> bytes:
        """Re-head this tensor as a standalone ``Tensor`` CDR message.

        The republish path — forwarding a camera frame's tensor onto a
        tensor topic. Copies metadata only; plane payloads stay behind
        their handles. Because the layout is position-independent the
        result is byte-identical to encoding the same tensor standalone
        from scratch.
        """
        ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Tensor: ...
    def __repr__(self) -> str: ...


class TensorStamped:
    """``edgefirst_msgs.TensorStamped`` — a timestamped tensor, for model input and output topics.

    A header (``stamp``, ``frame_id``), a ``seq``, and an embedded
    :class:`Tensor` reached through :attr:`tensor`.

    ``seq`` is more than drop detection: it is a uint64, so it forces the
    embedded tensor to an 8-aligned offset regardless of ``frame_id``
    length. That is what makes the nested tensor byte-identical in every
    wrapper — and what lets :meth:`Tensor.to_standalone_cdr` re-head an
    embedded tensor without re-encoding it.
    """

    def __init__(
        self,
        stamp: Optional[Time] = None,
        frame_id: str = "",
        seq: int = 0,
        tensor: Optional[Tensor] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def seq(self) -> int: ...
    @property
    def tensor(self) -> Tensor:
        """The embedded tensor, sharing this message's buffer.

        For a message decoded from ``bytes`` this shares the underlying
        object outright — no copy. For one just constructed in-process the
        metadata is duplicated; plane payloads travel behind handles either
        way, so nothing frame-sized is copied.
        """
        ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> TensorStamped: ...
    def __repr__(self) -> str: ...


class CameraFrame:
    """``edgefirst_msgs.CameraFrame`` — a timestamped camera frame, carried as a tensor.

    A header (``stamp``, ``frame_id``), a ``seq``, and an embedded
    :class:`Tensor` reached through :attr:`tensor`.

    ``seq`` is more than drop detection: it is a uint64, so it forces the
    embedded tensor to an 8-aligned offset regardless of ``frame_id``
    length. That is what makes the nested tensor byte-identical in every
    wrapper — and what lets :meth:`Tensor.to_standalone_cdr` re-head an
    embedded tensor without re-encoding it.
    """

    def __init__(
        self,
        stamp: Optional[Time] = None,
        frame_id: str = "",
        seq: int = 0,
        tensor: Optional[Tensor] = None,
    ) -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def seq(self) -> int: ...
    @property
    def tensor(self) -> Tensor:
        """The embedded tensor, sharing this message's buffer.

        For a message decoded from ``bytes`` this shares the underlying
        object outright — no copy. For one just constructed in-process the
        metadata is duplicated; plane payloads travel behind handles either
        way, so nothing frame-sized is copied.
        """
        ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> CameraFrame: ...
    def __repr__(self) -> str: ...

