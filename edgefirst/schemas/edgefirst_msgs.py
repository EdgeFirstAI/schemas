# SPDX-License-Identifier: Apache-2.0
# Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

import warnings
from dataclasses import dataclass
from enum import Enum

from pycdr2 import IdlStruct
from pycdr2.types import (float32, int16, int32, sequence, uint8, uint16,
                          uint32, uint64)

from . import default_field
from .builtin_interfaces import Duration, Time
from .geometry_msgs import Vector3
from .std_msgs import Header

class model_info(Enum):
    RAW = 0
    INT8 = 1
    UINT8 = 2
    INT16 = 3
    UINT16 = 4
    FLOAT16 = 5
    INT32 = 6
    UINT32 = 7
    FLOAT32 = 8
    INT64 = 9
    UINT64 = 10
    FLOAT64 = 11
    STRING = 12

@dataclass
class Date(IdlStruct, typename='edgefirst_msgs/Date'):
    """
    The Date type holds the year, month, and day of the month.  It is used in
    the LocalTime message to represent the date of the local time.
    """
    year: uint16 = 0
    month: uint8 = 0
    day: uint8 = 0


@dataclass
class LocalTime(IdlStruct, typename='edgefirst_msgs/LocalTime'):
    """
    The local time interface publishes the current time on the device.  It is
    mainly intended to allow synchronization of multiple MCAP files by the
    EdgeFirst Publisher.  The idea is to calculate the offset from the
    timestamp in the header with the actual local time, then when multiple MCAP
    files have the local time topic recorded the relative offsets can then be
    calculated.
    """

    header: Header = default_field(Header)
    """Message header containing the timestamp and frame id."""

    date: Date = default_field(Date)
    """
    The base date from which the local time is calculated.  This could be an
    epoch such as the standard UNIX 1 January 1970 or it could be the current
    date.  To calculate the real local time both the date, time, and timezone
    are combined into a valid date and time.
    """

    time: Time = default_field(Time)
    """
    The time offset from the date.  If the date is the current day then the
    time is the normal time of day.  If the date is an epoch than many days
    will be represented in the time.
    """

    timezone: int16 = 0
    """
    The timezone offset in minutes from UTC of the time value.  The timezone
    would be +/- 720 (+/- 12 hours).  Minutes are used to allow for partial
    offsets such as Newfoundland in Canada which is UTC-210 (UTC-3h30).
    """


@dataclass
class Track(IdlStruct, typename='edgefirst_msgs/Track'):
    id: str = ''
    """
    Unique identifier for the object track, empty if the object is not tracked.
    """
    lifetime: int32 = 0
    """
    Number of consecutive frames the object has been tracked
    """
    created: Time = default_field(Time)
    """
    Time the track was first added
    """


@dataclass
class Box(IdlStruct, typename='edgefirst_msgs/Box'):
    center_x: float32 = 0
    """
    Normalized x-coordinate of the center
    """
    center_y: float32 = 0
    """
    Normalized y-coordinate of the center
    """
    width: float32 = 0
    """
    Normalized width of the box
    """
    height: float32 = 0
    """
    Normalized height of the box
    """
    label: str = ''
    """
    object label
    """
    score: float32 = 0
    """
    confidence score for detection
    """
    distance: float32 = 0
    """
    Distance of object (if known)
    """
    speed: float32 = 0
    """
    Speed of object (if known)
    """
    track: Track = default_field(Track)
    """
    object tracking, each track includes ID and lifetime information
    """


@dataclass
class CameraPlane(IdlStruct, typename='edgefirst_msgs/CameraPlane'):
    """
    Descriptor for a single image plane within a CameraFrame.

    Two exclusive delivery modes:
      * fd >= 0  : plane bytes live in DMA-BUF; consumer uses offset/size/used
                   to mmap. `data` MUST be empty.
      * fd == -1 : plane bytes are inlined in `data`; offset is ignored;
                   size and used describe `data` length.
    """
    fd: int32 = -1
    offset: uint32 = 0
    stride: uint32 = 0
    size: uint32 = 0
    used: uint32 = 0
    data: sequence[uint8] = default_field([])


@dataclass
class CameraFrame(IdlStruct, typename='edgefirst_msgs/CameraFrame'):
    """
    Multi-plane video frame reference.

    Carries DMA-BUF file descriptors and/or inlined bytes to one or more image
    planes, plus frame-level metadata. Supersedes the single-plane DmaBuffer
    message (deprecated in 3.1.0, removed in 4.0.0).

    Supports raw video, planar model inputs (NV12, I420, RGB planar NCHW),
    hardware codec bitstreams (H.264/H.265/MJPEG with used < size), GPU fence
    synchronization (fence_fd), and off-device bridging via per-plane inlined
    `data` with fd == -1.
    """
    header: Header = default_field(Header)
    seq: uint64 = 0
    pid: uint32 = 0
    width: uint32 = 0
    height: uint32 = 0
    format: str = ''
    color_space: str = ''
    color_transfer: str = ''
    color_encoding: str = ''
    color_range: str = ''
    fence_fd: int32 = -1
    planes: sequence[CameraPlane] = default_field([])


@dataclass
class Mask(IdlStruct, typename='edgefirst_msgs/Mask'):
    height: uint32 = 0
    """
    The height of the mask, 0 if this dimension is unused.
    """

    width: uint32 = 0
    """
    The width of the mask, 0 if this dimension is unused.
    """

    length: uint32 = 0
    """
    The length of the mask, 0 if this dimension is unused.  The length would
    be used in 3D masks to represent the depth.  It could also be used for 2D
    bird's eye view masks along with width instead of height (elevation).
    """

    encoding: str = ''
    """
    The optional encoding for the mask (currently unused).
    """

    mask: sequence[uint8] = default_field([])
    """
    The segmentation mask data.  The array should be reshaped according to the
    height, width, and length dimensions.  The dimension order is row-major.
    """

    boxed: bool = False
    """
    If this mask is associated to a corresponding box
    """


@dataclass
class Model(IdlStruct, typename='edgefirst_msgs/Model'):
    header: Header = default_field(Header)
    """
    Metadata including timestamp and coordinate frame
    """

    input_time: Duration = default_field(Duration)
    """
    Duration to load inputs into the model
    """

    model_time: Duration = default_field(Duration)
    """
    Duration to run the model, not including input/output/decoding
    """

    output_time: Duration = default_field(Duration)
    """
    Duration to read outputs from the model
    """

    decode_time: Duration = default_field(Duration)
    """
    Duration to decode the outputs from the model, including nms and tracking.
    """

    boxes: sequence[Box] = default_field([])
    """
    Array of detected object bounding boxes.
    """

    mask: sequence[Mask] = default_field([])
    """
    Segmentation masks from the model.  Empty array if model does not generate
    masks.  Generally models will only generate a single mask if they do.
    """

@dataclass
class ModelInfo(IdlStruct, typename='edgefirst_msgs/ModelInfo'):
    header: Header = default_field(Header)
    """
    Metadata including timestamp and coordinate frame
    """
    input_shape: sequence[uint32] = default_field([])
    """
    Shape of the input tensor(s) in the format "height,width,channels" or "height,width,depth,channels"
    """
    input_type: uint8 = model_info.RAW
    """
    Data type of the input tensor(s) (e.g., "float32", "uint8")
    """
    output_shape: sequence[uint32] = default_field([])
    """
    Shape of the output tensor(s) in the format "height,width,channels" or "height,width,depth,channels"
    """
    output_type: uint8 = model_info.RAW
    """
    Data type of the output tensor(s) (e.g., "float32", "uint8")
    """
    labels: sequence[str] = default_field([])
    """
    Array of strings representing the labels used by the model, empty if no labels available
    """
    model_type: str = ''
    """
    Model tasks/types (e.g., ["object_detection", "classification"])
    """
    model_format: str = ''
    """
    Format of the model (e.g., "DeepViewRT", "HailoRT", "RKNN", "TensorRT", "TFLite")
    """
    model_name: str = ''
    """
    Name of the model (if available), otherwise use filename without extension or path
    """

@dataclass
class Detect(IdlStruct, typename='Detect'):
    header: Header = default_field(Header)
    """
    Metadata including timestamp and coordinate frame
    """
    input_timestamp: Time = default_field(Time)
    """
    Timestamp of the input data (e.g., from camera)
    """
    model_time: Time = default_field(Time)
    """
    Timestamp when the object was processed by the model
    """
    output_time: Time = default_field(Time)
    """
    Timestamp when the processed output was available
    """
    boxes: sequence[Box] = default_field([])
    """
    Array of detected object bounding boxes
    """


_DMABUFFER_DEPRECATION = (
    "edgefirst_msgs.DmaBuffer is deprecated since 3.1.0 and will be removed "
    "in 4.0.0; use edgefirst_msgs.CameraFrame instead."
)


@dataclass
class DmaBuffer(IdlStruct, typename='DmaBuffer'):
    """DEPRECATED since 3.1.0; use :class:`CameraFrame` instead.

    Removed in 4.0.0. CameraFrame adds multi-plane support (NV12, I420, planar
    RGB NCHW), compressed bitstream handling (used < size), GPU fence
    synchronization, frame sequence counter, colorimetry metadata, and an
    off-device bridge path via inlined per-plane bytes.

    Instantiation and decoding via :meth:`deserialize` emit a
    :class:`DeprecationWarning`.
    """

    def __post_init__(self):
        warnings.warn(_DMABUFFER_DEPRECATION, DeprecationWarning, stacklevel=2)
    header: Header = default_field(Header)
    """
    Metadata including timestamp and coordinate frame
    """
    pid: uint32 = 0
    """
    The process id of the service that created the DMA buffer
    """
    fd: int32 = 0
    """
    The file descriptor of the DMA buffer
    """
    width: uint32 = 0
    """
    The width of the image in pixels
    """
    height: uint32 = 0
    """
    The height of the image in pixels
    """
    stride: uint32 = 0
    """
    The stride of the image in bytes
    """
    fourcc: uint32 = 0
    """
    The fourcc code of the image
    """
    length: uint32 = 0
    """
    The length of the DMA buffer in bytes, used to mmap the buffer
    """


class RadarChannel(Enum):
    UNDEFINED = 0
    RANGE = 1
    DOPPLER = 2
    AZIMUTH = 3
    ELEVATION = 4
    RXCHANNEL = 5
    SEQUENCE = 6


@dataclass
class RadarCube(IdlStruct, typename='edgefirst_msgs/RadarCube'):
    """
    The RadarCube interface carries various radar cube reprensentations of the
    Radar FFT before generally being processed by CFAR into a point cloud.  The
    cube coud be R, RD, RAD, RA, and so on where R=Range, D=Dopper, and
    A=Azimuth.

    Dimensional labels are used to describe the radar cube layout.  Not all
    cubes include every label.  Undefined is used for dimensions not covered by
    this list.
    """

    header: Header = default_field(Header)
    """Message header containing the timestamp and frame id."""

    timestamp: uint64 = 0
    """Radar frame timestamp generated on the radar module"""

    layout: sequence[uint8] = default_field([])
    """Radar cube layout provides labels for each dimensions"""

    shape: sequence[uint16] = default_field([])
    """Radar cube shape provides the shape of each dimensions"""

    scales: sequence[float32] = default_field([])
    """
    The scaling factors for the dimensions representing bins.  For dimensions
    taken "as-is" the scale will be 1.0.
    """

    cube: sequence[int16] = default_field([])
    """
    The radar cube data as 16bit integers.  If the is_complex is true then each
    element will be pairs of integers with the first being real and the second
    being imaginary.
    """

    is_complex: bool = False
    """
    True if the radar cube is complex in which case the final dimension will be
    doubled in size to account for the pair of int16 elements representing
    [real,imaginary].
    """


@dataclass
class RadarInfo(IdlStruct, typename='edgefirst_msgs/RadarInfo'):
    """
    The RadarInfo interface carries the current radar configuration and status.
    """

    header: Header = default_field(Header)
    """Message header containing the timestamp and frame id."""

    center_frequency: str = ''
    """Radar center frequency band."""

    frequency_sweep: str = ''
    """The frequency sweep controls the detection range of the radar."""

    range_toggle: str = ''
    """
    The range-toggle mode allows the radar to alternate between various
    frequency sweep configurations.  Applications must handle range toggling as
    targets are not consistent between messages as the frequency alternates.
    """

    detection_sensitivity: str = ''
    """
    The detection sensitivity controls the sensitivity to recognize a target.
    """

    cube: bool = False
    """True if the radar is configured to output radar cubes."""


class VibrationMeasurement(Enum):
    """measurement_type enum values for Vibration."""
    MEASUREMENT_UNKNOWN = 0
    MEASUREMENT_RMS = 1
    MEASUREMENT_PEAK = 2
    MEASUREMENT_PEAK_TO_PEAK = 3


class VibrationUnit(Enum):
    """unit enum values for Vibration."""
    UNIT_UNKNOWN = 0
    UNIT_ACCEL_M_PER_S2 = 1
    UNIT_ACCEL_G = 2
    UNIT_VELOCITY_MM_PER_S = 3
    UNIT_DISPLACEMENT_UM = 4
    UNIT_VELOCITY_IN_PER_S = 5
    UNIT_DISPLACEMENT_MIL = 6


@dataclass
class Vibration(IdlStruct, typename='edgefirst_msgs/Vibration'):
    """
    Per-axis vibration magnitude with explicit measurement semantics
    (RMS/Peak/Peak-to-Peak), physical unit, and integration band.

    Generalizes MAVLink VIBRATION, ArduPilot VIBE, PX4 vehicle_imu_status,
    and industrial ISO 10816/20816 broadband vibration sensors.

    Unknown scalars: NaN. Unknown enums: *_UNKNOWN (value 0).
    Publishers without clipping source: clipping = [] (empty).
    MAVLink publishers emit exactly 3 clipping entries (clipping_0/1/2).
    """
    header: Header = default_field(Header)
    measurement_type: uint8 = 0
    """Broadband statistic reported in `vibration` (see VibrationMeasurement)."""
    unit: uint8 = 0
    """Physical unit of `vibration` (see VibrationUnit)."""
    band_lower_hz: float32 = 0.0
    """Integration band lower bound in Hz. NaN = unknown."""
    band_upper_hz: float32 = 0.0
    """Integration band upper bound in Hz. NaN = unknown."""
    vibration: Vector3 = default_field(Vector3)
    """Per-axis vibration magnitude in the chosen unit/statistic."""
    clipping: sequence[uint32] = default_field([])
    """Per-axis accelerometer saturation counters (monotonic)."""


# Schema registry support
_TYPES = {
    "Box": Box,
    "CameraFrame": CameraFrame,
    "CameraPlane": CameraPlane,
    "Date": Date,
    "Detect": Detect,
    "DmaBuffer": DmaBuffer,
    "LocalTime": LocalTime,
    "Mask": Mask,
    "Model": Model,
    "ModelInfo": ModelInfo,
    "RadarCube": RadarCube,
    "RadarInfo": RadarInfo,
    "Track": Track,
    "Vibration": Vibration,
}


def is_type_supported(type_name: str) -> bool:
    """Check if a type name is supported by this module."""
    return type_name in _TYPES


def list_types() -> list[str]:
    """List all type schema names in this module."""
    return [f"edgefirst_msgs/msg/{name}" for name in sorted(_TYPES.keys())]


def get_type(type_name: str) -> type:
    """Get the type class by name. Returns None if not found."""
    return _TYPES.get(type_name)
