# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.foxglove_msgs``."""

from __future__ import annotations

from typing import List, Optional, Sequence

from . import BorrowedBuf, BufferLike
from .builtin_interfaces import Time

__all__ = [
    "CircleAnnotations",
    "Color",
    "CompressedVideo",
    "ImageAnnotation",
    "Point2",
    "PointAnnotation",
    "TextAnnotation",
]


class CompressedVideo:
    """``foxglove_msgs.CompressedVideo`` — H.264 / H.265 NAL-unit frames
    with timestamp + format string.

    Mirrors the Foxglove schema used for browser-side video playback.
    """

    def __init__(
        self,
        timestamp: Time,
        frame_id: str,
        data: BufferLike,
        format: str,
    ) -> None: ...

    @property
    def timestamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def format(self) -> str:
        """Codec identifier — typically ``"h264"`` or ``"h265"``."""

    @property
    def cdr_size(self) -> int: ...

    @property
    def data(self) -> BorrowedBuf:
        """Zero-copy view of the encoded video payload."""

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> CompressedVideo: ...
    def __repr__(self) -> str: ...


class Point2:
    """``foxglove_msgs.Point2`` — 2D point (f64 x, y). CdrFixed (16 bytes)."""

    def __init__(self, x: float = 0.0, y: float = 0.0) -> None: ...
    @property
    def x(self) -> float: ...
    @property
    def y(self) -> float: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Point2: ...


class Color:
    """``foxglove_msgs.Color`` — RGBA float64 colour. CdrFixed (32 bytes).

    Note: this differs from :class:`std_msgs.ColorRGBA` (which uses f32).
    """

    def __init__(
        self, r: float = 0.0, g: float = 0.0, b: float = 0.0, a: float = 0.0
    ) -> None: ...

    @property
    def r(self) -> float: ...
    @property
    def g(self) -> float: ...
    @property
    def b(self) -> float: ...
    @property
    def a(self) -> float: ...
    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Color: ...


class CircleAnnotations:
    """``foxglove_msgs.CircleAnnotations`` — circle annotation overlay.
    CdrFixed.
    """

    def __init__(
        self,
        timestamp: Optional[Time] = None,
        position: Optional[Point2] = None,
        diameter: float = 0.0,
        thickness: float = 0.0,
        fill_color: Optional[Color] = None,
        outline_color: Optional[Color] = None,
    ) -> None: ...

    @property
    def timestamp(self) -> Time: ...
    @property
    def position(self) -> Point2: ...
    @property
    def diameter(self) -> float: ...
    @property
    def thickness(self) -> float: ...
    @property
    def fill_color(self) -> Color: ...
    @property
    def outline_color(self) -> Color: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> CircleAnnotations: ...


class TextAnnotation:
    """``foxglove_msgs.TextAnnotation`` — text overlay on an image.
    Buffer-backed.
    """

    def __init__(
        self,
        timestamp: Optional[Time] = None,
        position: Optional[Point2] = None,
        text: str = "",
        font_size: float = 0.0,
        text_color: Optional[Color] = None,
        background_color: Optional[Color] = None,
    ) -> None: ...

    @property
    def timestamp(self) -> Time: ...
    @property
    def position(self) -> Point2: ...
    @property
    def text(self) -> str: ...
    @property
    def font_size(self) -> float: ...
    @property
    def text_color(self) -> Color: ...
    @property
    def background_color(self) -> Color: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> TextAnnotation: ...


class PointAnnotation:
    """``foxglove_msgs.PointAnnotation`` — point cloud overlay on an image.
    Buffer-backed.
    """

    def __init__(
        self,
        timestamp: Optional[Time] = None,
        type_: int = 0,
        points: Optional[Sequence[Point2]] = None,
        outline_color: Optional[Color] = None,
        outline_colors: Optional[Sequence[Color]] = None,
        fill_color: Optional[Color] = None,
        thickness: float = 0.0,
    ) -> None: ...

    @property
    def timestamp(self) -> Time: ...
    @property
    def type_(self) -> int: ...
    @property
    def points(self) -> List[Point2]: ...
    @property
    def outline_color(self) -> Color: ...
    @property
    def outline_colors(self) -> List[Color]: ...
    @property
    def fill_color(self) -> Color: ...
    @property
    def thickness(self) -> float: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> PointAnnotation: ...


class ImageAnnotation:
    """``foxglove_msgs.ImageAnnotation`` — composite annotation with
    circles, points, and text overlays.
    """

    def __init__(
        self,
        circles: Optional[Sequence[CircleAnnotations]] = None,
        points: Optional[List[PointAnnotation]] = None,
        texts: Optional[List[TextAnnotation]] = None,
    ) -> None: ...

    @property
    def circles(self) -> List[CircleAnnotations]: ...
    @property
    def points(self) -> List[PointAnnotation]: ...
    @property
    def texts(self) -> List[TextAnnotation]: ...
    @property
    def cdr_size(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> ImageAnnotation: ...
