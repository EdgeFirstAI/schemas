# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.foxglove_msgs``."""

from __future__ import annotations

from . import BorrowedBuf, BufferLike
from .builtin_interfaces import Time

__all__ = ["Color", "CompressedVideo", "Point2"]


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
