# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.std_msgs``."""

from __future__ import annotations
from typing import Optional

from . import BufferLike
from .builtin_interfaces import Time

__all__ = ["ColorRGBA", "Header"]


class Header:
    """``std_msgs.Header`` — ``stamp`` + ``frame_id``. Buffer-backed.

    Stored as an owned CDR buffer; field accessors do O(1) reads at
    known offsets. Decoding via :meth:`from_cdr` parses the buffer in a
    single pass and stamps a 1-element offset table.
    """

    def __init__(self, stamp: Optional[Time] = None, frame_id: str = "") -> None: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def cdr_size(self) -> int:
        """Size of the underlying CDR buffer in bytes."""

    def to_bytes(self) -> bytes:
        """Return the full CDR buffer as ``bytes`` (one copy)."""

    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Header: ...

    def __repr__(self) -> str: ...


class ColorRGBA:
    """``std_msgs.ColorRGBA`` — RGBA float32 colour. CdrFixed (16 bytes)."""

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
    def from_cdr(cls, buf: BufferLike) -> ColorRGBA: ...
    def __repr__(self) -> str: ...
