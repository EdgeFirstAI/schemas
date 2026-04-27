# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.builtin_interfaces``."""

from __future__ import annotations
from typing import Optional

from . import BufferLike

__all__ = ["Duration", "Time"]


class Time:
    """``builtin_interfaces.Time`` — POSIX-style timestamp.

    Wire format: 12 bytes total (4-byte CDR encapsulation header +
    ``i32 sec`` + ``u32 nanosec``). Negative ``sec`` represents pre-epoch
    timestamps; :meth:`to_nanos` returns ``None`` in that case since the
    value can't be represented as ``u64``.
    """

    def __init__(self, sec: int = 0, nanosec: int = 0) -> None: ...

    @staticmethod
    def from_nanos(nanos: int) -> Time:
        """Construct from a 64-bit nanosecond count."""

    def to_nanos(self) -> Optional[int]:
        """Convert to nanoseconds, or ``None`` for pre-epoch (negative ``sec``)."""

    @property
    def sec(self) -> int: ...
    @property
    def nanosec(self) -> int: ...

    def to_bytes(self) -> bytes:
        """Encode as a standalone CDR buffer (12 bytes)."""

    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Time:
        """Decode a standalone CDR ``Time`` buffer."""

    def __eq__(self, other: object) -> bool: ...
    def __hash__(self) -> int: ...
    def __repr__(self) -> str: ...


class Duration:
    """``builtin_interfaces.Duration`` — signed time span (sec + nanosec).

    Same wire format as :class:`Time`.
    """

    def __init__(self, sec: int = 0, nanosec: int = 0) -> None: ...

    @property
    def sec(self) -> int: ...
    @property
    def nanosec(self) -> int: ...

    def to_bytes(self) -> bytes: ...
    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Duration: ...
    def __eq__(self, other: object) -> bool: ...
    def __hash__(self) -> int: ...
    def __repr__(self) -> str: ...
