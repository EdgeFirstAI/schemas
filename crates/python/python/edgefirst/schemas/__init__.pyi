# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for `edgefirst.schemas` (the pyo3-backed CDR codec).

The package re-exports submodules organised by ROS 2 / Foxglove / EdgeFirst
namespaces; bulk byte payloads (image data, mask bytes, radar cubes, point
clouds, compressed video) are returned via :class:`BorrowedBuf` for
zero-copy access:

* On Python 3.11+ or non-``abi3`` builds, ``BorrowedBuf`` implements the
  buffer protocol — ``np.frombuffer(buf, dtype=...)`` returns a typed
  ndarray aliasing the parent's bytes, no copy.
* On ``abi3-py38`` builds, the buffer protocol isn't in the limited API;
  use ``buf.view()`` to get a ``memoryview`` instead. The same zero-copy
  semantics apply, but you have to specify the dtype manually when
  reshaping into a numpy array.

Every CDR message type is either a ``CdrFixed`` value (small struct,
fixed wire size, exposed as a frozen pyclass with ``to_bytes()`` /
``from_cdr()``) or a buffer-backed type (header + offset table over an
owned ``Vec<u8>``, exposed with O(1) field accessors plus
``to_bytes()`` / ``from_cdr()``).
"""

from __future__ import annotations
from typing import Any, Union

from . import (
    builtin_interfaces as builtin_interfaces,
    edgefirst_msgs as edgefirst_msgs,
    foxglove_msgs as foxglove_msgs,
    geometry_msgs as geometry_msgs,
    nav_msgs as nav_msgs,
    rosgraph_msgs as rosgraph_msgs,
    sensor_msgs as sensor_msgs,
    std_msgs as std_msgs,
)

__version__: str

# Anything Python's buffer protocol can produce a contiguous byte view of.
# On non-abi3 / abi3-py311 builds, numpy arrays of any dtype are accepted —
# their byte payload is forwarded regardless of element type. On abi3-py38
# builds, typed numpy arrays must be passed via ``arr.tobytes()`` first.
BufferLike = Union[bytes, bytearray, memoryview]


class BorrowedBuf:
    """Zero-copy byte view over a parent message's buffer.

    The buffer remains valid for as long as this view is alive, which
    itself holds a strong reference to the parent message via the buffer
    protocol's ``obj`` slot. Use the buffer protocol
    (``np.frombuffer(buf)``, ``memoryview(buf)``) on Python 3.11+ or
    non-abi3 builds, or :meth:`view` on ``abi3-py38`` builds.
    """

    @property
    def parent(self) -> Any:
        """The owning message keeping this view's bytes alive."""

    def __len__(self) -> int: ...
    def __repr__(self) -> str: ...

    def tobytes(self) -> bytes:
        """Return a ``bytes`` copy of the view (one allocation)."""

    def view(self) -> memoryview:
        """Return a ``memoryview`` aliasing the parent's bytes (zero-copy).

        Always available — works on every Python ABI level. On Py 3.11+
        or non-``abi3`` builds, the buffer protocol
        (``np.frombuffer(buf)``) is preferred since it carries
        format/itemsize metadata; this ``view()`` helper is the
        ``abi3-py38`` fallback.
        """
