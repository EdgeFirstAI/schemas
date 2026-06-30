# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for zero-copy ``from_cdr`` input path.

The optimization:
- ``from_cdr(bytes)`` borrows the input zero-copy (no memcpy).
- ``from_cdr(bytearray)`` copies (GIL safety for mutable buffers).
- ``from_cdr(memoryview)`` copies (same reason).

These tests validate:
1. Correctness: from_cdr produces valid messages from both paths.
2. Isolation: mutable-source messages are unaffected by later mutations.
3. Zero-copy proof: bytes-backed messages share the underlying buffer.
4. Lifetime safety: messages remain valid after the source is unreferenced.
"""

import ctypes
import gc
import struct

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.nav_msgs import GridCells, OccupancyGrid, Path
from edgefirst.schemas.sensor_msgs import Image, CompressedImage, Imu, RelativeHumidity, TimeReference
from edgefirst.schemas.std_msgs import Header
from edgefirst.schemas.geometry_msgs import Point, TwistStamped


def _make_image_cdr() -> bytes:
    """Build a valid Image CDR buffer via the constructor path."""
    img = Image(
        header=Header(stamp=Time(42, 100), frame_id="cam0"),
        height=2,
        width=2,
        encoding="mono8",
        is_bigendian=0,
        step=2,
        data=b"\x01\x02\x03\x04",
    )
    return img.to_bytes()


def _make_imu_cdr() -> bytes:
    """Build a valid Imu CDR buffer."""
    imu = Imu(header=Header(stamp=Time(1, 0), frame_id="imu"))
    return imu.to_bytes()


def _make_twist_cdr() -> bytes:
    """Build a valid TwistStamped CDR buffer."""
    tw = TwistStamped(header=Header(stamp=Time(5, 500), frame_id="base"))
    return tw.to_bytes()


class TestFromCdrBytes:
    """from_cdr(bytes) should be zero-copy (no memcpy of the buffer)."""

    def test_image_from_bytes(self):
        cdr = _make_image_cdr()
        img = Image.from_cdr(cdr)
        assert img.height == 2
        assert img.width == 2
        assert img.encoding == "mono8"
        assert img.data.tobytes() == b"\x01\x02\x03\x04"

    def test_imu_from_bytes(self):
        cdr = _make_imu_cdr()
        imu = Imu.from_cdr(cdr)
        assert imu.frame_id == "imu"
        assert imu.stamp == Time(1, 0)

    def test_twist_from_bytes(self):
        cdr = _make_twist_cdr()
        tw = TwistStamped.from_cdr(cdr)
        assert tw.frame_id == "base"

    def test_zero_copy_shares_buffer(self):
        """Prove that from_cdr(bytes) aliases the input buffer.

        We mutate the underlying bytes memory via ctypes and observe the
        change through the message's accessor — proving the message reads
        directly from the input bytes without any intermediate copy.

        NOTE: mutating a `bytes` object is undefined behavior in Python,
        but it's the only way to prove true aliasing in a test. This test
        is purely diagnostic — production code must never mutate bytes.
        """
        cdr = _make_image_cdr()
        img = Image.from_cdr(cdr)

        # Read the original height (should be 2)
        assert img.height == 2

        # Get writable access to the bytes buffer via CPython internals.
        # PyBytes_AsString returns a char* to the internal buffer.
        PyBytes_AsString = ctypes.pythonapi.PyBytes_AsString
        PyBytes_AsString.restype = ctypes.POINTER(ctypes.c_ubyte)
        PyBytes_AsString.argtypes = [ctypes.py_object]
        buf = PyBytes_AsString(cdr)

        # Find the height field (u32 LE value 2) in the CDR buffer.
        # Scan for the pattern to be robust against header size changes.
        height_le = (2).to_bytes(4, "little")
        raw = bytes(cdr)
        offset = raw.find(height_le)
        assert offset >= 0, "Could not locate height field in CDR"

        # Overwrite height to 99 (u32 LE: 0x63 0x00 0x00 0x00)
        buf[offset] = 99
        buf[offset + 1] = 0
        buf[offset + 2] = 0
        buf[offset + 3] = 0

        # The message should now see height=99 — proving zero-copy aliasing
        assert img.height == 99

    def test_message_valid_after_source_unreferenced(self):
        """Message stays valid after the original bytes is GC'd."""
        cdr = _make_image_cdr()
        img = Image.from_cdr(cdr)
        # Delete our reference to the source bytes
        del cdr
        gc.collect()
        # Message should still be fully accessible
        assert img.height == 2
        assert img.width == 2
        assert img.encoding == "mono8"
        assert img.data.tobytes() == b"\x01\x02\x03\x04"
        assert img.stamp == Time(42, 100)

    def test_multiple_messages_from_same_bytes(self):
        """Multiple from_cdr calls on the same bytes are independent."""
        cdr = _make_image_cdr()
        img1 = Image.from_cdr(cdr)
        img2 = Image.from_cdr(cdr)
        assert img1.height == img2.height
        assert img1.data.tobytes() == img2.data.tobytes()


class TestFromCdrBytearray:
    """from_cdr(bytearray) should copy for GIL safety."""

    def test_image_from_bytearray(self):
        cdr = bytearray(_make_image_cdr())
        img = Image.from_cdr(cdr)
        assert img.height == 2
        assert img.encoding == "mono8"

    def test_mutation_after_from_cdr_does_not_affect_message(self):
        """After from_cdr(bytearray), mutating the bytearray is safe."""
        cdr = bytearray(_make_image_cdr())
        img = Image.from_cdr(cdr)
        # Corrupt the source
        cdr[:] = b"\x00" * len(cdr)
        # Message must be unaffected
        assert img.height == 2
        assert img.width == 2
        assert img.encoding == "mono8"
        assert img.data.tobytes() == b"\x01\x02\x03\x04"

    def test_imu_from_bytearray_isolated(self):
        cdr = bytearray(_make_imu_cdr())
        imu = Imu.from_cdr(cdr)
        cdr[:] = b"\xff" * len(cdr)
        assert imu.frame_id == "imu"


class TestFromCdrMemoryview:
    """from_cdr(memoryview) should copy (mutable source)."""

    def test_image_from_memoryview(self):
        cdr = memoryview(_make_image_cdr())
        img = Image.from_cdr(cdr)
        assert img.height == 2
        assert img.encoding == "mono8"

    def test_readonly_memoryview_from_bytes(self):
        """Readonly memoryview over bytes should still work."""
        cdr = _make_image_cdr()
        mv = memoryview(cdr)
        img = Image.from_cdr(mv)
        assert img.height == 2

    def test_mutable_memoryview_isolated(self):
        """Mutable memoryview: mutation after from_cdr doesn't affect msg."""
        source = bytearray(_make_image_cdr())
        mv = memoryview(source)
        img = Image.from_cdr(mv)
        source[:] = b"\x00" * len(source)
        assert img.height == 2
        assert img.data.tobytes() == b"\x01\x02\x03\x04"


# ── helpers for new types ────────────────────────────────────────────────────

def _make_occupancy_grid_cdr() -> bytes:
    """Build a 2×2 OccupancyGrid CDR buffer."""
    og = OccupancyGrid(
        header=Header(stamp=Time(10, 0), frame_id="map"),
        data=bytes([0, 50, 75, 100]),  # 4 non-negative cells (int8-safe)
    )
    return og.to_bytes()


def _make_grid_cells_cdr() -> bytes:
    """Build a GridCells CDR buffer with one known cell."""
    gc_msg = GridCells(
        header=Header(stamp=Time(7, 0), frame_id="map"),
        cell_width=0.25,
        cell_height=0.25,
        cells=[Point(1.0, 2.0, 0.0)],
    )
    return gc_msg.to_bytes()


def _make_relative_humidity_cdr() -> bytes:
    """Build a RelativeHumidity CDR buffer."""
    rh = RelativeHumidity(
        header=Header(stamp=Time(3, 0), frame_id="env"),
        relative_humidity=0.55,
        variance=0.002,
    )
    return rh.to_bytes()


def _make_time_reference_cdr() -> bytes:
    """Build a TimeReference CDR buffer."""
    tr = TimeReference(
        header=Header(stamp=Time(5, 0), frame_id="gps"),
        time_ref=Time(1000, 500),
        source="GPS",
    )
    return tr.to_bytes()


def _make_path_cdr() -> bytes:
    """Build a Path CDR buffer with no poses (header only).

    The aliasing test mutates the header stamp, which is present regardless of
    the pose sequence, so an empty path keeps the fixture minimal.
    """
    path = Path(
        header=Header(stamp=Time(8, 0), frame_id="world"),
        poses=[],
    )
    return path.to_bytes()


def _mutate_bytes(cdr: bytes, offset: int, new_value: bytes) -> None:
    """Write ``new_value`` into the C buffer backing ``cdr`` at ``offset``.

    Uses ``PyBytes_AsString`` to get a writable pointer to the immutable
    bytes object's internal buffer — this is undefined behavior in Python
    and exists solely to prove zero-copy aliasing in tests.
    """
    PyBytes_AsString = ctypes.pythonapi.PyBytes_AsString
    PyBytes_AsString.restype = ctypes.POINTER(ctypes.c_ubyte)
    PyBytes_AsString.argtypes = [ctypes.py_object]
    buf = PyBytes_AsString(cdr)
    for i, b in enumerate(new_value):
        buf[offset + i] = b


class _Py_buffer(ctypes.Structure):
    """Mirror of CPython's ``Py_buffer`` for read-only pointer inspection."""

    _fields_ = [
        ("buf", ctypes.c_void_p),
        ("obj", ctypes.c_void_p),
        ("len", ctypes.c_ssize_t),
        ("itemsize", ctypes.c_ssize_t),
        ("readonly", ctypes.c_int),
        ("ndim", ctypes.c_int),
        ("format", ctypes.c_char_p),
        ("shape", ctypes.POINTER(ctypes.c_ssize_t)),
        ("strides", ctypes.POINTER(ctypes.c_ssize_t)),
        ("suboffsets", ctypes.POINTER(ctypes.c_ssize_t)),
        ("internal", ctypes.c_void_p),
    ]


def _bytes_base(b: bytes) -> int:
    """Base address of a ``bytes`` object's internal buffer (read-only — safe)."""
    fn = ctypes.pythonapi.PyBytes_AsString
    fn.restype = ctypes.c_void_p
    fn.argtypes = [ctypes.py_object]
    return int(fn(b))


def _buffer_base_and_len(obj) -> tuple:
    """Return ``(base_address, length)`` for an object exposing the buffer
    protocol, via a read-only ``PyObject_GetBuffer`` request.

    This is the safe, deterministic way to prove a view aliases its source —
    by pointer identity rather than by observing an (undefined-behavior)
    in-place mutation of an immutable ``bytes`` object.
    """
    view = _Py_buffer()
    get = ctypes.pythonapi.PyObject_GetBuffer
    get.argtypes = [ctypes.py_object, ctypes.POINTER(_Py_buffer), ctypes.c_int]
    get.restype = ctypes.c_int
    if get(ctypes.py_object(obj), ctypes.byref(view), 0) != 0:  # 0 = PyBUF_SIMPLE
        raise RuntimeError("PyObject_GetBuffer failed")
    try:
        return int(view.buf), int(view.len)
    finally:
        rel = ctypes.pythonapi.PyBuffer_Release
        rel.argtypes = [ctypes.POINTER(_Py_buffer)]
        rel(ctypes.byref(view))


class TestZeroCopyNewTypes:
    """Zero-copy proofs for the 5 new buffer-backed message types.

    ``OccupancyGrid.data`` is proven by pointer identity (safe — the returned
    view points *inside* the input allocation, with no mutation).  The
    scalar-field cases use the mutate-and-observe diagnostic established by
    ``test_zero_copy_shares_buffer`` (Image): scalar accessors return by value,
    so there is no buffer handle to pointer-check, and mutable inputs are copied
    by design, leaving in-place mutation of the immutable ``bytes`` as the only
    way to prove the read aliases the source.  That mutation is UB by spec but
    confined to freshly-allocated, refcount-1 test buffers; production code must
    never mutate ``bytes``.
    """

    def test_occupancy_grid_data_points_into_input(self):
        """OccupancyGrid.data is a zero-copy view into the input CDR bytes.

        Proven safely by pointer identity: the buffer returned by ``data`` lies
        inside the input ``bytes`` allocation, so from_cdr borrowed it rather
        than copying.
        """
        cdr = _make_occupancy_grid_cdr()
        og = OccupancyGrid.from_cdr(cdr)
        base = _bytes_base(cdr)
        data_ptr, data_len = _buffer_base_and_len(og.data)
        assert base <= data_ptr, "data view starts before the input buffer"
        assert data_ptr + data_len <= base + len(cdr), "data view overruns input"
        assert bytes(og.data) == bytes([0, 50, 75, 100])

    def test_grid_cells_cell_width_aliases_input(self):
        """GridCells cell_width f32 is read directly from the CDR bytes."""
        cdr = _make_grid_cells_cdr()
        gc_msg = GridCells.from_cdr(cdr)
        assert abs(gc_msg.cell_width - 0.25) < 1e-6
        # Find the f32 0.25 pattern in the buffer
        raw = bytes(cdr)
        target = struct.pack("<f", 0.25)
        offset = raw.find(target)
        assert offset >= 0, "cell_width pattern not found in CDR"
        # Overwrite with f32(0.75)
        _mutate_bytes(cdr, offset, struct.pack("<f", 0.75))
        assert abs(gc_msg.cell_width - 0.75) < 1e-6

    def test_relative_humidity_value_aliases_input(self):
        """RelativeHumidity.relative_humidity f64 reads from the CDR bytes."""
        cdr = _make_relative_humidity_cdr()
        rh = RelativeHumidity.from_cdr(cdr)
        assert abs(rh.relative_humidity - 0.55) < 1e-12
        raw = bytes(cdr)
        target = struct.pack("<d", 0.55)
        offset = raw.find(target)
        assert offset >= 0, "relative_humidity pattern not found in CDR"
        _mutate_bytes(cdr, offset, struct.pack("<d", 0.99))
        assert abs(rh.relative_humidity - 0.99) < 1e-12

    def test_time_reference_time_ref_aliases_input(self):
        """TimeReference.time_ref.sec i32 reads from the CDR bytes."""
        cdr = _make_time_reference_cdr()
        tr = TimeReference.from_cdr(cdr)
        assert tr.time_ref.sec == 1000
        raw = bytes(cdr)
        target = struct.pack("<i", 1000)
        offset = raw.find(target)
        assert offset >= 0, "time_ref.sec pattern not found in CDR"
        _mutate_bytes(cdr, offset, struct.pack("<i", 9999))
        assert tr.time_ref.sec == 9999

    def test_path_stamp_aliases_input(self):
        """Path.stamp.sec i32 reads from the CDR bytes."""
        cdr = _make_path_cdr()
        path = Path.from_cdr(cdr)
        assert path.stamp.sec == 8
        raw = bytes(cdr)
        # stamp.sec=8 encoded as i32 LE at the start of the CDR body
        target = struct.pack("<i", 8)
        offset = raw.find(target)
        assert offset >= 0, "stamp.sec pattern not found in CDR"
        _mutate_bytes(cdr, offset, struct.pack("<i", 42))
        assert path.stamp.sec == 42


class TestCdrRoundTrip:
    """Verify from_cdr → to_bytes round-trip for both input paths."""

    def test_bytes_round_trip(self):
        cdr = _make_image_cdr()
        img = Image.from_cdr(cdr)
        assert img.to_bytes() == cdr

    def test_bytearray_round_trip(self):
        cdr_bytes = _make_image_cdr()
        cdr_ba = bytearray(cdr_bytes)
        img = Image.from_cdr(cdr_ba)
        assert img.to_bytes() == cdr_bytes

    def test_compressed_image_round_trip(self):
        ci = CompressedImage(
            header=Header(stamp=Time(1, 0), frame_id="c"),
            format="jpeg",
            data=b"\xff\xd8\xff\xe0",
        )
        cdr = ci.to_bytes()
        ci2 = CompressedImage.from_cdr(cdr)
        assert ci2.format == "jpeg"
        assert ci2.data.tobytes() == b"\xff\xd8\xff\xe0"
        assert ci2.to_bytes() == cdr
