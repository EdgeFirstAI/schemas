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

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.sensor_msgs import Image, CompressedImage, Imu
from edgefirst.schemas.std_msgs import Header
from edgefirst.schemas.geometry_msgs import TwistStamped


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
