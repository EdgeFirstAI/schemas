# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for the zero-copy ``BorrowedBuf`` view contract.

The design contract is documented in ``crates/python/python/edgefirst/
schemas/__init__.pyi``:

1. Construction copies the bulk payload once into the CDR buffer (GIL
   released during the copy).
2. Read-side property accessors (``image.data``, ``mask.mask``, …) return
   a ``BorrowedBuf`` aliasing the parent's bytes — no copy.
3. The view holds a strong reference to the parent so the bytes stay
   valid until the view is itself dropped.
4. ``view.tobytes()`` materialises a Python ``bytes`` (one copy).
5. ``view.view()`` returns a ``memoryview`` (zero-copy, abi3-py38 fallback).
6. On Python 3.11+ / non-abi3 builds, the buffer protocol is also
   wired; we don't *require* it for tests because abi3-py38 wheels
   omit it from the limited API.
"""

import gc
import sys

import numpy as np
import pytest  # noqa: F401  (used by skip in conditional buffer-protocol tests)

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.sensor_msgs import Image
from edgefirst.schemas.std_msgs import Header


def _make_image(payload: bytes, width: int = 64, height: int = 48) -> Image:
    return Image(
        header=Header(stamp=Time(1, 0), frame_id="t"),
        height=height, width=width, encoding="rgb8",
        is_bigendian=0, step=width * 3, data=payload,
    )


class TestBorrowedBufBasics:
    def test_len_matches_payload(self):
        img = _make_image(b"\x00" * (64 * 48 * 3))
        assert len(img.data) == 64 * 48 * 3

    def test_tobytes_round_trips(self):
        rng = np.random.default_rng(seed=99)
        payload = rng.bytes(64 * 48 * 3)
        img = _make_image(payload)
        assert img.data.tobytes() == payload

    def test_view_returns_bytes_like(self):
        # `view()` returns a memoryview on Py 3.11+ / non-abi3 builds,
        # bytes on abi3-py38. Both are bytes-like and have the right len.
        img = _make_image(b"\x00" * 100)
        v = img.data.view()
        assert isinstance(v, (memoryview, bytes))
        assert len(v) == 100

    def test_repr_includes_length(self):
        img = _make_image(b"\x00" * 100)
        r = repr(img.data)
        assert "100" in r or str(len(img.data)) in r


class TestZeroCopyContract:
    def test_buffer_protocol_aliases_parent(self):
        # The headline zero-copy claim: `memoryview(borrowed_buf)` must
        # not copy. We prove this by reading via the buffer protocol
        # and confirming the resulting numpy array's data pointer
        # matches `to_bytes()` content but at a *stable* address
        # across two acquisitions.
        try:
            img = _make_image(b"\x00" * (640 * 480 * 3))
            mv1 = memoryview(img.data)
            mv2 = memoryview(img.data)
        except TypeError:
            pytest.skip("buffer protocol unavailable (abi3-py38 build)")
        a1 = np.frombuffer(mv1, dtype=np.uint8)
        a2 = np.frombuffer(mv2, dtype=np.uint8)
        # Same storage — proof that BorrowedBuf doesn't copy on each
        # `data` access nor on each buffer-protocol acquisition.
        assert a1.ctypes.data == a2.ctypes.data

    def test_view_aliases_payload_within_cdr(self):
        # `to_bytes()` returns the full CDR buffer — the data section
        # within it must be byte-equal to what the view exposes (the
        # view IS into that buffer, not a copy of separate storage).
        rng = np.random.default_rng(seed=1)
        # Pick a deterministic payload size matching width*height*3.
        payload = rng.bytes(16 * 16 * 3)
        img = _make_image(payload, width=16, height=16)
        cdr = img.to_bytes()
        view_bytes = img.data.tobytes()
        # Containment check — the data section is somewhere inside the
        # CDR buffer. Note: this is a content check, not a pointer
        # identity check. The pointer-identity proof lives in
        # `test_buffer_protocol_aliases_parent`.
        assert view_bytes == payload
        assert view_bytes in cdr


class TestParentLifetimeManagement:
    def test_view_keeps_parent_alive_via_tobytes(self):
        # The BorrowedBuf holds a strong ref to the parent message; the
        # parent mustn't be collected while the BorrowedBuf is alive
        # even when no other references to the parent exist.
        def make_view():
            img = _make_image(b"\x42" * 100)
            v = img.data
            assert v.tobytes()[:4] == b"\x42\x42\x42\x42"
            return v

        view = make_view()
        gc.collect()
        # View still works — parent kept alive by the BorrowedBuf's
        # strong reference.
        assert view.tobytes()[:4] == b"\x42\x42\x42\x42"
        assert len(view) == 100

    def test_buffer_protocol_anchors_parent_after_drop(self):
        # The stronger contract: `memoryview(borrowed_buf)` (via the
        # buffer protocol) must keep the parent alive even after all
        # Python-side references to both the message AND the BorrowedBuf
        # are dropped. The Py_buffer.obj slot points back at the
        # BorrowedBuf, which Python's buffer machinery refcounts for
        # the memoryview's lifetime.
        def make_mv():
            local_img = _make_image(b"\x77" * 200)
            try:
                return memoryview(local_img.data)
            except TypeError:
                pytest.skip("buffer protocol unavailable (abi3-py38 build)")

        mv = make_mv()
        # `local_img`, its BorrowedBuf, and the local `data` getter
        # result are all unreachable from this scope. If the buffer
        # protocol's parent anchoring is wrong, the next read is UAF.
        gc.collect()
        gc.collect()  # second pass for cycles, paranoid but cheap
        assert bytes(mv[:4]) == b"\x77\x77\x77\x77"
        assert len(mv) == 200

    def test_parent_attribute_holds_image(self):
        img = _make_image(b"\x00" * 30)
        view = img.data
        # `parent` exposes the message keeping the view valid.
        assert view.parent is not None
        # Same image — refcount on the parent should be at least the
        # view's strong ref + this scope's `img` binding.
        assert sys.getrefcount(view.parent) >= 2
