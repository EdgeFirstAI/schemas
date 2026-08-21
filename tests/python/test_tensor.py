# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for the composed tensor message family.

Covers ``Tensor`` (the payload), ``TensorPlane``, and the two byte-identical
wrappers ``TensorStamped`` and ``CameraFrame``.

Beyond field round-trips these pin the contracts that make composition work:
the embedded tensor's bytes are position-independent (identical no matter
which wrapper carries it, and no matter how long ``frame_id`` is), an
embedded tensor re-parses standalone, and a wrapper decoded from ``bytes``
hands out a tensor that aliases the same buffer rather than a copy.
"""

import ctypes
import gc

import numpy as np
import pytest

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.edgefirst_msgs import (
    CameraFrame,
    Tensor,
    TensorPlane,
    TensorStamped,
)

WIDTH, HEIGHT = 640, 480
Y_SIZE = WIDTH * HEIGHT
UV_SIZE = Y_SIZE // 2
HANDLE_BYTES = b"\xde\xad\xbe\xef"


def nv12_planes() -> list[TensorPlane]:
    """Two referenced planes, as a single-allocation NV12 dma-buf carries."""
    return [
        TensorPlane(
            handle=7,
            offset=0,
            stride=WIDTH,
            size=Y_SIZE,
            used=Y_SIZE,
            handle_bytes=HANDLE_BYTES,
        ),
        TensorPlane(
            handle=7,
            offset=Y_SIZE,
            stride=WIDTH,
            size=UV_SIZE,
            used=UV_SIZE,
        ),
    ]


def nv12_tensor() -> Tensor:
    """An unquantized NV12 camera-frame tensor."""
    return Tensor(
        storage_kind=2,
        pid=4242,
        fence_fd=-1,
        dtype=1,
        quant_axis=-2,
        shape=[HEIGHT, WIDTH],
        strides=[WIDTH, 1],
        format="NV12",
        color_space="bt709",
        color_transfer="bt709",
        color_encoding="bt709",
        color_range="limited",
        planes=nv12_planes(),
    )


class TestTensorPlane:
    def test_defaults_are_the_schema_absent_values(self):
        p = TensorPlane()
        # An all-default plane is an empty inline plane, not a referenced one.
        assert p.handle == -1
        assert p.is_inline is True
        assert p.offset == 0
        assert p.stride == 0
        assert p.size == 0
        assert p.used == 0
        assert p.modifier == 0
        assert p.handle_bytes == b""
        assert p.data == b""

    def test_referenced_plane_fields(self):
        p = TensorPlane(
            handle=7, offset=16, stride=WIDTH, size=Y_SIZE, used=Y_SIZE,
            modifier=3, handle_bytes=HANDLE_BYTES,
        )
        assert p.handle == 7
        assert p.is_inline is False
        assert p.offset == 16
        assert p.stride == WIDTH
        assert p.size == Y_SIZE
        assert p.used == Y_SIZE
        assert p.modifier == 3
        assert p.handle_bytes == HANDLE_BYTES

    def test_repr(self):
        assert "TensorPlane(handle=7" in repr(TensorPlane(handle=7))


class TestTensor:
    def test_round_trips_every_field(self):
        t = Tensor.from_cdr(nv12_tensor().to_bytes())
        assert t.storage_kind == 2
        assert t.pid == 4242
        assert t.fence_fd == -1
        assert t.dtype == 1
        assert t.quant_axis == -2
        assert t.shape == [HEIGHT, WIDTH]
        assert t.strides == [WIDTH, 1]
        assert t.format == "NV12"
        assert t.color_space == "bt709"
        assert t.color_transfer == "bt709"
        assert t.color_encoding == "bt709"
        assert t.color_range == "limited"
        assert t.num_planes == 2

    def test_defaults_are_not_all_zero(self):
        """fence_fd and quant_axis default to their 'absent' values, not 0."""
        t = Tensor()
        assert t.fence_fd == -1
        assert t.quant_axis == -2
        assert t.storage_kind == 0
        assert t.num_planes == 0

    def test_planes_round_trip(self):
        t = Tensor.from_cdr(nv12_tensor().to_bytes())
        planes = t.planes
        assert len(planes) == 2
        assert [p.offset for p in planes] == [0, Y_SIZE]
        assert all(p.handle == 7 for p in planes)
        assert all(not p.is_inline for p in planes)
        # A referenced plane carries no inline bytes.
        assert all(p.data == b"" for p in planes)
        assert planes[0].handle_bytes == HANDLE_BYTES
        assert planes[1].handle_bytes == b""

    def test_inline_plane_round_trip(self):
        payload = bytes(range(8))
        t = Tensor(
            dtype=1,
            planes=[TensorPlane(handle=-1, size=len(payload), used=len(payload),
                                data=payload)],
        )
        got = Tensor.from_cdr(t.to_bytes())
        assert got.num_planes == 1
        p = got.planes[0]
        assert p.is_inline is True
        assert p.handle == -1
        assert p.data == payload

    def test_plane_data_is_zero_copy(self):
        """``plane_data`` returns a view over the message buffer, not a copy."""
        payload = bytes(range(32))
        t = Tensor(
            dtype=1,
            planes=[TensorPlane(handle=-1, size=len(payload), used=len(payload),
                                data=payload)],
        )
        got = Tensor.from_cdr(t.to_bytes())
        view = got.plane_data(0)
        arr = np.frombuffer(view, dtype=np.uint8)
        assert arr.tobytes() == payload
        # numpy did not take ownership; the view still refers to the message.
        assert len(view) == len(payload)

    def test_plane_data_empty_for_referenced_plane(self):
        t = Tensor.from_cdr(nv12_tensor().to_bytes())
        assert len(t.plane_data(0)) == 0

    def test_plane_data_out_of_range(self):
        t = Tensor.from_cdr(nv12_tensor().to_bytes())
        with pytest.raises(ValueError):
            t.plane_data(5)

    def test_shape_is_the_addressing_grid_not_the_byte_layout(self):
        """NV12 is [h, w] with a U8 dtype against an h*w*3/2 allocation.

        The encoder must never validate shape against any buffer size; this
        tensor is well-formed precisely because it does not.
        """
        t = Tensor.from_cdr(nv12_tensor().to_bytes())
        assert t.shape == [HEIGHT, WIDTH]
        assert t.dtype == 1
        assert sum(p.size for p in t.planes) == HEIGHT * WIDTH * 3 // 2

    def test_repr(self):
        assert "Tensor(dtype=1" in repr(nv12_tensor())

    def test_cdr_size_matches_bytes(self):
        t = nv12_tensor()
        assert t.cdr_size == len(t.to_bytes())


class TestTensorValidation:
    """`quant_axis` selects which shape the quantization params must take."""

    def test_per_tensor_quantization(self):
        t = Tensor(dtype=3, shape=[4], quant_axis=-1,
                   quant_scales=[0.125], quant_zero_points=[7])
        got = Tensor.from_cdr(t.to_bytes())
        assert got.quant_axis == -1
        assert got.quant_scales == [0.125]
        assert got.quant_zero_points == [7]

    def test_per_axis_quantization(self):
        t = Tensor(shape=[3, 8], quant_axis=0, quant_scales=[0.5, 0.25, 0.125])
        got = Tensor.from_cdr(t.to_bytes())
        assert got.quant_axis == 0
        assert len(got.quant_scales) == 3

    def test_unquantized_must_have_no_scales(self):
        with pytest.raises(ValueError):
            Tensor(quant_axis=-2, quant_scales=[0.5])

    def test_per_tensor_needs_exactly_one_scale(self):
        with pytest.raises(ValueError):
            Tensor(shape=[4], quant_axis=-1, quant_scales=[0.5, 0.25])

    def test_per_axis_scale_count_must_match_that_axis(self):
        with pytest.raises(ValueError):
            Tensor(shape=[3, 8], quant_axis=0, quant_scales=[0.5, 0.25])

    def test_zero_points_must_match_scales_length(self):
        with pytest.raises(ValueError):
            Tensor(shape=[4], quant_axis=-1, quant_scales=[0.5],
                   quant_zero_points=[1, 2])

    def test_strides_rank_must_match_shape_rank(self):
        with pytest.raises(ValueError):
            Tensor(shape=[4, 4], strides=[4])

    def test_inline_plane_size_must_match_data(self):
        with pytest.raises(ValueError):
            Tensor(planes=[TensorPlane(handle=-1, size=99, data=b"\x00" * 8)])

    def test_used_may_not_exceed_size(self):
        with pytest.raises(ValueError):
            Tensor(planes=[TensorPlane(handle=3, size=16, used=17)])

    def test_transport_modes_may_not_be_mixed(self):
        """A frame is all-inline or all-referenced; it has one storage_kind."""
        with pytest.raises(ValueError):
            Tensor(planes=[
                TensorPlane(handle=3, size=16),
                TensorPlane(handle=-1, size=4, data=b"\x00" * 4),
            ])

    def test_colorimetry_without_format_is_rejected(self):
        with pytest.raises(ValueError):
            Tensor(color_space="bt709")


@pytest.mark.parametrize("wrapper", [TensorStamped, CameraFrame])
class TestWrappers:
    """Both wrappers are byte-identical, so both run every case."""

    def test_round_trips_header_and_payload(self, wrapper):
        msg = wrapper(stamp=Time(1234567890, 123456789), frame_id="camera_0",
                      seq=99, tensor=nv12_tensor())
        got = wrapper.from_cdr(msg.to_bytes())
        assert got.stamp == Time(1234567890, 123456789)
        assert got.frame_id == "camera_0"
        assert got.seq == 99
        t = got.tensor
        assert t.format == "NV12"
        assert t.num_planes == 2
        assert t.pid == 4242

    def test_defaults(self, wrapper):
        msg = wrapper()
        assert msg.frame_id == ""
        assert msg.seq == 0
        assert msg.stamp == Time(0, 0)
        assert msg.tensor.num_planes == 0

    def test_repr(self, wrapper):
        msg = wrapper(frame_id="cam", seq=3, tensor=nv12_tensor())
        assert wrapper.__name__ in repr(msg)
        assert "seq=3" in repr(msg)

    def test_from_cdr_rejects_garbage(self, wrapper):
        with pytest.raises(ValueError):
            wrapper.from_cdr(b"\xde\xad\xbe\xef")

    def test_nested_tensor_aliases_the_message_buffer(self, wrapper):
        """``.tensor`` shares the parent's bytes rather than re-encoding.

        Mutating the source ``bytes`` through ctypes and observing the change
        through the nested tensor proves the two views read the same memory.

        NOTE: mutating a ``bytes`` object is undefined behaviour in Python;
        this is purely diagnostic. Production code must never do it.
        """
        cdr = wrapper(frame_id="cam", seq=1, tensor=nv12_tensor()).to_bytes()
        msg = wrapper.from_cdr(cdr)
        assert msg.tensor.pid == 4242

        PyBytes_AsString = ctypes.pythonapi.PyBytes_AsString
        PyBytes_AsString.restype = ctypes.POINTER(ctypes.c_ubyte)
        PyBytes_AsString.argtypes = [ctypes.py_object]
        buf = PyBytes_AsString(cdr)

        offset = bytes(cdr).find((4242).to_bytes(4, "little"))
        assert offset >= 0, "could not locate pid in the CDR buffer"
        for i, b in enumerate((99).to_bytes(4, "little")):
            buf[offset + i] = b

        # Read through a freshly-taken nested tensor: it must see the change.
        assert msg.tensor.pid == 99

    def test_valid_after_source_bytes_released(self, wrapper):
        cdr = wrapper(frame_id="cam", seq=7, tensor=nv12_tensor()).to_bytes()
        msg = wrapper.from_cdr(cdr)
        nested = msg.tensor
        del cdr
        gc.collect()
        assert msg.seq == 7
        assert nested.format == "NV12"
        assert nested.num_planes == 2


class TestPositionIndependence:
    """The invariant `seq` buys.

    `seq` is a uint64, so it forces the embedded tensor to an 8-aligned
    offset regardless of `frame_id` length. That is what makes the nested
    tensor's bytes identical in every wrapper and at every frame_id length.
    """

    FRAME_IDS = ["", "a", "camera_0", "a_very_long_frame_identifier_x"]

    def test_tensor_bytes_identical_across_wrappers(self):
        t = nv12_tensor()
        for fid in self.FRAME_IDS:
            cf = CameraFrame.from_cdr(
                CameraFrame(frame_id=fid, seq=5, tensor=t).to_bytes())
            ts = TensorStamped.from_cdr(
                TensorStamped(frame_id=fid, seq=5, tensor=t).to_bytes())
            assert cf.tensor.to_standalone_cdr() == ts.tensor.to_standalone_cdr()

    def test_tensor_bytes_identical_across_frame_id_lengths(self):
        t = nv12_tensor()
        seen = {
            CameraFrame.from_cdr(
                CameraFrame(frame_id=fid, seq=5, tensor=t).to_bytes()
            ).tensor.to_standalone_cdr()
            for fid in self.FRAME_IDS
        }
        assert len(seen) == 1, "tensor bytes shifted with frame_id length"

    def test_embedded_tensor_matches_standalone_encoding(self):
        """Re-heading an embedded tensor equals encoding it from scratch."""
        t = nv12_tensor()
        frame = CameraFrame.from_cdr(
            CameraFrame(frame_id="camera_0", seq=1, tensor=t).to_bytes())
        assert frame.tensor.to_standalone_cdr() == t.to_bytes()

    def test_republished_tensor_parses_as_a_standalone_message(self):
        t = nv12_tensor()
        frame = CameraFrame.from_cdr(
            CameraFrame(frame_id="camera_0", seq=1, tensor=t).to_bytes())
        republished = Tensor.from_cdr(frame.tensor.to_standalone_cdr())
        assert republished.format == "NV12"
        assert republished.pid == 4242
        assert republished.num_planes == 2
        assert republished.shape == [HEIGHT, WIDTH]

    def test_wrappers_reinterpret_each_other(self):
        """Byte-identical means a CameraFrame parses as a TensorStamped."""
        raw = CameraFrame(frame_id="cam", seq=11, tensor=nv12_tensor()).to_bytes()
        as_stamped = TensorStamped.from_cdr(raw)
        assert as_stamped.seq == 11
        assert as_stamped.frame_id == "cam"
        assert as_stamped.tensor.format == "NV12"
