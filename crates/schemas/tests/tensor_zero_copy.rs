// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Zero-copy contract tests for the Tensor message set.
//!
//! Every property asserted here fails silently in production if it regresses:
//! a misaligned tensor still parses, a rescanning accessor still returns the
//! right value, and a parser that rejects trailing bytes only breaks against a
//! producer that does not exist yet.

use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::cdr::{CdrError, CDR_HEADER_SIZE};
use edgefirst_schemas::edgefirst_msgs::{CameraFrame, TensorStamped};
use edgefirst_schemas::tensor::{Tensor, TensorFields, TensorPlaneView};

fn fields<'a>(shape: &'a [u64], planes: &'a [TensorPlaneView<'a>]) -> TensorFields<'a> {
    TensorFields {
        storage_kind: 2,
        pid: 1234,
        dtype: 1,
        shape,
        planes,
        format: std::borrow::Cow::Borrowed("NV12"),
        color_space: std::borrow::Cow::Borrowed("bt709"),
        ..Default::default()
    }
}

fn two_planes() -> [TensorPlaneView<'static>; 2] {
    [
        TensorPlaneView {
            handle: 5,
            offset: 0,
            stride: 1920,
            size: 1920 * 1080,
            used: 1920 * 1080,
            modifier: 0,
            handle_bytes: &[],
            data: &[],
        },
        TensorPlaneView {
            handle: 5,
            offset: 1920 * 1080,
            stride: 1920,
            size: 1920 * 540,
            // Deliberately < size (a partially-filled plane) and distinct
            // from `size`, so size/used cannot be transposed and still pass.
            used: 1920 * 540 - 500,
            modifier: 1,
            handle_bytes: &[],
            data: &[],
        },
    ]
}

/// Two planes in inline transport mode: bytes travel in `data` rather than
/// behind a handle. `validate_plane_set` requires all-inline-or-none, so both
/// planes must agree.
fn two_inline_planes() -> [TensorPlaneView<'static>; 2] {
    const DATA0: [u8; 4] = [0xAA, 0xBB, 0xCC, 0xDD];
    const DATA1: [u8; 3] = [0x11, 0x22, 0x33];
    [
        TensorPlaneView {
            handle: -1,
            offset: 0,
            stride: 0,
            size: DATA0.len() as u64,
            used: DATA0.len() as u64,
            modifier: 0,
            handle_bytes: &[],
            data: &DATA0,
        },
        TensorPlaneView {
            handle: -1,
            offset: 0,
            stride: 0,
            size: DATA1.len() as u64,
            used: DATA1.len() as u64,
            modifier: 0,
            handle_bytes: &[],
            data: &DATA1,
        },
    ]
}

/// The tensor's bytes must not depend on the frame_id length that precedes
/// them. Each frame_id below lands the header end on a different residue mod 8.
#[test]
fn tensor_bytes_are_independent_of_position() {
    let shape: [u64; 2] = [1080, 1920];
    let planes = two_planes();
    let f = fields(&shape, &planes);

    let standalone = Tensor::builder()
        .storage_kind(f.storage_kind)
        .pid(f.pid)
        .dtype(f.dtype)
        .shape(&shape)
        .format("NV12")
        .color_space("bt709")
        .planes(&planes)
        .build()
        .unwrap();
    let want = &standalone.as_cdr()[CDR_HEADER_SIZE..];

    for frame_id in ["", "a", "ab", "abc", "abcd", "abcde", "abcdef", "abcdefg"] {
        let cf = CameraFrame::builder()
            .stamp(Time { sec: 1, nanosec: 2 })
            .frame_id(frame_id)
            .seq(7)
            .tensor(&f)
            .build()
            .unwrap();
        let got = CameraFrame::<&[u8]>::from_cdr(cf.as_cdr()).unwrap();
        assert_eq!(
            got.tensor().tensor_bytes(),
            want,
            "tensor bytes shifted for frame_id {frame_id:?}"
        );
    }
}

/// A tensor lifted out of a wrapper and re-headed must parse standalone and
/// compare equal field for field. This is the republish path.
///
/// Every scalar and variable-length field gets a non-default value in this
/// test's own fixture, and every one is compared. That matters specifically
/// for `pid`/`fence_fd`: they are two adjacent same-width (4-byte) scalars,
/// so a bug that swapped them would corrupt both values without shifting a
/// single byte downstream — nothing else in this file would notice.
#[test]
fn embedded_tensor_reparses_standalone() {
    let shape: [u64; 2] = [1080, 1920];
    let strides: [i64; 2] = [1920, 1];
    let quant_scales: [f32; 1] = [0.5];
    let quant_zero_points: [i32; 1] = [3];
    let planes = two_planes();
    let f = TensorFields {
        storage_kind: 2,
        pid: 4242,
        fence_fd: 7,
        dtype: 1,
        quant_axis: -1,
        shape: &shape,
        strides: &strides,
        quant_scales: &quant_scales,
        quant_zero_points: &quant_zero_points,
        format: std::borrow::Cow::Borrowed("NV12"),
        color_space: std::borrow::Cow::Borrowed("bt709"),
        planes: &planes,
        ..Default::default()
    };
    let cf = CameraFrame::builder()
        .frame_id("cam0")
        .seq(3)
        .tensor(&f)
        .build()
        .unwrap();

    let view = CameraFrame::<&[u8]>::from_cdr(cf.as_cdr()).unwrap();
    let lifted = view.tensor().to_standalone_cdr();
    let t = Tensor::<&[u8]>::from_cdr(&lifted[..]).unwrap();

    assert_eq!(t.storage_kind(), view.tensor().storage_kind());
    assert_eq!(t.pid(), view.tensor().pid());
    assert_eq!(t.fence_fd(), view.tensor().fence_fd());
    assert_eq!(t.dtype(), view.tensor().dtype());
    assert_eq!(t.quant_axis(), view.tensor().quant_axis());
    assert_eq!(
        t.shape().collect::<Vec<_>>(),
        view.tensor().shape().collect::<Vec<_>>()
    );
    assert_eq!(
        t.strides().collect::<Vec<_>>(),
        view.tensor().strides().collect::<Vec<_>>()
    );
    assert_eq!(t.quant_scales(), view.tensor().quant_scales());
    assert_eq!(t.quant_zero_points(), view.tensor().quant_zero_points());
    assert_eq!(t.format(), view.tensor().format());
    assert_eq!(t.num_planes(), view.tensor().num_planes());
    assert_eq!(t.plane_at(1).unwrap().modifier, 1);

    // `size` and `used` are adjacent same-width fields; assert both with
    // distinct expected values so a transposition of the two in
    // write_plane_element/scan_plane_element cannot pass unnoticed.
    assert_eq!(t.plane_at(1).unwrap().size, planes[1].size);
    assert_eq!(t.plane_at(1).unwrap().used, planes[1].used);
    assert_ne!(planes[1].size, planes[1].used);

    // The fixture must actually exercise non-default values, or the
    // equality assertions above would pass trivially (both sides Default).
    assert_ne!(t.pid(), TensorFields::default().pid);
    assert_ne!(t.fence_fd(), TensorFields::default().fence_fd);
    assert_ne!(t.quant_axis(), TensorFields::default().quant_axis);
    assert!(!t.strides().collect::<Vec<_>>().is_empty());
    assert!(!t.quant_scales().is_empty());
    assert!(!t.quant_zero_points().is_empty());
}

/// The two wrappers have identical layouts, so a CameraFrame buffer parses as
/// a TensorStamped and vice versa. This is what makes reinterpretation free.
#[test]
fn wrappers_reinterpret_without_reparsing_content() {
    let shape: [u64; 1] = [16];
    let planes: [TensorPlaneView; 0] = [];
    let cf = CameraFrame::builder()
        .frame_id("cam0")
        .seq(9)
        .tensor(&fields(&shape, &planes))
        .build()
        .unwrap();

    let as_ts = TensorStamped::<&[u8]>::from_cdr(cf.as_cdr()).unwrap();
    assert_eq!(as_ts.frame_id(), "cam0");
    assert_eq!(as_ts.seq(), 9);
    assert_eq!(as_ts.tensor().shape().collect::<Vec<_>>(), vec![16u64]);
}

/// Appending unknown trailing bytes must not break an existing parser. This is
/// the forward-compatibility contract that lets 4.x append fields to Tensor:
/// Tensor is the last field of every wrapper, so appending to it appends to the
/// message. If someone adds a strict length check, this test fails.
#[test]
fn parsers_ignore_unknown_trailing_bytes() {
    let shape: [u64; 1] = [8];
    let planes: [TensorPlaneView; 0] = [];
    let cf = CameraFrame::builder()
        .frame_id("cam0")
        .seq(2)
        .tensor(&fields(&shape, &planes))
        .build()
        .unwrap();

    let mut future = cf.to_cdr();
    future.extend_from_slice(&[0xAA; 32]); // a field this version does not know

    let v = CameraFrame::<&[u8]>::from_cdr(&future[..]).unwrap();
    assert_eq!(v.frame_id(), "cam0");
    assert_eq!(v.seq(), 2);
    assert_eq!(v.tensor().shape().collect::<Vec<_>>(), vec![8u64]);
}

/// A forged plane count must be rejected against the remaining buffer before
/// anything is allocated. Guards MIN_PLANE_BYTES.
#[test]
fn forged_plane_count_is_rejected() {
    let shape: [u64; 1] = [8];
    let planes: [TensorPlaneView; 0] = [];
    let cf = CameraFrame::builder()
        .frame_id("cam0")
        .tensor(&fields(&shape, &planes))
        .build()
        .unwrap();

    let mut forged = cf.to_cdr();
    // Overwrite the trailing plane-count u32 with a count no buffer could hold.
    let n = forged.len();
    forged[n - 4..].copy_from_slice(&u32::MAX.to_le_bytes());

    // `is_err()` alone is too weak here: once the count is past the buffer,
    // *any* attempt to actually scan `u32::MAX` plane elements also fails
    // with `BufferTooShort`, from the very first element read — a buffer
    // exhaustion that would happen regardless of whether the up-front
    // `check_seq_count` guard exists. That failure reports a `need` on the
    // order of the buffer size itself. The guard, by contrast, fails before
    // touching a single plane element and reports `need` computed from the
    // full forged count (`pos + u32::MAX as usize * MIN_PLANE_BYTES`), which
    // is billions of bytes — nothing the tiny test buffer could ever produce
    // by exhaustion alone. Asserting that magnitude is what pins this test to
    // the guard rather than to incidental short-buffer behaviour deeper in
    // the parser.
    match CameraFrame::<&[u8]>::from_cdr(&forged[..]) {
        Err(CdrError::BufferTooShort { need, .. }) => {
            assert!(
                need > forged.len() * 1000,
                "need={need} is too small to have come from the plane-count \
                 guard; the forged count was likely rejected by ordinary \
                 buffer exhaustion instead of check_seq_count"
            );
        }
        other => panic!("expected BufferTooShort from the plane-count guard, got {other:?}"),
    }
}

/// Plane iteration must not allocate. Asserted structurally: the iterator is
/// ExactSizeIterator, so a caller learns the length without walking, and the
/// yielded views borrow from the buffer.
///
/// Uses `two_inline_planes()`, not `two_planes()`: `validate_plane` requires
/// `data` to be empty whenever `handle >= 0` (referenced mode), so a
/// referenced-mode fixture makes `!p.data.is_empty()` always false and the
/// aliasing check dead code that never runs. Only an inline-mode fixture
/// actually populates `data` on the wire, which is what this test needs to
/// mean anything.
#[test]
fn plane_iteration_borrows_and_does_not_allocate() {
    let shape: [u64; 2] = [1080, 1920];
    let planes = two_inline_planes();
    let cf = CameraFrame::builder()
        .frame_id("cam0")
        .tensor(&fields(&shape, &planes))
        .build()
        .unwrap();
    let buf = cf.to_cdr();
    let v = CameraFrame::<&[u8]>::from_cdr(&buf[..]).unwrap();
    let t = v.tensor();

    assert_eq!(t.planes().len(), 2); // ExactSizeIterator: no walk required
    let mut aliased = 0;
    for p in t.planes() {
        // The fixture is inline-mode, so `data` must be populated here; if a
        // future fixture change makes it empty again this assertion catches
        // it instead of letting the aliasing check below silently stop
        // running.
        assert!(
            !p.data.is_empty(),
            "fixture must carry inline bytes for this test to mean anything"
        );
        // Every borrowed slice must alias the source buffer, not a copy.
        assert!(
            buf.as_ptr_range().contains(&p.data.as_ptr()),
            "plane data must alias the source buffer, not a copy"
        );
        aliased += 1;
    }
    assert_eq!(
        aliased, 2,
        "every plane must have been checked for aliasing"
    );
}
