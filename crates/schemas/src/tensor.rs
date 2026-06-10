// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Tensor message core: plane codec, `Tensor` view, and `TensorBuilder`.

use crate::cdr::{
    align, cdr_align, rd_i32, rd_i64, rd_slice_f32, rd_slice_i32, rd_string, rd_u32, rd_u64,
    CdrCursor, CdrError, CdrSizer, CdrWriter, CDR_HEADER_SIZE,
};
use std::borrow::Cow;

/// Minimum encoded size of one `TensorPlane` element, in bytes past the CDR
/// header: the 48-byte scalar block plus two 4-byte sequence counts.
///
/// Passed to `CdrCursor::check_seq_count` to bound a forged plane count
/// against the remaining buffer before anything is allocated. This is a
/// denial-of-service guard, not a tuning value — if the element layout
/// changes, this constant MUST change with it.
pub const MIN_PLANE_BYTES: usize = 56;

/// Zero-copy view of a single `TensorPlane` element, borrowed from a CDR buffer.
///
/// Two exclusive transport modes:
/// * `handle >= 0` — bytes live behind the handle; `data` is empty.
/// * `handle == -1` — bytes are inlined in `data`; `size == data.len()`,
///   `modifier == 0`, `handle_bytes` empty.
///
/// Any other negative handle is invalid.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct TensorPlaneView<'a> {
    pub handle: i64,
    pub offset: u64,
    pub stride: u64,
    pub size: u64,
    pub used: u64,
    pub modifier: u64,
    pub handle_bytes: &'a [u8],
    pub data: &'a [u8],
}

impl TensorPlaneView<'_> {
    /// True when this plane's bytes travel inline rather than by reference.
    #[inline]
    pub fn is_inline(&self) -> bool {
        self.handle == -1
    }
}

pub(crate) fn scan_plane_element<'a>(
    c: &mut CdrCursor<'a>,
) -> Result<TensorPlaneView<'a>, CdrError> {
    let handle = c.read_i64()?;
    let offset = c.read_u64()?;
    let stride = c.read_u64()?;
    let size = c.read_u64()?;
    let used = c.read_u64()?;
    let modifier = c.read_u64()?;
    let handle_bytes = c.read_bytes()?;
    let data = c.read_bytes()?;
    Ok(TensorPlaneView {
        handle,
        offset,
        stride,
        size,
        used,
        modifier,
        handle_bytes,
        data,
    })
}

pub(crate) fn write_plane_element(w: &mut CdrWriter<'_>, p: &TensorPlaneView<'_>) {
    w.write_i64(p.handle);
    w.write_u64(p.offset);
    w.write_u64(p.stride);
    w.write_u64(p.size);
    w.write_u64(p.used);
    w.write_u64(p.modifier);
    w.write_bytes(p.handle_bytes);
    w.write_bytes(p.data);
}

pub(crate) fn size_plane_element(s: &mut CdrSizer, handle_bytes_len: usize, data_len: usize) {
    s.size_i64();
    s.size_u64();
    s.size_u64();
    s.size_u64();
    s.size_u64();
    s.size_u64();
    s.size_bytes(handle_bytes_len);
    s.size_bytes(data_len);
}

/// Validate one plane against the schema contract (see `TensorPlane.msg`).
pub(crate) fn validate_plane(p: &TensorPlaneView<'_>) -> Result<(), CdrError> {
    if p.handle < -1 || p.used > p.size {
        return Err(CdrError::InvalidHeader);
    }
    if p.handle >= 0 {
        // Referenced: bytes live behind the handle, never inline.
        if !p.data.is_empty() {
            return Err(CdrError::InvalidHeader);
        }
    } else {
        // Inline: `size` describes `data`, and an inline buffer is never
        // tiled or backed by an opaque handle.
        if p.size as usize != p.data.len() || p.modifier != 0 || !p.handle_bytes.is_empty() {
            return Err(CdrError::InvalidHeader);
        }
    }
    Ok(())
}

/// Validate a whole plane sequence.
///
/// Beyond per-plane rules: all planes are inline, or none are. A frame that
/// mixes transport modes has no coherent meaning — the tensor has a single
/// `storage_kind`, `pid` and `fence_fd` covering every plane.
pub(crate) fn validate_plane_set(planes: &[TensorPlaneView<'_>]) -> Result<(), CdrError> {
    let mut mode: Option<bool> = None;
    for p in planes {
        validate_plane(p)?;
        match mode {
            None => mode = Some(p.is_inline()),
            Some(m) if m == p.is_inline() => {}
            Some(_) => return Err(CdrError::InvalidHeader),
        }
    }
    Ok(())
}

/// Cached positions of a `Tensor`'s variable-length fields.
///
/// Populated during the single validating walk in [`scan_tensor`]. Every
/// position is an absolute buffer offset pointing at the field's sequence
/// count (or, for `strings`, at the `format` length prefix).
#[derive(Copy, Clone, Debug)]
pub(crate) struct TensorOffsets {
    /// Absolute position of `storage_kind`; all five scalars are at fixed
    /// offsets from here. Guaranteed `(base - CDR_HEADER_SIZE) % 8 == 0`.
    pub(crate) base: usize,
    pub(crate) shape: usize,
    pub(crate) strides: usize,
    pub(crate) quant_scales: usize,
    pub(crate) quant_zero_points: usize,
    pub(crate) strings: usize,
    pub(crate) planes: usize,
}

/// Read a sequence header, bound the count against the remaining buffer, then
/// skip the elements. Returns the element count.
///
/// `check_seq_count` is the denial-of-service guard — it rejects a forged count
/// against the bytes actually remaining before any large multiply. The skip
/// itself reuses `CdrCursor::skip_seq_{4,8}`, which already handle alignment
/// and bounds; do not hand-roll an align+skip here.
fn read_seq_header(c: &mut CdrCursor<'_>, elem_bytes: usize) -> Result<usize, CdrError> {
    let raw = c.read_seq_len()?;
    c.check_seq_count(raw, elem_bytes)
}

/// Walk a `Tensor` at `base`, validating structure and recording offsets.
///
/// Validates only structure, never meaning: `dtype`, `storage_kind`, `format`
/// and `modifier` are carried verbatim. In particular this MUST NOT check
/// `shape` against buffer size — `shape` is the addressing grid, so for a
/// subsampled format such as NV12 `product(shape) * dtype_size` is smaller
/// than the allocation by design.
pub(crate) fn scan_tensor(buf: &[u8], base: usize) -> Result<TensorOffsets, CdrError> {
    // The position-independence invariant. Without this every wrapper would
    // produce a different interior layout for the same tensor.
    if base < CDR_HEADER_SIZE || !(base - CDR_HEADER_SIZE).is_multiple_of(8) {
        return Err(CdrError::InvalidHeader);
    }

    let mut c = CdrCursor::resume(buf, base);
    c.read_u32()?; // storage_kind — carried, not interpreted
    c.read_u32()?; // pid
    c.read_i32()?; // fence_fd
    c.read_u32()?; // dtype — carried, not interpreted
    let quant_axis = c.read_i32()?;

    // Each cached position is recorded AFTER aligning to 4, so accessors can
    // read the sequence count directly without re-aligning. This matters most
    // for `planes`, which follows `color_range` — a string ends at an
    // arbitrary byte, so the raw cursor offset there is usually misaligned.
    c.align(4);
    let shape_pos = c.offset();
    let shape_count = read_seq_header(&mut c, 8)?;
    c.skip_seq_8(shape_count)?;

    c.align(4);
    let strides_pos = c.offset();
    let strides_count = read_seq_header(&mut c, 8)?;
    c.skip_seq_8(strides_count)?;

    c.align(4);
    let quant_scales_pos = c.offset();
    let scales_count = read_seq_header(&mut c, 4)?;
    c.skip_seq_4(scales_count)?;

    c.align(4);
    let quant_zero_points_pos = c.offset();
    let zeros_count = read_seq_header(&mut c, 4)?;
    c.skip_seq_4(zeros_count)?;

    c.align(4);
    let strings_pos = c.offset();
    let format = c.read_string()?;
    let color_space = c.read_string()?;
    let color_transfer = c.read_string()?;
    let color_encoding = c.read_string()?;
    let color_range = c.read_string()?;

    c.align(4);
    let planes_pos = c.offset();
    let raw_planes = c.read_seq_len()?;
    let plane_count = c.check_seq_count(raw_planes, MIN_PLANE_BYTES)?;
    // Track transport mode inline rather than collecting, so from_cdr never
    // allocates.
    let mut inline_mode: Option<bool> = None;
    for _ in 0..plane_count {
        let p = scan_plane_element(&mut c)?;
        validate_plane(&p)?;
        match inline_mode {
            None => inline_mode = Some(p.is_inline()),
            Some(m) if m == p.is_inline() => {}
            Some(_) => return Err(CdrError::InvalidHeader),
        }
    }

    // ── Structural validation ────────────────────────────────────────
    // strides: absent (contiguous) or one entry per dimension.
    if strides_count != 0 && strides_count != shape_count {
        return Err(CdrError::InvalidHeader);
    }

    // Quantization arity, per Tensor.msg.
    match quant_axis {
        -2 => {
            if scales_count != 0 {
                return Err(CdrError::InvalidHeader);
            }
        }
        -1 => {
            if scales_count != 1 {
                return Err(CdrError::InvalidHeader);
            }
        }
        a if a >= 0 => {
            let axis = a as usize;
            if axis >= shape_count {
                return Err(CdrError::InvalidHeader);
            }
            // Element read, not a slice transmute: CDR 8-byte alignment lands
            // at absolute offset 4 mod 8, so a &[u64] view here would be UB.
            // See the NOTE above rd_slice_i32 in src/cdr.rs.
            let dim = rd_u64(buf, cdr_align(shape_pos + 4, 8) + axis * 8);
            if scales_count as u64 != dim {
                return Err(CdrError::InvalidHeader);
            }
        }
        _ => return Err(CdrError::InvalidHeader), // < -2
    }
    if zeros_count != 0 && zeros_count != scales_count {
        return Err(CdrError::InvalidHeader);
    }

    // Colorimetry is meaningless without a format.
    if format.is_empty()
        && !(color_space.is_empty()
            && color_transfer.is_empty()
            && color_encoding.is_empty()
            && color_range.is_empty())
    {
        return Err(CdrError::InvalidHeader);
    }

    Ok(TensorOffsets {
        base,
        shape: shape_pos,
        strides: strides_pos,
        quant_scales: quant_scales_pos,
        quant_zero_points: quant_zero_points_pos,
        strings: strings_pos,
        planes: planes_pos,
    })
}

/// Zero-copy view of a `Tensor`, standalone or embedded in a wrapper message.
///
/// The five scalars sit in a 20-byte prefix at fixed offsets from
/// [`base`](Self::base), so reads and in-place writes are O(1). Variable-length
/// fields are reached through the offset table built during `from_cdr`.
#[derive(Clone, Debug)]
pub struct Tensor<B> {
    buf: B,
    off: TensorOffsets,
}

impl<B> Tensor<B> {
    /// Replace the buffer container while keeping the parsed offset table.
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> Tensor<C> {
        Tensor {
            buf: f(self.buf),
            off: self.off,
        }
    }

    /// Absolute buffer position of `storage_kind`, the start of the tensor.
    #[inline]
    pub fn base(&self) -> usize {
        self.off.base
    }

    /// The backing buffer container itself, not its bytes.
    ///
    /// [`as_cdr`](Self::as_cdr) yields `&[u8]`, which is what almost every
    /// caller wants. This returns `&B` so a binding that owns a shareable
    /// container — a refcounted Python `bytes`, say — can duplicate the
    /// container to hand out a second view over the same bytes rather than
    /// copying them.
    #[inline]
    pub fn buffer(&self) -> &B {
        &self.buf
    }
}

impl<B: AsRef<[u8]>> Tensor<B> {
    /// Parse a standalone `Tensor` message.
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        Self::from_cdr_at(buf, CDR_HEADER_SIZE)
    }

    /// Parse a `Tensor` embedded at `base` in a larger CDR buffer.
    ///
    /// `base` MUST be 8-aligned relative to the CDR data start; a misaligned
    /// base is rejected rather than tolerated, because it would produce an
    /// interior layout incompatible with every other message.
    pub fn from_cdr_at(buf: B, base: usize) -> Result<Self, CdrError> {
        let off = scan_tensor(buf.as_ref(), base)?;
        Ok(Tensor { buf, off })
    }

    /// Build a view from an offset table a wrapper already parsed. No rescan.
    ///
    /// Consumed by Task 7's wrapper messages (`TensorStamped`, `CameraFrame`),
    /// which parse their own header then hand off the already-scanned offsets.
    pub(crate) fn from_parts(buf: B, off: TensorOffsets) -> Self {
        Tensor { buf, off }
    }

    #[inline]
    pub fn storage_kind(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.off.base)
    }
    #[inline]
    pub fn pid(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.off.base + 4)
    }
    #[inline]
    pub fn fence_fd(&self) -> i32 {
        rd_i32(self.buf.as_ref(), self.off.base + 8)
    }
    #[inline]
    pub fn dtype(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.off.base + 12)
    }
    #[inline]
    pub fn quant_axis(&self) -> i32 {
        rd_i32(self.buf.as_ref(), self.off.base + 16)
    }

    /// Dimensions, outermost first. Allocation-free iterator.
    ///
    /// This is the *addressing grid*, not the byte layout. For a subsampled
    /// format such as NV12 the product of `shape` times the element size is
    /// smaller than the allocation; extent lives in `planes[].size`.
    ///
    /// Returns an iterator rather than `&[u64]` because a borrowed slice of
    /// 64-bit CDR elements is unsound — see `DimIter`.
    #[inline]
    pub fn shape(&self) -> DimIter<'_, u64> {
        let b = self.buf.as_ref();
        DimIter::new(
            b,
            cdr_align(self.off.shape + 4, 8),
            rd_u32(b, self.off.shape) as usize,
        )
    }

    #[inline]
    pub fn shape_len(&self) -> usize {
        rd_u32(self.buf.as_ref(), self.off.shape) as usize
    }

    #[inline]
    pub fn shape_at(&self, index: usize) -> Option<u64> {
        (index < self.shape_len()).then(|| {
            rd_u64(
                self.buf.as_ref(),
                cdr_align(self.off.shape + 4, 8) + index * 8,
            )
        })
    }

    /// Stride per dimension in BYTES. Empty means densely packed C-order.
    #[inline]
    pub fn strides(&self) -> DimIter<'_, i64> {
        let b = self.buf.as_ref();
        DimIter::new(
            b,
            cdr_align(self.off.strides + 4, 8),
            rd_u32(b, self.off.strides) as usize,
        )
    }

    #[inline]
    pub fn strides_len(&self) -> usize {
        rd_u32(self.buf.as_ref(), self.off.strides) as usize
    }

    #[inline]
    pub fn strides_at(&self, index: usize) -> Option<i64> {
        (index < self.strides_len()).then(|| {
            rd_i64(
                self.buf.as_ref(),
                cdr_align(self.off.strides + 4, 8) + index * 8,
            )
        })
    }

    #[inline]
    pub fn quant_scales(&self) -> &[f32] {
        let b = self.buf.as_ref();
        let count = rd_u32(b, self.off.quant_scales) as usize;
        rd_slice_f32(b, align(self.off.quant_scales + 4, 4), count)
    }

    #[inline]
    pub fn quant_zero_points(&self) -> &[i32] {
        let b = self.buf.as_ref();
        let count = rd_u32(b, self.off.quant_zero_points) as usize;
        rd_slice_i32(b, align(self.off.quant_zero_points + 4, 4), count)
    }

    /// Walk the five adjacent strings, returning all of them.
    ///
    /// String accessors unavoidably re-walk their predecessors because CDR
    /// string lengths are variable. The walk is bounded at five, which is why
    /// these fields are adjacent in the layout, and it never touches the
    /// arrays or planes — those use cached offsets.
    fn scan_strings(&self) -> (&str, &str, &str, &str, &str) {
        let b = self.buf.as_ref();
        let (format, p1) = rd_string(b, self.off.strings);
        let (cs, p2) = rd_string(b, p1);
        let (ct, p3) = rd_string(b, p2);
        let (ce, p4) = rd_string(b, p3);
        let (cr, _) = rd_string(b, p4);
        (format, cs, ct, ce, cr)
    }

    /// Format descriptor; `""` means the tensor is not an image.
    #[inline]
    pub fn format(&self) -> &str {
        self.scan_strings().0
    }
    #[inline]
    pub fn color_space(&self) -> &str {
        self.scan_strings().1
    }
    #[inline]
    pub fn color_transfer(&self) -> &str {
        self.scan_strings().2
    }
    #[inline]
    pub fn color_encoding(&self) -> &str {
        self.scan_strings().3
    }
    #[inline]
    pub fn color_range(&self) -> &str {
        self.scan_strings().4
    }

    /// Number of planes. O(1) via the cached offset.
    #[inline]
    pub fn num_planes(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.off.planes)
    }

    /// Allocation-free forward iterator over the planes.
    ///
    /// Prefer this to `planes_vec()`; collecting is opt-in precisely because
    /// the old `CameraFrame::planes()` allocated on every call.
    pub fn planes(&self) -> TensorPlaneIter<'_> {
        let b = self.buf.as_ref();
        TensorPlaneIter {
            cursor: CdrCursor::resume(b, self.off.planes + 4),
            remaining: rd_u32(b, self.off.planes) as usize,
        }
    }

    /// One plane by index. Scans to `index` (O(n)) without allocating; for a
    /// full traversal use [`planes`](Self::planes) to avoid O(n²).
    pub fn plane_at(&self, index: usize) -> Option<TensorPlaneView<'_>> {
        self.planes().nth(index)
    }

    /// Collect every plane. Allocates — use [`planes`](Self::planes) unless a
    /// materialized slice is genuinely required.
    pub fn planes_vec(&self) -> Vec<TensorPlaneView<'_>> {
        self.planes().collect()
    }

    /// This tensor's own bytes, from `base` to the end of the buffer.
    ///
    /// `Tensor` is always the last field of its wrapper, so this is exactly
    /// the tensor and nothing else.
    #[inline]
    pub fn tensor_bytes(&self) -> &[u8] {
        &self.buf.as_ref()[self.off.base..]
    }

    /// Re-head this tensor as a standalone `Tensor` CDR message.
    ///
    /// Copies the tensor's metadata bytes behind a fresh CDR encapsulation
    /// header. Because the layout is position-independent, the result is
    /// byte-identical to encoding the same tensor standalone from scratch.
    ///
    /// This is the republish path — forwarding a camera frame's tensor onto a
    /// tensor topic. It copies only metadata; plane payloads stay behind their
    /// handles. To *read* an embedded tensor, use `from_cdr_at` instead, which
    /// copies nothing at all.
    pub fn to_standalone_cdr(&self) -> Vec<u8> {
        let body = self.tensor_bytes();
        let mut out = Vec::with_capacity(CDR_HEADER_SIZE + body.len());
        out.extend_from_slice(&self.buf.as_ref()[..CDR_HEADER_SIZE]);
        out.extend_from_slice(body);
        out
    }

    /// The whole backing buffer. For an embedded tensor this is the wrapper's
    /// buffer, not the tensor's slice — use `tensor_bytes` for that.
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
}

/// Accessors for a `Tensor` whose buffer is itself a borrow.
///
/// [`Tensor::planes`] ties every `TensorPlaneView` to the borrow of `&self`,
/// which is correct for iteration but useless to a caller that needs to
/// *store* the views — the C FFI handle materializes its child planes once at
/// `from_cdr` time and hands out borrowed pointers to them for the handle's
/// whole life.
///
/// When the buffer is already a borrow, the views can carry the buffer's
/// lifetime directly. That keeps the FFI free of the `mem::transmute`
/// lifetime-widening the module preamble in `ffi.rs` explicitly rules out:
/// the `'static` views it stores come from the `&'static [u8]` input, not
/// from widening a `&self`-bound return.
impl<'a> Tensor<&'a [u8]> {
    /// Plane iterator whose views borrow the backing buffer, not `self`.
    ///
    /// Identical to [`planes`](Self::planes) in cost and output — allocation
    /// free, same elements, same order — differing only in the lifetime the
    /// yielded views carry.
    pub fn planes_borrowed(&self) -> TensorPlaneIter<'a> {
        let b: &'a [u8] = self.buf;
        TensorPlaneIter {
            cursor: CdrCursor::resume(b, self.off.planes + 4),
            remaining: rd_u32(b, self.off.planes) as usize,
        }
    }
}

/// Allocation-free iterator over a 64-bit CDR sequence (`shape`, `strides`).
///
/// These cannot be exposed as `&[u64]` / `&[i64]`. CDR 8-byte alignment is
/// relative to the data start at absolute offset 4, so the elements sit at
/// absolute offset ≡ 4 (mod 8) and can never satisfy `align_of::<u64>()` —
/// a slice transmute there is undefined behaviour. Elements are read with
/// `rd_u64`/`rd_i64`, which go through `from_le_bytes` and are
/// alignment-safe. Nothing is allocated and nothing is copied beyond the
/// eight bytes of the element being yielded.
pub struct DimIter<'a, T> {
    buf: &'a [u8],
    pos: usize,
    remaining: usize,
    _marker: std::marker::PhantomData<T>,
}

impl<'a, T> DimIter<'a, T> {
    #[inline]
    fn new(buf: &'a [u8], pos: usize, count: usize) -> Self {
        DimIter {
            buf,
            pos,
            remaining: count,
            _marker: std::marker::PhantomData,
        }
    }
}

macro_rules! dim_iter_impl {
    ($t:ty, $rd:path) => {
        impl Iterator for DimIter<'_, $t> {
            type Item = $t;

            fn next(&mut self) -> Option<$t> {
                if self.remaining == 0 {
                    return None;
                }
                self.remaining -= 1;
                let v = $rd(self.buf, self.pos);
                self.pos += 8;
                Some(v)
            }

            fn size_hint(&self) -> (usize, Option<usize>) {
                (self.remaining, Some(self.remaining))
            }
        }

        impl ExactSizeIterator for DimIter<'_, $t> {}
    };
}

dim_iter_impl!(u64, rd_u64);
dim_iter_impl!(i64, rd_i64);

/// Allocation-free forward iterator over a `Tensor`'s planes.
pub struct TensorPlaneIter<'a> {
    cursor: CdrCursor<'a>,
    remaining: usize,
}

impl<'a> Iterator for TensorPlaneIter<'a> {
    type Item = TensorPlaneView<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }
        self.remaining -= 1;
        Some(scan_plane_element(&mut self.cursor).expect("planes validated during from_cdr"))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}

impl ExactSizeIterator for TensorPlaneIter<'_> {}

/// The complete field set of a `Tensor`, borrowed from caller memory.
///
/// Shared by `TensorBuilder` and by the wrapper builders so the tensor body is
/// encoded by exactly one piece of code regardless of which message carries it.
/// All borrows must stay valid until the encode call returns.
#[derive(Clone, Debug)]
pub struct TensorFields<'a> {
    pub storage_kind: u32,
    pub pid: u32,
    pub fence_fd: i32,
    pub dtype: u32,
    pub quant_axis: i32,
    pub shape: &'a [u64],
    pub strides: &'a [i64],
    pub quant_scales: &'a [f32],
    pub quant_zero_points: &'a [i32],
    pub format: Cow<'a, str>,
    pub color_space: Cow<'a, str>,
    pub color_transfer: Cow<'a, str>,
    pub color_encoding: Cow<'a, str>,
    pub color_range: Cow<'a, str>,
    pub planes: &'a [TensorPlaneView<'a>],
}

impl Default for TensorFields<'_> {
    fn default() -> Self {
        Self {
            storage_kind: 0,
            pid: 0,
            fence_fd: -1,
            dtype: 0,
            quant_axis: -2,
            shape: &[],
            strides: &[],
            quant_scales: &[],
            quant_zero_points: &[],
            format: Cow::Borrowed(""),
            color_space: Cow::Borrowed(""),
            color_transfer: Cow::Borrowed(""),
            color_encoding: Cow::Borrowed(""),
            color_range: Cow::Borrowed(""),
            planes: &[],
        }
    }
}

impl TensorFields<'_> {
    /// Structural validation, mirroring `scan_tensor`. Never checks meaning,
    /// and never checks `shape` against buffer size.
    pub fn validate(&self) -> Result<(), CdrError> {
        validate_plane_set(self.planes)?;

        if !self.strides.is_empty() && self.strides.len() != self.shape.len() {
            return Err(CdrError::InvalidHeader);
        }

        match self.quant_axis {
            -2 => {
                if !self.quant_scales.is_empty() {
                    return Err(CdrError::InvalidHeader);
                }
            }
            -1 => {
                if self.quant_scales.len() != 1 {
                    return Err(CdrError::InvalidHeader);
                }
            }
            a if a >= 0 => {
                let axis = a as usize;
                if axis >= self.shape.len() || self.quant_scales.len() as u64 != self.shape[axis] {
                    return Err(CdrError::InvalidHeader);
                }
            }
            _ => return Err(CdrError::InvalidHeader),
        }
        if !self.quant_zero_points.is_empty()
            && self.quant_zero_points.len() != self.quant_scales.len()
        {
            return Err(CdrError::InvalidHeader);
        }

        if self.format.is_empty()
            && !(self.color_space.is_empty()
                && self.color_transfer.is_empty()
                && self.color_encoding.is_empty()
                && self.color_range.is_empty())
        {
            return Err(CdrError::InvalidHeader);
        }
        Ok(())
    }

    /// Accumulate this tensor's encoded size. Field order MUST match
    /// `write_into` and `scan_tensor` exactly.
    pub fn size_into(&self, s: &mut CdrSizer) {
        s.size_u32(); // storage_kind
        s.size_u32(); // pid
        s.size_i32(); // fence_fd
        s.size_u32(); // dtype
        s.size_i32(); // quant_axis
                      // CdrSizer::size_seq_N sizes ELEMENTS ONLY -- the 4-byte length prefix
                      // is NOT included, so each sequence needs its own size_u32() here to
                      // match the write_u32(len) in write_into. Omitting these under-
                      // allocates by 16 bytes and CdrWriter::finish() fails on every encode.
                      // (Existing precedent: RadarCube, src/edgefirst_msgs.rs:946-953.)
        s.size_u32();
        s.size_seq_8(self.shape.len());
        s.size_u32();
        s.size_seq_8(self.strides.len());
        s.size_u32();
        s.size_seq_4(self.quant_scales.len());
        s.size_u32();
        s.size_seq_4(self.quant_zero_points.len());
        s.size_string(&self.format);
        s.size_string(&self.color_space);
        s.size_string(&self.color_transfer);
        s.size_string(&self.color_encoding);
        s.size_string(&self.color_range);
        s.size_u32(); // planes count
        for p in self.planes {
            size_plane_element(s, p.handle_bytes.len(), p.data.len());
        }
    }

    /// Write this tensor's body. Field order MUST match `size_into` and
    /// `scan_tensor` exactly.
    pub fn write_into(&self, w: &mut CdrWriter<'_>) {
        w.write_u32(self.storage_kind);
        w.write_u32(self.pid);
        w.write_i32(self.fence_fd);
        w.write_u32(self.dtype);
        w.write_i32(self.quant_axis);
        w.write_u32(self.shape.len() as u32);
        w.write_slice_u64(self.shape);
        w.write_u32(self.strides.len() as u32);
        w.write_slice_i64(self.strides);
        w.write_u32(self.quant_scales.len() as u32);
        w.write_slice_f32(self.quant_scales);
        w.write_u32(self.quant_zero_points.len() as u32);
        w.write_slice_i32(self.quant_zero_points);
        w.write_string(&self.format);
        w.write_string(&self.color_space);
        w.write_string(&self.color_transfer);
        w.write_string(&self.color_encoding);
        w.write_string(&self.color_range);
        w.write_u32(self.planes.len() as u32);
        for p in self.planes {
            write_plane_element(w, p);
        }
    }
}

/// Builder for a standalone `Tensor<Vec<u8>>`, with buffer-reuse finalizers.
#[derive(Clone, Debug, Default)]
pub struct TensorBuilder<'a> {
    fields: TensorFields<'a>,
}

impl<'a> TensorBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    /// The underlying field set, for wrapper builders that embed a tensor.
    pub fn fields(&self) -> &TensorFields<'a> {
        &self.fields
    }

    pub fn storage_kind(&mut self, v: u32) -> &mut Self {
        self.fields.storage_kind = v;
        self
    }
    pub fn pid(&mut self, v: u32) -> &mut Self {
        self.fields.pid = v;
        self
    }
    pub fn fence_fd(&mut self, v: i32) -> &mut Self {
        self.fields.fence_fd = v;
        self
    }
    pub fn dtype(&mut self, v: u32) -> &mut Self {
        self.fields.dtype = v;
        self
    }
    pub fn quant_axis(&mut self, v: i32) -> &mut Self {
        self.fields.quant_axis = v;
        self
    }
    pub fn shape(&mut self, v: &'a [u64]) -> &mut Self {
        self.fields.shape = v;
        self
    }
    pub fn strides(&mut self, v: &'a [i64]) -> &mut Self {
        self.fields.strides = v;
        self
    }
    pub fn quant_scales(&mut self, v: &'a [f32]) -> &mut Self {
        self.fields.quant_scales = v;
        self
    }
    pub fn quant_zero_points(&mut self, v: &'a [i32]) -> &mut Self {
        self.fields.quant_zero_points = v;
        self
    }
    pub fn format(&mut self, s: impl Into<Cow<'a, str>>) -> &mut Self {
        self.fields.format = s.into();
        self
    }
    pub fn color_space(&mut self, s: impl Into<Cow<'a, str>>) -> &mut Self {
        self.fields.color_space = s.into();
        self
    }
    pub fn color_transfer(&mut self, s: impl Into<Cow<'a, str>>) -> &mut Self {
        self.fields.color_transfer = s.into();
        self
    }
    pub fn color_encoding(&mut self, s: impl Into<Cow<'a, str>>) -> &mut Self {
        self.fields.color_encoding = s.into();
        self
    }
    pub fn color_range(&mut self, s: impl Into<Cow<'a, str>>) -> &mut Self {
        self.fields.color_range = s.into();
        self
    }
    pub fn planes(&mut self, p: &'a [TensorPlaneView<'a>]) -> &mut Self {
        self.fields.planes = p;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        self.fields.size_into(&mut s);
        s.size()
    }

    fn write_all(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.fields.write_into(&mut w);
        w.finish()
    }

    pub fn build(&self) -> Result<Tensor<Vec<u8>>, CdrError> {
        self.fields.validate()?;
        let mut buf = vec![0u8; self.size()];
        self.write_all(&mut buf)?;
        Tensor::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        self.fields.validate()?;
        buf.resize(self.size(), 0);
        self.write_all(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        self.fields.validate()?;
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_all(&mut buf[..need])?;
        Ok(need)
    }
}

impl Tensor<Vec<u8>> {
    /// Start a `TensorBuilder` with zero-valued defaults
    /// (`fence_fd = -1`, `quant_axis = -2`).
    pub fn builder<'a>() -> TensorBuilder<'a> {
        TensorBuilder::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cdr::{CdrCursor, CdrSizer, CdrWriter};

    fn referenced_plane() -> TensorPlaneView<'static> {
        TensorPlaneView {
            handle: 7,
            offset: 4096,
            stride: 1920,
            size: 1920 * 1080,
            used: 1920 * 1080,
            modifier: 0,
            handle_bytes: &[],
            data: &[],
        }
    }

    fn roundtrip(p: &TensorPlaneView<'_>) -> Vec<u8> {
        let mut s = CdrSizer::new();
        size_plane_element(&mut s, p.handle_bytes.len(), p.data.len());
        let mut buf = vec![0u8; s.size()];
        let mut w = CdrWriter::new(&mut buf).unwrap();
        write_plane_element(&mut w, p);
        w.finish().unwrap();
        buf
    }

    #[test]
    fn plane_element_roundtrips() {
        let p = referenced_plane();
        let buf = roundtrip(&p);
        let mut c = CdrCursor::new(&buf).unwrap();
        let got = scan_plane_element(&mut c).unwrap();
        assert_eq!(got.handle, p.handle);
        assert_eq!(got.offset, p.offset);
        assert_eq!(got.stride, p.stride);
        assert_eq!(got.size, p.size);
        assert_eq!(got.used, p.used);
        assert_eq!(got.modifier, p.modifier);
        assert!(got.handle_bytes.is_empty());
        assert!(got.data.is_empty());
    }

    #[test]
    fn plane_scalar_block_is_48_bytes_with_no_interior_padding() {
        // The six 8-byte scalars must be contiguous. Sizing an element with
        // both sequences empty gives 48 + 4 + 4 = 56 bytes past the CDR header.
        let mut s = CdrSizer::new();
        size_plane_element(&mut s, 0, 0);
        assert_eq!(s.size() - crate::cdr::CDR_HEADER_SIZE, MIN_PLANE_BYTES);
        assert_eq!(MIN_PLANE_BYTES, 56);
    }

    #[test]
    fn inline_plane_accepts_matching_size() {
        let data = [1u8, 2, 3, 4];
        let p = TensorPlaneView {
            handle: -1,
            offset: 0,
            stride: 4,
            size: 4,
            used: 4,
            modifier: 0,
            handle_bytes: &[],
            data: &data,
        };
        validate_plane(&p).unwrap();
    }

    #[test]
    fn plane_validation_rejects_contract_violations() {
        // Each sub-case starts from a VALID plane, asserts it validates, then
        // introduces exactly one violation. Without the positive control an
        // assertion can pass because an unrelated earlier guard fired -- which
        // is how the first version of this test silently lost coverage on four
        // of its six rules.
        let bytes = [0u8; 4];

        let referenced_ok = referenced_plane();
        validate_plane(&referenced_ok).unwrap();

        let inline_ok = TensorPlaneView {
            handle: -1,
            offset: 0,
            stride: 4,
            size: 4,
            used: 4,
            modifier: 0,
            handle_bytes: &[],
            data: &bytes,
        };
        validate_plane(&inline_ok).unwrap();

        // handle < -1 is never valid. Isolated: every other field is valid for
        // the inline branch this handle would otherwise fall into.
        let mut p = inline_ok;
        p.handle = -2;
        assert!(validate_plane(&p).is_err());

        // used > size.
        let mut p = referenced_ok;
        p.used = p.size + 1;
        assert!(validate_plane(&p).is_err());

        // Referenced plane must not carry inline bytes.
        let mut p = referenced_ok;
        p.data = &bytes;
        assert!(validate_plane(&p).is_err());

        // Inline plane: size must equal data length.
        let mut p = inline_ok;
        p.size = 8;
        p.used = 8;
        assert!(validate_plane(&p).is_err());

        // Inline plane must have modifier == 0 -- an inline buffer is never tiled.
        let mut p = inline_ok;
        p.modifier = 1;
        assert!(validate_plane(&p).is_err());

        // Inline plane must not carry an opaque handle.
        let mut p = inline_ok;
        p.handle_bytes = &bytes;
        assert!(validate_plane(&p).is_err());
    }

    #[test]
    fn plane_set_rejects_mixed_transport_modes() {
        let bytes = [0u8; 4];
        let referenced = referenced_plane();
        let inline = TensorPlaneView {
            handle: -1,
            offset: 0,
            stride: 4,
            size: 4,
            used: 4,
            modifier: 0,
            handle_bytes: &[],
            data: &bytes,
        };
        validate_plane_set(&[referenced, referenced]).unwrap();
        validate_plane_set(&[inline, inline]).unwrap();
        validate_plane_set(&[]).unwrap();
        assert!(validate_plane_set(&[referenced, inline]).is_err());
    }

    /// Hand-encode a standalone Tensor with the given scalars and empty
    /// variable fields, so the view can be tested before the builder exists.
    fn encode_minimal_tensor(
        storage_kind: u32,
        pid: u32,
        fence_fd: i32,
        dtype: u32,
        quant_axis: i32,
    ) -> Vec<u8> {
        let mut s = CdrSizer::new();
        s.size_u32(); // storage_kind
        s.size_u32(); // pid
        s.size_i32(); // fence_fd
        s.size_u32(); // dtype
        s.size_i32(); // quant_axis
                      // size_seq_N sizes ELEMENTS ONLY -- each needs its own u32 count prefix.
        s.size_u32();
        s.size_seq_8(0); // shape
        s.size_u32();
        s.size_seq_8(0); // strides
        s.size_u32();
        s.size_seq_4(0); // quant_scales
        s.size_u32();
        s.size_seq_4(0); // quant_zero_points
        s.size_string(""); // format
        s.size_string(""); // color_space
        s.size_string(""); // color_transfer
        s.size_string(""); // color_encoding
        s.size_string(""); // color_range
        s.size_u32(); // planes count
        let mut buf = vec![0u8; s.size()];
        let mut w = CdrWriter::new(&mut buf).unwrap();
        w.write_u32(storage_kind);
        w.write_u32(pid);
        w.write_i32(fence_fd);
        w.write_u32(dtype);
        w.write_i32(quant_axis);
        w.write_u32(0); // shape
        w.write_u32(0); // strides
        w.write_u32(0); // quant_scales
        w.write_u32(0); // quant_zero_points
        w.write_string("");
        w.write_string("");
        w.write_string("");
        w.write_string("");
        w.write_string("");
        w.write_u32(0); // planes
        w.finish().unwrap();
        buf
    }

    /// Hand-encode a two-plane NV12-shaped tensor with populated arrays and
    /// strings, so every accessor has something non-trivial to return.
    fn encode_rich_tensor() -> Vec<u8> {
        let shape: [u64; 2] = [1080, 1920];
        let strides: [i64; 2] = [2048, 1];
        let scales: [f32; 1] = [0.0125];
        let zeros: [i32; 1] = [-7];
        let planes = [
            TensorPlaneView {
                handle: 5,
                offset: 0,
                stride: 2048,
                size: 2048 * 1080,
                used: 2048 * 1080,
                modifier: 0,
                handle_bytes: &[],
                data: &[],
            },
            TensorPlaneView {
                handle: 5,
                offset: 2048 * 1080,
                stride: 2048,
                size: 2048 * 540,
                // Deliberately < size: a partially-filled (e.g. compressed)
                // plane, and distinct from `size` so a test that swaps the
                // size/used read or write order cannot pass by accident.
                used: 2048 * 540 - 1000,
                modifier: 1, // chroma tiled, luma linear -- legal per plane
                handle_bytes: &[],
                data: &[],
            },
        ];

        let mut s = CdrSizer::new();
        s.size_u32();
        s.size_u32();
        s.size_i32();
        s.size_u32();
        s.size_i32();
        // size_seq_N sizes ELEMENTS ONLY -- each needs its own u32 count prefix.
        s.size_u32();
        s.size_seq_8(shape.len());
        s.size_u32();
        s.size_seq_8(strides.len());
        s.size_u32();
        s.size_seq_4(scales.len());
        s.size_u32();
        s.size_seq_4(zeros.len());
        s.size_string("NV12");
        s.size_string("bt709");
        s.size_string("bt709");
        s.size_string("bt601");
        s.size_string("limited");
        s.size_u32();
        for p in &planes {
            size_plane_element(&mut s, p.handle_bytes.len(), p.data.len());
        }

        let mut buf = vec![0u8; s.size()];
        let mut w = CdrWriter::new(&mut buf).unwrap();
        w.write_u32(2); // storage_kind
        w.write_u32(99); // pid
        w.write_i32(-1); // fence_fd
        w.write_u32(1); // dtype
        w.write_i32(-1); // quant_axis: per-tensor, so exactly one scale
        w.write_u32(shape.len() as u32);
        w.write_slice_u64(&shape);
        w.write_u32(strides.len() as u32);
        w.write_slice_i64(&strides);
        w.write_u32(scales.len() as u32);
        w.write_slice_f32(&scales);
        w.write_u32(zeros.len() as u32);
        w.write_slice_i32(&zeros);
        w.write_string("NV12");
        w.write_string("bt709");
        w.write_string("bt709");
        w.write_string("bt601");
        w.write_string("limited");
        w.write_u32(planes.len() as u32);
        for p in &planes {
            write_plane_element(&mut w, p);
        }
        w.finish().unwrap();
        buf
    }

    #[test]
    fn tensor_arrays_are_borrowed_slices() {
        let buf = encode_rich_tensor();
        let t = Tensor::<&[u8]>::from_cdr(&buf[..]).unwrap();
        assert_eq!(t.shape().collect::<Vec<_>>(), vec![1080u64, 1920]);
        assert_eq!(t.strides().collect::<Vec<_>>(), vec![2048i64, 1]);
        assert_eq!(t.quant_scales(), &[0.0125f32]);
        assert_eq!(t.quant_zero_points(), &[-7i32]);
    }

    #[test]
    fn tensor_strings_are_readable() {
        let buf = encode_rich_tensor();
        let t = Tensor::<&[u8]>::from_cdr(&buf[..]).unwrap();
        assert_eq!(t.format(), "NV12");
        assert_eq!(t.color_space(), "bt709");
        assert_eq!(t.color_transfer(), "bt709");
        assert_eq!(t.color_encoding(), "bt601");
        assert_eq!(t.color_range(), "limited");
    }

    #[test]
    fn plane_iterator_yields_every_plane_with_per_plane_modifier() {
        let buf = encode_rich_tensor();
        let t = Tensor::<&[u8]>::from_cdr(&buf[..]).unwrap();
        assert_eq!(t.num_planes(), 2);

        let planes: Vec<_> = t.planes().collect();
        assert_eq!(planes.len(), 2);
        assert_eq!(planes[0].modifier, 0);
        assert_eq!(planes[1].modifier, 1);
        assert_eq!(planes[1].offset, 2048 * 1080);

        // `size` and `used` are adjacent same-width fields; assert both with
        // distinct expected values so a transposition of the two in
        // write_plane_element/scan_plane_element cannot pass unnoticed.
        assert_eq!(planes[0].size, 2048 * 1080);
        assert_eq!(planes[0].used, 2048 * 1080);
        assert_eq!(planes[1].size, 2048 * 540);
        assert_eq!(planes[1].used, 2048 * 540 - 1000);

        // ExactSizeIterator so callers can pre-size without walking.
        assert_eq!(t.planes().len(), 2);
        assert_eq!(t.plane_at(1).unwrap().modifier, 1);
        assert!(t.plane_at(2).is_none());
    }

    #[test]
    fn plane_data_borrows_from_the_source_buffer() {
        // An inline plane's bytes must alias the CDR buffer, never a copy.
        let payload = [9u8, 8, 7, 6];
        let plane = TensorPlaneView {
            handle: -1,
            offset: 0,
            stride: 4,
            size: 4,
            used: 4,
            modifier: 0,
            handle_bytes: &[],
            data: &payload,
        };
        let mut s = CdrSizer::new();
        s.size_u32();
        s.size_u32();
        s.size_i32();
        s.size_u32();
        s.size_i32();
        s.size_u32();
        s.size_seq_8(0);
        s.size_u32();
        s.size_seq_8(0);
        s.size_u32();
        s.size_seq_4(0);
        s.size_u32();
        s.size_seq_4(0);
        s.size_string("");
        s.size_string("");
        s.size_string("");
        s.size_string("");
        s.size_string("");
        s.size_u32();
        size_plane_element(&mut s, 0, payload.len());
        let mut buf = vec![0u8; s.size()];
        let mut w = CdrWriter::new(&mut buf).unwrap();
        w.write_u32(0);
        w.write_u32(0);
        w.write_i32(-1);
        w.write_u32(0);
        w.write_i32(-2);
        w.write_u32(0);
        w.write_u32(0);
        w.write_u32(0);
        w.write_u32(0);
        w.write_string("");
        w.write_string("");
        w.write_string("");
        w.write_string("");
        w.write_string("");
        w.write_u32(1);
        write_plane_element(&mut w, &plane);
        w.finish().unwrap();

        let t = Tensor::<&[u8]>::from_cdr(&buf[..]).unwrap();
        let got = t.plane_at(0).unwrap();
        assert_eq!(got.data, &payload[..]);
        // Pointer identity proves the slice aliases the CDR buffer.
        let buf_range = buf.as_ptr_range();
        let data_ptr = got.data.as_ptr();
        assert!(
            buf_range.contains(&data_ptr),
            "plane data must alias the buffer"
        );
    }

    #[test]
    fn standalone_cdr_of_an_embedded_tensor_matches_a_native_encoding() {
        // Position independence: the tensor's own bytes do not depend on the
        // offset it was parsed at, so re-heading them yields the same message.
        let buf = encode_rich_tensor();
        let t = Tensor::<&[u8]>::from_cdr(&buf[..]).unwrap();
        assert_eq!(t.to_standalone_cdr(), buf);
    }

    #[test]
    fn tensor_scalars_are_readable() {
        let buf = encode_minimal_tensor(2, 4321, -1, 7, -2);
        let t = Tensor::<&[u8]>::from_cdr(&buf[..]).unwrap();
        assert_eq!(t.storage_kind(), 2);
        assert_eq!(t.pid(), 4321);
        assert_eq!(t.fence_fd(), -1);
        assert_eq!(t.dtype(), 7);
        assert_eq!(t.quant_axis(), -2);
    }

    #[test]
    fn tensor_scalar_offsets_are_constant_from_base() {
        // The 20-byte scalar prefix must be reachable by arithmetic alone.
        // If a variable-length field is ever moved into the prefix, the
        // second read here diverges from the first and this fails.
        // quant_axis is fixed at -2 (not e.g. -1) because encode_minimal_tensor
        // always writes zero quant_scales, and quant_axis == -1 requires
        // exactly one scale -- see quant_axis_minus_two_requires_empty_scales.
        let buf = encode_minimal_tensor(1, 2, 3, 4, -2);
        let t = Tensor::<&[u8]>::from_cdr(&buf[..]).unwrap();
        let b = t.base();
        assert_eq!(crate::cdr::rd_u32(&buf, b), t.storage_kind());
        assert_eq!(crate::cdr::rd_u32(&buf, b + 4), t.pid());
        assert_eq!(crate::cdr::rd_i32(&buf, b + 8), t.fence_fd());
        assert_eq!(crate::cdr::rd_u32(&buf, b + 12), t.dtype());
        assert_eq!(crate::cdr::rd_i32(&buf, b + 16), t.quant_axis());
    }

    #[test]
    fn from_cdr_at_rejects_misaligned_base() {
        let buf = encode_minimal_tensor(1, 0, -1, 0, -2);

        // Positive control: the aligned base parses.
        Tensor::<&[u8]>::from_cdr_at(&buf[..], crate::cdr::CDR_HEADER_SIZE).unwrap();

        // A base whose data-relative offset is not a multiple of 8 breaks the
        // position-independence invariant and must be refused, not tolerated.
        //
        // Assert the error VARIANT, not merely is_err(). Parsing at a
        // misaligned offset would also fail by reading garbage, so a bare
        // is_err() would pass even with the guard deleted. InvalidHeader is
        // the guard; BufferTooShort is what garbage parsing yields.
        for bad in [1usize, 2, 3, 4, 5, 6, 7] {
            match Tensor::<&[u8]>::from_cdr_at(&buf[..], crate::cdr::CDR_HEADER_SIZE + bad) {
                Err(CdrError::InvalidHeader) => {}
                other => panic!("base offset +{bad} must fail the alignment guard, got {other:?}"),
            }
        }
    }

    #[test]
    fn quant_axis_minus_two_requires_empty_scales() {
        // quant_axis == -2 means "no quantization"; scales must be empty.
        // encode_minimal_tensor writes zero scales, so -2 is valid and -1
        // (which requires exactly one scale) is not.
        assert!(Tensor::<&[u8]>::from_cdr(&encode_minimal_tensor(0, 0, -1, 0, -2)[..]).is_ok());
        assert!(Tensor::<&[u8]>::from_cdr(&encode_minimal_tensor(0, 0, -1, 0, -1)[..]).is_err());
    }

    #[test]
    fn quant_axis_below_minus_two_is_invalid() {
        assert!(Tensor::<&[u8]>::from_cdr(&encode_minimal_tensor(0, 0, -1, 0, -3)[..]).is_err());
    }

    #[test]
    fn builder_roundtrips_through_the_view() {
        let planes = [TensorPlaneView {
            handle: 3,
            offset: 0,
            stride: 640,
            size: 640 * 480,
            used: 640 * 480,
            modifier: 0,
            handle_bytes: &[],
            data: &[],
        }];
        let shape: [u64; 3] = [480, 640, 3];
        let strides: [i64; 3] = [640 * 3, 3, 1];

        let t = Tensor::builder()
            .storage_kind(2)
            .pid(1234)
            .fence_fd(-1)
            .dtype(1)
            .quant_axis(-2)
            .shape(&shape)
            .strides(&strides)
            .format("rgb8")
            .color_space("srgb")
            .planes(&planes)
            .build()
            .unwrap();

        assert_eq!(t.storage_kind(), 2);
        assert_eq!(t.pid(), 1234);
        assert_eq!(t.dtype(), 1);
        assert_eq!(t.quant_axis(), -2);
        assert_eq!(t.shape().collect::<Vec<_>>(), shape.to_vec());
        assert_eq!(t.strides().collect::<Vec<_>>(), strides.to_vec());
        assert_eq!(t.format(), "rgb8");
        assert_eq!(t.color_space(), "srgb");
        assert_eq!(t.num_planes(), 1);
        assert_eq!(t.plane_at(0).unwrap().handle, 3);
    }

    #[test]
    fn builder_defaults_produce_a_valid_empty_tensor() {
        let t = Tensor::builder().build().unwrap();
        assert_eq!(t.fence_fd(), -1);
        assert_eq!(t.quant_axis(), -2);
        assert_eq!(t.shape().len(), 0);
        assert_eq!(t.strides().len(), 0);
        assert!(t.quant_scales().is_empty());
        assert_eq!(t.format(), "");
        assert_eq!(t.num_planes(), 0);
    }

    #[test]
    fn builder_rejects_structural_violations() {
        let shape: [u64; 2] = [4, 8];

        // Positive control: the base shape alone builds. Every assertion below
        // adds exactly one violation to this, so a failure can only come from
        // the rule under test.
        Tensor::builder().shape(&shape).build().unwrap();

        // strides must be empty or match shape arity.
        let bad_strides: [i64; 1] = [1];
        assert!(Tensor::builder()
            .shape(&shape)
            .strides(&bad_strides)
            .build()
            .is_err());

        // quant_axis >= 0 requires one scale per element of that axis.
        let scales: [f32; 3] = [1.0, 2.0, 3.0];
        assert!(Tensor::builder()
            .shape(&shape)
            .quant_axis(1) // shape[1] == 8, but only 3 scales given
            .quant_scales(&scales)
            .build()
            .is_err());

        // quant_axis out of range for the shape.
        let one: [f32; 1] = [1.0];
        assert!(Tensor::builder()
            .shape(&shape)
            .quant_axis(2)
            .quant_scales(&one)
            .build()
            .is_err());

        // zero_points must match scales arity when present.
        let zeros: [i32; 2] = [0, 0];
        assert!(Tensor::builder()
            .shape(&shape)
            .quant_axis(-1)
            .quant_scales(&one)
            .quant_zero_points(&zeros)
            .build()
            .is_err());

        // Colorimetry without a format is meaningless.
        assert!(Tensor::builder().color_space("bt709").build().is_err());
    }

    #[test]
    fn builder_accepts_per_axis_quantization() {
        // Positive control for `quant_axis >= 0`: the negative cases in
        // builder_rejects_structural_violations never exercise the
        // matching-arity path, so this is the only place that proves the
        // per-axis branch (in both scan_tensor and TensorFields::validate)
        // actually accepts a well-formed tensor and round-trips its fields.
        let shape: [u64; 2] = [4, 8];
        let scales: [f32; 8] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let zeros: [i32; 8] = [0, 1, 2, 3, 4, 5, 6, 7];

        let t = Tensor::builder()
            .shape(&shape)
            .quant_axis(1) // shape[1] == 8, matches quant_scales.len()
            .quant_scales(&scales)
            .quant_zero_points(&zeros)
            .build()
            .unwrap();

        let bytes = t.to_standalone_cdr();
        let parsed = Tensor::<&[u8]>::from_cdr(&bytes[..]).unwrap();

        assert_eq!(parsed.quant_axis(), 1);
        assert_eq!(parsed.quant_scales(), &scales[..]);
        assert_eq!(parsed.quant_zero_points(), &zeros[..]);
    }

    #[test]
    fn builder_rejects_mixed_transport_modes() {
        let bytes = [1u8, 2, 3, 4];
        let planes = [
            TensorPlaneView {
                handle: 3,
                offset: 0,
                stride: 4,
                size: 4,
                used: 4,
                modifier: 0,
                handle_bytes: &[],
                data: &[],
            },
            TensorPlaneView {
                handle: -1,
                offset: 0,
                stride: 4,
                size: 4,
                used: 4,
                modifier: 0,
                handle_bytes: &[],
                data: &bytes,
            },
        ];
        assert!(Tensor::builder().planes(&planes).build().is_err());
    }

    #[test]
    fn encode_into_vec_reuses_the_buffer() {
        let mut buf = Vec::new();
        Tensor::builder()
            .dtype(1)
            .encode_into_vec(&mut buf)
            .unwrap();
        let first_len = buf.len();
        let cap = buf.capacity();

        Tensor::builder()
            .dtype(2)
            .encode_into_vec(&mut buf)
            .unwrap();
        assert_eq!(buf.len(), first_len);
        assert_eq!(buf.capacity(), cap, "re-encoding must not reallocate");
        assert_eq!(Tensor::<&[u8]>::from_cdr(&buf[..]).unwrap().dtype(), 2);
    }
}
