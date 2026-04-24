// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! EdgeFirst custom message types for perception pipelines.
//!
//! CdrFixed: `Date`
//!
//! Buffer-backed: `Mask` (`MaskView`), `DmaBuffer`, `LocalTime`,
//! `RadarCube`, `RadarInfo`, `Track`, `DetectBox` (`DetectBoxView`),
//! `Detect`, `Model`, `ModelInfo`

use crate::builtin_interfaces::{Duration, Time};
use crate::cdr::*;
use crate::std_msgs::Header;

// ── CdrFixed types ──────────────────────────────────────────────────

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Date {
    pub year: u16,
    pub month: u8,
    pub day: u8,
}

impl CdrFixed for Date {
    const CDR_SIZE: usize = 4; // u16(2) + u8(1) + u8(1)
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Date {
            year: cursor.read_u16()?,
            month: cursor.read_u8()?,
            day: cursor.read_u8()?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_u16(self.year);
        writer.write_u8(self.month);
        writer.write_u8(self.day);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_u16();
        sizer.size_u8();
        sizer.size_u8();
    }
}

// ── Constants ───────────────────────────────────────────────────────

pub mod radar_cube_dimension {
    pub const UNDEFINED: u8 = 0;
    pub const RANGE: u8 = 1;
    pub const DOPPLER: u8 = 2;
    pub const AZIMUTH: u8 = 3;
    pub const ELEVATION: u8 = 4;
    pub const RXCHANNEL: u8 = 5;
    pub const SEQUENCE: u8 = 6;
}

pub mod model_info {
    pub const RAW: u8 = 0;
    pub const INT8: u8 = 1;
    pub const UINT8: u8 = 2;
    pub const INT16: u8 = 3;
    pub const UINT16: u8 = 4;
    pub const FLOAT16: u8 = 5;
    pub const INT32: u8 = 6;
    pub const UINT32: u8 = 7;
    pub const FLOAT32: u8 = 8;
    pub const INT64: u8 = 9;
    pub const UINT64: u8 = 10;
    pub const FLOAT64: u8 = 11;
    pub const STRING: u8 = 12;
}

// ── Buffer-backed types ─────────────────────────────────────────────

// ── Mask<B> — edgefirst_msgs/msg/Mask ───────────────────────────────
//
// CDR layout:
//   4: height (u32), width (u32), length (u32)
//  16: encoding (string) → offsets[0]
//   ~: mask (byte seq) → offsets[1]
//   ~: boxed (bool)

pub struct Mask<B> {
    buf: B,
    offsets: [usize; 2],
}

impl<B: AsRef<[u8]>> Mask<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let mut c = CdrCursor::new(buf.as_ref())?;
        let _ = c.read_u32()?; // height
        let _ = c.read_u32()?; // width
        let _ = c.read_u32()?; // length
        let _ = c.read_string()?; // encoding
        let o0 = c.offset();
        let _ = c.read_bytes()?; // mask
        let o1 = c.offset();
        let _ = c.read_bool()?; // boxed
        Ok(Mask {
            offsets: [o0, o1],
            buf,
        })
    }

    #[inline]
    pub fn height(&self) -> u32 {
        rd_u32(self.buf.as_ref(), CDR_HEADER_SIZE)
    }

    #[inline]
    pub fn width(&self) -> u32 {
        rd_u32(self.buf.as_ref(), CDR_HEADER_SIZE + 4)
    }

    #[inline]
    pub fn length(&self) -> u32 {
        rd_u32(self.buf.as_ref(), CDR_HEADER_SIZE + 8)
    }

    #[inline]
    pub fn encoding(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 12).0
    }

    #[inline]
    pub fn mask_data(&self) -> &[u8] {
        rd_bytes(self.buf.as_ref(), self.offsets[0]).0
    }

    #[inline]
    pub fn boxed(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[1])
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }

    #[inline]
    pub fn cdr_size(&self) -> usize {
        self.buf.as_ref().len()
    }

    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl Mask<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use Mask::builder() for allocation-free buffer reuse; Mask::new will be removed in 4.0"
    )]
    pub fn new(
        height: u32,
        width: u32,
        length: u32,
        encoding: &str,
        mask: &[u8],
        boxed: bool,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        sizer.size_u32(); // height
        sizer.size_u32(); // width
        sizer.size_u32(); // length
        sizer.size_string(encoding);
        let o0 = sizer.offset();
        sizer.size_bytes(mask.len());
        let o1 = sizer.offset();
        sizer.size_bool();

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        w.write_u32(height);
        w.write_u32(width);
        w.write_u32(length);
        w.write_string(encoding);
        w.write_bytes(mask);
        w.write_bool(boxed);
        w.finish()?;

        Ok(Mask {
            offsets: [o0, o1],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `MaskBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> MaskBuilder<'a> {
        MaskBuilder::new()
    }
}

// ── MaskBuilder<'a> ─────────────────────────────────────────────────

/// Builder for `Mask<Vec<u8>>` with buffer-reuse finalizers.
///
/// `encoding` is `Cow<'a, str>` so string literals borrow and owned strings
/// move in; `mask` borrows from caller memory. All borrows must remain valid
/// until `build()`, `encode_into_vec()`, or `encode_into_slice()` is called.
pub struct MaskBuilder<'a> {
    height: u32,
    width: u32,
    length: u32,
    encoding: std::borrow::Cow<'a, str>,
    mask: &'a [u8],
    boxed: bool,
}

impl<'a> Default for MaskBuilder<'a> {
    fn default() -> Self {
        Self {
            height: 0,
            width: 0,
            length: 0,
            encoding: std::borrow::Cow::Borrowed(""),
            mask: &[],
            boxed: false,
        }
    }
}

impl<'a> MaskBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn height(&mut self, v: u32) -> &mut Self {
        self.height = v;
        self
    }
    pub fn width(&mut self, v: u32) -> &mut Self {
        self.width = v;
        self
    }
    pub fn length(&mut self, v: u32) -> &mut Self {
        self.length = v;
        self
    }
    pub fn encoding(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.encoding = s.into();
        self
    }
    pub fn mask(&mut self, m: &'a [u8]) -> &mut Self {
        self.mask = m;
        self
    }
    pub fn boxed(&mut self, v: bool) -> &mut Self {
        self.boxed = v;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        s.size_u32(); // height
        s.size_u32(); // width
        s.size_u32(); // length
        s.size_string(&self.encoding);
        s.size_bytes(self.mask.len());
        s.size_bool();
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        w.write_u32(self.height);
        w.write_u32(self.width);
        w.write_u32(self.length);
        w.write_string(&self.encoding);
        w.write_bytes(self.mask);
        w.write_bool(self.boxed);
        w.finish()
    }

    pub fn build(&self) -> Result<Mask<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        Mask::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

impl Mask<&'static [u8]> {
    /// Parse a standalone Mask CDR buffer and return a `'static`-lifetimed view.
    ///
    /// This is an FFI-oriented helper. Unlike the field-by-field copy path via
    /// `Mask::from_cdr(...)` followed by accessor methods, the returned
    /// `MaskView<'static>` borrows directly from the input buffer with the
    /// buffer's native `'static` lifetime — no `mem::transmute` is required at
    /// the FFI layer to widen method-returned references whose lifetimes are
    /// tied to a temporary `&self`.
    ///
    /// The parse is a single pass: `scan_mask_element` consumes the fields
    /// and produces the view directly, so there is no double-validation cost.
    pub(crate) fn from_cdr_as_view(buf: &'static [u8]) -> Result<MaskView<'static>, CdrError> {
        let mut c = CdrCursor::new(buf)?;
        scan_mask_element(&mut c)
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> Mask<B> {
    pub fn set_height(&mut self, h: u32) -> Result<(), CdrError> {
        wr_u32(self.buf.as_mut(), CDR_HEADER_SIZE, h)
    }

    pub fn set_width(&mut self, w: u32) -> Result<(), CdrError> {
        wr_u32(self.buf.as_mut(), CDR_HEADER_SIZE + 4, w)
    }

    pub fn set_length(&mut self, l: u32) -> Result<(), CdrError> {
        wr_u32(self.buf.as_mut(), CDR_HEADER_SIZE + 8, l)
    }

    pub fn set_boxed(&mut self, v: bool) -> Result<(), CdrError> {
        wr_u8(self.buf.as_mut(), self.offsets[1], v as u8)
    }
}

/// Zero-copy view of a Mask element within a CDR sequence.
#[derive(Copy, Clone, Debug)]
pub struct MaskView<'a> {
    pub height: u32,
    pub width: u32,
    pub length: u32,
    pub encoding: &'a str,
    pub mask: &'a [u8],
    pub boxed: bool,
}

pub(crate) fn scan_mask_element<'a>(c: &mut CdrCursor<'a>) -> Result<MaskView<'a>, CdrError> {
    let height = c.read_u32()?;
    let width = c.read_u32()?;
    let length = c.read_u32()?;
    let encoding = c.read_string()?;
    let mask = c.read_bytes()?;
    let boxed = c.read_bool()?;
    Ok(MaskView {
        height,
        width,
        length,
        encoding,
        mask,
        boxed,
    })
}

pub(crate) fn write_mask_element(w: &mut CdrWriter<'_>, m: &MaskView<'_>) {
    w.write_u32(m.height);
    w.write_u32(m.width);
    w.write_u32(m.length);
    w.write_string(m.encoding);
    w.write_bytes(m.mask);
    w.write_bool(m.boxed);
}

pub(crate) fn size_mask_element(s: &mut CdrSizer, encoding: &str, mask_len: usize) {
    s.size_u32();
    s.size_u32();
    s.size_u32();
    s.size_string(encoding);
    s.size_bytes(mask_len);
    s.size_bool();
}

// ── DmaBuffer<B> — edgefirst_msgs/msg/DmaBuffer ────────────────────
//
// CDR layout: Header → offsets[0], then:
//   pid(u32) + fd(i32) + width(u32) + height(u32)
//   + stride(u32) + fourcc(u32) + length(u32) = 28 bytes
//
// DEPRECATED since 3.1.0: use CameraFrame instead. Will be removed in 4.0.0.

#[deprecated(
    since = "3.1.0",
    note = "Use CameraFrame / CameraPlane for multi-plane support, colorimetry, \
            GPU fences, and off-device bridging. DmaBuffer will be removed in 4.0.0."
)]
pub struct DmaBuffer<B> {
    buf: B,
    offsets: [usize; 1],
}

// The DmaBuffer impls remain until 4.0.0; allow(deprecated) here so the
// crate's own use of the deprecated struct (fields, methods) compiles
// cleanly. User code still gets the deprecation warning.
#[allow(deprecated)]
impl<B: AsRef<[u8]>> DmaBuffer<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        for _ in 0..7 {
            c.read_u32()?;
        }
        Ok(DmaBuffer { offsets: [o0], buf })
    }

    /// Returns a `Header` view (re-parses CDR prefix; prefer `stamp()`/`frame_id()`).
    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }

    #[inline]
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }

    #[inline]
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }

    #[inline]
    pub fn pid(&self) -> u32 {
        let p = align(self.offsets[0], 4);
        rd_u32(self.buf.as_ref(), p)
    }

    #[inline]
    pub fn fd(&self) -> i32 {
        let p = align(self.offsets[0], 4) + 4;
        rd_i32(self.buf.as_ref(), p)
    }

    #[inline]
    pub fn width(&self) -> u32 {
        let p = align(self.offsets[0], 4) + 8;
        rd_u32(self.buf.as_ref(), p)
    }

    #[inline]
    pub fn height(&self) -> u32 {
        let p = align(self.offsets[0], 4) + 12;
        rd_u32(self.buf.as_ref(), p)
    }

    #[inline]
    pub fn stride(&self) -> u32 {
        let p = align(self.offsets[0], 4) + 16;
        rd_u32(self.buf.as_ref(), p)
    }

    #[inline]
    pub fn fourcc(&self) -> u32 {
        let p = align(self.offsets[0], 4) + 20;
        rd_u32(self.buf.as_ref(), p)
    }

    #[inline]
    pub fn length(&self) -> u32 {
        let p = align(self.offsets[0], 4) + 24;
        rd_u32(self.buf.as_ref(), p)
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }

    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

#[allow(deprecated)]
impl DmaBuffer<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        pid: u32,
        fd: i32,
        width: u32,
        height: u32,
        stride: u32,
        fourcc: u32,
        length: u32,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        for _ in 0..7 {
            sizer.size_u32();
        }

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_u32(pid);
        w.write_i32(fd);
        w.write_u32(width);
        w.write_u32(height);
        w.write_u32(stride);
        w.write_u32(fourcc);
        w.write_u32(length);
        w.finish()?;

        Ok(DmaBuffer { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── LocalTime<B> — edgefirst_msgs/msg/LocalTime ────────────────────
//
// CDR layout: Header → offsets[0], then:
//   Date(4) + pad(2) + Time(8) + timezone(i16) = fixed payload

pub struct LocalTime<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B: AsRef<[u8]>> LocalTime<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        Date::read_cdr(&mut c)?;
        Time::read_cdr(&mut c)?;
        c.read_i16()?; // timezone
        Ok(LocalTime { offsets: [o0], buf })
    }

    /// Returns a `Header` view (re-parses CDR prefix; prefer `stamp()`/`frame_id()`).
    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }

    #[inline]
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }

    #[inline]
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }

    pub fn date(&self) -> Date {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        Date::read_cdr(&mut c).expect("date field validated during from_cdr")
    }

    pub fn time(&self) -> Time {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        Date::read_cdr(&mut c).expect("date field validated during from_cdr");
        Time::read_cdr(&mut c).expect("time field validated during from_cdr")
    }

    pub fn timezone(&self) -> i16 {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        Date::read_cdr(&mut c).expect("date field validated during from_cdr");
        Time::read_cdr(&mut c).expect("time field validated during from_cdr");
        c.read_i16()
            .expect("timezone field validated during from_cdr")
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }

    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl LocalTime<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use LocalTime::builder() for allocation-free buffer reuse; LocalTime::new will be removed in 4.0"
    )]
    pub fn new(
        stamp: Time,
        frame_id: &str,
        date: Date,
        time: Time,
        timezone: i16,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        Date::size_cdr(&mut sizer);
        Time::size_cdr(&mut sizer);
        sizer.size_i16();

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        date.write_cdr(&mut w);
        time.write_cdr(&mut w);
        w.write_i16(timezone);
        w.finish()?;

        Ok(LocalTime { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `LocalTimeBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> LocalTimeBuilder<'a> {
        LocalTimeBuilder::new()
    }
}

// ── LocalTimeBuilder<'a> ────────────────────────────────────────────

/// Builder for `LocalTime<Vec<u8>>` with buffer-reuse finalizers.
pub struct LocalTimeBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    date: Date,
    time: Time,
    timezone: i16,
}

impl<'a> Default for LocalTimeBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            date: Date {
                year: 0,
                month: 0,
                day: 0,
            },
            time: Time { sec: 0, nanosec: 0 },
            timezone: 0,
        }
    }
}

impl<'a> LocalTimeBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn date(&mut self, d: Date) -> &mut Self {
        self.date = d;
        self
    }
    pub fn time(&mut self, t: Time) -> &mut Self {
        self.time = t;
        self
    }
    pub fn timezone(&mut self, v: i16) -> &mut Self {
        self.timezone = v;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        Date::size_cdr(&mut s);
        Time::size_cdr(&mut s);
        s.size_i16();
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        self.date.write_cdr(&mut w);
        self.time.write_cdr(&mut w);
        w.write_i16(self.timezone);
        w.finish()
    }

    pub fn build(&self) -> Result<LocalTime<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        LocalTime::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

// ── RadarCube<B> — edgefirst_msgs/msg/RadarCube ─────────────────────
//
// CDR layout: Header → offsets[0],
//   timestamp(u64), layout(Vec<u8>) → offsets[1],
//   shape(Vec<u16>) → offsets[2], scales(Vec<f32>) → offsets[3],
//   cube(Vec<i16>) → offsets[4], is_complex(bool)

pub struct RadarCube<B> {
    buf: B,
    offsets: [usize; 5],
}

impl<B: AsRef<[u8]>> RadarCube<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        c.read_u64()?; // timestamp
        let layout_count = c.read_u32()? as usize;
        c.skip(layout_count)?;
        let o1 = c.offset();
        let shape_count = c.read_u32()? as usize;
        c.skip_seq_2(shape_count)?;
        let o2 = c.offset();
        let scales_count = c.read_u32()? as usize;
        c.skip_seq_4(scales_count)?;
        let o3 = c.offset();
        let cube_count = c.read_u32()? as usize;
        c.skip_seq_2(cube_count)?;
        let o4 = c.offset();
        c.read_bool()?;
        Ok(RadarCube {
            offsets: [o0, o1, o2, o3, o4],
            buf,
        })
    }

    #[inline]
    /// Returns a `Header` view by re-parsing the CDR buffer prefix.
    /// Prefer `stamp()` / `frame_id()` for direct O(1) field access.
    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }
    #[inline]
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    #[inline]
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }

    pub fn timestamp(&self) -> u64 {
        let p = cdr_align(self.offsets[0], 8);
        rd_u64(self.buf.as_ref(), p)
    }

    pub fn layout(&self) -> &[u8] {
        let b = self.buf.as_ref();
        let p = align(cdr_align(self.offsets[0], 8) + 8, 4);
        let count = rd_u32(b, p) as usize;
        &b[p + 4..p + 4 + count]
    }

    pub fn shape(&self) -> &[u16] {
        let b = self.buf.as_ref();
        let p = align(self.offsets[1], 4);
        let count = rd_u32(b, p) as usize;
        rd_slice_u16(b, align(p + 4, 2), count)
    }

    pub fn scales(&self) -> &[f32] {
        let b = self.buf.as_ref();
        let p = align(self.offsets[2], 4);
        let count = rd_u32(b, p) as usize;
        rd_slice_f32(b, align(p + 4, 4), count)
    }

    /// Zero-copy view of the radar cube data as `&[i16]`.
    pub fn cube(&self) -> &[i16] {
        let b = self.buf.as_ref();
        let p = align(self.offsets[3], 4);
        let count = rd_u32(b, p) as usize;
        rd_slice_i16(b, align(p + 4, 2), count)
    }

    pub fn cube_raw(&self) -> &[u8] {
        let b = self.buf.as_ref();
        let p = align(self.offsets[3], 4);
        let count = rd_u32(b, p) as usize;
        &b[p + 4..p + 4 + count * 2]
    }

    pub fn cube_len(&self) -> u32 {
        rd_u32(self.buf.as_ref(), align(self.offsets[3], 4))
    }

    pub fn is_complex(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[4])
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl RadarCube<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use RadarCube::builder() for allocation-free buffer reuse; RadarCube::new will be removed in 4.0"
    )]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        stamp: Time,
        frame_id: &str,
        timestamp: u64,
        layout: &[u8],
        shape: &[u16],
        scales: &[f32],
        cube: &[i16],
        is_complex: bool,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_u64();
        sizer.size_bytes(layout.len());
        let o1 = sizer.offset();
        sizer.size_u32();
        sizer.size_seq_2(shape.len());
        let o2 = sizer.offset();
        sizer.size_u32();
        sizer.size_seq_4(scales.len());
        let o3 = sizer.offset();
        sizer.size_u32();
        sizer.size_seq_2(cube.len());
        let o4 = sizer.offset();
        sizer.size_bool();

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_u64(timestamp);
        w.write_bytes(layout);
        w.write_u32(shape.len() as u32);
        w.write_slice_u16(shape);
        w.write_u32(scales.len() as u32);
        w.write_slice_f32(scales);
        w.write_u32(cube.len() as u32);
        w.write_slice_i16(cube);
        w.write_bool(is_complex);
        w.finish()?;

        Ok(RadarCube {
            offsets: [o0, o1, o2, o3, o4],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `RadarCubeBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> RadarCubeBuilder<'a> {
        RadarCubeBuilder::new()
    }
}

// ── RadarCubeBuilder<'a> ────────────────────────────────────────────

/// Builder for `RadarCube<Vec<u8>>` with buffer-reuse finalizers.
///
/// The variable-length arrays (`layout`, `shape`, `scales`, `cube`) are
/// borrowed from caller memory. All borrows must remain valid until
/// `build()`, `encode_into_vec()`, or `encode_into_slice()` is called.
pub struct RadarCubeBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    timestamp: u64,
    layout: &'a [u8],
    shape: &'a [u16],
    scales: &'a [f32],
    cube: &'a [i16],
    is_complex: bool,
}

impl<'a> Default for RadarCubeBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            timestamp: 0,
            layout: &[],
            shape: &[],
            scales: &[],
            cube: &[],
            is_complex: false,
        }
    }
}

impl<'a> RadarCubeBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn timestamp(&mut self, v: u64) -> &mut Self {
        self.timestamp = v;
        self
    }
    pub fn layout(&mut self, v: &'a [u8]) -> &mut Self {
        self.layout = v;
        self
    }
    pub fn shape(&mut self, v: &'a [u16]) -> &mut Self {
        self.shape = v;
        self
    }
    pub fn scales(&mut self, v: &'a [f32]) -> &mut Self {
        self.scales = v;
        self
    }
    pub fn cube(&mut self, v: &'a [i16]) -> &mut Self {
        self.cube = v;
        self
    }
    pub fn is_complex(&mut self, v: bool) -> &mut Self {
        self.is_complex = v;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        s.size_u64();
        s.size_bytes(self.layout.len());
        s.size_u32();
        s.size_seq_2(self.shape.len());
        s.size_u32();
        s.size_seq_4(self.scales.len());
        s.size_u32();
        s.size_seq_2(self.cube.len());
        s.size_bool();
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_u64(self.timestamp);
        w.write_bytes(self.layout);
        w.write_u32(self.shape.len() as u32);
        w.write_slice_u16(self.shape);
        w.write_u32(self.scales.len() as u32);
        w.write_slice_f32(self.scales);
        w.write_u32(self.cube.len() as u32);
        w.write_slice_i16(self.cube);
        w.write_bool(self.is_complex);
        w.finish()
    }

    pub fn build(&self) -> Result<RadarCube<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        RadarCube::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

// ── RadarInfo<B> — edgefirst_msgs/msg/RadarInfo ─────────────────────
//
// CDR layout: Header → offsets[0],
//   center_frequency → offsets[1], frequency_sweep → offsets[2],
//   range_toggle → offsets[3], detection_sensitivity → offsets[4],
//   cube(bool)

pub struct RadarInfo<B> {
    buf: B,
    offsets: [usize; 5],
}

impl<B: AsRef<[u8]>> RadarInfo<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        let _ = c.read_string()?;
        let o1 = c.offset();
        let _ = c.read_string()?;
        let o2 = c.offset();
        let _ = c.read_string()?;
        let o3 = c.offset();
        let _ = c.read_string()?;
        let o4 = c.offset();
        c.read_bool()?;
        Ok(RadarInfo {
            offsets: [o0, o1, o2, o3, o4],
            buf,
        })
    }

    #[inline]
    /// Returns a `Header` view by re-parsing the CDR buffer prefix.
    /// Prefer `stamp()` / `frame_id()` for direct O(1) field access.
    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }
    #[inline]
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    #[inline]
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }
    #[inline]
    pub fn center_frequency(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[0]).0
    }
    #[inline]
    pub fn frequency_sweep(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[1]).0
    }
    #[inline]
    pub fn range_toggle(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[2]).0
    }
    #[inline]
    pub fn detection_sensitivity(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[3]).0
    }
    #[inline]
    pub fn cube(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[4])
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl RadarInfo<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use RadarInfo::builder() for allocation-free buffer reuse; RadarInfo::new will be removed in 4.0"
    )]
    pub fn new(
        stamp: Time,
        frame_id: &str,
        center_frequency: &str,
        frequency_sweep: &str,
        range_toggle: &str,
        detection_sensitivity: &str,
        cube: bool,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_string(center_frequency);
        let o1 = sizer.offset();
        sizer.size_string(frequency_sweep);
        let o2 = sizer.offset();
        sizer.size_string(range_toggle);
        let o3 = sizer.offset();
        sizer.size_string(detection_sensitivity);
        let o4 = sizer.offset();
        sizer.size_bool();

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_string(center_frequency);
        w.write_string(frequency_sweep);
        w.write_string(range_toggle);
        w.write_string(detection_sensitivity);
        w.write_bool(cube);
        w.finish()?;

        Ok(RadarInfo {
            offsets: [o0, o1, o2, o3, o4],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `RadarInfoBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> RadarInfoBuilder<'a> {
        RadarInfoBuilder::new()
    }
}

// ── RadarInfoBuilder<'a> ────────────────────────────────────────────

/// Builder for `RadarInfo<Vec<u8>>` with buffer-reuse finalizers.
pub struct RadarInfoBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    center_frequency: std::borrow::Cow<'a, str>,
    frequency_sweep: std::borrow::Cow<'a, str>,
    range_toggle: std::borrow::Cow<'a, str>,
    detection_sensitivity: std::borrow::Cow<'a, str>,
    cube: bool,
}

impl<'a> Default for RadarInfoBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            center_frequency: std::borrow::Cow::Borrowed(""),
            frequency_sweep: std::borrow::Cow::Borrowed(""),
            range_toggle: std::borrow::Cow::Borrowed(""),
            detection_sensitivity: std::borrow::Cow::Borrowed(""),
            cube: false,
        }
    }
}

impl<'a> RadarInfoBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn center_frequency(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.center_frequency = s.into();
        self
    }
    pub fn frequency_sweep(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frequency_sweep = s.into();
        self
    }
    pub fn range_toggle(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.range_toggle = s.into();
        self
    }
    pub fn detection_sensitivity(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.detection_sensitivity = s.into();
        self
    }
    pub fn cube(&mut self, v: bool) -> &mut Self {
        self.cube = v;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        s.size_string(&self.center_frequency);
        s.size_string(&self.frequency_sweep);
        s.size_string(&self.range_toggle);
        s.size_string(&self.detection_sensitivity);
        s.size_bool();
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_string(&self.center_frequency);
        w.write_string(&self.frequency_sweep);
        w.write_string(&self.range_toggle);
        w.write_string(&self.detection_sensitivity);
        w.write_bool(self.cube);
        w.finish()
    }

    pub fn build(&self) -> Result<RadarInfo<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        RadarInfo::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

// ── Track<B> — edgefirst_msgs/msg/Track ─────────────────────────────
//
// CDR layout: id(string) → offsets[0], lifetime(i32), created(Time)

pub struct Track<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B: AsRef<[u8]>> Track<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let mut c = CdrCursor::new(buf.as_ref())?;
        let _ = c.read_string()?;
        let o0 = c.offset();
        c.read_i32()?;
        Time::read_cdr(&mut c)?;
        Ok(Track { offsets: [o0], buf })
    }

    #[inline]
    pub fn id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE).0
    }

    pub fn lifetime(&self) -> i32 {
        rd_i32(self.buf.as_ref(), align(self.offsets[0], 4))
    }

    pub fn created(&self) -> Time {
        rd_time(self.buf.as_ref(), align(self.offsets[0], 4) + 4)
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl Track<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use Track::builder() for allocation-free buffer reuse; Track::new will be removed in 4.0"
    )]
    pub fn new(id: &str, lifetime: i32, created: Time) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        sizer.size_string(id);
        let o0 = sizer.offset();
        sizer.size_i32();
        Time::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        w.write_string(id);
        w.write_i32(lifetime);
        created.write_cdr(&mut w);
        w.finish()?;

        Ok(Track { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `TrackBuilder` with zero-valued defaults.
    ///
    /// Note: `Track` is a standalone message without a `Header` (no
    /// `stamp`/`frame_id` prefix) — the builder only exposes the three
    /// schema fields `id`, `lifetime`, and `created`.
    pub fn builder<'a>() -> TrackBuilder<'a> {
        TrackBuilder::new()
    }
}

// ── TrackBuilder<'a> ────────────────────────────────────────────────

/// Builder for `Track<Vec<u8>>` with buffer-reuse finalizers.
///
/// `Track` has no `Header` — it is a standalone sequence element-style
/// message whose CDR payload is just `(id, lifetime, created)`.
pub struct TrackBuilder<'a> {
    id: std::borrow::Cow<'a, str>,
    lifetime: i32,
    created: Time,
}

impl<'a> Default for TrackBuilder<'a> {
    fn default() -> Self {
        Self {
            id: std::borrow::Cow::Borrowed(""),
            lifetime: 0,
            created: Time { sec: 0, nanosec: 0 },
        }
    }
}

impl<'a> TrackBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.id = s.into();
        self
    }
    pub fn lifetime(&mut self, v: i32) -> &mut Self {
        self.lifetime = v;
        self
    }
    pub fn created(&mut self, t: Time) -> &mut Self {
        self.created = t;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        s.size_string(&self.id);
        s.size_i32();
        Time::size_cdr(&mut s);
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        w.write_string(&self.id);
        w.write_i32(self.lifetime);
        self.created.write_cdr(&mut w);
        w.finish()
    }

    pub fn build(&self) -> Result<Track<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        Track::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

// ── DetectBox<B> — edgefirst_msgs/msg/Box ───────────────────────────
//
// Named DetectBox to avoid conflict with std::boxed::Box.
// CDR layout: center_x(f32), center_y(f32), width(f32), height(f32),
//   label(string) → offsets[0], score(f32), distance(f32), speed(f32),
//   track.id(string) → offsets[1], track.lifetime(i32), track.created(Time)

pub struct DetectBox<B> {
    buf: B,
    offsets: [usize; 2],
}

/// Zero-copy view of a Box element within a CDR sequence.
#[derive(Copy, Clone, Debug)]
pub struct DetectBoxView<'a> {
    pub center_x: f32,
    pub center_y: f32,
    pub width: f32,
    pub height: f32,
    pub label: &'a str,
    pub score: f32,
    pub distance: f32,
    pub speed: f32,
    pub track_id: &'a str,
    pub track_lifetime: i32,
    pub track_created: Time,
}

pub(crate) fn scan_box_element<'a>(c: &mut CdrCursor<'a>) -> Result<DetectBoxView<'a>, CdrError> {
    let center_x = c.read_f32()?;
    let center_y = c.read_f32()?;
    let width = c.read_f32()?;
    let height = c.read_f32()?;
    let label = c.read_string()?;
    let score = c.read_f32()?;
    let distance = c.read_f32()?;
    let speed = c.read_f32()?;
    let track_id = c.read_string()?;
    let track_lifetime = c.read_i32()?;
    let track_created = Time::read_cdr(c)?;
    Ok(DetectBoxView {
        center_x,
        center_y,
        width,
        height,
        label,
        score,
        distance,
        speed,
        track_id,
        track_lifetime,
        track_created,
    })
}

pub(crate) fn write_box_element(w: &mut CdrWriter<'_>, b: &DetectBoxView<'_>) {
    w.write_f32(b.center_x);
    w.write_f32(b.center_y);
    w.write_f32(b.width);
    w.write_f32(b.height);
    w.write_string(b.label);
    w.write_f32(b.score);
    w.write_f32(b.distance);
    w.write_f32(b.speed);
    w.write_string(b.track_id);
    w.write_i32(b.track_lifetime);
    b.track_created.write_cdr(w);
}

pub(crate) fn size_box_element(s: &mut CdrSizer, label: &str, track_id: &str) {
    s.size_f32();
    s.size_f32();
    s.size_f32();
    s.size_f32(); // center_x/y, width, height
    s.size_string(label);
    s.size_f32();
    s.size_f32();
    s.size_f32(); // score, distance, speed
    s.size_string(track_id);
    s.size_i32();
    Time::size_cdr(s);
}

impl DetectBox<&'static [u8]> {
    /// Parse a standalone DetectBox CDR buffer and return a `'static`-lifetimed
    /// view.
    ///
    /// This is an FFI-oriented helper. Unlike the field-by-field copy path via
    /// `DetectBox::from_cdr(...)` followed by accessor methods, the returned
    /// `DetectBoxView<'static>` borrows directly from the input buffer with the
    /// buffer's native `'static` lifetime — no `mem::transmute` is required at
    /// the FFI layer to widen method-returned references whose lifetimes are
    /// tied to a temporary `&self`.
    ///
    /// The parse is a single pass: `scan_box_element` consumes the fields and
    /// produces the view directly, so there is no double-validation cost.
    pub(crate) fn from_cdr_as_view(buf: &'static [u8]) -> Result<DetectBoxView<'static>, CdrError> {
        let mut c = CdrCursor::new(buf)?;
        scan_box_element(&mut c)
    }
}

impl<B: AsRef<[u8]>> DetectBox<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let mut c = CdrCursor::new(buf.as_ref())?;
        c.read_f32()?;
        c.read_f32()?;
        c.read_f32()?;
        c.read_f32()?;
        let _ = c.read_string()?;
        let o0 = c.offset();
        c.read_f32()?;
        c.read_f32()?;
        c.read_f32()?;
        let _ = c.read_string()?;
        let o1 = c.offset();
        c.read_i32()?;
        Time::read_cdr(&mut c)?;
        Ok(DetectBox {
            offsets: [o0, o1],
            buf,
        })
    }

    #[inline]
    pub fn center_x(&self) -> f32 {
        rd_f32(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    #[inline]
    pub fn center_y(&self) -> f32 {
        rd_f32(self.buf.as_ref(), CDR_HEADER_SIZE + 4)
    }
    #[inline]
    pub fn width(&self) -> f32 {
        rd_f32(self.buf.as_ref(), CDR_HEADER_SIZE + 8)
    }
    #[inline]
    pub fn height(&self) -> f32 {
        rd_f32(self.buf.as_ref(), CDR_HEADER_SIZE + 12)
    }

    pub fn label(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 16).0
    }

    pub fn score(&self) -> f32 {
        rd_f32(self.buf.as_ref(), align(self.offsets[0], 4))
    }
    pub fn distance(&self) -> f32 {
        rd_f32(self.buf.as_ref(), align(self.offsets[0], 4) + 4)
    }
    pub fn speed(&self) -> f32 {
        rd_f32(self.buf.as_ref(), align(self.offsets[0], 4) + 8)
    }

    pub fn track_id(&self) -> &str {
        rd_string(self.buf.as_ref(), align(self.offsets[0], 4) + 12).0
    }

    pub fn track_lifetime(&self) -> i32 {
        rd_i32(self.buf.as_ref(), align(self.offsets[1], 4))
    }

    pub fn track_created(&self) -> Time {
        rd_time(self.buf.as_ref(), align(self.offsets[1], 4) + 4)
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl DetectBox<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use DetectBox::builder() for allocation-free buffer reuse; DetectBox::new will be removed in 4.0"
    )]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        center_x: f32,
        center_y: f32,
        width: f32,
        height: f32,
        label: &str,
        score: f32,
        distance: f32,
        speed: f32,
        track_id: &str,
        track_lifetime: i32,
        track_created: Time,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_string(label);
        let o0 = sizer.offset();
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_string(track_id);
        let o1 = sizer.offset();
        sizer.size_i32();
        Time::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        w.write_f32(center_x);
        w.write_f32(center_y);
        w.write_f32(width);
        w.write_f32(height);
        w.write_string(label);
        w.write_f32(score);
        w.write_f32(distance);
        w.write_f32(speed);
        w.write_string(track_id);
        w.write_i32(track_lifetime);
        track_created.write_cdr(&mut w);
        w.finish()?;

        Ok(DetectBox {
            offsets: [o0, o1],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `DetectBoxBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> DetectBoxBuilder<'a> {
        DetectBoxBuilder::new()
    }
}

// ── DetectBoxBuilder<'a> ────────────────────────────────────────────

/// Builder for `DetectBox<Vec<u8>>` with buffer-reuse finalizers.
///
/// Mirrors `DetectBoxView` — the same shape used inside sequences in
/// `Detect` and `Model`.
pub struct DetectBoxBuilder<'a> {
    center_x: f32,
    center_y: f32,
    width: f32,
    height: f32,
    label: std::borrow::Cow<'a, str>,
    score: f32,
    distance: f32,
    speed: f32,
    track_id: std::borrow::Cow<'a, str>,
    track_lifetime: i32,
    track_created: Time,
}

impl<'a> Default for DetectBoxBuilder<'a> {
    fn default() -> Self {
        Self {
            center_x: 0.0,
            center_y: 0.0,
            width: 0.0,
            height: 0.0,
            label: std::borrow::Cow::Borrowed(""),
            score: 0.0,
            distance: 0.0,
            speed: 0.0,
            track_id: std::borrow::Cow::Borrowed(""),
            track_lifetime: 0,
            track_created: Time { sec: 0, nanosec: 0 },
        }
    }
}

impl<'a> DetectBoxBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn center_x(&mut self, v: f32) -> &mut Self {
        self.center_x = v;
        self
    }
    pub fn center_y(&mut self, v: f32) -> &mut Self {
        self.center_y = v;
        self
    }
    pub fn width(&mut self, v: f32) -> &mut Self {
        self.width = v;
        self
    }
    pub fn height(&mut self, v: f32) -> &mut Self {
        self.height = v;
        self
    }
    pub fn label(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.label = s.into();
        self
    }
    pub fn score(&mut self, v: f32) -> &mut Self {
        self.score = v;
        self
    }
    pub fn distance(&mut self, v: f32) -> &mut Self {
        self.distance = v;
        self
    }
    pub fn speed(&mut self, v: f32) -> &mut Self {
        self.speed = v;
        self
    }
    pub fn track_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.track_id = s.into();
        self
    }
    pub fn track_lifetime(&mut self, v: i32) -> &mut Self {
        self.track_lifetime = v;
        self
    }
    pub fn track_created(&mut self, t: Time) -> &mut Self {
        self.track_created = t;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        size_box_element(&mut s, &self.label, &self.track_id);
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        let view = DetectBoxView {
            center_x: self.center_x,
            center_y: self.center_y,
            width: self.width,
            height: self.height,
            label: &self.label,
            score: self.score,
            distance: self.distance,
            speed: self.speed,
            track_id: &self.track_id,
            track_lifetime: self.track_lifetime,
            track_created: self.track_created,
        };
        write_box_element(&mut w, &view);
        w.finish()
    }

    pub fn build(&self) -> Result<DetectBox<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        DetectBox::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

// ── Detect<B> — edgefirst_msgs/msg/Detect ───────────────────────────
//
// CDR layout: Header → offsets[0],
//   input_timestamp(Time), model_time(Time), output_time(Time),
//   boxes(Vec<Box>) → offsets[1]

pub struct Detect<B> {
    buf: B,
    offsets: [usize; 2],
}

impl<B: AsRef<[u8]>> Detect<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        Time::read_cdr(&mut c)?; // input_timestamp
        Time::read_cdr(&mut c)?; // model_time
        Time::read_cdr(&mut c)?; // output_time
        let raw_count = c.read_u32()?;
        let count = c.check_seq_count(raw_count, 24)?;
        for _ in 0..count {
            scan_box_element(&mut c)?;
        }
        let o1 = c.offset();
        Ok(Detect {
            offsets: [o0, o1],
            buf,
        })
    }

    #[inline]
    /// Returns a `Header` view by re-parsing the CDR buffer prefix.
    /// Prefer `stamp()` / `frame_id()` for direct O(1) field access.
    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }
    #[inline]
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    #[inline]
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }

    pub fn input_timestamp(&self) -> Time {
        let p = align(self.offsets[0], 4);
        rd_time(self.buf.as_ref(), p)
    }

    pub fn model_time(&self) -> Time {
        rd_time(self.buf.as_ref(), align(self.offsets[0], 4) + 8)
    }

    pub fn output_time(&self) -> Time {
        rd_time(self.buf.as_ref(), align(self.offsets[0], 4) + 16)
    }

    pub fn boxes_len(&self) -> u32 {
        rd_u32(self.buf.as_ref(), align(self.offsets[0], 4) + 24)
    }

    pub fn boxes(&self) -> Vec<DetectBoxView<'_>> {
        let b = self.buf.as_ref();
        let p = align(self.offsets[0], 4) + 24;
        let count = rd_u32(b, p) as usize;
        let mut c = CdrCursor::resume(b, p + 4);
        (0..count)
            .map(|_| scan_box_element(&mut c).expect("box elements validated during from_cdr"))
            .collect()
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl Detect<&'static [u8]> {
    /// Parse a Detect message and simultaneously collect the box views
    /// encountered during validation, avoiding a second parse pass in the
    /// FFI layer.
    ///
    /// The views in the returned `Vec` naturally have `'static` lifetime
    /// because they borrow from the `&'static [u8]` buffer. No unsafe
    /// transmute is required.
    ///
    /// This is a crate-private helper used by the FFI layer to avoid the
    /// cost of a second walk in `inner.boxes()` after `from_cdr`.
    pub(crate) fn from_cdr_collect_boxes(
        buf: &'static [u8],
    ) -> Result<(Self, Vec<DetectBoxView<'static>>), CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf)?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf, o0);
        Time::read_cdr(&mut c)?; // input_timestamp
        Time::read_cdr(&mut c)?; // model_time
        Time::read_cdr(&mut c)?; // output_time
        let raw_count = c.read_u32()?;
        let count = c.check_seq_count(raw_count, 24)?;
        let mut box_views = Vec::with_capacity(count);
        for _ in 0..count {
            box_views.push(scan_box_element(&mut c)?);
        }
        let o1 = c.offset();
        Ok((
            Detect {
                offsets: [o0, o1],
                buf,
            },
            box_views,
        ))
    }
}

impl Detect<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use Detect::builder() for allocation-free buffer reuse; Detect::new will be removed in 4.0"
    )]
    pub fn new(
        stamp: Time,
        frame_id: &str,
        input_timestamp: Time,
        model_time: Time,
        output_time: Time,
        boxes: &[DetectBoxView<'_>],
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        Time::size_cdr(&mut sizer);
        Time::size_cdr(&mut sizer);
        Time::size_cdr(&mut sizer);
        sizer.size_u32();
        for b in boxes {
            size_box_element(&mut sizer, b.label, b.track_id);
        }
        let o1 = sizer.offset();

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        input_timestamp.write_cdr(&mut w);
        model_time.write_cdr(&mut w);
        output_time.write_cdr(&mut w);
        w.write_u32(boxes.len() as u32);
        for b in boxes {
            write_box_element(&mut w, b);
        }
        w.finish()?;

        Ok(Detect {
            offsets: [o0, o1],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `DetectBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> DetectBuilder<'a> {
        DetectBuilder::new()
    }
}

// ── DetectBuilder<'a> ───────────────────────────────────────────────

/// Builder for `Detect<Vec<u8>>` with buffer-reuse finalizers.
///
/// `boxes` is borrowed from a caller-owned slice for the lifetime of the
/// builder. Each `DetectBoxView` itself borrows `label` and `track_id`
/// from caller memory — all borrows must remain valid until `build()`,
/// `encode_into_vec()`, or `encode_into_slice()` is called.
pub struct DetectBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    input_timestamp: Time,
    model_time: Time,
    output_time: Time,
    boxes: &'a [DetectBoxView<'a>],
}

impl<'a> Default for DetectBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            input_timestamp: Time { sec: 0, nanosec: 0 },
            model_time: Time { sec: 0, nanosec: 0 },
            output_time: Time { sec: 0, nanosec: 0 },
            boxes: &[],
        }
    }
}

impl<'a> DetectBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn input_timestamp(&mut self, t: Time) -> &mut Self {
        self.input_timestamp = t;
        self
    }
    pub fn model_time(&mut self, t: Time) -> &mut Self {
        self.model_time = t;
        self
    }
    pub fn output_time(&mut self, t: Time) -> &mut Self {
        self.output_time = t;
        self
    }
    pub fn boxes(&mut self, b: &'a [DetectBoxView<'a>]) -> &mut Self {
        self.boxes = b;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        Time::size_cdr(&mut s);
        Time::size_cdr(&mut s);
        Time::size_cdr(&mut s);
        s.size_u32();
        for b in self.boxes {
            size_box_element(&mut s, b.label, b.track_id);
        }
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        self.input_timestamp.write_cdr(&mut w);
        self.model_time.write_cdr(&mut w);
        self.output_time.write_cdr(&mut w);
        w.write_u32(self.boxes.len() as u32);
        for b in self.boxes {
            write_box_element(&mut w, b);
        }
        w.finish()
    }

    pub fn build(&self) -> Result<Detect<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        Detect::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

// ── CameraFrame / CameraPlane — edgefirst_msgs/msg/CameraFrame ──────
//
// CameraFrame CDR layout:
//   Header → offsets[0], then
//     sequence(u64) + pid(u32) + width(u32) + height(u32)
//     + format(string) + color_space(string) + color_transfer(string)
//     + color_encoding(string) + color_range(string)
//     + fence_fd(i32)
//     + planes(seq<CameraPlane>) → offsets[1]
//
// CameraPlane element layout (variable-sized due to trailing data[]):
//   fd(i32) + offset(u32) + stride(u32) + size(u32) + used(u32) + data(seq<u8>)

/// Zero-copy view of a single CameraPlane element, borrowed from a CDR buffer.
///
/// `fd == -1` signals that the plane's bytes are inlined in `data`; any other
/// negative fd is invalid. When `fd >= 0`, `data` must be empty.
#[derive(Copy, Clone, Debug)]
pub struct CameraPlaneView<'a> {
    pub fd: i32,
    pub offset: u32,
    pub stride: u32,
    pub size: u32,
    pub used: u32,
    pub data: &'a [u8],
}

pub(crate) fn scan_plane_element<'a>(
    c: &mut CdrCursor<'a>,
) -> Result<CameraPlaneView<'a>, CdrError> {
    let fd = c.read_i32()?;
    let offset = c.read_u32()?;
    let stride = c.read_u32()?;
    let size = c.read_u32()?;
    let used = c.read_u32()?;
    let data = c.read_bytes()?;
    Ok(CameraPlaneView {
        fd,
        offset,
        stride,
        size,
        used,
        data,
    })
}

pub(crate) fn write_plane_element(w: &mut CdrWriter<'_>, p: &CameraPlaneView<'_>) {
    w.write_i32(p.fd);
    w.write_u32(p.offset);
    w.write_u32(p.stride);
    w.write_u32(p.size);
    w.write_u32(p.used);
    w.write_bytes(p.data);
}

pub(crate) fn size_plane_element(s: &mut CdrSizer, data_len: usize) {
    s.size_i32();
    s.size_u32();
    s.size_u32();
    s.size_u32();
    s.size_u32();
    s.size_bytes(data_len);
}

/// Validate a CameraPlane against the schema contract (see CameraPlane.msg).
///
/// Contract:
///   - `fd >= -1` (only `-1` is a valid negative value; other negatives invalid)
///   - `used <= size`
///   - `fd >= 0`  => `data` empty (bytes live in DMA-BUF, not inlined)
///   - `fd == -1` => `size as usize == data.len()` (inlined: size describes data)
pub(crate) fn validate_plane(
    fd: i32,
    size: u32,
    used: u32,
    data_len: usize,
) -> Result<(), CdrError> {
    if fd < -1
        || used > size
        || (fd >= 0 && data_len != 0)
        || (fd == -1 && size as usize != data_len)
    {
        return Err(CdrError::InvalidHeader);
    }
    Ok(())
}

/// Multi-plane video frame reference message.
///
/// Replaces the single-plane `DmaBuffer` with a schema that supports planar
/// formats (NV12, I420, planar RGB NCHW), hardware codec bitstreams (H.264
/// with `used` < `size`), GPU fence synchronization, and off-device bridging
/// via inlined per-plane bytes.
///
/// # Example
///
/// ```
/// use edgefirst_schemas::edgefirst_msgs::{CameraFrame, CameraPlaneView};
/// use edgefirst_schemas::builtin_interfaces::Time;
///
/// let y = CameraPlaneView {
///     fd: 42, offset: 0, stride: 1920,
///     size: 2_073_600, used: 2_073_600, data: &[],
/// };
/// let uv = CameraPlaneView {
///     fd: 42, offset: 2_073_600, stride: 1920,
///     size: 1_036_800, used: 1_036_800, data: &[],
/// };
/// let cf = CameraFrame::new(
///     Time::new(1, 0), "cam0",
///     /*seq*/ 1, /*pid*/ 1234, /*w*/ 1920, /*h*/ 1080,
///     "NV12", "bt709", "bt709", "bt709", "limited",
///     /*fence_fd*/ -1, &[y, uv],
/// ).unwrap();
/// let view = CameraFrame::<&[u8]>::from_cdr(cf.as_cdr()).unwrap();
/// assert_eq!(view.format(), "NV12");
/// assert_eq!(view.planes().len(), 2);
/// ```
pub struct CameraFrame<B> {
    buf: B,
    // [0]: after Header (start of `seq`).
    // [1]: position of the `planes` sequence-count u32 prefix (the field
    // immediately after fence_fd). Caching this avoids rescanning the five
    // variable-length colorimetry strings on every `planes()`/`num_planes()`
    // call — important for high-frame-rate consumers.
    offsets: [usize; 2],
}

impl<B: AsRef<[u8]>> CameraFrame<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        c.read_u64()?; // seq
        c.read_u32()?; // pid
        let width = c.read_u32()?;
        let height = c.read_u32()?;
        c.read_string()?; // format
        c.read_string()?; // color_space
        c.read_string()?; // color_transfer
        c.read_string()?; // color_encoding
        c.read_string()?; // color_range
        c.read_i32()?; // fence_fd
        let planes_pos = c.offset();
        let raw_count = c.read_u32()?;
        // min plane size: 5×u32 + 4-byte data seq count = 24 bytes
        let count = c.check_seq_count(raw_count, 24)?;
        for _ in 0..count {
            let plane = scan_plane_element(&mut c)?;
            validate_plane(plane.fd, plane.size, plane.used, plane.data.len())?;
        }

        if width == 0 || height == 0 {
            return Err(CdrError::InvalidHeader);
        }

        Ok(CameraFrame {
            offsets: [o0, planes_pos],
            buf,
        })
    }

    #[inline]
    /// Returns a `Header` view by re-parsing the CDR buffer prefix.
    /// Prefer `stamp()` / `frame_id()` for direct O(1) field access.
    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }
    #[inline]
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    #[inline]
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }

    #[inline]
    pub fn seq(&self) -> u64 {
        // u64 needs 8-byte alignment relative to CDR data start.
        rd_u64(self.buf.as_ref(), cdr_align(self.offsets[0], 8))
    }
    #[inline]
    pub fn pid(&self) -> u32 {
        rd_u32(self.buf.as_ref(), cdr_align(self.offsets[0], 8) + 8)
    }
    #[inline]
    pub fn width(&self) -> u32 {
        rd_u32(self.buf.as_ref(), cdr_align(self.offsets[0], 8) + 12)
    }
    #[inline]
    pub fn height(&self) -> u32 {
        rd_u32(self.buf.as_ref(), cdr_align(self.offsets[0], 8) + 16)
    }

    fn strings_start(&self) -> usize {
        // Position of `format` string length prefix.
        cdr_align(self.offsets[0], 8) + 20
    }

    /// Walk format + 4 color strings, returning each string and the fence_fd
    /// that follows. String accessors unavoidably re-walk preceding strings
    /// because CDR string lengths are variable; plane access uses the cached
    /// `offsets[1]` and does not hit this path.
    fn scan_strings_and_fence(&self) -> (&str, &str, &str, &str, &str, i32) {
        let b = self.buf.as_ref();
        let (format, p1) = rd_string(b, self.strings_start());
        let (cs, p2) = rd_string(b, p1);
        let (ct, p3) = rd_string(b, p2);
        let (ce, p4) = rd_string(b, p3);
        let (cr, p5) = rd_string(b, p4);
        let fence_fd = rd_i32(b, align(p5, 4));
        (format, cs, ct, ce, cr, fence_fd)
    }

    #[inline]
    pub fn format(&self) -> &str {
        self.scan_strings_and_fence().0
    }
    #[inline]
    pub fn color_space(&self) -> &str {
        self.scan_strings_and_fence().1
    }
    #[inline]
    pub fn color_transfer(&self) -> &str {
        self.scan_strings_and_fence().2
    }
    #[inline]
    pub fn color_encoding(&self) -> &str {
        self.scan_strings_and_fence().3
    }
    #[inline]
    pub fn color_range(&self) -> &str {
        self.scan_strings_and_fence().4
    }
    #[inline]
    pub fn fence_fd(&self) -> i32 {
        self.scan_strings_and_fence().5
    }

    /// Number of planes in the sequence. O(1) via cached `offsets[1]`.
    #[inline]
    pub fn num_planes(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[1])
    }

    /// Collect all plane views by walking the CDR sequence. O(n_planes) via
    /// cached `offsets[1]` — does not rescan the colorimetry strings.
    pub fn planes(&self) -> Vec<CameraPlaneView<'_>> {
        let b = self.buf.as_ref();
        let count = rd_u32(b, self.offsets[1]) as usize;
        let mut c = CdrCursor::resume(b, self.offsets[1] + 4);
        (0..count)
            .map(|_| scan_plane_element(&mut c).expect("planes validated during from_cdr"))
            .collect()
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl CameraFrame<&'static [u8]> {
    /// Parse and simultaneously collect plane views for the FFI layer,
    /// avoiding a second walk after `from_cdr`. Mirrors `Detect::from_cdr_collect_boxes`.
    pub(crate) fn from_cdr_collect_planes(
        buf: &'static [u8],
    ) -> Result<(Self, Vec<CameraPlaneView<'static>>), CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf)?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf, o0);
        c.read_u64()?;
        c.read_u32()?;
        let width = c.read_u32()?;
        let height = c.read_u32()?;
        c.read_string()?;
        c.read_string()?;
        c.read_string()?;
        c.read_string()?;
        c.read_string()?;
        c.read_i32()?;
        let planes_pos = c.offset();
        let raw_count = c.read_u32()?;
        let count = c.check_seq_count(raw_count, 24)?;
        let mut planes = Vec::with_capacity(count);
        for _ in 0..count {
            let plane = scan_plane_element(&mut c)?;
            validate_plane(plane.fd, plane.size, plane.used, plane.data.len())?;
            planes.push(plane);
        }

        if width == 0 || height == 0 {
            return Err(CdrError::InvalidHeader);
        }

        Ok((
            CameraFrame {
                offsets: [o0, planes_pos],
                buf,
            },
            planes,
        ))
    }
}

impl CameraFrame<Vec<u8>> {
    /// Build a new CameraFrame, serializing its fields into a fresh CDR buffer.
    ///
    /// Enforces the schema contracts:
    /// - `width > 0` and `height > 0`
    /// - `plane.used <= plane.size`
    /// - `plane.fd >= -1` (only -1 is a valid negative sentinel)
    /// - when `plane.fd >= 0`, `plane.data` must be empty
    /// - when `plane.fd == -1` (inlined), `plane.size as usize == plane.data.len()`
    #[deprecated(
        since = "3.2.0",
        note = "use CameraFrame::builder() for allocation-free buffer reuse; CameraFrame::new will be removed in 4.0"
    )]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        stamp: Time,
        frame_id: &str,
        seq: u64,
        pid: u32,
        width: u32,
        height: u32,
        format: &str,
        color_space: &str,
        color_transfer: &str,
        color_encoding: &str,
        color_range: &str,
        fence_fd: i32,
        planes: &[CameraPlaneView<'_>],
    ) -> Result<Self, CdrError> {
        if width == 0 || height == 0 {
            return Err(CdrError::InvalidHeader);
        }
        for p in planes {
            validate_plane(p.fd, p.size, p.used, p.data.len())?;
        }

        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_u64();
        sizer.size_u32();
        sizer.size_u32();
        sizer.size_u32();
        sizer.size_string(format);
        sizer.size_string(color_space);
        sizer.size_string(color_transfer);
        sizer.size_string(color_encoding);
        sizer.size_string(color_range);
        sizer.size_i32();
        let planes_pos = sizer.offset();
        sizer.size_u32();
        for p in planes {
            size_plane_element(&mut sizer, p.data.len());
        }

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_u64(seq);
        w.write_u32(pid);
        w.write_u32(width);
        w.write_u32(height);
        w.write_string(format);
        w.write_string(color_space);
        w.write_string(color_transfer);
        w.write_string(color_encoding);
        w.write_string(color_range);
        w.write_i32(fence_fd);
        w.write_u32(planes.len() as u32);
        for p in planes {
            write_plane_element(&mut w, p);
        }
        w.finish()?;

        Ok(CameraFrame {
            offsets: [o0, planes_pos],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `CameraFrameBuilder` with zero-valued defaults and
    /// `fence_fd = -1` (the "no fence" sentinel).
    ///
    /// Generic in `'a` so the compiler infers it from subsequent
    /// `.planes(...)` borrows rather than forcing `'static`.
    pub fn builder<'a>() -> CameraFrameBuilder<'a> {
        CameraFrameBuilder::new()
    }
}

// ── CameraFrameBuilder<'a> ──────────────────────────────────────────

/// Builder for `CameraFrame<Vec<u8>>` with buffer-reuse finalizers.
///
/// `planes` is borrowed from a caller-owned slice for the lifetime of the
/// builder. Each `CameraPlaneView` in that slice itself borrows its `data`
/// from caller memory — all borrows must remain valid until `build()`,
/// `encode_into_vec()`, or `encode_into_slice()` is called.
pub struct CameraFrameBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    seq: u64,
    pid: u32,
    width: u32,
    height: u32,
    format: std::borrow::Cow<'a, str>,
    color_space: std::borrow::Cow<'a, str>,
    color_transfer: std::borrow::Cow<'a, str>,
    color_encoding: std::borrow::Cow<'a, str>,
    color_range: std::borrow::Cow<'a, str>,
    fence_fd: i32,
    planes: &'a [CameraPlaneView<'a>],
}

impl<'a> Default for CameraFrameBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            seq: 0,
            pid: 0,
            width: 0,
            height: 0,
            format: std::borrow::Cow::Borrowed(""),
            color_space: std::borrow::Cow::Borrowed(""),
            color_transfer: std::borrow::Cow::Borrowed(""),
            color_encoding: std::borrow::Cow::Borrowed(""),
            color_range: std::borrow::Cow::Borrowed(""),
            fence_fd: -1,
            planes: &[],
        }
    }
}

impl<'a> CameraFrameBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn seq(&mut self, v: u64) -> &mut Self {
        self.seq = v;
        self
    }
    pub fn pid(&mut self, v: u32) -> &mut Self {
        self.pid = v;
        self
    }
    pub fn width(&mut self, v: u32) -> &mut Self {
        self.width = v;
        self
    }
    pub fn height(&mut self, v: u32) -> &mut Self {
        self.height = v;
        self
    }
    pub fn format(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.format = s.into();
        self
    }
    pub fn color_space(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.color_space = s.into();
        self
    }
    pub fn color_transfer(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.color_transfer = s.into();
        self
    }
    pub fn color_encoding(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.color_encoding = s.into();
        self
    }
    pub fn color_range(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.color_range = s.into();
        self
    }
    pub fn fence_fd(&mut self, v: i32) -> &mut Self {
        self.fence_fd = v;
        self
    }
    pub fn planes(&mut self, p: &'a [CameraPlaneView<'a>]) -> &mut Self {
        self.planes = p;
        self
    }

    fn validate(&self) -> Result<(), CdrError> {
        if self.width == 0 || self.height == 0 {
            return Err(CdrError::InvalidHeader);
        }
        for p in self.planes {
            validate_plane(p.fd, p.size, p.used, p.data.len())?;
        }
        Ok(())
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        s.size_u64(); // seq
        s.size_u32(); // pid
        s.size_u32(); // width
        s.size_u32(); // height
        s.size_string(&self.format);
        s.size_string(&self.color_space);
        s.size_string(&self.color_transfer);
        s.size_string(&self.color_encoding);
        s.size_string(&self.color_range);
        s.size_i32(); // fence_fd
        s.size_u32(); // planes count
        for p in self.planes {
            size_plane_element(&mut s, p.data.len());
        }
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_u64(self.seq);
        w.write_u32(self.pid);
        w.write_u32(self.width);
        w.write_u32(self.height);
        w.write_string(&self.format);
        w.write_string(&self.color_space);
        w.write_string(&self.color_transfer);
        w.write_string(&self.color_encoding);
        w.write_string(&self.color_range);
        w.write_i32(self.fence_fd);
        w.write_u32(self.planes.len() as u32);
        for p in self.planes {
            write_plane_element(&mut w, p);
        }
        w.finish()
    }

    pub fn build(&self) -> Result<CameraFrame<Vec<u8>>, CdrError> {
        self.validate()?;
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        CameraFrame::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        self.validate()?;
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        self.validate()?;
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

// ── Model<B> — edgefirst_msgs/msg/Model ─────────────────────────────
//
// CDR layout: Header → offsets[0],
//   input_time(Duration), model_time(Duration),
//   output_time(Duration), decode_time(Duration),
//   boxes(Vec<Box>) → offsets[1], masks(Vec<Mask>) → offsets[2]

pub struct Model<B> {
    buf: B,
    offsets: [usize; 3],
}

impl<B: AsRef<[u8]>> Model<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        Duration::read_cdr(&mut c)?;
        Duration::read_cdr(&mut c)?;
        Duration::read_cdr(&mut c)?;
        Duration::read_cdr(&mut c)?;
        let raw_boxes = c.read_u32()?;
        let boxes_count = c.check_seq_count(raw_boxes, 24)?;
        for _ in 0..boxes_count {
            scan_box_element(&mut c)?;
        }
        let o1 = c.offset();
        let raw_masks = c.read_u32()?;
        let masks_count = c.check_seq_count(raw_masks, 13)?;
        for _ in 0..masks_count {
            scan_mask_element(&mut c)?;
        }
        let o2 = c.offset();
        Ok(Model {
            offsets: [o0, o1, o2],
            buf,
        })
    }

    #[inline]
    /// Returns a `Header` view by re-parsing the CDR buffer prefix.
    /// Prefer `stamp()` / `frame_id()` for direct O(1) field access.
    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }
    #[inline]
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    #[inline]
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }

    pub fn input_time(&self) -> Duration {
        rd_duration(self.buf.as_ref(), align(self.offsets[0], 4))
    }

    pub fn model_time(&self) -> Duration {
        rd_duration(self.buf.as_ref(), align(self.offsets[0], 4) + 8)
    }

    pub fn output_time(&self) -> Duration {
        rd_duration(self.buf.as_ref(), align(self.offsets[0], 4) + 16)
    }

    pub fn decode_time(&self) -> Duration {
        rd_duration(self.buf.as_ref(), align(self.offsets[0], 4) + 24)
    }

    pub fn boxes_len(&self) -> u32 {
        rd_u32(self.buf.as_ref(), align(self.offsets[0], 4) + 32)
    }

    pub fn boxes(&self) -> Vec<DetectBoxView<'_>> {
        let b = self.buf.as_ref();
        let p = align(self.offsets[0], 4) + 32;
        let count = rd_u32(b, p) as usize;
        let mut c = CdrCursor::resume(b, p + 4);
        (0..count)
            .map(|_| scan_box_element(&mut c).expect("box elements validated during from_cdr"))
            .collect()
    }

    pub fn masks_len(&self) -> u32 {
        rd_u32(self.buf.as_ref(), align(self.offsets[1], 4))
    }

    pub fn masks(&self) -> Vec<MaskView<'_>> {
        let b = self.buf.as_ref();
        let p = align(self.offsets[1], 4);
        let count = rd_u32(b, p) as usize;
        let mut c = CdrCursor::resume(b, p + 4);
        (0..count)
            .map(|_| scan_mask_element(&mut c).expect("mask elements validated during from_cdr"))
            .collect()
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl Model<&'static [u8]> {
    /// Parse a Model message and simultaneously collect the box and mask views
    /// encountered during validation, avoiding a second parse pass in the
    /// FFI layer.
    ///
    /// The views in the returned `Vec`s naturally have `'static` lifetime
    /// because they borrow from the `&'static [u8]` buffer. No unsafe
    /// transmute is required.
    ///
    /// This is a crate-private helper used by the FFI layer to avoid the
    /// cost of a second walk in `inner.boxes()` / `inner.masks()` after
    /// `from_cdr`.
    pub(crate) fn from_cdr_collect_children(
        buf: &'static [u8],
    ) -> Result<(Self, Vec<DetectBoxView<'static>>, Vec<MaskView<'static>>), CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf)?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf, o0);
        Duration::read_cdr(&mut c)?;
        Duration::read_cdr(&mut c)?;
        Duration::read_cdr(&mut c)?;
        Duration::read_cdr(&mut c)?;
        let raw_boxes = c.read_u32()?;
        let boxes_count = c.check_seq_count(raw_boxes, 24)?;
        let mut box_views = Vec::with_capacity(boxes_count);
        for _ in 0..boxes_count {
            box_views.push(scan_box_element(&mut c)?);
        }
        let o1 = c.offset();
        let raw_masks = c.read_u32()?;
        let masks_count = c.check_seq_count(raw_masks, 13)?;
        let mut mask_views = Vec::with_capacity(masks_count);
        for _ in 0..masks_count {
            mask_views.push(scan_mask_element(&mut c)?);
        }
        let o2 = c.offset();
        Ok((
            Model {
                offsets: [o0, o1, o2],
                buf,
            },
            box_views,
            mask_views,
        ))
    }
}

impl Model<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use Model::builder() for allocation-free buffer reuse; Model::new will be removed in 4.0"
    )]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        stamp: Time,
        frame_id: &str,
        input_time: Duration,
        model_time: Duration,
        output_time: Duration,
        decode_time: Duration,
        boxes: &[DetectBoxView<'_>],
        masks: &[MaskView<'_>],
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        Duration::size_cdr(&mut sizer);
        Duration::size_cdr(&mut sizer);
        Duration::size_cdr(&mut sizer);
        Duration::size_cdr(&mut sizer);
        sizer.size_u32();
        for b in boxes {
            size_box_element(&mut sizer, b.label, b.track_id);
        }
        let o1 = sizer.offset();
        sizer.size_u32();
        for m in masks {
            size_mask_element(&mut sizer, m.encoding, m.mask.len());
        }
        let o2 = sizer.offset();

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        input_time.write_cdr(&mut w);
        model_time.write_cdr(&mut w);
        output_time.write_cdr(&mut w);
        decode_time.write_cdr(&mut w);
        w.write_u32(boxes.len() as u32);
        for b in boxes {
            write_box_element(&mut w, b);
        }
        w.write_u32(masks.len() as u32);
        for m in masks {
            write_mask_element(&mut w, m);
        }
        w.finish()?;

        Ok(Model {
            offsets: [o0, o1, o2],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `ModelBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> ModelBuilder<'a> {
        ModelBuilder::new()
    }
}

// ── ModelBuilder<'a> ────────────────────────────────────────────────

/// Builder for `Model<Vec<u8>>` with buffer-reuse finalizers.
///
/// `boxes` and `masks` are borrowed from caller-owned slices. Each view
/// inside those slices itself borrows strings/byte-data from caller
/// memory — all borrows must remain valid until `build()`,
/// `encode_into_vec()`, or `encode_into_slice()` is called.
pub struct ModelBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    input_time: Duration,
    model_time: Duration,
    output_time: Duration,
    decode_time: Duration,
    boxes: &'a [DetectBoxView<'a>],
    masks: &'a [MaskView<'a>],
}

impl<'a> Default for ModelBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            input_time: Duration { sec: 0, nanosec: 0 },
            model_time: Duration { sec: 0, nanosec: 0 },
            output_time: Duration { sec: 0, nanosec: 0 },
            decode_time: Duration { sec: 0, nanosec: 0 },
            boxes: &[],
            masks: &[],
        }
    }
}

impl<'a> ModelBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn input_time(&mut self, d: Duration) -> &mut Self {
        self.input_time = d;
        self
    }
    pub fn model_time(&mut self, d: Duration) -> &mut Self {
        self.model_time = d;
        self
    }
    pub fn output_time(&mut self, d: Duration) -> &mut Self {
        self.output_time = d;
        self
    }
    pub fn decode_time(&mut self, d: Duration) -> &mut Self {
        self.decode_time = d;
        self
    }
    pub fn boxes(&mut self, b: &'a [DetectBoxView<'a>]) -> &mut Self {
        self.boxes = b;
        self
    }
    pub fn masks(&mut self, m: &'a [MaskView<'a>]) -> &mut Self {
        self.masks = m;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        Duration::size_cdr(&mut s);
        Duration::size_cdr(&mut s);
        Duration::size_cdr(&mut s);
        Duration::size_cdr(&mut s);
        s.size_u32();
        for b in self.boxes {
            size_box_element(&mut s, b.label, b.track_id);
        }
        s.size_u32();
        for m in self.masks {
            size_mask_element(&mut s, m.encoding, m.mask.len());
        }
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        self.input_time.write_cdr(&mut w);
        self.model_time.write_cdr(&mut w);
        self.output_time.write_cdr(&mut w);
        self.decode_time.write_cdr(&mut w);
        w.write_u32(self.boxes.len() as u32);
        for b in self.boxes {
            write_box_element(&mut w, b);
        }
        w.write_u32(self.masks.len() as u32);
        for m in self.masks {
            write_mask_element(&mut w, m);
        }
        w.finish()
    }

    pub fn build(&self) -> Result<Model<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        Model::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

// ── ModelInfo<B> — edgefirst_msgs/msg/ModelInfo ─────────────────────
//
// CDR layout: Header → offsets[0],
//   input_shape(Vec<u32>) → offsets[1], input_type(u8),
//   output_shape(Vec<u32>) → offsets[2], output_type(u8),
//   labels(Vec<String>) → offsets[3],
//   model_type(string) → offsets[4], model_format(string) → offsets[5],
//   model_name(string) → offsets[6]

pub struct ModelInfo<B> {
    buf: B,
    offsets: [usize; 6],
}

impl<B: AsRef<[u8]>> ModelInfo<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        let is_count = c.read_u32()? as usize;
        c.skip_seq_4(is_count)?;
        let o1 = c.offset();
        c.read_u8()?; // input_type
        let os_count = c.read_u32()? as usize;
        c.skip_seq_4(os_count)?;
        let o2 = c.offset();
        c.read_u8()?; // output_type
        let raw_lab = c.read_u32()?;
        let lab_count = c.check_seq_count(raw_lab, 5)?;
        for _ in 0..lab_count {
            c.read_string()?;
        }
        let o3 = c.offset();
        let _ = c.read_string()?;
        let o4 = c.offset();
        let _ = c.read_string()?;
        let o5 = c.offset();
        let _ = c.read_string()?;
        Ok(ModelInfo {
            offsets: [o0, o1, o2, o3, o4, o5],
            buf,
        })
    }

    #[inline]
    /// Returns a `Header` view by re-parsing the CDR buffer prefix.
    /// Prefer `stamp()` / `frame_id()` for direct O(1) field access.
    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }
    #[inline]
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    #[inline]
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }

    pub fn input_shape(&self) -> &[u32] {
        let b = self.buf.as_ref();
        let p = align(self.offsets[0], 4);
        let count = rd_u32(b, p) as usize;
        rd_slice_u32(b, p + 4, count)
    }

    pub fn input_type(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[1])
    }

    pub fn output_shape(&self) -> &[u32] {
        let b = self.buf.as_ref();
        let p = align(self.offsets[1] + 1, 4);
        let count = rd_u32(b, p) as usize;
        rd_slice_u32(b, p + 4, count)
    }

    pub fn output_type(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[2])
    }

    pub fn labels(&self) -> Vec<&str> {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[2] + 1);
        let count = c.read_u32().expect("label data validated during from_cdr") as usize;
        (0..count)
            .map(|_| {
                c.read_string()
                    .expect("label data validated during from_cdr")
            })
            .collect()
    }

    pub fn labels_len(&self) -> u32 {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[2] + 1);
        c.read_u32().expect("label data validated during from_cdr")
    }

    #[inline]
    pub fn model_type(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[3]).0
    }
    #[inline]
    pub fn model_format(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[4]).0
    }
    #[inline]
    pub fn model_name(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[5]).0
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl ModelInfo<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use ModelInfo::builder() for allocation-free buffer reuse; ModelInfo::new will be removed in 4.0"
    )]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        stamp: Time,
        frame_id: &str,
        input_shape: &[u32],
        input_type: u8,
        output_shape: &[u32],
        output_type: u8,
        labels: &[&str],
        model_type: &str,
        model_format: &str,
        model_name: &str,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_u32();
        sizer.size_seq_4(input_shape.len());
        let o1 = sizer.offset();
        sizer.size_u8();
        sizer.size_u32();
        sizer.size_seq_4(output_shape.len());
        let o2 = sizer.offset();
        sizer.size_u8();
        sizer.size_u32();
        for l in labels {
            sizer.size_string(l);
        }
        let o3 = sizer.offset();
        sizer.size_string(model_type);
        let o4 = sizer.offset();
        sizer.size_string(model_format);
        let o5 = sizer.offset();
        sizer.size_string(model_name);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_u32(input_shape.len() as u32);
        w.write_slice_u32(input_shape);
        w.write_u8(input_type);
        w.write_u32(output_shape.len() as u32);
        w.write_slice_u32(output_shape);
        w.write_u8(output_type);
        w.write_u32(labels.len() as u32);
        for l in labels {
            w.write_string(l);
        }
        w.write_string(model_type);
        w.write_string(model_format);
        w.write_string(model_name);
        w.finish()?;

        Ok(ModelInfo {
            offsets: [o0, o1, o2, o3, o4, o5],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `ModelInfoBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> ModelInfoBuilder<'a> {
        ModelInfoBuilder::new()
    }
}

// ── ModelInfoBuilder<'a> ────────────────────────────────────────────

/// Builder for `ModelInfo<Vec<u8>>` with buffer-reuse finalizers.
///
/// `labels` is borrowed as `&'a [&'a str]` so string literals or caller-
/// owned string slices flow through without copy.
pub struct ModelInfoBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    input_shape: &'a [u32],
    input_type: u8,
    output_shape: &'a [u32],
    output_type: u8,
    labels: &'a [&'a str],
    model_type: std::borrow::Cow<'a, str>,
    model_format: std::borrow::Cow<'a, str>,
    model_name: std::borrow::Cow<'a, str>,
}

impl<'a> Default for ModelInfoBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            input_shape: &[],
            input_type: 0,
            output_shape: &[],
            output_type: 0,
            labels: &[],
            model_type: std::borrow::Cow::Borrowed(""),
            model_format: std::borrow::Cow::Borrowed(""),
            model_name: std::borrow::Cow::Borrowed(""),
        }
    }
}

impl<'a> ModelInfoBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn input_shape(&mut self, v: &'a [u32]) -> &mut Self {
        self.input_shape = v;
        self
    }
    pub fn input_type(&mut self, v: u8) -> &mut Self {
        self.input_type = v;
        self
    }
    pub fn output_shape(&mut self, v: &'a [u32]) -> &mut Self {
        self.output_shape = v;
        self
    }
    pub fn output_type(&mut self, v: u8) -> &mut Self {
        self.output_type = v;
        self
    }
    pub fn labels(&mut self, v: &'a [&'a str]) -> &mut Self {
        self.labels = v;
        self
    }
    pub fn model_type(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.model_type = s.into();
        self
    }
    pub fn model_format(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.model_format = s.into();
        self
    }
    pub fn model_name(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.model_name = s.into();
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        s.size_u32();
        s.size_seq_4(self.input_shape.len());
        s.size_u8();
        s.size_u32();
        s.size_seq_4(self.output_shape.len());
        s.size_u8();
        s.size_u32();
        for l in self.labels {
            s.size_string(l);
        }
        s.size_string(&self.model_type);
        s.size_string(&self.model_format);
        s.size_string(&self.model_name);
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_u32(self.input_shape.len() as u32);
        w.write_slice_u32(self.input_shape);
        w.write_u8(self.input_type);
        w.write_u32(self.output_shape.len() as u32);
        w.write_slice_u32(self.output_shape);
        w.write_u8(self.output_type);
        w.write_u32(self.labels.len() as u32);
        for l in self.labels {
            w.write_string(l);
        }
        w.write_string(&self.model_type);
        w.write_string(&self.model_format);
        w.write_string(&self.model_name);
        w.finish()
    }

    pub fn build(&self) -> Result<ModelInfo<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        ModelInfo::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

// ── Vibration<B> ────────────────────────────────────────────────────
//
// CDR layout: Header → pad to 8 → offsets[0] (Vector3 vibration start),
//   Vector3 vibration (24 bytes),
//   float32 band_lower_hz, float32 band_upper_hz,
//   uint8 measurement_type, uint8 unit,
//   pad to 4 → uint32 count + uint32[] clipping.
//
// offsets contains a single cached value:
//   offsets[0] = aligned start of `vibration`.
//
// All remaining fields are accessed at fixed compile-time-constant
// deltas from offsets[0] (including the clipping sequence count/data)
// because fields are ordered by descending alignment, sidestepping the
// EDGEAI-1243 class of bug entirely.

/// `measurement_type` enum values for [`Vibration`].
pub mod vibration_measurement {
    pub const UNKNOWN: u8 = 0;
    pub const RMS: u8 = 1;
    pub const PEAK: u8 = 2;
    pub const PEAK_TO_PEAK: u8 = 3;
}

/// `unit` enum values for [`Vibration`].
pub mod vibration_unit {
    pub const UNKNOWN: u8 = 0;
    pub const ACCEL_M_PER_S2: u8 = 1;
    pub const ACCEL_G: u8 = 2;
    pub const VELOCITY_MM_PER_S: u8 = 3;
    pub const DISPLACEMENT_UM: u8 = 4;
    pub const VELOCITY_IN_PER_S: u8 = 5;
    pub const DISPLACEMENT_MIL: u8 = 6;
}

pub struct Vibration<B> {
    buf: B,
    // offsets[0]: Vector3 `vibration` start (8-aligned after Header).
    //
    // Fields laid out by descending alignment (Vector3 → f32 → u8 → seq),
    // so every subsequent field sits at a compile-time-constant delta
    // from offsets[0]:
    //
    //   vibration           offsets[0]       (24 B)
    //   band_lower_hz       offsets[0] + 24  (f32)
    //   band_upper_hz       offsets[0] + 28  (f32)
    //   measurement_type    offsets[0] + 32  (u8)
    //   unit                offsets[0] + 33  (u8)
    //   [ 2 bytes constant pad to 4-align ]
    //   clipping seq-count  offsets[0] + 36  (u32)
    //
    // The 2-byte pad between `unit` and `clipping` is invariant because
    // offsets[0] is 8-aligned (hence 4-aligned relative to CDR payload
    // start). No position-dependent padding anywhere.
    offsets: [usize; 1],
}

impl<B: AsRef<[u8]>> Vibration<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        use crate::geometry_msgs::Vector3;
        let header = crate::std_msgs::Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let pre = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), pre);
        c.align(8);
        let o0 = c.offset();
        Vector3::read_cdr(&mut c)?;
        c.read_f32()?; // band_lower_hz
        c.read_f32()?; // band_upper_hz
        c.read_u8()?; // measurement_type
        c.read_u8()?; // unit
        c.align(4);
        // u32 = 4 bytes each; hardening check against pathological counts.
        let raw = c.read_u32()?;
        let n = c.check_seq_count(raw, 4)?;
        for _ in 0..n {
            c.read_u32()?;
        }
        Ok(Vibration { offsets: [o0], buf })
    }

    /// Returns a `Header` view by re-parsing the CDR buffer prefix.
    pub fn header(&self) -> crate::std_msgs::Header<&[u8]> {
        crate::std_msgs::Header::from_cdr(self.buf.as_ref())
            .expect("header bytes validated during from_cdr")
    }
    pub fn stamp(&self) -> crate::builtin_interfaces::Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }
    pub fn vibration(&self) -> crate::geometry_msgs::Vector3 {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        crate::geometry_msgs::Vector3::read_cdr(&mut c)
            .expect("vibration validated during from_cdr")
    }
    pub fn band_lower_hz(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 24)
    }
    pub fn band_upper_hz(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 28)
    }
    pub fn measurement_type(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[0] + 32)
    }
    pub fn unit(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[0] + 33)
    }
    pub fn clipping_len(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0] + 36)
    }
    /// Byte offset of the `clipping` sequence (u32 count, then elements).
    /// Exposed for allocation-free decoders (e.g. FFI).
    pub fn clipping_seq_offset(&self) -> usize {
        self.offsets[0] + 36
    }
    pub fn clipping(&self) -> Vec<u32> {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0] + 36);
        let n = c
            .read_u32()
            .expect("clipping length validated during from_cdr") as usize;
        let mut out = Vec::with_capacity(n);
        for _ in 0..n {
            out.push(
                c.read_u32()
                    .expect("clipping element validated during from_cdr"),
            );
        }
        out
    }
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl Vibration<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use Vibration::builder() for allocation-free buffer reuse; Vibration::new will be removed in 4.0"
    )]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        stamp: crate::builtin_interfaces::Time,
        frame_id: &str,
        measurement_type: u8,
        unit: u8,
        band_lower_hz: f32,
        band_upper_hz: f32,
        vibration: crate::geometry_msgs::Vector3,
        clipping: &[u32],
    ) -> Result<Self, CdrError> {
        use crate::builtin_interfaces::Time;
        use crate::geometry_msgs::Vector3;
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        sizer.align(8);
        let o0 = sizer.offset();
        Vector3::size_cdr(&mut sizer);
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_u8();
        sizer.size_u8();
        sizer.align(4);
        sizer.size_u32();
        for _ in clipping {
            sizer.size_u32();
        }

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        vibration.write_cdr(&mut w);
        w.write_f32(band_lower_hz);
        w.write_f32(band_upper_hz);
        w.write_u8(measurement_type);
        w.write_u8(unit);
        w.write_u32(clipping.len() as u32);
        for v in clipping {
            w.write_u32(*v);
        }
        w.finish()?;

        Ok(Vibration { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `VibrationBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> VibrationBuilder<'a> {
        VibrationBuilder::new()
    }
}

// ── VibrationBuilder<'a> ────────────────────────────────────────────

/// Builder for `Vibration<Vec<u8>>` with buffer-reuse finalizers.
///
/// `clipping` is borrowed from a caller-owned slice of 32-bit sample
/// indices; the borrow must remain valid until `build()`,
/// `encode_into_vec()`, or `encode_into_slice()` is called.
pub struct VibrationBuilder<'a> {
    stamp: crate::builtin_interfaces::Time,
    frame_id: std::borrow::Cow<'a, str>,
    measurement_type: u8,
    unit: u8,
    band_lower_hz: f32,
    band_upper_hz: f32,
    vibration: crate::geometry_msgs::Vector3,
    clipping: &'a [u32],
}

impl<'a> Default for VibrationBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: crate::builtin_interfaces::Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            measurement_type: 0,
            unit: 0,
            band_lower_hz: 0.0,
            band_upper_hz: 0.0,
            vibration: crate::geometry_msgs::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            clipping: &[],
        }
    }
}

impl<'a> VibrationBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: crate::builtin_interfaces::Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn measurement_type(&mut self, v: u8) -> &mut Self {
        self.measurement_type = v;
        self
    }
    pub fn unit(&mut self, v: u8) -> &mut Self {
        self.unit = v;
        self
    }
    pub fn band_lower_hz(&mut self, v: f32) -> &mut Self {
        self.band_lower_hz = v;
        self
    }
    pub fn band_upper_hz(&mut self, v: f32) -> &mut Self {
        self.band_upper_hz = v;
        self
    }
    pub fn vibration(&mut self, v: crate::geometry_msgs::Vector3) -> &mut Self {
        self.vibration = v;
        self
    }
    pub fn clipping(&mut self, v: &'a [u32]) -> &mut Self {
        self.clipping = v;
        self
    }

    fn size(&self) -> usize {
        use crate::builtin_interfaces::Time;
        use crate::geometry_msgs::Vector3;
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        s.align(8);
        Vector3::size_cdr(&mut s);
        s.size_f32();
        s.size_f32();
        s.size_u8();
        s.size_u8();
        s.align(4);
        s.size_u32();
        for _ in self.clipping {
            s.size_u32();
        }
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        self.vibration.write_cdr(&mut w);
        w.write_f32(self.band_lower_hz);
        w.write_f32(self.band_upper_hz);
        w.write_u8(self.measurement_type);
        w.write_u8(self.unit);
        w.write_u32(self.clipping.len() as u32);
        for v in self.clipping {
            w.write_u32(*v);
        }
        w.finish()
    }

    pub fn build(&self) -> Result<Vibration<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        Vibration::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

// ── Registry ────────────────────────────────────────────────────────

/// Check if a type name is supported by this module.
pub fn is_type_supported(type_name: &str) -> bool {
    matches!(
        type_name,
        "Box"
            | "CameraFrame"
            | "CameraPlane"
            | "Date"
            | "Detect"
            | "DmaBuffer"
            | "LocalTime"
            | "Mask"
            | "Model"
            | "ModelInfo"
            | "RadarCube"
            | "RadarInfo"
            | "Track"
            | "Vibration"
    )
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &[
        "edgefirst_msgs/msg/Box",
        "edgefirst_msgs/msg/CameraFrame",
        "edgefirst_msgs/msg/CameraPlane",
        "edgefirst_msgs/msg/Date",
        "edgefirst_msgs/msg/Detect",
        "edgefirst_msgs/msg/DmaBuffer",
        "edgefirst_msgs/msg/LocalTime",
        "edgefirst_msgs/msg/Mask",
        "edgefirst_msgs/msg/Model",
        "edgefirst_msgs/msg/ModelInfo",
        "edgefirst_msgs/msg/RadarCube",
        "edgefirst_msgs/msg/RadarInfo",
        "edgefirst_msgs/msg/Track",
        "edgefirst_msgs/msg/Vibration",
    ]
}

// SchemaType implementations
use crate::schema_registry::SchemaType;

impl SchemaType for Date {
    const SCHEMA_NAME: &'static str = "edgefirst_msgs/msg/Date";
}

#[cfg(test)]
#[allow(deprecated)] // Tests exercise CameraFrame::new, which is deprecated in 3.2.0 but still supported until 4.0.
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::cdr::{decode_fixed, encode_fixed};

    #[test]
    fn date_roundtrip() {
        let cases = [
            (2025, 1, 27, "typical"),
            (2000, 12, 31, "end of year"),
            (1970, 1, 1, "unix epoch"),
        ];
        for (year, month, day, name) in cases {
            let date = Date { year, month, day };
            let bytes = encode_fixed(&date).unwrap();
            let decoded: Date = decode_fixed(&bytes).unwrap();
            assert_eq!(date, decoded, "failed for case: {}", name);
        }
    }

    #[test]
    fn mask_roundtrip() {
        let mask = Mask::new(480, 640, 0, "", &vec![0u8; 480 * 640], false).unwrap();
        assert_eq!(mask.height(), 480);
        assert_eq!(mask.width(), 640);
        assert_eq!(mask.length(), 0);
        assert_eq!(mask.encoding(), "");
        assert_eq!(mask.mask_data().len(), 480 * 640);
        assert!(!mask.boxed());

        let bytes = mask.to_cdr();
        let decoded = Mask::from_cdr(bytes).unwrap();
        assert_eq!(decoded.height(), 480);
        assert_eq!(decoded.width(), 640);

        // Compressed mask
        let compressed = Mask::new(1080, 1920, 5, "zstd", &[1, 2, 3, 4, 5], true).unwrap();
        assert_eq!(compressed.encoding(), "zstd");
        assert_eq!(compressed.mask_data(), &[1, 2, 3, 4, 5]);
        assert!(compressed.boxed());

        let bytes = compressed.to_cdr();
        let decoded = Mask::from_cdr(bytes).unwrap();
        assert_eq!(decoded.encoding(), "zstd");
        assert!(decoded.boxed());
    }

    #[test]
    #[allow(deprecated)]
    fn dmabuf_roundtrip() {
        let dmabuf = DmaBuffer::new(
            Time::new(100, 0),
            "camera",
            12345,
            42,
            1920,
            1080,
            1920 * 3,
            0x34325247,
            1920 * 1080 * 3,
        )
        .unwrap();
        assert_eq!(dmabuf.stamp(), Time::new(100, 0));
        assert_eq!(dmabuf.frame_id(), "camera");
        assert_eq!(dmabuf.pid(), 12345);
        assert_eq!(dmabuf.fd(), 42);
        assert_eq!(dmabuf.width(), 1920);
        assert_eq!(dmabuf.height(), 1080);
        assert_eq!(dmabuf.stride(), 1920 * 3);
        assert_eq!(dmabuf.fourcc(), 0x34325247);
        assert_eq!(dmabuf.length(), 1920 * 1080 * 3);

        let bytes = dmabuf.to_cdr();
        let decoded = DmaBuffer::from_cdr(bytes).unwrap();
        assert_eq!(decoded.pid(), 12345);
        assert_eq!(decoded.fd(), 42);
    }

    #[test]
    fn camera_frame_roundtrip_empty() {
        let cf = CameraFrame::new(
            Time::new(1, 0),
            "cam0",
            42,
            1234,
            1920,
            1080,
            "NV12",
            "bt709",
            "bt709",
            "bt709",
            "limited",
            -1,
            &[],
        )
        .unwrap();
        assert_eq!(cf.seq(), 42);
        assert_eq!(cf.pid(), 1234);
        assert_eq!(cf.width(), 1920);
        assert_eq!(cf.height(), 1080);
        assert_eq!(cf.format(), "NV12");
        assert_eq!(cf.color_space(), "bt709");
        assert_eq!(cf.color_range(), "limited");
        assert_eq!(cf.fence_fd(), -1);
        assert_eq!(cf.num_planes(), 0);

        let bytes = cf.to_cdr();
        let decoded = CameraFrame::<&[u8]>::from_cdr(&bytes[..]).unwrap();
        assert_eq!(decoded.seq(), 42);
        assert_eq!(decoded.format(), "NV12");
        assert_eq!(decoded.num_planes(), 0);
    }

    #[test]
    fn camera_frame_roundtrip_two_planes() {
        let y = CameraPlaneView {
            fd: 42,
            offset: 0,
            stride: 1920,
            size: 2_073_600,
            used: 2_073_600,
            data: &[],
        };
        let uv = CameraPlaneView {
            fd: 42,
            offset: 2_073_600,
            stride: 1920,
            size: 1_036_800,
            used: 1_036_800,
            data: &[],
        };
        let cf = CameraFrame::new(
            Time::new(2, 0),
            "cam0",
            100,
            1234,
            1920,
            1080,
            "NV12",
            "bt709",
            "bt709",
            "bt709",
            "limited",
            77,
            &[y, uv],
        )
        .unwrap();

        let bytes = cf.to_cdr();
        let decoded = CameraFrame::<&[u8]>::from_cdr(&bytes[..]).unwrap();
        assert_eq!(decoded.fence_fd(), 77);
        assert_eq!(decoded.num_planes(), 2);
        let planes = decoded.planes();
        assert_eq!(planes.len(), 2);
        assert_eq!(planes[0].fd, 42);
        assert_eq!(planes[0].offset, 0);
        assert_eq!(planes[1].offset, 2_073_600);
        assert_eq!(planes[0].used, planes[0].size);
    }

    #[test]
    fn camera_frame_inlined_data_roundtrip() {
        let data: Vec<u8> = (0..32u8).collect();
        let plane = CameraPlaneView {
            fd: -1,
            offset: 0,
            stride: 16,
            size: 32,
            used: 32,
            data: &data,
        };
        let cf = CameraFrame::new(
            Time::new(3, 0),
            "bridge",
            1,
            0,
            2,
            16,
            "rgb8",
            "srgb",
            "srgb",
            "",
            "full",
            -1,
            &[plane],
        )
        .unwrap();
        let decoded = CameraFrame::<&[u8]>::from_cdr(cf.as_cdr()).unwrap();
        let planes = decoded.planes();
        assert_eq!(planes[0].fd, -1);
        assert_eq!(planes[0].data.len(), 32);
        assert_eq!(planes[0].data[0], 0);
        assert_eq!(planes[0].data[31], 31);
    }

    #[test]
    fn camera_frame_contract_rejections() {
        let stamp = Time::new(0, 0);
        // Zero width rejected
        assert!(CameraFrame::new(stamp, "c", 0, 0, 0, 1, "rgb8", "", "", "", "", -1, &[]).is_err());
        // Zero height rejected
        assert!(CameraFrame::new(stamp, "c", 0, 0, 1, 0, "rgb8", "", "", "", "", -1, &[]).is_err());
        // used > size rejected
        let bad_plane = CameraPlaneView {
            fd: 1,
            offset: 0,
            stride: 1,
            size: 1,
            used: 2,
            data: &[],
        };
        assert!(CameraFrame::new(
            stamp,
            "c",
            0,
            0,
            1,
            1,
            "rgb8",
            "",
            "",
            "",
            "",
            -1,
            &[bad_plane]
        )
        .is_err());
        // fd < -1 rejected
        let bad_fd = CameraPlaneView {
            fd: -5,
            offset: 0,
            stride: 1,
            size: 1,
            used: 1,
            data: &[],
        };
        assert!(CameraFrame::new(
            stamp,
            "c",
            0,
            0,
            1,
            1,
            "rgb8",
            "",
            "",
            "",
            "",
            -1,
            &[bad_fd]
        )
        .is_err());
        // fd >= 0 with non-empty data rejected
        let data = vec![1u8];
        let both = CameraPlaneView {
            fd: 5,
            offset: 0,
            stride: 1,
            size: 1,
            used: 1,
            data: &data,
        };
        assert!(
            CameraFrame::new(stamp, "c", 0, 0, 1, 1, "rgb8", "", "", "", "", -1, &[both]).is_err()
        );
    }

    #[test]
    fn camera_frame_rejects_wrong_endianness() {
        // Build a valid single-plane frame, flip the CDR endianness marker
        // from little-endian (0x0001) to big-endian (0x0100) and confirm
        // from_cdr rejects it with an error rather than decoding garbage.
        let stamp = Time::new(1, 0);
        let plane = CameraPlaneView {
            fd: 1,
            offset: 0,
            stride: 1,
            size: 1,
            used: 1,
            data: &[],
        };
        let cf = CameraFrame::new(
            stamp,
            "cam",
            0,
            0,
            1,
            1,
            "rgb8",
            "",
            "",
            "",
            "",
            -1,
            &[plane],
        )
        .unwrap();
        let mut bytes = cf.to_cdr();
        // CDR header: [0]=repr-id hi, [1]=repr-id lo, [2..4]=options.
        // Valid LE payload encoding uses 0x00 0x01; invert to 0x00 0x00 (BE).
        bytes[1] = 0x00;
        assert!(CameraFrame::<&[u8]>::from_cdr(&bytes).is_err());
    }

    #[test]
    fn camera_frame_decoder_rejects_fd_below_minus_one() {
        // Build a valid single-plane frame (fd=1, size=1, used=1, data=[]) so
        // the plane occupies the last 24 bytes of the CDR buffer. Flip `fd`
        // to -5 and confirm the decoder's new contract check rejects it.
        let stamp = Time::new(1, 0);
        let plane = CameraPlaneView {
            fd: 1,
            offset: 0,
            stride: 1,
            size: 1,
            used: 1,
            data: &[],
        };
        let cf =
            CameraFrame::new(stamp, "c", 0, 0, 1, 1, "rgb8", "", "", "", "", -1, &[plane]).unwrap();
        let mut bytes = cf.to_cdr();
        let fd_off = bytes.len() - 24;
        bytes[fd_off..fd_off + 4].copy_from_slice(&(-5i32).to_le_bytes());
        assert!(CameraFrame::<&[u8]>::from_cdr(&bytes).is_err());
    }

    #[test]
    fn camera_frame_decoder_rejects_used_greater_than_size() {
        // Same base frame; `used` is at [len-8 .. len-4] for the 24-byte
        // trailing plane block. Overwrite with a value exceeding `size`.
        let stamp = Time::new(1, 0);
        let plane = CameraPlaneView {
            fd: 1,
            offset: 0,
            stride: 1,
            size: 1,
            used: 1,
            data: &[],
        };
        let cf =
            CameraFrame::new(stamp, "c", 0, 0, 1, 1, "rgb8", "", "", "", "", -1, &[plane]).unwrap();
        let mut bytes = cf.to_cdr();
        let used_off = bytes.len() - 8;
        bytes[used_off..used_off + 4].copy_from_slice(&99u32.to_le_bytes());
        assert!(CameraFrame::<&[u8]>::from_cdr(&bytes).is_err());
    }

    #[test]
    fn camera_frame_rejects_inlined_size_mismatch() {
        // fd == -1 requires size as usize == data.len() per CameraPlane.msg.
        // `new` must reject the mismatch.
        let data = [0x42u8, 0x43, 0x44, 0x45];
        let plane = CameraPlaneView {
            fd: -1,
            offset: 0,
            stride: 0,
            size: 99, // does not match data.len() == 4
            used: 4,
            data: &data,
        };
        assert!(CameraFrame::new(
            Time::new(0, 0),
            "c",
            0,
            0,
            1,
            1,
            "rgb8",
            "",
            "",
            "",
            "",
            -1,
            &[plane]
        )
        .is_err());
    }

    #[test]
    fn camera_frame_decoder_rejects_inlined_size_mismatch() {
        // Start with a valid inlined-data plane (fd=-1, size==data.len()), then
        // mutate `size` in the encoded CDR to break the invariant and confirm
        // from_cdr rejects it.
        let data = [0x42u8, 0x43, 0x44, 0x45];
        let plane = CameraPlaneView {
            fd: -1,
            offset: 0,
            stride: 0xDEADBEEF,
            size: 4,
            used: 4,
            data: &data,
        };
        let cf = CameraFrame::new(
            Time::new(1, 0),
            "c",
            0,
            0,
            1,
            1,
            "rgb8",
            "",
            "",
            "",
            "",
            -1,
            &[plane],
        )
        .unwrap();
        let mut bytes = cf.to_cdr();
        let needle = 0xDEADBEEFu32.to_le_bytes();
        let stride_off = bytes
            .windows(4)
            .position(|w| w == needle)
            .expect("stride sentinel");
        // size is the u32 immediately after stride.
        let size_off = stride_off + 4;
        bytes[size_off..size_off + 4].copy_from_slice(&99u32.to_le_bytes());
        assert!(CameraFrame::<&[u8]>::from_cdr(&bytes).is_err());
    }

    #[test]
    fn camera_frame_decoder_rejects_positive_fd_with_inline_data() {
        // Start with a valid inlined-data plane (fd=-1, data non-empty) so
        // `new` accepts it, then mutate `fd` to a non-negative value to
        // trigger the decoder's `fd >= 0 && data non-empty` rejection.
        // Uses a distinctive `stride` sentinel to locate the plane.
        let stamp = Time::new(1, 0);
        let data = [0x42u8, 0x43, 0x44, 0x45];
        let plane = CameraPlaneView {
            fd: -1,
            offset: 0,
            stride: 0xDEADBEEF,
            size: 4,
            used: 4,
            data: &data,
        };
        let cf =
            CameraFrame::new(stamp, "c", 0, 0, 1, 1, "rgb8", "", "", "", "", -1, &[plane]).unwrap();
        let mut bytes = cf.to_cdr();
        let needle = 0xDEADBEEFu32.to_le_bytes();
        let stride_off = bytes
            .windows(4)
            .position(|w| w == needle)
            .expect("stride sentinel");
        let fd_off = stride_off - 8;
        bytes[fd_off..fd_off + 4].copy_from_slice(&5i32.to_le_bytes());
        assert!(CameraFrame::<&[u8]>::from_cdr(&bytes).is_err());
    }

    #[test]
    fn camera_frame_registered_in_type_list() {
        assert!(is_type_supported("CameraFrame"));
        assert!(is_type_supported("CameraPlane"));
        assert!(list_types().contains(&"edgefirst_msgs/msg/CameraFrame"));
        assert!(list_types().contains(&"edgefirst_msgs/msg/CameraPlane"));
    }

    #[test]
    fn local_time_roundtrip() {
        let lt = LocalTime::new(
            Time::new(0, 0),
            "clock",
            Date {
                year: 2025,
                month: 1,
                day: 27,
            },
            Time::new(43200, 0),
            -300,
        )
        .unwrap();
        assert_eq!(lt.frame_id(), "clock");
        assert_eq!(
            lt.date(),
            Date {
                year: 2025,
                month: 1,
                day: 27
            }
        );
        assert_eq!(lt.time(), Time::new(43200, 0));
        assert_eq!(lt.timezone(), -300);

        let bytes = lt.to_cdr();
        let decoded = LocalTime::from_cdr(bytes).unwrap();
        assert_eq!(
            decoded.date(),
            Date {
                year: 2025,
                month: 1,
                day: 27
            }
        );
        assert_eq!(decoded.timezone(), -300);
    }

    #[test]
    fn radar_cube_roundtrip() {
        let cube = RadarCube::new(
            Time::new(1234567890, 123456789),
            "radar",
            1234567890123456,
            &[6, 1, 5, 2],
            &[16, 256, 4, 64],
            &[1.0, 2.5, 1.0, 0.5],
            &[100, 200, -100, -200],
            true,
        )
        .unwrap();
        assert_eq!(cube.stamp(), Time::new(1234567890, 123456789));
        assert_eq!(cube.frame_id(), "radar");
        assert_eq!(cube.timestamp(), 1234567890123456);
        assert_eq!(cube.layout(), &[6, 1, 5, 2]);
        assert_eq!(cube.shape(), vec![16, 256, 4, 64]);
        assert_eq!(cube.scales(), vec![1.0, 2.5, 1.0, 0.5]);
        assert!(cube.is_complex());

        let bytes = cube.to_cdr();
        let decoded = RadarCube::from_cdr(bytes).unwrap();
        assert_eq!(decoded.timestamp(), 1234567890123456);
        assert_eq!(decoded.layout(), &[6, 1, 5, 2]);
        assert!(decoded.is_complex());
    }

    #[test]
    fn radar_info_roundtrip() {
        let info = RadarInfo::new(
            Time::new(0, 0),
            "radar",
            "77GHz",
            "short",
            "off",
            "high",
            true,
        )
        .unwrap();
        assert_eq!(info.center_frequency(), "77GHz");
        assert_eq!(info.frequency_sweep(), "short");
        assert_eq!(info.range_toggle(), "off");
        assert_eq!(info.detection_sensitivity(), "high");
        assert!(info.cube());

        let bytes = info.to_cdr();
        let decoded = RadarInfo::from_cdr(bytes).unwrap();
        assert_eq!(decoded.center_frequency(), "77GHz");
        assert!(decoded.cube());
    }

    #[test]
    fn detect_roundtrip() {
        // Empty detections
        let empty = Detect::new(
            Time::new(0, 0),
            "",
            Time::new(0, 0),
            Time::new(0, 0),
            Time::new(0, 0),
            &[],
        )
        .unwrap();
        assert_eq!(empty.boxes_len(), 0);

        let bytes = empty.to_cdr();
        let decoded = Detect::from_cdr(bytes).unwrap();
        assert_eq!(decoded.boxes_len(), 0);

        // With detections
        let boxes = [DetectBoxView {
            center_x: 0.5,
            center_y: 0.5,
            width: 0.1,
            height: 0.2,
            label: "car",
            score: 0.98,
            distance: 10.0,
            speed: 5.0,
            track_id: "t1",
            track_lifetime: 5,
            track_created: Time::new(95, 0),
        }];
        let detect = Detect::new(
            Time::new(100, 500_000_000),
            "camera",
            Time::new(100, 400_000_000),
            Time::new(0, 50_000_000),
            Time::new(100, 500_000_000),
            &boxes,
        )
        .unwrap();
        assert_eq!(detect.boxes_len(), 1);
        let b = detect.boxes();
        assert_eq!(b[0].label, "car");
        assert_eq!(b[0].score, 0.98);

        let bytes = detect.to_cdr();
        let decoded = Detect::from_cdr(bytes).unwrap();
        assert_eq!(decoded.boxes_len(), 1);
        let b = decoded.boxes();
        assert_eq!(b[0].label, "car");
    }

    #[test]
    fn detect_multi_box_varying_strings() {
        let boxes = [
            DetectBoxView {
                center_x: 0.1,
                center_y: 0.2,
                width: 0.5,
                height: 0.6,
                label: "a",
                score: 0.95,
                distance: 5.0,
                speed: 1.0,
                track_id: "t",
                track_lifetime: 1,
                track_created: Time::new(1, 0),
            },
            DetectBoxView {
                center_x: 0.3,
                center_y: 0.4,
                width: 0.2,
                height: 0.3,
                label: "person",
                score: 0.87,
                distance: 12.0,
                speed: 3.0,
                track_id: "track_long_id",
                track_lifetime: 10,
                track_created: Time::new(2, 0),
            },
            DetectBoxView {
                center_x: 0.7,
                center_y: 0.8,
                width: 0.1,
                height: 0.1,
                label: "ab",
                score: 0.50,
                distance: 0.0,
                speed: 0.0,
                track_id: "abc",
                track_lifetime: 0,
                track_created: Time::new(0, 0),
            },
        ];
        let detect = Detect::new(
            Time::new(100, 0),
            "camera",
            Time::new(99, 0),
            Time::new(0, 50_000_000),
            Time::new(100, 0),
            &boxes,
        )
        .unwrap();
        assert_eq!(detect.boxes_len(), 3);
        let decoded_boxes = detect.boxes();
        assert_eq!(decoded_boxes[0].label, "a");
        assert_eq!(decoded_boxes[0].track_id, "t");
        assert_eq!(decoded_boxes[1].label, "person");
        assert_eq!(decoded_boxes[1].track_id, "track_long_id");
        assert_eq!(decoded_boxes[2].label, "ab");
        assert_eq!(decoded_boxes[2].track_id, "abc");

        let bytes = detect.to_cdr();
        let decoded = Detect::from_cdr(bytes).unwrap();
        assert_eq!(decoded.boxes_len(), 3);
        let b = decoded.boxes();
        assert_eq!(b[0].label, "a");
        assert_eq!(b[0].track_id, "t");
        assert_eq!(b[0].score, 0.95);
        assert_eq!(b[1].label, "person");
        assert_eq!(b[1].track_id, "track_long_id");
        assert_eq!(b[1].track_lifetime, 10);
        assert_eq!(b[2].label, "ab");
        assert_eq!(b[2].track_id, "abc");
    }

    #[test]
    fn model_roundtrip() {
        let model = Model::new(
            Time::new(0, 0),
            "model",
            Duration::new(0, 1_000_000),
            Duration::new(0, 5_000_000),
            Duration::new(0, 500_000),
            Duration::new(0, 2_000_000),
            &[],
            &[],
        )
        .unwrap();
        assert_eq!(model.input_time(), Duration::new(0, 1_000_000));
        assert_eq!(model.boxes_len(), 0);
        assert_eq!(model.masks_len(), 0);

        let bytes = model.to_cdr();
        let decoded = Model::from_cdr(bytes).unwrap();
        assert_eq!(decoded.input_time(), Duration::new(0, 1_000_000));
    }

    #[test]
    fn model_info_roundtrip() {
        let info = ModelInfo::new(
            Time::new(0, 0),
            "",
            &[1, 3, 640, 640],
            8,
            &[1, 25200, 85],
            8,
            &["person", "car"],
            "yolov8",
            "onnx",
            "yolov8n",
        )
        .unwrap();
        assert_eq!(info.input_shape(), vec![1, 3, 640, 640]);
        assert_eq!(info.input_type(), 8);
        assert_eq!(info.output_shape(), vec![1, 25200, 85]);
        assert_eq!(info.output_type(), 8);
        assert_eq!(info.labels(), vec!["person", "car"]);
        assert_eq!(info.model_type(), "yolov8");
        assert_eq!(info.model_format(), "onnx");
        assert_eq!(info.model_name(), "yolov8n");

        let bytes = info.to_cdr();
        let decoded = ModelInfo::from_cdr(bytes).unwrap();
        assert_eq!(decoded.input_shape(), vec![1, 3, 640, 640]);
        assert_eq!(decoded.labels(), vec!["person", "car"]);
        assert_eq!(decoded.model_name(), "yolov8n");
    }

    #[test]
    fn model_info_empty_labels() {
        let info = ModelInfo::new(
            Time::new(1, 0),
            "cam",
            &[1, 3, 224, 224],
            8,
            &[1, 10],
            8,
            &[],
            "classifier",
            "onnx",
            "mobilenet",
        )
        .unwrap();
        assert_eq!(info.labels(), Vec::<&str>::new());
        assert_eq!(info.input_shape(), &[1, 3, 224, 224]);
        assert_eq!(info.output_shape(), &[1, 10]);
        assert_eq!(info.model_type(), "classifier");
        assert_eq!(info.model_format(), "onnx");
        assert_eq!(info.model_name(), "mobilenet");

        let bytes = info.to_cdr();
        let decoded = ModelInfo::from_cdr(bytes).unwrap();
        assert_eq!(decoded.labels(), Vec::<&str>::new());
        assert_eq!(decoded.model_type(), "classifier");
        assert_eq!(decoded.model_name(), "mobilenet");
    }

    #[test]
    fn model_info_single_empty_label() {
        let info = ModelInfo::new(
            Time::new(0, 0),
            "",
            &[1],
            0,
            &[1],
            0,
            &[""],
            "det",
            "tflite",
            "m",
        )
        .unwrap();
        assert_eq!(info.labels(), vec![""]);

        let bytes = info.to_cdr();
        let decoded = ModelInfo::from_cdr(bytes).unwrap();
        assert_eq!(decoded.labels(), vec![""]);
        assert_eq!(decoded.model_type(), "det");
    }

    #[test]
    fn model_info_alignment_stressing_labels() {
        let info = ModelInfo::new(
            Time::new(0, 0),
            "f",
            &[1, 3, 320, 320],
            8,
            &[1, 100, 6],
            8,
            &["a", "ab", "abc", "abcd", "abcde"],
            "object_detection",
            "DeepViewRT",
            "yolov8n",
        )
        .unwrap();
        assert_eq!(info.labels(), vec!["a", "ab", "abc", "abcd", "abcde"]);
        assert_eq!(info.input_shape(), &[1, 3, 320, 320]);
        assert_eq!(info.model_type(), "object_detection");

        let bytes = info.to_cdr();
        let decoded = ModelInfo::from_cdr(bytes).unwrap();
        assert_eq!(decoded.labels(), vec!["a", "ab", "abc", "abcd", "abcde"]);
        assert_eq!(decoded.input_shape(), &[1, 3, 320, 320]);
        assert_eq!(decoded.model_type(), "object_detection");
        assert_eq!(decoded.model_name(), "yolov8n");
    }

    #[test]
    fn model_info_many_labels() {
        let label_strs: Vec<String> = (0..80).map(|i| format!("class_{i}")).collect();
        let labels: Vec<&str> = label_strs.iter().map(|s| s.as_str()).collect();
        let info = ModelInfo::new(
            Time::new(0, 0),
            "cam0",
            &[1, 3, 640, 640],
            8,
            &[1, 84, 8400],
            8,
            &labels,
            "object_detection",
            "DeepViewRT",
            "yolov8n",
        )
        .unwrap();
        assert_eq!(info.labels().len(), 80);
        assert_eq!(info.labels()[0], "class_0");
        assert_eq!(info.labels()[79], "class_79");

        let bytes = info.to_cdr();
        let decoded = ModelInfo::from_cdr(bytes).unwrap();
        assert_eq!(decoded.labels().len(), 80);
        assert_eq!(decoded.labels()[0], "class_0");
        assert_eq!(decoded.labels()[79], "class_79");
        assert_eq!(decoded.model_name(), "yolov8n");
    }

    #[test]
    fn model_info_empty_shapes() {
        let info = ModelInfo::new(
            Time::new(0, 0),
            "",
            &[],
            0,
            &[],
            0,
            &["label"],
            "type",
            "format",
            "name",
        )
        .unwrap();
        assert_eq!(info.input_shape(), &[] as &[u32]);
        assert_eq!(info.output_shape(), &[] as &[u32]);
        assert_eq!(info.labels(), vec!["label"]);

        let bytes = info.to_cdr();
        let decoded = ModelInfo::from_cdr(bytes).unwrap();
        assert_eq!(decoded.input_shape(), &[] as &[u32]);
        assert_eq!(decoded.output_shape(), &[] as &[u32]);
        assert_eq!(decoded.labels(), vec!["label"]);
    }

    #[test]
    fn track_roundtrip() {
        let track = Track::new("t1", 5, Time::new(95, 0)).unwrap();
        assert_eq!(track.id(), "t1");
        assert_eq!(track.lifetime(), 5);
        assert_eq!(track.created(), Time::new(95, 0));

        let bytes = track.to_cdr();
        let decoded = Track::from_cdr(bytes).unwrap();
        assert_eq!(decoded.id(), "t1");
        assert_eq!(decoded.lifetime(), 5);
    }

    #[test]
    fn detect_box_roundtrip() {
        let b = DetectBox::new(
            0.5,
            0.5,
            0.1,
            0.2,
            "car",
            0.98,
            10.0,
            5.0,
            "t1",
            5,
            Time::new(95, 0),
        )
        .unwrap();
        assert_eq!(b.center_x(), 0.5);
        assert_eq!(b.label(), "car");
        assert_eq!(b.score(), 0.98);
        assert_eq!(b.track_id(), "t1");

        let bytes = b.to_cdr();
        let decoded = DetectBox::from_cdr(bytes).unwrap();
        assert_eq!(decoded.label(), "car");
        assert_eq!(decoded.track_id(), "t1");
    }

    #[test]
    fn detect_box_empty_strings() {
        let b = DetectBox::new(
            0.5,
            0.5,
            0.1,
            0.2,
            "",
            0.0,
            0.0,
            0.0,
            "",
            0,
            Time::new(0, 0),
        )
        .unwrap();
        assert_eq!(b.label(), "");
        assert_eq!(b.track_id(), "");
        assert_eq!(b.center_x(), 0.5);

        let bytes = b.to_cdr();
        let decoded = DetectBox::from_cdr(bytes).unwrap();
        assert_eq!(decoded.label(), "");
        assert_eq!(decoded.track_id(), "");
        assert_eq!(decoded.score(), 0.0);
    }

    /// Verify that `Detect::from_cdr_collect_boxes` produces box views with
    /// fields identical to those returned by the two-pass
    /// `Detect::from_cdr(…).boxes()` path.
    #[test]
    fn detect_from_cdr_collect_boxes_matches_boxes() {
        static BYTES: &[u8] = include_bytes!("../testdata/cdr/edgefirst_msgs/Detect_multi.cdr");

        // Path 1: legacy two-pass (from_cdr then .boxes()).
        let detect_ref = Detect::from_cdr(BYTES).expect("reference decode");
        let boxes_ref = detect_ref.boxes();

        // Path 2: single-pass collect helper.
        let (detect_new, boxes_new) =
            Detect::from_cdr_collect_boxes(BYTES).expect("collect decode");

        // Parent-level fields must agree.
        assert_eq!(detect_ref.stamp().sec, detect_new.stamp().sec);
        assert_eq!(detect_ref.stamp().nanosec, detect_new.stamp().nanosec);
        assert_eq!(detect_ref.frame_id(), detect_new.frame_id());
        assert_eq!(detect_ref.boxes_len(), detect_new.boxes_len());

        // Both paths must yield the same number of box views.
        assert_eq!(boxes_ref.len(), boxes_new.len());

        // Every field in every box must be identical.
        for (i, (a, b)) in boxes_ref.iter().zip(boxes_new.iter()).enumerate() {
            assert_eq!(a.center_x, b.center_x, "box[{i}].center_x");
            assert_eq!(a.center_y, b.center_y, "box[{i}].center_y");
            assert_eq!(a.width, b.width, "box[{i}].width");
            assert_eq!(a.height, b.height, "box[{i}].height");
            assert_eq!(a.label, b.label, "box[{i}].label");
            assert_eq!(a.score, b.score, "box[{i}].score");
            assert_eq!(a.distance, b.distance, "box[{i}].distance");
            assert_eq!(a.speed, b.speed, "box[{i}].speed");
            assert_eq!(a.track_id, b.track_id, "box[{i}].track_id");
            assert_eq!(
                a.track_lifetime, b.track_lifetime,
                "box[{i}].track_lifetime"
            );
            assert_eq!(
                a.track_created.sec, b.track_created.sec,
                "box[{i}].track_created.sec"
            );
            assert_eq!(
                a.track_created.nanosec, b.track_created.nanosec,
                "box[{i}].track_created.nanosec"
            );
        }
    }

    /// Verify that `Model::from_cdr_collect_children` produces box and mask
    /// views with fields identical to those returned by the two-pass
    /// `Model::from_cdr(…).boxes()` / `.masks()` path.
    #[test]
    fn model_from_cdr_collect_children_matches_boxes_and_masks() {
        static BYTES: &[u8] = include_bytes!("../testdata/cdr/edgefirst_msgs/Model.cdr");

        // Path 1: legacy two-pass.
        let model_ref = Model::from_cdr(BYTES).expect("reference decode");
        let boxes_ref = model_ref.boxes();
        let masks_ref = model_ref.masks();

        // Path 2: single-pass collect helper.
        let (model_new, boxes_new, masks_new) =
            Model::from_cdr_collect_children(BYTES).expect("collect decode");

        // Parent-level fields must agree.
        assert_eq!(model_ref.stamp().sec, model_new.stamp().sec);
        assert_eq!(model_ref.stamp().nanosec, model_new.stamp().nanosec);
        assert_eq!(model_ref.frame_id(), model_new.frame_id());
        assert_eq!(model_ref.boxes_len(), model_new.boxes_len());
        assert_eq!(model_ref.masks_len(), model_new.masks_len());

        // Box views.
        assert_eq!(boxes_ref.len(), boxes_new.len());
        for (i, (a, b)) in boxes_ref.iter().zip(boxes_new.iter()).enumerate() {
            assert_eq!(a.center_x, b.center_x, "box[{i}].center_x");
            assert_eq!(a.center_y, b.center_y, "box[{i}].center_y");
            assert_eq!(a.width, b.width, "box[{i}].width");
            assert_eq!(a.height, b.height, "box[{i}].height");
            assert_eq!(a.label, b.label, "box[{i}].label");
            assert_eq!(a.score, b.score, "box[{i}].score");
            assert_eq!(a.distance, b.distance, "box[{i}].distance");
            assert_eq!(a.speed, b.speed, "box[{i}].speed");
            assert_eq!(a.track_id, b.track_id, "box[{i}].track_id");
            assert_eq!(
                a.track_lifetime, b.track_lifetime,
                "box[{i}].track_lifetime"
            );
            assert_eq!(
                a.track_created.sec, b.track_created.sec,
                "box[{i}].track_created.sec"
            );
            assert_eq!(
                a.track_created.nanosec, b.track_created.nanosec,
                "box[{i}].track_created.nanosec"
            );
        }

        // Mask views.
        assert_eq!(masks_ref.len(), masks_new.len());
        for (i, (a, b)) in masks_ref.iter().zip(masks_new.iter()).enumerate() {
            assert_eq!(a.height, b.height, "mask[{i}].height");
            assert_eq!(a.width, b.width, "mask[{i}].width");
            assert_eq!(a.length, b.length, "mask[{i}].length");
            assert_eq!(a.encoding, b.encoding, "mask[{i}].encoding");
            assert_eq!(a.mask, b.mask, "mask[{i}].mask");
            assert_eq!(a.boxed, b.boxed, "mask[{i}].boxed");
        }
    }
}
