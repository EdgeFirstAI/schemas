// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! Foxglove message types for visualization.
//!
//! CdrFixed: `FoxglovePoint2`, `FoxgloveColor`, `FoxgloveCircleAnnotations`
//!
//! Buffer-backed: `FoxgloveCompressedVideo`, `FoxgloveTextAnnotation`
//! (`FoxgloveTextAnnotationView`), `FoxglovePointAnnotation`
//! (`FoxglovePointAnnotationView`), `FoxgloveImageAnnotation`

use crate::builtin_interfaces::Time;
use crate::cdr::*;
use crate::std_msgs::Header;

// ── CdrFixed types ──────────────────────────────────────────────────

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct FoxglovePoint2 {
    pub x: f64,
    pub y: f64,
}

impl CdrFixed for FoxglovePoint2 {
    const CDR_SIZE: usize = 16; // 2 x f64
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(FoxglovePoint2 {
            x: cursor.read_f64()?,
            y: cursor.read_f64()?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_f64(self.x);
        writer.write_f64(self.y);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_f64();
        sizer.size_f64();
    }
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct FoxgloveColor {
    pub r: f64,
    pub g: f64,
    pub b: f64,
    pub a: f64,
}

impl CdrFixed for FoxgloveColor {
    const CDR_SIZE: usize = 32; // 4 x f64
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(FoxgloveColor {
            r: cursor.read_f64()?,
            g: cursor.read_f64()?,
            b: cursor.read_f64()?,
            a: cursor.read_f64()?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_f64(self.r);
        writer.write_f64(self.g);
        writer.write_f64(self.b);
        writer.write_f64(self.a);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_f64();
        sizer.size_f64();
        sizer.size_f64();
        sizer.size_f64();
    }
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct FoxgloveCircleAnnotations {
    pub timestamp: Time,
    pub position: FoxglovePoint2,
    pub diameter: f64,
    pub thickness: f64,
    pub fill_color: FoxgloveColor,
    pub outline_color: FoxgloveColor,
}

impl CdrFixed for FoxgloveCircleAnnotations {
    const CDR_SIZE: usize = 104; // Time(8) + Point2(16) + 2*f64(16) + 2*Color(64)
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(FoxgloveCircleAnnotations {
            timestamp: Time::read_cdr(cursor)?,
            position: FoxglovePoint2::read_cdr(cursor)?,
            diameter: cursor.read_f64()?,
            thickness: cursor.read_f64()?,
            fill_color: FoxgloveColor::read_cdr(cursor)?,
            outline_color: FoxgloveColor::read_cdr(cursor)?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        self.timestamp.write_cdr(writer);
        self.position.write_cdr(writer);
        writer.write_f64(self.diameter);
        writer.write_f64(self.thickness);
        self.fill_color.write_cdr(writer);
        self.outline_color.write_cdr(writer);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        Time::size_cdr(sizer);
        FoxglovePoint2::size_cdr(sizer);
        sizer.size_f64();
        sizer.size_f64();
        FoxgloveColor::size_cdr(sizer);
        FoxgloveColor::size_cdr(sizer);
    }
}

// ── Constants ───────────────────────────────────────────────────────

pub mod point_annotation_type {
    pub const UNKNOWN: u8 = 0;
    pub const POINTS: u8 = 1;
    pub const LINE_LOOP: u8 = 2;
    pub const LINE_STRIP: u8 = 3;
    pub const LINE_LIST: u8 = 4;
}

// ── Buffer-backed types ─────────────────────────────────────────────

// ── FoxgloveCompressedVideo<B> — foxglove_msgs/msg/CompressedVideo ──
//
// CDR layout (NOTE: data comes BEFORE format in the struct):
//   4: stamp (8 bytes)
//  12: frame_id (string) → offsets[0]
//   ~: data (byte seq) → offsets[1]
//   ~: format (string) → offsets[2]

pub struct FoxgloveCompressedVideo<B> {
    buf: B,
    offsets: [usize; 3],
}

impl<B: AsRef<[u8]>> FoxgloveCompressedVideo<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        let _ = c.read_bytes()?; // data
        let o1 = c.offset();
        let _ = c.read_string()?; // format
        let o2 = c.offset();
        Ok(FoxgloveCompressedVideo {
            offsets: [o0, o1, o2],
            buf,
        })
    }

    /// Returns a `Header` view (re-parses CDR prefix; prefer `stamp()`/`frame_id()`).
    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }

    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }

    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }

    pub fn data(&self) -> &[u8] {
        rd_bytes(self.buf.as_ref(), self.offsets[0]).0
    }

    pub fn format(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[1]).0
    }

    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }

    pub fn cdr_size(&self) -> usize {
        self.buf.as_ref().len()
    }

    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl FoxgloveCompressedVideo<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use FoxgloveCompressedVideo::builder() for allocation-free buffer reuse; FoxgloveCompressedVideo::new will be removed in 4.0"
    )]
    pub fn new(stamp: Time, frame_id: &str, data: &[u8], format: &str) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_bytes(data.len());
        let o1 = sizer.offset();
        sizer.size_string(format);
        let o2 = sizer.offset();

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_bytes(data);
        w.write_string(format);
        w.finish()?;

        Ok(FoxgloveCompressedVideo {
            offsets: [o0, o1, o2],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `FoxgloveCompressedVideoBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> FoxgloveCompressedVideoBuilder<'a> {
        FoxgloveCompressedVideoBuilder::new()
    }
}

// ── FoxgloveCompressedVideoBuilder<'a> ──────────────────────────────

/// Builder for `FoxgloveCompressedVideo<Vec<u8>>` with buffer-reuse finalizers.
///
/// Strings use `Cow<'a, str>`; bulk `data` is borrowed for zero-copy input
/// semantics. All borrows must remain valid until `build()`,
/// `encode_into_vec()`, or `encode_into_slice()` is called.
pub struct FoxgloveCompressedVideoBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    data: &'a [u8],
    format: std::borrow::Cow<'a, str>,
}

impl<'a> Default for FoxgloveCompressedVideoBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            data: &[],
            format: std::borrow::Cow::Borrowed(""),
        }
    }
}

impl<'a> FoxgloveCompressedVideoBuilder<'a> {
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
    pub fn data(&mut self, d: &'a [u8]) -> &mut Self {
        self.data = d;
        self
    }
    pub fn format(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.format = s.into();
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        s.size_bytes(self.data.len());
        s.size_string(&self.format);
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_bytes(self.data);
        w.write_string(&self.format);
        w.finish()
    }

    /// Allocate a fresh `Vec<u8>` and return a fully-parsed
    /// `FoxgloveCompressedVideo<Vec<u8>>`.
    pub fn build(&self) -> Result<FoxgloveCompressedVideo<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        FoxgloveCompressedVideo::from_cdr(buf)
    }

    /// Serialize into the caller's `Vec<u8>`, resizing to exactly the encoded
    /// size. Reuses existing allocation when capacity suffices.
    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    /// Serialize into `buf` and return bytes written. Errors with
    /// `BufferTooShort` when `buf` is smaller than the required size;
    /// nothing is mutated in that case.
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

impl<B: AsRef<[u8]> + AsMut<[u8]>> FoxgloveCompressedVideo<B> {
    pub fn set_stamp(&mut self, t: Time) -> Result<(), CdrError> {
        let b = self.buf.as_mut();
        wr_i32(b, CDR_HEADER_SIZE, t.sec)?;
        wr_u32(b, CDR_HEADER_SIZE + 4, t.nanosec)?;
        Ok(())
    }
}

// ── FoxgloveTextAnnotation<B> — foxglove_msgs/msg/FoxgloveTextAnnotations
//
// CDR layout: timestamp(Time), position(FoxglovePoint2),
//   text(string) → offsets[0], font_size(f64),
//   text_color(FoxgloveColor), background_color(FoxgloveColor)

pub struct FoxgloveTextAnnotation<B> {
    buf: B,
    offsets: [usize; 1],
}

/// View of a FoxgloveTextAnnotations element within a CDR sequence.
pub struct FoxgloveTextAnnotationView<'a> {
    pub timestamp: Time,
    pub position: FoxglovePoint2,
    pub text: &'a str,
    pub font_size: f64,
    pub text_color: FoxgloveColor,
    pub background_color: FoxgloveColor,
}

fn scan_text_annotation<'a>(
    c: &mut CdrCursor<'a>,
) -> Result<FoxgloveTextAnnotationView<'a>, CdrError> {
    let timestamp = Time::read_cdr(c)?;
    let position = FoxglovePoint2::read_cdr(c)?;
    let text = c.read_string()?;
    let font_size = c.read_f64()?;
    let text_color = FoxgloveColor::read_cdr(c)?;
    let background_color = FoxgloveColor::read_cdr(c)?;
    Ok(FoxgloveTextAnnotationView {
        timestamp,
        position,
        text,
        font_size,
        text_color,
        background_color,
    })
}

fn write_text_annotation(w: &mut CdrWriter<'_>, t: &FoxgloveTextAnnotationView<'_>) {
    t.timestamp.write_cdr(w);
    t.position.write_cdr(w);
    w.write_string(t.text);
    w.write_f64(t.font_size);
    t.text_color.write_cdr(w);
    t.background_color.write_cdr(w);
}

fn size_text_annotation(s: &mut CdrSizer, text: &str) {
    Time::size_cdr(s);
    FoxglovePoint2::size_cdr(s);
    s.size_string(text);
    s.size_f64();
    FoxgloveColor::size_cdr(s);
    FoxgloveColor::size_cdr(s);
}

impl<B: AsRef<[u8]>> FoxgloveTextAnnotation<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let mut c = CdrCursor::new(buf.as_ref())?;
        Time::read_cdr(&mut c)?;
        FoxglovePoint2::read_cdr(&mut c)?;
        let _ = c.read_string()?;
        let o0 = c.offset();
        c.read_f64()?;
        FoxgloveColor::read_cdr(&mut c)?;
        FoxgloveColor::read_cdr(&mut c)?;
        Ok(FoxgloveTextAnnotation { offsets: [o0], buf })
    }

    pub fn timestamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }

    pub fn position(&self) -> FoxglovePoint2 {
        let mut c = CdrCursor::resume(self.buf.as_ref(), CDR_HEADER_SIZE + 8);
        FoxglovePoint2::read_cdr(&mut c).expect("point2 field validated during from_cdr")
    }

    pub fn text(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 24).0
    }

    pub fn font_size(&self) -> f64 {
        let p = cdr_align(self.offsets[0], 8);
        rd_f64(self.buf.as_ref(), p)
    }

    pub fn text_color(&self) -> FoxgloveColor {
        let p = cdr_align(self.offsets[0], 8) + 8;
        let mut c = CdrCursor::resume(self.buf.as_ref(), p);
        FoxgloveColor::read_cdr(&mut c).expect("color field validated during from_cdr")
    }

    pub fn background_color(&self) -> FoxgloveColor {
        let p = cdr_align(self.offsets[0], 8) + 40;
        let mut c = CdrCursor::resume(self.buf.as_ref(), p);
        FoxgloveColor::read_cdr(&mut c).expect("color field validated during from_cdr")
    }

    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl FoxgloveTextAnnotation<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use FoxgloveTextAnnotation::builder() for allocation-free buffer reuse; FoxgloveTextAnnotation::new will be removed in 4.0"
    )]
    pub fn new(
        timestamp: Time,
        position: FoxglovePoint2,
        text: &str,
        font_size: f64,
        text_color: FoxgloveColor,
        background_color: FoxgloveColor,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        FoxglovePoint2::size_cdr(&mut sizer);
        sizer.size_string(text);
        let o0 = sizer.offset();
        sizer.size_f64();
        FoxgloveColor::size_cdr(&mut sizer);
        FoxgloveColor::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        timestamp.write_cdr(&mut w);
        position.write_cdr(&mut w);
        w.write_string(text);
        w.write_f64(font_size);
        text_color.write_cdr(&mut w);
        background_color.write_cdr(&mut w);
        w.finish()?;

        Ok(FoxgloveTextAnnotation { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `FoxgloveTextAnnotationBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> FoxgloveTextAnnotationBuilder<'a> {
        FoxgloveTextAnnotationBuilder::new()
    }
}

// ── FoxgloveTextAnnotationBuilder<'a> ───────────────────────────────

/// Builder for `FoxgloveTextAnnotation<Vec<u8>>` with buffer-reuse finalizers.
pub struct FoxgloveTextAnnotationBuilder<'a> {
    timestamp: Time,
    position: FoxglovePoint2,
    text: std::borrow::Cow<'a, str>,
    font_size: f64,
    text_color: FoxgloveColor,
    background_color: FoxgloveColor,
}

impl<'a> Default for FoxgloveTextAnnotationBuilder<'a> {
    fn default() -> Self {
        Self {
            timestamp: Time { sec: 0, nanosec: 0 },
            position: FoxglovePoint2 { x: 0.0, y: 0.0 },
            text: std::borrow::Cow::Borrowed(""),
            font_size: 0.0,
            text_color: FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.0,
            },
            background_color: FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.0,
            },
        }
    }
}

impl<'a> FoxgloveTextAnnotationBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn timestamp(&mut self, t: Time) -> &mut Self {
        self.timestamp = t;
        self
    }
    pub fn position(&mut self, p: FoxglovePoint2) -> &mut Self {
        self.position = p;
        self
    }
    pub fn text(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.text = s.into();
        self
    }
    pub fn font_size(&mut self, v: f64) -> &mut Self {
        self.font_size = v;
        self
    }
    pub fn text_color(&mut self, c: FoxgloveColor) -> &mut Self {
        self.text_color = c;
        self
    }
    pub fn background_color(&mut self, c: FoxgloveColor) -> &mut Self {
        self.background_color = c;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        FoxglovePoint2::size_cdr(&mut s);
        s.size_string(&self.text);
        s.size_f64();
        FoxgloveColor::size_cdr(&mut s);
        FoxgloveColor::size_cdr(&mut s);
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.timestamp.write_cdr(&mut w);
        self.position.write_cdr(&mut w);
        w.write_string(&self.text);
        w.write_f64(self.font_size);
        self.text_color.write_cdr(&mut w);
        self.background_color.write_cdr(&mut w);
        w.finish()
    }

    pub fn build(&self) -> Result<FoxgloveTextAnnotation<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        FoxgloveTextAnnotation::from_cdr(buf)
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

// ── FoxglovePointAnnotation<B> — foxglove_msgs/msg/FoxglovePointAnnotations
//
// CDR layout: timestamp(Time), type_(u8),
//   points(Vec<FoxglovePoint2>) → offsets[0],
//   outline_color(FoxgloveColor),
//   outline_colors(Vec<FoxgloveColor>) → offsets[1],
//   fill_color(FoxgloveColor), thickness(f64)

pub struct FoxglovePointAnnotation<B> {
    buf: B,
    offsets: [usize; 2],
}

/// View of a FoxglovePointAnnotations element within a CDR sequence.
pub struct FoxglovePointAnnotationView {
    pub timestamp: Time,
    pub type_: u8,
    pub points: Vec<FoxglovePoint2>,
    pub outline_color: FoxgloveColor,
    pub outline_colors: Vec<FoxgloveColor>,
    pub fill_color: FoxgloveColor,
    pub thickness: f64,
}

fn scan_point_annotation(c: &mut CdrCursor<'_>) -> Result<FoxglovePointAnnotationView, CdrError> {
    let timestamp = Time::read_cdr(c)?;
    let type_ = c.read_u8()?;
    let pts_count = c.read_u32()? as usize;
    let mut points = Vec::with_capacity(pts_count);
    for _ in 0..pts_count {
        points.push(FoxglovePoint2::read_cdr(c)?);
    }
    let outline_color = FoxgloveColor::read_cdr(c)?;
    let oc_count = c.read_u32()? as usize;
    let mut outline_colors = Vec::with_capacity(oc_count);
    for _ in 0..oc_count {
        outline_colors.push(FoxgloveColor::read_cdr(c)?);
    }
    let fill_color = FoxgloveColor::read_cdr(c)?;
    let thickness = c.read_f64()?;
    Ok(FoxglovePointAnnotationView {
        timestamp,
        type_,
        points,
        outline_color,
        outline_colors,
        fill_color,
        thickness,
    })
}

fn write_point_annotation(w: &mut CdrWriter<'_>, p: &FoxglovePointAnnotationView) {
    p.timestamp.write_cdr(w);
    w.write_u8(p.type_);
    w.write_u32(p.points.len() as u32);
    for pt in &p.points {
        pt.write_cdr(w);
    }
    p.outline_color.write_cdr(w);
    w.write_u32(p.outline_colors.len() as u32);
    for oc in &p.outline_colors {
        oc.write_cdr(w);
    }
    p.fill_color.write_cdr(w);
    w.write_f64(p.thickness);
}

fn size_point_annotation(s: &mut CdrSizer, p: &FoxglovePointAnnotationView) {
    Time::size_cdr(s);
    s.size_u8();
    s.size_u32();
    for _ in 0..p.points.len() {
        FoxglovePoint2::size_cdr(s);
    }
    FoxgloveColor::size_cdr(s);
    s.size_u32();
    for _ in 0..p.outline_colors.len() {
        FoxgloveColor::size_cdr(s);
    }
    FoxgloveColor::size_cdr(s);
    s.size_f64();
}

impl<B: AsRef<[u8]>> FoxglovePointAnnotation<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let mut c = CdrCursor::new(buf.as_ref())?;
        Time::read_cdr(&mut c)?;
        c.read_u8()?;
        let pts_count = c.read_u32()? as usize;
        for _ in 0..pts_count {
            FoxglovePoint2::read_cdr(&mut c)?;
        }
        let o0 = c.offset();
        FoxgloveColor::read_cdr(&mut c)?;
        let oc_count = c.read_u32()? as usize;
        for _ in 0..oc_count {
            FoxgloveColor::read_cdr(&mut c)?;
        }
        let o1 = c.offset();
        FoxgloveColor::read_cdr(&mut c)?;
        c.read_f64()?;
        Ok(FoxglovePointAnnotation {
            offsets: [o0, o1],
            buf,
        })
    }

    pub fn timestamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }

    pub fn type_(&self) -> u8 {
        rd_u8(self.buf.as_ref(), CDR_HEADER_SIZE + 8)
    }

    pub fn points(&self) -> Vec<FoxglovePoint2> {
        let b = self.buf.as_ref();
        let p = align(CDR_HEADER_SIZE + 9, 4);
        let count = rd_u32(b, p) as usize;
        let mut c = CdrCursor::resume(b, p + 4);
        (0..count)
            .map(|_| {
                FoxglovePoint2::read_cdr(&mut c).expect("point2 field validated during from_cdr")
            })
            .collect()
    }

    pub fn outline_color(&self) -> FoxgloveColor {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        FoxgloveColor::read_cdr(&mut c).expect("color field validated during from_cdr")
    }

    pub fn outline_colors(&self) -> Vec<FoxgloveColor> {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        FoxgloveColor::read_cdr(&mut c).expect("color field validated during from_cdr"); // skip outline_color
        let count = c
            .read_u32()
            .expect("outline colors count validated during from_cdr") as usize;
        (0..count)
            .map(|_| {
                FoxgloveColor::read_cdr(&mut c).expect("color field validated during from_cdr")
            })
            .collect()
    }

    pub fn fill_color(&self) -> FoxgloveColor {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[1]);
        FoxgloveColor::read_cdr(&mut c).expect("color field validated during from_cdr")
    }

    pub fn thickness(&self) -> f64 {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[1]);
        FoxgloveColor::read_cdr(&mut c).expect("color field validated during from_cdr"); // skip fill_color
        c.read_f64()
            .expect("thickness field validated during from_cdr")
    }

    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl FoxglovePointAnnotation<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use FoxglovePointAnnotation::builder() for allocation-free buffer reuse; FoxglovePointAnnotation::new will be removed in 4.0"
    )]
    pub fn new(
        timestamp: Time,
        type_: u8,
        points: &[FoxglovePoint2],
        outline_color: FoxgloveColor,
        outline_colors: &[FoxgloveColor],
        fill_color: FoxgloveColor,
        thickness: f64,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_u8();
        sizer.size_u32();
        for _ in 0..points.len() {
            FoxglovePoint2::size_cdr(&mut sizer);
        }
        let o0 = sizer.offset();
        FoxgloveColor::size_cdr(&mut sizer);
        sizer.size_u32();
        for _ in 0..outline_colors.len() {
            FoxgloveColor::size_cdr(&mut sizer);
        }
        let o1 = sizer.offset();
        FoxgloveColor::size_cdr(&mut sizer);
        sizer.size_f64();

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        timestamp.write_cdr(&mut w);
        w.write_u8(type_);
        w.write_u32(points.len() as u32);
        for pt in points {
            pt.write_cdr(&mut w);
        }
        outline_color.write_cdr(&mut w);
        w.write_u32(outline_colors.len() as u32);
        for oc in outline_colors {
            oc.write_cdr(&mut w);
        }
        fill_color.write_cdr(&mut w);
        w.write_f64(thickness);
        w.finish()?;

        Ok(FoxglovePointAnnotation {
            offsets: [o0, o1],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `FoxglovePointAnnotationBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> FoxglovePointAnnotationBuilder<'a> {
        FoxglovePointAnnotationBuilder::new()
    }
}

// ── FoxglovePointAnnotationBuilder<'a> ──────────────────────────────

/// Builder for `FoxglovePointAnnotation<Vec<u8>>` with buffer-reuse finalizers.
///
/// Point and color slices are borrowed from caller memory; the borrows must
/// remain valid until `build()`, `encode_into_vec()`, or
/// `encode_into_slice()` is called.
pub struct FoxglovePointAnnotationBuilder<'a> {
    timestamp: Time,
    type_: u8,
    points: &'a [FoxglovePoint2],
    outline_color: FoxgloveColor,
    outline_colors: &'a [FoxgloveColor],
    fill_color: FoxgloveColor,
    thickness: f64,
}

impl<'a> Default for FoxglovePointAnnotationBuilder<'a> {
    fn default() -> Self {
        Self {
            timestamp: Time { sec: 0, nanosec: 0 },
            type_: 0,
            points: &[],
            outline_color: FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.0,
            },
            outline_colors: &[],
            fill_color: FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.0,
            },
            thickness: 0.0,
        }
    }
}

impl<'a> FoxglovePointAnnotationBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn timestamp(&mut self, t: Time) -> &mut Self {
        self.timestamp = t;
        self
    }
    pub fn type_(&mut self, v: u8) -> &mut Self {
        self.type_ = v;
        self
    }
    pub fn points(&mut self, p: &'a [FoxglovePoint2]) -> &mut Self {
        self.points = p;
        self
    }
    pub fn outline_color(&mut self, c: FoxgloveColor) -> &mut Self {
        self.outline_color = c;
        self
    }
    pub fn outline_colors(&mut self, c: &'a [FoxgloveColor]) -> &mut Self {
        self.outline_colors = c;
        self
    }
    pub fn fill_color(&mut self, c: FoxgloveColor) -> &mut Self {
        self.fill_color = c;
        self
    }
    pub fn thickness(&mut self, v: f64) -> &mut Self {
        self.thickness = v;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_u8();
        s.size_u32();
        for _ in 0..self.points.len() {
            FoxglovePoint2::size_cdr(&mut s);
        }
        FoxgloveColor::size_cdr(&mut s);
        s.size_u32();
        for _ in 0..self.outline_colors.len() {
            FoxgloveColor::size_cdr(&mut s);
        }
        FoxgloveColor::size_cdr(&mut s);
        s.size_f64();
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.timestamp.write_cdr(&mut w);
        w.write_u8(self.type_);
        w.write_u32(self.points.len() as u32);
        for pt in self.points {
            pt.write_cdr(&mut w);
        }
        self.outline_color.write_cdr(&mut w);
        w.write_u32(self.outline_colors.len() as u32);
        for oc in self.outline_colors {
            oc.write_cdr(&mut w);
        }
        self.fill_color.write_cdr(&mut w);
        w.write_f64(self.thickness);
        w.finish()
    }

    pub fn build(&self) -> Result<FoxglovePointAnnotation<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        FoxglovePointAnnotation::from_cdr(buf)
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

// ── FoxgloveImageAnnotation<B> — foxglove_msgs/msg/FoxgloveImageAnnotations
//
// CDR layout:
//   circles(Vec<FoxgloveCircleAnnotations>) → offsets[0],
//   points(Vec<FoxglovePointAnnotations>) → offsets[1],
//   texts(Vec<FoxgloveTextAnnotations>)

pub struct FoxgloveImageAnnotation<B> {
    buf: B,
    offsets: [usize; 2],
}

impl<B: AsRef<[u8]>> FoxgloveImageAnnotation<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let mut c = CdrCursor::new(buf.as_ref())?;
        let raw_circ = c.read_u32()?;
        let circ_count = c.check_seq_count(raw_circ, 40)?;
        for _ in 0..circ_count {
            FoxgloveCircleAnnotations::read_cdr(&mut c)?;
        }
        let o0 = c.offset();
        let raw_pts = c.read_u32()?;
        let pts_count = c.check_seq_count(raw_pts, 12)?;
        for _ in 0..pts_count {
            scan_point_annotation(&mut c)?;
        }
        let o1 = c.offset();
        let raw_txt = c.read_u32()?;
        let txt_count = c.check_seq_count(raw_txt, 13)?;
        for _ in 0..txt_count {
            scan_text_annotation(&mut c)?;
        }
        Ok(FoxgloveImageAnnotation {
            offsets: [o0, o1],
            buf,
        })
    }

    pub fn circles(&self) -> Vec<FoxgloveCircleAnnotations> {
        let b = self.buf.as_ref();
        let count = rd_u32(b, CDR_HEADER_SIZE) as usize;
        let mut c = CdrCursor::resume(b, CDR_HEADER_SIZE + 4);
        (0..count)
            .map(|_| {
                FoxgloveCircleAnnotations::read_cdr(&mut c)
                    .expect("circle elements validated during from_cdr")
            })
            .collect()
    }

    pub fn points(&self) -> Vec<FoxglovePointAnnotationView> {
        let b = self.buf.as_ref();
        let p = align(self.offsets[0], 4);
        let count = rd_u32(b, p) as usize;
        let mut c = CdrCursor::resume(b, p + 4);
        (0..count)
            .map(|_| {
                scan_point_annotation(&mut c)
                    .expect("point annotation elements validated during from_cdr")
            })
            .collect()
    }

    pub fn texts(&self) -> Vec<FoxgloveTextAnnotationView<'_>> {
        let b = self.buf.as_ref();
        let p = align(self.offsets[1], 4);
        let count = rd_u32(b, p) as usize;
        let mut c = CdrCursor::resume(b, p + 4);
        (0..count)
            .map(|_| {
                scan_text_annotation(&mut c)
                    .expect("text annotation elements validated during from_cdr")
            })
            .collect()
    }

    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl FoxgloveImageAnnotation<Vec<u8>> {
    #[deprecated(
        since = "3.2.0",
        note = "use FoxgloveImageAnnotation::builder() for allocation-free buffer reuse; FoxgloveImageAnnotation::new will be removed in 4.0"
    )]
    pub fn new(
        circles: &[FoxgloveCircleAnnotations],
        points: &[FoxglovePointAnnotationView],
        texts: &[FoxgloveTextAnnotationView<'_>],
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        sizer.size_u32();
        for _ in 0..circles.len() {
            FoxgloveCircleAnnotations::size_cdr(&mut sizer);
        }
        let o0 = sizer.offset();
        sizer.size_u32();
        for p in points {
            size_point_annotation(&mut sizer, p);
        }
        let o1 = sizer.offset();
        sizer.size_u32();
        for t in texts {
            size_text_annotation(&mut sizer, t.text);
        }

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        w.write_u32(circles.len() as u32);
        for c in circles {
            c.write_cdr(&mut w);
        }
        w.write_u32(points.len() as u32);
        for p in points {
            write_point_annotation(&mut w, p);
        }
        w.write_u32(texts.len() as u32);
        for t in texts {
            write_text_annotation(&mut w, t);
        }
        w.finish()?;

        Ok(FoxgloveImageAnnotation {
            offsets: [o0, o1],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `FoxgloveImageAnnotationBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> FoxgloveImageAnnotationBuilder<'a> {
        FoxgloveImageAnnotationBuilder::new()
    }
}

// ── FoxgloveImageAnnotationBuilder<'a> ──────────────────────────────

/// Builder for `FoxgloveImageAnnotation<Vec<u8>>` with buffer-reuse finalizers.
///
/// All slices (circles, points, texts) are borrowed from caller memory. The
/// `points` elements own their inner `Vec<FoxglovePoint2>`/`Vec<FoxgloveColor>`;
/// the `texts` elements borrow their string payloads. All borrows must remain
/// valid until `build()`, `encode_into_vec()`, or `encode_into_slice()` is
/// called.
#[derive(Default)]
pub struct FoxgloveImageAnnotationBuilder<'a> {
    circles: &'a [FoxgloveCircleAnnotations],
    points: &'a [FoxglovePointAnnotationView],
    texts: &'a [FoxgloveTextAnnotationView<'a>],
}

impl<'a> FoxgloveImageAnnotationBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn circles(&mut self, c: &'a [FoxgloveCircleAnnotations]) -> &mut Self {
        self.circles = c;
        self
    }
    pub fn points(&mut self, p: &'a [FoxglovePointAnnotationView]) -> &mut Self {
        self.points = p;
        self
    }
    pub fn texts(&mut self, t: &'a [FoxgloveTextAnnotationView<'a>]) -> &mut Self {
        self.texts = t;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        s.size_u32();
        for _ in 0..self.circles.len() {
            FoxgloveCircleAnnotations::size_cdr(&mut s);
        }
        s.size_u32();
        for p in self.points {
            size_point_annotation(&mut s, p);
        }
        s.size_u32();
        for t in self.texts {
            size_text_annotation(&mut s, t.text);
        }
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        w.write_u32(self.circles.len() as u32);
        for c in self.circles {
            c.write_cdr(&mut w);
        }
        w.write_u32(self.points.len() as u32);
        for p in self.points {
            write_point_annotation(&mut w, p);
        }
        w.write_u32(self.texts.len() as u32);
        for t in self.texts {
            write_text_annotation(&mut w, t);
        }
        w.finish()
    }

    pub fn build(&self) -> Result<FoxgloveImageAnnotation<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        FoxgloveImageAnnotation::from_cdr(buf)
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
    matches!(type_name, "CompressedVideo")
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &["foxglove_msgs/msg/CompressedVideo"]
}

// SchemaType implementations
use crate::schema_registry::SchemaType;

impl SchemaType for FoxglovePoint2 {
    const SCHEMA_NAME: &'static str = "foxglove_msgs/msg/FoxglovePoint2";
}

impl SchemaType for FoxgloveColor {
    const SCHEMA_NAME: &'static str = "foxglove_msgs/msg/FoxgloveColor";
}

impl SchemaType for FoxgloveCircleAnnotations {
    const SCHEMA_NAME: &'static str = "foxglove_msgs/msg/FoxgloveCircleAnnotations";
}

#[allow(deprecated)]
#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::cdr::{decode_fixed, encode_fixed};

    #[test]
    fn foxglove_color_roundtrip() {
        let cases = [
            (0.0, 0.0, 0.0, 0.0, "transparent black"),
            (1.0, 1.0, 1.0, 1.0, "opaque white"),
            (1.0, 0.0, 0.0, 1.0, "red"),
            (0.5, 0.5, 0.5, 0.75, "gray"),
        ];
        for (r, g, b, a, name) in cases {
            let color = FoxgloveColor { r, g, b, a };
            let bytes = encode_fixed(&color).unwrap();
            let decoded: FoxgloveColor = decode_fixed(&bytes).unwrap();
            assert_eq!(color, decoded, "failed for case: {}", name);
        }
    }

    #[test]
    fn foxglove_point2_roundtrip() {
        let cases = [
            (0.0, 0.0, "origin"),
            (100.0, 200.0, "positive"),
            (-50.0, -75.0, "negative"),
        ];
        for (x, y, name) in cases {
            let point = FoxglovePoint2 { x, y };
            let bytes = encode_fixed(&point).unwrap();
            let decoded: FoxglovePoint2 = decode_fixed(&bytes).unwrap();
            assert_eq!(point, decoded, "failed for case: {}", name);
        }
    }

    #[test]
    fn foxglove_circle_annotations_roundtrip() {
        let circle = FoxgloveCircleAnnotations {
            timestamp: Time::new(100, 0),
            position: FoxglovePoint2 { x: 320.0, y: 240.0 },
            diameter: 50.0,
            thickness: 2.0,
            fill_color: FoxgloveColor {
                r: 1.0,
                g: 0.0,
                b: 0.0,
                a: 0.5,
            },
            outline_color: FoxgloveColor {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 1.0,
            },
        };
        let bytes = encode_fixed(&circle).unwrap();
        let decoded: FoxgloveCircleAnnotations = decode_fixed(&bytes).unwrap();
        assert_eq!(circle, decoded);
    }

    #[test]
    fn foxglove_compressed_video_roundtrip() {
        let video = FoxgloveCompressedVideo::new(
            Time::new(100, 500_000_000),
            "camera",
            &[0x00, 0x00, 0x00, 0x01, 0x67, 0x42],
            "h264",
        )
        .unwrap();
        assert_eq!(video.stamp(), Time::new(100, 500_000_000));
        assert_eq!(video.frame_id(), "camera");
        assert_eq!(video.data(), &[0x00, 0x00, 0x00, 0x01, 0x67, 0x42]);
        assert_eq!(video.format(), "h264");

        let bytes = video.to_cdr();
        let decoded = FoxgloveCompressedVideo::from_cdr(bytes).unwrap();
        assert_eq!(decoded.format(), "h264");
        assert_eq!(decoded.data().len(), 6);
    }

    #[test]
    fn foxglove_text_annotation_roundtrip() {
        let text = FoxgloveTextAnnotation::new(
            Time::new(100, 0),
            FoxglovePoint2 { x: 50.0, y: 50.0 },
            "Detection: car (98%)",
            14.0,
            FoxgloveColor {
                r: 1.0,
                g: 1.0,
                b: 1.0,
                a: 1.0,
            },
            FoxgloveColor {
                r: 0.0,
                g: 0.0,
                b: 0.0,
                a: 0.7,
            },
        )
        .unwrap();
        assert_eq!(text.text(), "Detection: car (98%)");
        assert_eq!(text.font_size(), 14.0);

        let bytes = text.to_cdr();
        let decoded = FoxgloveTextAnnotation::from_cdr(bytes).unwrap();
        assert_eq!(decoded.text(), "Detection: car (98%)");
    }

    #[test]
    fn foxglove_point_annotation_roundtrip() {
        let pa = FoxglovePointAnnotation::new(
            Time::new(100, 0),
            point_annotation_type::LINE_LOOP,
            &[
                FoxglovePoint2 { x: 0.0, y: 0.0 },
                FoxglovePoint2 { x: 100.0, y: 0.0 },
                FoxglovePoint2 { x: 100.0, y: 100.0 },
            ],
            FoxgloveColor {
                r: 0.0,
                g: 1.0,
                b: 0.0,
                a: 1.0,
            },
            &[],
            FoxgloveColor {
                r: 0.0,
                g: 0.5,
                b: 0.0,
                a: 0.3,
            },
            3.0,
        )
        .unwrap();
        assert_eq!(pa.type_(), point_annotation_type::LINE_LOOP);
        assert_eq!(pa.points().len(), 3);
        assert_eq!(pa.thickness(), 3.0);

        let bytes = pa.to_cdr();
        let decoded = FoxglovePointAnnotation::from_cdr(bytes).unwrap();
        assert_eq!(decoded.points().len(), 3);
    }

    #[test]
    fn foxglove_image_annotation_roundtrip() {
        let ia = FoxgloveImageAnnotation::new(&[], &[], &[]).unwrap();
        assert_eq!(ia.circles().len(), 0);
        assert_eq!(ia.points().len(), 0);
        assert_eq!(ia.texts().len(), 0);

        let bytes = ia.to_cdr();
        let decoded = FoxgloveImageAnnotation::from_cdr(bytes).unwrap();
        assert_eq!(decoded.circles().len(), 0);
    }
}
