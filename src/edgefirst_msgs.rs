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

pub struct DmaBuffer<B> {
    buf: B,
    offsets: [usize; 1],
}

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
        Ok(Detect { offsets: [o0, o1], buf })
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
        Ok((Detect { offsets: [o0, o1], buf }, box_views))
    }
}

impl Detect<Vec<u8>> {
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
        Ok(Model { offsets: [o0, o1, o2], buf })
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
        Ok((Model { offsets: [o0, o1, o2], buf }, box_views, mask_views))
    }
}

impl Model<Vec<u8>> {
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
}

// ── Registry ────────────────────────────────────────────────────────

/// Check if a type name is supported by this module.
pub fn is_type_supported(type_name: &str) -> bool {
    matches!(
        type_name,
        "Box"
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
    )
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &[
        "edgefirst_msgs/msg/Box",
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
    ]
}

// SchemaType implementations
use crate::schema_registry::SchemaType;

impl SchemaType for Date {
    const SCHEMA_NAME: &'static str = "edgefirst_msgs/msg/Date";
}

#[cfg(test)]
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
        static BYTES: &[u8] =
            include_bytes!("../testdata/cdr/edgefirst_msgs/Detect_multi.cdr");

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
            assert_eq!(a.track_lifetime, b.track_lifetime, "box[{i}].track_lifetime");
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
        static BYTES: &[u8] =
            include_bytes!("../testdata/cdr/edgefirst_msgs/Model.cdr");

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
            assert_eq!(a.track_lifetime, b.track_lifetime, "box[{i}].track_lifetime");
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
