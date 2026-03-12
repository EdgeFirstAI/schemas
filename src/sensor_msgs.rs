// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! ROS 2 `sensor_msgs` message types.
//!
//! CdrFixed: `NavSatStatus`, `RegionOfInterest`
//!
//! Buffer-backed: `Image`, `CompressedImage`, `Imu`, `NavSatFix`,
//! `PointCloud2`, `PointField` (with `PointFieldView`), `CameraInfo`

use crate::builtin_interfaces::Time;
use crate::cdr::*;
use crate::geometry_msgs::{Quaternion, Vector3};
use crate::std_msgs::Header;

// ── CdrFixed types ──────────────────────────────────────────────────

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct NavSatStatus {
    pub status: i8,
    pub service: u16,
}

impl CdrFixed for NavSatStatus {
    const CDR_SIZE: usize = 4; // i8(1) + pad(1) + u16(2)
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        let status = cursor.read_i8()?;
        let service = cursor.read_u16()?;
        Ok(NavSatStatus { status, service })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_i8(self.status);
        writer.write_u16(self.service);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_i8();
        sizer.size_u16();
    }
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct RegionOfInterest {
    pub x_offset: u32,
    pub y_offset: u32,
    pub height: u32,
    pub width: u32,
    pub do_rectify: bool,
}

impl CdrFixed for RegionOfInterest {
    const CDR_SIZE: usize = 17; // 4 x u32(16) + bool(1)
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(RegionOfInterest {
            x_offset: cursor.read_u32()?,
            y_offset: cursor.read_u32()?,
            height: cursor.read_u32()?,
            width: cursor.read_u32()?,
            do_rectify: cursor.read_bool()?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_u32(self.x_offset);
        writer.write_u32(self.y_offset);
        writer.write_u32(self.height);
        writer.write_u32(self.width);
        writer.write_bool(self.do_rectify);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_u32();
        sizer.size_u32();
        sizer.size_u32();
        sizer.size_u32();
        sizer.size_bool();
    }
}

// ── Helper arrays ───────────────────────────────────────────────────

fn read_f64_array9(c: &mut CdrCursor<'_>) -> Result<[f64; 9], CdrError> {
    Ok([
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
    ])
}

fn write_f64_array9(w: &mut CdrWriter<'_>, a: &[f64; 9]) {
    for v in a {
        w.write_f64(*v);
    }
}

fn size_f64_array9(s: &mut CdrSizer) {
    for _ in 0..9 {
        s.size_f64();
    }
}

fn read_f64_array12(c: &mut CdrCursor<'_>) -> Result<[f64; 12], CdrError> {
    Ok([
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
        c.read_f64()?,
    ])
}

fn write_f64_array12(w: &mut CdrWriter<'_>, a: &[f64; 12]) {
    for v in a {
        w.write_f64(*v);
    }
}

fn size_f64_array12(s: &mut CdrSizer) {
    for _ in 0..12 {
        s.size_f64();
    }
}

// ── PointField helpers ──────────────────────────────────────────────

/// Zero-copy view of a PointField element within a CDR sequence.
pub struct PointFieldView<'a> {
    pub name: &'a str,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

fn scan_point_field_element<'a>(c: &mut CdrCursor<'a>) -> Result<PointFieldView<'a>, CdrError> {
    let name = c.read_string()?;
    let offset = c.read_u32()?;
    let datatype = c.read_u8()?;
    let count = c.read_u32()?;
    Ok(PointFieldView {
        name,
        offset,
        datatype,
        count,
    })
}

fn write_point_field_element(w: &mut CdrWriter<'_>, f: &PointFieldView<'_>) {
    w.write_string(f.name);
    w.write_u32(f.offset);
    w.write_u8(f.datatype);
    w.write_u32(f.count);
}

fn size_point_field_element(s: &mut CdrSizer, name: &str) {
    s.size_string(name);
    s.size_u32();
    s.size_u8();
    s.size_u32();
}

/// Non-allocating iterator over PointField elements in a PointCloud2.
pub struct PointFieldIter<'a> {
    cursor: CdrCursor<'a>,
    remaining: usize,
}

impl<'a> Iterator for PointFieldIter<'a> {
    type Item = PointFieldView<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }
        self.remaining -= 1;
        Some(
            scan_point_field_element(&mut self.cursor)
                .expect("point field elements validated during from_cdr"),
        )
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}

impl ExactSizeIterator for PointFieldIter<'_> {}

// ── Buffer-backed types ─────────────────────────────────────────────

// ── CompressedImage<B> ──────────────────────────────────────────────
//
// CDR layout:
//   4: stamp (8 bytes)
//  12: frame_id (string) → offsets[0]
//   ~: format (string) → offsets[1]
//   ~: data (byte seq) → offsets[2]

pub struct CompressedImage<B> {
    buf: B,
    offsets: [usize; 3],
}

impl<B: AsRef<[u8]>> CompressedImage<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        let _ = c.read_string()?; // format
        let o1 = c.offset();
        let _ = c.read_bytes()?; // data
        let o2 = c.offset();
        Ok(CompressedImage {
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
    pub fn format(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[0]).0
    }
    pub fn data(&self) -> &[u8] {
        rd_bytes(self.buf.as_ref(), self.offsets[1]).0
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

impl CompressedImage<Vec<u8>> {
    pub fn new(stamp: Time, frame_id: &str, format: &str, data: &[u8]) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_string(format);
        let o1 = sizer.offset();
        sizer.size_bytes(data.len());
        let o2 = sizer.offset();

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_string(format);
        w.write_bytes(data);
        w.finish()?;

        Ok(CompressedImage {
            offsets: [o0, o1, o2],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> CompressedImage<B> {
    pub fn set_stamp(&mut self, t: Time) -> Result<(), CdrError> {
        let b = self.buf.as_mut();
        wr_i32(b, CDR_HEADER_SIZE, t.sec)?;
        wr_u32(b, CDR_HEADER_SIZE + 4, t.nanosec)
    }
}

// ── Image<B> ────────────────────────────────────────────────────────
//
// CDR layout:
//   4: stamp (8 bytes)
//  12: frame_id (string) → offsets[0]
//   ~: height (u32), width (u32)
//   ~: encoding (string) → offsets[1]
//   ~: is_bigendian (u8)
//   ~: step (u32)
//   ~: data (byte seq) → offsets[2]

pub struct Image<B> {
    buf: B,
    offsets: [usize; 3],
}

impl<B: AsRef<[u8]>> Image<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        let _ = c.read_u32()?; // height
        let _ = c.read_u32()?; // width
        let _ = c.read_string()?; // encoding
        let o1 = c.offset();
        let _ = c.read_u8()?; // is_bigendian
        let _ = c.read_u32()?; // step
        let _ = c.read_bytes()?; // data
        let o2 = c.offset();
        Ok(Image {
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

    pub fn height(&self) -> u32 {
        let p = align(self.offsets[0], 4);
        rd_u32(self.buf.as_ref(), p)
    }

    pub fn width(&self) -> u32 {
        let p = align(self.offsets[0], 4) + 4;
        rd_u32(self.buf.as_ref(), p)
    }

    pub fn encoding(&self) -> &str {
        let p = align(self.offsets[0], 4) + 8;
        rd_string(self.buf.as_ref(), p).0
    }

    pub fn is_bigendian(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[1])
    }

    pub fn step(&self) -> u32 {
        let p = align(self.offsets[1] + 1, 4);
        rd_u32(self.buf.as_ref(), p)
    }

    pub fn data(&self) -> &[u8] {
        let p = align(self.offsets[1] + 1, 4) + 4;
        rd_bytes(self.buf.as_ref(), p).0
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

impl Image<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        height: u32,
        width: u32,
        encoding: &str,
        is_bigendian: u8,
        step: u32,
        data: &[u8],
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_u32(); // height
        sizer.size_u32(); // width
        sizer.size_string(encoding);
        let o1 = sizer.offset();
        sizer.size_u8(); // is_bigendian
        sizer.size_u32(); // step
        sizer.size_bytes(data.len());
        let o2 = sizer.offset();

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_u32(height);
        w.write_u32(width);
        w.write_string(encoding);
        w.write_u8(is_bigendian);
        w.write_u32(step);
        w.write_bytes(data);
        w.finish()?;

        Ok(Image {
            offsets: [o0, o1, o2],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> Image<B> {
    pub fn set_stamp(&mut self, t: Time) -> Result<(), CdrError> {
        let b = self.buf.as_mut();
        wr_i32(b, CDR_HEADER_SIZE, t.sec)?;
        wr_u32(b, CDR_HEADER_SIZE + 4, t.nanosec)
    }

    pub fn set_height(&mut self, h: u32) -> Result<(), CdrError> {
        let p = align(self.offsets[0], 4);
        wr_u32(self.buf.as_mut(), p, h)
    }

    pub fn set_width(&mut self, w: u32) -> Result<(), CdrError> {
        let p = align(self.offsets[0], 4) + 4;
        wr_u32(self.buf.as_mut(), p, w)
    }

    pub fn set_is_bigendian(&mut self, v: u8) -> Result<(), CdrError> {
        wr_u8(self.buf.as_mut(), self.offsets[1], v)
    }

    pub fn set_step(&mut self, v: u32) -> Result<(), CdrError> {
        let p = align(self.offsets[1] + 1, 4);
        wr_u32(self.buf.as_mut(), p, v)
    }
}

// ── Imu<B> ──────────────────────────────────────────────────────────
//
// CDR layout: Header → offsets[0], then:
//   Quaternion(32) + [f64;9](72) + Vector3(24) + [f64;9](72)
//   + Vector3(24) + [f64;9](72) = 296 bytes fixed payload

pub struct Imu<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B: AsRef<[u8]>> Imu<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        Quaternion::read_cdr(&mut c)?;
        read_f64_array9(&mut c)?;
        Vector3::read_cdr(&mut c)?;
        read_f64_array9(&mut c)?;
        Vector3::read_cdr(&mut c)?;
        read_f64_array9(&mut c)?;
        Ok(Imu { offsets: [o0], buf })
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

    // Imu fixed layout after orientation (Quaternion=32):
    //   orientation_cov[9](72), angular_vel(24), angular_vel_cov[9](72),
    //   linear_acc(24), linear_acc_cov[9](72)
    fn fixed_base(&self) -> usize {
        cdr_align(self.offsets[0], 8)
    }

    pub fn orientation(&self) -> Quaternion {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        Quaternion::read_cdr(&mut c).expect("orientation field validated during from_cdr")
    }

    pub fn orientation_covariance(&self) -> [f64; 9] {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.fixed_base() + 32);
        read_f64_array9(&mut c).expect("covariance field validated during from_cdr")
    }

    pub fn angular_velocity(&self) -> Vector3 {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.fixed_base() + 104);
        Vector3::read_cdr(&mut c).expect("vector3 field validated during from_cdr")
    }

    pub fn angular_velocity_covariance(&self) -> [f64; 9] {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.fixed_base() + 128);
        read_f64_array9(&mut c).expect("covariance field validated during from_cdr")
    }

    pub fn linear_acceleration(&self) -> Vector3 {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.fixed_base() + 200);
        Vector3::read_cdr(&mut c).expect("vector3 field validated during from_cdr")
    }

    pub fn linear_acceleration_covariance(&self) -> [f64; 9] {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.fixed_base() + 224);
        read_f64_array9(&mut c).expect("covariance field validated during from_cdr")
    }

    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl Imu<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        orientation: Quaternion,
        orientation_covariance: [f64; 9],
        angular_velocity: Vector3,
        angular_velocity_covariance: [f64; 9],
        linear_acceleration: Vector3,
        linear_acceleration_covariance: [f64; 9],
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        Quaternion::size_cdr(&mut sizer);
        size_f64_array9(&mut sizer);
        Vector3::size_cdr(&mut sizer);
        size_f64_array9(&mut sizer);
        Vector3::size_cdr(&mut sizer);
        size_f64_array9(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        orientation.write_cdr(&mut w);
        write_f64_array9(&mut w, &orientation_covariance);
        angular_velocity.write_cdr(&mut w);
        write_f64_array9(&mut w, &angular_velocity_covariance);
        linear_acceleration.write_cdr(&mut w);
        write_f64_array9(&mut w, &linear_acceleration_covariance);
        w.finish()?;

        Ok(Imu { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── NavSatFix<B> ────────────────────────────────────────────────────
//
// CDR layout: Header → offsets[0], then:
//   NavSatStatus(4) + pad(4) + latitude(f64) + longitude(f64) + altitude(f64)
//   + position_covariance([f64;9]) + position_covariance_type(u8)

pub struct NavSatFix<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B: AsRef<[u8]>> NavSatFix<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        NavSatStatus::read_cdr(&mut c)?;
        c.read_f64()?; // latitude
        c.read_f64()?; // longitude
        c.read_f64()?; // altitude
        read_f64_array9(&mut c)?; // position_covariance
        c.read_u8()?; // position_covariance_type
        Ok(NavSatFix { offsets: [o0], buf })
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

    pub fn status(&self) -> NavSatStatus {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        NavSatStatus::read_cdr(&mut c).expect("status field validated during from_cdr")
    }

    // NavSatFix fixed layout after status (NavSatStatus=4):
    //   lat(8), lon(8), alt(8), pos_cov[9](72), pos_cov_type(1)
    fn fixed_base(&self) -> usize {
        cdr_align(self.offsets[0] + 4, 8)
    }

    pub fn latitude(&self) -> f64 {
        rd_f64(self.buf.as_ref(), self.fixed_base())
    }
    pub fn longitude(&self) -> f64 {
        rd_f64(self.buf.as_ref(), self.fixed_base() + 8)
    }
    pub fn altitude(&self) -> f64 {
        rd_f64(self.buf.as_ref(), self.fixed_base() + 16)
    }

    pub fn position_covariance(&self) -> [f64; 9] {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.fixed_base() + 24);
        read_f64_array9(&mut c).expect("covariance field validated during from_cdr")
    }

    pub fn position_covariance_type(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.fixed_base() + 96)
    }

    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl NavSatFix<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        status: NavSatStatus,
        latitude: f64,
        longitude: f64,
        altitude: f64,
        position_covariance: [f64; 9],
        position_covariance_type: u8,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        NavSatStatus::size_cdr(&mut sizer);
        sizer.size_f64(); // latitude
        sizer.size_f64(); // longitude
        sizer.size_f64(); // altitude
        size_f64_array9(&mut sizer);
        sizer.size_u8(); // position_covariance_type

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        status.write_cdr(&mut w);
        w.write_f64(latitude);
        w.write_f64(longitude);
        w.write_f64(altitude);
        write_f64_array9(&mut w, &position_covariance);
        w.write_u8(position_covariance_type);
        w.finish()?;

        Ok(NavSatFix { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── PointField<B> ───────────────────────────────────────────────────
//
// CDR layout: name (string) → offsets[0], then offset(u32), datatype(u8), count(u32)

pub struct PointField<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B: AsRef<[u8]>> PointField<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let mut c = CdrCursor::new(buf.as_ref())?;
        let _ = c.read_string()?;
        let o0 = c.offset();
        c.read_u32()?;
        c.read_u8()?;
        c.read_u32()?;
        Ok(PointField { offsets: [o0], buf })
    }

    #[inline]
    pub fn name(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE).0
    }

    pub fn offset(&self) -> u32 {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        c.read_u32()
            .expect("point field element validated during from_cdr")
    }

    pub fn datatype(&self) -> u8 {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        c.read_u32()
            .expect("point field element validated during from_cdr"); // skip offset
        c.read_u8()
            .expect("point field element validated during from_cdr")
    }

    pub fn count(&self) -> u32 {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        c.read_u32()
            .expect("point field element validated during from_cdr"); // skip offset
        c.read_u8()
            .expect("point field element validated during from_cdr"); // skip datatype
        c.read_u32()
            .expect("point field element validated during from_cdr")
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl PointField<Vec<u8>> {
    pub fn new(name: &str, offset: u32, datatype: u8, count: u32) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        sizer.size_string(name);
        let o0 = sizer.offset();
        sizer.size_u32();
        sizer.size_u8();
        sizer.size_u32();

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        w.write_string(name);
        w.write_u32(offset);
        w.write_u8(datatype);
        w.write_u32(count);
        w.finish()?;

        Ok(PointField { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── PointCloud2<B> ──────────────────────────────────────────────────
//
// CDR layout: Header → offsets[0],
//   height(u32), width(u32),
//   fields(Vec<PointField>) → offsets[1],
//   is_bigendian(bool), point_step(u32), row_step(u32),
//   data(Vec<u8>) → offsets[2], is_dense(bool)

pub struct PointCloud2<B> {
    buf: B,
    offsets: [usize; 3],
}

impl<B: AsRef<[u8]>> PointCloud2<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        c.read_u32()?; // height
        c.read_u32()?; // width
        let raw_fields = c.read_u32()?;
        let fields_count = c.check_seq_count(raw_fields, 9)?;
        for _ in 0..fields_count {
            scan_point_field_element(&mut c)?;
        }
        let o1 = c.offset();
        c.read_bool()?; // is_bigendian
        c.read_u32()?; // point_step
        c.read_u32()?; // row_step
        let _ = c.read_bytes()?; // data
        let o2 = c.offset();
        c.read_bool()?; // is_dense
        Ok(PointCloud2 {
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

    pub fn height(&self) -> u32 {
        rd_u32(self.buf.as_ref(), align(self.offsets[0], 4))
    }
    pub fn width(&self) -> u32 {
        rd_u32(self.buf.as_ref(), align(self.offsets[0], 4) + 4)
    }
    pub fn fields_len(&self) -> u32 {
        rd_u32(self.buf.as_ref(), align(self.offsets[0], 4) + 8)
    }

    pub fn fields(&self) -> Vec<PointFieldView<'_>> {
        let b = self.buf.as_ref();
        let p = align(self.offsets[0], 4) + 8;
        let count = rd_u32(b, p) as usize;
        let mut c = CdrCursor::resume(b, p + 4);
        (0..count)
            .map(|_| {
                scan_point_field_element(&mut c)
                    .expect("point field elements validated during from_cdr")
            })
            .collect()
    }

    /// Non-allocating iterator over the PointField descriptors.
    pub fn fields_iter(&self) -> PointFieldIter<'_> {
        let b = self.buf.as_ref();
        let p = align(self.offsets[0], 4) + 8;
        let count = rd_u32(b, p) as usize;
        let cursor = CdrCursor::resume(b, p + 4);
        PointFieldIter {
            cursor,
            remaining: count,
        }
    }

    /// Total number of points (height × width).
    pub fn point_count(&self) -> usize {
        (self.height() as usize) * (self.width() as usize)
    }

    pub fn is_bigendian(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[1])
    }
    pub fn point_step(&self) -> u32 {
        rd_u32(self.buf.as_ref(), align(self.offsets[1] + 1, 4))
    }
    pub fn row_step(&self) -> u32 {
        rd_u32(self.buf.as_ref(), align(self.offsets[1] + 1, 4) + 4)
    }

    pub fn data(&self) -> &[u8] {
        rd_bytes(self.buf.as_ref(), align(self.offsets[1] + 1, 4) + 8).0
    }

    pub fn is_dense(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[2])
    }

    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl PointCloud2<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        height: u32,
        width: u32,
        fields: &[PointFieldView<'_>],
        is_bigendian: bool,
        point_step: u32,
        row_step: u32,
        data: &[u8],
        is_dense: bool,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_u32(); // height
        sizer.size_u32(); // width
        sizer.size_u32(); // fields count
        for f in fields {
            size_point_field_element(&mut sizer, f.name);
        }
        let o1 = sizer.offset();
        sizer.size_bool(); // is_bigendian
        sizer.size_u32(); // point_step
        sizer.size_u32(); // row_step
        sizer.size_bytes(data.len());
        let o2 = sizer.offset();
        sizer.size_bool(); // is_dense

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_u32(height);
        w.write_u32(width);
        w.write_u32(fields.len() as u32);
        for f in fields {
            write_point_field_element(&mut w, f);
        }
        w.write_bool(is_bigendian);
        w.write_u32(point_step);
        w.write_u32(row_step);
        w.write_bytes(data);
        w.write_bool(is_dense);
        w.finish()?;

        Ok(PointCloud2 {
            offsets: [o0, o1, o2],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── CameraInfo<B> ───────────────────────────────────────────────────
//
// CDR layout: Header → offsets[0],
//   height(u32), width(u32), distortion_model(string) → offsets[1],
//   d(Vec<f64>) → offsets[2], k[9], r[9], p[12],
//   binning_x(u32), binning_y(u32), roi(RegionOfInterest)

pub struct CameraInfo<B> {
    buf: B,
    offsets: [usize; 3],
}

impl<B: AsRef<[u8]>> CameraInfo<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        c.read_u32()?; // height
        c.read_u32()?; // width
        let _ = c.read_string()?; // distortion_model
        let o1 = c.offset();
        let d_count = c.read_u32()? as usize;
        c.skip_seq_8(d_count)?;
        let o2 = c.offset();
        read_f64_array9(&mut c)?; // k
        read_f64_array9(&mut c)?; // r
        read_f64_array12(&mut c)?; // p
        c.read_u32()?; // binning_x
        c.read_u32()?; // binning_y
        RegionOfInterest::read_cdr(&mut c)?;
        Ok(CameraInfo {
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

    pub fn height(&self) -> u32 {
        rd_u32(self.buf.as_ref(), align(self.offsets[0], 4))
    }
    pub fn width(&self) -> u32 {
        rd_u32(self.buf.as_ref(), align(self.offsets[0], 4) + 4)
    }

    pub fn distortion_model(&self) -> &str {
        rd_string(self.buf.as_ref(), align(self.offsets[0], 4) + 8).0
    }

    /// Number of distortion coefficients.
    pub fn d_len(&self) -> usize {
        rd_u32(self.buf.as_ref(), align(self.offsets[1], 4)) as usize
    }

    /// Read the i-th distortion coefficient (zero-copy, on-demand).
    pub fn d_get(&self, i: usize) -> f64 {
        let start = cdr_align(align(self.offsets[1], 4) + 4, 8);
        rd_f64(self.buf.as_ref(), start + i * 8)
    }

    // Fixed region after d: k[9](72) + r[9](72) + p[12](96) + binning(8) + roi(17)
    fn fixed_base(&self) -> usize {
        cdr_align(self.offsets[2], 8)
    }

    pub fn k(&self) -> [f64; 9] {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.fixed_base());
        read_f64_array9(&mut c).expect("covariance field validated during from_cdr")
    }

    pub fn r(&self) -> [f64; 9] {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.fixed_base() + 72);
        read_f64_array9(&mut c).expect("covariance field validated during from_cdr")
    }

    pub fn p(&self) -> [f64; 12] {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.fixed_base() + 144);
        read_f64_array12(&mut c).expect("projection matrix validated during from_cdr")
    }

    pub fn binning_x(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.fixed_base() + 240)
    }
    pub fn binning_y(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.fixed_base() + 244)
    }

    pub fn roi(&self) -> RegionOfInterest {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.fixed_base() + 248);
        RegionOfInterest::read_cdr(&mut c).expect("roi field validated during from_cdr")
    }

    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl CameraInfo<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        height: u32,
        width: u32,
        distortion_model: &str,
        d: &[f64],
        k: [f64; 9],
        r: [f64; 9],
        p: [f64; 12],
        binning_x: u32,
        binning_y: u32,
        roi: RegionOfInterest,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_u32(); // height
        sizer.size_u32(); // width
        sizer.size_string(distortion_model);
        let o1 = sizer.offset();
        sizer.size_u32();
        sizer.size_seq_8(d.len());
        let o2 = sizer.offset();
        size_f64_array9(&mut sizer);
        size_f64_array9(&mut sizer);
        size_f64_array12(&mut sizer);
        sizer.size_u32(); // binning_x
        sizer.size_u32(); // binning_y
        RegionOfInterest::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_u32(height);
        w.write_u32(width);
        w.write_string(distortion_model);
        w.write_u32(d.len() as u32);
        w.write_slice_f64(d);
        write_f64_array9(&mut w, &k);
        write_f64_array9(&mut w, &r);
        write_f64_array12(&mut w, &p);
        w.write_u32(binning_x);
        w.write_u32(binning_y);
        roi.write_cdr(&mut w);
        w.finish()?;

        Ok(CameraInfo {
            offsets: [o0, o1, o2],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── Constants ───────────────────────────────────────────────────────

pub mod nav_sat_fix {
    pub const COVARIANCE_TYPE_UNKNOWN: u8 = 0;
    pub const COVARIANCE_TYPE_APPROXIMATED: u8 = 1;
    pub const COVARIANCE_TYPE_DIAGONAL_KNOWN: u8 = 2;
    pub const COVARIANCE_TYPE_KNOWN: u8 = 3;
}

pub mod nav_sat_status {
    pub const STATUS_NO_FIX: i8 = -1;
    pub const STATUS_FIX: i8 = 0;
    pub const STATUS_SBAS_FIX: i8 = 1;
    pub const STATUS_GBAS_FIX: i8 = 2;
    pub const SERVICE_GPS: u8 = 1;
    pub const SERVICE_GLONASS: u8 = 2;
    pub const SERVICE_COMPASS: u8 = 4;
    pub const SERVICE_GALILEO: u8 = 8;
}

pub mod point_field {
    pub const INT8: u8 = 1;
    pub const UINT8: u8 = 2;
    pub const INT16: u8 = 3;
    pub const UINT16: u8 = 4;
    pub const INT32: u8 = 5;
    pub const UINT32: u8 = 6;
    pub const FLOAT32: u8 = 7;
    pub const FLOAT64: u8 = 8;
}

// ── Registry ────────────────────────────────────────────────────────

/// Check if a type name is supported by this module.
pub fn is_type_supported(type_name: &str) -> bool {
    matches!(
        type_name,
        "CameraInfo"
            | "CompressedImage"
            | "Image"
            | "Imu"
            | "NavSatFix"
            | "NavSatStatus"
            | "PointCloud2"
            | "PointField"
            | "RegionOfInterest"
    )
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &[
        "sensor_msgs/msg/CameraInfo",
        "sensor_msgs/msg/CompressedImage",
        "sensor_msgs/msg/Image",
        "sensor_msgs/msg/Imu",
        "sensor_msgs/msg/NavSatFix",
        "sensor_msgs/msg/NavSatStatus",
        "sensor_msgs/msg/PointCloud2",
        "sensor_msgs/msg/PointField",
        "sensor_msgs/msg/RegionOfInterest",
    ]
}

// SchemaType implementations
use crate::schema_registry::SchemaType;

impl SchemaType for NavSatStatus {
    const SCHEMA_NAME: &'static str = "sensor_msgs/msg/NavSatStatus";
}

impl SchemaType for RegionOfInterest {
    const SCHEMA_NAME: &'static str = "sensor_msgs/msg/RegionOfInterest";
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::cdr::{decode_fixed, encode_fixed};
    use crate::geometry_msgs::{Quaternion, Vector3};

    #[test]
    fn compressed_image_roundtrip() {
        let img = CompressedImage::new(
            Time::new(100, 500_000_000),
            "camera",
            "jpeg",
            &[0xFF, 0xD8, 0xFF],
        )
        .unwrap();
        assert_eq!(img.stamp(), Time::new(100, 500_000_000));
        assert_eq!(img.frame_id(), "camera");
        assert_eq!(img.format(), "jpeg");
        assert_eq!(img.data(), &[0xFF, 0xD8, 0xFF]);

        let bytes = img.to_cdr();
        let decoded = CompressedImage::from_cdr(bytes).unwrap();
        assert_eq!(decoded.format(), "jpeg");
        assert_eq!(decoded.data(), &[0xFF, 0xD8, 0xFF]);
    }

    #[test]
    fn image_roundtrip() {
        let data = vec![128u8; 1920 * 480];
        let img = Image::new(
            Time::new(100, 500_000_000),
            "camera_optical",
            480,
            640,
            "rgb8",
            0,
            1920,
            &data,
        )
        .unwrap();
        assert_eq!(img.height(), 480);
        assert_eq!(img.width(), 640);
        assert_eq!(img.encoding(), "rgb8");
        assert_eq!(img.is_bigendian(), 0);
        assert_eq!(img.step(), 1920);
        assert_eq!(img.data().len(), 1920 * 480);

        let bytes = img.to_cdr();
        let decoded = Image::from_cdr(bytes).unwrap();
        assert_eq!(decoded.height(), 480);
        assert_eq!(decoded.width(), 640);
    }

    #[test]
    fn imu_roundtrip() {
        let imu = Imu::new(
            Time::new(100, 0),
            "imu_link",
            Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
            [0.0; 9],
            Vector3 {
                x: 0.1,
                y: 0.2,
                z: 9.8,
            },
            [0.0; 9],
            Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            [0.0; 9],
        )
        .unwrap();
        assert_eq!(imu.stamp(), Time::new(100, 0));
        let bytes = imu.to_cdr();
        let decoded = Imu::from_cdr(bytes).unwrap();
        assert_eq!(decoded.orientation().w, 1.0);
        assert!((decoded.angular_velocity().z - 9.8).abs() < 1e-10);
    }

    #[test]
    fn nav_sat_fix_roundtrip() {
        let fix = NavSatFix::new(
            Time::new(100, 0),
            "gps",
            NavSatStatus {
                status: 0,
                service: 1,
            },
            45.5017,
            -73.5673,
            100.0,
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            2,
        )
        .unwrap();
        let bytes = fix.to_cdr();
        let decoded = NavSatFix::from_cdr(bytes).unwrap();
        assert!((decoded.latitude() - 45.5017).abs() < 1e-10);
        assert_eq!(decoded.position_covariance_type(), 2);
    }

    #[test]
    fn nav_sat_status_roundtrip() {
        let status = NavSatStatus {
            status: 0,
            service: 1,
        };
        let bytes = encode_fixed(&status).unwrap();
        let decoded: NavSatStatus = decode_fixed(&bytes).unwrap();
        assert_eq!(status, decoded);
    }

    #[test]
    fn region_of_interest_roundtrip() {
        let roi = RegionOfInterest {
            x_offset: 10,
            y_offset: 20,
            height: 100,
            width: 200,
            do_rectify: true,
        };
        let bytes = encode_fixed(&roi).unwrap();
        let decoded: RegionOfInterest = decode_fixed(&bytes).unwrap();
        assert_eq!(roi, decoded);
    }

    #[test]
    fn point_cloud2_roundtrip() {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        let data = vec![0u8; 12288];
        let cloud = PointCloud2::new(
            Time::new(100, 0),
            "lidar",
            1,
            1024,
            &fields,
            false,
            12,
            12288,
            &data,
            true,
        )
        .unwrap();
        let bytes = cloud.to_cdr();
        let decoded = PointCloud2::from_cdr(bytes).unwrap();
        assert_eq!(decoded.height(), 1);
        assert_eq!(decoded.width(), 1024);
        assert_eq!(decoded.fields_len(), 3);
        assert!(decoded.is_dense());
    }

    #[test]
    fn point_cloud2_fields_iter() {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        let data = vec![0u8; 36]; // 3 points × 12 bytes
        let cloud = PointCloud2::new(
            Time::new(100, 0),
            "lidar",
            1,
            3,
            &fields,
            false,
            12,
            36,
            &data,
            true,
        )
        .unwrap();
        let cdr = cloud.to_cdr();
        let decoded = PointCloud2::from_cdr(cdr).unwrap();

        let iter_fields: Vec<_> = decoded.fields_iter().collect();
        assert_eq!(iter_fields.len(), 3);
        assert_eq!(iter_fields[0].name, "x");
        assert_eq!(iter_fields[1].name, "y");
        assert_eq!(iter_fields[2].name, "z");
        assert_eq!(iter_fields[0].offset, 0);
        assert_eq!(iter_fields[1].offset, 4);
        assert_eq!(iter_fields[2].offset, 8);
        assert_eq!(decoded.point_count(), 3);
    }

    #[test]
    fn camera_info_roundtrip() {
        let roi = RegionOfInterest {
            x_offset: 0,
            y_offset: 0,
            height: 480,
            width: 640,
            do_rectify: false,
        };
        let cam = CameraInfo::new(
            Time::new(100, 0),
            "camera",
            480,
            640,
            "plumb_bob",
            &[0.1, -0.2, 0.0, 0.0, 0.0],
            [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            [
                500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            ],
            1,
            1,
            roi,
        )
        .unwrap();
        let bytes = cam.to_cdr();
        let decoded = CameraInfo::from_cdr(bytes).unwrap();
        assert_eq!(decoded.height(), 480);
        assert_eq!(decoded.width(), 640);
        assert_eq!(decoded.distortion_model(), "plumb_bob");
        assert_eq!(decoded.d_len(), 5);
        assert_eq!(decoded.binning_x(), 1);
    }
}
