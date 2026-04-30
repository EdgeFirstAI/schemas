// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! ROS 2 `geometry_msgs` message types.
//!
//! CdrFixed: `Vector3`, `Point`, `Point32`, `Quaternion`, `Pose`,
//! `Pose2D`, `Transform`, `Accel`, `Twist`, `Inertia`,
//! `PoseWithCovariance`, `TwistWithCovariance`
//!
//! Buffer-backed (stamped wrappers): `AccelStamped`, `TwistStamped`,
//! `InertiaStamped`, `PointStamped`, `TransformStamped`

use crate::builtin_interfaces::Time;
use crate::cdr::*;
use crate::std_msgs::Header;

// ── CdrFixed types ──────────────────────────────────────────────────

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Point32 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Accel {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct PoseWithCovariance {
    pub pose: Pose,
    /// Row-major 6×6 covariance of (x, y, z, rotX, rotY, rotZ).
    pub covariance: [f64; 36],
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct TwistWithCovariance {
    pub twist: Twist,
    /// Row-major 6×6 covariance of (x, y, z, rotX, rotY, rotZ).
    pub covariance: [f64; 36],
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Wrench {
    pub force: Vector3,
    pub torque: Vector3,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct AccelWithCovariance {
    pub accel: Accel,
    /// Row-major 6×6 covariance of (x, y, z, rotX, rotY, rotZ).
    pub covariance: [f64; 36],
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Inertia {
    pub m: f64,
    pub com: Vector3,
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}

// ── CdrFixed implementations ────────────────────────────────────────

impl CdrFixed for Vector3 {
    const CDR_SIZE: usize = 24;
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Vector3 {
            x: cursor.read_f64()?,
            y: cursor.read_f64()?,
            z: cursor.read_f64()?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_f64(self.x);
        writer.write_f64(self.y);
        writer.write_f64(self.z);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_f64();
        sizer.size_f64();
        sizer.size_f64();
    }
}

impl CdrFixed for Point {
    const CDR_SIZE: usize = 24;
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Point {
            x: cursor.read_f64()?,
            y: cursor.read_f64()?,
            z: cursor.read_f64()?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_f64(self.x);
        writer.write_f64(self.y);
        writer.write_f64(self.z);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_f64();
        sizer.size_f64();
        sizer.size_f64();
    }
}

impl CdrFixed for Point32 {
    const CDR_SIZE: usize = 12;
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Point32 {
            x: cursor.read_f32()?,
            y: cursor.read_f32()?,
            z: cursor.read_f32()?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_f32(self.x);
        writer.write_f32(self.y);
        writer.write_f32(self.z);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_f32();
    }
}

impl CdrFixed for Quaternion {
    const CDR_SIZE: usize = 32;
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Quaternion {
            x: cursor.read_f64()?,
            y: cursor.read_f64()?,
            z: cursor.read_f64()?,
            w: cursor.read_f64()?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_f64(self.x);
        writer.write_f64(self.y);
        writer.write_f64(self.z);
        writer.write_f64(self.w);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_f64();
        sizer.size_f64();
        sizer.size_f64();
        sizer.size_f64();
    }
}

impl CdrFixed for Pose {
    const CDR_SIZE: usize = 56;
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Pose {
            position: Point::read_cdr(cursor)?,
            orientation: Quaternion::read_cdr(cursor)?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        self.position.write_cdr(writer);
        self.orientation.write_cdr(writer);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        Point::size_cdr(sizer);
        Quaternion::size_cdr(sizer);
    }
}

impl CdrFixed for Pose2D {
    const CDR_SIZE: usize = 24;
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Pose2D {
            x: cursor.read_f64()?,
            y: cursor.read_f64()?,
            theta: cursor.read_f64()?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_f64(self.x);
        writer.write_f64(self.y);
        writer.write_f64(self.theta);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_f64();
        sizer.size_f64();
        sizer.size_f64();
    }
}

impl CdrFixed for Transform {
    const CDR_SIZE: usize = 56;
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Transform {
            translation: Vector3::read_cdr(cursor)?,
            rotation: Quaternion::read_cdr(cursor)?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        self.translation.write_cdr(writer);
        self.rotation.write_cdr(writer);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        Vector3::size_cdr(sizer);
        Quaternion::size_cdr(sizer);
    }
}

impl CdrFixed for Accel {
    const CDR_SIZE: usize = 48;
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Accel {
            linear: Vector3::read_cdr(cursor)?,
            angular: Vector3::read_cdr(cursor)?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        self.linear.write_cdr(writer);
        self.angular.write_cdr(writer);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        Vector3::size_cdr(sizer);
        Vector3::size_cdr(sizer);
    }
}

impl CdrFixed for Twist {
    const CDR_SIZE: usize = 48;
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Twist {
            linear: Vector3::read_cdr(cursor)?,
            angular: Vector3::read_cdr(cursor)?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        self.linear.write_cdr(writer);
        self.angular.write_cdr(writer);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        Vector3::size_cdr(sizer);
        Vector3::size_cdr(sizer);
    }
}

impl CdrFixed for PoseWithCovariance {
    const CDR_SIZE: usize = 56 + 36 * 8; // Pose(56) + [f64; 36](288) = 344
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        let pose = Pose::read_cdr(cursor)?;
        let mut covariance = [0.0_f64; 36];
        for slot in covariance.iter_mut() {
            *slot = cursor.read_f64()?;
        }
        Ok(PoseWithCovariance { pose, covariance })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        self.pose.write_cdr(writer);
        for v in &self.covariance {
            writer.write_f64(*v);
        }
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        Pose::size_cdr(sizer);
        for _ in 0..36 {
            sizer.size_f64();
        }
    }
}

impl CdrFixed for TwistWithCovariance {
    const CDR_SIZE: usize = 48 + 36 * 8; // Twist(48) + [f64; 36](288) = 336
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        let twist = Twist::read_cdr(cursor)?;
        let mut covariance = [0.0_f64; 36];
        for slot in covariance.iter_mut() {
            *slot = cursor.read_f64()?;
        }
        Ok(TwistWithCovariance { twist, covariance })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        self.twist.write_cdr(writer);
        for v in &self.covariance {
            writer.write_f64(*v);
        }
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        Twist::size_cdr(sizer);
        for _ in 0..36 {
            sizer.size_f64();
        }
    }
}

impl CdrFixed for Wrench {
    const CDR_SIZE: usize = 48;
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Wrench {
            force: Vector3::read_cdr(cursor)?,
            torque: Vector3::read_cdr(cursor)?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        self.force.write_cdr(writer);
        self.torque.write_cdr(writer);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        Vector3::size_cdr(sizer);
        Vector3::size_cdr(sizer);
    }
}

impl CdrFixed for AccelWithCovariance {
    const CDR_SIZE: usize = 48 + 36 * 8; // Accel(48) + [f64; 36](288) = 336
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        let accel = Accel::read_cdr(cursor)?;
        let mut covariance = [0.0_f64; 36];
        for slot in covariance.iter_mut() {
            *slot = cursor.read_f64()?;
        }
        Ok(AccelWithCovariance { accel, covariance })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        self.accel.write_cdr(writer);
        for v in &self.covariance {
            writer.write_f64(*v);
        }
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        Accel::size_cdr(sizer);
        for _ in 0..36 {
            sizer.size_f64();
        }
    }
}

impl CdrFixed for Inertia {
    const CDR_SIZE: usize = 80;
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Inertia {
            m: cursor.read_f64()?,
            com: Vector3::read_cdr(cursor)?,
            ixx: cursor.read_f64()?,
            ixy: cursor.read_f64()?,
            ixz: cursor.read_f64()?,
            iyy: cursor.read_f64()?,
            iyz: cursor.read_f64()?,
            izz: cursor.read_f64()?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_f64(self.m);
        self.com.write_cdr(writer);
        writer.write_f64(self.ixx);
        writer.write_f64(self.ixy);
        writer.write_f64(self.ixz);
        writer.write_f64(self.iyy);
        writer.write_f64(self.iyz);
        writer.write_f64(self.izz);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_f64();
        Vector3::size_cdr(sizer);
        sizer.size_f64();
        sizer.size_f64();
        sizer.size_f64();
        sizer.size_f64();
        sizer.size_f64();
        sizer.size_f64();
    }
}

// ── Buffer-backed stamped types ─────────────────────────────────────

// ── AccelStamped<B> ─────────────────────────────────────────────────

pub struct AccelStamped<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> AccelStamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> AccelStamped<C> {
        AccelStamped {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> AccelStamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        Accel::read_cdr(&mut c)?;
        Ok(AccelStamped { offsets: [o0], buf })
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
    pub fn accel(&self) -> Accel {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        Accel::read_cdr(&mut c).expect("accel field validated during from_cdr")
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl AccelStamped<Vec<u8>> {
    pub fn new(stamp: Time, frame_id: &str, accel: Accel) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        Accel::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        accel.write_cdr(&mut w);
        w.finish()?;

        Ok(AccelStamped { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── TwistStamped<B> ─────────────────────────────────────────────────

pub struct TwistStamped<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> TwistStamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> TwistStamped<C> {
        TwistStamped {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> TwistStamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        Twist::read_cdr(&mut c)?;
        Ok(TwistStamped { offsets: [o0], buf })
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
    pub fn twist(&self) -> Twist {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        Twist::read_cdr(&mut c).expect("twist field validated during from_cdr")
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl TwistStamped<Vec<u8>> {
    pub fn new(stamp: Time, frame_id: &str, twist: Twist) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        Twist::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        twist.write_cdr(&mut w);
        w.finish()?;

        Ok(TwistStamped { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── InertiaStamped<B> ───────────────────────────────────────────────

pub struct InertiaStamped<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> InertiaStamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> InertiaStamped<C> {
        InertiaStamped {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> InertiaStamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        Inertia::read_cdr(&mut c)?;
        Ok(InertiaStamped { offsets: [o0], buf })
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
    pub fn inertia(&self) -> Inertia {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        Inertia::read_cdr(&mut c).expect("inertia field validated during from_cdr")
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl InertiaStamped<Vec<u8>> {
    pub fn new(stamp: Time, frame_id: &str, inertia: Inertia) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        Inertia::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        inertia.write_cdr(&mut w);
        w.finish()?;

        Ok(InertiaStamped { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── PointStamped<B> ─────────────────────────────────────────────────

pub struct PointStamped<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> PointStamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> PointStamped<C> {
        PointStamped {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> PointStamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        Point::read_cdr(&mut c)?;
        Ok(PointStamped { offsets: [o0], buf })
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
    pub fn point(&self) -> Point {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        Point::read_cdr(&mut c).expect("point field validated during from_cdr")
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl PointStamped<Vec<u8>> {
    pub fn new(stamp: Time, frame_id: &str, point: Point) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        Point::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        point.write_cdr(&mut w);
        w.finish()?;

        Ok(PointStamped { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── TransformStamped<B> ─────────────────────────────────────────────
//
// CDR layout: Header → offsets[0], child_frame_id (string) → offsets[1],
//   then Transform (CdrFixed, 56)

pub struct TransformStamped<B> {
    buf: B,
    offsets: [usize; 2],
}

impl<B> TransformStamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> TransformStamped<C> {
        TransformStamped {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> TransformStamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        let _ = c.read_string()?; // child_frame_id
        let o1 = c.offset();
        Transform::read_cdr(&mut c)?;
        Ok(TransformStamped {
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
    #[inline]
    pub fn child_frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[0]).0
    }

    pub fn transform(&self) -> Transform {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[1]);
        Transform::read_cdr(&mut c).expect("transform field validated during from_cdr")
    }

    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl TransformStamped<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        child_frame_id: &str,
        transform: Transform,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_string(child_frame_id);
        let o1 = sizer.offset();
        Transform::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_string(child_frame_id);
        transform.write_cdr(&mut w);
        w.finish()?;

        Ok(TransformStamped {
            offsets: [o0, o1],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── Vector3Stamped<B> ───────────────────────────────────────────────

pub struct Vector3Stamped<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> Vector3Stamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> Vector3Stamped<C> {
        Vector3Stamped {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> Vector3Stamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        Vector3::read_cdr(&mut c)?;
        Ok(Vector3Stamped { offsets: [o0], buf })
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
    pub fn vector(&self) -> Vector3 {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        Vector3::read_cdr(&mut c).expect("vector3 field validated during from_cdr")
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl Vector3Stamped<Vec<u8>> {
    pub fn new(stamp: Time, frame_id: &str, vector: Vector3) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        Vector3::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        vector.write_cdr(&mut w);
        w.finish()?;

        Ok(Vector3Stamped { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── PoseStamped<B> ──────────────────────────────────────────────────

pub struct PoseStamped<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> PoseStamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> PoseStamped<C> {
        PoseStamped {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> PoseStamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        Pose::read_cdr(&mut c)?;
        Ok(PoseStamped { offsets: [o0], buf })
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
    pub fn pose(&self) -> Pose {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        Pose::read_cdr(&mut c).expect("pose field validated during from_cdr")
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl PoseStamped<Vec<u8>> {
    pub fn new(stamp: Time, frame_id: &str, pose: Pose) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        Pose::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        pose.write_cdr(&mut w);
        w.finish()?;

        Ok(PoseStamped { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── QuaternionStamped<B> ────────────────────────────────────────────

pub struct QuaternionStamped<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> QuaternionStamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> QuaternionStamped<C> {
        QuaternionStamped {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> QuaternionStamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        Quaternion::read_cdr(&mut c)?;
        Ok(QuaternionStamped { offsets: [o0], buf })
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
    pub fn quaternion(&self) -> Quaternion {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        Quaternion::read_cdr(&mut c).expect("quaternion field validated during from_cdr")
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl QuaternionStamped<Vec<u8>> {
    pub fn new(stamp: Time, frame_id: &str, quaternion: Quaternion) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        Quaternion::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        quaternion.write_cdr(&mut w);
        w.finish()?;

        Ok(QuaternionStamped { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── WrenchStamped<B> ────────────────────────────────────────────────

pub struct WrenchStamped<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> WrenchStamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> WrenchStamped<C> {
        WrenchStamped {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> WrenchStamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        Wrench::read_cdr(&mut c)?;
        Ok(WrenchStamped { offsets: [o0], buf })
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
    pub fn wrench(&self) -> Wrench {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        Wrench::read_cdr(&mut c).expect("wrench field validated during from_cdr")
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl WrenchStamped<Vec<u8>> {
    pub fn new(stamp: Time, frame_id: &str, wrench: Wrench) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        Wrench::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        wrench.write_cdr(&mut w);
        w.finish()?;

        Ok(WrenchStamped { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── PoseWithCovarianceStamped<B> ────────────────────────────────────

pub struct PoseWithCovarianceStamped<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> PoseWithCovarianceStamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> PoseWithCovarianceStamped<C> {
        PoseWithCovarianceStamped {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> PoseWithCovarianceStamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        PoseWithCovariance::read_cdr(&mut c)?;
        Ok(PoseWithCovarianceStamped { offsets: [o0], buf })
    }

    #[inline]
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
    pub fn pose_with_covariance(&self) -> PoseWithCovariance {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        PoseWithCovariance::read_cdr(&mut c)
            .expect("pose_with_covariance field validated during from_cdr")
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl PoseWithCovarianceStamped<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        pose_with_covariance: PoseWithCovariance,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        PoseWithCovariance::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        pose_with_covariance.write_cdr(&mut w);
        w.finish()?;

        Ok(PoseWithCovarianceStamped { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── TwistWithCovarianceStamped<B> ───────────────────────────────────

pub struct TwistWithCovarianceStamped<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> TwistWithCovarianceStamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> TwistWithCovarianceStamped<C> {
        TwistWithCovarianceStamped {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> TwistWithCovarianceStamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        TwistWithCovariance::read_cdr(&mut c)?;
        Ok(TwistWithCovarianceStamped { offsets: [o0], buf })
    }

    #[inline]
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
    pub fn twist_with_covariance(&self) -> TwistWithCovariance {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        TwistWithCovariance::read_cdr(&mut c)
            .expect("twist_with_covariance field validated during from_cdr")
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl TwistWithCovarianceStamped<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        twist_with_covariance: TwistWithCovariance,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        TwistWithCovariance::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        twist_with_covariance.write_cdr(&mut w);
        w.finish()?;

        Ok(TwistWithCovarianceStamped { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── AccelWithCovarianceStamped<B> ───────────────────────────────────

pub struct AccelWithCovarianceStamped<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> AccelWithCovarianceStamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> AccelWithCovarianceStamped<C> {
        AccelWithCovarianceStamped {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> AccelWithCovarianceStamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        AccelWithCovariance::read_cdr(&mut c)?;
        Ok(AccelWithCovarianceStamped { offsets: [o0], buf })
    }

    #[inline]
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
    pub fn accel_with_covariance(&self) -> AccelWithCovariance {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        AccelWithCovariance::read_cdr(&mut c)
            .expect("accel_with_covariance field validated during from_cdr")
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl AccelWithCovarianceStamped<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        accel_with_covariance: AccelWithCovariance,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        AccelWithCovariance::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        accel_with_covariance.write_cdr(&mut w);
        w.finish()?;

        Ok(AccelWithCovarianceStamped { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── Polygon<B> ──────────────────────────────────────────────────────
//
// CDR layout: CDR header (4 bytes) → seq_len (u32) → Point32[] (12 bytes each)

pub struct Polygon<B> {
    buf: B,
    offsets: [usize; 1], // offset to start of sequence data (past length prefix)
    count: usize,
}

impl<B> Polygon<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> Polygon<C> {
        Polygon {
            buf: f(self.buf),
            offsets: self.offsets,
            count: self.count,
        }
    }
}

impl<B: AsRef<[u8]>> Polygon<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let mut c = CdrCursor::new(buf.as_ref())?;
        let count = c.read_seq_len()? as usize;
        let o0 = c.offset();
        // Validate all points are readable
        for _ in 0..count {
            Point32::read_cdr(&mut c)?;
        }
        Ok(Polygon {
            offsets: [o0],
            count,
            buf,
        })
    }

    /// Number of points in the polygon.
    #[inline]
    pub fn len(&self) -> usize {
        self.count
    }

    /// Returns true if the polygon has no points.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Get a single point by index.
    #[inline]
    pub fn point(&self, index: usize) -> Option<Point32> {
        if index >= self.count {
            return None;
        }
        let offset = self.offsets[0] + index * Point32::CDR_SIZE;
        let mut c = CdrCursor::resume(self.buf.as_ref(), offset);
        Some(Point32::read_cdr(&mut c).expect("point validated during from_cdr"))
    }

    /// Get all points as a Vec.
    pub fn points(&self) -> Vec<Point32> {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        (0..self.count)
            .map(|_| Point32::read_cdr(&mut c).expect("point validated during from_cdr"))
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

impl Polygon<Vec<u8>> {
    pub fn new(points: &[Point32]) -> Result<Self, CdrError> {
        let count = points.len();
        let mut sizer = CdrSizer::new();
        sizer.size_u32(); // seq_len
        let o0 = sizer.offset();
        for _ in 0..count {
            Point32::size_cdr(&mut sizer);
        }

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        w.write_u32(count as u32);
        for p in points {
            p.write_cdr(&mut w);
        }
        w.finish()?;

        Ok(Polygon {
            offsets: [o0],
            count,
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── PolygonStamped<B> ───────────────────────────────────────────────
//
// CDR layout: Header → seq_len (u32) → Point32[]

pub struct PolygonStamped<B> {
    buf: B,
    offsets: [usize; 2], // [0] = start of seq_len, [1] = start of points data
    count: usize,
}

impl<B> PolygonStamped<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> PolygonStamped<C> {
        PolygonStamped {
            buf: f(self.buf),
            offsets: self.offsets,
            count: self.count,
        }
    }
}

impl<B: AsRef<[u8]>> PolygonStamped<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        let count = c.read_seq_len()? as usize;
        let o1 = c.offset();
        for _ in 0..count {
            Point32::read_cdr(&mut c)?;
        }
        Ok(PolygonStamped {
            offsets: [o0, o1],
            count,
            buf,
        })
    }

    #[inline]
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

    /// Number of points in the polygon.
    #[inline]
    pub fn len(&self) -> usize {
        self.count
    }

    /// Returns true if the polygon has no points.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Get a single point by index.
    #[inline]
    pub fn point(&self, index: usize) -> Option<Point32> {
        if index >= self.count {
            return None;
        }
        let offset = self.offsets[1] + index * Point32::CDR_SIZE;
        let mut c = CdrCursor::resume(self.buf.as_ref(), offset);
        Some(Point32::read_cdr(&mut c).expect("point validated during from_cdr"))
    }

    /// Get all points as a Vec.
    pub fn points(&self) -> Vec<Point32> {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[1]);
        (0..self.count)
            .map(|_| Point32::read_cdr(&mut c).expect("point validated during from_cdr"))
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

impl PolygonStamped<Vec<u8>> {
    pub fn new(stamp: Time, frame_id: &str, points: &[Point32]) -> Result<Self, CdrError> {
        let count = points.len();
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_u32(); // seq_len
        let o1 = sizer.offset();
        for _ in 0..count {
            Point32::size_cdr(&mut sizer);
        }

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_u32(count as u32);
        for p in points {
            p.write_cdr(&mut w);
        }
        w.finish()?;

        Ok(PolygonStamped {
            offsets: [o0, o1],
            count,
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── PoseArray<B> ────────────────────────────────────────────────────
//
// CDR layout: Header → seq_len (u32) → Pose[] (56 bytes each)

pub struct PoseArray<B> {
    buf: B,
    offsets: [usize; 2], // [0] = start of seq_len, [1] = start of poses data
    count: usize,
}

impl<B> PoseArray<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> PoseArray<C> {
        PoseArray {
            buf: f(self.buf),
            offsets: self.offsets,
            count: self.count,
        }
    }
}

impl<B: AsRef<[u8]>> PoseArray<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        let count = c.read_seq_len()? as usize;
        let o1 = c.offset();
        for _ in 0..count {
            Pose::read_cdr(&mut c)?;
        }
        Ok(PoseArray {
            offsets: [o0, o1],
            count,
            buf,
        })
    }

    #[inline]
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

    /// Number of poses in the array.
    #[inline]
    pub fn len(&self) -> usize {
        self.count
    }

    /// Returns true if the array is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Get a single pose by index.
    #[inline]
    pub fn pose(&self, index: usize) -> Option<Pose> {
        if index >= self.count {
            return None;
        }
        let offset = self.offsets[1] + index * Pose::CDR_SIZE;
        let mut c = CdrCursor::resume(self.buf.as_ref(), offset);
        Some(Pose::read_cdr(&mut c).expect("pose validated during from_cdr"))
    }

    /// Get all poses as a Vec.
    pub fn poses(&self) -> Vec<Pose> {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[1]);
        (0..self.count)
            .map(|_| Pose::read_cdr(&mut c).expect("pose validated during from_cdr"))
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

impl PoseArray<Vec<u8>> {
    pub fn new(stamp: Time, frame_id: &str, poses: &[Pose]) -> Result<Self, CdrError> {
        let count = poses.len();
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_u32(); // seq_len
        let o1 = sizer.offset();
        for _ in 0..count {
            Pose::size_cdr(&mut sizer);
        }

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_u32(count as u32);
        for p in poses {
            p.write_cdr(&mut w);
        }
        w.finish()?;

        Ok(PoseArray {
            offsets: [o0, o1],
            count,
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

/// Check if a type name is supported by this module.
pub fn is_type_supported(type_name: &str) -> bool {
    matches!(
        type_name,
        "Accel"
            | "AccelStamped"
            | "AccelWithCovariance"
            | "AccelWithCovarianceStamped"
            | "Inertia"
            | "InertiaStamped"
            | "Point"
            | "Point32"
            | "PointStamped"
            | "Polygon"
            | "PolygonStamped"
            | "Pose"
            | "Pose2D"
            | "PoseArray"
            | "PoseStamped"
            | "PoseWithCovariance"
            | "PoseWithCovarianceStamped"
            | "Quaternion"
            | "QuaternionStamped"
            | "Transform"
            | "TransformStamped"
            | "Twist"
            | "TwistStamped"
            | "TwistWithCovariance"
            | "TwistWithCovarianceStamped"
            | "Vector3"
            | "Vector3Stamped"
            | "Wrench"
            | "WrenchStamped"
    )
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &[
        "geometry_msgs/msg/Accel",
        "geometry_msgs/msg/AccelStamped",
        "geometry_msgs/msg/AccelWithCovariance",
        "geometry_msgs/msg/AccelWithCovarianceStamped",
        "geometry_msgs/msg/Inertia",
        "geometry_msgs/msg/InertiaStamped",
        "geometry_msgs/msg/Point",
        "geometry_msgs/msg/Point32",
        "geometry_msgs/msg/PointStamped",
        "geometry_msgs/msg/Polygon",
        "geometry_msgs/msg/PolygonStamped",
        "geometry_msgs/msg/Pose",
        "geometry_msgs/msg/Pose2D",
        "geometry_msgs/msg/PoseArray",
        "geometry_msgs/msg/PoseStamped",
        "geometry_msgs/msg/PoseWithCovariance",
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        "geometry_msgs/msg/Quaternion",
        "geometry_msgs/msg/QuaternionStamped",
        "geometry_msgs/msg/Transform",
        "geometry_msgs/msg/TransformStamped",
        "geometry_msgs/msg/Twist",
        "geometry_msgs/msg/TwistStamped",
        "geometry_msgs/msg/TwistWithCovariance",
        "geometry_msgs/msg/TwistWithCovarianceStamped",
        "geometry_msgs/msg/Vector3",
        "geometry_msgs/msg/Vector3Stamped",
        "geometry_msgs/msg/Wrench",
        "geometry_msgs/msg/WrenchStamped",
    ]
}

// SchemaType implementations
use crate::schema_registry::SchemaType;

impl SchemaType for Accel {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Accel";
}
impl SchemaType for Inertia {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Inertia";
}
impl SchemaType for Point {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Point";
}
impl SchemaType for Point32 {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Point32";
}
impl SchemaType for Pose {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Pose";
}
impl SchemaType for Pose2D {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Pose2D";
}
impl SchemaType for Quaternion {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Quaternion";
}
impl SchemaType for Transform {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Transform";
}
impl SchemaType for Twist {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Twist";
}
impl SchemaType for Vector3 {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Vector3";
}
impl SchemaType for Wrench {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/Wrench";
}
impl SchemaType for AccelWithCovariance {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/AccelWithCovariance";
}
impl SchemaType for PoseWithCovariance {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/PoseWithCovariance";
}
impl SchemaType for TwistWithCovariance {
    const SCHEMA_NAME: &'static str = "geometry_msgs/msg/TwistWithCovariance";
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::cdr::{decode_fixed, encode_fixed};

    #[test]
    fn primitive_types_roundtrip() {
        let vec = Vector3 {
            x: 1.5,
            y: -2.5,
            z: f64::MAX,
        };
        let bytes = encode_fixed(&vec).unwrap();
        assert_eq!(vec, decode_fixed::<Vector3>(&bytes).unwrap());

        let point = Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let bytes = encode_fixed(&point).unwrap();
        assert_eq!(point, decode_fixed::<Point>(&bytes).unwrap());

        let point32 = Point32 {
            x: 1.0f32,
            y: 2.0,
            z: f32::MIN,
        };
        let bytes = encode_fixed(&point32).unwrap();
        assert_eq!(point32, decode_fixed::<Point32>(&bytes).unwrap());

        let quat = Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        };
        let bytes = encode_fixed(&quat).unwrap();
        assert_eq!(quat, decode_fixed::<Quaternion>(&bytes).unwrap());
    }

    #[test]
    fn composite_types_roundtrip() {
        let pose = Pose {
            position: Point {
                x: 1.0,
                y: 2.0,
                z: 0.5,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        };
        let bytes = encode_fixed(&pose).unwrap();
        assert_eq!(pose, decode_fixed::<Pose>(&bytes).unwrap());

        let transform = Transform {
            translation: Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            rotation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.707,
                w: 0.707,
            },
        };
        let bytes = encode_fixed(&transform).unwrap();
        assert_eq!(transform, decode_fixed::<Transform>(&bytes).unwrap());

        let twist = Twist {
            linear: Vector3 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.5,
            },
        };
        let bytes = encode_fixed(&twist).unwrap();
        assert_eq!(twist, decode_fixed::<Twist>(&bytes).unwrap());

        let accel = Accel {
            linear: Vector3 {
                x: 9.8,
                y: 0.0,
                z: 0.0,
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        };
        let bytes = encode_fixed(&accel).unwrap();
        assert_eq!(accel, decode_fixed::<Accel>(&bytes).unwrap());
    }

    #[test]
    fn transform_stamped_roundtrip() {
        let ts = TransformStamped::new(
            Time::new(100, 0),
            "map",
            "base_link",
            Transform {
                translation: Vector3 {
                    x: 1.0,
                    y: 2.0,
                    z: 0.0,
                },
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        )
        .unwrap();
        assert_eq!(ts.frame_id(), "map");
        assert_eq!(ts.child_frame_id(), "base_link");
        let bytes = ts.to_cdr();
        let decoded = TransformStamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.child_frame_id(), "base_link");
    }

    #[test]
    fn accel_stamped_roundtrip() {
        let a = AccelStamped::new(
            Time::new(1, 0),
            "base",
            Accel {
                linear: Vector3 {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.5,
                },
            },
        )
        .unwrap();
        let bytes = a.to_cdr();
        let decoded = AccelStamped::from_cdr(bytes).unwrap();
        assert!((decoded.accel().linear.x - 1.0).abs() < 1e-10);
    }

    #[test]
    fn twist_stamped_roundtrip() {
        let t = TwistStamped::new(
            Time::new(1, 0),
            "base",
            Twist {
                linear: Vector3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.5,
                },
            },
        )
        .unwrap();
        let bytes = t.to_cdr();
        let decoded = TwistStamped::from_cdr(bytes).unwrap();
        assert!((decoded.twist().angular.z - 0.5).abs() < 1e-10);
    }

    #[test]
    fn point_stamped_roundtrip() {
        let p = PointStamped::new(
            Time::new(1, 0),
            "map",
            Point {
                x: 10.0,
                y: 20.0,
                z: 30.0,
            },
        )
        .unwrap();
        let bytes = p.to_cdr();
        let decoded = PointStamped::from_cdr(bytes).unwrap();
        assert!((decoded.point().x - 10.0).abs() < 1e-10);
    }

    #[test]
    fn inertia_stamped_roundtrip() {
        let i = InertiaStamped::new(
            Time::new(1, 0),
            "body",
            Inertia {
                m: 10.0,
                com: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                ixx: 1.0,
                ixy: 0.0,
                ixz: 0.0,
                iyy: 1.0,
                iyz: 0.0,
                izz: 1.0,
            },
        )
        .unwrap();
        let bytes = i.to_cdr();
        let decoded = InertiaStamped::from_cdr(bytes).unwrap();
        assert!((decoded.inertia().m - 10.0).abs() < 1e-10);
    }

    #[test]
    fn wrench_roundtrip() {
        let w = Wrench {
            force: Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            torque: Vector3 {
                x: 0.1,
                y: 0.2,
                z: 0.3,
            },
        };
        let bytes = encode_fixed(&w).unwrap();
        assert_eq!(w, decode_fixed::<Wrench>(&bytes).unwrap());
    }

    #[test]
    fn accel_with_covariance_roundtrip() {
        let mut cov = [0.0_f64; 36];
        cov[0] = 1.0;
        cov[7] = 2.0;
        cov[14] = 3.0;
        let awc = AccelWithCovariance {
            accel: Accel {
                linear: Vector3 {
                    x: 9.8,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
            },
            covariance: cov,
        };
        let bytes = encode_fixed(&awc).unwrap();
        let decoded = decode_fixed::<AccelWithCovariance>(&bytes).unwrap();
        assert!((decoded.accel.linear.x - 9.8).abs() < 1e-10);
        assert!((decoded.covariance[0] - 1.0).abs() < 1e-10);
        assert!((decoded.covariance[7] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn vector3_stamped_roundtrip() {
        let v = Vector3Stamped::new(
            Time::new(1, 0),
            "sensor",
            Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
        )
        .unwrap();
        let bytes = v.to_cdr();
        let decoded = Vector3Stamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "sensor");
        assert!((decoded.vector().x - 1.0).abs() < 1e-10);
        assert!((decoded.vector().z - 3.0).abs() < 1e-10);
    }

    #[test]
    fn pose_stamped_roundtrip() {
        let p = PoseStamped::new(
            Time::new(1, 0),
            "map",
            Pose {
                position: Point {
                    x: 1.0,
                    y: 2.0,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        )
        .unwrap();
        let bytes = p.to_cdr();
        let decoded = PoseStamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "map");
        assert!((decoded.pose().position.x - 1.0).abs() < 1e-10);
        assert!((decoded.pose().orientation.w - 1.0).abs() < 1e-10);
    }

    #[test]
    fn quaternion_stamped_roundtrip() {
        let q = QuaternionStamped::new(
            Time::new(1, 0),
            "imu",
            Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.707,
                w: 0.707,
            },
        )
        .unwrap();
        let bytes = q.to_cdr();
        let decoded = QuaternionStamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "imu");
        assert!((decoded.quaternion().z - 0.707).abs() < 1e-10);
    }

    #[test]
    fn wrench_stamped_roundtrip() {
        let w = WrenchStamped::new(
            Time::new(1, 0),
            "tool",
            Wrench {
                force: Vector3 {
                    x: 10.0,
                    y: 0.0,
                    z: -9.8,
                },
                torque: Vector3 {
                    x: 0.0,
                    y: 0.5,
                    z: 0.0,
                },
            },
        )
        .unwrap();
        let bytes = w.to_cdr();
        let decoded = WrenchStamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "tool");
        assert!((decoded.wrench().force.x - 10.0).abs() < 1e-10);
    }

    #[test]
    fn pose_with_covariance_stamped_roundtrip() {
        let mut cov = [0.0_f64; 36];
        cov[0] = 0.1;
        let p = PoseWithCovarianceStamped::new(
            Time::new(1, 0),
            "odom",
            PoseWithCovariance {
                pose: Pose {
                    position: Point {
                        x: 1.0,
                        y: 2.0,
                        z: 0.0,
                    },
                    orientation: Quaternion {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                        w: 1.0,
                    },
                },
                covariance: cov,
            },
        )
        .unwrap();
        let bytes = p.to_cdr();
        let decoded = PoseWithCovarianceStamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "odom");
        assert!((decoded.pose_with_covariance().covariance[0] - 0.1).abs() < 1e-10);
    }

    #[test]
    fn twist_with_covariance_stamped_roundtrip() {
        let mut cov = [0.0_f64; 36];
        cov[0] = 0.5;
        let t = TwistWithCovarianceStamped::new(
            Time::new(1, 0),
            "base",
            TwistWithCovariance {
                twist: Twist {
                    linear: Vector3 {
                        x: 1.0,
                        y: 0.0,
                        z: 0.0,
                    },
                    angular: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.5,
                    },
                },
                covariance: cov,
            },
        )
        .unwrap();
        let bytes = t.to_cdr();
        let decoded = TwistWithCovarianceStamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "base");
        assert!((decoded.twist_with_covariance().covariance[0] - 0.5).abs() < 1e-10);
    }

    #[test]
    fn accel_with_covariance_stamped_roundtrip() {
        let mut cov = [0.0_f64; 36];
        cov[0] = 0.2;
        let a = AccelWithCovarianceStamped::new(
            Time::new(1, 0),
            "imu",
            AccelWithCovariance {
                accel: Accel {
                    linear: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 9.8,
                    },
                    angular: Vector3 {
                        x: 0.0,
                        y: 0.0,
                        z: 0.0,
                    },
                },
                covariance: cov,
            },
        )
        .unwrap();
        let bytes = a.to_cdr();
        let decoded = AccelWithCovarianceStamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "imu");
        assert!((decoded.accel_with_covariance().accel.linear.z - 9.8).abs() < 1e-10);
    }

    #[test]
    fn polygon_roundtrip() {
        let pts = vec![
            Point32 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            Point32 {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            Point32 {
                x: 0.0,
                y: 1.0,
                z: 0.0,
            },
        ];
        let poly = Polygon::new(&pts).unwrap();
        assert_eq!(poly.len(), 3);
        let bytes = poly.to_cdr();
        let decoded = Polygon::from_cdr(bytes).unwrap();
        assert_eq!(decoded.len(), 3);
        assert!((decoded.point(0).unwrap().x - 0.0).abs() < 1e-6);
        assert!((decoded.point(1).unwrap().x - 1.0).abs() < 1e-6);
        assert!((decoded.point(2).unwrap().y - 1.0).abs() < 1e-6);
        assert!(decoded.point(3).is_none());
    }

    #[test]
    fn polygon_empty() {
        let poly = Polygon::new(&[]).unwrap();
        assert!(poly.is_empty());
        assert_eq!(poly.points(), vec![]);
    }

    #[test]
    fn polygon_stamped_roundtrip() {
        let pts = vec![
            Point32 {
                x: 1.0,
                y: 2.0,
                z: 0.0,
            },
            Point32 {
                x: 3.0,
                y: 4.0,
                z: 0.0,
            },
        ];
        let ps = PolygonStamped::new(Time::new(1, 0), "map", &pts).unwrap();
        assert_eq!(ps.len(), 2);
        let bytes = ps.to_cdr();
        let decoded = PolygonStamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "map");
        assert_eq!(decoded.len(), 2);
        assert!((decoded.point(0).unwrap().x - 1.0).abs() < 1e-6);
    }

    #[test]
    fn pose_array_roundtrip() {
        let poses = vec![
            Pose {
                position: Point {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
            Pose {
                position: Point {
                    x: 2.0,
                    y: 0.0,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.707,
                    w: 0.707,
                },
            },
        ];
        let pa = PoseArray::new(Time::new(1, 0), "map", &poses).unwrap();
        assert_eq!(pa.len(), 2);
        let bytes = pa.to_cdr();
        let decoded = PoseArray::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "map");
        assert_eq!(decoded.len(), 2);
        assert!((decoded.pose(0).unwrap().position.x - 1.0).abs() < 1e-10);
        assert!((decoded.pose(1).unwrap().orientation.z - 0.707).abs() < 1e-10);
    }

    #[test]
    fn pose_array_empty() {
        let pa = PoseArray::new(Time::new(1, 0), "empty", &[]).unwrap();
        assert!(pa.is_empty());
        assert_eq!(pa.poses(), vec![]);
        assert_eq!(pa.frame_id(), "empty");
    }
}
