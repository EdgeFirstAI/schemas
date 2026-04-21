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

// ── Registry ────────────────────────────────────────────────────────

/// Check if a type name is supported by this module.
pub fn is_type_supported(type_name: &str) -> bool {
    matches!(
        type_name,
        "Accel"
            | "AccelStamped"
            | "Inertia"
            | "InertiaStamped"
            | "Point"
            | "Point32"
            | "PointStamped"
            | "Pose"
            | "Pose2D"
            | "Quaternion"
            | "Transform"
            | "TransformStamped"
            | "Twist"
            | "TwistStamped"
            | "Vector3"
    )
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &[
        "geometry_msgs/msg/Accel",
        "geometry_msgs/msg/AccelStamped",
        "geometry_msgs/msg/Inertia",
        "geometry_msgs/msg/InertiaStamped",
        "geometry_msgs/msg/Point",
        "geometry_msgs/msg/Point32",
        "geometry_msgs/msg/PointStamped",
        "geometry_msgs/msg/Pose",
        "geometry_msgs/msg/Pose2D",
        "geometry_msgs/msg/Quaternion",
        "geometry_msgs/msg/Transform",
        "geometry_msgs/msg/TransformStamped",
        "geometry_msgs/msg/Twist",
        "geometry_msgs/msg/TwistStamped",
        "geometry_msgs/msg/Vector3",
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
}
