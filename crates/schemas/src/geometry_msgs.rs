// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! ROS 2 `geometry_msgs` message types.
//!
//! CdrFixed: `Vector3`, `Point`, `Point32`, `Quaternion`, `Pose`,
//! `Pose2D`, `Transform`, `Accel`, `Twist`, `Inertia`,
//! `PoseWithCovariance`, `TwistWithCovariance`
//!
//! Buffer-backed (stamped wrappers): `AccelStamped`, `TwistStamped`,
//! `InertiaStamped`, `PointStamped`, `TransformStamped`, `Vector3Stamped`,
//! `PoseStamped`, `QuaternionStamped`, `WrenchStamped`, covariance-stamped
//! types, `Polygon`, `PolygonStamped`, `PoseArray`. Construct owned messages
//! with `Type::builder()`.

use crate::builtin_interfaces::Time;
use crate::cdr::*;
use crate::std_msgs::Header;
use std::borrow::Cow;

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

/// Shared encode path for header + one `CdrFixed` payload (stamped geometry types).
macro_rules! impl_stamped_cdrfixed_builder {
    ($Type:ident, $Builder:ident, $field:ident, $FieldTy:ty, $zero:expr) => {
        #[doc = concat!("Builder for `", stringify!($Type), "<Vec<u8>>` with buffer-reuse finalizers.")]
        pub struct $Builder<'a> {
            stamp: Time,
            frame_id: Cow<'a, str>,
            $field: $FieldTy,
        }

        impl<'a> Default for $Builder<'a> {
            fn default() -> Self {
                Self {
                    stamp: Time { sec: 0, nanosec: 0 },
                    frame_id: Cow::Borrowed(""),
                    $field: $zero,
                }
            }
        }

        impl<'a> $Builder<'a> {
            pub fn new() -> Self {
                Self::default()
            }

            pub fn stamp(&mut self, t: Time) -> &mut Self {
                self.stamp = t;
                self
            }
            pub fn frame_id(&mut self, s: impl Into<Cow<'a, str>>) -> &mut Self {
                self.frame_id = s.into();
                self
            }
            pub fn $field(&mut self, v: $FieldTy) -> &mut Self {
                self.$field = v;
                self
            }

            fn size(&self) -> usize {
                let mut s = CdrSizer::new();
                Time::size_cdr(&mut s);
                s.size_string(&self.frame_id);
                <$FieldTy>::size_cdr(&mut s);
                s.size()
            }

            fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
                let mut w = CdrWriter::new(buf)?;
                self.stamp.write_cdr(&mut w);
                w.write_string(&self.frame_id);
                self.$field.write_cdr(&mut w);
                w.finish()
            }

            pub fn build(&self) -> Result<$Type<Vec<u8>>, CdrError> {
                let mut buf = vec![0u8; self.size()];
                self.write_into(&mut buf)?;
                $Type::from_cdr(buf)
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
    };
}

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
    /// Start a new `AccelStampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> AccelStampedBuilder<'a> {
        AccelStampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl_stamped_cdrfixed_builder!(
    AccelStamped,
    AccelStampedBuilder,
    accel,
    Accel,
    Accel {
        linear: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        },
        angular: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        }
    }
);

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
    /// Start a new `TwistStampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> TwistStampedBuilder<'a> {
        TwistStampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl_stamped_cdrfixed_builder!(
    TwistStamped,
    TwistStampedBuilder,
    twist,
    Twist,
    Twist {
        linear: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        },
        angular: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        }
    }
);

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
    /// Start a new `InertiaStampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> InertiaStampedBuilder<'a> {
        InertiaStampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl_stamped_cdrfixed_builder!(
    InertiaStamped,
    InertiaStampedBuilder,
    inertia,
    Inertia,
    Inertia {
        m: 0.0,
        com: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        },
        ixx: 0.0,
        ixy: 0.0,
        ixz: 0.0,
        iyy: 0.0,
        iyz: 0.0,
        izz: 0.0
    }
);

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
    /// Start a new `PointStampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> PointStampedBuilder<'a> {
        PointStampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl_stamped_cdrfixed_builder!(
    PointStamped,
    PointStampedBuilder,
    point,
    Point,
    Point {
        x: 0.0,
        y: 0.0,
        z: 0.0
    }
);

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
    /// Start a new `TransformStampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> TransformStampedBuilder<'a> {
        TransformStampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

/// Builder for `TransformStamped<Vec<u8>>` with buffer-reuse finalizers.
pub struct TransformStampedBuilder<'a> {
    stamp: Time,
    frame_id: Cow<'a, str>,
    child_frame_id: Cow<'a, str>,
    transform: Transform,
}

impl<'a> Default for TransformStampedBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: Cow::Borrowed(""),
            child_frame_id: Cow::Borrowed(""),
            transform: Transform {
                translation: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                rotation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        }
    }
}

impl<'a> TransformStampedBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn child_frame_id(&mut self, s: impl Into<Cow<'a, str>>) -> &mut Self {
        self.child_frame_id = s.into();
        self
    }
    pub fn transform(&mut self, t: Transform) -> &mut Self {
        self.transform = t;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        s.size_string(&self.child_frame_id);
        Transform::size_cdr(&mut s);
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_string(&self.child_frame_id);
        self.transform.write_cdr(&mut w);
        w.finish()
    }

    pub fn build(&self) -> Result<TransformStamped<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        TransformStamped::from_cdr(buf)
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
    /// Start a new `Vector3StampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> Vector3StampedBuilder<'a> {
        Vector3StampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl_stamped_cdrfixed_builder!(
    Vector3Stamped,
    Vector3StampedBuilder,
    vector,
    Vector3,
    Vector3 {
        x: 0.0,
        y: 0.0,
        z: 0.0
    }
);

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
    /// Start a new `PoseStampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> PoseStampedBuilder<'a> {
        PoseStampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl_stamped_cdrfixed_builder!(
    PoseStamped,
    PoseStampedBuilder,
    pose,
    Pose,
    Pose {
        position: Point {
            x: 0.0,
            y: 0.0,
            z: 0.0
        },
        orientation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0
        }
    }
);

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
    /// Start a new `QuaternionStampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> QuaternionStampedBuilder<'a> {
        QuaternionStampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl_stamped_cdrfixed_builder!(
    QuaternionStamped,
    QuaternionStampedBuilder,
    quaternion,
    Quaternion,
    Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 1.0
    }
);

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
    /// Start a new `WrenchStampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> WrenchStampedBuilder<'a> {
        WrenchStampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl_stamped_cdrfixed_builder!(
    WrenchStamped,
    WrenchStampedBuilder,
    wrench,
    Wrench,
    Wrench {
        force: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        },
        torque: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0
        }
    }
);

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
    /// Start a new `PoseWithCovarianceStampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> PoseWithCovarianceStampedBuilder<'a> {
        PoseWithCovarianceStampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl_stamped_cdrfixed_builder!(
    PoseWithCovarianceStamped,
    PoseWithCovarianceStampedBuilder,
    pose_with_covariance,
    PoseWithCovariance,
    PoseWithCovariance {
        pose: Pose {
            position: Point {
                x: 0.0,
                y: 0.0,
                z: 0.0
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0
            }
        },
        covariance: [0.0; 36]
    }
);

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
    /// Start a new `TwistWithCovarianceStampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> TwistWithCovarianceStampedBuilder<'a> {
        TwistWithCovarianceStampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl_stamped_cdrfixed_builder!(
    TwistWithCovarianceStamped,
    TwistWithCovarianceStampedBuilder,
    twist_with_covariance,
    TwistWithCovariance,
    TwistWithCovariance {
        twist: Twist {
            linear: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        },
        covariance: [0.0; 36]
    }
);

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
    /// Start a new `AccelWithCovarianceStampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> AccelWithCovarianceStampedBuilder<'a> {
        AccelWithCovarianceStampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl_stamped_cdrfixed_builder!(
    AccelWithCovarianceStamped,
    AccelWithCovarianceStampedBuilder,
    accel_with_covariance,
    AccelWithCovariance,
    AccelWithCovariance {
        accel: Accel {
            linear: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0
            },
            angular: Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        },
        covariance: [0.0; 36]
    }
);

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
        let raw = c.read_seq_len()?;
        let count = c.check_seq_count(raw, Point32::CDR_SIZE)?;
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
    /// Start a new `PolygonBuilder` with an empty point list.
    pub fn builder<'a>() -> PolygonBuilder<'a> {
        PolygonBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

/// Builder for `Polygon<Vec<u8>>` with buffer-reuse finalizers.
pub struct PolygonBuilder<'a> {
    points: Cow<'a, [Point32]>,
}

impl<'a> Default for PolygonBuilder<'a> {
    fn default() -> Self {
        Self {
            points: Cow::Borrowed(&[]),
        }
    }
}

impl<'a> PolygonBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn points(&mut self, p: &'a [Point32]) -> &mut Self {
        self.points = Cow::Borrowed(p);
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        s.size_u32();
        for _ in 0..self.points.len() {
            Point32::size_cdr(&mut s);
        }
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        w.write_u32(self.points.len() as u32);
        for p in self.points.iter() {
            p.write_cdr(&mut w);
        }
        w.finish()
    }

    pub fn build(&self) -> Result<Polygon<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        Polygon::from_cdr(buf)
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

// ── PolygonStamped<B> ───────────────────────────────────────────────
//
// CDR layout: Header → seq_len (u32) → Point32[]

pub struct PolygonStamped<B> {
    buf: B,
    offsets: [usize; 2], // [0] = post-header cursor (seq_len position), [1] = start of points data
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
        let raw = c.read_seq_len()?;
        let count = c.check_seq_count(raw, Point32::CDR_SIZE)?;
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
    /// Start a new `PolygonStampedBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> PolygonStampedBuilder<'a> {
        PolygonStampedBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

/// Builder for `PolygonStamped<Vec<u8>>` with buffer-reuse finalizers.
pub struct PolygonStampedBuilder<'a> {
    stamp: Time,
    frame_id: Cow<'a, str>,
    points: Cow<'a, [Point32]>,
}

impl<'a> Default for PolygonStampedBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: Cow::Borrowed(""),
            points: Cow::Borrowed(&[]),
        }
    }
}

impl<'a> PolygonStampedBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn points(&mut self, p: &'a [Point32]) -> &mut Self {
        self.points = Cow::Borrowed(p);
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        s.size_u32();
        for _ in 0..self.points.len() {
            Point32::size_cdr(&mut s);
        }
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_u32(self.points.len() as u32);
        for p in self.points.iter() {
            p.write_cdr(&mut w);
        }
        w.finish()
    }

    pub fn build(&self) -> Result<PolygonStamped<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        PolygonStamped::from_cdr(buf)
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

// ── PoseArray<B> ────────────────────────────────────────────────────
//
// CDR layout: Header → seq_len (u32) → Pose[] (56 bytes each)

pub struct PoseArray<B> {
    buf: B,
    offsets: [usize; 2], // [0] = post-header cursor (seq_len position), [1] = start of poses data
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
        let raw = c.read_seq_len()?;
        let count = c.check_seq_count(raw, Pose::CDR_SIZE)?;
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
    /// Start a new `PoseArrayBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> PoseArrayBuilder<'a> {
        PoseArrayBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

/// Builder for `PoseArray<Vec<u8>>` with buffer-reuse finalizers.
pub struct PoseArrayBuilder<'a> {
    stamp: Time,
    frame_id: Cow<'a, str>,
    poses: Cow<'a, [Pose]>,
}

impl<'a> Default for PoseArrayBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: Cow::Borrowed(""),
            poses: Cow::Borrowed(&[]),
        }
    }
}

impl<'a> PoseArrayBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn poses(&mut self, p: &'a [Pose]) -> &mut Self {
        self.poses = Cow::Borrowed(p);
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        s.size_u32();
        for _ in 0..self.poses.len() {
            Pose::size_cdr(&mut s);
        }
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_u32(self.poses.len() as u32);
        for p in self.poses.iter() {
            p.write_cdr(&mut w);
        }
        w.finish()
    }

    pub fn build(&self) -> Result<PoseArray<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        PoseArray::from_cdr(buf)
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

// ── PoseStamped sequence helpers (pub(crate)) ───────────────────────
//
// Used by nav_msgs::Path to read/write/size embedded PoseStamped
// elements in a CDR sequence.  These helpers operate on the raw fields
// (stamp + frame_id + Pose) without the top-level 4-byte CDR
// encapsulation header, which only appears once at the outermost message
// boundary.

/// Scan one embedded PoseStamped element from `c` (no CDR header).
pub(crate) fn scan_pose_stamped(c: &mut CdrCursor<'_>) -> Result<(), CdrError> {
    crate::builtin_interfaces::Time::read_cdr(c)?;
    c.read_string()?;
    Pose::read_cdr(c)?;
    Ok(())
}

/// Advance `s` by the size of one embedded PoseStamped element.
pub(crate) fn size_pose_stamped(s: &mut CdrSizer, frame_id: &str) {
    crate::builtin_interfaces::Time::size_cdr(s);
    s.size_string(frame_id);
    Pose::size_cdr(s);
}

/// Write one embedded PoseStamped element to `w` (no CDR header).
pub(crate) fn write_pose_stamped(
    w: &mut CdrWriter<'_>,
    stamp: crate::builtin_interfaces::Time,
    frame_id: &str,
    pose: Pose,
) {
    stamp.write_cdr(w);
    w.write_string(frame_id);
    pose.write_cdr(w);
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
        let ts = TransformStamped::builder()
            .stamp(Time::new(100, 0))
            .frame_id("map")
            .child_frame_id("base_link")
            .transform(Transform {
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
            })
            .build()
            .unwrap();
        assert_eq!(ts.frame_id(), "map");
        assert_eq!(ts.child_frame_id(), "base_link");
        let bytes = ts.to_cdr();
        let decoded = TransformStamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.child_frame_id(), "base_link");
    }

    #[test]
    fn accel_stamped_roundtrip() {
        let a = AccelStamped::builder()
            .stamp(Time::new(1, 0))
            .frame_id("base")
            .accel(Accel {
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
            })
            .build()
            .unwrap();
        let bytes = a.to_cdr();
        let decoded = AccelStamped::from_cdr(bytes).unwrap();
        assert!((decoded.accel().linear.x - 1.0).abs() < 1e-10);
    }

    #[test]
    fn twist_stamped_roundtrip() {
        let t = TwistStamped::builder()
            .stamp(Time::new(1, 0))
            .frame_id("base")
            .twist(Twist {
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
            })
            .build()
            .unwrap();
        let bytes = t.to_cdr();
        let decoded = TwistStamped::from_cdr(bytes).unwrap();
        assert!((decoded.twist().angular.z - 0.5).abs() < 1e-10);
    }

    #[test]
    fn point_stamped_roundtrip() {
        let p = PointStamped::builder()
            .stamp(Time::new(1, 0))
            .frame_id("map")
            .point(Point {
                x: 10.0,
                y: 20.0,
                z: 30.0,
            })
            .build()
            .unwrap();
        let bytes = p.to_cdr();
        let decoded = PointStamped::from_cdr(bytes).unwrap();
        assert!((decoded.point().x - 10.0).abs() < 1e-10);
    }

    #[test]
    fn inertia_stamped_roundtrip() {
        let i = InertiaStamped::builder()
            .stamp(Time::new(1, 0))
            .frame_id("body")
            .inertia(Inertia {
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
            })
            .build()
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
        let v = Vector3Stamped::builder()
            .stamp(Time::new(1, 0))
            .frame_id("sensor")
            .vector(Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            })
            .build()
            .unwrap();
        let bytes = v.to_cdr();
        let decoded = Vector3Stamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "sensor");
        assert!((decoded.vector().x - 1.0).abs() < 1e-10);
        assert!((decoded.vector().z - 3.0).abs() < 1e-10);
    }

    #[test]
    fn pose_stamped_roundtrip() {
        let p = PoseStamped::builder()
            .stamp(Time::new(1, 0))
            .frame_id("map")
            .pose(Pose {
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
            })
            .build()
            .unwrap();
        let bytes = p.to_cdr();
        let decoded = PoseStamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "map");
        assert!((decoded.pose().position.x - 1.0).abs() < 1e-10);
        assert!((decoded.pose().orientation.w - 1.0).abs() < 1e-10);
    }

    #[test]
    fn quaternion_stamped_roundtrip() {
        let q = QuaternionStamped::builder()
            .stamp(Time::new(1, 0))
            .frame_id("imu")
            .quaternion(Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.707,
                w: 0.707,
            })
            .build()
            .unwrap();
        let bytes = q.to_cdr();
        let decoded = QuaternionStamped::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "imu");
        assert!((decoded.quaternion().z - 0.707).abs() < 1e-10);
    }

    #[test]
    fn wrench_stamped_roundtrip() {
        let w = WrenchStamped::builder()
            .stamp(Time::new(1, 0))
            .frame_id("tool")
            .wrench(Wrench {
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
            })
            .build()
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
        let p = PoseWithCovarianceStamped::builder()
            .stamp(Time::new(1, 0))
            .frame_id("odom")
            .pose_with_covariance(PoseWithCovariance {
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
            })
            .build()
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
        let t = TwistWithCovarianceStamped::builder()
            .stamp(Time::new(1, 0))
            .frame_id("base")
            .twist_with_covariance(TwistWithCovariance {
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
            })
            .build()
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
        let a = AccelWithCovarianceStamped::builder()
            .stamp(Time::new(1, 0))
            .frame_id("imu")
            .accel_with_covariance(AccelWithCovariance {
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
            })
            .build()
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
        let poly = Polygon::builder().points(&pts).build().unwrap();
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
        let poly = Polygon::builder().points(&[]).build().unwrap();
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
        let ps = PolygonStamped::builder()
            .stamp(Time::new(1, 0))
            .frame_id("map")
            .points(&pts)
            .build()
            .unwrap();
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
        let pa = PoseArray::builder()
            .stamp(Time::new(1, 0))
            .frame_id("map")
            .poses(&poses)
            .build()
            .unwrap();
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
        let pa = PoseArray::builder()
            .stamp(Time::new(1, 0))
            .frame_id("empty")
            .poses(&[])
            .build()
            .unwrap();
        assert!(pa.is_empty());
        assert_eq!(pa.poses(), vec![]);
        assert_eq!(pa.frame_id(), "empty");
    }
}
