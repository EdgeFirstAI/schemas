// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! MAVLink/MAVROS message types.
//!
//! Buffer-backed: `Altitude`, `VfrHud`, `EstimatorStatus`, `ExtendedState`,
//! `SysStatus`, `State`, `StatusText`, `GpsRaw`, `TimesyncStatus`
//!
//! Message definitions derived from [mavros_msgs](https://github.com/mavlink/mavros)
//! (BSD-3-Clause licensed). Zero-copy CDR implementation is original work.

use crate::builtin_interfaces::Time;
use crate::cdr::*;
use crate::std_msgs::Header;

// ── Altitude<B> ─────────────────────────────────────────────────────
//
// CDR layout: Header, then:
//   align(4) → offsets[0]
//   f32 monotonic         +0
//   f32 amsl              +4
//   f32 local             +8
//   f32 relative          +12
//   f32 terrain           +16
//   f32 bottom_clearance  +20
//   payload = 24 bytes

pub struct Altitude<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> Altitude<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> Altitude<C> {
        Altitude {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> Altitude<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let pre = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), pre);
        c.align(4);
        let o0 = c.offset();
        c.read_f32()?; // monotonic
        c.read_f32()?; // amsl
        c.read_f32()?; // local
        c.read_f32()?; // relative
        c.read_f32()?; // terrain
        c.read_f32()?; // bottom_clearance
        Ok(Altitude { offsets: [o0], buf })
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
    pub fn monotonic(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0])
    }
    #[inline]
    pub fn amsl(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 4)
    }
    #[inline]
    pub fn local(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 8)
    }
    #[inline]
    pub fn relative(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 12)
    }
    #[inline]
    pub fn terrain(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 16)
    }
    #[inline]
    pub fn bottom_clearance(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 20)
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl Altitude<Vec<u8>> {
    /// Start a new `AltitudeBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> AltitudeBuilder<'a> {
        AltitudeBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

/// Builder for `Altitude<Vec<u8>>` with buffer-reuse finalizers.
pub struct AltitudeBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    monotonic: f32,
    amsl: f32,
    local: f32,
    relative: f32,
    terrain: f32,
    bottom_clearance: f32,
}

impl<'a> Default for AltitudeBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            monotonic: 0.0,
            amsl: 0.0,
            local: 0.0,
            relative: 0.0,
            terrain: 0.0,
            bottom_clearance: 0.0,
        }
    }
}

impl<'a> AltitudeBuilder<'a> {
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
    pub fn monotonic(&mut self, v: f32) -> &mut Self {
        self.monotonic = v;
        self
    }
    pub fn amsl(&mut self, v: f32) -> &mut Self {
        self.amsl = v;
        self
    }
    pub fn local(&mut self, v: f32) -> &mut Self {
        self.local = v;
        self
    }
    pub fn relative(&mut self, v: f32) -> &mut Self {
        self.relative = v;
        self
    }
    pub fn terrain(&mut self, v: f32) -> &mut Self {
        self.terrain = v;
        self
    }
    pub fn bottom_clearance(&mut self, v: f32) -> &mut Self {
        self.bottom_clearance = v;
        self
    }

    fn size(&self) -> usize {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(&self.frame_id);
        sizer.align(4);
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_f32();
        sizer.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.align(4);
        w.write_f32(self.monotonic);
        w.write_f32(self.amsl);
        w.write_f32(self.local);
        w.write_f32(self.relative);
        w.write_f32(self.terrain);
        w.write_f32(self.bottom_clearance);
        w.finish()
    }

    pub fn build(&self) -> Result<Altitude<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        Altitude::from_cdr(buf)
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

// ── VfrHud<B> ────────────────────────────────────────────────────────
//
// CDR layout: Header, then:
//   align(4) → offsets[0]
//   f32 airspeed       +0
//   f32 groundspeed    +4
//   i16 heading        +8
//   [2B pad]           +10
//   f32 throttle       +12
//   f32 altitude       +16
//   f32 climb          +20
//   payload = 24 bytes (including 2B pad)

pub struct VfrHud<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> VfrHud<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> VfrHud<C> {
        VfrHud {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> VfrHud<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let pre = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), pre);
        c.align(4);
        let o0 = c.offset();
        c.read_f32()?; // airspeed
        c.read_f32()?; // groundspeed
        c.read_i16()?; // heading
        c.read_f32()?; // throttle (auto-aligns to 4)
        c.read_f32()?; // altitude
        c.read_f32()?; // climb
        Ok(VfrHud { offsets: [o0], buf })
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
    pub fn airspeed(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0])
    }
    #[inline]
    pub fn groundspeed(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 4)
    }
    #[inline]
    pub fn heading(&self) -> i16 {
        rd_i16(self.buf.as_ref(), self.offsets[0] + 8)
    }
    #[inline]
    pub fn throttle(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 12)
    }
    #[inline]
    pub fn altitude(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 16)
    }
    #[inline]
    pub fn climb(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 20)
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl VfrHud<Vec<u8>> {
    pub fn builder<'a>() -> VfrHudBuilder<'a> {
        VfrHudBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

pub struct VfrHudBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    airspeed: f32,
    groundspeed: f32,
    heading: i16,
    throttle: f32,
    altitude: f32,
    climb: f32,
}

impl<'a> Default for VfrHudBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            airspeed: 0.0,
            groundspeed: 0.0,
            heading: 0,
            throttle: 0.0,
            altitude: 0.0,
            climb: 0.0,
        }
    }
}

impl<'a> VfrHudBuilder<'a> {
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
    pub fn airspeed(&mut self, v: f32) -> &mut Self {
        self.airspeed = v;
        self
    }
    pub fn groundspeed(&mut self, v: f32) -> &mut Self {
        self.groundspeed = v;
        self
    }
    pub fn heading(&mut self, v: i16) -> &mut Self {
        self.heading = v;
        self
    }
    pub fn throttle(&mut self, v: f32) -> &mut Self {
        self.throttle = v;
        self
    }
    pub fn altitude(&mut self, v: f32) -> &mut Self {
        self.altitude = v;
        self
    }
    pub fn climb(&mut self, v: f32) -> &mut Self {
        self.climb = v;
        self
    }

    fn size(&self) -> usize {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(&self.frame_id);
        sizer.align(4);
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_i16();
        sizer.align(4);
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_f32();
        sizer.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.align(4);
        w.write_f32(self.airspeed);
        w.write_f32(self.groundspeed);
        w.write_i16(self.heading);
        w.align(4);
        w.write_f32(self.throttle);
        w.write_f32(self.altitude);
        w.write_f32(self.climb);
        w.finish()
    }

    pub fn build(&self) -> Result<VfrHud<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        VfrHud::from_cdr(buf)
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

// ── EstimatorStatus<B> ──────────────────────────────────────────────
//
// CDR layout: Header, then 12 × bool (each 1 byte), payload = 12B

pub struct EstimatorStatus<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> EstimatorStatus<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> EstimatorStatus<C> {
        EstimatorStatus {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> EstimatorStatus<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        for _ in 0..12 {
            c.read_bool()?;
        }
        Ok(EstimatorStatus { offsets: [o0], buf })
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
    pub fn attitude_status_flag(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0])
    }
    #[inline]
    pub fn velocity_horiz_status_flag(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 1)
    }
    #[inline]
    pub fn velocity_vert_status_flag(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 2)
    }
    #[inline]
    pub fn pos_horiz_rel_status_flag(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 3)
    }
    #[inline]
    pub fn pos_horiz_abs_status_flag(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 4)
    }
    #[inline]
    pub fn pos_vert_abs_status_flag(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 5)
    }
    #[inline]
    pub fn pos_vert_agl_status_flag(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 6)
    }
    #[inline]
    pub fn const_pos_mode_status_flag(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 7)
    }
    #[inline]
    pub fn pred_pos_horiz_rel_status_flag(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 8)
    }
    #[inline]
    pub fn pred_pos_horiz_abs_status_flag(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 9)
    }
    #[inline]
    pub fn gps_glitch_status_flag(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 10)
    }
    #[inline]
    pub fn accel_error_status_flag(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 11)
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl EstimatorStatus<Vec<u8>> {
    pub fn builder<'a>() -> EstimatorStatusBuilder<'a> {
        EstimatorStatusBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

pub struct EstimatorStatusBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    attitude_status_flag: bool,
    velocity_horiz_status_flag: bool,
    velocity_vert_status_flag: bool,
    pos_horiz_rel_status_flag: bool,
    pos_horiz_abs_status_flag: bool,
    pos_vert_abs_status_flag: bool,
    pos_vert_agl_status_flag: bool,
    const_pos_mode_status_flag: bool,
    pred_pos_horiz_rel_status_flag: bool,
    pred_pos_horiz_abs_status_flag: bool,
    gps_glitch_status_flag: bool,
    accel_error_status_flag: bool,
}

impl<'a> Default for EstimatorStatusBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            attitude_status_flag: false,
            velocity_horiz_status_flag: false,
            velocity_vert_status_flag: false,
            pos_horiz_rel_status_flag: false,
            pos_horiz_abs_status_flag: false,
            pos_vert_abs_status_flag: false,
            pos_vert_agl_status_flag: false,
            const_pos_mode_status_flag: false,
            pred_pos_horiz_rel_status_flag: false,
            pred_pos_horiz_abs_status_flag: false,
            gps_glitch_status_flag: false,
            accel_error_status_flag: false,
        }
    }
}

impl<'a> EstimatorStatusBuilder<'a> {
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
    pub fn attitude_status_flag(&mut self, v: bool) -> &mut Self {
        self.attitude_status_flag = v;
        self
    }
    pub fn velocity_horiz_status_flag(&mut self, v: bool) -> &mut Self {
        self.velocity_horiz_status_flag = v;
        self
    }
    pub fn velocity_vert_status_flag(&mut self, v: bool) -> &mut Self {
        self.velocity_vert_status_flag = v;
        self
    }
    pub fn pos_horiz_rel_status_flag(&mut self, v: bool) -> &mut Self {
        self.pos_horiz_rel_status_flag = v;
        self
    }
    pub fn pos_horiz_abs_status_flag(&mut self, v: bool) -> &mut Self {
        self.pos_horiz_abs_status_flag = v;
        self
    }
    pub fn pos_vert_abs_status_flag(&mut self, v: bool) -> &mut Self {
        self.pos_vert_abs_status_flag = v;
        self
    }
    pub fn pos_vert_agl_status_flag(&mut self, v: bool) -> &mut Self {
        self.pos_vert_agl_status_flag = v;
        self
    }
    pub fn const_pos_mode_status_flag(&mut self, v: bool) -> &mut Self {
        self.const_pos_mode_status_flag = v;
        self
    }
    pub fn pred_pos_horiz_rel_status_flag(&mut self, v: bool) -> &mut Self {
        self.pred_pos_horiz_rel_status_flag = v;
        self
    }
    pub fn pred_pos_horiz_abs_status_flag(&mut self, v: bool) -> &mut Self {
        self.pred_pos_horiz_abs_status_flag = v;
        self
    }
    pub fn gps_glitch_status_flag(&mut self, v: bool) -> &mut Self {
        self.gps_glitch_status_flag = v;
        self
    }
    pub fn accel_error_status_flag(&mut self, v: bool) -> &mut Self {
        self.accel_error_status_flag = v;
        self
    }

    fn size(&self) -> usize {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(&self.frame_id);
        for _ in 0..12 {
            sizer.size_bool();
        }
        sizer.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_bool(self.attitude_status_flag);
        w.write_bool(self.velocity_horiz_status_flag);
        w.write_bool(self.velocity_vert_status_flag);
        w.write_bool(self.pos_horiz_rel_status_flag);
        w.write_bool(self.pos_horiz_abs_status_flag);
        w.write_bool(self.pos_vert_abs_status_flag);
        w.write_bool(self.pos_vert_agl_status_flag);
        w.write_bool(self.const_pos_mode_status_flag);
        w.write_bool(self.pred_pos_horiz_rel_status_flag);
        w.write_bool(self.pred_pos_horiz_abs_status_flag);
        w.write_bool(self.gps_glitch_status_flag);
        w.write_bool(self.accel_error_status_flag);
        w.finish()
    }

    pub fn build(&self) -> Result<EstimatorStatus<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        EstimatorStatus::from_cdr(buf)
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

// ── ExtendedState<B> ────────────────────────────────────────────────
//
// CDR layout: Header, then:
//   u8 vtol_state     +0
//   u8 landed_state   +1
//   payload = 2 bytes

/// VTOL state constants.
pub mod vtol_state {
    pub const UNDEFINED: u8 = 0;
    pub const TRANSITION_TO_FW: u8 = 1;
    pub const TRANSITION_TO_MC: u8 = 2;
    pub const MC: u8 = 3;
    pub const FW: u8 = 4;
}

/// Landed state constants.
pub mod landed_state {
    pub const UNDEFINED: u8 = 0;
    pub const ON_GROUND: u8 = 1;
    pub const IN_AIR: u8 = 2;
    pub const TAKEOFF: u8 = 3;
    pub const LANDING: u8 = 4;
}

pub struct ExtendedState<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> ExtendedState<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> ExtendedState<C> {
        ExtendedState {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> ExtendedState<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        c.read_u8()?; // vtol_state
        c.read_u8()?; // landed_state
        Ok(ExtendedState { offsets: [o0], buf })
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
    pub fn vtol_state(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[0])
    }
    #[inline]
    pub fn landed_state(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[0] + 1)
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl ExtendedState<Vec<u8>> {
    pub fn builder<'a>() -> ExtendedStateBuilder<'a> {
        ExtendedStateBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

pub struct ExtendedStateBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    vtol_state: u8,
    landed_state: u8,
}

impl<'a> Default for ExtendedStateBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            vtol_state: 0,
            landed_state: 0,
        }
    }
}

impl<'a> ExtendedStateBuilder<'a> {
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
    pub fn vtol_state(&mut self, v: u8) -> &mut Self {
        self.vtol_state = v;
        self
    }
    pub fn landed_state(&mut self, v: u8) -> &mut Self {
        self.landed_state = v;
        self
    }

    fn size(&self) -> usize {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(&self.frame_id);
        sizer.size_u8();
        sizer.size_u8();
        sizer.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_u8(self.vtol_state);
        w.write_u8(self.landed_state);
        w.finish()
    }

    pub fn build(&self) -> Result<ExtendedState<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        ExtendedState::from_cdr(buf)
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

// ── SysStatus<B> ────────────────────────────────────────────────────
//
// CDR layout: Header, then:
//   align(4) → offsets[0]
//   u32 sensors_present      +0
//   u32 sensors_enabled      +4
//   u32 sensors_health       +8
//   u16 load                 +12
//   u16 voltage_battery      +14
//   i16 current_battery      +16
//   i8  battery_remaining    +18
//   [1B pad to 2-align]      +19
//   u16 drop_rate_comm       +20
//   u16 errors_comm          +22
//   u16 errors_count1        +24
//   u16 errors_count2        +26
//   u16 errors_count3        +28
//   u16 errors_count4        +30
//   payload = 32 bytes

/// MAVLink SYS_STATUS system health and power telemetry.
///
/// Field units follow the raw MAVLink SYS_STATUS message definition
/// (<https://mavlink.io/en/messages/common.html#SYS_STATUS>).
/// No unit conversions are applied — values are lossless from the wire.
pub struct SysStatus<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> SysStatus<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> SysStatus<C> {
        SysStatus {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> SysStatus<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let pre = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), pre);
        c.align(4);
        let o0 = c.offset();
        c.read_u32()?; // sensors_present
        c.read_u32()?; // sensors_enabled
        c.read_u32()?; // sensors_health
        c.read_u16()?; // load
        c.read_u16()?; // voltage_battery
        c.read_i16()?; // current_battery
        c.read_i8()?; // battery_remaining
        c.read_u16()?; // drop_rate_comm (auto-aligns to 2)
        c.read_u16()?; // errors_comm
        c.read_u16()?; // errors_count1
        c.read_u16()?; // errors_count2
        c.read_u16()?; // errors_count3
        c.read_u16()?; // errors_count4
        Ok(SysStatus { offsets: [o0], buf })
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
    /// Bitmask of onboard sensors present.
    #[inline]
    pub fn sensors_present(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0])
    }
    /// Bitmask of onboard sensors enabled.
    #[inline]
    pub fn sensors_enabled(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0] + 4)
    }
    /// Bitmask of onboard sensors reporting healthy.
    #[inline]
    pub fn sensors_health(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0] + 8)
    }
    /// Maximum CPU/MCU usage in per-mille (‰). 0–1000.
    #[inline]
    pub fn load(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 12)
    }
    /// Battery voltage in millivolts (mV). UINT16_MAX if unknown.
    #[inline]
    pub fn voltage_battery(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 14)
    }
    /// Battery current in centi-amperes (cA, 10 mA steps). −1 if unknown.
    #[inline]
    pub fn current_battery(&self) -> i16 {
        rd_i16(self.buf.as_ref(), self.offsets[0] + 16)
    }
    /// Battery remaining, 0–100 (%). −1 if not estimated.
    #[inline]
    pub fn battery_remaining(&self) -> i8 {
        rd_i8(self.buf.as_ref(), self.offsets[0] + 18)
    }
    /// Communication drop rate in per-mille (‰).
    #[inline]
    pub fn drop_rate_comm(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 20)
    }
    /// Communication error count.
    #[inline]
    pub fn errors_comm(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 22)
    }
    /// Autopilot-specific error count 1.
    #[inline]
    pub fn errors_count1(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 24)
    }
    /// Autopilot-specific error count 2.
    #[inline]
    pub fn errors_count2(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 26)
    }
    /// Autopilot-specific error count 3.
    #[inline]
    pub fn errors_count3(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 28)
    }
    /// Autopilot-specific error count 4.
    #[inline]
    pub fn errors_count4(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 30)
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl SysStatus<Vec<u8>> {
    pub fn builder<'a>() -> SysStatusBuilder<'a> {
        SysStatusBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

pub struct SysStatusBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    sensors_present: u32,
    sensors_enabled: u32,
    sensors_health: u32,
    load: u16,
    voltage_battery: u16,
    current_battery: i16,
    battery_remaining: i8,
    drop_rate_comm: u16,
    errors_comm: u16,
    errors_count1: u16,
    errors_count2: u16,
    errors_count3: u16,
    errors_count4: u16,
}

impl<'a> Default for SysStatusBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            sensors_present: 0,
            sensors_enabled: 0,
            sensors_health: 0,
            load: 0,
            voltage_battery: 0,
            current_battery: 0,
            battery_remaining: 0,
            drop_rate_comm: 0,
            errors_comm: 0,
            errors_count1: 0,
            errors_count2: 0,
            errors_count3: 0,
            errors_count4: 0,
        }
    }
}

impl<'a> SysStatusBuilder<'a> {
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
    pub fn sensors_present(&mut self, v: u32) -> &mut Self {
        self.sensors_present = v;
        self
    }
    pub fn sensors_enabled(&mut self, v: u32) -> &mut Self {
        self.sensors_enabled = v;
        self
    }
    pub fn sensors_health(&mut self, v: u32) -> &mut Self {
        self.sensors_health = v;
        self
    }
    pub fn load(&mut self, v: u16) -> &mut Self {
        self.load = v;
        self
    }
    pub fn voltage_battery(&mut self, v: u16) -> &mut Self {
        self.voltage_battery = v;
        self
    }
    pub fn current_battery(&mut self, v: i16) -> &mut Self {
        self.current_battery = v;
        self
    }
    pub fn battery_remaining(&mut self, v: i8) -> &mut Self {
        self.battery_remaining = v;
        self
    }
    pub fn drop_rate_comm(&mut self, v: u16) -> &mut Self {
        self.drop_rate_comm = v;
        self
    }
    pub fn errors_comm(&mut self, v: u16) -> &mut Self {
        self.errors_comm = v;
        self
    }
    pub fn errors_count1(&mut self, v: u16) -> &mut Self {
        self.errors_count1 = v;
        self
    }
    pub fn errors_count2(&mut self, v: u16) -> &mut Self {
        self.errors_count2 = v;
        self
    }
    pub fn errors_count3(&mut self, v: u16) -> &mut Self {
        self.errors_count3 = v;
        self
    }
    pub fn errors_count4(&mut self, v: u16) -> &mut Self {
        self.errors_count4 = v;
        self
    }

    fn size(&self) -> usize {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(&self.frame_id);
        sizer.align(4);
        sizer.size_u32();
        sizer.size_u32();
        sizer.size_u32();
        sizer.size_u16();
        sizer.size_u16();
        sizer.size_i16();
        sizer.size_i8();
        sizer.align(2);
        sizer.size_u16();
        sizer.size_u16();
        sizer.size_u16();
        sizer.size_u16();
        sizer.size_u16();
        sizer.size_u16();
        sizer.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.align(4);
        w.write_u32(self.sensors_present);
        w.write_u32(self.sensors_enabled);
        w.write_u32(self.sensors_health);
        w.write_u16(self.load);
        w.write_u16(self.voltage_battery);
        w.write_i16(self.current_battery);
        w.write_i8(self.battery_remaining);
        w.align(2);
        w.write_u16(self.drop_rate_comm);
        w.write_u16(self.errors_comm);
        w.write_u16(self.errors_count1);
        w.write_u16(self.errors_count2);
        w.write_u16(self.errors_count3);
        w.write_u16(self.errors_count4);
        w.finish()
    }

    pub fn build(&self) -> Result<SysStatus<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        SysStatus::from_cdr(buf)
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

// ── State<B> ─────────────────────────────────────────────────────────
//
// CDR layout: Header, then:
//   offsets[0] = start of bools
//   bool connected           +0
//   bool armed               +1
//   bool guided              +2
//   bool manual_input        +3
//   offsets[1] = start of mode string (aligned to 4)
//   string mode              (u32 len + chars + NUL)
//   offsets[2] = start of system_status (after mode string)
//   u8 system_status

/// MAV_STATE constants for system_status field.
pub mod mav_state {
    pub const UNINIT: u8 = 0;
    pub const BOOT: u8 = 1;
    pub const CALIBRATING: u8 = 2;
    pub const STANDBY: u8 = 3;
    pub const ACTIVE: u8 = 4;
    pub const CRITICAL: u8 = 5;
    pub const EMERGENCY: u8 = 6;
    pub const POWEROFF: u8 = 7;
    pub const FLIGHT_TERMINATION: u8 = 8;
}

pub struct State<B> {
    buf: B,
    offsets: [usize; 3],
}

impl<B> State<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> State<C> {
        State {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> State<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        c.read_bool()?; // connected
        c.read_bool()?; // armed
        c.read_bool()?; // guided
        c.read_bool()?; // manual_input
        c.align(4);
        let o1 = c.offset();
        let _ = c.read_string()?; // mode
        let o2 = c.offset();
        c.read_u8()?; // system_status
        Ok(State {
            offsets: [o0, o1, o2],
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
    #[inline]
    pub fn connected(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0])
    }
    #[inline]
    pub fn armed(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 1)
    }
    #[inline]
    pub fn guided(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 2)
    }
    #[inline]
    pub fn manual_input(&self) -> bool {
        rd_bool(self.buf.as_ref(), self.offsets[0] + 3)
    }
    #[inline]
    pub fn mode(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[1]).0
    }
    #[inline]
    pub fn system_status(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[2])
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl State<Vec<u8>> {
    pub fn builder<'a>() -> StateBuilder<'a> {
        StateBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

pub struct StateBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    connected: bool,
    armed: bool,
    guided: bool,
    manual_input: bool,
    mode: std::borrow::Cow<'a, str>,
    system_status: u8,
}

impl<'a> Default for StateBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            connected: false,
            armed: false,
            guided: false,
            manual_input: false,
            mode: std::borrow::Cow::Borrowed(""),
            system_status: 0,
        }
    }
}

impl<'a> StateBuilder<'a> {
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
    pub fn connected(&mut self, v: bool) -> &mut Self {
        self.connected = v;
        self
    }
    pub fn armed(&mut self, v: bool) -> &mut Self {
        self.armed = v;
        self
    }
    pub fn guided(&mut self, v: bool) -> &mut Self {
        self.guided = v;
        self
    }
    pub fn manual_input(&mut self, v: bool) -> &mut Self {
        self.manual_input = v;
        self
    }
    pub fn mode(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.mode = s.into();
        self
    }
    pub fn system_status(&mut self, v: u8) -> &mut Self {
        self.system_status = v;
        self
    }

    fn size(&self) -> usize {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(&self.frame_id);
        sizer.size_bool();
        sizer.size_bool();
        sizer.size_bool();
        sizer.size_bool();
        sizer.align(4);
        sizer.size_string(&self.mode);
        sizer.size_u8();
        sizer.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_bool(self.connected);
        w.write_bool(self.armed);
        w.write_bool(self.guided);
        w.write_bool(self.manual_input);
        w.write_string(&self.mode);
        w.write_u8(self.system_status);
        w.finish()
    }

    pub fn build(&self) -> Result<State<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        State::from_cdr(buf)
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

// ── StatusText<B> ───────────────────────────────────────────────────
//
// CDR layout: Header, then:
//   offsets[0] = start of severity
//   u8 severity             +0
//   string text             (variable: align(4) + u32 len + chars + NUL)
//   offsets[1] = start of text string (u32 len prefix)

/// Severity constants matching MAVLink MAV_SEVERITY.
pub mod severity {
    pub const EMERGENCY: u8 = 0;
    pub const ALERT: u8 = 1;
    pub const CRITICAL: u8 = 2;
    pub const ERROR: u8 = 3;
    pub const WARNING: u8 = 4;
    pub const NOTICE: u8 = 5;
    pub const INFO: u8 = 6;
    pub const DEBUG: u8 = 7;
}

pub struct StatusText<B> {
    buf: B,
    offsets: [usize; 2],
}

impl<B> StatusText<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> StatusText<C> {
        StatusText {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> StatusText<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        c.read_u8()?; // severity
        c.align(4);
        let o1 = c.offset();
        let _ = c.read_string()?; // text
        Ok(StatusText {
            offsets: [o0, o1],
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
    #[inline]
    pub fn severity(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[0])
    }
    #[inline]
    pub fn text(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[1]).0
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl StatusText<Vec<u8>> {
    pub fn builder<'a>() -> StatusTextBuilder<'a> {
        StatusTextBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

pub struct StatusTextBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    severity: u8,
    text: std::borrow::Cow<'a, str>,
}

impl<'a> Default for StatusTextBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            severity: 0,
            text: std::borrow::Cow::Borrowed(""),
        }
    }
}

impl<'a> StatusTextBuilder<'a> {
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
    pub fn severity(&mut self, v: u8) -> &mut Self {
        self.severity = v;
        self
    }
    pub fn text(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.text = s.into();
        self
    }

    fn size(&self) -> usize {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(&self.frame_id);
        sizer.size_u8();
        sizer.align(4);
        sizer.size_string(&self.text);
        sizer.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_u8(self.severity);
        w.align(4);
        w.write_string(&self.text);
        w.finish()
    }

    pub fn build(&self) -> Result<StatusText<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        StatusText::from_cdr(buf)
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

// ── GpsRaw<B> ───────────────────────────────────────────────────────
//
// CDR layout: Header, then:
//   align(4) → offsets[0]
//   u8  fix_type             +0
//   [3B pad]                 +1..+3
//   i32 lat                  +4
//   i32 lon                  +8
//   i32 alt                  +12
//   u16 eph                  +16
//   u16 epv                  +18
//   u16 vel                  +20
//   u16 cog                  +22
//   u8  satellites_visible   +24
//   [3B pad]                 +25..+27
//   i32 alt_ellipsoid        +28
//   u32 h_acc                +32
//   u32 v_acc                +36
//   u32 vel_acc              +40
//   i32 hdg_acc              +44
//   u16 yaw                  +48
//   u8  dgps_numch           +50
//   [1B pad]                 +51
//   u32 dgps_age             +52
//   payload = 56 bytes

/// GPS fix type constants.
pub mod gps_fix_type {
    pub const NO_GPS: u8 = 0;
    pub const NO_FIX: u8 = 1;
    pub const FIX_2D: u8 = 2;
    pub const FIX_3D: u8 = 3;
    pub const DGPS: u8 = 4;
    pub const RTK_FLOAT: u8 = 5;
    pub const RTK_FIXED: u8 = 6;
    pub const STATIC: u8 = 7;
    pub const PPP: u8 = 8;
}

/// MAVLink GPS_RAW_INT raw GNSS fix data.
///
/// Field units follow the raw MAVLink GPS_RAW_INT message definition
/// (<https://mavlink.io/en/messages/common.html#GPS_RAW_INT>).
/// No unit conversions are applied — values are lossless from the wire.
pub struct GpsRaw<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> GpsRaw<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> GpsRaw<C> {
        GpsRaw {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> GpsRaw<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let pre = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), pre);
        // Align to 4 so that subsequent i32 fields are correctly placed.
        // The u8 fix_type sits at the aligned offset; i32 lat follows with
        // automatic alignment by read_i32.
        c.align(4);
        let o0 = c.offset();
        c.read_u8()?; // fix_type
        c.read_i32()?; // lat (auto-aligns to 4)
        c.read_i32()?; // lon
        c.read_i32()?; // alt
        c.read_u16()?; // eph
        c.read_u16()?; // epv
        c.read_u16()?; // vel
        c.read_u16()?; // cog
        c.read_u8()?; // satellites_visible
        c.read_i32()?; // alt_ellipsoid (auto-aligns to 4)
        c.read_u32()?; // h_acc
        c.read_u32()?; // v_acc
        c.read_u32()?; // vel_acc
        c.read_i32()?; // hdg_acc
        c.read_u16()?; // yaw
        c.read_u8()?; // dgps_numch
        c.read_u32()?; // dgps_age (auto-aligns to 4)
        Ok(GpsRaw { offsets: [o0], buf })
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
    /// GPS fix type (see [`gps_fix_type`] constants).
    #[inline]
    pub fn fix_type(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[0])
    }
    /// Latitude in degrees × 10⁷ (degE7).
    #[inline]
    pub fn lat(&self) -> i32 {
        rd_i32(self.buf.as_ref(), self.offsets[0] + 4)
    }
    /// Longitude in degrees × 10⁷ (degE7).
    #[inline]
    pub fn lon(&self) -> i32 {
        rd_i32(self.buf.as_ref(), self.offsets[0] + 8)
    }
    /// Altitude above MSL in millimetres (mm).
    #[inline]
    pub fn alt(&self) -> i32 {
        rd_i32(self.buf.as_ref(), self.offsets[0] + 12)
    }
    /// GPS HDOP (horizontal dilution) in cm. UINT16_MAX if unknown.
    #[inline]
    pub fn eph(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 16)
    }
    /// GPS VDOP (vertical dilution) in cm. UINT16_MAX if unknown.
    #[inline]
    pub fn epv(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 18)
    }
    /// GPS ground speed in cm/s. UINT16_MAX if unknown.
    #[inline]
    pub fn vel(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 20)
    }
    /// Course over ground in centidegrees (cdeg, 0–35999). UINT16_MAX if unknown.
    #[inline]
    pub fn cog(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 22)
    }
    /// Number of satellites visible. UINT8_MAX if unknown.
    #[inline]
    pub fn satellites_visible(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[0] + 24)
    }
    /// Altitude above WGS84 ellipsoid in millimetres (mm).
    #[inline]
    pub fn alt_ellipsoid(&self) -> i32 {
        rd_i32(self.buf.as_ref(), self.offsets[0] + 28)
    }
    /// Position uncertainty (horizontal) in millimetres (mm).
    #[inline]
    pub fn h_acc(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0] + 32)
    }
    /// Position uncertainty (vertical) in millimetres (mm).
    #[inline]
    pub fn v_acc(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0] + 36)
    }
    /// Speed uncertainty in millimetres per second (mm/s).
    #[inline]
    pub fn vel_acc(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0] + 40)
    }
    /// Heading uncertainty in centidegrees (cdeg × 10⁻⁵).
    #[inline]
    pub fn hdg_acc(&self) -> i32 {
        rd_i32(self.buf.as_ref(), self.offsets[0] + 44)
    }
    /// Yaw in centidegrees (cdeg). 0 if unknown.
    #[inline]
    pub fn yaw(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 48)
    }
    /// Number of DGPS satellites.
    #[inline]
    pub fn dgps_numch(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[0] + 50)
    }
    /// Age of DGPS correction in milliseconds (ms).
    #[inline]
    pub fn dgps_age(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0] + 52)
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl GpsRaw<Vec<u8>> {
    pub fn builder<'a>() -> GpsRawBuilder<'a> {
        GpsRawBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

pub struct GpsRawBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    fix_type: u8,
    lat: i32,
    lon: i32,
    alt: i32,
    eph: u16,
    epv: u16,
    vel: u16,
    cog: u16,
    satellites_visible: u8,
    alt_ellipsoid: i32,
    h_acc: u32,
    v_acc: u32,
    vel_acc: u32,
    hdg_acc: i32,
    yaw: u16,
    dgps_numch: u8,
    dgps_age: u32,
}

impl<'a> Default for GpsRawBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            fix_type: 0,
            lat: 0,
            lon: 0,
            alt: 0,
            eph: 0,
            epv: 0,
            vel: 0,
            cog: 0,
            satellites_visible: 0,
            alt_ellipsoid: 0,
            h_acc: 0,
            v_acc: 0,
            vel_acc: 0,
            hdg_acc: 0,
            yaw: 0,
            dgps_numch: 0,
            dgps_age: 0,
        }
    }
}

impl<'a> GpsRawBuilder<'a> {
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
    pub fn fix_type(&mut self, v: u8) -> &mut Self {
        self.fix_type = v;
        self
    }
    pub fn lat(&mut self, v: i32) -> &mut Self {
        self.lat = v;
        self
    }
    pub fn lon(&mut self, v: i32) -> &mut Self {
        self.lon = v;
        self
    }
    pub fn alt(&mut self, v: i32) -> &mut Self {
        self.alt = v;
        self
    }
    pub fn eph(&mut self, v: u16) -> &mut Self {
        self.eph = v;
        self
    }
    pub fn epv(&mut self, v: u16) -> &mut Self {
        self.epv = v;
        self
    }
    pub fn vel(&mut self, v: u16) -> &mut Self {
        self.vel = v;
        self
    }
    pub fn cog(&mut self, v: u16) -> &mut Self {
        self.cog = v;
        self
    }
    pub fn satellites_visible(&mut self, v: u8) -> &mut Self {
        self.satellites_visible = v;
        self
    }
    pub fn alt_ellipsoid(&mut self, v: i32) -> &mut Self {
        self.alt_ellipsoid = v;
        self
    }
    pub fn h_acc(&mut self, v: u32) -> &mut Self {
        self.h_acc = v;
        self
    }
    pub fn v_acc(&mut self, v: u32) -> &mut Self {
        self.v_acc = v;
        self
    }
    pub fn vel_acc(&mut self, v: u32) -> &mut Self {
        self.vel_acc = v;
        self
    }
    pub fn hdg_acc(&mut self, v: i32) -> &mut Self {
        self.hdg_acc = v;
        self
    }
    pub fn yaw(&mut self, v: u16) -> &mut Self {
        self.yaw = v;
        self
    }
    pub fn dgps_numch(&mut self, v: u8) -> &mut Self {
        self.dgps_numch = v;
        self
    }
    pub fn dgps_age(&mut self, v: u32) -> &mut Self {
        self.dgps_age = v;
        self
    }

    fn size(&self) -> usize {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(&self.frame_id);
        sizer.align(4);
        sizer.size_u8();
        sizer.align(4);
        sizer.size_i32();
        sizer.size_i32();
        sizer.size_i32();
        sizer.size_u16();
        sizer.size_u16();
        sizer.size_u16();
        sizer.size_u16();
        sizer.size_u8();
        sizer.align(4);
        sizer.size_i32();
        sizer.size_u32();
        sizer.size_u32();
        sizer.size_u32();
        sizer.size_i32();
        sizer.size_u16();
        sizer.size_u8();
        sizer.align(4);
        sizer.size_u32();
        sizer.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.align(4);
        w.write_u8(self.fix_type);
        w.align(4);
        w.write_i32(self.lat);
        w.write_i32(self.lon);
        w.write_i32(self.alt);
        w.write_u16(self.eph);
        w.write_u16(self.epv);
        w.write_u16(self.vel);
        w.write_u16(self.cog);
        w.write_u8(self.satellites_visible);
        w.align(4);
        w.write_i32(self.alt_ellipsoid);
        w.write_u32(self.h_acc);
        w.write_u32(self.v_acc);
        w.write_u32(self.vel_acc);
        w.write_i32(self.hdg_acc);
        w.write_u16(self.yaw);
        w.write_u8(self.dgps_numch);
        w.align(4);
        w.write_u32(self.dgps_age);
        w.finish()
    }

    pub fn build(&self) -> Result<GpsRaw<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        GpsRaw::from_cdr(buf)
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

// ── TimesyncStatus<B> ───────────────────────────────────────────────
//
// CDR layout: Header, then:
//   align(8) → offsets[0]
//   u64 remote_timestamp_ns    +0
//   i64 observed_offset_ns     +8
//   i64 estimated_offset_ns    +16
//   f32 round_trip_time_ms     +24 (needs align(4), already aligned since +24 from 8-aligned base)
//   payload = 28 bytes

pub struct TimesyncStatus<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B> TimesyncStatus<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> TimesyncStatus<C> {
        TimesyncStatus {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> TimesyncStatus<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let pre = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), pre);
        c.align(8);
        let o0 = c.offset();
        c.read_u64()?; // remote_timestamp_ns
        c.read_i64()?; // observed_offset_ns
        c.read_i64()?; // estimated_offset_ns
        c.read_f32()?; // round_trip_time_ms
        Ok(TimesyncStatus { offsets: [o0], buf })
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
    pub fn remote_timestamp_ns(&self) -> u64 {
        rd_u64(self.buf.as_ref(), self.offsets[0])
    }
    #[inline]
    pub fn observed_offset_ns(&self) -> i64 {
        rd_i64(self.buf.as_ref(), self.offsets[0] + 8)
    }
    #[inline]
    pub fn estimated_offset_ns(&self) -> i64 {
        rd_i64(self.buf.as_ref(), self.offsets[0] + 16)
    }
    #[inline]
    pub fn round_trip_time_ms(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 24)
    }
    #[inline]
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl TimesyncStatus<Vec<u8>> {
    pub fn builder<'a>() -> TimesyncStatusBuilder<'a> {
        TimesyncStatusBuilder::new()
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

pub struct TimesyncStatusBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    remote_timestamp_ns: u64,
    observed_offset_ns: i64,
    estimated_offset_ns: i64,
    round_trip_time_ms: f32,
}

impl<'a> Default for TimesyncStatusBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            remote_timestamp_ns: 0,
            observed_offset_ns: 0,
            estimated_offset_ns: 0,
            round_trip_time_ms: 0.0,
        }
    }
}

impl<'a> TimesyncStatusBuilder<'a> {
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
    pub fn remote_timestamp_ns(&mut self, v: u64) -> &mut Self {
        self.remote_timestamp_ns = v;
        self
    }
    pub fn observed_offset_ns(&mut self, v: i64) -> &mut Self {
        self.observed_offset_ns = v;
        self
    }
    pub fn estimated_offset_ns(&mut self, v: i64) -> &mut Self {
        self.estimated_offset_ns = v;
        self
    }
    pub fn round_trip_time_ms(&mut self, v: f32) -> &mut Self {
        self.round_trip_time_ms = v;
        self
    }

    fn size(&self) -> usize {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(&self.frame_id);
        sizer.align(8);
        sizer.size_u64();
        sizer.size_i64();
        sizer.size_i64();
        sizer.size_f32();
        sizer.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.align(8);
        w.write_u64(self.remote_timestamp_ns);
        w.write_i64(self.observed_offset_ns);
        w.write_i64(self.estimated_offset_ns);
        w.write_f32(self.round_trip_time_ms);
        w.finish()
    }

    pub fn build(&self) -> Result<TimesyncStatus<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        TimesyncStatus::from_cdr(buf)
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
        "Altitude"
            | "VfrHud"
            | "EstimatorStatus"
            | "ExtendedState"
            | "SysStatus"
            | "State"
            | "StatusText"
            | "GPSRAW"
            | "TimesyncStatus"
    )
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &[
        "mavros_msgs/msg/Altitude",
        "mavros_msgs/msg/VfrHud",
        "mavros_msgs/msg/EstimatorStatus",
        "mavros_msgs/msg/ExtendedState",
        "mavros_msgs/msg/SysStatus",
        "mavros_msgs/msg/State",
        "mavros_msgs/msg/StatusText",
        "mavros_msgs/msg/GPSRAW",
        "mavros_msgs/msg/TimesyncStatus",
    ]
}

// ── Tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;

    fn test_stamp() -> Time {
        Time {
            sec: 1234,
            nanosec: 5678,
        }
    }

    #[test]
    fn test_altitude_round_trip() {
        let msg = Altitude::builder()
            .stamp(test_stamp())
            .frame_id("map")
            .monotonic(100.0)
            .amsl(200.0)
            .local(50.0)
            .relative(150.0)
            .terrain(45.0)
            .bottom_clearance(10.0)
            .build()
            .unwrap();
        let parsed = Altitude::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.stamp(), test_stamp());
        assert_eq!(parsed.frame_id(), "map");
        assert_eq!(parsed.monotonic(), 100.0);
        assert_eq!(parsed.amsl(), 200.0);
        assert_eq!(parsed.local(), 50.0);
        assert_eq!(parsed.relative(), 150.0);
        assert_eq!(parsed.terrain(), 45.0);
        assert_eq!(parsed.bottom_clearance(), 10.0);
    }

    #[test]
    fn test_vfrhud_round_trip() {
        let msg = VfrHud::builder()
            .stamp(test_stamp())
            .frame_id("base_link")
            .airspeed(15.5)
            .groundspeed(12.3)
            .heading(180)
            .throttle(0.75)
            .altitude(100.0)
            .climb(2.5)
            .build()
            .unwrap();
        let parsed = VfrHud::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.stamp(), test_stamp());
        assert_eq!(parsed.frame_id(), "base_link");
        assert_eq!(parsed.airspeed(), 15.5);
        assert_eq!(parsed.groundspeed(), 12.3);
        assert_eq!(parsed.heading(), 180);
        assert_eq!(parsed.throttle(), 0.75);
        assert_eq!(parsed.altitude(), 100.0);
        assert_eq!(parsed.climb(), 2.5);
    }

    #[test]
    fn test_estimator_status_round_trip() {
        let msg = EstimatorStatus::builder()
            .stamp(test_stamp())
            .frame_id("fcu")
            .attitude_status_flag(true)
            .velocity_horiz_status_flag(true)
            .velocity_vert_status_flag(false)
            .pos_horiz_rel_status_flag(true)
            .pos_horiz_abs_status_flag(true)
            .pos_vert_abs_status_flag(false)
            .pos_vert_agl_status_flag(false)
            .const_pos_mode_status_flag(false)
            .pred_pos_horiz_rel_status_flag(true)
            .pred_pos_horiz_abs_status_flag(true)
            .gps_glitch_status_flag(false)
            .accel_error_status_flag(false)
            .build()
            .unwrap();
        let parsed = EstimatorStatus::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.frame_id(), "fcu");
        assert!(parsed.attitude_status_flag());
        assert!(parsed.velocity_horiz_status_flag());
        assert!(!parsed.velocity_vert_status_flag());
        assert!(parsed.pos_horiz_rel_status_flag());
        assert!(parsed.pos_horiz_abs_status_flag());
        assert!(!parsed.pos_vert_abs_status_flag());
        assert!(!parsed.pos_vert_agl_status_flag());
        assert!(!parsed.const_pos_mode_status_flag());
        assert!(parsed.pred_pos_horiz_rel_status_flag());
        assert!(parsed.pred_pos_horiz_abs_status_flag());
        assert!(!parsed.gps_glitch_status_flag());
        assert!(!parsed.accel_error_status_flag());
    }

    #[test]
    fn test_extended_state_round_trip() {
        let msg = ExtendedState::builder()
            .stamp(test_stamp())
            .frame_id("fcu")
            .vtol_state(vtol_state::MC)
            .landed_state(landed_state::IN_AIR)
            .build()
            .unwrap();
        let parsed = ExtendedState::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.frame_id(), "fcu");
        assert_eq!(parsed.vtol_state(), vtol_state::MC);
        assert_eq!(parsed.landed_state(), landed_state::IN_AIR);
    }

    #[test]
    fn test_sys_status_round_trip() {
        let msg = SysStatus::builder()
            .stamp(test_stamp())
            .frame_id("fcu")
            .sensors_present(0xFFFF_FFFF)
            .sensors_enabled(0x0000_FFFF)
            .sensors_health(0x0000_00FF)
            .load(500)
            .voltage_battery(12600)
            .current_battery(-1500)
            .battery_remaining(75)
            .drop_rate_comm(10)
            .errors_comm(2)
            .errors_count1(0)
            .errors_count2(0)
            .errors_count3(1)
            .errors_count4(0)
            .build()
            .unwrap();
        let parsed = SysStatus::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.frame_id(), "fcu");
        assert_eq!(parsed.sensors_present(), 0xFFFF_FFFF);
        assert_eq!(parsed.sensors_enabled(), 0x0000_FFFF);
        assert_eq!(parsed.sensors_health(), 0x0000_00FF);
        assert_eq!(parsed.load(), 500);
        assert_eq!(parsed.voltage_battery(), 12600);
        assert_eq!(parsed.current_battery(), -1500);
        assert_eq!(parsed.battery_remaining(), 75);
        assert_eq!(parsed.drop_rate_comm(), 10);
        assert_eq!(parsed.errors_comm(), 2);
        assert_eq!(parsed.errors_count1(), 0);
        assert_eq!(parsed.errors_count2(), 0);
        assert_eq!(parsed.errors_count3(), 1);
        assert_eq!(parsed.errors_count4(), 0);
    }

    #[test]
    fn test_state_round_trip() {
        let msg = State::builder()
            .stamp(test_stamp())
            .frame_id("fcu")
            .connected(true)
            .armed(true)
            .guided(false)
            .manual_input(false)
            .mode("GUIDED")
            .system_status(mav_state::ACTIVE)
            .build()
            .unwrap();
        let parsed = State::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.frame_id(), "fcu");
        assert!(parsed.connected());
        assert!(parsed.armed());
        assert!(!parsed.guided());
        assert!(!parsed.manual_input());
        assert_eq!(parsed.mode(), "GUIDED");
        assert_eq!(parsed.system_status(), mav_state::ACTIVE);
    }

    #[test]
    fn test_state_empty_mode() {
        let msg = State::builder()
            .stamp(test_stamp())
            .frame_id("")
            .connected(false)
            .armed(false)
            .guided(false)
            .manual_input(false)
            .mode("")
            .system_status(mav_state::UNINIT)
            .build()
            .unwrap();
        let parsed = State::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.mode(), "");
        assert_eq!(parsed.system_status(), mav_state::UNINIT);
    }

    #[test]
    fn test_status_text_round_trip() {
        let msg = StatusText::builder()
            .stamp(test_stamp())
            .frame_id("fcu")
            .severity(severity::WARNING)
            .text("Low battery warning")
            .build()
            .unwrap();
        let parsed = StatusText::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.frame_id(), "fcu");
        assert_eq!(parsed.severity(), severity::WARNING);
        assert_eq!(parsed.text(), "Low battery warning");
    }

    #[test]
    fn test_gps_raw_round_trip() {
        let msg = GpsRaw::builder()
            .stamp(test_stamp())
            .frame_id("gps")
            .fix_type(gps_fix_type::FIX_3D)
            .lat(473977070)
            .lon(85512540)
            .alt(408000)
            .eph(120)
            .epv(150)
            .vel(500)
            .cog(9000)
            .satellites_visible(12)
            .alt_ellipsoid(408500)
            .h_acc(1000)
            .v_acc(1500)
            .vel_acc(200)
            .hdg_acc(50000)
            .yaw(9000)
            .dgps_numch(4)
            .dgps_age(1000)
            .build()
            .unwrap();
        let parsed = GpsRaw::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.frame_id(), "gps");
        assert_eq!(parsed.fix_type(), gps_fix_type::FIX_3D);
        assert_eq!(parsed.lat(), 473977070);
        assert_eq!(parsed.lon(), 85512540);
        assert_eq!(parsed.alt(), 408000);
        assert_eq!(parsed.eph(), 120);
        assert_eq!(parsed.epv(), 150);
        assert_eq!(parsed.vel(), 500);
        assert_eq!(parsed.cog(), 9000);
        assert_eq!(parsed.satellites_visible(), 12);
        assert_eq!(parsed.alt_ellipsoid(), 408500);
        assert_eq!(parsed.h_acc(), 1000);
        assert_eq!(parsed.v_acc(), 1500);
        assert_eq!(parsed.vel_acc(), 200);
        assert_eq!(parsed.hdg_acc(), 50000);
        assert_eq!(parsed.yaw(), 9000);
        assert_eq!(parsed.dgps_numch(), 4);
        assert_eq!(parsed.dgps_age(), 1000);
    }

    #[test]
    fn test_timesync_status_round_trip() {
        let msg = TimesyncStatus::builder()
            .stamp(test_stamp())
            .frame_id("fcu")
            .remote_timestamp_ns(1_000_000_000)
            .observed_offset_ns(-5000)
            .estimated_offset_ns(-4500)
            .round_trip_time_ms(2.5)
            .build()
            .unwrap();
        let parsed = TimesyncStatus::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.frame_id(), "fcu");
        assert_eq!(parsed.remote_timestamp_ns(), 1_000_000_000);
        assert_eq!(parsed.observed_offset_ns(), -5000);
        assert_eq!(parsed.estimated_offset_ns(), -4500);
        assert_eq!(parsed.round_trip_time_ms(), 2.5);
    }

    #[test]
    fn test_buffer_too_short() {
        assert!(Altitude::from_cdr(&[0u8; 4]).is_err());
        assert!(VfrHud::from_cdr(&[0u8; 4]).is_err());
        assert!(State::from_cdr(&[0u8; 4]).is_err());
        assert!(GpsRaw::from_cdr(&[0u8; 4]).is_err());
        assert!(TimesyncStatus::from_cdr(&[0u8; 4]).is_err());
    }

    #[test]
    fn test_owned_into_cdr() {
        let msg = ExtendedState::builder()
            .stamp(test_stamp())
            .frame_id("x")
            .vtol_state(0)
            .landed_state(1)
            .build()
            .unwrap();
        let cdr = msg.to_cdr();
        let owned = ExtendedState::builder()
            .stamp(test_stamp())
            .frame_id("x")
            .vtol_state(0)
            .landed_state(1)
            .build()
            .unwrap();
        assert_eq!(owned.into_cdr(), cdr);
    }
}
