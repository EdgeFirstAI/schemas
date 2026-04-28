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
    pub fn new(
        stamp: Time,
        frame_id: &str,
        monotonic: f32,
        amsl: f32,
        local: f32,
        relative: f32,
        terrain: f32,
        bottom_clearance: f32,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        sizer.align(4);
        let o0 = sizer.offset();
        sizer.size_f32(); // monotonic
        sizer.size_f32(); // amsl
        sizer.size_f32(); // local
        sizer.size_f32(); // relative
        sizer.size_f32(); // terrain
        sizer.size_f32(); // bottom_clearance

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.align(4);
        w.write_f32(monotonic);
        w.write_f32(amsl);
        w.write_f32(local);
        w.write_f32(relative);
        w.write_f32(terrain);
        w.write_f32(bottom_clearance);
        w.finish()?;

        Ok(Altitude { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
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
    pub fn new(
        stamp: Time,
        frame_id: &str,
        airspeed: f32,
        groundspeed: f32,
        heading: i16,
        throttle: f32,
        altitude: f32,
        climb: f32,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        sizer.align(4);
        let o0 = sizer.offset();
        sizer.size_f32(); // airspeed
        sizer.size_f32(); // groundspeed
        sizer.size_i16(); // heading
        sizer.align(4);
        sizer.size_f32(); // throttle
        sizer.size_f32(); // altitude
        sizer.size_f32(); // climb

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.align(4);
        w.write_f32(airspeed);
        w.write_f32(groundspeed);
        w.write_i16(heading);
        w.align(4);
        w.write_f32(throttle);
        w.write_f32(altitude);
        w.write_f32(climb);
        w.finish()?;

        Ok(VfrHud { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

// ── EstimatorStatus<B> ──────────────────────────────────────────────
//
// CDR layout: Header, then 12 × bool (each 1 byte), payload = 12B

pub struct EstimatorStatus<B> {
    buf: B,
    offsets: [usize; 1],
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
    pub fn new(
        stamp: Time,
        frame_id: &str,
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
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        for _ in 0..12 {
            sizer.size_bool();
        }

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_bool(attitude_status_flag);
        w.write_bool(velocity_horiz_status_flag);
        w.write_bool(velocity_vert_status_flag);
        w.write_bool(pos_horiz_rel_status_flag);
        w.write_bool(pos_horiz_abs_status_flag);
        w.write_bool(pos_vert_abs_status_flag);
        w.write_bool(pos_vert_agl_status_flag);
        w.write_bool(const_pos_mode_status_flag);
        w.write_bool(pred_pos_horiz_rel_status_flag);
        w.write_bool(pred_pos_horiz_abs_status_flag);
        w.write_bool(gps_glitch_status_flag);
        w.write_bool(accel_error_status_flag);
        w.finish()?;

        Ok(EstimatorStatus { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
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
    pub fn new(
        stamp: Time,
        frame_id: &str,
        vtol_state: u8,
        landed_state: u8,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_u8();
        sizer.size_u8();

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_u8(vtol_state);
        w.write_u8(landed_state);
        w.finish()?;

        Ok(ExtendedState { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
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

pub struct SysStatus<B> {
    buf: B,
    offsets: [usize; 1],
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
    #[inline]
    pub fn sensors_present(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0])
    }
    #[inline]
    pub fn sensors_enabled(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0] + 4)
    }
    #[inline]
    pub fn sensors_health(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0] + 8)
    }
    #[inline]
    pub fn load(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 12)
    }
    #[inline]
    pub fn voltage_battery(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 14)
    }
    #[inline]
    pub fn current_battery(&self) -> i16 {
        rd_i16(self.buf.as_ref(), self.offsets[0] + 16)
    }
    #[inline]
    pub fn battery_remaining(&self) -> i8 {
        rd_i8(self.buf.as_ref(), self.offsets[0] + 18)
    }
    #[inline]
    pub fn drop_rate_comm(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 20)
    }
    #[inline]
    pub fn errors_comm(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 22)
    }
    #[inline]
    pub fn errors_count1(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 24)
    }
    #[inline]
    pub fn errors_count2(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 26)
    }
    #[inline]
    pub fn errors_count3(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 28)
    }
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
    pub fn new(
        stamp: Time,
        frame_id: &str,
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
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        sizer.align(4);
        let o0 = sizer.offset();
        sizer.size_u32(); // sensors_present
        sizer.size_u32(); // sensors_enabled
        sizer.size_u32(); // sensors_health
        sizer.size_u16(); // load
        sizer.size_u16(); // voltage_battery
        sizer.size_i16(); // current_battery
        sizer.size_i8(); // battery_remaining
        sizer.align(2);
        sizer.size_u16(); // drop_rate_comm
        sizer.size_u16(); // errors_comm
        sizer.size_u16(); // errors_count1
        sizer.size_u16(); // errors_count2
        sizer.size_u16(); // errors_count3
        sizer.size_u16(); // errors_count4

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.align(4);
        w.write_u32(sensors_present);
        w.write_u32(sensors_enabled);
        w.write_u32(sensors_health);
        w.write_u16(load);
        w.write_u16(voltage_battery);
        w.write_i16(current_battery);
        w.write_i8(battery_remaining);
        w.align(2);
        w.write_u16(drop_rate_comm);
        w.write_u16(errors_comm);
        w.write_u16(errors_count1);
        w.write_u16(errors_count2);
        w.write_u16(errors_count3);
        w.write_u16(errors_count4);
        w.finish()?;

        Ok(SysStatus { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
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
//   string mode              (variable-length, starts with align(4) + u32 len)
//   offsets[1] = start of system_status (after mode string)
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
    offsets: [usize; 2],
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
        let _ = c.read_string()?; // mode
        let o1 = c.offset();
        c.read_u8()?; // system_status
        Ok(State {
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
        rd_string(self.buf.as_ref(), self.offsets[0] + 4).0
    }
    #[inline]
    pub fn system_status(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[1])
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
    pub fn new(
        stamp: Time,
        frame_id: &str,
        connected: bool,
        armed: bool,
        guided: bool,
        manual_input: bool,
        mode: &str,
        system_status: u8,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_bool(); // connected
        sizer.size_bool(); // armed
        sizer.size_bool(); // guided
        sizer.size_bool(); // manual_input
        sizer.size_string(mode);
        let o1 = sizer.offset();
        sizer.size_u8(); // system_status

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_bool(connected);
        w.write_bool(armed);
        w.write_bool(guided);
        w.write_bool(manual_input);
        w.write_string(mode);
        w.write_u8(system_status);
        w.finish()?;

        Ok(State {
            offsets: [o0, o1],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
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
    pub fn new(stamp: Time, frame_id: &str, severity: u8, text: &str) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_u8(); // severity
        sizer.align(4);
        let o1 = sizer.offset();
        sizer.size_string(text);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_u8(severity);
        w.align(4);
        w.write_string(text);
        w.finish()?;

        Ok(StatusText {
            offsets: [o0, o1],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
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

pub struct GpsRaw<B> {
    buf: B,
    offsets: [usize; 1],
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
    #[inline]
    pub fn fix_type(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[0])
    }
    #[inline]
    pub fn lat(&self) -> i32 {
        rd_i32(self.buf.as_ref(), self.offsets[0] + 4)
    }
    #[inline]
    pub fn lon(&self) -> i32 {
        rd_i32(self.buf.as_ref(), self.offsets[0] + 8)
    }
    #[inline]
    pub fn alt(&self) -> i32 {
        rd_i32(self.buf.as_ref(), self.offsets[0] + 12)
    }
    #[inline]
    pub fn eph(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 16)
    }
    #[inline]
    pub fn epv(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 18)
    }
    #[inline]
    pub fn vel(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 20)
    }
    #[inline]
    pub fn cog(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 22)
    }
    #[inline]
    pub fn satellites_visible(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[0] + 24)
    }
    #[inline]
    pub fn alt_ellipsoid(&self) -> i32 {
        rd_i32(self.buf.as_ref(), self.offsets[0] + 28)
    }
    #[inline]
    pub fn h_acc(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0] + 32)
    }
    #[inline]
    pub fn v_acc(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0] + 36)
    }
    #[inline]
    pub fn vel_acc(&self) -> u32 {
        rd_u32(self.buf.as_ref(), self.offsets[0] + 40)
    }
    #[inline]
    pub fn hdg_acc(&self) -> i32 {
        rd_i32(self.buf.as_ref(), self.offsets[0] + 44)
    }
    #[inline]
    pub fn yaw(&self) -> u16 {
        rd_u16(self.buf.as_ref(), self.offsets[0] + 48)
    }
    #[inline]
    pub fn dgps_numch(&self) -> u8 {
        rd_u8(self.buf.as_ref(), self.offsets[0] + 50)
    }
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
    pub fn new(
        stamp: Time,
        frame_id: &str,
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
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        sizer.align(4);
        let o0 = sizer.offset();
        sizer.size_u8(); // fix_type
        sizer.align(4);
        sizer.size_i32(); // lat
        sizer.size_i32(); // lon
        sizer.size_i32(); // alt
        sizer.size_u16(); // eph
        sizer.size_u16(); // epv
        sizer.size_u16(); // vel
        sizer.size_u16(); // cog
        sizer.size_u8(); // satellites_visible
        sizer.align(4);
        sizer.size_i32(); // alt_ellipsoid
        sizer.size_u32(); // h_acc
        sizer.size_u32(); // v_acc
        sizer.size_u32(); // vel_acc
        sizer.size_i32(); // hdg_acc
        sizer.size_u16(); // yaw
        sizer.size_u8(); // dgps_numch
        sizer.align(4);
        sizer.size_u32(); // dgps_age

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.align(4);
        w.write_u8(fix_type);
        w.align(4);
        w.write_i32(lat);
        w.write_i32(lon);
        w.write_i32(alt);
        w.write_u16(eph);
        w.write_u16(epv);
        w.write_u16(vel);
        w.write_u16(cog);
        w.write_u8(satellites_visible);
        w.align(4);
        w.write_i32(alt_ellipsoid);
        w.write_u32(h_acc);
        w.write_u32(v_acc);
        w.write_u32(vel_acc);
        w.write_i32(hdg_acc);
        w.write_u16(yaw);
        w.write_u8(dgps_numch);
        w.align(4);
        w.write_u32(dgps_age);
        w.finish()?;

        Ok(GpsRaw { offsets: [o0], buf })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
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
    pub fn new(
        stamp: Time,
        frame_id: &str,
        remote_timestamp_ns: u64,
        observed_offset_ns: i64,
        estimated_offset_ns: i64,
        round_trip_time_ms: f32,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        sizer.align(8);
        let o0 = sizer.offset();
        sizer.size_u64(); // remote_timestamp_ns
        sizer.size_i64(); // observed_offset_ns
        sizer.size_i64(); // estimated_offset_ns
        sizer.size_f32(); // round_trip_time_ms

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.align(8);
        w.write_u64(remote_timestamp_ns);
        w.write_i64(observed_offset_ns);
        w.write_i64(estimated_offset_ns);
        w.write_f32(round_trip_time_ms);
        w.finish()?;

        Ok(TimesyncStatus { offsets: [o0], buf })
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
        let msg =
            Altitude::new(test_stamp(), "map", 100.0, 200.0, 50.0, 150.0, 45.0, 10.0).unwrap();
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
        let msg =
            VfrHud::new(test_stamp(), "base_link", 15.5, 12.3, 180, 0.75, 100.0, 2.5).unwrap();
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
        let msg = EstimatorStatus::new(
            test_stamp(),
            "fcu",
            true,  // attitude
            true,  // vel_horiz
            false, // vel_vert
            true,  // pos_horiz_rel
            true,  // pos_horiz_abs
            false, // pos_vert_abs
            false, // pos_vert_agl
            false, // const_pos_mode
            true,  // pred_pos_horiz_rel
            true,  // pred_pos_horiz_abs
            false, // gps_glitch
            false, // accel_error
        )
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
        let msg =
            ExtendedState::new(test_stamp(), "fcu", vtol_state::MC, landed_state::IN_AIR).unwrap();
        let parsed = ExtendedState::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.frame_id(), "fcu");
        assert_eq!(parsed.vtol_state(), vtol_state::MC);
        assert_eq!(parsed.landed_state(), landed_state::IN_AIR);
    }

    #[test]
    fn test_sys_status_round_trip() {
        let msg = SysStatus::new(
            test_stamp(),
            "fcu",
            0xFFFF_FFFF, // sensors_present
            0x0000_FFFF, // sensors_enabled
            0x0000_00FF, // sensors_health
            500,         // load (0.1%)
            12600,       // voltage_battery (mV)
            -1500,       // current_battery (cA)
            75,          // battery_remaining (%)
            10,          // drop_rate_comm
            2,           // errors_comm
            0,           // errors_count1
            0,           // errors_count2
            1,           // errors_count3
            0,           // errors_count4
        )
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
        let msg = State::new(
            test_stamp(),
            "fcu",
            true,
            true,
            false,
            false,
            "GUIDED",
            mav_state::ACTIVE,
        )
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
        let msg = State::new(
            test_stamp(),
            "",
            false,
            false,
            false,
            false,
            "",
            mav_state::UNINIT,
        )
        .unwrap();
        let parsed = State::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.mode(), "");
        assert_eq!(parsed.system_status(), mav_state::UNINIT);
    }

    #[test]
    fn test_status_text_round_trip() {
        let msg = StatusText::new(
            test_stamp(),
            "fcu",
            severity::WARNING,
            "Low battery warning",
        )
        .unwrap();
        let parsed = StatusText::from_cdr(msg.as_cdr()).unwrap();
        assert_eq!(parsed.frame_id(), "fcu");
        assert_eq!(parsed.severity(), severity::WARNING);
        assert_eq!(parsed.text(), "Low battery warning");
    }

    #[test]
    fn test_gps_raw_round_trip() {
        let msg = GpsRaw::new(
            test_stamp(),
            "gps",
            gps_fix_type::FIX_3D,
            473977070, // lat (47.3977070 degE7)
            85512540,  // lon (8.5512540 degE7)
            408000,    // alt (408m in mm)
            120,       // eph
            150,       // epv
            500,       // vel (5 m/s in cm/s)
            9000,      // cog (90 degrees in cdeg)
            12,        // satellites_visible
            408500,    // alt_ellipsoid
            1000,      // h_acc
            1500,      // v_acc
            200,       // vel_acc
            50000,     // hdg_acc
            9000,      // yaw (90 deg in cdeg)
            4,         // dgps_numch
            1000,      // dgps_age
        )
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
        let msg = TimesyncStatus::new(
            test_stamp(),
            "fcu",
            1_000_000_000, // remote_timestamp_ns (1s)
            -5000,         // observed_offset_ns
            -4500,         // estimated_offset_ns
            2.5,           // round_trip_time_ms
        )
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
        let msg = ExtendedState::new(test_stamp(), "x", 0, 1).unwrap();
        let cdr = msg.to_cdr();
        let owned = ExtendedState::new(test_stamp(), "x", 0, 1).unwrap();
        assert_eq!(owned.into_cdr(), cdr);
    }
}
