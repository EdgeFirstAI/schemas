// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! ROS 2 `nav_msgs` message types.
//!
//! Buffer-backed: `Odometry`

use crate::builtin_interfaces::Time;
use crate::cdr::*;
use crate::geometry_msgs::{PoseWithCovariance, TwistWithCovariance};
use crate::std_msgs::Header;

// ── Odometry<B> ─────────────────────────────────────────────────────
//
// CDR layout: Header → pre, then:
//   string child_frame_id → offsets[0] (start of child_frame_id)
//   pad to 8 → offsets[1] (start of PoseWithCovariance)
//   PoseWithCovariance pose (344 bytes, all f64-aligned)
//   TwistWithCovariance twist (336 bytes, all f64-aligned)
//
// offsets[1] is captured from the cursor after aligning to 8 because
// `child_frame_id` is a variable-length string; PoseWithCovariance
// starts with a Point (f64 first), so an explicit align-to-8 is needed.

pub struct Odometry<B> {
    buf: B,
    offsets: [usize; 2],
}

impl<B: AsRef<[u8]>> Odometry<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let pre = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), pre);
        let o0 = c.offset();
        let _ = c.read_string()?; // child_frame_id
        c.align(8);
        let o1 = c.offset();
        PoseWithCovariance::read_cdr(&mut c)?;
        TwistWithCovariance::read_cdr(&mut c)?;
        Ok(Odometry {
            offsets: [o0, o1],
            buf,
        })
    }

    /// Returns a `Header` view by re-parsing the CDR buffer prefix.
    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }
    pub fn child_frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[0]).0
    }
    pub fn pose(&self) -> PoseWithCovariance {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[1]);
        PoseWithCovariance::read_cdr(&mut c).expect("pose validated during from_cdr")
    }
    pub fn twist(&self) -> TwistWithCovariance {
        // PoseWithCovariance::CDR_SIZE is 344 bytes, all f64-aligned,
        // so the twist starts exactly offsets[1] + 344.
        let mut c = CdrCursor::resume(
            self.buf.as_ref(),
            self.offsets[1] + PoseWithCovariance::CDR_SIZE,
        );
        TwistWithCovariance::read_cdr(&mut c).expect("twist validated during from_cdr")
    }
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl Odometry<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        child_frame_id: &str,
        pose: PoseWithCovariance,
        twist: TwistWithCovariance,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_string(child_frame_id);
        sizer.align(8);
        let o1 = sizer.offset();
        PoseWithCovariance::size_cdr(&mut sizer);
        TwistWithCovariance::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_string(child_frame_id);
        pose.write_cdr(&mut w);
        twist.write_cdr(&mut w);
        w.finish()?;

        Ok(Odometry {
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
    matches!(type_name, "Odometry")
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &["nav_msgs/msg/Odometry"]
}
