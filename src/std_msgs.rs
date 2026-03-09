// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! ROS 2 `std_msgs` message types.
//!
//! - `ColorRGBA` — CdrFixed, 16 bytes (4 × f32)
//! - `Header` — buffer-backed, contains stamp + frame_id

use crate::builtin_interfaces::Time;
use crate::cdr::*;

// ── CdrFixed types ──────────────────────────────────────────────────

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct ColorRGBA {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl CdrFixed for ColorRGBA {
    const CDR_SIZE: usize = 16; // 4 x f32
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(ColorRGBA {
            r: cursor.read_f32()?,
            g: cursor.read_f32()?,
            b: cursor.read_f32()?,
            a: cursor.read_f32()?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_f32(self.r);
        writer.write_f32(self.g);
        writer.write_f32(self.b);
        writer.write_f32(self.a);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_f32();
        sizer.size_f32();
    }
}

// ── Buffer-backed types ─────────────────────────────────────────────

// CDR layout (after 4-byte CDR LE header):
//   4: i32 stamp.sec
//   8: u32 stamp.nanosec
//  12: u32 frame_id_len (incl NUL)
//  16: [u8] frame_id bytes + NUL
//  offsets[0] = end of frame_id

pub struct Header<B> {
    buf: B,
    offsets: [usize; 1],
}

impl<B: AsRef<[u8]>> Header<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let mut c = CdrCursor::new(buf.as_ref())?;
        // skip stamp (8 bytes)
        c.skip(8)?;
        let _ = c.read_string()?;
        Ok(Header {
            offsets: [c.offset()],
            buf,
        })
    }

    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }

    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }

    /// Byte position after the last field (useful for composite types).
    pub fn end_offset(&self) -> usize {
        self.offsets[0]
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

impl Header<Vec<u8>> {
    pub fn new(stamp: Time, frame_id: &str) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        let offsets = [w.offset()];
        w.finish()?;
        Ok(Header { buf, offsets })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> Header<B> {
    pub fn set_stamp(&mut self, t: Time) -> Result<(), CdrError> {
        let b = self.buf.as_mut();
        wr_i32(b, CDR_HEADER_SIZE, t.sec)?;
        wr_u32(b, CDR_HEADER_SIZE + 4, t.nanosec)
    }
}

// ── Registry ────────────────────────────────────────────────────────

/// Check if a type name is supported by this module.
pub fn is_type_supported(type_name: &str) -> bool {
    matches!(type_name, "Header" | "ColorRGBA")
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &["std_msgs/msg/Header", "std_msgs/msg/ColorRGBA"]
}

// SchemaType implementations
use crate::schema_registry::SchemaType;

impl SchemaType for ColorRGBA {
    const SCHEMA_NAME: &'static str = "std_msgs/msg/ColorRGBA";
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;

    #[test]
    fn header_roundtrip() {
        let cases = [
            (0, 0, "", "empty"),
            (100, 500_000_000, "camera", "typical"),
            (42, 0, "a]long_frame_id", "with special chars"),
            (1, 0, "camera/optical_frame", "path separator"),
            (i32::MAX, 999_999_999, "max_frame", "max time"),
        ];
        for (sec, nanosec, frame_id, name) in cases {
            let header = Header::new(Time::new(sec, nanosec), frame_id).unwrap();
            assert_eq!(
                header.stamp(),
                Time::new(sec, nanosec),
                "stamp failed: {}",
                name
            );
            assert_eq!(header.frame_id(), frame_id, "frame_id failed: {}", name);

            // Round-trip through CDR bytes
            let bytes = header.to_cdr();
            let decoded = Header::from_cdr(bytes).unwrap();
            assert_eq!(
                decoded.stamp(),
                Time::new(sec, nanosec),
                "rt stamp failed: {}",
                name
            );
            assert_eq!(decoded.frame_id(), frame_id, "rt frame_id failed: {}", name);
        }
    }

    #[test]
    fn header_set_stamp() {
        let mut header = Header::new(Time::new(0, 0), "test").unwrap();
        header.set_stamp(Time::new(42, 123)).unwrap();
        assert_eq!(header.stamp(), Time::new(42, 123));
    }

    #[test]
    fn color_rgba_roundtrip() {
        use crate::cdr::{decode_fixed, encode_fixed};

        let cases = [
            (0.0, 0.0, 0.0, 0.0, "zero/transparent"),
            (1.0, 0.0, 0.0, 1.0, "red"),
            (0.0, 1.0, 0.0, 1.0, "green"),
            (0.0, 0.0, 1.0, 1.0, "blue"),
            (1.0, 1.0, 1.0, 1.0, "white"),
            (0.5, 0.5, 0.5, 0.5, "gray semi-transparent"),
        ];
        for (r, g, b, a, name) in cases {
            let color = ColorRGBA { r, g, b, a };
            let bytes = encode_fixed(&color).unwrap();
            let decoded: ColorRGBA = decode_fixed(&bytes).unwrap();
            assert_eq!(color, decoded, "failed for case: {}", name);
        }
    }
}
