// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

use std::time::Duration as Dur;

const NSEC_IN_SEC: u64 = 1_000_000_000;

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Duration {
    pub sec: i32,
    pub nanosec: u32,
}

impl Time {
    pub fn new(sec: i32, nanosec: u32) -> Self {
        Time { sec, nanosec }
    }

    pub fn from_nanos(nanos: u64) -> Self {
        Time {
            sec: (nanos / NSEC_IN_SEC) as i32,
            nanosec: (nanos % NSEC_IN_SEC) as u32,
        }
    }

    /// Convert to nanoseconds. Returns `None` for negative (pre-epoch) timestamps
    /// since they cannot be represented as `u64`.
    pub fn to_nanos(&self) -> Option<u64> {
        if self.sec >= 0 {
            Some(self.sec as u64 * NSEC_IN_SEC + self.nanosec as u64)
        } else {
            None
        }
    }
}

impl From<Time> for u64 {
    /// Converts to nanoseconds, saturating pre-epoch timestamps to 0.
    fn from(time: Time) -> Self {
        time.to_nanos().unwrap_or(0)
    }
}

impl Duration {
    pub fn new(sec: i32, nanosec: u32) -> Self {
        Duration { sec, nanosec }
    }
}

impl From<Dur> for Duration {
    fn from(dur: Dur) -> Self {
        Duration {
            sec: dur.as_secs() as i32,
            nanosec: dur.subsec_nanos(),
        }
    }
}

/// Check if a type name is supported by this module.
pub fn is_type_supported(type_name: &str) -> bool {
    matches!(type_name, "Duration" | "Time")
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &[
        "builtin_interfaces/msg/Duration",
        "builtin_interfaces/msg/Time",
    ]
}

// CdrFixed implementations
use crate::cdr::{CdrCursor, CdrError, CdrFixed, CdrSizer, CdrWriter};

impl CdrFixed for Time {
    const CDR_SIZE: usize = 8; // i32 + u32
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        let sec = cursor.read_i32()?;
        let nanosec = cursor.read_u32()?;
        Ok(Time { sec, nanosec })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_i32(self.sec);
        writer.write_u32(self.nanosec);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_i32();
        sizer.size_u32();
    }
}

impl CdrFixed for Duration {
    const CDR_SIZE: usize = 8; // i32 + u32
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        let sec = cursor.read_i32()?;
        let nanosec = cursor.read_u32()?;
        Ok(Duration { sec, nanosec })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_i32(self.sec);
        writer.write_u32(self.nanosec);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_i32();
        sizer.size_u32();
    }
}

// SchemaType implementations
use crate::schema_registry::SchemaType;

impl SchemaType for Duration {
    const SCHEMA_NAME: &'static str = "builtin_interfaces/msg/Duration";
}

impl SchemaType for Time {
    const SCHEMA_NAME: &'static str = "builtin_interfaces/msg/Time";
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cdr::{decode_fixed, encode_fixed};

    #[test]
    fn time_roundtrip() {
        let cases = [
            (0, 0, "zero"),
            (1, 500_000_000, "typical"),
            (-100, 500_000_000, "negative sec"),
            (i32::MAX, 999_999_999, "max values"),
            (i32::MIN, 0, "min sec"),
        ];
        for (sec, nanosec, name) in cases {
            let time = Time::new(sec, nanosec);
            let bytes = encode_fixed(&time).unwrap();
            let decoded: Time = decode_fixed(&bytes).unwrap();
            assert_eq!(time, decoded, "failed for case: {}", name);
        }
    }

    #[test]
    fn time_nanos_conversion() {
        let original_nanos = 123_456_789_012u64;
        let time = Time::from_nanos(original_nanos);
        let bytes = encode_fixed(&time).unwrap();
        let decoded: Time = decode_fixed(&bytes).unwrap();
        assert_eq!(decoded.to_nanos(), Some(original_nanos));
    }

    #[test]
    fn duration_roundtrip() {
        let cases = [
            (0, 0, "zero"),
            (5, 500_000_000, "typical"),
            (-5, 0, "negative"),
            (i32::MAX, 999_999_999, "max values"),
        ];
        for (sec, nanosec, name) in cases {
            let duration = Duration { sec, nanosec };
            let bytes = encode_fixed(&duration).unwrap();
            let decoded: Duration = decode_fixed(&bytes).unwrap();
            assert_eq!(duration, decoded, "failed for case: {}", name);
        }
    }

    #[test]
    fn duration_from_std() {
        let std_dur = Dur::new(10, 250_000_000);
        let duration: Duration = std_dur.into();
        let bytes = encode_fixed(&duration).unwrap();
        let decoded: Duration = decode_fixed(&bytes).unwrap();
        assert_eq!(decoded, duration);
    }
}
