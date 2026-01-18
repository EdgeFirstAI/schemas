// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

use std::time::Duration as Dur;

use serde_derive::{Deserialize, Serialize};

const NSEC_IN_SEC: u64 = 1_000_000_000;

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
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

    pub fn to_nanos(&self) -> u64 {
        self.sec as u64 * NSEC_IN_SEC + self.nanosec as u64
    }
}

impl From<Time> for u64 {
    fn from(time: Time) -> Self {
        time.to_nanos()
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serde_cdr::{deserialize, serialize};

    #[test]
    fn time_roundtrip() {
        // Test various edge cases in one comprehensive test
        let cases = [
            (0, 0, "zero"),
            (1, 500_000_000, "typical"),
            (-100, 500_000_000, "negative sec"),
            (i32::MAX, 999_999_999, "max values"),
            (i32::MIN, 0, "min sec"),
        ];
        for (sec, nanosec, name) in cases {
            let time = Time::new(sec, nanosec);
            let bytes = serialize(&time).unwrap();
            let decoded: Time = deserialize(&bytes).unwrap();
            assert_eq!(time, decoded, "failed for case: {}", name);
        }
    }

    #[test]
    fn time_nanos_conversion() {
        // Verify from_nanos/to_nanos are consistent after serialization
        let original_nanos = 123_456_789_012u64;
        let time = Time::from_nanos(original_nanos);
        let bytes = serialize(&time).unwrap();
        let decoded: Time = deserialize(&bytes).unwrap();
        assert_eq!(decoded.to_nanos(), original_nanos);
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
            let bytes = serialize(&duration).unwrap();
            let decoded: Duration = deserialize(&bytes).unwrap();
            assert_eq!(duration, decoded, "failed for case: {}", name);
        }
    }

    #[test]
    fn duration_from_std() {
        let std_dur = Dur::new(10, 250_000_000);
        let duration: Duration = std_dur.into();
        let bytes = serialize(&duration).unwrap();
        let decoded: Duration = deserialize(&bytes).unwrap();
        assert_eq!(decoded, duration);
    }
}
