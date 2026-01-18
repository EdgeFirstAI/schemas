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
    fn test_time_new() {
        let time = Time::new(42, 123456789);
        let bytes = serialize(&time).unwrap();
        let decoded: Time = deserialize(&bytes).unwrap();
        assert_eq!(decoded.sec, 42);
        assert_eq!(decoded.nanosec, 123456789);
    }

    #[test]
    fn test_time_from_nanos() {
        let time = Time::from_nanos(1_500_000_000);
        let bytes = serialize(&time).unwrap();
        let decoded: Time = deserialize(&bytes).unwrap();
        assert_eq!(decoded.sec, 1);
        assert_eq!(decoded.nanosec, 500_000_000);
    }

    #[test]
    fn test_time_to_nanos() {
        let time = Time::new(2, 300_000_000);
        let bytes = serialize(&time).unwrap();
        let decoded: Time = deserialize(&bytes).unwrap();
        assert_eq!(decoded.to_nanos(), 2_300_000_000);
    }

    #[test]
    fn test_time_roundtrip_nanos() {
        let original_nanos = 123_456_789_012;
        let time = Time::from_nanos(original_nanos);
        let bytes = serialize(&time).unwrap();
        let decoded: Time = deserialize(&bytes).unwrap();
        assert_eq!(decoded.to_nanos(), original_nanos);
    }

    #[test]
    fn test_time_serialize_deserialize() {
        let time = Time {
            sec: 1234567890,
            nanosec: 123456789,
        };
        let bytes = serialize(&time).unwrap();
        let decoded: Time = deserialize(&bytes).unwrap();
        assert_eq!(time, decoded);
    }

    #[test]
    fn test_time_zero() {
        let time = Time { sec: 0, nanosec: 0 };
        let bytes = serialize(&time).unwrap();
        let decoded: Time = deserialize(&bytes).unwrap();
        assert_eq!(time, decoded);
    }

    #[test]
    fn test_time_negative_sec() {
        let time = Time {
            sec: -100,
            nanosec: 500_000_000,
        };
        let bytes = serialize(&time).unwrap();
        let decoded: Time = deserialize(&bytes).unwrap();
        assert_eq!(time, decoded);
    }

    #[test]
    fn test_duration_new() {
        let duration = Duration {
            sec: 5,
            nanosec: 500_000_000,
        };
        let bytes = serialize(&duration).unwrap();
        let decoded: Duration = deserialize(&bytes).unwrap();
        assert_eq!(decoded.sec, 5);
        assert_eq!(decoded.nanosec, 500_000_000);
    }

    #[test]
    fn test_duration_from_std_duration() {
        let std_dur = Dur::new(10, 250_000_000);
        let duration: Duration = std_dur.into();
        let bytes = serialize(&duration).unwrap();
        let decoded: Duration = deserialize(&bytes).unwrap();
        assert_eq!(decoded.sec, 10);
        assert_eq!(decoded.nanosec, 250_000_000);
    }

    #[test]
    fn test_duration_serialize_deserialize() {
        let duration = Duration {
            sec: 42,
            nanosec: 999_999_999,
        };
        let bytes = serialize(&duration).unwrap();
        let decoded: Duration = deserialize(&bytes).unwrap();
        assert_eq!(duration, decoded);
    }

    #[test]
    fn test_duration_zero() {
        let duration = Duration { sec: 0, nanosec: 0 };
        let bytes = serialize(&duration).unwrap();
        let decoded: Duration = deserialize(&bytes).unwrap();
        assert_eq!(duration, decoded);
    }

    #[test]
    fn test_duration_negative() {
        let duration = Duration {
            sec: -5,
            nanosec: 0,
        };
        let bytes = serialize(&duration).unwrap();
        let decoded: Duration = deserialize(&bytes).unwrap();
        assert_eq!(duration, decoded);
    }
}
