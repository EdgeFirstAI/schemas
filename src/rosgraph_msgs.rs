// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

use crate::builtin_interfaces::Time;
use serde_derive::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct Clock {
    pub clock: Time,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serde_cdr::{deserialize, serialize};

    #[test]
    fn clock_roundtrip() {
        let cases = [
            (0, 0, "zero"),
            (100, 500_000_000, "typical"),
            (i32::MAX, 999_999_999, "max"),
            (-100, 0, "negative sec"),
        ];
        for (sec, nanosec, name) in cases {
            let clock = Clock {
                clock: Time::new(sec, nanosec),
            };
            let bytes = serialize(&clock).unwrap();
            let decoded: Clock = deserialize(&bytes).unwrap();
            assert_eq!(clock, decoded, "failed for case: {}", name);
        }
    }
}
