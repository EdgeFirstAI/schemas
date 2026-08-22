// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

use crate::builtin_interfaces::Time;

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct Clock {
    pub clock: Time,
}

use crate::cdr::{CdrCursor, CdrError, CdrFixed, CdrSizer, CdrWriter};

impl CdrFixed for Clock {
    const CDR_SIZE: usize = 8; // Time(8)
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(Clock {
            clock: Time::read_cdr(cursor)?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        self.clock.write_cdr(writer);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        Time::size_cdr(sizer);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cdr::{decode_fixed, encode_fixed};

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
            let bytes = encode_fixed(&clock).unwrap();
            let decoded: Clock = decode_fixed(&bytes).unwrap();
            assert_eq!(clock, decoded, "failed for case: {}", name);
        }
    }
}
