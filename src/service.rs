// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

/// The struct is used by ROS service.
/// If you want to sent ROS service with Zenoh directly. You should include the header.

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct ServiceHeader {
    pub guid: i64,
    pub seq: u64,
}

use crate::cdr::{CdrCursor, CdrError, CdrFixed, CdrSizer, CdrWriter};

impl CdrFixed for ServiceHeader {
    const CDR_SIZE: usize = 16; // i64 + u64
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        Ok(ServiceHeader {
            guid: cursor.read_i64()?,
            seq: cursor.read_u64()?,
        })
    }
    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        writer.write_i64(self.guid);
        writer.write_u64(self.seq);
    }
    fn size_cdr(sizer: &mut CdrSizer) {
        sizer.size_i64();
        sizer.size_u64();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cdr::{decode_fixed, encode_fixed};

    #[test]
    fn service_header_roundtrip() {
        let cases = [
            (0, 0, "zero"),
            (12345678901234567i64, 1, "typical"),
            (i64::MAX, u64::MAX, "max values"),
            (i64::MIN, 0, "min guid"),
            (-1, 999999, "negative guid"),
        ];
        for (guid, seq, name) in cases {
            let header = ServiceHeader { guid, seq };
            let bytes = encode_fixed(&header).unwrap();
            let decoded: ServiceHeader = decode_fixed(&bytes).unwrap();
            assert_eq!(header, decoded, "failed for case: {}", name);
        }
    }
}
