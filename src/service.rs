// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

use serde_derive::{Deserialize, Serialize};

/// The struct is used by ROS service.
/// If you want to sent ROS service with Zenoh directly. You should include the header.

#[derive(Serialize, Deserialize, PartialEq, Clone, Debug)]
pub struct ServiceHeader {
    pub guid: i64,
    pub seq: u64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::serde_cdr::{deserialize, serialize};

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
            let bytes = serialize(&header).unwrap();
            let decoded: ServiceHeader = deserialize(&bytes).unwrap();
            assert_eq!(header, decoded, "failed for case: {}", name);
        }
    }
}
