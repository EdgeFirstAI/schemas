// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! The C header's `SENSOR_MSGS_POINT_FIELD_*` macros match the Rust
//! `sensor_msgs::point_field` constants by name. `PointFieldType` and the
//! Python `PointField.*` class attributes are derived from those constants.

use std::collections::BTreeMap;
use std::path::Path;

use edgefirst_schemas::sensor_msgs::point_field;

#[test]
fn c_header_point_field_codes_match_rust_by_name() {
    let expected: BTreeMap<&str, u8> = [
        ("INT8", point_field::INT8),
        ("UINT8", point_field::UINT8),
        ("INT16", point_field::INT16),
        ("UINT16", point_field::UINT16),
        ("INT32", point_field::INT32),
        ("UINT32", point_field::UINT32),
        ("FLOAT32", point_field::FLOAT32),
        ("FLOAT64", point_field::FLOAT64),
    ]
    .into_iter()
    .collect();

    let path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../crates/capi/include/edgefirst/schemas.h");
    let header = std::fs::read_to_string(&path).unwrap();
    let got: BTreeMap<&str, u8> = header
        .lines()
        .filter_map(|l| {
            let rest = l.trim().strip_prefix("#define SENSOR_MSGS_POINT_FIELD_")?;
            let (name, value) = rest.split_once(' ')?;
            Some((name, value.trim().parse().ok()?))
        })
        .collect();
    assert_eq!(got, expected);
}
