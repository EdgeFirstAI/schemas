// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Byte parity tests for std_msgs builders vs legacy new() constructors.

#![allow(deprecated)]

use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::std_msgs::Header;

#[test]
fn header_builder_byte_parity_with_new() {
    let stamp = Time::new(1234, 567_891_234);
    let frame_id = "base_link";

    let via_new = Header::new(stamp, frame_id).expect("new() succeeds");
    let via_builder = Header::builder()
        .stamp(stamp)
        .frame_id(frame_id)
        .build()
        .expect("builder.build() succeeds");

    assert_eq!(
        via_new.as_cdr(),
        via_builder.as_cdr(),
        "builder and new() must produce identical CDR bytes",
    );
}
