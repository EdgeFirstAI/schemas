// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! Tests for `FooBuilder` allocation reuse and slice-bounds behaviour.
//!
//! These are orthogonal to the byte-parity tests in `cdr_golden.rs`:
//! they verify that (a) repeated `encode_into_vec` calls with equal-size
//! outputs perform no reallocation, (b) shrinking works, (c) slice-capacity
//! errors are reported cleanly without mutating the buffer.

use edgefirst_schemas::builtin_interfaces::Time;
use edgefirst_schemas::cdr::CdrError;
use edgefirst_schemas::sensor_msgs::Image;

#[test]
fn image_encode_into_vec_reuses_allocation() {
    let stamp = Time::new(0, 0);
    let pixels = vec![0u8; 64];

    let mut b = Image::builder();
    b.stamp(stamp)
        .frame_id("cam")
        .height(8)
        .width(8)
        .encoding("mono8")
        .is_bigendian(0)
        .step(8)
        .data(&pixels);

    let mut buf = Vec::new();
    b.encode_into_vec(&mut buf).expect("first encode");
    let cap_after_first = buf.capacity();
    assert!(cap_after_first > 0);

    for _ in 0..100 {
        b.encode_into_vec(&mut buf).expect("subsequent encode");
    }

    assert_eq!(
        buf.capacity(),
        cap_after_first,
        "capacity must not grow across identical encodes",
    );
}

#[test]
fn image_encode_into_vec_shrinks_len_when_payload_smaller() {
    let stamp = Time::new(0, 0);

    // First, encode a large image to grow the Vec.
    let large_pixels = vec![0u8; 4096];
    let mut b = Image::builder();
    b.stamp(stamp)
        .frame_id("cam")
        .height(64)
        .width(64)
        .encoding("mono8")
        .is_bigendian(0)
        .step(64)
        .data(&large_pixels);

    let mut buf = Vec::new();
    b.encode_into_vec(&mut buf).expect("large encode");
    let large_len = buf.len();
    let large_cap = buf.capacity();

    // Now encode a small image into the same Vec. len must shrink; cap retained.
    let small_pixels = vec![0u8; 4];
    b.height(2).width(2).step(2).data(&small_pixels);
    b.encode_into_vec(&mut buf).expect("small encode");

    assert!(buf.len() < large_len, "len must shrink to small payload size");
    assert_eq!(
        buf.capacity(),
        large_cap,
        "capacity must be retained when shrinking",
    );
}

#[test]
fn image_encode_into_slice_returns_bytes_written() {
    let stamp = Time::new(0, 0);
    let pixels = vec![0u8; 12];
    let mut b = Image::builder();
    b.stamp(stamp)
        .frame_id("cam")
        .height(2)
        .width(2)
        .encoding("rgb8")
        .is_bigendian(0)
        .step(6)
        .data(&pixels);

    let mut big_buf = [0u8; 256];
    let n = b
        .encode_into_slice(&mut big_buf)
        .expect("slice encode succeeds");
    assert!(n > 0);
    assert!(n <= big_buf.len());

    // Compare bytes against build() output.
    let via_build = b.build().expect("build succeeds");
    assert_eq!(&big_buf[..n], via_build.as_cdr());
}

#[test]
fn image_encode_into_slice_errors_when_too_small() {
    let stamp = Time::new(0, 0);
    let pixels = vec![0u8; 12];
    let mut b = Image::builder();
    b.stamp(stamp)
        .frame_id("cam")
        .height(2)
        .width(2)
        .encoding("rgb8")
        .is_bigendian(0)
        .step(6)
        .data(&pixels);

    // Canary bytes to detect any accidental write on the error path.
    let mut tiny = [0xABu8; 8];
    let err = b
        .encode_into_slice(&mut tiny)
        .expect_err("must error on undersized buffer");

    match err {
        CdrError::BufferTooShort { need, have } => {
            assert_eq!(have, 8);
            assert!(need > 8);
        }
        other => panic!("expected BufferTooShort, got {:?}", other),
    }

    // The buffer must be untouched on error.
    assert!(tiny.iter().all(|&b| b == 0xAB), "buffer mutated on error");
}
