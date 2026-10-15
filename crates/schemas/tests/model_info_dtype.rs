// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! `ModelInfo` dtype codes are the HAL tensor dtype codes.
//!
//! `edgefirst-tensor-abi` (`EfDtype`) is their only authority. These tests tie
//! every place this repository spells them out — the Rust `model_info::DTYPE_*`
//! constants, the C header's `EDGEFIRST_MSGS_MODEL_INFO_DTYPE_*` macros and the
//! `.msg` definition's `DTYPE_*` constants — to `EfDtype` by name. The Python
//! `ModelInfo.DTYPE_*` class attributes are the Rust constants re-exported.

use std::collections::BTreeMap;
use std::path::Path;

use edgefirst_schemas::edgefirst_msgs::model_info;
use edgefirst_tensor_abi::EfDtype;

/// The schemas constant declared under the same name as each `EfDtype`.
///
/// The match is exhaustive, so a dtype added to HAL fails to compile here
/// until it is mapped.
fn schemas_code(d: EfDtype) -> u8 {
    match d {
        EfDtype::U8 => model_info::DTYPE_U8,
        EfDtype::I8 => model_info::DTYPE_I8,
        EfDtype::U16 => model_info::DTYPE_U16,
        EfDtype::I16 => model_info::DTYPE_I16,
        EfDtype::U32 => model_info::DTYPE_U32,
        EfDtype::I32 => model_info::DTYPE_I32,
        EfDtype::U64 => model_info::DTYPE_U64,
        EfDtype::I64 => model_info::DTYPE_I64,
        EfDtype::F16 => model_info::DTYPE_F16,
        EfDtype::F32 => model_info::DTYPE_F32,
        EfDtype::F64 => model_info::DTYPE_F64,
    }
}

const ALL: [EfDtype; 11] = [
    EfDtype::U8,
    EfDtype::I8,
    EfDtype::U16,
    EfDtype::I16,
    EfDtype::U32,
    EfDtype::I32,
    EfDtype::U64,
    EfDtype::I64,
    EfDtype::F16,
    EfDtype::F32,
    EfDtype::F64,
];

/// `name -> code` for every dtype, named as HAL names it, plus `UNKNOWN`.
fn expected() -> BTreeMap<String, u8> {
    let mut m: BTreeMap<String, u8> = ALL
        .iter()
        .map(|&d| (format!("{d:?}").to_uppercase(), d as u32 as u8))
        .collect();
    m.insert("UNKNOWN".into(), model_info::DTYPE_UNKNOWN);
    m
}

fn repo_file(rel: &str) -> String {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(rel);
    std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()))
}

/// Collect `<prefix><NAME> ... <value>` declarations, one per line.
fn declared(text: &str, prefix: &str, sep: char) -> BTreeMap<String, u8> {
    text.lines()
        .filter_map(|l| {
            let rest = l.trim().strip_prefix(prefix)?;
            let (name, value) = rest.split_once(sep)?;
            let value = value.split('#').next()?.trim();
            Some((name.trim().to_string(), value.parse().ok()?))
        })
        .collect()
}

#[test]
fn rust_constants_match_hal_dtype_by_name() {
    for d in ALL {
        assert_eq!(
            schemas_code(d) as u32,
            d as u32,
            "model_info::DTYPE_{d:?} differs from EfDtype::{d:?}"
        );
    }
}

#[test]
fn unknown_is_not_a_hal_dtype() {
    for d in ALL {
        assert_ne!(model_info::DTYPE_UNKNOWN as u32, d as u32);
    }
}

#[test]
fn legacy_codes_round_trip_through_hal_dtype() {
    for d in ALL {
        let code = schemas_code(d);
        let legacy = model_info::legacy_from_dtype(code);
        assert_eq!(model_info::dtype_from_legacy(legacy), code, "{d:?}");
    }
    #[allow(deprecated)]
    {
        assert_eq!(
            model_info::dtype_from_legacy(model_info::RAW),
            model_info::DTYPE_UNKNOWN
        );
        assert_eq!(
            model_info::dtype_from_legacy(model_info::STRING),
            model_info::DTYPE_UNKNOWN
        );
        assert_eq!(
            model_info::legacy_from_dtype(model_info::DTYPE_UNKNOWN),
            model_info::RAW
        );
    }
}

#[test]
fn c_header_constants_match_hal_dtype_by_name() {
    let header = repo_file("crates/capi/include/edgefirst/schemas.h");
    let got = declared(&header, "#define EDGEFIRST_MSGS_MODEL_INFO_DTYPE_", ' ');
    assert_eq!(got, expected());
}

#[test]
fn msg_constants_match_hal_dtype_by_name() {
    let msg = repo_file("edgefirst_msgs/msg/ModelInfo.msg");
    let got = declared(&msg, "uint8 DTYPE_", '=');
    assert_eq!(got, expected());
}
