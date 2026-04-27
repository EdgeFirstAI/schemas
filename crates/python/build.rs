// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

fn main() {
    // Emit Py_LIMITED_API / Py_3_x cfg keys based on the PyO3 build config
    // and the abi3-* feature flags. Source files use these via
    // `cfg(any(not(Py_LIMITED_API), Py_3_11))` to gate buffer-protocol
    // implementations on the API surface that's actually available.
    pyo3_build_config::use_pyo3_cfgs();

    // Allow source files to detect nightly Rust (mirrors hal). We don't use
    // any nightly features today, but the cfg is here so future
    // optimisations (e.g. `feature(f16)`) can opt in cleanly.
    println!("cargo::rustc-check-cfg=cfg(nightly)");
    let is_nightly = rustc_version::version_meta()
        .map(|m| m.channel == rustc_version::Channel::Nightly)
        .unwrap_or(false);
    if is_nightly {
        println!("cargo::rustc-cfg=nightly");
    }
}
