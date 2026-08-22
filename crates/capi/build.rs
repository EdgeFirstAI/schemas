// SPDX-FileCopyrightText: Copyright 2025 Au-Zone Technologies
// SPDX-License-Identifier: Apache-2.0

use std::env;

fn main() {
    // Stamp a versioned identity into the shared library so the runtime loader
    // binds to a MAJOR-qualified name rather than the bare filename. `make
    // install` lays down the matching symlink chain; `make lib` creates the
    // single MAJOR hop the build tree needs.
    //
    // This is scoped to the capi crate deliberately. `rustc-cdylib-link-arg`
    // applies to every cdylib that depends on the emitting crate, so while this
    // lived in the workspace root it also landed on the PyO3 extension module
    // and made the wheel advertise the C library's SONAME. crates/python
    // depends on the schemas rlib, not on capi, so it can no longer reach this.
    let target_os = env::var("CARGO_CFG_TARGET_OS").unwrap_or_default();
    let major = env::var("CARGO_PKG_VERSION_MAJOR").unwrap();
    match target_os.as_str() {
        "linux" => {
            println!("cargo:rustc-cdylib-link-arg=-Wl,-soname,libedgefirst_schemas.so.{major}");
        }
        // macOS encodes the install name, and puts the version before the
        // extension. @rpath keeps the build tree working with -rpath; `make
        // install` retargets the id to the absolute path.
        "macos" => {
            println!(
                "cargo:rustc-cdylib-link-arg=-Wl,-install_name,@rpath/libedgefirst_schemas.{major}.dylib"
            );
        }
        // Windows DLLs carry no soversion in the filename and no equivalent
        // load-command; versioning is by side-by-side assembly or file layout.
        _ => {}
    }
}
