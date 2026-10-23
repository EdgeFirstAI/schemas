// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! SemVer to PEP 440 conversion behind `edgefirst.schemas.__version__`.
//! Expected values are the normalized form `packaging.version.Version`
//! gives for the same string, which is what maturin publishes.

#[path = "../src/pep440.rs"]
mod pep440;

use pep440::from_semver;

fn ok(semver: &str) -> String {
    from_semver(semver).unwrap_or_else(|e| panic!("{e}"))
}

#[test]
fn release_is_unchanged() {
    assert_eq!(ok("4.1.0"), "4.1.0");
    assert_eq!(ok("0.0.0"), "0.0.0");
    assert_eq!(ok("10.20.30"), "10.20.30");
}

#[test]
fn release_candidates() {
    assert_eq!(ok("4.1.0-rc.1"), "4.1.0rc1");
    assert_eq!(ok("4.1.0-rc1"), "4.1.0rc1");
    assert_eq!(ok("4.1.0-rc.12"), "4.1.0rc12");
    assert_eq!(ok("4.1.0-RC.2"), "4.1.0rc2");
    assert_eq!(ok("4.1.0-rc"), "4.1.0rc0");
    assert_eq!(ok("4.1.0-rc.01"), "4.1.0rc1");
    assert_eq!(ok("4.1.0-c.1"), "4.1.0rc1");
    assert_eq!(ok("4.1.0-pre.1"), "4.1.0rc1");
    assert_eq!(ok("4.1.0-preview.1"), "4.1.0rc1");
}

#[test]
fn alpha_and_beta() {
    assert_eq!(ok("4.1.0-alpha.1"), "4.1.0a1");
    assert_eq!(ok("4.1.0-a.1"), "4.1.0a1");
    assert_eq!(ok("4.1.0-alpha"), "4.1.0a0");
    assert_eq!(ok("4.1.0-beta.2"), "4.1.0b2");
    assert_eq!(ok("4.1.0-b2"), "4.1.0b2");
}

#[test]
fn post_and_dev_releases() {
    assert_eq!(ok("4.1.0-post.1"), "4.1.0.post1");
    assert_eq!(ok("4.1.0-rev.1"), "4.1.0.post1");
    assert_eq!(ok("4.1.0-dev.3"), "4.1.0.dev3");
    assert_eq!(ok("4.1.0-dev"), "4.1.0.dev0");
    assert_eq!(ok("4.1.0-rc.1.dev.2"), "4.1.0rc1.dev2");
    assert_eq!(ok("4.1.0-alpha.1.post.2.dev.3"), "4.1.0a1.post2.dev3");
}

#[test]
fn build_metadata_becomes_local_version() {
    assert_eq!(ok("4.1.0+build.5"), "4.1.0+build.5");
    assert_eq!(ok("4.1.0-rc.1+Git-ABC"), "4.1.0rc1+git.abc");
    assert_eq!(ok("4.1.0+exp.01"), "4.1.0+exp.1");
}

#[test]
fn rejects_versions_without_a_pep440_spelling() {
    for bad in [
        "4.1",
        "4.1.0.1",
        "4.1.x",
        "4.1.0-",
        "4.1.0-foo",
        "4.1.0-rc.1.x",
        "4.1.0-rc.1-rc.2",
        "4.1.0-dev.1.rc.1",
        "4.1.0+",
        "4.1.0+a..b",
    ] {
        assert!(from_semver(bad).is_err(), "{bad:?} should be rejected");
    }
}

#[test]
fn crate_version_converts() {
    ok(env!("CARGO_PKG_VERSION"));
}
