// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! SemVer to PEP 440 version conversion.
//!
//! maturin publishes the wheel under the PEP 440 normalization of the Cargo
//! version, so `build.rs` runs the same conversion to give the module a
//! `__version__` equal to `importlib.metadata.version("edgefirst-schemas")`
//! while Cargo and crates.io keep the SemVer spelling:
//!
//! | Cargo (SemVer)        | Python (PEP 440)  |
//! |-----------------------|-------------------|
//! | `4.1.0`               | `4.1.0`           |
//! | `4.1.0-rc.1`          | `4.1.0rc1`        |
//! | `4.1.0-alpha.2`       | `4.1.0a2`         |
//! | `4.1.0-beta.1.dev.3`  | `4.1.0b1.dev3`    |
//! | `4.1.0+build.5`       | `4.1.0+build.5`   |
//!
//! Shared with `tests/pep440.rs` through `#[path]`, since the cdylib crate
//! cannot be linked into an integration test.

const SEPARATORS: [char; 3] = ['.', '-', '_'];

/// PEP 440 pre-release spellings and their normalized form. Longer
/// spellings come first so `alpha` is not read as `a` followed by `lpha`.
const PRE: &[(&str, &str)] = &[
    ("alpha", "a"),
    ("a", "a"),
    ("beta", "b"),
    ("b", "b"),
    ("preview", "rc"),
    ("pre", "rc"),
    ("rc", "rc"),
    ("c", "rc"),
];
const POST: &[(&str, &str)] = &[("post", ".post"), ("rev", ".post"), ("r", ".post")];
const DEV: &[(&str, &str)] = &[("dev", ".dev")];

/// Converts a SemVer version into its normalized PEP 440 spelling.
///
/// The SemVer pre-release part must read as PEP 440 segments, in order: an
/// optional pre-release (`alpha`/`a`, `beta`/`b`, `rc`/`c`/`pre`/`preview`),
/// an optional post-release (`post`/`rev`/`r`) and an optional `dev`, each
/// with an optional number. Build metadata becomes the local version. Any
/// other version is an error.
pub fn from_semver(version: &str) -> Result<String, String> {
    let err = || format!("version {version:?} has no PEP 440 equivalent");

    let (version_part, local) = match version.split_once('+') {
        Some((v, l)) => (v, Some(l)),
        None => (version, None),
    };
    let (release, suffix) = match version_part.split_once('-') {
        Some((r, s)) => (r, Some(s)),
        None => (version_part, None),
    };

    let numbers = release
        .split('.')
        .map(|n| n.parse::<u64>().ok().filter(|_| is_digits(n)))
        .collect::<Option<Vec<_>>>()
        .filter(|n| n.len() == 3)
        .ok_or_else(err)?;
    let mut out = format!("{}.{}.{}", numbers[0], numbers[1], numbers[2]);

    if let Some(suffix) = suffix {
        let lower = suffix.to_ascii_lowercase();
        let mut rest = lower.as_str();
        for labels in [PRE, POST, DEV] {
            if let Some((label, number, tail)) = take_segment(rest, labels) {
                out.push_str(label);
                out.push_str(&number.to_string());
                rest = tail;
            }
        }
        if suffix.is_empty() || !rest.is_empty() {
            return Err(err());
        }
    }

    if let Some(local) = local {
        let segments = local
            .split(SEPARATORS)
            .map(|s| match s {
                "" => None,
                s if is_digits(s) => s.parse::<u64>().ok().map(|n| n.to_string()),
                s if s.chars().all(|c| c.is_ascii_alphanumeric()) => Some(s.to_ascii_lowercase()),
                _ => None,
            })
            .collect::<Option<Vec<_>>>()
            .ok_or_else(err)?;
        out.push('+');
        out.push_str(&segments.join("."));
    }

    Ok(out)
}

/// Reads one `[sep]label[sep][number]` segment from the front of `s`,
/// returning the normalized label, the number (0 when absent) and the rest.
fn take_segment<'a>(
    s: &'a str,
    labels: &[(&str, &'static str)],
) -> Option<(&'static str, u64, &'a str)> {
    let s = s.strip_prefix(SEPARATORS).unwrap_or(s);
    let (label, after_label) = labels
        .iter()
        .find_map(|(spelling, label)| s.strip_prefix(spelling).map(|r| (*label, r)))?;
    let body = after_label.strip_prefix(SEPARATORS).unwrap_or(after_label);
    let digits = body.len() - body.trim_start_matches(|c: char| c.is_ascii_digit()).len();
    if digits == 0 {
        return Some((label, 0, after_label));
    }
    let number = body[..digits].parse().ok()?;
    Some((label, number, &body[digits..]))
}

fn is_digits(s: &str) -> bool {
    !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit())
}
