# Changelog

All notable changes to EdgeFirst Perception Schemas will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

**Version Management:**
- Implemented cargo-release for automated version management across Rust, Python, and ROS2
- Added `release.toml` with pre-release-replacements for version synchronization
- Created `.github/scripts/check_version_sync.sh` to verify version consistency
- Added dedicated `version-check.yml` GitHub Actions workflow to block PRs with version mismatches

**CI/CD Improvements:**
- Created automated GitHub Release workflow (`release.yml`)
- Fixed GitHub Actions workflow syntax errors (branches array format)
- Removed version manipulation from rust.yml and pypi.yml (now managed by cargo-release)
- Added automated CHANGELOG extraction for GitHub Releases

**Documentation:**
- Updated CONTRIBUTING.md with comprehensive cargo-release workflow documentation
- Added visual workflow diagram showing developer actions vs automated steps
- Clarified that GitHub Release is auto-created by tag push, not manually
- Added troubleshooting section for failed releases

**Package Updates:**
- Updated ROS2 package.xml to use real version instead of 0.0.0 placeholder
- Updated debian/rules to extract version from package.xml instead of git tags

### Changed
- Python version now sourced from `__init__.py` __version__ (industry standard)
- All three version sources (Cargo.toml, __init__.py, package.xml) now synchronized via cargo-release
- Release process now uses standard cargo-release steps: version, replace, commit, tag, push

### Fixed
- Version synchronization issues between Rust, Python, and ROS2 packages
- GitHub Actions workflow schema validation errors

## [1.3.1] - 2024-11-15

### Changed
- Optimized PointCloud2 decode to decode entire point in one call instead of one call per field
- Use named tuple for even faster decode
- Added decoding fields where count > 1

[Unreleased]: https://github.com/EdgeFirstAI/schemas/compare/v1.3.1...HEAD
[1.3.1]: https://github.com/EdgeFirstAI/schemas/releases/tag/v1.3.1
