# Changelog

All notable changes to EdgeFirst Perception Schemas will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- **C API: Replaced bulk array getters with indexed accessors for opaque types**
  - `ros_point_cloud2_get_fields()` → `ros_point_cloud2_get_num_fields()` + `ros_point_cloud2_get_field_at()`
  - `edgefirst_detect_get_boxes()` → `edgefirst_detect_get_num_boxes()` + `edgefirst_detect_get_box_at()`
  - Returning `Vec::as_ptr()` for opaque Rust-layout structs (containing `String`) was unusable from C
    since forward-declared types have unknown `sizeof`, making `fields[i]` impossible to compile
- C API: Fixed out-of-bounds handling in `edgefirst_detect_get_box_at()` and
  `ros_point_cloud2_get_field_at()` — replaced `assert!` (which panics across FFI, causing UB)
  with safe `match .get(index)` returning `NULL`
- C API: Added 5 missing declarations to `schemas.h`:
  - `foxglove_point_annotations_get_num_outline_colors()`
  - `foxglove_point_annotations_get_outline_color_at()`
  - `foxglove_point_annotations_set_outline_colors()`
  - `foxglove_point_annotations_clear_outline_colors()`
  - `edgefirst_detect_clear_boxes()`
- Fixed `EdgeFirstDmaBuf` → `EdgeFirstDmaBuffer` type name in C example (`examples/c/example.c`)

## [1.5.2] - 2026-01-27

### Fixed
- C API errno handling now uses portable `errno` crate for Windows and macOS support
- Replaced platform-specific set_errno implementations with cross-platform solution

## [1.5.1] - 2026-01-26

### Fixed
- Fixed PyPI trusted publisher configuration for automated releases
- Removed redundant ROS workflow trigger on tag pushes
- Added cross-workflow Debian package publishing to GitHub Releases

## [1.5.0] - 2026-01-26

### Added

**C API Expansion:**
- Comprehensive C FFI bindings for all message types via `libedgefirst_schemas.so`
- New C header `include/edgefirst/schemas.h` with full Doxygen documentation
- Support for 60+ message types across all namespaces:
  - `builtin_interfaces`: Time, Duration
  - `std_msgs`: Header, ColorRGBA
  - `geometry_msgs`: Vector3, Point, Point32, Quaternion, Pose, Pose2D, PoseStamped, Transform, TransformStamped, Twist, TwistStamped, Accel, AccelStamped, Wrench, WrenchStamped, Inertia, InertiaStamped, Polygon, PolygonStamped, PoseArray, PoseWithCovariance, PoseWithCovarianceStamped, TwistWithCovariance, TwistWithCovarianceStamped, AccelWithCovariance, AccelWithCovarianceStamped
  - `sensor_msgs`: Image, PointCloud2, PointField, CompressedImage, CameraInfo, RegionOfInterest, Imu, NavSatFix, NavSatStatus, Range, Temperature, RelativeHumidity, Illuminance, FluidPressure, BatteryState, JointState, Joy, JoyFeedback, JoyFeedbackArray, TimeReference, LaserScan, MultiEchoLaserScan, LaserEcho, MagneticField
  - `nav_msgs`: Odometry, Path, OccupancyGrid, MapMetaData, GridCells
  - `rosgraph_msgs`: Log, Clock
  - `foxglove_msgs`: CompressedVideo
  - `edgefirst_msgs`: DmaBuf, RadarCube, RadarInfo, Mask, Model, ModelInfo, Detect, Box, Track
- Service message types: SetBool, Trigger, Empty
- Complete unit tests for C API using Criterion test framework

**Benchmark Suite:**
- Comprehensive CDR serialization benchmarks for all heavy message types
- On-target benchmark execution on NXP i.MX 8M Plus (Cortex-A53)
- SmartMicro DRVEGRD radar configurations:
  - DRVEGRD-169 (Corner Radar): Ultra-Short, Short, Medium, Long modes
  - DRVEGRD-171 (Front Radar): Short, Medium, Long modes
- Image benchmarks with RGB8, YUYV, NV12 encodings at VGA/HD/FHD
- Mask benchmarks at 320/640/1280 with 8 and 32 classes
- CompressedMask benchmarks with zstd compression
- DmaBuf zero-copy reference benchmarks (nanosecond scale)
- PointCloud2 benchmarks from sparse (1K) to very dense (131K points)

**Schema Registry:**
- Runtime schema name registry (`schema_registry.rs`, `registry.py`) for type lookup by ROS 2 schema name

**MCAP Validation:**
- Rust MCAP integration tests for real-world CDR validation against hardware recordings
- Python MCAP tests (`test_mcap.py`) and schema registry tests (`test_schema_registry.py`)

**Release Packaging:**
- Pre-built C API binary packages for Linux x86_64 and aarch64 attached to GitHub Releases
- Archives include shared library (.so), static library (.a), C header, README, and LICENSE

**CI/CD Improvements:**
- QuickChart integration for grouped bar charts in GitHub Actions Summary
- Criterion JSON parsing for reliable benchmark data extraction
- EnricoMi/publish-unit-test-result-action for C API test reporting
- SonarCloud integration with Rust and C coverage reporting via llvm-cov
- On-target ARM64 benchmark execution via self-hosted runner

### Changed
- Benchmark charts now auto-scale Y-axis to actual data
- DmaBuf benchmarks display in nanoseconds for precision
- All chart values show 2 decimal places for readability
- Updated `sonar-project.properties` to include C API sources and tests

### Fixed
- Criterion `slope: null` handling for benchmarks without linear fit
- UTF-8 encoding issues in Mermaid charts (switched to QuickChart)
- Stale benchmark cache causing legacy names to appear

## [1.4.1] - 2025-11-18

### Changed
- **All packages now licensed under Apache-2.0**: Updated Rust crate, Python package, and ROS2 package metadata to reflect Apache-2.0 license across all distributions
- SBOM generation is now 3x faster (17s → 5-6s), reducing build times for downstream users
- Updated dependency scanning for improved security and compliance

### Fixed
- Improved reliability of license policy enforcement

## [1.4.0] - 2025-11-17

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

## [1.3.1] - 2025-05-14

### Changed
- Optimized PointCloud2 decode to decode entire point in one call instead of one call per field
- Use named tuple for even faster decode
- Added decoding fields where count > 1

## [1.3.0] - 2025-05-12

### Added
- PointCloud2 parsing utilities moved from samples to schemas for better reuse
- Boxes field to mask in edgefirst_msgs.py

### Changed
- Migrated to pyproject.toml for Python packaging
- Removed Git hash from version for Python
- More explicit version string parsing for Python __init__.py

### Fixed
- Rust formatting fixes
- Python build module installation
- ModelInfo added to Python schemas
- LocalTime import issues
- Import error: cannot import name 'default_field'
- Added missing PCD fields to p.fields dictionary

## [1.2.11] - 2025-05-06

### Fixed
- Explicitly added edgefirst.schemas module to Python package

## [1.2.10] - 2025-05-04

### Changed
- Removed edgefirst/__init__.py to avoid issues with multiple packages using the edgefirst namespace

## [1.2.9] - 2025-05-03

### Fixed
- RadarInfo Duration initialization

## [1.2.8] - 2025-05-03

### Fixed
- RadarInfo and Model messages not using default_field for Header initialization

## [1.2.7] - 2025-04-30

### Added
- Boxed boolean to mask for instanced segmentation

### Fixed
- Comment about unused encoding
- Duration not using ROS Duration type

## [1.2.6] - 2025-03-27

### Changed
- Formatting and documentation updates

## [1.2.5] - 2025-03-27

### Added
- DmaBuffer message schema and Python API
- Python variants of all provided messages
- Float16 datatype support
- Additional datatypes for model_info
- ModelInfo to Rust schemas
- Custom Detect schema for Python
- CompressedImage schema for JPEG handling
- CompressedVideo schema
- CameraInfo and ImageAnnotation schema for Python parser
- Time conversion traits to u64

### Changed
- Renamed DetectBoxes2D to Detect
- Changed label type from String to u32, then back to String
- Moved Box and Track to separate files
- Corrected Detect to be a sequence of boxes
- Updated comments in detect schema
- Removed is_tracked field from Detect
- Re-organized Rust library to bring all name_msgs into base module

### Fixed
- Fixed edgefirst typo
- Moved detect.py to correct directory
- Updated setup.py to include all Python messages
- Python package long description using README.md
- Python build issues with wheel generation
- Removed auxiliary files from ROS2 schemas not required for this project

[Unreleased]: https://github.com/EdgeFirstAI/schemas/compare/v1.5.1...HEAD
[1.5.1]: https://github.com/EdgeFirstAI/schemas/compare/v1.5.0...v1.5.1
[1.5.0]: https://github.com/EdgeFirstAI/schemas/compare/v1.4.1...v1.5.0
[1.4.1]: https://github.com/EdgeFirstAI/schemas/compare/v1.4.0...v1.4.1
[1.4.0]: https://github.com/EdgeFirstAI/schemas/compare/v1.3.1...v1.4.0
[1.3.1]: https://github.com/EdgeFirstAI/schemas/compare/v1.3.0...v1.3.1
[1.3.0]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.11...v1.3.0
[1.2.11]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.10...v1.2.11
[1.2.10]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.9...v1.2.10
[1.2.9]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.8...v1.2.9
[1.2.8]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.7...v1.2.8
[1.2.7]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.6...v1.2.7
[1.2.6]: https://github.com/EdgeFirstAI/schemas/compare/v1.2.5...v1.2.6
[1.2.5]: https://github.com/EdgeFirstAI/schemas/releases/tag/v1.2.5
