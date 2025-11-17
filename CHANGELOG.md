# Changelog

All notable changes to EdgeFirst Perception Schemas will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2025-11-17

### Added

**Core Documentation:**
- README.md with comprehensive project overview and usage examples
- CONTRIBUTING.md with development guidelines and dependency management process
- SECURITY.md with vulnerability reporting and security best practices
- ARCHITECTURE.md with technical architecture and EdgeFirst Perception integration details
- AGENTS.md with AI assistant development guidelines
- CODE_OF_CONDUCT.md following Contributor Covenant

**Source Code Compliance:**
- SPDX headers (Apache-2.0) added to all source files:
  - 9 Rust files (src/*.rs)
  - 8 Python files (edgefirst/schemas/*.py)
  - 11 message definitions (edgefirst_msgs/msg/*.msg)
- Complete Apache-2.0 LICENSE file
- Cargo.toml license metadata

**SBOM Infrastructure:**
- Automated SBOM generation using merged workflow (cargo-cyclonedx + scancode-toolkit)
- License policy validation with configurable allow/deny lists
- NOTICE file generation for third-party attributions
- GitHub Actions workflow for CI/CD automation (sbom.yml)
- SBOM artifacts available in GitHub Actions and releases

**Message Types:**
- ROS2 Common Interfaces (std_msgs, sensor_msgs, geometry_msgs, nav_msgs, builtin_interfaces)
- Foxglove visualization schemas (SceneUpdate, ImageAnnotations, Grid)
- EdgeFirst custom messages:
  - Detect, Box, Track (object detection and tracking)
  - DmaBuffer (zero-copy hardware buffers)
  - RadarCube, RadarInfo (radar sensor data)
  - Model, ModelInfo (inference metadata)
  - Mask, Date, LocalTime (supporting types)

**Language Bindings:**
- Rust implementation with serde-based CDR serialization
- Python bindings with pycdr2 compatibility
- PointCloud2 decode utilities for both Rust and Python
- ROS2 Humble message compatibility

**Integration Features:**
- Zenoh pub/sub message patterns
- ROS2 DDS bridge compatibility via Zenoh
- DMA buffer file descriptor sharing for zero-copy (Linux)
- CDR (Common Data Representation) encoding

### Changed
- Updated Cargo.toml description to reflect EdgeFirst Perception focus
- Updated homepage URL to https://www.edgefirst.ai
- Added keywords and categories for crates.io discoverability

### Documentation
- ARCHITECTURE.md includes actual EdgeFirst Perception topic names (rt/camera/*, rt/radar/*, etc.)
- Real-world message flow examples showing camera, radar, and sensor fusion
- DMA buffer documentation with Linux kernel API details (pidfd_open/pidfd_getfd)
- Comprehensive SBOM and NOTICE maintenance procedures in CONTRIBUTING.md

[Unreleased]: https://github.com/EdgeFirstAI/schemas/compare/0.1.0...HEAD
[0.1.0]: https://github.com/EdgeFirstAI/schemas/releases/tag/0.1.0
