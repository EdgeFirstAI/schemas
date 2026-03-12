# AI Assistant Development Guidelines — EdgeFirst Perception Schemas

Instructions for AI coding assistants (GitHub Copilot, Cursor, Claude Code, etc.) working on the EdgeFirst Perception Schemas project.

**Version:** 2.0
**Last Updated:** March 2026
**Applies To:** EdgeFirst Perception Schemas repository

---

## Table of Contents

1. [Overview](#overview)
2. [Zero-Copy Architecture](#zero-copy-architecture)
3. [Git Workflow](#git-workflow)
4. [Code Quality Standards](#code-quality-standards)
5. [Testing Requirements](#testing-requirements)
6. [Documentation Expectations](#documentation-expectations)
7. [License Policy](#license-policy)
8. [Security Practices](#security-practices)
9. [Project-Specific Guidelines](#project-specific-guidelines)

---

## Process Documentation Reference

**For Au-Zone Internal Developers:**

This project follows Au-Zone Technologies Software Process standards. Complete process documentation is available in the internal Software Process repository.

### Key Process Areas

**Git Workflow:**
- Branch naming: `<type>/PROJECTKEY-###[-description]`
- Commit format: `PROJECTKEY-###: Brief description`
- Multi-repository coordination: Use same JIRA key across related repos
- Release management: Semantic versioning with comprehensive pre-release checklist

**JIRA Integration:**
- Automatic ticket transitions on branch/PR events
- Time tracking with Tempo
- Blocker management and dependency tracking
- External contributors: Use `feature/<description>` format without JIRA keys

**SBOM and License Compliance:**
- Automated via `.github/scripts/generate_sbom.sh`
- Scancode-toolkit for unified scanning (all languages)
- License policy enforced in CI/CD via `.github/scripts/check_license_policy.py`
- MPL-2.0 allowed as dependency only, not in source code
- Parent directory scanning approach for optimal performance

**Testing Standards:**
- 70% minimum coverage per component/language
- Testing pyramid: Unit (70%+), Integration (20-25%), E2E (5-10%)
- Multi-language testing strategies (Rust + C + Python)
- Performance benchmarks for serialization/deserialization

**For External Contributors:**
- Use simplified workflow: `feature/<description>`, `bugfix/<description>`
- No JIRA key required
- Follow PR checklist in CONTRIBUTING.md
- License policy automatically enforced in CI/CD

---

## Overview

EdgeFirst Perception Schemas provides **zero-copy message schema definitions and multi-language bindings** (Rust, C, Python) for the EdgeFirst Perception middleware. It implements ROS2 Common Interfaces, Foxglove visualization types, and EdgeFirst custom message types using CDR1 Little-Endian wire encoding — compatible with ROS2 DDS but without requiring ROS2 as a dependency.

The project is designed for **resource-constrained embedded edge AI platforms** where every allocation and copy matters.

When contributing, AI assistants must prioritize:
- **Zero-copy correctness**: Serialization is a no-op; deserialization builds offset tables. Never introduce unnecessary copies.
- **Resource efficiency**: Memory, CPU, and power consumption matter on embedded devices
- **Cross-language consistency**: Rust, C, and Python APIs must mirror each other with byte-compatible serialization
- **Testing**: Comprehensive coverage with cross-language CDR round-trip validation
- **License compliance**: Strict adherence to approved open source licenses (Apache-2.0)

---

## Zero-Copy Architecture

**This is the single most important design constraint. Zero-copy is an ironclad requirement unless physically impossible.**

### How It Works

The schema library does **not** use traditional serialize/deserialize. Instead:

1. **Serialization is a no-op.** Messages are constructed directly in-place within a buffer. Setting a field writes it at the correct CDR offset in the underlying buffer. The buffer *is* the serialized form — there is no separate serialization step.

2. **Deserialization builds an offset table.** When receiving a CDR buffer, the library constructs an offset table that maps variable-length fields (strings, arrays, sequences) to their positions in the buffer. This is the only work done at "deserialization" time.

3. **Field access uses the offset table.** Applications read fields directly from the buffer using message "views" backed by the offset table. Fixed-size fields are accessed by computed offset. Variable-length fields are accessed via the offset table.

4. **Applications use buffers in-place.** The expectation is that applications hold onto the raw CDR buffer and use message view types with their offset tables as accessors for interpreting the data. No data is copied out of the buffer unless the application explicitly chooses to.

### What This Means for Contributors

- **Never introduce `memcpy` or buffer duplication** in the hot path. If you find yourself copying data between buffers, reconsider the approach.
- **Fixed-size message types** (e.g., `Time`, `Vector3`, `Quaternion`) use `CdrFixed` — they are `repr(C)` structs whose memory layout *is* the CDR encoding. They can be cast directly to/from byte slices.
- **Variable-length message types** use buffer-backed views with offset tables for field access.
- **DMA buffers** pass through as file descriptors — the schema library never touches pixel/sensor data.
- The C API (`ffi.rs`) exposes this zero-copy model: `from_cdr` functions take a buffer pointer and return a view; the buffer must outlive the view.

---

## Git Workflow

### Branch Naming Convention

**REQUIRED FORMAT**: `<type>/<PROJECTKEY-###>[-optional-description]`

**Branch Types:**
- `feature/` — New features and enhancements
- `bugfix/` — Non-critical bug fixes
- `hotfix/` — Critical production issues requiring immediate fix
- `release/` — Release preparation branches

**Examples:**
```bash
feature/EDGEAI-123-add-radar-schema
bugfix/STUDIO-456-fix-cdr-alignment
hotfix/MAIVIN-789-security-patch
release/2.1.0
```

**Rules:**
- JIRA key is REQUIRED for Au-Zone staff (format: `PROJECTKEY-###`)
- External contributors: Use `feature/<description>` referencing GitHub issue numbers
- Use kebab-case for descriptions (lowercase with hyphens)
- Branch from `main` for all work
- Release branches: `release/X.Y.Z`

### Commit Message Format

**REQUIRED FORMAT**: `PROJECTKEY-###: Brief description of what was done`

**Rules:**
- Subject line: 50–72 characters ideal
- Focus on WHAT changed, not HOW
- No type prefixes (`feat:`, `fix:`, etc.) — JIRA provides context
- Optional body: Use bullet points for additional detail
- Sign all commits with `-s` (DCO sign-off)

**Good examples:**
```bash
EDGEAI-123: Add RadarCube message type for raw radar data

EDGEAI-456: Fix CDR alignment for variable-length arrays
- Corrected offset calculation for nested sequences
- Added golden test data for validation

EDGEAI-789: Make C API from_cdr truly zero-copy
```

**Bad examples:**
```bash
fix bug                           # Missing JIRA key, too vague
feat(auth): add OAuth2           # Has type prefix (not our convention)
EDGEAI-123                       # Missing description
```

### Pull Request Process

**Requirements:**
- **2 approvals required** for merging to `main`
- All CI/CD checks must pass (test, format, clippy, coverage, SonarCloud)
- PR title: `PROJECTKEY-### Brief description of changes`
- PR description must link to JIRA ticket

**PR Description Template:**
```markdown
## JIRA Ticket
Link: [PROJECTKEY-###](https://au-zone.atlassian.net/browse/PROJECTKEY-###)

## Changes
Brief summary of what changed and why

## Testing
- [ ] Rust unit tests added/updated
- [ ] C API tests added/updated
- [ ] Python tests added/updated
- [ ] Cross-language CDR round-trip validation

## Checklist
- [ ] Code follows project conventions
- [ ] Zero-copy contract maintained (no unnecessary copies)
- [ ] Documentation updated
- [ ] No secrets or credentials committed
- [ ] License policy compliance verified
```

### JIRA Integration

- **Branch naming triggers automation**: Creating a branch with format `<type>/PROJECTKEY-###` automatically updates the linked JIRA ticket
- **PR creation triggers status updates**: Opening a PR moves tickets to review status
- **Merge triggers closure**: Merging a PR to main closes the associated ticket
- **Commit messages link to JIRA**: Format `PROJECTKEY-###: Description` creates automatic linkage

---

## Code Quality Standards

### General Principles

- **Zero-copy first**: Every design decision must preserve the zero-copy contract
- **Consistency**: Follow existing codebase patterns and conventions
- **Readability**: Code is read more often than written — optimize for comprehension
- **Simplicity**: Prefer simple, straightforward solutions over clever ones
- **Error Handling**: Use `Result` types in Rust, `errno` in C API, exceptions in Python
- **Performance**: Consider time/space complexity, especially for embedded deployment

### Language-Specific Standards

**Rust:**
- Use `cargo fmt` (stable toolchain) and `cargo clippy -- -D warnings`
- Follow Rust API Guidelines
- `repr(C)` for all types exposed via FFI
- `CdrFixed` for fixed-size types (direct memory-mapped CDR)
- No `serde` — the project uses its own zero-copy CDR implementation

**C API:**
- Generated header via `cbindgen` (`include/edgefirst/schemas.h`)
- Never manually edit the generated header
- Function naming: `ros_<namespace>_<type>_<method>` (e.g., `ros_sensor_image_width()`)
- EdgeFirst types: `ros_detect_<type>_<method>`
- All functions that can fail set `errno` and return an error indicator
- Consistent `EINVAL` for NULL pointer arguments

**Python:**
- Follow PEP 8; use `black` formatter (line length: 100)
- Type hints for function signatures
- Uses `pycdr2` for CDR serialization
- Google-style docstrings

### Code Review Checklist

Before submitting code, verify:
- [ ] Zero-copy contract preserved — no unnecessary buffer copies
- [ ] Code follows project style guidelines
- [ ] No commented-out code or debug statements
- [ ] Error handling is comprehensive (`Result` in Rust, `errno` in C, exceptions in Python)
- [ ] Public APIs have documentation
- [ ] `repr(C)` on all FFI-visible types
- [ ] No hardcoded values that should be configuration
- [ ] Resource cleanup (buffers, file handles) is proper
- [ ] Cross-language consistency maintained (Rust ↔ C ↔ Python)

### Performance Considerations

- **No allocations in the hot path**: Reuse buffers, avoid `Vec` growth in serialization
- **Offset tables are the only "deserialization" cost**: Keep them small and cache-friendly
- **DMA buffer passthrough**: File descriptors only — never touch pixel/sensor data
- **Target latency**: < 1ms serialization/deserialization for typical messages
- **Target throughput**: Support for high-frequency sensor data (>100Hz camera frames)
- **Benchmark with criterion.rs**: All performance-sensitive changes need benchmarks in `benches/`

---

## Testing Requirements

### Coverage Standards

- **Minimum coverage**: 70% (enforced via `cargo-llvm-cov` for Rust/C, `pytest-cov` for Python)
- **Critical paths**: 90%+ for CDR encoding/decoding and offset table construction
- **Cross-language**: CDR round-trip validation between Rust, C, and Python

### Test Infrastructure

**Coverage tooling:**
- **Rust + C combined**: `cargo-llvm-cov` instruments the Rust library; C tests link against the instrumented library, producing combined LCOV coverage
- **Python**: `pytest-cov` with XML output for SonarCloud
- **CI merges** Rust and C profraw files into a single LCOV report

**Test runners:**
- **Rust**: `cargo nextest` (with `--profile ci` for JUnit XML output)
- **C**: Criterion framework (`libcriterion-dev`), built via Makefile
- **Python**: `pytest`

### Test Structure

**Rust:**
- Unit tests co-located in `#[cfg(test)] mod tests` at end of each module
- Integration tests in `tests/`:
  - `tests/cdr_golden.rs` — Golden CDR files generated by pycdr2 for cross-language validation
  - `tests/mcap_test.rs` — Message recording/playback tests
- Benchmarks in `benches/serialization.rs` using criterion.rs

**C:**
- Tests in `tests/c/test_*.c` using Criterion framework
- One test file per message namespace (e.g., `test_sensor_msgs.c`, `test_edgefirst_msgs.c`)
- Built to `build/test_*` binaries via Makefile
- XML output for CI: `make test-c-xml`

**Python:**
- Tests in `tests/python/test_*.py` using pytest
- One test file per message namespace
- Shared fixtures in `conftest.py`
- Golden test data in `testdata/` (CDR files, MCAP recordings)

### Test Naming

- `test_<message_type>_<scenario>` format across all languages
- Be explicit about what is being tested (e.g., `test_image_cdr_round_trip`, `test_detect_null_data_returns_einval`)

### Running Tests

```bash
# Rust tests
cargo nextest run

# Rust tests with coverage
cargo llvm-cov nextest --all-features --workspace

# C tests (builds library automatically)
make test-c

# C tests with XML output (for CI)
make test-c-xml

# Python tests
source venv/bin/activate
pytest tests/python/ --cov=edgefirst --cov-report=term-missing -v

# All formatting and linting
cargo fmt -- --check
cargo clippy --all-targets --all-features -- -D warnings
```

---

## Documentation Expectations

### Code Documentation

**When to document:**
- Public APIs, functions, and types (ALWAYS)
- Zero-copy semantics and buffer lifetime requirements
- Offset table behavior for variable-length fields
- Performance considerations or optimization rationale
- Platform-specific behavior (DMA buffers, endianness)

**Rust doc comments:**
```rust
/// Decodes a CDR-encoded Image message from a buffer.
///
/// Returns a zero-copy view into the provided buffer. The buffer must
/// outlive the returned `Image` reference.
///
/// # Errors
/// Returns `CdrError::TooShort` if the buffer is smaller than the
/// minimum fixed-size header.
pub fn from_cdr(data: &[u8]) -> Result<Image<'_>, CdrError> {
```

**C API documentation lives in `CAPI.md`** — update it when adding or changing C API functions.

### Project Documentation

**Essential files:**
- `README.md` — Project overview, quick start, schema inventory
- `CONTRIBUTING.md` — Development setup, contribution process, release workflow
- `ARCHITECTURE.md` — Design principles, zero-copy model, language binding architecture
- `TESTING.md` — Comprehensive testing strategy and conventions
- `CAPI.md` — Complete C API reference
- `CHANGELOG.md` — All user-visible changes ([Keep a Changelog](https://keepachangelog.com/))

### Documentation Updates

When modifying code, update corresponding documentation:
- `README.md` if user-facing behavior changes
- `CAPI.md` if C API function signatures or semantics change
- `CHANGELOG.md` for all user-visible changes
- `ARCHITECTURE.md` if design patterns evolve

---

## License Policy

**CRITICAL**: Au-Zone has strict license policy for all dependencies.

### Allowed Licenses

✅ **Permissive licenses (APPROVED)**:
- MIT, Apache-2.0
- BSD-2-Clause, BSD-3-Clause
- ISC, 0BSD, Unlicense

### Review Required

⚠️ **Weak copyleft (REQUIRES LEGAL REVIEW)**:
- MPL-2.0 (as dependency only, never in source)
- LGPL-2.1-or-later, LGPL-3.0-or-later (if dynamically linked)

### Strictly Disallowed

❌ **NEVER USE**:
- GPL (any version), AGPL (any version)
- Creative Commons NC or ND variants
- SSPL, BSL (before conversion), OSL-3.0

### Verification Process

**Before adding dependencies:**
1. Check license compatibility (project is Apache-2.0)
2. Verify no GPL/AGPL in dependency tree
3. Regenerate SBOM: `.github/scripts/generate_sbom.sh`
4. Check policy: `python3 .github/scripts/check_license_policy.py sbom.json`
5. Update NOTICE: auto-generated from SBOM (never manually edit)

**CI/CD will automatically:**
- Generate SBOM using scancode-toolkit
- Validate CycloneDX SBOM schema
- Check for disallowed licenses
- Block PR merges if violations detected

---

## Security Practices

### Secure Coding Guidelines

**Input Validation:**
- Validate all CDR buffer sizes before access — never trust buffer length
- Bounds-check all offset table entries
- Validate string and array lengths against remaining buffer
- Return `EINVAL` / `CdrError` for malformed inputs — never panic

**Memory Safety:**
- Rust ownership model enforced — buffer must outlive message views
- C API: caller owns the buffer; views borrow it
- No raw pointer arithmetic outside of `unsafe` blocks in Rust
- Buffer overread protection in offset table construction

**Common Vulnerabilities to Avoid:**
- Buffer Overflows: Bounds checking on all CDR field access
- Integer Overflow: Validate length fields before arithmetic
- Use-After-Free: Document lifetime requirements in C API
- Path Traversal: Not applicable (no file I/O in library)

### Vulnerability Reporting

Use SECURITY.md process: email `support@au-zone.com` with subject "Security Vulnerability"

---

## Project-Specific Guidelines

### Technology Stack

- **Languages**:
  - Rust (primary implementation, edition 2021, MSRV 1.70+)
  - C (API via FFI, C11 standard, header generated by cbindgen)
  - Python 3.8+ (bindings using pycdr2)
- **Build system**:
  - Cargo for Rust (lib + staticlib + cdylib)
  - Makefile for C API tests (Criterion framework)
  - setuptools for Python (pyproject.toml)
  - Debian packaging for ROS2 integration (edgefirst_msgs/)
- **Dependencies (minimal by design)**:
  - Rust: `errno` 0.3, `libc` 0.2
  - Rust dev: `criterion` (benchmarks), `mcap`, `memmap2`, `rand`
  - Rust build: `cbindgen` (C header generation)
  - Python: `pycdr2` (CDR serialization)
  - C tests: `libcriterion-dev`
- **Serialization**: Custom zero-copy CDR1 Little-Endian (not serde)
- **Target platforms**: Linux across diverse processors
  - x86_64, ARM64 (aarch64), ARMv7, ARMv8, RISC-V
  - Primary targets: NXP i.MX 8M Plus, Qualcomm platforms
  - Tested on: Maivin and Raivin edge AI platforms
- **Cross-compilation**: `cargo-zigbuild` for cross-architecture builds
- **Coverage instrumentation**: `cargo-llvm-cov` for combined Rust + C coverage

### Architecture

**Pattern**: Schema definition → Zero-copy CDR implementation → Multi-language bindings

**Schema layers:**
1. **ROS2 Common Interfaces** (std_msgs, sensor_msgs, geometry_msgs, nav_msgs, builtin_interfaces, rosgraph_msgs) — Apache-2.0
2. **Foxglove Schemas** (visualization types) — MIT
3. **EdgeFirst Custom Schemas** (edgefirst_msgs: Detect, Box, Track, DmaBuffer, RadarCube, RadarInfo, Model, ModelInfo) — Apache-2.0

**Data flow:**
- CDR buffer allocation → In-place field writes ("serialization") → Zenoh publish
- Zenoh subscribe → Offset table construction ("deserialization") → Direct field access via views

**Communication:** Zenoh pub/sub middleware (ROS2-compatible but not ROS2-dependent)

**ROS2 Integration:** Compatible with ROS2 Humble via Zenoh-ROS2 DDS bridge

**Library outputs:**
- `libedgefirst_schemas.so` / `.dylib` / `.dll` (shared, with SONAME versioning on Linux)
- `libedgefirst_schemas.a` / `.lib` (static)
- `edgefirst-schemas` Rust crate
- `edgefirst-schemas` Python package (PyPI)
- `ros-humble-edgefirst-msgs` Debian package (ROS2)

### Build and Deployment

**Rust Library:**
```bash
cargo build --release          # Build release library (LTO, stripped)
cargo nextest run              # Run tests with nextest
cargo clippy --all-targets --all-features -- -D warnings
cargo fmt                      # Format code
cargo doc --no-deps --open     # Generate documentation
```

**C API Tests:**
```bash
make lib                       # Build Rust library
make test-c                    # Build and run C tests
make test-c-xml                # C tests with XML output (CI)
RELEASE=0 make test-c          # Debug build (for coverage)
```

**Python:**
```bash
source venv/bin/activate
pip install -e ".[test]"       # Install with test dependencies
pytest tests/python/ -v        # Run tests
```

**Cross-compilation:**
```bash
# Using cargo-zigbuild for cross-architecture builds
cargo zigbuild --release --target aarch64-unknown-linux-gnu
cargo zigbuild --release --target armv7-unknown-linux-gnueabihf
```

**Coverage:**
```bash
# Combined Rust + C coverage via cargo-llvm-cov
cargo llvm-cov nextest --all-features --workspace --no-report
# ... run C tests against instrumented library ...
cargo llvm-cov report --lcov --output-path lcov.info
```

**Debian Package (ROS2 Integration):**
```bash
source /opt/ros/humble/setup.bash
cd edgefirst_msgs
fakeroot debian/rules build
```

### CI/CD Pipelines

**Workflows (`.github/workflows/`):**

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| `test.yml` | All pushes, PRs | Version sync, format, Rust+C tests with coverage, Python tests, SonarCloud |
| `release.yml` | Tag `v*.*.*` | Build C API (x86_64 + aarch64), collect Debian packages, create GitHub Release, publish to crates.io + PyPI |
| `benchmark.yml` | Manual dispatch | Run criterion.rs benchmarks on NXP i.MX 8M Plus hardware |
| `ros.yml` | All pushes | Build ROS2 Debian packages |
| `sbom.yml` | Scheduled | SBOM generation for license compliance |

**CI scripts (`.github/scripts/`):**
- `check_version_sync.sh` — Validates Cargo.toml, `__init__.py`, and package.xml versions match
- `generate_sbom.sh` — Generates CycloneDX SBOM from dependency tree
- `generate_notice.py` — Produces NOTICE file from SBOM
- `check_license_policy.py` — Validates all dependencies against license allowlist

### Version Management

**Three synchronized version sources (managed by `cargo-release`):**
1. `Cargo.toml` — Rust crate version
2. `edgefirst/schemas/__init__.py` — Python `__version__`
3. `edgefirst_msgs/package.xml` — ROS2 package version

**Current version:** 2.0.0

**Semantic Versioning (SemVer 2.0.0):**
- **MAJOR**: Breaking CDR wire format changes, removed message types, incompatible API changes
- **MINOR**: New message types, new fields with defaults, new helper functions
- **PATCH**: Bug fixes, performance improvements, documentation corrections

**Release process:** See `CONTRIBUTING.md` for full cargo-release workflow.

### Hardware Specifics

Optimized for EdgeFirst hardware platforms:

- **Target Devices:**
  - Maivin: NXP i.MX 8M Plus based edge AI platform
  - Raivin: Qualcomm-based edge AI platform
  - Custom designs using supported SoCs
- **NPU Integration**: Schemas support zero-copy DMA buffer passing to NXP i.MX 8M Plus NPU (via VAAL) and Qualcomm Hexagon DSP
- **Sensor Interfaces**: Message types for V4L2 cameras (DMA buffer file descriptors), MIPI CSI-2, IMU (I2C/SPI), GNSS/GPS (UART), radar, LiDAR
- **Platform Quirks**: DMA buffer handling is platform-specific; colorspace conversions may differ between platforms

### Special Considerations

**Zero-copy is non-negotiable:**
- All new message types must follow the zero-copy pattern
- Fixed-size types: `repr(C)` + `CdrFixed` trait
- Variable-length types: buffer-backed with offset table
- If a design cannot be zero-copy, document why and get explicit approval

**Cross-language consistency:**
- Rust, C, and Python APIs must mirror each other
- Field names and types must match exactly
- CDR encoding must be byte-compatible across all three languages
- Golden CDR test data (generated by pycdr2) validates cross-language compatibility

**Schema compatibility:**
- Maintain ROS2 Humble compatibility for standard messages
- Follow ROS2 message definition conventions
- Test compatibility with ROS2 tools (rosbag, rviz via Zenoh bridge)

---

## Working with AI Assistants

### Common Pitfalls for This Project

- **Do not introduce serde**: This project uses custom zero-copy CDR, not serde. Never add `#[derive(Serialize, Deserialize)]`.
- **Do not copy buffers**: If your implementation copies data out of a CDR buffer, you're violating the zero-copy contract. Rethink the approach.
- **Do not manually edit `include/edgefirst/schemas.h`**: It is generated by cbindgen from `build.rs`.
- **Verify cross-language consistency**: When adding a message type, it must be added to Rust, C (via FFI), and Python with matching field names and CDR layout.
- **Test with golden data**: New message types should have golden CDR test data for cross-language validation.

---

*This document helps AI assistants contribute effectively to EdgeFirst Perception Schemas while maintaining zero-copy correctness, quality, security, and consistency.*
*Organization: Au-Zone Technologies | License: Apache-2.0*
