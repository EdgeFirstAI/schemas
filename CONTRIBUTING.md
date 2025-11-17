# Contributing to EdgeFirst Perception Schemas

Thank you for your interest in contributing to EdgeFirst Perception Schemas! This document provides guidelines for contributing to this project.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Setup](#development-setup)
- [How to Contribute](#how-to-contribute)
- [Code Style Guidelines](#code-style-guidelines)
- [Dependency Management](#dependency-management)
- [Testing Requirements](#testing-requirements)
- [Pull Request Process](#pull-request-process)
- [Release Process](#release-process)
- [License](#license)

## Code of Conduct

This project adheres to the [Contributor Covenant Code of Conduct](CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code. Please report unacceptable behavior to support@au-zone.com.

## Getting Started

EdgeFirst Perception Schemas provides message schema definitions and language bindings (Rust, Python) for the EdgeFirst Perception middleware. Before contributing:

1. Read the [README.md](README.md) to understand the project
2. Browse the [EdgeFirst Documentation](https://doc.edgefirst.ai/latest/perception/) for context
3. Check existing [issues](https://github.com/EdgeFirstAI/schemas/issues) and [discussions](https://github.com/EdgeFirstAI/schemas/discussions)
4. Review the [EdgeFirst Samples](https://github.com/EdgeFirstAI/samples) to see usage examples

### Ways to Contribute

- **Code**: Bug fixes, new message types, performance improvements
- **Documentation**: README improvements, code comments, examples
- **Testing**: Unit tests, integration tests, validation scripts
- **Community**: Answer questions, write tutorials, share use cases

## Development Setup

### Prerequisites

- **Rust**: 1.70 or later ([install instructions](https://www.rust-lang.org/tools/install))
- **Python**: 3.8 or later
- **Git**: For version control

### Clone and Build

```bash
# Clone the repository
git clone https://github.com/EdgeFirstAI/schemas.git
cd schemas

# Build Rust bindings
cargo build

# Run Rust tests (when available)
cargo test

# Build Python bindings
python -m pip install -e .

# Run Python tests (when available)
python -m pytest
```

### Rust Development

```bash
# Build with all features
cargo build --all-features

# Format code
cargo fmt

# Run linter
cargo clippy -- -D warnings

# Generate documentation
cargo doc --open
```

### Python Development

```bash
# Install development dependencies
pip install -e ".[dev]"

# Format code
black edgefirst/
autopep8 --in-place --recursive edgefirst/

# Type checking
mypy edgefirst/

# Run linter
pylint edgefirst/
```

### ROS2 Debian Package (Optional)

```bash
# Build ROS2 package
cd edgefirst_msgs
source /opt/ros/humble/setup.bash
fakeroot debian/rules build
```

## How to Contribute

### Reporting Bugs

Before creating bug reports, please check existing issues to avoid duplicates.

**Good Bug Reports** include:

- Clear, descriptive title
- Steps to reproduce the behavior
- Expected vs. actual behavior
- Environment details (OS, Rust/Python version)
- Minimal code example demonstrating the issue
- Screenshots if applicable

### Suggesting Enhancements

Enhancement suggestions are tracked as GitHub issues. Provide:

- Clear, descriptive title
- Detailed description of the proposed functionality
- Use cases and motivation
- Examples of how the feature would be used
- Possible implementation approach (optional)

### Contributing Code

1. **Fork the repository** and create your branch from `main`
2. **Make your changes** following our code style guidelines
3. **Add tests** for new functionality (minimum 70% coverage)
4. **Ensure all tests pass** (`cargo test`, `pytest`)
5. **Update documentation** for API changes
6. **Run formatters and linters** (`cargo fmt`, `cargo clippy`, `black`)
7. **Submit a pull request** with a clear description

## Code Style Guidelines

### Rust

- Follow [Rust API Guidelines](https://rust-lang.github.io/api-guidelines/)
- Use `cargo fmt` (enforced in CI)
- Address all `cargo clippy` warnings
- Write doc comments for public APIs using `///`
- Maximum line length: 100 characters
- Use descriptive variable names

**Example:**

```rust
/// Decodes a PointCloud2 message field.
///
/// # Arguments
/// * `field` - The PointField definition
/// * `data` - Raw point cloud data
///
/// # Returns
/// Decoded field value as f64
pub fn decode_field(field: &PointField, data: &[u8]) -> f64 {
    // Implementation...
}
```

### Python

- Follow [PEP 8](https://pep8.org/) style guide
- Use `black` for formatting (line length: 100)
- Use type hints for function signatures
- Write docstrings for public functions (Google style)
- Use meaningful variable names

**Example:**

```python
def decode_pcd(pcd: PointCloud2) -> list[NamedTuple]:
    """
    Decodes the points from a PointCloud2 ROS message.

    Args:
        pcd: PointCloud2 message to decode

    Returns:
        List of named tuples with point data fields

    Example:
        >>> points = decode_pcd(pcd)
        >>> xyz = (points[0].x, points[0].y, points[0].z)
    """
    # Implementation...
```

### Message Definitions (.msg files)

- Follow [ROS2 Message Style Guide](https://docs.ros.org/en/humble/Contributing/Developer-Guide.html#msg-srv-action-style-guide)
- Use clear, descriptive field names
- Include comments for non-obvious fields
- Group related fields logically

## Dependency Management

### Adding New Dependencies

When adding dependencies to `Cargo.toml` or `pyproject.toml`, follow this process:

1. **Check License Compatibility**
   - Only use permissive licenses: MIT, Apache-2.0, BSD (2/3-clause), ISC
   - Avoid GPL, AGPL, or restrictive licenses
   - Review the entire dependency tree for license compliance

2. **Update Cargo.lock**
   ```bash
   # After modifying Cargo.toml
   cargo build
   git add Cargo.lock
   ```

3. **Regenerate NOTICE File**
   ```bash
   # Generate complete SBOM and updated NOTICE
   .github/scripts/generate_sbom.sh

   # Review the updated NOTICE file
   git diff NOTICE

   # Commit if changes are present
   git add NOTICE
   ```

4. **Verify License Policy**
   ```bash
   # Check for license violations
   python3 .github/scripts/check_license_policy.py sbom.json
   ```

5. **Include in Commit**
   - Commit `Cargo.lock` (or `poetry.lock`/`requirements.txt` for Python)
   - Commit updated `NOTICE` file
   - Reference NOTICE update in commit message or PR description

**Example commit message:**
```
EDGEAI-123: Add serde_json dependency for JSON serialization

- Added serde_json 1.0 (MIT license)
- Updated Cargo.lock and NOTICE file
- All dependencies pass license policy check
```

### Regular SBOM Audits

Project maintainers should perform regular SBOM audits:

**Quarterly Review:**
- Regenerate SBOM: `.github/scripts/generate_sbom.sh`
- Review dependency updates and security advisories
- Update NOTICE file if dependencies changed
- Check for deprecated or unmaintained dependencies

**Before Each Release:**
- Regenerate SBOM and NOTICE
- Verify all licenses are compliant
- Update dependency versions if security issues exist
- Commit updated NOTICE to release branch

**CI/CD Automation:**
- GitHub Actions automatically generates SBOM on every commit
- License policy violations fail the build
- SBOM artifacts are available for every PR and release

### NOTICE File Maintenance

The `NOTICE` file is **committed to the repository** and tracks third-party attributions.

**When to update NOTICE:**
- When adding new dependencies to the project
- When updating dependencies (if licenses change)
- During regular SBOM audits (quarterly)
- Before tagging a new release

**How to update NOTICE:**
```bash
# Automatic regeneration
.github/scripts/generate_sbom.sh

# The script will:
# 1. Scan Cargo.lock for Rust dependencies
# 2. Scan source files for embedded licenses
# 3. Merge into complete SBOM (sbom.json)
# 4. Generate updated NOTICE file
```

**Do NOT manually edit NOTICE** - it is auto-generated from the SBOM.

**NOTICE references the official SBOM:**
The NOTICE file points to the CycloneDX JSON SBOM (sbom.json) included in
GitHub release assets. This provides the complete dependency inventory.

## Testing Requirements

### Minimum Coverage

All contributions with new functionality must include tests:

- **Rust**: Minimum 70% code coverage
- **Python**: Minimum 70% code coverage
- Critical paths (serialization/deserialization) require 100% coverage

### Rust Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_cloud_decode() {
        // Test implementation...
    }

    #[test]
    fn test_message_serialization() {
        // Test round-trip serialization...
    }
}
```

### Python Tests

```python
import pytest
from edgefirst.schemas import decode_pcd, PointCloud2

def test_decode_pcd_basic():
    """Test basic point cloud decoding."""
    # Test implementation...
    pass

def test_decode_pcd_empty():
    """Test handling of empty point cloud."""
    # Test implementation...
    pass
```

### Running Tests

```bash
# Rust tests
cargo test
cargo test --all-features

# Python tests
pytest
pytest --cov=edgefirst --cov-report=html

# Check coverage
cargo tarpaulin --out Html
```

## Pull Request Process

### Branch Naming

For external contributors:

```
feature/<description>       # New features
bugfix/<description>        # Bug fixes
docs/<description>          # Documentation updates
```

**Examples:**
- `feature/add-radar-message`
- `bugfix/pointcloud-decode`
- `docs/update-python-examples`

### Commit Messages

Write clear, concise commit messages:

```
Add RadarCube message type for raw radar data

- Define RadarCube message structure
- Add Rust and Python bindings
- Include dimension labels for R, D, A axes
- Add comprehensive tests

Addresses discussion #123
```

**Guidelines:**
- Use imperative mood ("Add feature" not "Added feature")
- First line: 50 characters or less
- Body: Wrap at 72 characters
- Reference issues/discussions when applicable

### Pull Request Checklist

Before submitting, ensure:

- [ ] Code follows style guidelines (`cargo fmt`, `cargo clippy`, `black`)
- [ ] All tests pass (`cargo test`, `pytest`)
- [ ] New tests added for new functionality (70% coverage minimum)
- [ ] Documentation updated for API changes
- [ ] Commit messages are clear and descriptive
- [ ] Branch is up-to-date with `main`
- [ ] PR description clearly explains changes and motivation
- [ ] SPDX headers present in new files

### Review Process

1. **Automated Checks**: CI/CD runs tests, linters, formatters
2. **Maintainer Review**: Code review by project maintainers
3. **Community Feedback**: Other contributors may provide input
4. **Approval**: At least one maintainer approval required
5. **Merge**: Maintainer merges upon approval

### After Your Pull Request is Merged

- Update your local repository: `git pull upstream main`
- Delete your feature branch (optional)
- Celebrate! Thank you for contributing! 🎉

---

## Release Process

**For Project Maintainers Only**

EdgeFirst Perception Schemas follows the same release conventions as [edgefirst-client](https://github.com/EdgeFirstAI/client):

### Version Management

EdgeFirst Perception Schemas is a **multi-language project** (Rust + Python + ROS2) requiring version synchronization:

#### Version Sources

**Three sources are kept automatically synchronized using cargo-release:**

1. **Cargo.toml** (Rust ecosystem):
   ```toml
   [package]
   version = "0.2.0"
   ```

2. **edgefirst/schemas/__init__.py** (Python ecosystem):
   ```python
   __version__ = "0.2.0"
   ```

3. **edgefirst_msgs/package.xml** (ROS2 ecosystem):
   ```xml
   <version>0.2.0</version>
   ```

**Why three sources?**
- Rust (crates.io), Python (PyPI), and ROS2 (Debian packages) are separate packaging ecosystems
- Each requires its own version declaration
- This is standard practice for multi-language projects
- **cargo-release automatically synchronizes all three versions**

#### Version Access

**Rust**: Version is embedded in crate metadata
```rust
// Automatically available via Cargo.toml
const VERSION: &str = env!("CARGO_PKG_VERSION");
```

**Python**: Version accessible via `__version__` attribute
```python
import edgefirst.schemas
print(edgefirst.schemas.__version__)  # "0.2.0"
```

**Installed package**: Use `importlib.metadata`
```python
from importlib.metadata import version
print(version("edgefirst-schemas"))  # "0.2.0"
```

#### Semantic Versioning

Follow [SemVer 2.0.0](https://semver.org/):

- **MAJOR** (X.0.0): Incompatible API changes
  - Removing message types or fields
  - Changing field types in incompatible ways
  - Breaking serialization compatibility
- **MINOR** (0.X.0): Backwards-compatible functionality additions
  - Adding new message types
  - Adding new fields (with defaults)
  - New decode utilities or helper functions
- **PATCH** (0.0.X): Backwards-compatible bug fixes
  - Fixing serialization bugs
  - Documentation corrections
  - Performance improvements (no API changes)

#### Other Conventions

- **Tag format**: 'v' prefix (e.g., `v0.1.0`)
- **Changelog**: Maintain [CHANGELOG.md](CHANGELOG.md) following [Keep a Changelog](https://keepachangelog.com/)
- **Pre-releases**: Use `-rc1`, `-rc2` suffixes for release candidates (e.g., `v0.2.0-rc1`)

### Release Steps

EdgeFirst uses **cargo-release** to automate version management across all three languages.

#### Release Workflow Overview

```
Developer Actions                GitHub Actions (Automated)
─────────────────                ──────────────────────────
                                
1. cargo release version patch    
   ↓ (updates 3 version files)
                                
2. Update CHANGELOG.md manually
   ↓
                                
3. cargo release commit
   ↓ (commits changes)
                                
4. cargo release tag
   ↓ (creates tag: v0.2.0)
                                
5. cargo release push          → Tag pushed to GitHub
   ↓                             ↓
                                 ├→ release.yml
                                 │  └→ Creates GitHub Release
                                 │     - Extracts CHANGELOG
                                 │     - Attaches SBOM
                                 │
                                 ├→ rust.yml  
                                 │  └→ Publishes to crates.io
                                 │
                                 ├→ pypi.yml
                                 │  └→ Publishes to PyPI
                                 │
                                 └→ debian.yml
                                    └→ Builds .deb packages
```

**Key Points:**
- GitHub Release is **automatically created** when tag is pushed
- All publishing (crates.io, PyPI) happens **in parallel** from tag push
- No manual GitHub Release creation needed
- PyPI is **not** triggered by GitHub Release - it's triggered by the tag

#### Prerequisites

```bash
# Install cargo-release if not already installed
cargo install cargo-release
```

#### Standard Release Process

1. **Ensure clean state**
   ```bash
   # Verify you're on main branch
   git checkout main
   git pull origin main

   # Ensure working directory is clean
   git status  # Should show "nothing to commit, working tree clean"
   ```

2. **Bump version in Cargo.toml**
   ```bash
   # For patch release (0.1.0 -> 0.1.1)
   cargo release version patch --execute

   # For minor release (0.1.0 -> 0.2.0)
   cargo release version minor --execute

   # For major release (0.1.0 -> 1.0.0)
   cargo release version major --execute

   # Or specify exact version
   cargo release version 1.4.0 --execute
   ```

   This command updates `Cargo.toml` version only.

3. **Update Python and ROS2 versions**
   ```bash
   cargo release replace --execute
   ```

   This runs pre-release-replacements to update:
   - `edgefirst/schemas/__init__.py` __version__
   - `edgefirst_msgs/package.xml` <version>
   - `CHANGELOG.md` (adds new version section)

4. **Review version changes**
   ```bash
   git diff

   # Verify all three versions match
   .github/scripts/check_version_sync.sh
   # Should output: ✅ All versions are synchronized!
   ```

5. **Update CHANGELOG.md content manually**
   
   The `replace` step added the version header, now add the actual changes:
   
   ```markdown
   ## [Unreleased]

   ## [0.2.0] - 2025-11-20  # <-- Added by 'replace' step

   ### Added                # <-- Add your changes here
   - New feature descriptions

   ### Changed
   - Modified behavior descriptions

   ### Fixed
   - Bug fix descriptions
   ```

6. **Regenerate SBOM and NOTICE** (if dependencies changed)
   ```bash
   .github/scripts/generate_sbom.sh
   git diff NOTICE
   ```

7. **Commit version bump**
   ```bash
   cargo release commit --execute
   ```

   This creates a commit with message: "Release 0.2.0"

8. **Create release tag**
   ```bash
   cargo release tag --execute
   ```

   This creates a git tag (e.g., `v0.2.0` with 'v' prefix)

9. **Push to GitHub**
   ```bash
   cargo release push --execute
   ```

   This pushes both the commit and tag to origin

10. **Automated GitHub Actions workflow**
   
   Pushing the tag triggers multiple workflows automatically:
   
   **a) GitHub Release Creation** (`.github/workflows/release.yml`)
   - Extracts version from tag name
   - Extracts release notes from CHANGELOG.md
   - Generates SBOM (Software Bill of Materials)
   - Creates GitHub Release with:
     - Version number as title
     - Changelog section as description
     - SBOM attached as artifact
     - Marked as pre-release if version contains `-` (e.g., `0.2.0-rc1`)
   
   **b) Rust Publishing** (`.github/workflows/rust.yml`)
   - Runs tests and formatting checks
   - Publishes to crates.io using version from Cargo.toml
   
   **c) Python Publishing** (`.github/workflows/pypi.yml`)
   - Builds Python wheel using version from `__init__.py`
   - Publishes to PyPI
   
   **d) Debian Package Build** (`.github/workflows/debian.yml`)
   - Extracts version from package.xml
   - Builds .deb packages for AMD64 and ARM64
   - Uploads artifacts (can be attached to GitHub Release manually if needed)

10. **Verify successful release**
    
    Monitor the following:
    - GitHub Actions: All workflows complete successfully
    - GitHub Release: Created at https://github.com/EdgeFirstAI/schemas/releases
    - crates.io: Published at https://crates.io/crates/edgefirst-schemas
    - PyPI: Published at https://pypi.org/project/edgefirst-schemas/

#### Quick Release (All Steps Combined)

For experienced users who want to automate the entire process:

```bash
# Bump version and run all steps in one command
cargo release patch --execute

# Or specify exact version
cargo release 1.4.0 --execute
```

This runs all steps in sequence:
1. `version` - Updates Cargo.toml
2. `replace` - Updates __init__.py, package.xml, CHANGELOG.md
3. `commit` - Commits all changes
4. `tag` - Creates git tag
5. `push` - Pushes to remote

**Note**: You still need to manually edit CHANGELOG.md content before running this command (the `replace` step only adds the version header, not the actual changes).

### Pre-Release Checklist

Before creating a release, verify:

- [ ] All tests pass (`cargo test`, `pytest`)
- [ ] Documentation is up-to-date
- [ ] CHANGELOG.md updated with all changes since last release (following [Keep a Changelog](https://keepachangelog.com/) format)
- [ ] Versions synchronized via `cargo release version <level>` (validates all three: Cargo.toml, __init__.py, package.xml)
- [ ] NOTICE file regenerated if dependencies changed
- [ ] No uncommitted changes (`git status` clean)
- [ ] On `main` branch with latest changes pulled
- [ ] Version sync check passes: `.github/scripts/check_version_sync.sh`

### GitHub Actions Release Workflow

**Trigger**: Pushing a tag (format: `vX.Y.Z` or `vX.Y.Z-suffix` with 'v' prefix)

**Automated Steps**:

1. **GitHub Release** - Automatically created with:
   - Release title: Version number (e.g., "0.2.0")
   - Description: Extracted from CHANGELOG.md for that version
   - Assets: SBOM (sbom.json)
   - Pre-release flag: Set if version contains `-` (e.g., `0.2.0-rc1`)

2. **crates.io** - Rust crate published automatically

3. **PyPI** - Python package published automatically

4. **Debian packages** - Built for AMD64 and ARM64, available as workflow artifacts

**Important**: 
- The GitHub Release is created **automatically by the tag push**, not manually
- PyPI publishing is triggered by the **tag push**, not by the GitHub Release creation
- All publishing happens in parallel when the tag is pushed

### Troubleshooting Releases

**If a workflow fails:**
1. Check GitHub Actions logs for specific error
2. Common issues:
   - Missing secrets (CARGO_REGISTRY_TOKEN, PyPI credentials)
   - Version already exists on crates.io or PyPI
   - Test failures blocking deployment
3. Fix the issue and re-tag (delete old tag first):
   ```bash
   git tag -d 0.2.0
   git push origin :refs/tags/0.2.0
   # Fix the issue, then re-run cargo release
   ```

---

## License

By contributing to EdgeFirst Perception Schemas, you agree that your contributions will be licensed under the [Apache License 2.0](LICENSE).

All source files must include the SPDX license header:

**Rust/Python:**
```
// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.
```

**Message files:**
```
# SPDX-License-Identifier: Apache-2.0
# Copyright © 2025 Au-Zone Technologies. All Rights Reserved.
```

## Questions?

- **Documentation**: https://doc.edgefirst.ai/latest/perception/
- **Discussions**: https://github.com/EdgeFirstAI/schemas/discussions
- **Issues**: https://github.com/EdgeFirstAI/schemas/issues
- **Email**: support@au-zone.com

Thank you for helping make EdgeFirst Perception better!
