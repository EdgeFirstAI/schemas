# AGENTS.md - AI Assistant Development Guidelines

This document provides instructions for AI coding assistants (GitHub Copilot, Cursor, Claude Code, etc.) working on Au-Zone Technologies projects. These guidelines ensure consistent code quality, proper workflow adherence, and maintainable contributions.

**Version:** 1.0
**Last Updated:** November 2025
**Applies To:** All Au-Zone Technologies software repositories

---

## Table of Contents

1. [Overview](#overview)
2. [Git Workflow](#git-workflow)
3. [Code Quality Standards](#code-quality-standards)
4. [Testing Requirements](#testing-requirements)
5. [Documentation Expectations](#documentation-expectations)
6. [License Policy](#license-policy)
7. [Security Practices](#security-practices)
8. [Project-Specific Guidelines](#project-specific-guidelines)

---

## Overview

Au-Zone Technologies develops edge AI and computer vision solutions for resource-constrained embedded devices. Our software spans:
- Edge AI inference engines and model optimization tools
- Computer vision processing pipelines
- Embedded Linux device drivers and system software
- MLOps platform (EdgeFirst Studio) for model deployment and management
- Open source libraries and tools (Apache-2.0 licensed)

When contributing to Au-Zone projects, AI assistants should prioritize:
- **Resource efficiency**: Memory, CPU, and power consumption matter on embedded devices
- **Code quality**: Maintainability, readability, and adherence to established patterns
- **Testing**: Comprehensive coverage with unit, integration, and edge case tests
- **Documentation**: Clear explanations for complex logic and public APIs
- **License compliance**: Strict adherence to approved open source licenses

---

## Git Workflow

### Branch Naming Convention

**REQUIRED FORMAT**: `<type>/<PROJECTKEY-###>[-optional-description]`

**Branch Types:**
- `feature/` - New features and enhancements
- `bugfix/` - Non-critical bug fixes
- `hotfix/` - Critical production issues requiring immediate fix

**Examples:**
```bash
feature/EDGEAI-123-add-authentication
bugfix/STUDIO-456-fix-memory-leak
hotfix/MAIVIN-789-security-patch

# Minimal format (JIRA key only)
feature/EDGEAI-123
bugfix/STUDIO-456
```

**Rules:**
- JIRA key is REQUIRED (format: `PROJECTKEY-###`)
- Description is OPTIONAL but recommended for clarity
- Use kebab-case for descriptions (lowercase with hyphens)
- Branch from `develop` for features/bugfixes, from `main` for hotfixes

### Commit Message Format

**REQUIRED FORMAT**: `PROJECTKEY-###: Brief description of what was done`

**Rules:**
- Subject line: 50-72 characters ideal
- Focus on WHAT changed, not HOW (implementation details belong in code)
- No type prefixes (`feat:`, `fix:`, etc.) - JIRA provides context
- Optional body: Use bullet points for additional detail

**Examples of Good Commits:**
```bash
EDGEAI-123: Add JWT authentication to user API

STUDIO-456: Fix memory leak in CUDA kernel allocation

MAIVIN-789: Optimize tensor operations for inference
- Implemented tiled memory access pattern
- Reduced memory bandwidth by 40%
- Added benchmarks to verify improvements
```

**Examples of Bad Commits:**
```bash
fix bug                           # Missing JIRA key, too vague
feat(auth): add OAuth2           # Has type prefix (not our convention)
EDGEAI-123                       # Missing description
edgeai-123: update code          # Lowercase key, vague description
```

### Pull Request Process

**Requirements:**
- **2 approvals required** for merging to `main`
- **1 approval required** for merging to `develop`
- All CI/CD checks must pass
- PR title: `PROJECTKEY-### Brief description of changes`
- PR description must link to JIRA ticket

**PR Description Template:**
```markdown
## JIRA Ticket
Link: [PROJECTKEY-###](https://au-zone.atlassian.net/browse/PROJECTKEY-###)

## Changes
Brief summary of what changed and why

## Testing
- [ ] Unit tests added/updated
- [ ] Integration tests pass
- [ ] Manual testing completed

## Checklist
- [ ] Code follows project conventions
- [ ] Documentation updated
- [ ] No secrets or credentials committed
- [ ] License policy compliance verified
```

**Process:**
1. Create PR via GitHub/Bitbucket web interface
2. Link to JIRA ticket in description
3. Wait for CI/CD to complete successfully
4. Address reviewer feedback through additional commits
5. Obtain required approvals
6. Merge using squash or rebase to keep history clean

### JIRA Integration

While full JIRA details are internal, contributors should know:
- **Branch naming triggers automation**: Creating a branch with format `<type>/PROJECTKEY-###` automatically updates the linked JIRA ticket
- **PR creation triggers status updates**: Opening a PR moves tickets to review status
- **Merge triggers closure**: Merging a PR to main/develop closes the associated ticket
- **Commit messages link to JIRA**: Format `PROJECTKEY-###: Description` creates automatic linkage

**Note**: External contributors without JIRA access can use branch naming like `feature/issue-123-description` referencing GitHub issue numbers instead.

---

## Code Quality Standards

### General Principles

- **Consistency**: Follow existing codebase patterns and conventions
- **Readability**: Code is read more often than written - optimize for comprehension
- **Simplicity**: Prefer simple, straightforward solutions over clever ones
- **Error Handling**: Validate inputs, sanitize outputs, provide actionable error messages
- **Performance**: Consider time/space complexity, especially for edge deployment

### Language-Specific Standards

Follow established conventions for each language:
- **Rust**: Use `cargo fmt` and `cargo clippy`; follow Rust API guidelines
- **Python**: Follow PEP 8; use autopep8 formatter (or project-specified tool); type hints preferred
- **C/C++**: Follow project's .clang-format; use RAII patterns
- **Go**: Use `go fmt`; follow Effective Go guidelines
- **JavaScript/TypeScript**: Use ESLint; Prettier formatter; prefer TypeScript

### Code Quality Tools

**SonarQube Integration:**
- Projects with `sonar-project.properties` must follow SonarQube guidelines
- Verify code quality using:
  - MCP integration for automated checks
  - VSCode SonarLint plugin for real-time feedback
  - SonarCloud reports in CI/CD pipeline
- Address critical and high-severity issues before submitting PR
- Maintain or improve project quality gate scores

### Code Review Checklist

Before submitting code, verify:
- [ ] Code follows project style guidelines (check `.editorconfig`, `CONTRIBUTING.md`)
- [ ] No commented-out code or debug statements
- [ ] Error handling is comprehensive and provides useful messages
- [ ] Complex logic has explanatory comments
- [ ] Public APIs have documentation
- [ ] No hardcoded values that should be configuration
- [ ] Resource cleanup (memory, file handles, connections) is proper
- [ ] No obvious security vulnerabilities (SQL injection, XSS, etc.)
- [ ] SonarQube quality checks pass (if applicable)

### Performance Considerations

For edge AI applications, always consider:
- **Memory footprint**: Minimize allocations; reuse buffers where possible
- **CPU efficiency**: Profile critical paths; optimize hot loops
- **Power consumption**: Reduce wake-ups; batch operations
- **Latency**: Consider real-time requirements for vision processing
- **Hardware acceleration**: Leverage NPU/GPU/DSP when available

---

## Testing Requirements

### Coverage Standards

- **Minimum coverage**: 70% (project-specific thresholds may vary)
- **Critical paths**: 90%+ coverage for core functionality
- **Edge cases**: Explicit tests for boundary conditions
- **Error paths**: Validate error handling and recovery

### Test Types

**Unit Tests:**
- Test individual functions/methods in isolation
- Mock external dependencies
- Fast execution (< 1 second per test suite)
- Use property-based testing where applicable

**Integration Tests:**
- Test component interactions
- Use real dependencies when feasible
- Validate API contracts and data flows
- Test configuration and initialization

**Edge Case Tests:**
- Null/empty inputs
- Boundary values (min, max, overflow)
- Concurrent access and race conditions
- Resource exhaustion scenarios
- Platform-specific behaviors

### Test Organization

**Test layout follows language/framework conventions. Each project should define specific practices.**

**Rust (common pattern):**
```rust
// Unit tests at end of implementation file
// src/module/component.rs
pub fn process_data(input: &[u8]) -> Result<Vec<u8>, Error> {
    // implementation
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_process_data_valid_input() {
        // test implementation
    }
}
```

```
# Integration tests in separate directory
tests/
├── integration_test.rs
└── common/
    └── mod.rs
```

**Python (depends on pytest vs unittest):**
```
# Common patterns - follow project conventions
project/
├── src/
│   └── mypackage/
│       └── module.py
└── tests/
    ├── unit/
    │   └── test_module.py
    └── integration/
        └── test_api_workflow.py
```

**General guidance:**
- Follow common patterns for your language and testing framework
- Consult project's `CONTRIBUTING.md` for specific conventions
- Keep test organization consistent within the project
- Co-locate unit tests or separate - project decides

### Running Tests

```bash
# Run all tests
make test

# Run with coverage
make coverage

# Language-specific examples
cargo test --workspace              # Rust
pytest tests/                       # Python with pytest
python -m unittest discover tests/  # Python with unittest
go test ./...                       # Go
```

---

## Documentation Expectations

### Code Documentation

**When to document:**
- Public APIs, functions, and classes (ALWAYS)
- Complex algorithms or non-obvious logic
- Performance considerations or optimization rationale
- Edge cases and error conditions
- Thread safety and concurrency requirements
- Hardware-specific code or platform dependencies

**Documentation style:**
```python
def preprocess_image(image: np.ndarray, target_size: tuple[int, int]) -> np.ndarray:
    """
    Resize and normalize image for model inference.

    Args:
        image: Input image as HWC numpy array (uint8)
        target_size: Target dimensions as (width, height)

    Returns:
        Preprocessed image as CHW float32 array normalized to [0, 1]

    Raises:
        ValueError: If image dimensions are invalid or target_size is negative

    Performance:
        Uses bilinear interpolation. For better quality with 2x cost,
        use bicubic interpolation via config.interpolation = 'bicubic'
    """
```

### Project Documentation

**Essential files for public repositories:**
- `README.md` - Project overview, quick start, documentation links
- `CONTRIBUTING.md` - Development setup, contribution process, coding standards
- `CODE_OF_CONDUCT.md` - Community standards (Contributor Covenant)
- `SECURITY.md` - Vulnerability reporting process
- `LICENSE` - Complete license text (Apache-2.0 for open source)

**Additional documentation:**
- User guides for features and workflows
- API reference documentation
- Migration guides for breaking changes

### Documentation Updates

When modifying code, update corresponding documentation:
- README if user-facing behavior changes
- API docs if function signatures or semantics change
- CHANGELOG for all user-visible changes
- Configuration guides if new options added

---

## License Policy

**CRITICAL**: Au-Zone has strict license policy for all dependencies.

### Allowed Licenses

✅ **Permissive licenses (APPROVED)**:
- MIT
- Apache-2.0
- BSD-2-Clause, BSD-3-Clause
- ISC
- 0BSD
- Unlicense

### Review Required

⚠️ **Weak copyleft (REQUIRES LEGAL REVIEW)**:
- MPL-2.0 (Mozilla Public License)
- LGPL-2.1-or-later, LGPL-3.0-or-later (if dynamically linked)

### Strictly Disallowed

❌ **NEVER USE THESE LICENSES**:
- GPL (any version)
- AGPL (any version)
- Creative Commons with NC (Non-Commercial) or ND (No Derivatives)
- SSPL (Server Side Public License)
- BSL (Business Source License, before conversion)
- OSL-3.0 (Open Software License)

### Verification Process

**Before adding dependencies:**
1. Check license compatibility with project license (typically Apache-2.0)
2. Verify no GPL/AGPL in dependency tree
3. Review project's SBOM (Software Bill of Materials) if available
4. Update NOTICE file after adding dependencies (see process below)

**CI/CD will automatically:**
- Generate SBOM using scancode-toolkit
- Validate CycloneDX SBOM schema
- Check for disallowed licenses
- Block PR merges if violations detected

**If you need a library with incompatible license:**
- Search for alternatives with permissive licenses
- Consider implementing functionality yourself
- Escalate to technical leadership for approval (rare exceptions)

### Managing Dependencies and NOTICE File

**IMPORTANT:** The NOTICE file is committed to the repository and must be updated when dependencies change.

**When adding or updating dependencies:**

1. **Update dependency files:**
   ```bash
   # Rust: Update Cargo.toml and rebuild
   cargo build
   git add Cargo.lock

   # Python: Update pyproject.toml or requirements.txt
   pip install -e .
   git add pyproject.toml
   ```

2. **Regenerate NOTICE file:**
   ```bash
   # Run SBOM generation script
   .github/scripts/generate_sbom.sh

   # Review changes to NOTICE
   git diff NOTICE

   # Stage NOTICE file
   git add NOTICE
   ```

3. **Verify license compliance:**
   ```bash
   # Check for license policy violations
   python3 .github/scripts/check_license_policy.py sbom.json
   ```

4. **Commit changes together:**
   ```bash
   git commit -m "PROJECTKEY-###: Add dependency-name

   - Added dependency-name version X.Y (License-Type)
   - Updated Cargo.lock and NOTICE file
   - Verified license policy compliance"
   ```

**The NOTICE file:**
- Is auto-generated from the SBOM (never manually edit)
- Lists third-party components requiring attribution
- References the official CycloneDX SBOM in release assets
- Must be updated whenever dependencies are added or changed

**Regular SBOM audits:**
- **Quarterly**: Maintainers regenerate SBOM and update NOTICE
- **Before releases**: Always regenerate SBOM/NOTICE for release commits
- **Security updates**: Update dependencies and regenerate NOTICE

**CI/CD automation:**
- Every commit triggers SBOM generation
- SBOM artifacts available in GitHub Actions
- Release workflow attaches sbom.json to release assets

---

## Security Practices

### Vulnerability Reporting

**For security issues**, use project's SECURITY.md process:
- Email: `support@au-zone.com` with subject "Security Vulnerability"
- Expected acknowledgment: 48 hours
- Expected assessment: 7 days
- Fix timeline based on severity

### Secure Coding Guidelines

**Input Validation:**
- Validate all external inputs (API requests, file uploads, user input)
- Use allowlists rather than blocklists
- Enforce size/length limits
- Sanitize for appropriate context (HTML, SQL, shell)

**Authentication & Authorization:**
- Never hardcode credentials or API keys
- Use environment variables or secure vaults for secrets
- Implement proper session management
- Follow principle of least privilege

**Data Protection:**
- Encrypt sensitive data at rest and in transit
- Use secure protocols (HTTPS, TLS 1.2+)
- Implement proper key management
- Sanitize logs (no passwords, tokens, PII)

**Common Vulnerabilities to Avoid:**
- SQL Injection: Use parameterized queries
- XSS (Cross-Site Scripting): Escape output, use CSP headers
- CSRF (Cross-Site Request Forgery): Use tokens
- Path Traversal: Validate and sanitize file paths
- Command Injection: Avoid shell execution; use safe APIs
- Buffer Overflows: Use safe string functions; bounds checking

### Dependencies

- Keep dependencies up to date
- Monitor for security advisories
- Use dependency scanning tools (Dependabot, Snyk)
- Audit new dependencies before adding

---

## Project-Specific Guidelines

### Technology Stack

This repository provides messaging schemas for EdgeFirst Perception Middleware:

- **Languages**:
  - Rust 1.70+ (primary implementation)
  - Python 3.8+ (bindings)
- **Build system**:
  - Cargo workspace for Rust
  - setuptools for Python (pyproject.toml)
  - Debian packaging (fakeroot/dpkg) for ROS2 integration
- **Key dependencies**:
  - serde 1.0+ and serde_derive (serialization)
  - Future: zenoh for messaging middleware
- **Serialization**: ROS2 CDR (Common Data Representation)
- **Target platforms**: Linux across diverse processors
  - x86_64, ARM64, ARMv7, ARMv8, RISC-V
  - Primary targets: NXP i.MX 8M Plus, Qualcomm platforms
  - Tested on: Maivin and Raivin edge AI platforms
- **Cross-platform**: Optimized for embedded Linux, but designed for portability

### Architecture

This is a **schema-first, multi-language binding** project:

- **Pattern**: Schema definition → Code generation → Multi-language bindings
- **Schema layers**:
  1. **ROS2 Common Interfaces** (std_msgs, sensor_msgs, geometry_msgs, nav_msgs) - Apache-2.0
  2. **Foxglove Schemas** (visualization types) - MIT
  3. **EdgeFirst Custom Schemas** (edgefirst_msgs) - Apache-2.0
- **Data flow**:
  - CDR IDL definitions → Rust/Python structures → Zenoh publish/subscribe
  - Zero-copy optimization for DMA buffers (embedded devices)
- **Communication**: Zenoh pub/sub middleware (ROS2-compatible but not ROS2-dependent)
- **ROS2 Integration**: Compatible with ROS2 Humble via Zenoh-ROS2 DDS bridge
- **Error handling**:
  - Rust: Result types where applicable
  - Schema validation at compile-time via type system

### Build and Deployment

**Rust Library:**
```bash
# Build release library
cargo build --release

# Run tests (when implemented)
cargo test --workspace

# Generate documentation
cargo doc --no-deps --open

# Check for linting issues
cargo clippy --all-targets --all-features

# Format code
cargo fmt --all

# Publish to crates.io (maintainers only)
cargo publish
```

**Python Package:**
```bash
# Install for development
pip install -e .

# Run tests (when implemented)
pytest tests/

# Type checking
mypy edgefirst/

# Format code
black edgefirst/

# Build distribution
python -m build

# Publish to PyPI (maintainers only)
twine upload dist/*
```

**Debian Package (ROS2 Integration):**
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Build package
cd edgefirst_msgs
fakeroot debian/rules build

# Install package
sudo dpkg -i ../ros-humble-edgefirst-msgs_*.deb
```

**Docker Build:**
```bash
# Build container with schemas
docker build -t edgefirst-schemas .
```

### Performance Targets

As a schema library, performance focuses on serialization efficiency:

- **Serialization overhead**: < 5% compared to raw memory operations
- **Memory footprint**: Minimal schema metadata overhead
- **Zero-copy support**: DMA buffer integration for camera/sensor data
- **Message latency**: < 1ms serialization/deserialization for typical messages
- **Throughput**: Support for high-frequency sensor data (>100Hz camera frames)
- **Compatibility**: Efficient CDR encoding matching ROS2 performance

### Hardware Specifics

This library is optimized for EdgeFirst hardware platforms:

- **Target Devices**:
  - Maivin: NXP i.MX 8M Plus based edge AI platform
  - Raivin: Qualcomm-based edge AI platform
  - Custom designs using supported SoCs
- **NPU Integration**: Schemas support zero-copy DMA buffer passing to:
  - NXP i.MX 8M Plus NPU (via VAAL library)
  - Qualcomm Hexagon DSP
- **Sensor Interfaces**: Message types for:
  - V4L2 cameras (DMA buffer file descriptors)
  - MIPI CSI-2 camera interfaces
  - IMU sensors (I2C/SPI)
  - GNSS/GPS (UART)
  - Radar sensors
  - LiDAR sensors
- **Platform Quirks**:
  - DMA buffer handling is platform-specific (document in integration code)
  - Colorspace conversions may differ between platforms

### Testing Conventions

**Rust (to be implemented):**
- **Unit tests**: Co-located in `#[cfg(test)] mod tests` at end of each module:
  ```rust
  // src/sensor_msgs.rs
  #[cfg(test)]
  mod tests {
      use super::*;

      #[test]
      fn test_image_serialization() {
          // Test implementation
      }
  }
  ```
- **Integration tests**: Separate `tests/` directory at project root
  ```
  tests/
  ├── integration_test.rs (cross-module schema tests)
  ├── serialization_test.rs (CDR round-trip tests)
  └── common/
      └── mod.rs (test utilities)
  ```
- **Test naming**: `test_<message_type>_<scenario>` format
- **Fixtures**: Common test messages in `tests/common/`
- **Coverage target**: 70% minimum
- **Benchmarks**: Use criterion.rs in `benches/` directory for serialization performance

**Python (to be implemented):**
- **Framework**: pytest
- **Unit tests**:
  ```
  tests/
  ├── unit/
  │   ├── test_std_msgs.py
  │   ├── test_sensor_msgs.py
  │   ├── test_geometry_msgs.py
  │   ├── test_nav_msgs.py
  │   ├── test_foxglove_msgs.py
  │   └── test_edgefirst_msgs.py
  └── integration/
      └── test_schema_compatibility.py
  ```
- **Test naming**: `test_<message_type>_<scenario>` format
- **Fixtures**: pytest fixtures in `conftest.py`
- **Coverage target**: 70% minimum
- **Type checking**: Use mypy for all test code

**Schema Validation Tests:**
- Test all message types for:
  - Correct field types
  - Default values
  - Nested message handling
  - Array/sequence handling
  - CDR serialization round-trips
  - Cross-language compatibility (Rust ↔ Python)

### Special Considerations for This Project

**Schema Compatibility:**
- Maintain ROS2 Humble compatibility for standard messages
- Follow ROS2 message definition conventions
- Test compatibility with ROS2 tools (rosbag, rviz via Zenoh bridge)

**Code Generation:**
- Schemas are the source of truth
- Generated Rust/Python code should not be manually edited
- Document code generation process in ARCHITECTURE.md

**Zenoh Integration:**
- Message types designed for Zenoh pub/sub patterns
- DMA buffer schemas use file descriptor passing (Linux-specific)
- Consider Zenoh query/reply patterns for service types

**EdgeFirst Studio Integration:**
- Schemas must be importable in Studio for validation
- Support Foxglove visualization types
- Document Studio-specific message conventions

**Multi-Language Consistency:**
- Rust and Python APIs should mirror each other
- Field names and types must match exactly
- Serialization must be byte-compatible between languages

---

## Working with AI Assistants

### For GitHub Copilot / Cursor

These tools provide inline suggestions. Ensure:
- Suggestions match project conventions (run linters after accepting)
- Complex logic has explanatory comments
- Generated tests have meaningful assertions
- Security best practices are followed

### For Claude Code / Chat-Based Assistants

When working with conversational AI:
1. **Provide context**: Share relevant files, error messages, and requirements
2. **Verify outputs**: Review generated code critically before committing
3. **Iterate**: Refine solutions through follow-up questions
4. **Document decisions**: Capture architectural choices and tradeoffs
5. **Test thoroughly**: AI-generated code needs human verification

### Common AI Assistant Pitfalls

- **Hallucinated APIs**: Verify library functions exist before using
- **Outdated patterns**: Check if suggestions match current best practices
- **Over-engineering**: Prefer simple solutions over complex ones
- **Missing edge cases**: Explicitly test boundary conditions
- **License violations**: AI may suggest code with incompatible licenses

---

## Workflow Example

**Implementing a new feature:**

```bash
# 1. Create branch from develop
git checkout develop
git pull origin develop
git checkout -b feature/EDGEAI-123-add-image-preprocessing

# 2. Implement feature with tests
# - Write unit tests first (TDD)
# - Implement functionality
# - Add integration tests
# - Update documentation

# 3. Verify quality
make format    # Auto-format code
make lint      # Run linters
make test      # Run all tests
make coverage  # Check coverage meets threshold

# 4. Commit with proper message
git add .
git commit -m "EDGEAI-123: Add image preprocessing pipeline

- Implemented resize, normalize, and augment functions
- Added comprehensive unit and integration tests
- Documented API with usage examples
- Achieved 85% test coverage"

# 5. Push and create PR
git push -u origin feature/EDGEAI-123-add-image-preprocessing
# Create PR via GitHub/Bitbucket UI with template

# 6. Address review feedback
# - Make requested changes
# - Push additional commits
# - Respond to comments

# 7. Merge after approvals
# Maintainer merges via PR interface (squash or rebase)
```

---

## Getting Help

**For development questions:**
- Check project's `CONTRIBUTING.md` for setup instructions
- Review existing code for patterns and conventions
- Search GitHub Issues for similar problems
- Ask in GitHub Discussions (for public repos)

**For security concerns:**
- Email `support@au-zone.com` with subject "Security Vulnerability"
- Do not disclose vulnerabilities publicly

**For license questions:**
- Review license policy section above
- Check project's `LICENSE` file
- Contact technical leadership if unclear

**For contribution guidelines:**
- Read project's `CONTRIBUTING.md`
- Review recent merged PRs for examples
- Follow PR template and checklist

---

## Document Maintenance

**Project maintainers should:**
- Update [Project-Specific Guidelines](#project-specific-guidelines) with repository details
- Add technology stack, architecture patterns, and performance targets
- Document build/test/deployment procedures specific to the project
- Specify testing conventions (unit test location, framework choice, etc.)
- Keep examples and code snippets current
- Review and update annually or when major changes occur

**This template version**: 1.0 (November 2025)
**Organization**: Au-Zone Technologies
**License**: Apache-2.0 (for open source projects)

---

*This document helps AI assistants contribute effectively to Au-Zone projects while maintaining quality, security, and consistency. For questions or suggestions, contact `support@au-zone.com`.*
