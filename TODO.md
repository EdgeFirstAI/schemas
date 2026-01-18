# TODO - EdgeFirst Schemas Comprehensive Development Plan

**Created:** January 16, 2026
**Last Updated:** January 16, 2026
**Status:** Planning Complete
**Version:** 3.0

---

## Executive Summary

This document outlines the complete development plan to bring EdgeFirst Schemas to production readiness with:

1. **Full CDR encoding/decoding** for all message types (Rust + Python + TypeScript)
2. **Complete C API** with FFI bindings for all interfaces
3. **Comprehensive test coverage** across all four languages (Rust, C, Python, TypeScript)
4. **Multi-language coverage instrumentation** (C tests driving Rust coverage)
5. **Performance benchmarks** for heavy message types used in production
6. **Cross-browser JavaScript support** with comprehensive testing and benchmarks

**Total Estimated Effort:** 80-110 hours across 7 phases

---

## Phase Overview

| Phase | Name | Focus Area | Estimate | Details |
|-------|------|------------|----------|---------|
| **A** | Rust Benchmarks | Performance testing | 6-8 hrs | [TODO_A.md](./TODO_A.md) ✅ |
| **B** | C API Expansion | Missing FFI bindings | 16-24 hrs | [TODO_B.md](./TODO_B.md) |
| **C** | C Coverage Instrumentation | Multi-language coverage | 10-14 hrs | [TODO_C.md](./TODO_C.md) |
| **D** | Python Test Suite | Unit tests + coverage | 10-12 hrs | [TODO_D.md](./TODO_D.md) |
| **E** | Python Benchmarks | Performance testing | 4-6 hrs | [TODO_E.md](./TODO_E.md) |
| **F** | Cross-Language Validation | Compatibility tests | 12-16 hrs | [TODO_F.md](./TODO_F.md) |
| **G** | TypeScript/JavaScript | Browser + Node.js support | 20-30 hrs | [TODO_G.md](./TODO_G.md) |

**Phase B Sub-phases:**
- **B.1** Priority 1 types (FoxgloveCompressedVideo, RadarCube) - unblocks Phase C
- **B.2** Priority 2-4 types (geometry_msgs, sensor_msgs expansion) - completes before Phase F

**Dependency Graph:**
```
Phase A (Rust Benchmarks) ────────────────────────────────┐
                                                          │
Phase B.1 (Priority 1 C API) ──┬──► Phase C (Coverage) ───┼──► Phase F (Cross-Language)
                               │                          │
Phase D (Python Tests) ────────┼──► Early CDR Check* ─────┤
       │                       │                          │
       └──► Phase E (Py Bench) ┘                          │
                                                          │
Phase B.2 (Priority 2-4 C API) ──────────────────────────►┤
                                                          │
Phase G (TypeScript/JS) ──────────────────────────────────┘

* Early CDR Check: Validate 2-3 message types cross-language before Phase F
```

---

## Current State Assessment

### Rust Library (`src/`)

| Component | Status | Notes |
|-----------|--------|-------|
| CDR serialize/deserialize | ✅ Complete | `serde_cdr.rs` using `cdr` crate |
| builtin_interfaces | ✅ Complete | Time, Duration |
| std_msgs | ✅ Complete | Header, ColorRGBA |
| geometry_msgs | ✅ Complete | 15 types (Vector3, Point, Pose, etc.) |
| sensor_msgs | ✅ Complete | PointCloud2, Image, NavSatFix, IMU, etc. |
| foxglove_msgs | ✅ Complete | CompressedVideo, ImageAnnotations |
| edgefirst_msgs | ✅ Complete | RadarCube, DmaBuf, Detect, Mask, etc. |
| Inline unit tests | ✅ 44 tests | All serialize/deserialize roundtrips |
| Benchmarks | ✅ Complete | `benches/serialization.rs` implemented |

### C API (`src/ffi.rs` + `include/`)

| Component | Status | Notes |
|-----------|--------|-------|
| FFI bindings | ⚠️ Partial | 216 functions, ~60% coverage |
| CompressedVideo | ❌ Missing | Critical for samples |
| RadarCube | ❌ Missing | Critical for samples |
| geometry_msgs expansion | ❌ Missing | Only 4/15 types exposed |
| IMU, CameraInfo | ❌ Missing | Common robotics types |
| C header | ⚠️ Needs sync | Match FFI after expansion |
| C tests | ✅ 97 tests | 6 test files with Criterion |
| Coverage instrumentation | ❌ Missing | No gcov/llvm-cov setup |

### Python Package (`edgefirst/schemas/`)

| Component | Status | Notes |
|-----------|--------|-------|
| Schema definitions | ✅ Complete | 7 modules using pycdr2 |
| CDR via pycdr2 | ✅ Implicit | IdlStruct inheritance |
| Unit tests | ❌ Missing | No test files exist |
| pytest-cov | ❌ Not configured | Not in pyproject.toml |
| pytest-benchmark | ❌ Not configured | Not in pyproject.toml |
| Benchmarks | ❌ Missing | Empty directory |

### TypeScript/JavaScript (`typescript/`)

| Component | Status | Notes |
|-----------|--------|-------|
| Directory structure | ❌ Not created | New Phase G deliverable |
| CDR serialization | ❌ Missing | Clean implementation needed |
| Message types | ❌ Missing | All types need implementation |
| TypeScript definitions | ❌ Missing | Strong typing required |
| Unit tests | ❌ Missing | Browser + Node.js required |
| Benchmarks | ❌ Missing | Cross-browser testing required |
| npm package | ❌ Missing | @edgefirst/schemas |

---

## Architectural Guidelines

### C FFI Serialization Pattern (Khronos-style)

All `_serialize()` functions MUST use the Khronos buffer pattern (common in OpenCL, OpenGL, Vulkan):

```c
/**
 * Serializes a message to CDR format.
 *
 * @param obj       The object to serialize (must not be NULL)
 * @param buffer    Output buffer for CDR bytes (may be NULL to query size)
 * @param capacity  Size of buffer in bytes (ignored if buffer is NULL)
 * @param size      If non-NULL, receives the number of bytes written/required
 *
 * @return 0 on success, -1 on error with errno set:
 *         - EINVAL: obj is NULL
 *         - ENOBUFS: buffer too small (size still written if provided)
 *         - EBADMSG: serialization failed
 *
 * Usage patterns:
 *   // Query required size
 *   size_t required;
 *   edgefirst_type_serialize(obj, NULL, 0, &required);
 *
 *   // Allocate and serialize
 *   uint8_t* buffer = malloc(required);
 *   edgefirst_type_serialize(obj, buffer, required, NULL);
 *
 *   // Reusable buffer pattern
 *   if (edgefirst_type_serialize(obj, buf, buf_cap, &size) == -1 && errno == ENOBUFS) {
 *       buf = realloc(buf, size);
 *       buf_cap = size;
 *       edgefirst_type_serialize(obj, buf, buf_cap, NULL);
 *   }
 */
int edgefirst_type_serialize(
    const EdgeFirstType* obj,
    uint8_t* buffer,
    size_t capacity,
    size_t* size
);
```

**Benefits:**
- Caller owns all memory - no cross-allocator issues
- Query-then-allocate for exact sizing
- Buffer reuse for hot paths (zero allocations)
- No `edgefirst_free_bytes()` function needed

**Type lifecycle functions** (`_new`/`_free`) remain unchanged - they correctly manage Rust-allocated types.

### Error Handling Policy

**Rust code:**
- NEVER use `assert!()` outside of unit tests
- ALWAYS return `Result<T, E>` from internal functions
- FFI boundary functions return `0` (success) or `-1` (error) with `errno` set

**C FFI errno codes** (must be documented in `schemas.h` doxygen):

| errno | Meaning |
|-------|---------|
| `EINVAL` | NULL pointer or invalid argument |
| `ENOMEM` | Memory allocation failed |
| `ENOBUFS` | Buffer too small (size still written) |
| `EBADMSG` | CDR serialization/deserialization failed |
| `EILSEQ` | Invalid UTF-8 in string field |

### CDR Cross-Language Compatibility

CDR is a well-defined standard (OMG XCDR). Compatibility between implementations is expected, but MUST be validated:

- **Phase F requirement:** Encode with cdr-rs → decode with pycdr2 (and vice versa)
- **Phase G requirement:** Encode with TypeScript → decode with Rust/Python (and vice versa)
- These tests also serve as baselines for future CDR optimization work

---

## High-Level Requirements Mapping

### Requirement 1: Rust CDR Encoding/Decoding
- **Status:** ✅ COMPLETE
- **Evidence:** All structs derive `Serialize, Deserialize`; `serde_cdr.rs` provides `serialize()`/`deserialize()`
- **Remaining:** None (covered by existing implementation)

### Requirement 2: Rust Tests + Benchmarks
- **Status:** ✅ COMPLETE
- **Covered by:** [TODO_A.md](./TODO_A.md) - Rust Benchmarks
- **Key deliverables:**
  - [x] Criterion benchmark setup in `Cargo.toml`
  - [x] `benches/serialization.rs`
  - [x] CompressedVideo, RadarCube, PointCloud2 benchmarks

### Requirement 3: Complete C API
- **Status:** ⚠️ PARTIAL (~60% coverage)
- **Covered by:** [TODO_B.md](./TODO_B.md) - C API Expansion
- **Key deliverables:**
  - [ ] FFI for FoxgloveCompressedVideo (~15 functions)
  - [ ] FFI for RadarCube (~20 functions)
  - [ ] FFI for remaining geometry_msgs (~60 functions)
  - [ ] FFI for IMU, CameraInfo, CompressedImage (~40 functions)
  - [ ] Updated C header and tests

### Requirement 4: C Tests with Rust Coverage
- **Status:** ❌ NOT STARTED
- **Covered by:** [TODO_C.md](./TODO_C.md) - C Coverage Instrumentation
- **Key deliverables:**
  - [ ] Rust library build with coverage instrumentation
  - [ ] C tests compiled with gcov flags
  - [ ] Unified coverage report (lcov/grcov)
  - [ ] CI/CD integration

### Requirement 5: Python Tests + Coverage
- **Status:** ❌ NOT STARTED
- **Covered by:** [TODO_D.md](./TODO_D.md) - Python Test Suite
- **Key deliverables:**
  - [ ] pytest-cov in pyproject.toml
  - [ ] `tests/python/test_*.py` for all modules
  - [ ] 70%+ coverage target
  - [ ] Cross-language validation tests

### Requirement 6: Python Benchmarks
- **Status:** ❌ NOT STARTED
- **Covered by:** [TODO_E.md](./TODO_E.md) - Python Benchmarks
- **Key deliverables:**
  - [ ] pytest-benchmark in pyproject.toml
  - [ ] `tests/python/benchmarks/bench_serialization.py`
  - [ ] CompressedVideo, RadarCube, PointCloud2 benchmarks
  - [ ] Comparison with Rust benchmarks

### Requirement 7: TypeScript/JavaScript Support
- **Status:** ❌ NOT STARTED
- **Covered by:** [TODO_G.md](./TODO_G.md) - TypeScript/JavaScript
- **Key deliverables:**
  - [ ] `typescript/` directory with clean modern TypeScript implementation
  - [ ] CDR serialization/deserialization (CdrReader, CdrWriter)
  - [ ] All message types with strong TypeScript typing
  - [ ] Unit tests running in browsers (Chrome, Firefox, Safari) and Node.js
  - [ ] Performance benchmarks across browser engines
  - [ ] npm package `@edgefirst/schemas`
  - [ ] Cross-language CDR validation (TypeScript ↔ Rust ↔ Python)

---

## Message Types Inventory

### Priority 1: Heavy Messages (Benchmark Required)

These are used extensively in `~/Software/EdgeFirst/samples`:

| Message | Rust | C FFI | Python | TypeScript | Benchmark |
|---------|------|-------|--------|------------|-----------|
| `FoxgloveCompressedVideo` | ✅ | ❌ | ✅ | ❌ | ❌ |
| `RadarCube` | ✅ | ❌ | ✅ | ❌ | ❌ |
| `PointCloud2` | ✅ | ✅ | ✅ | ❌ | ❌ |
| `Image` | ✅ | ✅ | ✅ | ❌ | ❌ |
| `Mask` | ✅ | ✅ | ✅ | ❌ | ❌ |

### Priority 2: Common Robotics Types

| Message | Rust | C FFI | Python | TypeScript |
|---------|------|-------|--------|------------|
| `IMU` | ✅ | ❌ | ✅ | ❌ |
| `CameraInfo` | ✅ | ❌ | ✅ | ❌ |
| `CompressedImage` | ✅ | ❌ | ✅ | ❌ |
| `Pose` | ✅ | ❌ | ✅ | ❌ |
| `PoseStamped` | ✅ | ❌ | ✅ | ❌ |
| `Transform` | ✅ | ❌ | ✅ | ❌ |
| `TransformStamped` | ✅ | ❌ | ✅ | ❌ |
| `Twist` | ✅ | ❌ | ✅ | ❌ |
| `TwistStamped` | ✅ | ❌ | ✅ | ❌ |
| `Wrench` | ✅ | ❌ | ✅ | ❌ |
| `WrenchStamped` | ✅ | ❌ | ✅ | ❌ |

### Priority 3: EdgeFirst Custom Types

| Message | Rust | C FFI | Python | TypeScript |
|---------|------|-------|--------|------------|
| `DmaBuf` | ✅ | ✅ | ✅ | ❌ |
| `Detect` | ✅ | ✅ | ✅ | ❌ |
| `DetectBox2D` | ✅ | ✅ | ✅ | ❌ |
| `DetectTrack` | ✅ | ✅ | ✅ | ❌ |
| `Model` | ✅ | ❌ | ✅ | ❌ |
| `ModelInfo` | ✅ | ✅ | ✅ | ❌ |
| `RadarInfo` | ✅ | ❌ | ✅ | ❌ |
| `LocalTime` | ✅ | ❌ | ✅ | ❌ |

### Priority 4: Foxglove Visualization Types

| Message | Rust | C FFI | Python | TypeScript |
|---------|------|-------|--------|------------|
| `FoxgloveImageAnnotations` | ✅ | ❌ | ✅ | ❌ |
| `FoxgloveCircleAnnotations` | ✅ | ❌ | ✅ | ❌ |
| `FoxglovePointAnnotations` | ✅ | ❌ | ✅ | ❌ |
| `FoxgloveTextAnnotations` | ✅ | ❌ | ✅ | ❌ |
| `FoxglovePoint2` | ✅ | ❌ | ✅ | ❌ |
| `FoxgloveColor` | ✅ | ❌ | ✅ | ❌ |

---

## Success Criteria

### Phase Completion Gates

| Phase | Gate Criteria |
|-------|---------------|
| A | All benchmarks run; baseline metrics captured |
| B.1 | Priority 1 FFI functions implemented; C tests pass |
| B.2 | All remaining FFI functions implemented; C tests pass |
| C | Coverage reports generated for both C and Rust |
| D | Python tests pass with ≥70% coverage |
| E | Python benchmarks run; comparison with Rust available |
| F | Cross-language CDR compatibility verified (Rust ↔ Python ↔ C) |
| G | TypeScript tests pass in all browsers + Node.js; benchmarks complete |

### Final Acceptance Criteria

- [ ] `cargo test` passes (all 44+ Rust tests)
- [ ] `cargo bench` completes with metrics for heavy messages
- [ ] `cd tests/c && make test` passes (all 150+ C tests)
- [ ] `cd tests/c && make coverage` generates combined report
- [ ] `pytest tests/python/` passes with ≥70% coverage
- [ ] `pytest tests/python/benchmarks/` completes with metrics
- [ ] `cd typescript && npm test` passes in Node.js
- [ ] `cd typescript && npm run test:browser` passes in Chrome, Firefox, Safari
- [ ] `cd typescript && npm run bench` completes with metrics
- [ ] Cross-language validation passes (Rust ↔ Python ↔ TypeScript CDR)
- [ ] Documentation updated (README, TESTING, ARCHITECTURE)

---

## Quick Start Commands

```bash
# Rust: Build and test
cargo build --release
cargo test --workspace
cargo bench  # After Phase A

# C: Build and test
cd tests/c
make test
make coverage  # After Phase C

# Python: Install and test
pip install -e ".[test]"  # After Phase D
pytest tests/python/ --cov=edgefirst --cov-report=html
pytest tests/python/benchmarks/  # After Phase E

# TypeScript: Build and test (After Phase G)
cd typescript
npm install
npm run build
npm test                # Node.js tests
npm run test:browser    # Browser tests (Chrome, Firefox, Safari)
npm run bench           # Node.js benchmarks
npm run bench:browser   # Browser benchmarks
```

---

## Related Documentation

- [TODO_A.md](./TODO_A.md) - Phase A: Rust Benchmarks
- [TODO_B.md](./TODO_B.md) - Phase B: C API Expansion (B.1 + B.2)
- [TODO_C.md](./TODO_C.md) - Phase C: C Coverage Instrumentation
- [TODO_D.md](./TODO_D.md) - Phase D: Python Test Suite
- [TODO_E.md](./TODO_E.md) - Phase E: Python Benchmarks
- [TODO_F.md](./TODO_F.md) - Phase F: Cross-Language Validation
- [TODO_G.md](./TODO_G.md) - Phase G: TypeScript/JavaScript Support
- [C_API_NAMING.md](./C_API_NAMING.md) - C API naming conventions
- [C_API_GUIDE.md](./C_API_GUIDE.md) - C API usage guide
- [TESTING.md](./TESTING.md) - Testing documentation
- [ARCHITECTURE.md](./ARCHITECTURE.md) - Architecture overview

---

**Next Action:** Begin with Phase A (Rust Benchmarks) or Phase B.1 (Priority 1 C API) - these can proceed in parallel.
