# TODO Phase C: C Coverage Instrumentation

**Phase:** C  
**Status:** Not Started  
**Estimate:** 8-10 hours  
**Dependencies:** Phase B (C API Expansion) - need complete FFI  
**Blocks:** None (final coverage setup)

---

## Objective

Configure multi-language coverage instrumentation so that C tests generate **both** C coverage (gcov) **and** Rust coverage (llvm-cov). This enables a unified coverage report showing test coverage of the Rust FFI code when exercised from C.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    C Test Suite                              │
│   ┌─────────────────────────────────────────────────────┐   │
│   │  C Tests (gcc --coverage)                           │   │
│   │  → generates gcda files (C coverage)                │   │
│   └─────────────────┬───────────────────────────────────┘   │
│                     │ FFI calls                             │
│   ┌─────────────────▼───────────────────────────────────┐   │
│   │  Rust Library (RUSTFLAGS="-C instrument-coverage")  │   │
│   │  → generates profraw files (Rust coverage)          │   │
│   └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘

Coverage Data Flow:
├── C tests → gcda files → gcovr → coverage_c.xml
└── Rust FFI → profraw files → llvm-cov → coverage_rust.lcov
```

When C tests call into the Rust FFI (`libedgefirst_schemas.so`), **both** coverage systems generate data simultaneously.

---

## Deliverables

### C.1 Rust Library Coverage Build

#### C.1.1 Add Profiling Profile to Cargo.toml

**File:** `Cargo.toml`

```toml
[profile.profiling]
inherits = "release"
strip = false     # Required for coverage
debug = true      # Required for coverage mapping
```

**Why not `--release`?** The release profile has `strip = true` by default, which removes debug symbols needed for LLVM coverage to map profraw data back to source code.

#### C.1.2 Build Script for Coverage

**File:** `scripts/build-coverage.sh`

```bash
#!/bin/bash
# Build Rust library with coverage instrumentation
set -e

echo "Building Rust library with coverage instrumentation..."

# Install cargo-llvm-cov if not present
if ! command -v cargo-llvm-cov &> /dev/null; then
    cargo install cargo-llvm-cov
fi

# Source the coverage environment
source <(cargo llvm-cov show-env --export-prefix)
export CARGO_TARGET_DIR=target/llvm-cov-target

# Build the library (cdylib for C linkage)
cargo build --lib --profile profiling

# Verify the instrumented library was built
if [ ! -f "target/llvm-cov-target/profiling/libedgefirst_schemas.so" ] && \
   [ ! -f "target/llvm-cov-target/profiling/libedgefirst_schemas.dylib" ]; then
    echo "ERROR: Instrumented library not found!"
    exit 1
fi

echo "Instrumented library built successfully"
ls -la target/llvm-cov-target/profiling/libedgefirst_schemas.*
```

**Acceptance:**
- [ ] Script builds instrumented `libedgefirst_schemas.so`
- [ ] Library is in `target/llvm-cov-target/profiling/`
- [ ] Debug symbols present (`strip = false`)

---

### C.2 C Test Coverage Configuration

#### C.2.1 Update Makefile for Coverage

**File:** `tests/c/Makefile`

Add coverage-specific targets:

```makefile
# EdgeFirst Schemas C Test Suite Makefile
# Uses Criterion unit testing framework with coverage support

CC = gcc
CRITERION_PREFIX = $(shell brew --prefix criterion 2>/dev/null || echo /usr/local)
CFLAGS = -Wall -Wextra -Werror -std=c11 -I../../include -I$(CRITERION_PREFIX)/include

# Coverage flags for C code
COVERAGE_CFLAGS = --coverage -fprofile-arcs -ftest-coverage -g3 -O0
COVERAGE_LDFLAGS = --coverage

# Standard flags (non-coverage)
LDFLAGS = -L../../target/release -ledgefirst_schemas -L$(CRITERION_PREFIX)/lib -lcriterion -Wl,-rpath,../../target/release

# Coverage flags (use instrumented Rust library)
LDFLAGS_COV = -L../../target/llvm-cov-target/profiling -ledgefirst_schemas \
              -L$(CRITERION_PREFIX)/lib -lcriterion \
              -Wl,-rpath,../../target/llvm-cov-target/profiling

# Test sources
TEST_SOURCES = test_builtin_interfaces.c \
               test_std_msgs.c \
               test_geometry_msgs.c \
               test_edgefirst_msgs.c \
               test_sensor_msgs.c \
               test_foxglove_msgs.c \
               test_errno.c

TEST_BINARIES = $(TEST_SOURCES:.c=)
COV_BINARIES = $(TEST_SOURCES:.c=_cov)

.PHONY: all clean test build-lib coverage coverage-report

# Standard build (no coverage)
all: build-lib $(TEST_BINARIES)

# Coverage build
coverage-build: build-lib-cov $(COV_BINARIES)

build-lib:
	@echo "Building Rust library (release)..."
	@cd ../.. && cargo build --release

build-lib-cov:
	@echo "Building Rust library with coverage instrumentation..."
	@cd ../.. && ./scripts/build-coverage.sh

# Standard test binaries
$(TEST_BINARIES): %: %.c
	@echo "Compiling $@..."
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

# Coverage test binaries
$(COV_BINARIES): %_cov: %.c
	@echo "Compiling $@ with coverage..."
	$(CC) $(CFLAGS) $(COVERAGE_CFLAGS) -o $@ $< $(LDFLAGS_COV) $(COVERAGE_LDFLAGS)

# Run standard tests
test: all
	@echo "Running C test suite..."
	@for test in $(TEST_BINARIES); do \
		echo "Running $$test"; \
		./$$test --verbose || exit 1; \
	done
	@echo "All C tests passed!"

# Run tests with coverage collection
coverage: coverage-build
	@mkdir -p coverage/profraw coverage/gcda
	@echo "Running C tests with coverage..."
	@export LLVM_PROFILE_FILE="$(shell pwd)/coverage/profraw/test-%p-%m.profraw"; \
	export GCOV_PREFIX="$(shell pwd)/coverage/gcda"; \
	export GCOV_PREFIX_STRIP=0; \
	for test in $(COV_BINARIES); do \
		echo "Running $$test with coverage..."; \
		./$$test --verbose || exit 1; \
	done
	@echo "Coverage data collected!"
	@echo "Profraw files: $$(find coverage/profraw -name '*.profraw' | wc -l)"
	@echo "GCDA files: $$(find coverage/gcda -name '*.gcda' 2>/dev/null | wc -l || echo 0)"

# Generate coverage reports
coverage-report: coverage
	@echo "Generating coverage reports..."
	@./generate-coverage-report.sh

clean:
	rm -f $(TEST_BINARIES) $(COV_BINARIES)
	rm -f *.o *.gcno *.gcda
	rm -rf coverage/

help:
	@echo "EdgeFirst Schemas C Test Suite"
	@echo ""
	@echo "Targets:"
	@echo "  all            - Build library and compile all tests"
	@echo "  test           - Run all test suites"
	@echo "  coverage       - Run tests with coverage collection"
	@echo "  coverage-report - Generate coverage reports (LCOV + HTML)"
	@echo "  clean          - Remove test binaries and coverage data"
```

**Acceptance:**
- [ ] `make coverage-build` compiles tests with `--coverage`
- [ ] Tests link against instrumented Rust library
- [ ] Coverage environment variables set correctly

---

### C.3 Coverage Report Generation

#### C.3.1 Coverage Report Script

**File:** `tests/c/generate-coverage-report.sh`

```bash
#!/bin/bash
# Generate unified coverage reports for C tests exercising Rust FFI
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/../.."
COVERAGE_DIR="$SCRIPT_DIR/coverage"

echo "=== EdgeFirst Schemas Coverage Report Generator ==="

# ============================================================================
# RUST COVERAGE (profraw → LCOV)
# ============================================================================

echo ""
echo "Processing Rust coverage..."

# Find LLVM tools from Rust toolchain
TOOLCHAIN_ROOT=$(rustc --print sysroot)
LLVM_PROFDATA=$(find "$TOOLCHAIN_ROOT" -name "llvm-profdata" -type f 2>/dev/null | head -1)
LLVM_COV=$(find "$TOOLCHAIN_ROOT" -name "llvm-cov" -type f 2>/dev/null | head -1)

if [ -z "$LLVM_PROFDATA" ] || [ -z "$LLVM_COV" ]; then
    echo "ERROR: LLVM tools not found in Rust toolchain"
    echo "Install with: rustup component add llvm-tools-preview"
    exit 1
fi

# Count profraw files
PROFRAW_COUNT=$(find "$COVERAGE_DIR/profraw" -name "*.profraw" 2>/dev/null | wc -l)
echo "Found $PROFRAW_COUNT profraw files"

if [ "$PROFRAW_COUNT" -eq 0 ]; then
    echo "WARNING: No Rust profraw files found. Rust FFI coverage will be empty."
else
    # Merge profraw files
    echo "Merging profraw files..."
    $LLVM_PROFDATA merge -sparse \
        $(find "$COVERAGE_DIR/profraw" -name "*.profraw") \
        -o "$COVERAGE_DIR/rust.profdata"

    # Find the instrumented library
    RUST_LIB=""
    for lib in "$PROJECT_ROOT/target/llvm-cov-target/profiling/libedgefirst_schemas.so" \
               "$PROJECT_ROOT/target/llvm-cov-target/profiling/libedgefirst_schemas.dylib"; do
        if [ -f "$lib" ]; then
            RUST_LIB="$lib"
            break
        fi
    done

    if [ -z "$RUST_LIB" ]; then
        echo "ERROR: Instrumented Rust library not found"
        exit 1
    fi

    # Generate LCOV report for Rust
    echo "Generating Rust LCOV report..."
    $LLVM_COV export \
        --format=lcov \
        --instr-profile="$COVERAGE_DIR/rust.profdata" \
        --ignore-filename-regex='/.cargo/registry|/rustc/' \
        "$RUST_LIB" \
        > "$COVERAGE_DIR/coverage_rust.lcov"

    echo "Rust coverage: $COVERAGE_DIR/coverage_rust.lcov"
fi

# ============================================================================
# C COVERAGE (gcda → SonarQube XML / LCOV)
# ============================================================================

echo ""
echo "Processing C coverage..."

# Check for gcovr
if ! command -v gcovr &> /dev/null; then
    echo "Installing gcovr..."
    pip3 install gcovr --quiet
fi

# Count gcda files
GCDA_COUNT=$(find "$SCRIPT_DIR" -name "*.gcda" 2>/dev/null | wc -l)
echo "Found $GCDA_COUNT gcda files"

if [ "$GCDA_COUNT" -eq 0 ]; then
    echo "WARNING: No C gcda files found. C coverage will be empty."
else
    # Generate SonarQube XML format
    echo "Generating C SonarQube XML report..."
    gcovr -r "$SCRIPT_DIR" \
        --sonarqube \
        -o "$COVERAGE_DIR/coverage_c_sonar.xml" \
        "$SCRIPT_DIR"

    # Generate LCOV format (for merging)
    echo "Generating C LCOV report..."
    gcovr -r "$SCRIPT_DIR" \
        --lcov \
        -o "$COVERAGE_DIR/coverage_c.lcov" \
        "$SCRIPT_DIR"

    # Generate HTML for debugging
    echo "Generating C HTML report..."
    gcovr -r "$SCRIPT_DIR" \
        --html --html-details \
        -o "$COVERAGE_DIR/coverage_c.html" \
        "$SCRIPT_DIR"

    echo "C coverage: $COVERAGE_DIR/coverage_c.lcov"
    echo "C HTML: $COVERAGE_DIR/coverage_c.html"
fi

# ============================================================================
# MERGE COVERAGE (Optional - combined LCOV)
# ============================================================================

echo ""
echo "Merging coverage reports..."

# Check for lcov
if command -v lcov &> /dev/null; then
    LCOV_FILES=""
    if [ -f "$COVERAGE_DIR/coverage_rust.lcov" ]; then
        LCOV_FILES="$LCOV_FILES -a $COVERAGE_DIR/coverage_rust.lcov"
    fi
    if [ -f "$COVERAGE_DIR/coverage_c.lcov" ]; then
        LCOV_FILES="$LCOV_FILES -a $COVERAGE_DIR/coverage_c.lcov"
    fi

    if [ -n "$LCOV_FILES" ]; then
        lcov $LCOV_FILES -o "$COVERAGE_DIR/coverage_combined.lcov"
        echo "Combined coverage: $COVERAGE_DIR/coverage_combined.lcov"

        # Generate combined HTML
        if command -v genhtml &> /dev/null; then
            genhtml "$COVERAGE_DIR/coverage_combined.lcov" \
                --output-directory "$COVERAGE_DIR/html" \
                --title "EdgeFirst Schemas Coverage"
            echo "Combined HTML: $COVERAGE_DIR/html/index.html"
        fi
    fi
else
    echo "lcov not installed - skipping combined report"
    echo "Install with: brew install lcov (macOS) or apt install lcov (Linux)"
fi

# ============================================================================
# SUMMARY
# ============================================================================

echo ""
echo "=== Coverage Summary ==="
echo "Rust profraw files: $PROFRAW_COUNT"
echo "C gcda files: $GCDA_COUNT"
echo ""
echo "Generated reports:"
ls -la "$COVERAGE_DIR"/*.lcov "$COVERAGE_DIR"/*.xml 2>/dev/null || echo "(none)"
echo ""
echo "To view HTML report:"
echo "  open $COVERAGE_DIR/html/index.html"
```

Make executable:
```bash
chmod +x tests/c/generate-coverage-report.sh
```

**Acceptance:**
- [ ] Script processes both profraw and gcda files
- [ ] Generates `coverage_rust.lcov` from profraw
- [ ] Generates `coverage_c.lcov` and `coverage_c_sonar.xml` from gcda
- [ ] Optionally merges into combined report

---

### C.4 SonarCloud Integration

#### C.4.1 Update sonar-project.properties

**File:** `sonar-project.properties`

```properties
sonar.projectKey=EdgeFirstAI_schemas
sonar.organization=edgefirst

# Source directories
sonar.sources=src,edgefirst
sonar.tests=tests

# Rust coverage (LCOV format)
sonar.rust.lcov.reportPaths=tests/c/coverage/coverage_rust.lcov

# C coverage (SonarQube XML format) 
sonar.coverageReportPaths=tests/c/coverage/coverage_c_sonar.xml

# Exclusions
sonar.exclusions=**/target/**,**/.venv/**,**/node_modules/**

# C/C++ specific
sonar.cfamily.compile-commands=compile_commands.json
```

**Acceptance:**
- [ ] SonarCloud accepts coverage reports
- [ ] Rust and C coverage shown separately
- [ ] No path mapping issues

---

### C.5 GitHub Actions Workflow

#### C.5.1 Coverage Workflow

**File:** `.github/workflows/coverage.yml`

```yaml
name: Coverage

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main, develop]

jobs:
  coverage:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Install Rust toolchain
        uses: dtolnay/rust-action@stable
        with:
          components: llvm-tools-preview

      - name: Install cargo-llvm-cov
        run: cargo install cargo-llvm-cov

      - name: Install C dependencies
        run: |
          sudo apt-get update
          sudo apt-get install -y libcriterion-dev lcov
          pip3 install gcovr

      - name: Build Rust library with coverage
        run: |
          chmod +x scripts/build-coverage.sh
          ./scripts/build-coverage.sh

      - name: Build and run C tests with coverage
        working-directory: tests/c
        run: |
          make coverage
          make coverage-report

      - name: Run Rust tests with coverage (for comparison)
        run: |
          cargo llvm-cov --workspace --lcov --output-path tests/c/coverage/coverage_rust_unit.lcov

      - name: Upload coverage reports
        uses: actions/upload-artifact@v4
        with:
          name: coverage-reports
          path: |
            tests/c/coverage/*.lcov
            tests/c/coverage/*.xml
            tests/c/coverage/html/

      - name: SonarCloud Scan
        uses: SonarSource/sonarqube-scan-action@v3
        env:
          SONAR_TOKEN: ${{ secrets.SONAR_TOKEN }}
        with:
          args: >
            -Dsonar.rust.lcov.reportPaths=tests/c/coverage/coverage_rust.lcov
            -Dsonar.coverageReportPaths=tests/c/coverage/coverage_c_sonar.xml
```

**Acceptance:**
- [ ] Workflow runs on push/PR
- [ ] Coverage artifacts uploaded
- [ ] SonarCloud receives reports

---

### C.6 Local Development Commands

#### C.6.1 Quick Reference

Add to project README or TESTING.md:

```bash
# === Coverage Commands ===

# Build with coverage and run C tests
cd tests/c
make coverage

# Generate reports (after running tests)
make coverage-report

# View HTML report
open coverage/html/index.html

# Run Rust unit tests with coverage (separate)
cargo llvm-cov --workspace --html

# Clean coverage data
make clean
```

---

## Coverage Data Flow Reference

### Files Generated

| Phase | Files | Location |
|-------|-------|----------|
| Rust build | `libedgefirst_schemas.so` | `target/llvm-cov-target/profiling/` |
| C build | `*.gcno` | `tests/c/` (alongside .o files) |
| Test run | `*.profraw` | `tests/c/coverage/profraw/` |
| Test run | `*.gcda` | `tests/c/` (alongside .gcno) |
| Report | `coverage_rust.lcov` | `tests/c/coverage/` |
| Report | `coverage_c.lcov` | `tests/c/coverage/` |
| Report | `coverage_c_sonar.xml` | `tests/c/coverage/` |

### Environment Variables

| Variable | Purpose | Example |
|----------|---------|---------|
| `LLVM_PROFILE_FILE` | Where Rust writes profraw | `coverage/profraw/test-%p-%m.profraw` |
| `GCOV_PREFIX` | Prefix for gcda output path | `/workspace/coverage/gcda` |
| `GCOV_PREFIX_STRIP` | Path components to strip | `0` (depends on build path) |
| `LD_LIBRARY_PATH` | Find instrumented library | `target/llvm-cov-target/profiling` |

---

## Validation Checklist

### Build Verification

- [ ] `./scripts/build-coverage.sh` completes successfully
- [ ] Instrumented library exists: `target/llvm-cov-target/profiling/libedgefirst_schemas.so`
- [ ] Library has debug symbols: `file libedgefirst_schemas.so` shows "not stripped"

### Test Verification

- [ ] `make coverage` runs all tests successfully
- [ ] Profraw files generated: `find coverage/profraw -name "*.profraw" | wc -l` > 0
- [ ] GCDA files generated: `find . -name "*.gcda" | wc -l` > 0

### Report Verification

- [ ] `coverage_rust.lcov` contains source file entries
- [ ] `coverage_c_sonar.xml` is valid XML with coverage data
- [ ] Combined HTML report shows both languages

### SonarCloud Verification

- [ ] Coverage appears in SonarCloud dashboard
- [ ] No "coverage report not found" warnings
- [ ] Coverage percentages are reasonable (>0%)

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| No profraw files | LLVM_PROFILE_FILE not set | Verify export before running tests |
| Empty profraw files | Test crashed | Check test output for errors |
| 0% Rust coverage | Wrong library linked | Verify using instrumented library in profiling/ |
| 0% C coverage | gcda not paired with gcno | Check GCOV_PREFIX_STRIP value |
| "malformed profile" | Architecture mismatch | Build and process on same architecture |
| SonarCloud missing coverage | Wrong paths in config | Verify paths in sonar-project.properties |

---

## Success Criteria

- [ ] `Cargo.toml` has `[profile.profiling]` with correct settings
- [ ] `scripts/build-coverage.sh` builds instrumented library
- [ ] `tests/c/Makefile` has `coverage` and `coverage-report` targets
- [ ] `tests/c/generate-coverage-report.sh` generates all reports
- [ ] Profraw files generated when C tests run
- [ ] GCDA files generated when C tests run  
- [ ] Rust LCOV report shows FFI function coverage
- [ ] C SonarQube XML report shows test code coverage
- [ ] SonarCloud receives and displays coverage
- [ ] CI workflow runs coverage on every PR

---

**Next Phase:** [TODO_D.md](./TODO_D.md) - Python Test Suite (can proceed in parallel)
