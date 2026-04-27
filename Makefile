# EdgeFirst Schemas Makefile
#
# Targets:
#   all       - Build library and C tests
#   lib       - Build Rust library (release by default)
#   test-c    - Build and run C tests
#   clean     - Remove build artifacts
#
# Variables:
#   RELEASE=1 - Build release library (default)
#   RELEASE=0 - Build debug library (for coverage testing)

# Rust library output - configurable between debug and release
RELEASE ?= 1
ifeq ($(RELEASE),1)
  LIB_DIR = target/release
  CARGO_FLAGS = --release
else
  LIB_DIR = target/debug
  CARGO_FLAGS =
endif
LIB_NAME = libedgefirst_schemas

# C test configuration
CC = gcc
CRITERION_PREFIX = $(shell brew --prefix criterion 2>/dev/null || echo /usr/local)
CFLAGS = -Wall -Wextra -Werror -std=c11 -I./include -I$(CRITERION_PREFIX)/include
LDFLAGS = -L$(LIB_DIR) -ledgefirst_schemas -L$(CRITERION_PREFIX)/lib -lcriterion -lm -Wl,-rpath,$(LIB_DIR)

# C++ test configuration
CXX ?= g++
CXXSTD ?= c++17
CXXFLAGS_BASE = -std=$(CXXSTD) -Wall -Wextra -Werror -I./include -Itests/cpp
CXXFLAGS = $(CXXFLAGS_BASE) -O2
CXXFLAGS_ASAN = $(CXXFLAGS_BASE) -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -Wno-maybe-uninitialized
CXXLDFLAGS = -L$(LIB_DIR) -ledgefirst_schemas -Wl,-rpath,$(LIB_DIR)
CXXLDFLAGS_ASAN = $(CXXLDFLAGS) -fsanitize=address,undefined

# C++ test sources and binaries
CPP_TEST_DIR = tests/cpp
CPP_TEST_SOURCES = $(wildcard $(CPP_TEST_DIR)/test_*.cpp)
CPP_TEST_BINARIES = $(patsubst $(CPP_TEST_DIR)/%.cpp,$(BUILD_DIR)/%,$(CPP_TEST_SOURCES))
CPP_TEST_BINARIES_ASAN = $(patsubst $(CPP_TEST_DIR)/%.cpp,$(BUILD_DIR)/%_asan,$(CPP_TEST_SOURCES))

# Install prefix and destdir (standard GNU conventions)
DESTDIR ?=
PREFIX  ?= /usr/local
INCDIR  = $(DESTDIR)$(PREFIX)/include
LIBDIR  = $(DESTDIR)$(PREFIX)/lib

# Build output directory
BUILD_DIR = build

# C test sources
TEST_DIR = tests/c
TEST_SOURCES = $(wildcard $(TEST_DIR)/test_*.c)
TEST_BINARIES = $(patsubst $(TEST_DIR)/%.c,$(BUILD_DIR)/%,$(TEST_SOURCES))

.PHONY: all lib test-c test-c-xml test-cpp test-cpp-asan test-cpp-xml test-cpp-asan-xml example-cpp install docs docs-clean clean help \
        test-python test-python-coverage

all: lib $(TEST_BINARIES)

# Parse semver components from Cargo.toml for the SOVERSION chain.
# Using := (immediate expansion) so the shell runs once per make invocation.
VERSION_FULL  := $(shell grep '^version = ' Cargo.toml | head -1 | sed 's/version = "\(.*\)"/\1/')
VERSION_MAJOR := $(word 1,$(subst ., ,$(VERSION_FULL)))
VERSION_MINOR := $(word 2,$(subst ., ,$(VERSION_FULL)))

# Build the Rust library and arrange the GNU/Linux SOVERSION symlink chain:
#
#   libedgefirst_schemas.so                      symlink -> .so.MAJOR
#   libedgefirst_schemas.so.MAJOR                symlink -> .so.MAJOR.MINOR
#   libedgefirst_schemas.so.MAJOR.MINOR          symlink -> .so.MAJOR.MINOR.PATCH
#   libedgefirst_schemas.so.MAJOR.MINOR.PATCH    real file (renamed from cargo output)
#
# build.rs embeds DT_SONAME = libedgefirst_schemas.so.MAJOR; that is the name
# the runtime loader actually opens, and it resolves through the chain above
# to the real file. Rationale: rustc writes the cdylib to `libedgefirst_schemas.so`; on first build
# we rename that file to the fully-qualified name and create the chain of symlinks
# up from it. Incremental rebuilds write through the `.so` symlink (open() follows
# symlinks on O_TRUNC|O_WRONLY), so the real file at .so.MAJOR.MINOR.PATCH is
# updated in place and the chain stays intact. On a version bump, stale versioned
# files/symlinks from prior versions are removed before the chain is rebuilt.
lib:
	@echo "Building Rust library..."
	@cargo build $(CARGO_FLAGS)
	@set -e; \
	LIB_DIR='$(LIB_DIR)'; LIB='$(LIB_NAME)'; \
	VERSION='$(VERSION_FULL)'; MAJOR='$(VERSION_MAJOR)'; MINOR='$(VERSION_MINOR)'; \
	REAL="$$LIB_DIR/$$LIB.so.$$VERSION"; \
	if [ -f "$$LIB_DIR/$$LIB.so" ] && [ ! -L "$$LIB_DIR/$$LIB.so" ]; then \
	    find "$$LIB_DIR" -maxdepth 1 \( -type l -o -type f \) -name "$$LIB.so.*" \
	        ! -name "$$LIB.so.$$VERSION" -exec rm -f {} +; \
	    mv -f "$$LIB_DIR/$$LIB.so" "$$REAL"; \
	elif [ ! -e "$$REAL" ]; then \
	    echo "error: cargo output missing and no prior $$LIB.so.$$VERSION found" >&2; \
	    echo "hint: run 'make clean' then retry (version bump without clean build)" >&2; \
	    exit 1; \
	fi; \
	rm -f "$$LIB_DIR/$$LIB.so" \
	      "$$LIB_DIR/$$LIB.so.$$MAJOR" \
	      "$$LIB_DIR/$$LIB.so.$$MAJOR.$$MINOR"; \
	ln -s "$$LIB.so.$$VERSION"        "$$LIB_DIR/$$LIB.so.$$MAJOR.$$MINOR"; \
	ln -s "$$LIB.so.$$MAJOR.$$MINOR"  "$$LIB_DIR/$$LIB.so.$$MAJOR"; \
	ln -s "$$LIB.so.$$MAJOR"          "$$LIB_DIR/$$LIB.so"

# Ensure build directory exists
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Build C test binaries (depends on library)
$(BUILD_DIR)/%: $(TEST_DIR)/%.c lib | $(BUILD_DIR)
	@echo "Compiling $@..."
	@$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

# Run C tests
test-c: $(TEST_BINARIES)
	@echo "Running C test suite..."
	@for test in $(TEST_BINARIES); do \
		echo ""; \
		echo "========================================"; \
		echo "Running $$test"; \
		echo "========================================"; \
		./$$test --verbose || exit 1; \
	done
	@echo ""
	@echo "========================================"
	@echo "All C tests passed!"
	@echo "========================================"

# Run C tests with XML output (for CI reporting)
test-c-xml: $(TEST_BINARIES)
	@mkdir -p $(BUILD_DIR)/test-results
	@echo "Running C test suite with XML reporting..."
	@for test in $(TEST_BINARIES); do \
		name=$$(basename $$test); \
		echo "Running $$name..."; \
		./$$test --output=xml:$(BUILD_DIR)/test-results/$$name.xml || exit 1; \
	done
	@echo "Test results written to $(BUILD_DIR)/test-results/"

# Build C++ test binaries (depends on library)
$(BUILD_DIR)/test_%: $(CPP_TEST_DIR)/test_%.cpp lib | $(BUILD_DIR)
	@echo "Compiling $@ (C++)..."
	@$(CXX) $(CXXFLAGS) -o $@ $< $(CXXLDFLAGS)

$(BUILD_DIR)/test_%_asan: $(CPP_TEST_DIR)/test_%.cpp lib | $(BUILD_DIR)
	@echo "Compiling $@ (C++ ASan)..."
	@$(CXX) $(CXXFLAGS_ASAN) -o $@ $< $(CXXLDFLAGS_ASAN)

# test_zero_copy deliberately overrides operator new with __builtin_malloc,
# which GCC 15 flags as mismatched. The suppression is scoped to this one
# translation unit only.
$(BUILD_DIR)/test_zero_copy:      CXXFLAGS += -Wno-mismatched-new-delete
$(BUILD_DIR)/test_zero_copy_asan: CXXFLAGS_ASAN += -Wno-mismatched-new-delete

# Run C++ tests
test-cpp: $(CPP_TEST_BINARIES)
	@echo "Running C++ test suite..."
	@for test in $(CPP_TEST_BINARIES); do \
		echo ""; \
		echo "========================================"; \
		echo "Running $$test"; \
		echo "========================================"; \
		./$$test || exit 1; \
	done
	@echo ""
	@echo "========================================"
	@echo "All C++ tests passed!"
	@echo "========================================"

# Run C++ tests under AddressSanitizer + UBSan
test-cpp-asan: $(CPP_TEST_BINARIES_ASAN)
	@echo "Running C++ test suite under ASan/UBSan..."
	@for test in $(CPP_TEST_BINARIES_ASAN); do \
		echo ""; \
		echo "========================================"; \
		echo "Running $$test"; \
		echo "========================================"; \
		./$$test || exit 1; \
	done
	@echo ""
	@echo "========================================"
	@echo "All C++ tests passed under ASan/UBSan!"
	@echo "========================================"

# Run C++ tests with Catch2 JUnit XML output (for CI reporting)
test-cpp-xml: $(CPP_TEST_BINARIES)
	@mkdir -p $(BUILD_DIR)/test-results
	@echo "Running C++ test suite with XML reporting..."
	@for test in $(CPP_TEST_BINARIES); do \
		name=$$(basename $$test); \
		echo "Running $$name..."; \
		./$$test --reporter junit --out $(BUILD_DIR)/test-results/$$name.xml || exit 1; \
	done
	@echo "Test results written to $(BUILD_DIR)/test-results/"

# Run C++ tests under AddressSanitizer + UBSan with JUnit XML output
test-cpp-asan-xml: $(CPP_TEST_BINARIES_ASAN)
	@mkdir -p $(BUILD_DIR)/test-results
	@echo "Running C++ test suite under ASan/UBSan with JUnit XML reporting..."
	@for test in $(CPP_TEST_BINARIES_ASAN); do \
		name=$$(basename $$test); \
		echo "Running $$name..."; \
		./$$test --reporter junit --out $(BUILD_DIR)/test-results/$$name.xml || exit 1; \
	done
	@echo "ASan test results written to $(BUILD_DIR)/test-results/"

# ============================================================================
# Python tests + Python-driven coverage
# ============================================================================
# Since the Python module is now a pyo3 cdylib (Rust-backed) rather than
# pure Python, we measure Python-driven coverage by instrumenting the
# Rust build and accumulating profraw files via cargo-llvm-cov. This
# matches the rust-and-c-test job's coverage flow.
#
# Usage:
#   make test-python                         # run pytest, no coverage
#   make test-python-coverage                # pytest + emit coverage-python.lcov
#
# Both targets expect a `venv/` at the repo root with maturin + pytest
# installed; if none exists, they fall back to the system-installed tools.

PYTHON_VENV       := $(shell if [ -x venv/bin/python ]; then echo "venv/bin/python"; else echo "python3"; fi)
PYTHON_PYTEST     := $(shell if [ -x venv/bin/pytest ]; then echo "venv/bin/pytest"; else echo "pytest"; fi)
PYTHON_MATURIN    := $(shell if [ -x venv/bin/maturin ]; then echo "venv/bin/maturin"; else echo "maturin"; fi)

test-python:
	@echo "Building pyo3 module (release) and running Python tests..."
	@$(PYTHON_MATURIN) develop --release --manifest-path crates/python/Cargo.toml
	@$(PYTHON_PYTEST) tests/python/ -v

test-python-coverage:
	@command -v cargo-llvm-cov >/dev/null 2>&1 || { \
		echo "ERROR: cargo-llvm-cov not installed. Install with: cargo install cargo-llvm-cov"; \
		exit 1; \
	}
	@echo "Running Python tests under cargo-llvm-cov instrumentation..."
	@# cargo llvm-cov show-env exports RUSTFLAGS / LLVM_PROFILE_FILE so
	@# the subsequent maturin build links the cdylib with profraw emission.
	@# The shell `eval` (rather than `source`) keeps this portable across
	@# /bin/sh implementations make may invoke.
	@eval "$$(cargo llvm-cov show-env --export-prefix)" \
		&& $(PYTHON_MATURIN) develop --manifest-path crates/python/Cargo.toml \
		&& $(PYTHON_PYTEST) tests/python/ -v \
		&& cargo llvm-cov report --lcov --output-path coverage-python.lcov
	@echo "Python-driven coverage written to coverage-python.lcov"
	@echo "Lines with coverage: $$(grep -c '^DA:' coverage-python.lcov)"

# ============================================================================
# Documentation
# ============================================================================
# Generate API documentation using Doxygen. Reads Doxyfile.in as a template
# and substitutes @PROJECT_VERSION@ from Cargo.toml at generation time.
# Output lands in build/docs/html (browse-friendly) and build/docs/man.

docs: | $(BUILD_DIR)
	@command -v doxygen >/dev/null 2>&1 || { \
	    echo "error: doxygen not found — install via: sudo apt-get install doxygen" >&2; \
	    exit 1; \
	}
	@echo "Generating API documentation (doxygen)..."
	@mkdir -p $(BUILD_DIR)/docs
	@sed 's|@PROJECT_VERSION@|$(VERSION_FULL)|g' Doxyfile.in > $(BUILD_DIR)/Doxyfile
	@doxygen $(BUILD_DIR)/Doxyfile
	@echo ""
	@echo "========================================"
	@echo "Documentation generated:"
	@echo "  HTML: $(BUILD_DIR)/docs/html/index.html"
	@echo "  Man:  $(BUILD_DIR)/docs/man/"
	@echo "========================================"

docs-clean:
	@rm -rf $(BUILD_DIR)/docs $(BUILD_DIR)/Doxyfile

# Build the C++ example (if present)
example-cpp: lib | $(BUILD_DIR)
	@if [ -f examples/cpp/example.cpp ]; then \
		echo "Compiling examples/cpp/example.cpp..."; \
		$(CXX) $(CXXFLAGS) -o $(BUILD_DIR)/example_cpp examples/cpp/example.cpp $(CXXLDFLAGS); \
		echo "Built $(BUILD_DIR)/example_cpp"; \
	else \
		echo "examples/cpp/example.cpp not found — skipping"; \
	fi

# Install headers and library to PREFIX (default /usr/local)
# Respects DESTDIR for staged installs.
install: lib
	@echo "Installing headers to $(INCDIR)/edgefirst/..."
	@install -d $(INCDIR)/edgefirst/stdlib
	@install -m 644 include/edgefirst/schemas.h             $(INCDIR)/edgefirst/schemas.h
	@install -m 644 include/edgefirst/schemas.hpp           $(INCDIR)/edgefirst/schemas.hpp
	@install -m 644 include/edgefirst/stdlib/expected.hpp   $(INCDIR)/edgefirst/stdlib/expected.hpp
	@install -m 644 include/edgefirst/stdlib/span.hpp       $(INCDIR)/edgefirst/stdlib/span.hpp
	@echo "Installing library to $(LIBDIR)/..."
	@install -d $(LIBDIR)
	@set -e; \
	LIB='$(LIB_NAME)'; VERSION='$(VERSION_FULL)'; \
	MAJOR='$(VERSION_MAJOR)'; MINOR='$(VERSION_MINOR)'; \
	install -m 755 $(LIB_DIR)/$$LIB.so.$$VERSION $(LIBDIR)/$$LIB.so.$$VERSION; \
	ln -sf $$LIB.so.$$VERSION        $(LIBDIR)/$$LIB.so.$$MAJOR.$$MINOR; \
	ln -sf $$LIB.so.$$MAJOR.$$MINOR  $(LIBDIR)/$$LIB.so.$$MAJOR; \
	ln -sf $$LIB.so.$$MAJOR          $(LIBDIR)/$$LIB.so
	@echo "Installing pkg-config file to $(LIBDIR)/pkgconfig/..."
	@install -d $(LIBDIR)/pkgconfig
	@sed \
		-e 's|@VERSION@|$(VERSION_FULL)|g' \
		edgefirst-schemas.pc.in > $(BUILD_DIR)/edgefirst-schemas.pc
	@install -m 644 $(BUILD_DIR)/edgefirst-schemas.pc \
		$(LIBDIR)/pkgconfig/edgefirst-schemas.pc
	@echo "Installed edgefirst-schemas $(VERSION_FULL) to $(DESTDIR)$(PREFIX)"

clean:
	@echo "Cleaning build artifacts..."
	@rm -rf $(BUILD_DIR)
	@cargo clean

help:
	@echo "EdgeFirst Schemas Build System"
	@echo ""
	@echo "Targets:"
	@echo "  all          - Build library and C tests"
	@echo "  lib          - Build Rust library"
	@echo "  test-c       - Build and run C tests"
	@echo "  test-c-xml   - Build and run C tests with XML output (for CI)"
	@echo "  test-cpp     - Build and run C++ tests"
	@echo "  test-cpp-asan - Build and run C++ tests under ASan/UBSan"
	@echo "  test-cpp-xml - Build and run C++ tests with JUnit XML output"
	@echo "  test-cpp-asan-xml - Build and run C++ tests under ASan/UBSan with JUnit XML output"
	@echo "  test-python  - Run Python tests (pyo3 binding); requires maturin + a venv"
	@echo "  test-python-coverage - Run Python tests under cargo-llvm-cov; emits"
	@echo "                          coverage-python.lcov attributing Python-driven"
	@echo "                          execution to the Rust source (the .pyi/.so module"
	@echo "                          is now Rust, so we use llvm-cov rather than"
	@echo "                          coverage.py)"
	@echo "  example-cpp  - Build the C++ example"
	@echo "  install      - Install headers and library to PREFIX (default /usr/local)"
	@echo "  docs         - Generate API documentation (Doxygen) to build/docs/"
	@echo "  docs-clean   - Remove generated documentation"
	@echo "  clean        - Remove all build artifacts"
	@echo ""
	@echo "Variables:"
	@echo "  RELEASE=1    - Build release library (default)"
	@echo "  RELEASE=0    - Build debug library (for coverage testing)"
	@echo "  CXXSTD=c++17 - C++ standard (c++17 default, c++20 supported)"
	@echo "  PREFIX=/usr/local - Install prefix"
	@echo "  DESTDIR=     - Stage directory for packaging"
	@echo ""
	@echo "C test binaries are built to: $(BUILD_DIR)/"
