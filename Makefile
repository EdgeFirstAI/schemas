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
# Shared-library naming is platform-specific. crates/capi sets
# `[lib] name = "edgefirst_schemas"`, so cargo writes the shipped artifact name
# directly on every platform — there is no second basename to rename around.
#
#   Linux    libedgefirst_schemas.so     libedgefirst_schemas.a
#   macOS    libedgefirst_schemas.dylib  libedgefirst_schemas.a
#   Windows  edgefirst_schemas.dll       edgefirst_schemas.lib (+ .dll.lib import)
UNAME_S   := $(shell uname -s)
WIN_HOST  := $(filter MINGW% MSYS% CYGWIN% Windows%,$(UNAME_S))
ifeq ($(UNAME_S),Darwin)
  HOST_OS = macos
else ifneq (,$(WIN_HOST))
  HOST_OS = windows
else
  HOST_OS = linux
endif

ifeq ($(HOST_OS),windows)
  LIB_NAME  = edgefirst_schemas
  SHLIB     = $(LIB_NAME).dll
  STATICLIB = $(LIB_NAME).lib
  IMPLIB    = $(LIB_NAME).dll.lib
else
  LIB_NAME  = libedgefirst_schemas
  STATICLIB = $(LIB_NAME).a
ifeq ($(HOST_OS),macos)
  SHLIB      = $(LIB_NAME).dylib
  # macOS puts the compatibility version before the extension.
  SOVER_FULL  = $(LIB_NAME).$(VERSION_FULL).dylib
  SOVER_MM    = $(LIB_NAME).$(VERSION_MAJOR).$(VERSION_MINOR).dylib
  SOVER_MAJOR = $(LIB_NAME).$(VERSION_MAJOR).dylib
else
  SHLIB      = $(LIB_NAME).so
  SOVER_FULL  = $(LIB_NAME).so.$(VERSION_FULL)
  SOVER_MM    = $(LIB_NAME).so.$(VERSION_MAJOR).$(VERSION_MINOR)
  SOVER_MAJOR = $(LIB_NAME).so.$(VERSION_MAJOR)
endif
endif

# C test configuration
CC = gcc
CRITERION_PREFIX = $(shell brew --prefix criterion 2>/dev/null || echo /usr/local)
CFLAGS = -Wall -Wextra -Werror -std=c11 -I./crates/capi/include -I$(CRITERION_PREFIX)/include
LDFLAGS = -L$(LIB_DIR) -ledgefirst_schemas -L$(CRITERION_PREFIX)/lib -lcriterion -lm -Wl,-rpath,$(LIB_DIR)

# C++ test configuration
CXX ?= g++
CXXSTD ?= c++17
CXXFLAGS_BASE = -std=$(CXXSTD) -Wall -Wextra -Werror -I./crates/capi/include -Icrates/capi/tests/cpp
CXXFLAGS = $(CXXFLAGS_BASE) -O2
CXXFLAGS_ASAN = $(CXXFLAGS_BASE) -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -Wno-maybe-uninitialized
CXXLDFLAGS = -L$(LIB_DIR) -ledgefirst_schemas -Wl,-rpath,$(LIB_DIR)
CXXLDFLAGS_ASAN = $(CXXLDFLAGS) -fsanitize=address,undefined

# C++ test sources and binaries
CPP_TEST_DIR = crates/capi/tests/cpp
CPP_TEST_SOURCES = $(wildcard $(CPP_TEST_DIR)/test_*.cpp)
CPP_TEST_BINARIES = $(patsubst $(CPP_TEST_DIR)/%.cpp,$(BUILD_DIR)/%,$(CPP_TEST_SOURCES))
CPP_TEST_BINARIES_ASAN = $(patsubst $(CPP_TEST_DIR)/%.cpp,$(BUILD_DIR)/%_asan,$(CPP_TEST_SOURCES))

# Install prefix and destdir (standard GNU conventions)
DESTDIR ?=
PREFIX  ?= /usr/local
INCDIR  = $(DESTDIR)$(PREFIX)/include
LIBDIR  = $(DESTDIR)$(PREFIX)/lib
BINDIR  = $(DESTDIR)$(PREFIX)/bin

# Build output directory
BUILD_DIR = build

# C test sources
TEST_DIR = crates/capi/tests/c
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

# Build the Rust library and, on Linux/macOS, drop the soversion link that the
# freshly built test binaries need at run time.
#
# crates/capi/build.rs stamps a versioned identity into the shared library:
#   Linux    DT_SONAME     libedgefirst_schemas.so.MAJOR
#   macOS    LC_ID_DYLIB   @rpath/libedgefirst_schemas.MAJOR.dylib
# That is the name the loader opens, but cargo writes the file under the
# unversioned name, so the build tree needs SOVER_MAJOR -> SHLIB for anything
# linked with -rpath $(LIB_DIR) (the C and C++ suites) to start. The full
# release chain is laid down by `install`, not here — the build tree only needs
# the one hop, and keeping cargo's file as the real one means incremental
# rebuilds cannot leave a stale real file behind a live symlink.
#
# Windows has no soversion-in-filename convention, so there is nothing to link.
lib:
	@echo "Building Rust library ($(HOST_OS))..."
	@cargo build $(CARGO_FLAGS)
	@set -e; \
	if [ ! -f "$(LIB_DIR)/$(SHLIB)" ]; then \
	    echo "error: expected cargo output $(LIB_DIR)/$(SHLIB) not found" >&2; \
	    exit 1; \
	fi
ifneq ($(HOST_OS),windows)
	@set -e; \
	find "$(LIB_DIR)" -maxdepth 1 -type l -name "$(LIB_NAME)*" -exec rm -f {} +; \
	ln -s "$(SHLIB)" "$(LIB_DIR)/$(SOVER_MAJOR)"
endif

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
	@install -m 644 crates/capi/include/edgefirst/schemas.h             $(INCDIR)/edgefirst/schemas.h
	@install -m 644 crates/capi/include/edgefirst/schemas.hpp           $(INCDIR)/edgefirst/schemas.hpp
	@install -m 644 crates/capi/include/edgefirst/stdlib/expected.hpp   $(INCDIR)/edgefirst/stdlib/expected.hpp
	@install -m 644 crates/capi/include/edgefirst/stdlib/span.hpp       $(INCDIR)/edgefirst/stdlib/span.hpp
	@echo "Installing library to $(LIBDIR)/..."
	@install -d $(LIBDIR)
ifeq ($(HOST_OS),windows)
	@install -d $(BINDIR)
	@install -m 755 $(LIB_DIR)/$(SHLIB) $(BINDIR)/$(SHLIB)
	@install -m 644 $(LIB_DIR)/$(STATICLIB) $(LIBDIR)/$(STATICLIB)
	@if [ -f "$(LIB_DIR)/$(IMPLIB)" ]; then \
	    install -m 644 $(LIB_DIR)/$(IMPLIB) $(LIBDIR)/$(IMPLIB); \
	fi
else
	@set -e; \
	install -m 755 $(LIB_DIR)/$(SHLIB) $(LIBDIR)/$(SOVER_FULL); \
	ln -sf $(SOVER_FULL)  $(LIBDIR)/$(SOVER_MM); \
	ln -sf $(SOVER_MM)    $(LIBDIR)/$(SOVER_MAJOR); \
	ln -sf $(SOVER_MAJOR) $(LIBDIR)/$(SHLIB); \
	install -m 644 $(LIB_DIR)/$(STATICLIB) $(LIBDIR)/$(STATICLIB)
endif
ifeq ($(HOST_OS),macos)
	@# The build stamps LC_ID_DYLIB as @rpath/... so the test binaries resolve it
	@# from the build tree. An installed library is found by absolute path, so
	@# retarget the id; consumers that do use an rpath are unaffected.
	@set -e; \
	if command -v install_name_tool >/dev/null 2>&1; then \
	    install_name_tool -id "$(PREFIX)/lib/$(SOVER_MAJOR)" "$(LIBDIR)/$(SOVER_FULL)"; \
	fi
endif
	@echo "Installing pkg-config file to $(LIBDIR)/pkgconfig/..."
	@install -d $(LIBDIR)/pkgconfig
	@sed \
		-e 's|@VERSION@|$(VERSION_FULL)|g' \
		crates/capi/edgefirst-schemas.pc.in > $(BUILD_DIR)/edgefirst-schemas.pc
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
