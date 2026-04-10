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

# Build output directory
BUILD_DIR = build

# C test sources
TEST_DIR = tests/c
TEST_SOURCES = $(wildcard $(TEST_DIR)/test_*.c)
TEST_BINARIES = $(patsubst $(TEST_DIR)/%.c,$(BUILD_DIR)/%,$(TEST_SOURCES))

.PHONY: all lib test-c test-c-xml clean help

all: lib $(TEST_BINARIES)

# Parse semver components from Cargo.toml for the SOVERSION chain.
# Using := (immediate expansion) so the shell runs once per make invocation.
VERSION_FULL  := $(shell grep '^version = ' Cargo.toml | head -1 | sed 's/version = "\(.*\)"/\1/')
VERSION_MAJOR := $(word 1,$(subst ., ,$(VERSION_FULL)))
VERSION_MINOR := $(word 2,$(subst ., ,$(VERSION_FULL)))

# Build the Rust library and arrange the GNU/Linux SOVERSION symlink chain:
#
#   libedgefirst_schemas.so                         -> .so.MAJOR
#   libedgefirst_schemas.so.MAJOR                   -> .so.MAJOR.MINOR   (embedded SONAME)
#   libedgefirst_schemas.so.MAJOR.MINOR             -> .so.MAJOR.MINOR.PATCH
#   libedgefirst_schemas.so.MAJOR.MINOR.PATCH       real file (renamed from cargo output)
#
# Rationale: rustc writes the cdylib to `libedgefirst_schemas.so`; on first build
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

clean:
	@echo "Cleaning build artifacts..."
	@rm -rf $(BUILD_DIR)
	@cargo clean

help:
	@echo "EdgeFirst Schemas Build System"
	@echo ""
	@echo "Targets:"
	@echo "  all        - Build library and C tests"
	@echo "  lib        - Build Rust library"
	@echo "  test-c     - Build and run C tests"
	@echo "  test-c-xml - Build and run C tests with XML output (for CI)"
	@echo "  clean      - Remove all build artifacts"
	@echo ""
	@echo "Variables:"
	@echo "  RELEASE=1  - Build release library (default)"
	@echo "  RELEASE=0  - Build debug library (for coverage testing)"
	@echo ""
	@echo "C test binaries are built to: $(BUILD_DIR)/"
