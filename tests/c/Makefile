# EdgeFirst Schemas C Test Suite Makefile
# Uses Criterion unit testing framework

CC = gcc
CRITERION_PREFIX = $(shell brew --prefix criterion 2>/dev/null || echo /usr/local)
CFLAGS = -Wall -Wextra -Werror -std=c11 -I../../include -I$(CRITERION_PREFIX)/include
LDFLAGS = -L../../target/release -ledgefirst_schemas -L$(CRITERION_PREFIX)/lib -lcriterion -Wl,-rpath,../../target/release

# Test sources
TEST_SOURCES = test_builtin_interfaces.c \
               test_std_msgs.c \
               test_geometry_msgs.c \
               test_edgefirst_msgs.c \
               test_sensor_msgs.c \
               test_errno.c

TEST_BINARIES = $(TEST_SOURCES:.c=)

.PHONY: all clean test build-lib

all: build-lib $(TEST_BINARIES)

build-lib:
	@echo "Building Rust library..."
	@cd ../.. && cargo build --release

$(TEST_BINARIES): %: %.c
	@echo "Compiling $@..."
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

test: all
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

clean:
	rm -f $(TEST_BINARIES)
	rm -f *.o

help:
	@echo "EdgeFirst Schemas C Test Suite"
	@echo ""
	@echo "Targets:"
	@echo "  all        - Build library and compile all tests"
	@echo "  test       - Run all test suites"
	@echo "  clean      - Remove test binaries"
	@echo "  build-lib  - Build Rust library"
	@echo ""
	@echo "Individual tests:"
	@for test in $(TEST_BINARIES); do \
		echo "  $$test"; \
	done
