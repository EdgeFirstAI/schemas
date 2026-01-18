# Makefile for EdgeFirst Schemas C API examples
# SPDX-License-Identifier: Apache-2.0

CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -I../../include
LDFLAGS = -L../../target/release -ledgefirst_schemas

# Determine platform-specific library extension
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
    DYLIB_PATH_VAR = DYLD_LIBRARY_PATH
    LIB_EXT = dylib
else ifeq ($(UNAME_S),Linux)
    DYLIB_PATH_VAR = LD_LIBRARY_PATH
    LIB_EXT = so
else
    # Windows
    DYLIB_PATH_VAR = PATH
    LIB_EXT = dll
endif

.PHONY: all clean run

all: example

example: example.c
	@echo "Building C example..."
	$(CC) $(CFLAGS) -o example example.c $(LDFLAGS)
	@echo "Build complete!"

run: example
	@echo "Running C example..."
	$(DYLIB_PATH_VAR)=../../target/release ./example

clean:
	rm -f example
	@echo "Cleaned build artifacts"
