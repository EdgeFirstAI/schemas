# C++ Codec Benchmarks

Standalone benchmark suite comparing edgefirst-schemas C++ wrappers against
**eProsima Fast-CDR** and **Eclipse Cyclone DDS** — the codecs under the two
major rmw vendors in current ROS 2 releases. Not part of the main
cargo/pip build.

For methodology, fixture definitions, and full results, see [BENCHMARKS.md](../../BENCHMARKS.md)
in the repository root.

## Quick start

Cross-compile, deploy to an aarch64 target (e.g., Raspberry Pi 5 or NXP
i.MX 8M Plus), run, and render report:

    export BENCH_TARGET_HOST=user@your-target.local
    ./benchmark.sh --render

See `benchmark.sh --help` for the full set of options.

## Layout

- `bench_edgefirst.cpp`, `bench_fastcdr.cpp`, `bench_cyclonedds.cpp` —
  Google Benchmark suites per backend
- `common.hpp` — naming-convention constants, fixture definitions
- `idl/` — hand-written OMG IDL inputs (committed)
- `types/fastcdr/` — fastddsgen-generated C++ types (committed; regenerable
  via `scripts/regen_fastcdr_types.sh`)
- `types/cyclonedds/` — `idlc -l cxx` generated types for Cyclone DDS
  (committed; regenerable via `scripts/regen_cyclonedds_types.sh`)
- `tests/parity_test.cpp` — wire-byte parity assertion across all three
  backends (Cyclone-specific encoders live in `tests/parity_cyclonedds.cpp`
  to isolate generated-type namespace conflicts with Fast-CDR)
- `cmake/aarch64-{zig,gcc}.cmake` — cross-compile toolchain files
- `benchmark.sh` — local driver

## Adding a new backend

See the "Adding a new backend" section in [BENCHMARKS.md](../../BENCHMARKS.md).
