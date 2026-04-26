#!/usr/bin/env bash
# benchmark.sh — Build, deploy, and run on-target C++ codec benchmarks.
#
# Drives the full local-dev workflow:
#   1. cargo cross-compile the Rust shared library (aarch64)
#   2. CMake cross-build the C++ benchmark targets
#   3. rsync binaries + libedgefirst_schemas.so to the target
#   4. Run each bench on target with JSON output
#   5. Pull results back into a local results/ directory
#   6. Optionally invoke render_benchmarks.py for a local Markdown report
#
# Default toolchain is zig (single-tool cross-compile, matches the wider
# repo philosophy). gcc-aarch64-linux-gnu is supported as a fallback.
#
# Author: Sébastien Taylor <sebastien@au-zone.com>

set -euo pipefail

# ---------------------------------------------------------------------------
# Resolve the script's own directory so all paths are stable regardless of
# where the caller invokes it from.
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# ---------------------------------------------------------------------------
# Defaults (can be overridden by env vars or flags)
# ---------------------------------------------------------------------------
TOOLCHAIN="${BENCH_TOOLCHAIN:-zig}"
BUILD_DIR="${SCRIPT_DIR}/build/aarch64"
CLEAN=0
BUILD_ONLY=0
TARGET_HOST="${BENCH_TARGET_HOST:-}"
REMOTE_PATH="${BENCH_REMOTE_PATH:-/tmp/edgefirst-bench}"
RUN_ONLY=0
IMPLS_ALL=(edgefirst fastcdr cyclonedds)
IMPLS=()        # empty = all
FILTER=""
QUICK=0
OUTPUT_DIR=""
RENDER=0

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"

# ---------------------------------------------------------------------------
# Usage
# ---------------------------------------------------------------------------
usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Build, deploy, and run on-target C++ codec benchmarks.

Build:
  --toolchain {zig|gcc}     Cross-compile toolchain (default: zig)
  --build-dir DIR           CMake build directory (default: build/aarch64 under benches/cpp/)
  --clean                   Wipe build dir before configuring
  --build-only              Stop after build; --target is not required

Deploy & run:
  --target HOST             SSH destination, e.g. user@imx8mp.local
                            (env: BENCH_TARGET_HOST)
  --remote-path PATH        Remote directory for binaries (default: /tmp/edgefirst-bench)
                            (env: BENCH_REMOTE_PATH)
  --run-only                Skip build and deploy; re-run and collect from target
  --impl LIST               Comma-separated subset of: edgefirst,fastcdr,cyclonedds
                            (default: all)
  --filter REGEX            Passed as --benchmark_filter=<REGEX> to each binary
  --quick                   Pass --benchmark_min_time=0.05s for fast feedback

Output:
  --output-dir DIR          Local results directory (default: results/<utc-timestamp>)
  --render                  Generate charts + Markdown report after collecting results

  -h, --help                Show this message and exit

Environment variables (lowest precedence; CLI flags override):
  BENCH_TARGET_HOST         Same as --target
  BENCH_REMOTE_PATH         Same as --remote-path
  BENCH_TOOLCHAIN           Same as --toolchain
EOF
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --toolchain)
            TOOLCHAIN="$2"; shift 2 ;;
        --build-dir)
            BUILD_DIR="$2"; shift 2 ;;
        --clean)
            CLEAN=1; shift ;;
        --build-only)
            BUILD_ONLY=1; shift ;;
        --target)
            TARGET_HOST="$2"; shift 2 ;;
        --remote-path)
            REMOTE_PATH="$2"; shift 2 ;;
        --run-only)
            RUN_ONLY=1; shift ;;
        --impl)
            IFS=',' read -ra IMPLS <<< "$2"; shift 2 ;;
        --filter)
            FILTER="$2"; shift 2 ;;
        --quick)
            QUICK=1; shift ;;
        --output-dir)
            OUTPUT_DIR="$2"; shift 2 ;;
        --render)
            RENDER=1; shift ;;
        -h|--help)
            usage; exit 0 ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 1 ;;
    esac
done

# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------
if [[ "$TOOLCHAIN" != "zig" && "$TOOLCHAIN" != "gcc" ]]; then
    echo "error: --toolchain must be 'zig' or 'gcc', got: '$TOOLCHAIN'" >&2
    exit 1
fi

if [[ ${#IMPLS[@]} -eq 0 ]]; then
    IMPLS=("${IMPLS_ALL[@]}")
fi

for impl in "${IMPLS[@]}"; do
    found=0
    for valid in "${IMPLS_ALL[@]}"; do
        [[ "$impl" == "$valid" ]] && found=1 && break
    done
    if [[ $found -eq 0 ]]; then
        echo "error: unknown impl '$impl'; valid values: ${IMPLS_ALL[*]}" >&2
        exit 1
    fi
done

# --target is required unless we're only building (deploy and run-only both
# need an SSH destination).
if [[ $BUILD_ONLY -eq 0 && -z "$TARGET_HOST" ]]; then
    echo "error: --target HOST is required unless --build-only is set" >&2
    echo "       (or set BENCH_TARGET_HOST in the environment)" >&2
    exit 1
fi

# Default output dir after validation so TIMESTAMP is stable
if [[ -z "$OUTPUT_DIR" ]]; then
    OUTPUT_DIR="${SCRIPT_DIR}/results/${TIMESTAMP}"
fi

# ---------------------------------------------------------------------------
# Phase: BUILD
# ---------------------------------------------------------------------------
if [[ $RUN_ONLY -eq 0 ]]; then
    echo ""
    echo "=== [1/4] Cargo cross-compile (aarch64-unknown-linux-gnu) ==="
    cd "${REPO_ROOT}"
    # Set the cross-linker for the aarch64 target unless the caller has
    # already provided a .cargo/config.toml or the env var themselves.
    if [[ -z "${CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER:-}" ]]; then
        case "$TOOLCHAIN" in
            gcc) export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER=aarch64-linux-gnu-gcc ;;
            zig) export CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER="${SCRIPT_DIR}/cmake/zig-wrappers/zig-cc" ;;
        esac
    fi
    cargo build --release --target aarch64-unknown-linux-gnu

    echo ""
    echo "=== [2/4] CMake configure & build (toolchain: ${TOOLCHAIN}) ==="
    cd "${SCRIPT_DIR}"

    if [[ $CLEAN -eq 1 && -d "$BUILD_DIR" ]]; then
        echo "    Cleaning ${BUILD_DIR} ..."
        rm -rf "$BUILD_DIR"
    fi

    mkdir -p "$BUILD_DIR"

    # EF_LIB_DIR_OVERRIDE is read by CMakeLists.txt via ENV{EF_LIB_DIR_OVERRIDE}.
    # Export it so cmake inherits it; do not override if the caller has set it.
    if [[ -z "${EF_LIB_DIR_OVERRIDE:-}" ]]; then
        export EF_LIB_DIR_OVERRIDE="${REPO_ROOT}/target/aarch64-unknown-linux-gnu/release"
    fi
    EF_LIB_DIR="${EF_LIB_DIR_OVERRIDE}"

    # Pre-set EF_SCHEMAS_LIB to bypass CMake's find_library, which is
    # restricted by CMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY in the toolchain
    # file and cannot resolve paths outside /usr/aarch64-linux-gnu.
    cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
        -DCMAKE_TOOLCHAIN_FILE="${SCRIPT_DIR}/cmake/aarch64-${TOOLCHAIN}.cmake" \
        -DCMAKE_BUILD_TYPE=Release \
        "-DEF_SCHEMAS_LIB=${EF_LIB_DIR}/libedgefirst_schemas.so"

    cmake --build "${BUILD_DIR}" --parallel

    if [[ $BUILD_ONLY -eq 1 ]]; then
        echo ""
        echo "=== Build complete (--build-only). ==="
        exit 0
    fi
fi

# ---------------------------------------------------------------------------
# Phase: DEPLOY
# ---------------------------------------------------------------------------
if [[ $RUN_ONLY -eq 0 ]]; then
    echo ""
    echo "=== [3/4] Deploy to ${TARGET_HOST}:${REMOTE_PATH} ==="

    ssh "${TARGET_HOST}" "mkdir -p '${REMOTE_PATH}'"

    EF_LIB_DIR="${EF_LIB_DIR_OVERRIDE:-${REPO_ROOT}/target/aarch64-unknown-linux-gnu/release}"

    # Collect bench binaries; fail fast if any requested impl's binary is
    # missing so the user gets a clear actionable error instead of a confusing
    # rsync/cp failure later.
    BENCH_BINS=()
    MISSING_BINS=()
    for impl in "${IMPLS[@]}"; do
        bin="${BUILD_DIR}/bench_${impl}"
        if [[ -f "$bin" ]]; then
            BENCH_BINS+=("$bin")
        else
            MISSING_BINS+=("$bin")
        fi
    done
    if [[ ${#MISSING_BINS[@]} -gt 0 ]]; then
        echo "error: bench binaries not built (build the missing targets or remove from --impl):" >&2
        for b in "${MISSING_BINS[@]}"; do
            echo "       ${b}" >&2
        done
        exit 1
    fi

    # Shared library. Cargo emits the file as libedgefirst_schemas.so but the
    # binaries link against it via the SONAME (e.g. libedgefirst_schemas.so.3).
    # Read the SONAME and stage the .so under that name so the dynamic linker
    # finds it at runtime on the target.
    EF_SO="${EF_LIB_DIR}/libedgefirst_schemas.so"
    if [[ ! -f "$EF_SO" ]]; then
        echo "error: shared library not found: ${EF_SO}" >&2
        exit 1
    fi
    EF_SO_SONAME="$(readelf -d "$EF_SO" | awk '/SONAME/ { gsub(/[][]/, "", $NF); print $NF }')"
    if [[ -z "$EF_SO_SONAME" ]]; then
        EF_SO_SONAME="libedgefirst_schemas.so"
    fi

    DEPLOY_STAGING="$(mktemp -d)"
    trap 'rm -rf "$DEPLOY_STAGING"' EXIT
    cp "${BENCH_BINS[@]}" "$DEPLOY_STAGING/"
    cp "$EF_SO" "$DEPLOY_STAGING/$EF_SO_SONAME"

    rsync -az --progress "$DEPLOY_STAGING/"* "${TARGET_HOST}:${REMOTE_PATH}/"
fi

# ---------------------------------------------------------------------------
# Phase: RUN
# ---------------------------------------------------------------------------
echo ""
echo "=== [4/4] Run benchmarks on ${TARGET_HOST} ==="

mkdir -p "${OUTPUT_DIR}"

for impl in "${IMPLS[@]}"; do
    echo ""
    echo "    Running bench_${impl} ..."

    BENCH_ARGS=(
        "--benchmark_format=json"
        "--benchmark_out=${REMOTE_PATH}/${impl}.json"
    )
    [[ -n "$FILTER" ]] && BENCH_ARGS+=("--benchmark_filter=${FILTER}")
    [[ $QUICK -eq 1 ]]  && BENCH_ARGS+=("--benchmark_min_time=0.05s")

    # Shell-escape every argument before embedding into the remote command so
    # values containing shell metacharacters (regex alternation, wildcards,
    # spaces) reach the target binary literally instead of being reinterpreted
    # by the remote shell.
    printf -v REMOTE_PATH_Q '%q' "${REMOTE_PATH}"
    REMOTE_BIN_Q="$(printf '%q' "./bench_${impl}")"
    BENCH_ARGS_Q=""
    for arg in "${BENCH_ARGS[@]}"; do
        BENCH_ARGS_Q+=" $(printf '%q' "$arg")"
    done

    # The JSON is captured on the target via --benchmark_out; suppress the
    # stdout copy locally so a large stream doesn't choke the SSH session.
    # shellcheck disable=SC2029
    ssh "${TARGET_HOST}" \
        "cd ${REMOTE_PATH_Q} && LD_LIBRARY_PATH=${REMOTE_PATH_Q} ${REMOTE_BIN_Q}${BENCH_ARGS_Q} > /dev/null"

    echo "    Pulling ${impl}.json ..."
    rsync -az "${TARGET_HOST}:${REMOTE_PATH}/${impl}.json" \
        "${OUTPUT_DIR}/${impl}.json"
done

# ---------------------------------------------------------------------------
# Collect system metadata
# ---------------------------------------------------------------------------
echo ""
echo "    Collecting system metadata ..."

UNAME_STR="$(ssh "${TARGET_HOST}" 'uname -a' 2>/dev/null || echo 'unavailable')"
CPUINFO_STR="$(ssh "${TARGET_HOST}" 'head -30 /proc/cpuinfo' 2>/dev/null || echo 'unavailable')"

case "$TOOLCHAIN" in
    gcc)
        TC_VERSION="$(aarch64-linux-gnu-gcc --version 2>&1 | head -1 || echo 'unavailable')" ;;
    zig)
        TC_VERSION="$(zig version 2>&1 || echo 'unavailable')" ;;
esac

GIT_REV="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo 'unavailable')"

SYSTEM_JSON="${OUTPUT_DIR}/system.json"

# Write system.json via Python so all strings (including multiline cpuinfo)
# are properly JSON-escaped.  Pass values through environment variables to
# avoid word-splitting / quoting hazards on newlines.
BENCH_TIMESTAMP="$TIMESTAMP" \
BENCH_GIT_REV="$GIT_REV" \
BENCH_TOOLCHAIN_NAME="$TOOLCHAIN" \
BENCH_TC_VERSION="$TC_VERSION" \
BENCH_TARGET_H="$TARGET_HOST" \
BENCH_UNAME="$UNAME_STR" \
BENCH_CPUINFO="$CPUINFO_STR" \
BENCH_OUT="$SYSTEM_JSON" \
python3 - <<'PYEOF'
import json, os

data = {
    "timestamp":         os.environ["BENCH_TIMESTAMP"],
    "git_rev":           os.environ["BENCH_GIT_REV"],
    "toolchain":         os.environ["BENCH_TOOLCHAIN_NAME"],
    "toolchain_version": os.environ["BENCH_TC_VERSION"],
    "target_host":       os.environ["BENCH_TARGET_H"],
    "uname":             os.environ["BENCH_UNAME"],
    "cpuinfo_head30":    os.environ["BENCH_CPUINFO"],
}
out = os.environ["BENCH_OUT"]
with open(out, "w") as f:
    json.dump(data, f, indent=2)
print(f"    Wrote {out}")
PYEOF

# ---------------------------------------------------------------------------
# Phase: RENDER (optional)
# ---------------------------------------------------------------------------
if [[ $RENDER -eq 1 ]]; then
    echo ""
    echo "=== Rendering report ==="
    RENDERER="${REPO_ROOT}/scripts/render_benchmarks.py"
    REPORT="${OUTPUT_DIR}/BENCHMARKS-${TIMESTAMP}.md"
    python3 "${RENDERER}" "${OUTPUT_DIR}" > "${REPORT}"
    echo "    Report: ${REPORT}"
fi

echo ""
echo "=== Done. Results in: ${OUTPUT_DIR} ==="
