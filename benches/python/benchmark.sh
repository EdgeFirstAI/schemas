#!/usr/bin/env bash
# benchmark.sh — Build, deploy, and run on-target Python CDR benchmarks.
#
# Drives the full local-dev workflow:
#   1. maturin --zig cross-compile of the pyo3 wheel (aarch64 manylinux2014)
#   2. rsync wheel + bench files + legacy pycdr2 module to the target
#   3. ssh: install the wheel + pycdr2 + cyclonedds in a target-side venv
#   4. ssh: run pytest --benchmark-json for each impl
#   5. Pull result JSON files back into a local results/ directory
#   6. Optionally invoke render_python_benchmarks.py for a Markdown report
#
# Mirror of benches/cpp/benchmark.sh — same flag taxonomy, same output
# layout, same SSH + rsync patterns.
#
# Author: Sébastien Taylor <sebastien@au-zone.com>

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# ---------------------------------------------------------------------------
# Defaults (overridable by env vars or flags)
# ---------------------------------------------------------------------------
TARGET_HOST="${BENCH_TARGET_HOST:-}"
REMOTE_PATH="${BENCH_REMOTE_PATH:-/tmp/edgefirst-bench-py}"
PYTHON_INTERPRETER="${BENCH_PYTHON_INTERPRETER:-python3.11}"  # versioned name needed for maturin --zig
TARGET_TRIPLE="${BENCH_TARGET_TRIPLE:-aarch64-unknown-linux-gnu}"
BUILD_ONLY=0
RUN_ONLY=0
IMPLS_ALL=(native cyclonedds pycdr2)
IMPLS=()                # empty = all
FILTER=""
FAST=0
OUTPUT_DIR=""
RENDER=0

TIMESTAMP="$(date -u +%Y%m%dT%H%M%SZ)"

# ---------------------------------------------------------------------------
# Usage
# ---------------------------------------------------------------------------
usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Build, deploy, and run on-target Python CDR benchmarks.

Build:
  --python NAME             Python interpreter on the target (versioned name
                            required for maturin --zig; default: python3.11)
  --target-triple TRIPLE    Cross-compile target (default: aarch64-unknown-linux-gnu)
  --build-only              Stop after building the wheel; --target not required

Deploy & run:
  --target HOST             SSH destination, e.g. user@rpi5-hailo.local
                            (env: BENCH_TARGET_HOST)
  --remote-path PATH        Remote directory (default: /tmp/edgefirst-bench-py)
                            (env: BENCH_REMOTE_PATH)
  --run-only                Skip build and deploy; rerun on target and pull JSON
  --impl LIST               Comma-separated subset of: native,cyclonedds,pycdr2
                            (default: all)
  --filter REGEX            -k REGEX passed to pytest
  --fast                    Set BENCH_FAST=1 on the target for shorter benches

Output:
  --output-dir DIR          Local results directory (default: results/<utc-timestamp>)
  --render                  Generate BENCHMARKS-<timestamp>.md after collecting

  -h, --help                Show this message and exit

Environment variables:
  BENCH_TARGET_HOST         Same as --target
  BENCH_REMOTE_PATH         Same as --remote-path
  BENCH_PYTHON_INTERPRETER  Same as --python
  BENCH_TARGET_TRIPLE       Same as --target-triple
EOF
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --python)            PYTHON_INTERPRETER="$2"; shift 2 ;;
        --target-triple)     TARGET_TRIPLE="$2"; shift 2 ;;
        --target)            TARGET_HOST="$2"; shift 2 ;;
        --remote-path)       REMOTE_PATH="$2"; shift 2 ;;
        --build-only)        BUILD_ONLY=1; shift ;;
        --run-only)          RUN_ONLY=1; shift ;;
        --impl)              IFS=',' read -ra IMPLS <<< "$2"; shift 2 ;;
        --filter)            FILTER="$2"; shift 2 ;;
        --fast)              FAST=1; shift ;;
        --output-dir)        OUTPUT_DIR="$2"; shift 2 ;;
        --render)            RENDER=1; shift ;;
        -h|--help)           usage; exit 0 ;;
        *)                   echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    esac
done

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

if [[ $BUILD_ONLY -eq 0 && -z "$TARGET_HOST" ]]; then
    echo "error: --target HOST is required unless --build-only is set" >&2
    exit 1
fi

if [[ -z "$OUTPUT_DIR" ]]; then
    OUTPUT_DIR="${SCRIPT_DIR}/results/${TIMESTAMP}"
fi

WHEEL_DIR="${REPO_ROOT}/target/wheels"

# ---------------------------------------------------------------------------
# Phase: BUILD
# ---------------------------------------------------------------------------
if [[ $RUN_ONLY -eq 0 ]]; then
    echo ""
    echo "=== [1/4] Cross-compile pyo3 wheel (${TARGET_TRIPLE}, manylinux2014) ==="
    cd "${REPO_ROOT}"

    if ! command -v zig >/dev/null 2>&1; then
        echo "error: zig not on PATH (required for maturin --zig)" >&2
        exit 1
    fi

    # Use the schemas venv's maturin if present, otherwise system maturin.
    MATURIN="maturin"
    if [[ -x "${REPO_ROOT}/venv/bin/maturin" ]]; then
        MATURIN="${REPO_ROOT}/venv/bin/maturin"
    fi

    "$MATURIN" build --release --zig \
        --compatibility manylinux2014 \
        --target "$TARGET_TRIPLE" \
        --interpreter "$PYTHON_INTERPRETER" \
        --manifest-path "${REPO_ROOT}/crates/python/Cargo.toml"

    if [[ $BUILD_ONLY -eq 1 ]]; then
        echo ""
        echo "=== Build complete (--build-only). Wheel in ${WHEEL_DIR}/ ==="
        exit 0
    fi
fi

# ---------------------------------------------------------------------------
# Locate the freshly built wheel
# ---------------------------------------------------------------------------
WHEEL_FILE="$(ls -1t "${WHEEL_DIR}"/edgefirst_schemas-*.whl 2>/dev/null | head -1 || true)"
if [[ -z "$WHEEL_FILE" ]]; then
    echo "error: no wheel found in ${WHEEL_DIR}; run without --run-only first" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Phase: DEPLOY
# ---------------------------------------------------------------------------
if [[ $RUN_ONLY -eq 0 ]]; then
    echo ""
    echo "=== [2/4] Deploy wheel + bench harness to ${TARGET_HOST}:${REMOTE_PATH} ==="

    ssh "${TARGET_HOST}" "mkdir -p '${REMOTE_PATH}'"

    DEPLOY_STAGING="$(mktemp -d)"
    trap 'rm -rf "$DEPLOY_STAGING"' EXIT

    cp "$WHEEL_FILE" "$DEPLOY_STAGING/"
    mkdir -p "$DEPLOY_STAGING/benches/python"
    cp "${SCRIPT_DIR}/conftest.py"          "$DEPLOY_STAGING/benches/python/"
    cp "${SCRIPT_DIR}/shapes.py"            "$DEPLOY_STAGING/benches/python/"
    cp "${SCRIPT_DIR}/bench_native.py"      "$DEPLOY_STAGING/benches/python/"
    cp "${SCRIPT_DIR}/bench_cyclonedds.py"  "$DEPLOY_STAGING/benches/python/"
    cp "${SCRIPT_DIR}/bench_pycdr2.py"      "$DEPLOY_STAGING/benches/python/"
    rsync -a "${SCRIPT_DIR}/legacy/" "$DEPLOY_STAGING/benches/python/legacy/"

    rsync -az --delete "$DEPLOY_STAGING/" "${TARGET_HOST}:${REMOTE_PATH}/"

    echo ""
    echo "    Setting up target-side venv ..."
    # Use the user-specified python interpreter on the target. Create a venv
    # if needed and install the wheel + bench dependencies. Each impl's
    # dependency is installed separately so a single failure (e.g. missing
    # cyclonedds C++ library) doesn't block the others — a missing impl
    # later shows up as "skipped" rather than aborting the whole run.
    REMOTE_WHEEL="$(basename "$WHEEL_FILE")"
    ssh "${TARGET_HOST}" \
        "CYCLONEDDS_HOME_REMOTE='${CYCLONEDDS_HOME_REMOTE:-}' \
         REMOTE_PATH='${REMOTE_PATH}' \
         REMOTE_WHEEL='${REMOTE_WHEEL}' \
         PYTHON_INTERPRETER='${PYTHON_INTERPRETER}' \
         bash -s" <<'EOF'
set -uo pipefail
cd "${REMOTE_PATH}"
if [[ ! -d venv ]]; then
    ${PYTHON_INTERPRETER} -m venv venv
fi
. venv/bin/activate
pip install --quiet --upgrade pip
pip install --quiet pytest pytest-benchmark numpy
pip install --quiet --force-reinstall "./${REMOTE_WHEEL}"
# Optional impls — failures are non-fatal so the rest of the run still proceeds.
pip install --quiet pycdr2==1.0.0 || echo "    (warning) pycdr2 install failed — skipping bench_pycdr2"

# cyclonedds-python's setup.py searches CYCLONEDDS_HOME expecting a layout
# of <home>/{include, bin, lib/libddsc.so}. Two install paths:
#
#   1. CYCLONEDDS_HOME_REMOTE was passed from the host — use it as-is.
#      Typical: a from-source build at ~/.local/cdds-0.10.2 whose ABI
#      matches a published cyclonedds-python release.
#
#   2. Otherwise: build a symlink farm at $REMOTE_PATH/cdds-prefix that
#      flattens Debian's multiarch layout (lib/aarch64-linux-gnu/) into
#      the flat lib/ that the python setup.py probe recognises.
#
# Either way, cyclonedds-python is pinned to 0.10.2 because its C extension
# references the legacy q_radmin.h header (renamed to ddsi_radmin.h in
# 0.10.4+, which Debian's apt package strips as internal).
EFFECTIVE_HOME=""
if [[ -n "${CYCLONEDDS_HOME_REMOTE:-}" && -f "${CYCLONEDDS_HOME_REMOTE}/lib/libddsc.so" ]]; then
    EFFECTIVE_HOME="${CYCLONEDDS_HOME_REMOTE}"
    echo "    Using user-provided CYCLONEDDS_HOME=${EFFECTIVE_HOME}"
else
    CDDS_DEB_LIB=""
    for cand in /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/lib64 /usr/lib; do
        if [[ -f "${cand}/libddsc.so" ]]; then
            CDDS_DEB_LIB="${cand}"
            break
        fi
    done
    if [[ -n "${CDDS_DEB_LIB}" && -d /usr/include/dds ]]; then
        CDDS_PREFIX="${REMOTE_PATH}/cdds-prefix"
        rm -rf "${CDDS_PREFIX}"
        mkdir -p "${CDDS_PREFIX}/lib" "${CDDS_PREFIX}/bin"
        ln -sfn /usr/include "${CDDS_PREFIX}/include"
        for lib in libddsc.so libcycloneddsidl.so; do
            if [[ -f "${CDDS_DEB_LIB}/${lib}" ]]; then
                ln -sfn "${CDDS_DEB_LIB}/${lib}" "${CDDS_PREFIX}/lib/${lib}"
            fi
        done
        for tool in idlc cyclonedds; do
            if [[ -x "/usr/bin/${tool}" ]]; then
                ln -sfn "/usr/bin/${tool}" "${CDDS_PREFIX}/bin/${tool}"
            fi
        done
        EFFECTIVE_HOME="${CDDS_PREFIX}"
    fi
fi

if [[ -n "${EFFECTIVE_HOME}" ]]; then
    # cyclonedds-python 0.10.2 calls the private CPython symbol
    # _Py_IsFinalizing, which Python 3.13 removed in favour of the public
    # Py_IsFinalizing. Download the sdist, patch the one call site, then
    # install from the patched source. The const-discard warnings on
    # ddspy_typeid_ser are harmless (warning, not error in mainline GCC).
    CDDS_PY_SRC="${REMOTE_PATH}/cyclonedds-py-src"
    rm -rf "${CDDS_PY_SRC}"
    mkdir -p "${CDDS_PY_SRC}"
    # Fetch sdist directly from PyPI — avoids pip's metadata-prep step,
    # which fails because cyclonedds-python's setup.py runs find_cyclonedds()
    # at metadata time and trips the same compile-side const-discard / Py3.13
    # symbol issues we're patching around.
    SDIST="${CDDS_PY_SRC}/cyclonedds-0.10.2.tar.gz"
    curl -sL -o "${SDIST}" \
        "https://files.pythonhosted.org/packages/source/c/cyclonedds/cyclonedds-0.10.2.tar.gz" \
        || rm -f "${SDIST}"
    if [[ -n "${SDIST}" ]]; then
        tar -xzf "${SDIST}" -C "${CDDS_PY_SRC}"
        SRC_DIR="$(ls -d "${CDDS_PY_SRC}"/cyclonedds-*/ | head -1)"
        # Py 3.13 public-name fix.
        sed -i 's/_Py_IsFinalizing/Py_IsFinalizing/g' "${SRC_DIR}/clayer/pysertype.c"
        CYCLONEDDS_HOME="${EFFECTIVE_HOME}" \
        LD_LIBRARY_PATH="${EFFECTIVE_HOME}/lib:${LD_LIBRARY_PATH:-}" \
            pip install --quiet "${SRC_DIR}" \
            || echo "    (warning) cyclonedds install failed — skipping bench_cyclonedds"
    else
        echo "    (warning) could not download cyclonedds 0.10.2 sdist — skipping bench_cyclonedds"
    fi
    # Persist LD_LIBRARY_PATH for the bench-run step by appending to the
    # venv's activate script so `import cyclonedds` finds libddsc.so.
    grep -q "edgefirst-bench cyclonedds env" venv/bin/activate || \
    cat >> venv/bin/activate <<ACTIVATE_END
# edgefirst-bench cyclonedds env
export LD_LIBRARY_PATH="${EFFECTIVE_HOME}/lib:\${LD_LIBRARY_PATH:-}"
ACTIVATE_END
else
    echo "    (warning) Cyclone DDS not found — skipping bench_cyclonedds"
    echo "    (warning) Install via 'sudo apt-get install cyclonedds-dev' OR build"
    echo "    (warning) from source and pass CYCLONEDDS_HOME_REMOTE=<prefix> to this script"
fi
EOF
fi

# ---------------------------------------------------------------------------
# Phase: RUN
# ---------------------------------------------------------------------------
echo ""
echo "=== [3/4] Run benchmarks on ${TARGET_HOST} ==="

mkdir -p "${OUTPUT_DIR}"

SKIPPED_IMPLS=()
for impl in "${IMPLS[@]}"; do
    echo ""
    echo "    Running bench_${impl} ..."

    # Sanity-check: confirm the impl's import works before running its bench.
    # bench_pycdr2 needs `pycdr2`, bench_cyclonedds needs `cyclonedds`,
    # bench_native needs the wheel itself (which is the primary dep).
    case "$impl" in
        pycdr2)     IMPORT_TEST="import pycdr2" ;;
        cyclonedds) IMPORT_TEST="import cyclonedds.idl" ;;
        native)     IMPORT_TEST="import edgefirst.schemas" ;;
    esac
    if ! ssh "${TARGET_HOST}" \
        "cd ${REMOTE_PATH} && . venv/bin/activate && python -c '${IMPORT_TEST}'" \
        >/dev/null 2>&1; then
        echo "    (skipped) ${impl} dependency not importable on target"
        SKIPPED_IMPLS+=("$impl")
        continue
    fi

    REMOTE_JSON="${REMOTE_PATH}/${impl}.json"
    BENCH_ARGS=(
        "benches/python/bench_${impl}.py"
        "--benchmark-only"
        "--benchmark-json=${REMOTE_JSON}"
        "--benchmark-warmup=off"
        "-q"
    )
    [[ -n "$FILTER" ]] && BENCH_ARGS+=("-k" "$FILTER")

    BENCH_ARGS_Q=""
    for arg in "${BENCH_ARGS[@]}"; do
        BENCH_ARGS_Q+=" $(printf '%q' "$arg")"
    done
    REMOTE_PATH_Q="$(printf '%q' "${REMOTE_PATH}")"
    FAST_PREFIX=""
    if [[ $FAST -eq 1 ]]; then
        FAST_PREFIX="BENCH_FAST=1 "
    fi

    # shellcheck disable=SC2029
    ssh "${TARGET_HOST}" \
        "cd ${REMOTE_PATH_Q} && . venv/bin/activate && ${FAST_PREFIX}python -m pytest${BENCH_ARGS_Q} > /dev/null"

    echo "    Pulling ${impl}.json ..."
    rsync -az "${TARGET_HOST}:${REMOTE_JSON}" "${OUTPUT_DIR}/${impl}.json"
done

if [[ ${#SKIPPED_IMPLS[@]} -gt 0 ]]; then
    echo ""
    echo "    Skipped impls: ${SKIPPED_IMPLS[*]}"
fi

# ---------------------------------------------------------------------------
# Collect system metadata
# ---------------------------------------------------------------------------
echo ""
echo "    Collecting system metadata ..."

UNAME_STR="$(ssh "${TARGET_HOST}" 'uname -a' 2>/dev/null || echo 'unavailable')"
CPUINFO_STR="$(ssh "${TARGET_HOST}" 'head -30 /proc/cpuinfo' 2>/dev/null || echo 'unavailable')"
PY_VERSION="$(ssh "${TARGET_HOST}" "${REMOTE_PATH}/venv/bin/python --version" 2>/dev/null || echo 'unavailable')"
GIT_REV="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo 'unavailable')"

SYSTEM_JSON="${OUTPUT_DIR}/system.json"
BENCH_TIMESTAMP="$TIMESTAMP" \
BENCH_GIT_REV="$GIT_REV" \
BENCH_TARGET_H="$TARGET_HOST" \
BENCH_PY_VERSION="$PY_VERSION" \
BENCH_UNAME="$UNAME_STR" \
BENCH_CPUINFO="$CPUINFO_STR" \
BENCH_OUT="$SYSTEM_JSON" \
python3 - <<'PYEOF'
import json, os
data = {
    "timestamp":      os.environ["BENCH_TIMESTAMP"],
    "git_rev":        os.environ["BENCH_GIT_REV"],
    "target_host":    os.environ["BENCH_TARGET_H"],
    "python_version": os.environ["BENCH_PY_VERSION"],
    "uname":          os.environ["BENCH_UNAME"],
    "cpuinfo_head30": os.environ["BENCH_CPUINFO"],
}
with open(os.environ["BENCH_OUT"], "w") as f:
    json.dump(data, f, indent=2)
print(f"    Wrote {os.environ['BENCH_OUT']}")
PYEOF

# ---------------------------------------------------------------------------
# Phase: RENDER (optional)
# ---------------------------------------------------------------------------
if [[ $RENDER -eq 1 ]]; then
    echo ""
    echo "=== [4/4] Render report ==="
    RENDERER="${REPO_ROOT}/scripts/render_python_benchmarks.py"
    REPORT="${OUTPUT_DIR}/BENCHMARKS_PYTHON-${TIMESTAMP}.md"
    python3 "${RENDERER}" "${OUTPUT_DIR}" > "${REPORT}"
    echo "    Report: ${REPORT}"
fi

echo ""
echo "=== Done. Results in: ${OUTPUT_DIR} ==="
