#!/usr/bin/env bash
# Regenerate Cyclone DDS C++ types from idl/ via idlc with the CXX plugin.
#
# Requires a local Cyclone DDS build (v0.10.5) that includes the idlc compiler
# and the CXX language plugin (libcycloneddsidlcxx.so).
#
# Usage:
#   CYCLONE_INSTALL=/path/to/cyclonedds/install \
#   ./scripts/regen_cyclonedds_types.sh
#
# The default value of CYCLONE_INSTALL is /tmp/cyclone-gen/install which is
# produced by the build steps in docs/cyclonedds_type_gen.md.
set -euo pipefail

CYCLONE_INSTALL="${CYCLONE_INSTALL:-/tmp/cyclone-gen/install}"

IDLC="$CYCLONE_INSTALL/bin/idlc"
IDLCXX_PLUGIN="$CYCLONE_INSTALL/lib/libcycloneddsidlcxx.so"

if [[ ! -x "$IDLC" ]]; then
    echo "ERROR: idlc not found or not executable at $IDLC" >&2
    echo "Build Cyclone DDS 0.10.5 from source or set CYCLONE_INSTALL." >&2
    exit 1
fi

if [[ ! -f "$IDLCXX_PLUGIN" ]]; then
    echo "ERROR: CXX plugin not found at $IDLCXX_PLUGIN" >&2
    echo "Build cyclonedds-cxx 0.10.5 from source or set CYCLONE_INSTALL." >&2
    exit 1
fi

cd "$(dirname "$0")/.."

OUT=types/cyclonedds
rm -rf "$OUT"

mapfile -t IDLS < <(find idl -name '*.idl' | sort)

for IDL in "${IDLS[@]}"; do
    # Derive the output directory from the IDL path:
    #   idl/std_msgs/Header.idl -> types/cyclonedds/std_msgs/
    #   idl/builtin_interfaces/Time.idl -> types/cyclonedds/builtin_interfaces/
    MODULE_DIR=$(dirname "${IDL#idl/}")
    DEST="$OUT/$MODULE_DIR"
    mkdir -p "$DEST"

    # -l <plugin>     : use C++ language plugin
    # -f case-sensitive : avoid collision between 'Mask' struct and 'mask' field
    # -I idl           : search path for transitive IDL includes
    "$IDLC" -l "$IDLCXX_PLUGIN" -f case-sensitive -I idl -o "$DEST" "$IDL"
done

echo "Generated types under $OUT/"
