#!/usr/bin/env bash
# Regenerate Fast-CDR C++ types from idl/ via fastddsgen.
# Requires $FASTDDS_PATH pointing at the Fast-DDS-Gen install dir.
set -euo pipefail

if [[ -z "${FASTDDS_PATH:-}" ]]; then
    echo "ERROR: FASTDDS_PATH env var not set." >&2
    echo "Set it to your Fast-DDS-Gen install dir, e.g.:" >&2
    echo "  export FASTDDS_PATH=\$HOME/Software/ROS2/Fast-DDS-Gen" >&2
    exit 1
fi

FASTDDSGEN="$FASTDDS_PATH/scripts/fastddsgen"
if [[ ! -x "$FASTDDSGEN" ]]; then
    echo "ERROR: fastddsgen not found or not executable at $FASTDDSGEN" >&2
    exit 1
fi

cd "$(dirname "$0")/.."

OUT=types/fastcdr
rm -rf "$OUT"
mkdir -p "$OUT"

# Single output root so transitive includes (e.g., std_msgs/Header used by
# sensor_msgs/Image) deduplicate. fastddsgen places generated files under
# $OUT/<module-path>/<Type>.{hpp,cxx,...} based on each IDL's module declaration.
# All IDLs are passed in one invocation so the tool can deduplicate internally.
mapfile -t IDLS < <(find idl -name '*.idl' | sort)
"$FASTDDSGEN" -d "$OUT" -replace -I idl "${IDLS[@]}"

echo "Generated types under $OUT/"
