#!/usr/bin/env bash
# Compatibility wrapper — the C API prefix rename is driven by
# scripts/c_prefix_rename_map.py (package-name prefixes per the 4.0 design).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec python3 "$ROOT/scripts/c_prefix_rename_map.py" "$@"
