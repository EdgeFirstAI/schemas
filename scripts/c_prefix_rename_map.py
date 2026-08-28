#!/usr/bin/env python3
"""Ordered C API prefix rename: ros_* → ROS package-name prefixes.

See docs/superpowers/specs/2026-08-26-schemas-4.0-completion-and-c-prefix-design.md

Usage:
  python3 scripts/c_prefix_rename_map.py --dry-run
  python3 scripts/c_prefix_rename_map.py --apply
  python3 scripts/c_prefix_rename_map.py --self-test
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent

# Longest stems first so accel_with_covariance_stamped beats accel.
TYPE_TO_PACKAGE: dict[str, str] = {
    # builtin_interfaces
    "time": "builtin_interfaces",
    "duration": "builtin_interfaces",
    # std_msgs
    "header": "std_msgs",
    # sensor_msgs
    "compressed_image": "sensor_msgs",
    "point_cloud2": "sensor_msgs",
    "point_field": "sensor_msgs",
    "camera_info": "sensor_msgs",
    "nav_sat_fix": "sensor_msgs",
    "nav_sat_status": "sensor_msgs",
    "magnetic_field": "sensor_msgs",
    "fluid_pressure": "sensor_msgs",
    "relative_humidity": "sensor_msgs",
    "time_reference": "sensor_msgs",
    "battery_state": "sensor_msgs",
    "temperature": "sensor_msgs",
    "image": "sensor_msgs",
    "imu": "sensor_msgs",
    # geometry_msgs
    "accel_with_covariance_stamped": "geometry_msgs",
    "twist_with_covariance_stamped": "geometry_msgs",
    "pose_with_covariance_stamped": "geometry_msgs",
    "accel_with_covariance": "geometry_msgs",
    "twist_with_covariance": "geometry_msgs",
    "pose_with_covariance": "geometry_msgs",
    "quaternion_stamped": "geometry_msgs",
    "transform_stamped": "geometry_msgs",
    "vector3_stamped": "geometry_msgs",
    "inertia_stamped": "geometry_msgs",
    "wrench_stamped": "geometry_msgs",
    "accel_stamped": "geometry_msgs",
    "twist_stamped": "geometry_msgs",
    "point_stamped": "geometry_msgs",
    "polygon_stamped": "geometry_msgs",
    "pose_stamped": "geometry_msgs",
    "pose_array": "geometry_msgs",
    "quaternion": "geometry_msgs",
    "transform": "geometry_msgs",
    "vector3": "geometry_msgs",
    "polygon": "geometry_msgs",
    "wrench": "geometry_msgs",
    "accel": "geometry_msgs",
    "twist": "geometry_msgs",
    "point": "geometry_msgs",
    "pose": "geometry_msgs",
    # nav_msgs
    "map_meta_data": "nav_msgs",
    "occupancy_grid": "nav_msgs",
    "grid_cells": "nav_msgs",
    "path_iter": "nav_msgs",
    "odometry": "nav_msgs",
    "path": "nav_msgs",
    # foxglove_msgs — only compressed_video uses bare stem under ros_*
    # (foxglove_* stems handled by strip rule below)
    "compressed_video": "foxglove_msgs",
    # edgefirst_msgs
    "camera_frame": "edgefirst_msgs",
    "tensor_plane": "edgefirst_msgs",
    "tensor_stamped": "edgefirst_msgs",
    "detect_box": "edgefirst_msgs",
    "model_info": "edgefirst_msgs",
    "radar_cube": "edgefirst_msgs",
    "radar_info": "edgefirst_msgs",
    "local_time": "edgefirst_msgs",
    "vibration": "edgefirst_msgs",
    "detect": "edgefirst_msgs",
    "tensor": "edgefirst_msgs",
    "model": "edgefirst_msgs",
    "track": "edgefirst_msgs",
    "mask": "edgefirst_msgs",
    "box": "edgefirst_msgs",
}

# Sort stems longest-first for matching.
_STEMS_LONGEST = sorted(TYPE_TO_PACKAGE.keys(), key=len, reverse=True)

ROS_ID_RE = re.compile(r"\bros_[a-z0-9_]+\b")

DEFAULT_GLOBS = [
    "crates/capi/src/*.rs",
    "crates/capi/include/edgefirst/schemas.h",
    "crates/capi/include/edgefirst/schemas.hpp",
    "crates/capi/tests/**/*.c",
    "crates/capi/tests/**/*.cpp",
    "crates/capi/tests/**/*.h",
    "crates/capi/tests/**/*.hpp",
    "crates/capi/tests/**/*.rs",
    "examples/c/**/*.c",
    "examples/cpp/**/*.cpp",
    "benches/cpp/**/*.cpp",
    "benches/cpp/**/*.hpp",
    "benches/cpp/**/*.h",
    "CAPI.md",
    "ARCHITECTURE.md",
    "CONTRIBUTING.md",
    "README.md",
    "BENCHMARKS.md",
    "TESTING.md",
    ".github/copilot-instructions.md",
    "examples/**/*.md",
    "crates/capi/tests/c/README.md",
]


# Doc anti-patterns / bare stems that historically belonged under ros_mavros_*.
_MAVROS_BARE_STEMS = frozenset(
    {
        "altitude",
        "vfrhud",
        "estimator_status",
        "extended_state",
        "sys_status",
        "state",
        "status_text",
        "gps_raw",
        "timesync_status",
    }
)


def rename_identifier(name: str) -> str:
    """Map a single ros_* identifier to its package-prefixed form."""
    if not name.startswith("ros_"):
        return name
    rest = name[4:]

    # Library-level helpers (no message package).
    if rest == "bytes_free" or rest.startswith("bytes_"):
        return f"edgefirst_schemas_{rest}"

    # Stem cleanup: ros_mavros_<type>_* → mavros_msgs_<type>_*
    if rest.startswith("mavros_"):
        return f"mavros_msgs_{rest[len('mavros_'):]}"

    # Stem cleanup: ros_foxglove_<type>_* → foxglove_msgs_<type>_*
    if rest.startswith("foxglove_"):
        return f"foxglove_msgs_{rest[len('foxglove_'):]}"

    # Package lookup by longest type stem.
    for stem in _STEMS_LONGEST:
        if rest == stem or rest.startswith(stem + "_"):
            pkg = TYPE_TO_PACKAGE[stem]
            return f"{pkg}_{rest}"

    # Documentation wildcards like `ros_altitude_*` → token `ros_altitude_`.
    if rest.endswith("_"):
        bare = rest.rstrip("_")
        if bare in TYPE_TO_PACKAGE:
            return f"{TYPE_TO_PACKAGE[bare]}_{rest}"
        if bare in _MAVROS_BARE_STEMS:
            return f"mavros_msgs_{rest}"

    if rest in _MAVROS_BARE_STEMS:
        return f"mavros_msgs_{rest}"

    raise KeyError(f"unmapped ros_ identifier: {name}")


def rename_text(text: str) -> tuple[str, int, list[str]]:
    """Replace all ros_* identifiers. Returns (new_text, count, unmapped)."""
    unmapped: list[str] = []
    count = 0

    def repl(m: re.Match[str]) -> str:
        nonlocal count
        old = m.group(0)
        try:
            new = rename_identifier(old)
        except KeyError:
            unmapped.append(old)
            return old
        if new != old:
            count += 1
        return new

    return ROS_ID_RE.sub(repl, text), count, unmapped


def expand_globs(globs: list[str]) -> list[Path]:
    files: list[Path] = []
    seen: set[Path] = set()
    for pattern in globs:
        for path in sorted(REPO.glob(pattern)):
            if path.is_file() and path not in seen:
                seen.add(path)
                files.append(path)
    return files


def process_files(
    files: list[Path], *, apply: bool
) -> tuple[int, int, dict[str, list[str]]]:
    """Returns (files_changed, total_replacements, unmapped_by_file)."""
    files_changed = 0
    total = 0
    unmapped_by_file: dict[str, list[str]] = {}
    for path in files:
        text = path.read_text(encoding="utf-8")
        if "ros_" not in text:
            continue
        new_text, count, unmapped = rename_text(text)
        if unmapped:
            unmapped_by_file[str(path.relative_to(REPO))] = sorted(set(unmapped))
        if count == 0:
            continue
        total += count
        files_changed += 1
        rel = path.relative_to(REPO)
        print(f"  {rel}: {count} replacements")
        if apply:
            path.write_text(new_text, encoding="utf-8")
    return files_changed, total, unmapped_by_file


def self_test() -> None:
    cases = {
        "ros_image_from_cdr": "sensor_msgs_image_from_cdr",
        "ros_header_builder_new": "std_msgs_header_builder_new",
        "ros_time_encode": "builtin_interfaces_time_encode",
        "ros_detect_from_cdr": "edgefirst_msgs_detect_from_cdr",
        "ros_compressed_image_from_cdr": "sensor_msgs_compressed_image_from_cdr",
        "ros_foxglove_compressed_image_from_cdr": "foxglove_msgs_compressed_image_from_cdr",
        "ros_compressed_video_from_cdr": "foxglove_msgs_compressed_video_from_cdr",
        "ros_foxglove_compressed_video_builder_new": "foxglove_msgs_compressed_video_builder_new",
        "ros_mavros_altitude_from_cdr": "mavros_msgs_altitude_from_cdr",
        "ros_bytes_free": "edgefirst_schemas_bytes_free",
        "ros_odometry_builder_new": "nav_msgs_odometry_builder_new",
        "ros_box_from_cdr": "edgefirst_msgs_box_from_cdr",
        "ros_detect_box_builder_new": "edgefirst_msgs_detect_box_builder_new",
        "ros_foxglove_image_annotation_builder_new": "foxglove_msgs_image_annotation_builder_new",
        "ros_foxglove_circle_annotation_elem_t": "foxglove_msgs_circle_annotation_elem_t",
        "ros_accel_with_covariance_stamped_builder_new": (
            "geometry_msgs_accel_with_covariance_stamped_builder_new"
        ),
        "ros_path_iter_next": "nav_msgs_path_iter_next",
        "ros_tensor_copy_shape": "edgefirst_msgs_tensor_copy_shape",
    }
    failed = 0
    for old, expected in cases.items():
        got = rename_identifier(old)
        if got != expected:
            print(f"FAIL: {old} → {got!r} (expected {expected!r})")
            failed += 1
        else:
            print(f"ok:   {old} → {got}")

    # Must not double-prefix.
    doubled = rename_identifier("ros_foxglove_compressed_image_t")
    if doubled != "foxglove_msgs_compressed_image_t":
        print(f"FAIL: doubling check → {doubled!r}")
        failed += 1

    # Text with mixed identifiers.
    sample = "ros_image_from_cdr(buf); ros_bytes_free(p, n); ros_foxglove_compressed_image_t* v;"
    out, n, unmapped = rename_text(sample)
    assert not unmapped, unmapped
    assert "sensor_msgs_image_from_cdr" in out
    assert "edgefirst_schemas_bytes_free" in out
    assert "foxglove_msgs_compressed_image_t" in out
    assert "foxglove_msgs_foxglove_" not in out
    print(f"ok:   text rewrite ({n} replacements)")

    if failed:
        raise SystemExit(f"{failed} self-test failure(s)")
    print("All self-tests passed.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", help="Report only")
    parser.add_argument("--apply", action="store_true", help="Write changes")
    parser.add_argument("--self-test", action="store_true", help="Run unit checks")
    parser.add_argument(
        "--files",
        nargs="*",
        help="Optional explicit file paths (repo-relative or absolute)",
    )
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return

    if not args.dry_run and not args.apply:
        parser.error("specify --dry-run, --apply, or --self-test")

    if args.files:
        files = []
        for f in args.files:
            p = Path(f)
            if not p.is_absolute():
                p = REPO / p
            files.append(p)
    else:
        files = expand_globs(DEFAULT_GLOBS)

    print(f"Scanning {len(files)} files ({'APPLY' if args.apply else 'DRY-RUN'})...")
    changed, total, unmapped = process_files(files, apply=args.apply)
    print(f"Files with replacements: {changed}")
    print(f"Total replacements: {total}")

    if unmapped:
        print("UNMAPPED identifiers remain:", file=sys.stderr)
        for path, ids in sorted(unmapped.items()):
            print(f"  {path}:", file=sys.stderr)
            for i in ids:
                print(f"    {i}", file=sys.stderr)
        raise SystemExit(1)

    # After apply, refuse leftover ros_ in target globs (except CHANGELOG history).
    if args.apply:
        leftovers: list[str] = []
        for path in files:
            if path.name == "CHANGELOG.md":
                continue
            text = path.read_text(encoding="utf-8")
            found = sorted(set(ROS_ID_RE.findall(text)))
            if found:
                leftovers.append(f"{path.relative_to(REPO)}: {found[:5]}")
        if leftovers:
            print("Leftover ros_ after apply:", file=sys.stderr)
            for line in leftovers:
                print(f"  {line}", file=sys.stderr)
            raise SystemExit(1)
        print("Apply complete; no leftover ros_ in target files.")


if __name__ == "__main__":
    main()
