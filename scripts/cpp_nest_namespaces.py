#!/usr/bin/env python3
"""Nest C++ wrapper types under edgefirst::schemas::<package> (range-based, safe).

Does not rewrite the whole file via a stream walker. Instead:
1. Locate each top-level class/struct range (with leading doc comment)
2. Skip anything inside namespace detail { ... }
3. Wrap ranges whose names are in TYPE_PACKAGE; leave Error/Released/etc. alone
4. Rename Foxglove*/Mavros* public type names inside those wrapped blocks

Usage:
  python3 scripts/cpp_nest_namespaces.py --dry-run
  python3 scripts/cpp_nest_namespaces.py --apply
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
HPP = REPO / "crates/capi/include/edgefirst/schemas.hpp"

TYPE_PACKAGE: dict[str, str] = {}


def _add(pkg: str, *names: str) -> None:
    for n in names:
        TYPE_PACKAGE[n] = pkg


_add("builtin_interfaces", "Time", "Duration")
_add(
    "geometry_msgs",
    "Vector3",
    "Point",
    "Point32",
    "Quaternion",
    "Pose",
    "Transform",
    "Twist",
    "PoseWithCovariance",
    "TwistWithCovariance",
    "Accel",
    "Wrench",
    "AccelWithCovariance",
    "AccelStampedView",
    "AccelStampedBuilder",
    "TwistStampedView",
    "TwistStampedBuilder",
    "WrenchStampedView",
    "WrenchStampedBuilder",
    "PointStampedView",
    "PointStampedBuilder",
    "InertiaStampedView",
    "InertiaStampedBuilder",
    "Vector3StampedView",
    "Vector3StampedBuilder",
    "PoseStampedView",
    "PoseStampedBuilder",
    "QuaternionStampedView",
    "QuaternionStampedBuilder",
    "PoseWithCovarianceStampedView",
    "PoseWithCovarianceStampedBuilder",
    "TwistWithCovarianceStampedView",
    "TwistWithCovarianceStampedBuilder",
    "AccelWithCovarianceStampedView",
    "AccelWithCovarianceStampedBuilder",
    "PolygonView",
    "PolygonBuilder",
    "PolygonStampedView",
    "PolygonStampedBuilder",
    "PoseArrayView",
    "PoseArrayBuilder",
    "TransformStampedView",
    "TransformStampedBuilder",
)
_add(
    "sensor_msgs",
    "NavSatStatus",
    "CompressedImage",
    "CompressedImageView",
    "CompressedImageBuilder",
    "Image",
    "ImageView",
    "ImageBuilder",
    "ImuView",
    "ImuBuilder",
    "NavSatFixView",
    "NavSatFixBuilder",
    "CameraInfoView",
    "CameraInfoBuilder",
    "PointCloud2View",
    "PointCloud2Builder",
    "PointFieldBuilder",
    "MagneticFieldView",
    "MagneticFieldBuilder",
    "FluidPressureView",
    "FluidPressureBuilder",
    "TemperatureView",
    "TemperatureBuilder",
    "BatteryStateView",
    "BatteryStateBuilder",
    "RelativeHumidityView",
    "RelativeHumidityBuilder",
    "TimeReferenceView",
    "TimeReferenceBuilder",
)
_add(
    "nav_msgs",
    "MapMetaData",
    "OdometryView",
    "OdometryBuilder",
    "GridCellsView",
    "GridCellsBuilder",
    "OccupancyGridView",
    "OccupancyGridBuilder",
    "PathView",
    "PathBuilder",
)
_add("std_msgs", "Header", "HeaderView", "HeaderBuilder")
_add(
    "foxglove_msgs",
    "FoxgloveCompressedImage",
    "FoxgloveCompressedImageView",
    "FoxgloveCompressedImageBuilder",
    "CompressedVideo",
    "CompressedVideoView",
    "FoxgloveCompressedVideoBuilder",
    "FoxgloveTextAnnotationBuilder",
    "FoxglovePointAnnotationBuilder",
    "FoxgloveImageAnnotationBuilder",
)
_add(
    "mavros_msgs",
    "MavrosAltitudeView",
    "MavrosAltitudeBuilder",
    "MavrosVfrHudView",
    "MavrosVfrHudBuilder",
    "MavrosEstimatorStatusView",
    "MavrosEstimatorStatusBuilder",
    "MavrosExtendedStateView",
    "MavrosExtendedStateBuilder",
    "MavrosSysStatusView",
    "MavrosSysStatusBuilder",
    "MavrosStateView",
    "MavrosStateBuilder",
    "MavrosStatusTextView",
    "MavrosStatusTextBuilder",
    "MavrosGpsRawView",
    "MavrosGpsRawBuilder",
    "MavrosTimesyncStatusView",
    "MavrosTimesyncStatusBuilder",
)
_add(
    "edgefirst_msgs",
    "Mask",
    "MaskView",
    "MaskBuilder",
    "LocalTimeView",
    "LocalTimeBuilder",
    "TrackView",
    "TrackBuilder",
    "BoxView",
    "DetectView",
    "DetectBuilder",
    "DetectBoxBuilder",
    "ModelView",
    "ModelBuilder",
    "ModelInfoView",
    "ModelInfoBuilder",
    "RadarCubeView",
    "RadarCubeBuilder",
    "RadarInfoView",
    "RadarInfoBuilder",
    "VibrationView",
    "VibrationBuilder",
    "TensorView",
    "TensorBuilder",
    "TensorStampedView",
    "TensorStampedBuilder",
    "CameraFrameView",
    "CameraFrameBuilder",
    # Borrowed*/ChildRange/TensorAccessors stay in detail::
)

PUBLIC_RENAME = {
    "FoxgloveCompressedImage": "CompressedImage",
    "FoxgloveCompressedImageView": "CompressedImageView",
    "FoxgloveCompressedImageBuilder": "CompressedImageBuilder",
    "FoxgloveCompressedVideoBuilder": "CompressedVideoBuilder",
    "FoxgloveTextAnnotationBuilder": "TextAnnotationBuilder",
    "FoxglovePointAnnotationBuilder": "PointAnnotationBuilder",
    "FoxgloveImageAnnotationBuilder": "ImageAnnotationBuilder",
    "MavrosAltitudeView": "AltitudeView",
    "MavrosAltitudeBuilder": "AltitudeBuilder",
    "MavrosVfrHudView": "VfrHudView",
    "MavrosVfrHudBuilder": "VfrHudBuilder",
    "MavrosEstimatorStatusView": "EstimatorStatusView",
    "MavrosEstimatorStatusBuilder": "EstimatorStatusBuilder",
    "MavrosExtendedStateView": "ExtendedStateView",
    "MavrosExtendedStateBuilder": "ExtendedStateBuilder",
    "MavrosSysStatusView": "SysStatusView",
    "MavrosSysStatusBuilder": "SysStatusBuilder",
    "MavrosStateView": "StateView",
    "MavrosStateBuilder": "StateBuilder",
    "MavrosStatusTextView": "StatusTextView",
    "MavrosStatusTextBuilder": "StatusTextBuilder",
    "MavrosGpsRawView": "GpsRawView",
    "MavrosGpsRawBuilder": "GpsRawBuilder",
    "MavrosTimesyncStatusView": "TimesyncStatusView",
    "MavrosTimesyncStatusBuilder": "TimesyncStatusBuilder",
}

# Injected once per package the first time it is opened.
PACKAGE_USINGS: dict[str, list[str]] = {
    "geometry_msgs": [
        "using ::edgefirst::schemas::builtin_interfaces::Time;",
        "using ::edgefirst::schemas::builtin_interfaces::Duration;",
        "using ::edgefirst::schemas::Error;",
        "using ::edgefirst::schemas::span;",
        "using ::edgefirst::schemas::expected;",
        "using ::edgefirst::schemas::unexpected;",
    ],
    "std_msgs": [
        "using ::edgefirst::schemas::builtin_interfaces::Time;",
        "using ::edgefirst::schemas::Error;",
        "using ::edgefirst::schemas::Released;",
        "using ::edgefirst::schemas::span;",
        "using ::edgefirst::schemas::expected;",
        "using ::edgefirst::schemas::unexpected;",
    ],
    "sensor_msgs": [
        "using ::edgefirst::schemas::builtin_interfaces::Time;",
        "using ::edgefirst::schemas::geometry_msgs::Vector3;",
        "using ::edgefirst::schemas::geometry_msgs::Quaternion;",
        "using ::edgefirst::schemas::Error;",
        "using ::edgefirst::schemas::Released;",
        "using ::edgefirst::schemas::span;",
        "using ::edgefirst::schemas::expected;",
        "using ::edgefirst::schemas::unexpected;",
    ],
    "nav_msgs": [
        "using ::edgefirst::schemas::builtin_interfaces::Time;",
        "using ::edgefirst::schemas::geometry_msgs::Point;",
        "using ::edgefirst::schemas::geometry_msgs::Pose;",
        "using ::edgefirst::schemas::geometry_msgs::Twist;",
        "using ::edgefirst::schemas::geometry_msgs::PoseWithCovariance;",
        "using ::edgefirst::schemas::geometry_msgs::TwistWithCovariance;",
        "using ::edgefirst::schemas::Error;",
        "using ::edgefirst::schemas::Released;",
        "using ::edgefirst::schemas::span;",
        "using ::edgefirst::schemas::expected;",
        "using ::edgefirst::schemas::unexpected;",
    ],
    "foxglove_msgs": [
        "using ::edgefirst::schemas::builtin_interfaces::Time;",
        "using ::edgefirst::schemas::Error;",
        "using ::edgefirst::schemas::Released;",
        "using ::edgefirst::schemas::span;",
        "using ::edgefirst::schemas::expected;",
        "using ::edgefirst::schemas::unexpected;",
    ],
    "mavros_msgs": [
        "using ::edgefirst::schemas::builtin_interfaces::Time;",
        "using ::edgefirst::schemas::Error;",
        "using ::edgefirst::schemas::Released;",
        "using ::edgefirst::schemas::span;",
        "using ::edgefirst::schemas::expected;",
        "using ::edgefirst::schemas::unexpected;",
    ],
    "edgefirst_msgs": [
        "using ::edgefirst::schemas::builtin_interfaces::Time;",
        "using ::edgefirst::schemas::geometry_msgs::Vector3;",
        "using ::edgefirst::schemas::Error;",
        "using ::edgefirst::schemas::Released;",
        "using ::edgefirst::schemas::span;",
        "using ::edgefirst::schemas::expected;",
        "using ::edgefirst::schemas::unexpected;",
    ],
}


def find_matching_brace(text: str, open_idx: int) -> int:
    depth = 0
    i = open_idx
    in_str = False
    str_ch = ""
    in_line_comment = False
    in_block_comment = False
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""
        if in_line_comment:
            if ch == "\n":
                in_line_comment = False
            i += 1
            continue
        if in_block_comment:
            if ch == "*" and nxt == "/":
                in_block_comment = False
                i += 2
                continue
            i += 1
            continue
        if in_str:
            if ch == "\\" and nxt:
                i += 2
                continue
            if ch == str_ch:
                in_str = False
            i += 1
            continue
        if ch == "/" and nxt == "/":
            in_line_comment = True
            i += 2
            continue
        if ch == "/" and nxt == "*":
            in_block_comment = True
            i += 2
            continue
        if ch in ('"', "'"):
            in_str = True
            str_ch = ch
            i += 1
            continue
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    raise ValueError("unbalanced braces")


def detail_ranges(body: str) -> list[tuple[int, int]]:
    ranges: list[tuple[int, int]] = []
    for m in re.finditer(r"namespace\s+detail\s*\{", body):
        open_idx = m.end() - 1
        close = find_matching_brace(body, open_idx)
        ranges.append((m.start(), close + 1))
    return ranges


def in_ranges(pos: int, ranges: list[tuple[int, int]]) -> bool:
    return any(a <= pos < b for a, b in ranges)


def leading_doc_start(text: str, decl_start: int) -> int:
    """Return start index of contiguous /// or /** doc immediately before decl."""
    # Skip blank/whitespace-only lines upward for ///
    pos = decl_start
    while pos > 0 and text[pos - 1] in " \t":
        pos -= 1
    if pos > 0 and text[pos - 1] == "\n":
        pos -= 1

    # Block comment /**
    k = pos
    while k > 0 and text[k - 1] in " \t\n":
        k -= 1
    if k >= 2 and text[k - 2 : k] == "*/":
        block_start = text.rfind("/**", 0, k)
        if block_start != -1:
            return block_start

    # /// lines
    lines_start = decl_start
    cur = decl_start
    while cur > 0:
        ls = text.rfind("\n", 0, cur) + 1
        line = text[ls:cur]
        stripped = line.strip()
        if stripped.startswith("///"):
            lines_start = ls
            cur = ls - 1 if ls > 0 else 0
            continue
        if stripped == "":
            cur = ls - 1 if ls > 0 else 0
            # don't include blank line in doc; stop
            break
        break
    return lines_start if lines_start < decl_start else decl_start


def rewrite_names(block: str) -> str:
    # Longest old names first
    for old in sorted(PUBLIC_RENAME.keys(), key=len, reverse=True):
        new = PUBLIC_RENAME[old]
        block = re.sub(rf"\b{re.escape(old)}\b", new, block)
    return block


def transform(text: str) -> str:
    ns_marker = "namespace edgefirst::schemas {"
    ns_start = text.find(ns_marker)
    if ns_start < 0:
        raise SystemExit("namespace edgefirst::schemas not found")
    body_start = ns_start + len(ns_marker)
    close_marker = "} // namespace edgefirst::schemas"
    ns_close = text.rfind(close_marker)
    if ns_close < 0:
        raise SystemExit("closing namespace marker not found")

    header = text[:body_start]
    body = text[body_start:ns_close]
    footer = text[ns_close:]

    d_ranges = detail_ranges(body)

    # Find top-level class/struct declarations (not in detail).
    decl_re = re.compile(r"(?m)^(class|struct)\s+([A-Za-z_][A-Za-z0-9_]*)\b")
    items: list[tuple[int, int, str, str]] = []  # start, end, name, pkg

    for m in decl_re.finditer(body):
        if in_ranges(m.start(), d_ranges):
            continue
        name = m.group(2)
        if name not in TYPE_PACKAGE:
            continue
        start = leading_doc_start(body, m.start())
        brace = body.find("{", m.start())
        if brace < 0:
            raise SystemExit(f"no opening brace for {name}")
        close = find_matching_brace(body, brace)
        end = close + 1
        if end < len(body) and body[end] == ";":
            end += 1
        if end < len(body) and body[end] == "\n":
            end += 1
        items.append((start, end, name, TYPE_PACKAGE[name]))

    items.sort(key=lambda x: x[0])

    # Ensure no overlaps
    for a, b in zip(items, items[1:]):
        if a[1] > b[0]:
            raise SystemExit(f"overlapping ranges: {a[2]} and {b[2]}")

    missing = set(TYPE_PACKAGE) - {n for _, _, n, _ in items}
    # Point32 is struct - covered. ChildRange etc. must be found.
    if missing:
        print(f"WARNING: types not found in hpp: {sorted(missing)}", file=sys.stderr)

    out: list[str] = [
        "\n\n// Public message types live in nested package namespaces "
        "(builtin_interfaces, std_msgs, …).\n"
        "// Shared utilities (Error, Released, detail::) remain in edgefirst::schemas.\n"
    ]
    pos = 0
    current_pkg: str | None = None
    usings_done: set[str] = set()

    def close_pkg() -> None:
        nonlocal current_pkg
        if current_pkg is not None:
            out.append(f"}} // namespace {current_pkg}\n")
            current_pkg = None

    for start, end, name, pkg in items:
        # Copy gap (may need to close package first)
        if start > pos:
            close_pkg()
            out.append(body[pos:start])
        if current_pkg != pkg:
            close_pkg()
            out.append(f"\nnamespace {pkg} {{\n")
            if pkg not in usings_done:
                usings_done.add(pkg)
                for u in PACKAGE_USINGS.get(pkg, []):
                    out.append(f"{u}\n")
                if pkg in PACKAGE_USINGS:
                    out.append("\n")
            current_pkg = pkg
        block = rewrite_names(body[start:end])
        out.append(block)
        pos = end

    close_pkg()
    out.append(body[pos:])
    result = header + "".join(out) + footer

    # detail:: helpers (BorrowedBoxView, etc.) need foundation types in scope.
    detail_inject = (
        "namespace detail {\n"
        "using ::edgefirst::schemas::builtin_interfaces::Time;\n"
        "using ::edgefirst::schemas::geometry_msgs::Point;\n"
        "using ::edgefirst::schemas::geometry_msgs::Vector3;\n"
        "using ::edgefirst::schemas::Error;\n"
        "using ::edgefirst::schemas::span;\n"
        "using ::edgefirst::schemas::expected;\n"
        "using ::edgefirst::schemas::unexpected;\n"
        "\n"
    )
    needle = "namespace detail {\n"
    idx = 0
    while True:
        i = result.find(needle, idx)
        if i < 0:
            break
        window = result[i : i + 5000]
        if "ViewBase" in window or "ChildRange" in window:
            if "using ::edgefirst::schemas::builtin_interfaces::Time;" not in window[:400]:
                result = result[:i] + detail_inject + result[i + len(needle) :]
            break
        idx = i + len(needle)

    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    if not args.dry_run and not args.apply:
        parser.error("specify --dry-run or --apply")

    text = HPP.read_text(encoding="utf-8")
    if "namespace builtin_interfaces {" in text and "struct Error {" in text:
        # Detect prior successful nest
        if "class Time {" in text[text.find("namespace builtin_interfaces") :]:
            print("schemas.hpp already nested; nothing to do")
            return

    new_text = transform(text)
    print(f"Old size: {len(text)} New size: {len(new_text)}")
    checks = {
        "struct Error": "struct Error {" in new_text,
        "struct Released": "struct Released {" in new_text,
        "namespace builtin_interfaces": "namespace builtin_interfaces {" in new_text,
        "namespace sensor_msgs": "namespace sensor_msgs {" in new_text,
        "namespace foxglove_msgs": "namespace foxglove_msgs {" in new_text,
        "namespace mavros_msgs": "namespace mavros_msgs {" in new_text,
        "class AltitudeView": "class AltitudeView" in new_text,
        "no MavrosAltitudeView": "class MavrosAltitudeView" not in new_text,
        "no FoxgloveCompressedImageView class": "class FoxgloveCompressedImageView"
        not in new_text,
    }
    for label, ok in checks.items():
        print(("OK" if ok else "FAIL"), label)
        if not ok:
            raise SystemExit(1)

    if args.apply:
        HPP.write_text(new_text, encoding="utf-8")
        print(f"Wrote {HPP}")
    else:
        print("Dry-run only")


if __name__ == "__main__":
    main()
