#!/usr/bin/env python3
"""Migrate message Type::new(...) calls to Type::builder()....build().unwrap()."""

from __future__ import annotations

import re
import sys
from pathlib import Path

# (regex for Type::new, builder method chain template using {0},{1},...)
# Args are split at top-level commas.
RULES: list[tuple[str, str]] = [
    (r"(?:std_msgs::)?Header::new", ".stamp({0}).frame_id({1})"),
    (r"(?:sensor_msgs::)?CompressedImage::new", ".stamp({0}).frame_id({1}).format({2}).data({3})"),
    (
        r"(?:sensor_msgs::)?Image::new",
        ".stamp({0}).frame_id({1}).height({2}).width({3}).encoding({4}).is_bigendian({5}).step({6}).data({7})",
    ),
    (
        r"(?:sensor_msgs::)?Imu::new",
        ".stamp({0}).frame_id({1}).orientation({2}).orientation_covariance({3}).angular_velocity({4}).angular_velocity_covariance({5}).linear_acceleration({6}).linear_acceleration_covariance({7})",
    ),
    (
        r"(?:sensor_msgs::)?NavSatFix::new",
        ".stamp({0}).frame_id({1}).status({2}).latitude({3}).longitude({4}).altitude({5}).position_covariance({6}).position_covariance_type({7})",
    ),
    (
        r"(?:sensor_msgs::)?PointField::new",
        ".name({0}).offset({1}).datatype({2}).count({3})",
    ),
    (
        r"(?:sensor_msgs::)?PointCloud2::new",
        ".stamp({0}).frame_id({1}).height({2}).width({3}).fields({4}).is_bigendian({5}).point_step({6}).row_step({7}).data({8}).is_dense({9})",
    ),
    (
        r"(?:sensor_msgs::)?CameraInfo::new",
        ".stamp({0}).frame_id({1}).height({2}).width({3}).distortion_model({4}).d({5}).k({6}).r({7}).p({8}).binning_x({9}).binning_y({10}).roi({11})",
    ),
    (
        r"(?:sensor_msgs::)?MagneticField::new",
        ".stamp({0}).frame_id({1}).magnetic_field({2}).magnetic_field_covariance({3})",
    ),
    (
        r"(?:sensor_msgs::)?FluidPressure::new",
        ".stamp({0}).frame_id({1}).fluid_pressure({2}).variance({3})",
    ),
    (
        r"(?:sensor_msgs::)?Temperature::new",
        ".stamp({0}).frame_id({1}).temperature({2}).variance({3})",
    ),
    (
        r"(?:sensor_msgs::)?BatteryState::new",
        ".stamp({0}).frame_id({1}).voltage({2}).current({3}).charge({4}).capacity({5}).design_capacity({6}).percentage({7}).power_supply_status({8}).power_supply_health({9}).power_supply_technology({10}).present({11}).cell_voltage({12}).cell_temperature({13}).location({14}).serial_number({15})",
    ),
    (
        r"(?:sensor_msgs::)?RelativeHumidity::new",
        ".stamp({0}).frame_id({1}).relative_humidity({2}).variance({3})",
    ),
    (
        r"(?:sensor_msgs::)?TimeReference::new",
        ".stamp({0}).frame_id({1}).time_ref({2}).source({3})",
    ),
    (
        r"(?:foxglove_msgs::)?FoxgloveCompressedVideo::new",
        ".stamp({0}).frame_id({1}).data({2}).format({3})",
    ),
    (
        r"(?:foxglove_msgs::)?FoxgloveCompressedImage::new",
        ".stamp({0}).frame_id({1}).data({2}).format({3})",
    ),
    (
        r"(?:edgefirst_msgs::)?Mask::new",
        ".height({0}).width({1}).length({2}).encoding({3}).mask({4}).boxed({5})",
    ),
    (
        r"(?:edgefirst_msgs::)?LocalTime::new",
        ".stamp({0}).frame_id({1}).date({2}).time({3}).timezone({4})",
    ),
    (
        r"(?:edgefirst_msgs::)?RadarCube::new",
        ".stamp({0}).frame_id({1}).timestamp({2}).layout({3}).shape({4}).scales({5}).cube({6}).is_complex({7})",
    ),
    (
        r"(?:edgefirst_msgs::)?RadarInfo::new",
        ".stamp({0}).frame_id({1}).center_frequency({2}).frequency_sweep({3}).range_toggle({4}).detection_sensitivity({5}).cube({6})",
    ),
    (
        r"(?:edgefirst_msgs::)?Track::new",
        ".id({0}).lifetime({1}).created({2})",
    ),
    (
        r"(?:edgefirst_msgs::)?DetectBox::new",
        ".center_x({0}).center_y({1}).width({2}).height({3}).label({4}).score({5}).distance({6}).speed({7}).track_id({8}).track_lifetime({9}).track_created({10})",
    ),
    (
        r"(?:edgefirst_msgs::)?Detect::new",
        ".stamp({0}).frame_id({1}).input_timestamp({2}).model_time({3}).output_time({4}).boxes({5})",
    ),
    (
        r"(?:edgefirst_msgs::)?Model::new",
        ".stamp({0}).frame_id({1}).input_time({2}).model_time({3}).output_time({4}).decode_time({5}).boxes({6}).masks({7})",
    ),
    (
        r"(?:edgefirst_msgs::)?ModelInfo::new",
        ".stamp({0}).frame_id({1}).input_shape({2}).input_type({3}).output_shape({4}).output_type({5}).labels({6}).model_type({7}).model_format({8}).model_name({9})",
    ),
    (
        r"(?:edgefirst_msgs::)?Vibration::new",
        ".stamp({0}).frame_id({1}).measurement_type({2}).unit({3}).band_lower_hz({4}).band_upper_hz({5}).vibration({6}).clipping({7})",
    ),
    (
        r"(?:foxglove_msgs::)?FoxgloveTextAnnotation::new",
        ".timestamp({0}).position({1}).text({2}).font_size({3}).text_color({4}).background_color({5})",
    ),
    (
        r"(?:foxglove_msgs::)?FoxglovePointAnnotation::new",
        ".timestamp({0}).type_({1}).points({2}).outline_color({3}).outline_colors({4}).fill_color({5}).thickness({6})",
    ),
    (
        r"(?:foxglove_msgs::)?FoxgloveImageAnnotation::new",
        ".circles({0}).points({1}).texts({2})",
    ),
    (
        r"(?:nav_msgs::)?Odometry::new",
        ".stamp({0}).frame_id({1}).child_frame_id({2}).pose({3}).twist({4})",
    ),
    (
        r"(?:nav_msgs::)?GridCells::new",
        ".stamp({0}).frame_id({1}).cell_width({2}).cell_height({3}).cells({4})",
    ),
    (
        r"(?:nav_msgs::)?OccupancyGrid::new",
        ".stamp({0}).frame_id({1}).info({2}).data({3})",
    ),
    (r"(?:nav_msgs::)?Path::new", ".stamp({0}).frame_id({1}).poses({2})"),
]


def split_args(s: str) -> list[str]:
    args: list[str] = []
    cur: list[str] = []
    depth = 0
    for ch in s:
        if ch == "," and depth == 0:
            args.append("".join(cur).strip())
            cur = []
            continue
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        cur.append(ch)
    tail = "".join(cur).strip()
    if tail:
        args.append(tail)
    return args


def type_prefix(match_text: str) -> str:
    """Return the module path prefix for a `Type::new` / `mod::Type::new` match.

    `Header::new` → ``; `std_msgs::Header::new` → `std_msgs::`.
    """
    parts = match_text.split("::")
    if len(parts) <= 2:
        return ""
    return "::".join(parts[:-2]) + "::"


def transform_call(full: str, type_pat: str, chain: str) -> str | None:
    # process_content already delimited the call; strip optional trailing unwrap
    body = full.rstrip()
    if body.endswith(".unwrap()"):
        body = body[: -len(".unwrap()")]
    m = re.match(type_pat + r"\((.*)\)$", body, re.DOTALL)
    if not m:
        return None
    args = split_args(m.group(1))
    needed = chain.count("{")
    if len(args) != needed:
        return None
    matched = re.search(type_pat, body)
    if matched is None:
        return None
    prefix = type_prefix(matched.group(0))
    bare = re.findall(r"(\w+)::new", type_pat)
    typename = bare[-1] if bare else "Type"
    return prefix + typename + "::builder()" + chain.format(*args) + ".build().unwrap()"


def process_content(content: str) -> tuple[str, int]:
    changed = 0
    for type_pat, chain in RULES:
        pat = re.compile(type_pat + r"\(")
        pos = 0
        out: list[str] = []
        while True:
            m = pat.search(content, pos)
            if not m:
                out.append(content[pos:])
                break
            out.append(content[pos : m.start()])
            # find matching close paren
            start = m.end() - 1
            depth = 0
            i = start
            while i < len(content):
                if content[i] == "(":
                    depth += 1
                elif content[i] == ")":
                    depth -= 1
                    if depth == 0:
                        end = i + 1
                        break
                i += 1
            else:
                out.append(content[m.start() :])
                break
            # include optional .unwrap()
            extra = 0
            if content[end : end + 9] == ".unwrap()":
                extra = 9
            full = content[m.start() : end + extra]
            repl = transform_call(full, type_pat, chain)
            if repl:
                out.append(repl)
                changed += 1
                pos = end + extra
            else:
                out.append(full)
                pos = end + extra
        content = "".join(out)
    return content, changed


def main() -> int:
    paths = [Path(p) for p in sys.argv[1:]] if len(sys.argv) > 1 else list(Path("crates/schemas").rglob("*.rs"))
    paths += [Path("examples/rust/basic_types.rs"), Path("crates/python/src/lib.rs")]
    total = 0
    for p in sorted(set(paths)):
        if not p.exists():
            continue
        text = p.read_text()
        new, n = process_content(text)
        if n:
            new = new.replace("#![allow(deprecated)]\n\n", "")
            new = new.replace("#![allow(deprecated)]\n", "")
            new = re.sub(r"#\[allow\(deprecated\)\]\s*\n", "", new)
            p.write_text(new)
            print(f"{p}: {n} replacements")
            total += n
    print(f"total: {total}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
