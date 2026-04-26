#!/usr/bin/env python3
"""Render benchmark results from multiple sources into a Markdown report.

Usage:
    render_benchmarks.py <results_dir>

Inputs in <results_dir>:
    criterion-data.tar.gz       -- Rust criterion (existing pipeline)
    benchmark.json              -- Python pytest-benchmark (existing pipeline)
    edgefirst.json, fastcdr.json, ...  -- C++ Google Benchmark JSON (keyed by stem)
    system.json (optional)      -- uname/cpuinfo metadata

Output: markdown to stdout.
"""

import glob
import json
import os
import re
import sys
import tarfile
import tempfile
import urllib.parse
from pathlib import Path


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def _format_time(ns: float) -> str:
    """Convert nanoseconds to a human-readable time string."""
    if ns >= 1e9:
        return f"{ns / 1e9:.4f} s"
    elif ns >= 1e6:
        return f"{ns / 1e6:.4f} ms"
    elif ns >= 1e3:
        return f"{ns / 1e3:.4f} µs"
    else:
        return f"{ns:.4f} ns"


def _parse_time_to_ms(time_str: str):
    """Convert time string to milliseconds, or None for N/A."""
    if time_str == "N/A":
        return None
    m = re.match(r"([\d.]+)\s*(\S+)", time_str)
    if not m:
        return None
    val, unit = float(m.group(1)), m.group(2)
    multipliers = {"ns": 0.000001, "µs": 0.001, "us": 0.001, "ms": 1.0, "s": 1000.0}
    return val * multipliers.get(unit, 1.0)


def _parse_time_to_ns(time_str: str):
    """Convert time string to nanoseconds, or None for N/A."""
    if time_str == "N/A":
        return None
    m = re.match(r"([\d.]+)\s*(\S+)", time_str)
    if not m:
        return None
    val, unit = float(m.group(1)), m.group(2)
    multipliers = {"ns": 1.0, "µs": 1000.0, "us": 1000.0, "ms": 1_000_000.0, "s": 1_000_000_000.0}
    return val * multipliers.get(unit, 1.0)


# ---------------------------------------------------------------------------
# Size label / order tables (shared with Rust/Python sections)
# ---------------------------------------------------------------------------

SIZE_LABELS = {
    # Image: resolution + encoding
    "VGA_rgb8": "VGA RGB", "VGA_yuyv": "VGA YUYV", "VGA_nv12": "VGA NV12",
    "HD_rgb8": "HD RGB", "HD_yuyv": "HD YUYV", "HD_nv12": "HD NV12",
    "FHD_rgb8": "FHD RGB", "FHD_yuyv": "FHD YUYV", "FHD_nv12": "FHD NV12",
    # PointCloud2
    "sparse_1K": "1K points", "medium_10K": "10K points",
    "dense_65K": "65K points", "very_dense_131K": "131K points",
    # Mask
    "320x320_8class": "320x320 8 classes", "320x320_32class": "320x320 32 classes",
    "640x640_8class": "640x640 8 classes", "640x640_32class": "640x640 32 classes",
    "1280x1280_8class": "1280x1280 8 classes", "1280x1280_32class": "1280x1280 32 classes",
    # RadarCube
    "DRVEGRD169_ultra_short": "DRVEGRD-169 Ultra-Short", "DRVEGRD169_short": "DRVEGRD-169 Short",
    "DRVEGRD169_medium": "DRVEGRD-169 Medium", "DRVEGRD169_long": "DRVEGRD-169 Long",
    "DRVEGRD171_short": "DRVEGRD-171 Short", "DRVEGRD171_medium": "DRVEGRD-171 Medium",
    "DRVEGRD171_long": "DRVEGRD-171 Long",
    # CompressedVideo
    "10KB": "10 KB", "100KB": "100 KB", "500KB": "500 KB", "1MB": "1 MB",
}

SIZE_ORDER = {
    "VGA_rgb8": 1, "VGA_yuyv": 2, "VGA_nv12": 3,
    "HD_rgb8": 4, "HD_yuyv": 5, "HD_nv12": 6,
    "FHD_rgb8": 7, "FHD_yuyv": 8, "FHD_nv12": 9,
    "sparse_1K": 1, "medium_10K": 2, "dense_65K": 3, "very_dense_131K": 4,
    "320x320_8class": 1, "320x320_32class": 2,
    "640x640_8class": 3, "640x640_32class": 4,
    "1280x1280_8class": 5, "1280x1280_32class": 6,
    "DRVEGRD169_ultra_short": 1, "DRVEGRD169_short": 2,
    "DRVEGRD169_medium": 3, "DRVEGRD169_long": 4,
    "DRVEGRD171_short": 5, "DRVEGRD171_medium": 6, "DRVEGRD171_long": 7,
    "10KB": 1, "100KB": 2, "500KB": 3, "1MB": 4,
}

# ---------------------------------------------------------------------------
# Parsers
# ---------------------------------------------------------------------------

def parse_criterion(tarball: Path) -> list:
    """Parse criterion-data.tar.gz into a list of benchmark dicts."""
    benchmarks = []
    with tempfile.TemporaryDirectory() as tmpdir:
        with tarfile.open(tarball) as tar:
            tar.extractall(tmpdir)
        criterion_dir = os.path.join(tmpdir, "criterion")
        if not os.path.isdir(criterion_dir):
            return benchmarks
        for bench_json in glob.glob(f"{criterion_dir}/**/new/benchmark.json", recursive=True):
            bench_dir = os.path.dirname(bench_json)
            estimates_json = os.path.join(bench_dir, "estimates.json")
            if not os.path.exists(estimates_json):
                continue
            try:
                with open(bench_json) as f:
                    bench_data = json.load(f)
                with open(estimates_json) as f:
                    estimates = json.load(f)
                full_id = bench_data.get("full_id", "")
                if not full_id:
                    continue
                time_data = estimates.get("slope") or estimates.get("median", {})
                point_estimate = time_data.get("point_estimate")
                if point_estimate is None:
                    continue
                ns = float(point_estimate)
                time_str = _format_time(ns)
                throughput = bench_data.get("throughput", {})
                thrpt_str = "N/A"
                if throughput:
                    bytes_per_iter = throughput.get("Bytes")
                    if bytes_per_iter and ns > 0:
                        bytes_per_sec = bytes_per_iter * 1e9 / ns
                        if bytes_per_sec >= 1e9:
                            thrpt_str = f"{bytes_per_sec / 1e9:.2f} GiB/s"
                        elif bytes_per_sec >= 1e6:
                            thrpt_str = f"{bytes_per_sec / 1e6:.2f} MiB/s"
                        else:
                            thrpt_str = f"{bytes_per_sec / 1e3:.2f} KiB/s"
                benchmarks.append({"name": full_id, "time": time_str, "throughput": thrpt_str})
            except Exception as e:
                print(f"Warning: Failed to parse {bench_json}: {e}", file=sys.stderr)
    return benchmarks


def parse_pytest_benchmark(path: Path) -> list:
    """Parse a pytest-benchmark JSON file."""
    benchmarks = []
    try:
        data = json.loads(path.read_text())
        for bench in data.get("benchmarks", []):
            name = bench.get("name", "")
            stats = bench.get("stats", {})
            mean_ns = stats.get("mean", 0) * 1e9
            benchmarks.append({"name": name, "time": _format_time(mean_ns), "throughput": "N/A"})
    except Exception as e:
        print(f"Warning: Failed to parse Python benchmarks: {e}", file=sys.stderr)
    return benchmarks


def parse_googlebench(path: Path) -> list:
    """Parse a Google Benchmark JSON file.

    Returns benchmarks list with 'name', 'time', 'throughput', 'impl'.
    The 'impl' key is derived from the file stem (e.g. edgefirst.json -> 'edgefirst').
    """
    impl = path.stem
    benchmarks = []
    try:
        data = json.loads(path.read_text())
        for b in data.get("benchmarks", []):
            name = b.get("name", "")
            ns = b.get("real_time")
            if ns is None or not name:
                continue
            # Skip aggregate rows (mean, median, stddev) if multiple repetitions
            if b.get("run_type", "iteration") == "aggregate":
                continue
            benchmarks.append({
                "name": name,
                "time": _format_time(float(ns)),
                "throughput": "N/A",
                "impl": impl,
            })
    except Exception as e:
        print(f"Warning: Failed to parse {path}: {e}", file=sys.stderr)
    return benchmarks


# ---------------------------------------------------------------------------
# QuickChart helpers
# ---------------------------------------------------------------------------

def _quickchart_url(chart_config: dict, width: int = 600, height: int = 300) -> str:
    chart_json = json.dumps(chart_config, separators=(",", ":"))
    return f"https://quickchart.io/chart?c={urllib.parse.quote(chart_json)}&w={width}&h={height}"


# Distinct colors for up to 6 impls (Tailwind 500-level palette).
IMPL_COLORS = [
    {"label": "edgefirst",  "bg": "rgba(59,130,246,0.85)",  "border": "rgba(59,130,246,1)"},   # blue-500
    {"label": "fastcdr",    "bg": "rgba(245,158,11,0.85)",  "border": "rgba(245,158,11,1)"},   # amber-500
    {"label": "cyclonedds", "bg": "rgba(16,185,129,0.85)",  "border": "rgba(16,185,129,1)"},   # emerald-500
    {"label": "rclcpp",     "bg": "rgba(239,68,68,0.85)",   "border": "rgba(239,68,68,1)"},    # red-500
    {"label": "rclpy",      "bg": "rgba(139,92,246,0.85)",  "border": "rgba(139,92,246,1)"},   # violet-500
    {"label": "extra",      "bg": "rgba(6,182,212,0.85)",   "border": "rgba(6,182,212,1)"},    # cyan-500
]

# Fallback color list for unknown impl names (round-robin by index).
_FALLBACK_COLORS = [e for e in IMPL_COLORS]

_IMPL_COLOR_MAP = {e["label"]: e for e in IMPL_COLORS}


def _color_for_impl(impl: str, fallback_index: int) -> tuple[str, str]:
    """Return (bg, border) for the given impl name."""
    entry = _IMPL_COLOR_MAP.get(impl)
    if entry:
        return entry["bg"], entry["border"]
    return _FALLBACK_COLORS[fallback_index % len(_FALLBACK_COLORS)]["bg"], \
           _FALLBACK_COLORS[fallback_index % len(_FALLBACK_COLORS)]["border"]


def pick_unit(max_ns: float) -> tuple[str, float]:
    """Return (unit_label, scale_divisor) for a max value in nanoseconds."""
    if max_ns >= 1e9:
        return ("s",  1e9)
    if max_ns >= 1e6:
        return ("ms", 1e6)
    if max_ns >= 1e3:
        return ("µs", 1e3)
    return ("ns", 1.0)


def should_use_log(values: list) -> bool:
    """Return True when max/min ratio exceeds 1000 (high dynamic range)."""
    nonzero = [v for v in values if v is not None and v > 0]
    if not nonzero or len(nonzero) < 2:
        return False
    return (max(nonzero) / min(nonzero)) > 1000


# Ops excluded from charts (stay in tables).
CHART_OP_DENYLIST = {"workflow/pub_loop_rebuild"}


def _make_horizontal_bar_chart(title: str, labels: list, datasets: list, unit: str) -> dict:
    """Build a Chart.js horizontalBar config dict.

    datasets: list of (impl_name, values_in_ns) — values will be scaled to unit.
    unit: already-chosen display unit string (e.g. 'ns', 'µs', 'ms', 's').

    For high-dynamic-range datasets (max/min > 1000), data is kept in
    nanoseconds and rendered on a logarithmic axis with a per-value humanizer
    so each bar's label uses an appropriate unit (e.g. "55 ns" next to
    "1.74 ms" instead of "0.00 ms" next to "1.74 ms").
    """
    # Decide log vs linear before deciding scale, since the two interact.
    raw_values = []
    for _, values in datasets:
        raw_values.extend(values)
    use_log = should_use_log(raw_values)

    # Number formatting rule (used by both linear and log paths):
    #   - ns: integer always (decimals on nanoseconds are noise).
    #   - µs / ms: one decimal when value < 100 in unit, integer when >= 100.
    #   - s: two decimals (rare in our data).
    # The JS expressions below implement this per-value.
    fmt_humanize_js = (
        "(v) => { if (!v) return ''; "
        "if (v < 1e3) return v.toFixed(0)+' ns'; "
        "if (v < 1e6) { var x = v/1e3; "
        "  return (x<100 ? x.toFixed(1) : x.toFixed(0))+' \\u00b5s'; } "
        "if (v < 1e9) { var x = v/1e6; "
        "  return (x<100 ? x.toFixed(1) : x.toFixed(0))+' ms'; } "
        "return (v/1e9).toFixed(2)+' s'; }"
    )

    if use_log:
        # Keep data in ns; let a per-value humanizer pick the unit per label.
        chart_datasets = []
        for i, (ds_label, values) in enumerate(datasets):
            data = [round(v, 2) if v else 0 for v in values]
            bg, border = _color_for_impl(ds_label, i)
            chart_datasets.append({
                "label": ds_label,
                "data": data,
                "backgroundColor": bg,
                "borderColor": border,
                "borderWidth": 1,
            })
        humanize_js = fmt_humanize_js
        x_axis = {
            "type": "logarithmic",
            "scaleLabel": {
                "display": True,
                "labelString": "Time (ns, log)",
                "fontSize": 9,
            },
            "ticks": {"fontSize": 8},
        }
        chart_title = f"{title} (log scale)"
    else:
        # Single unit for the whole chart — sensible when range is narrow.
        divisor_map = {"ns": 1.0, "µs": 1e3, "ms": 1e6, "s": 1e9}
        divisor = divisor_map.get(unit, 1.0)
        chart_datasets = []
        for i, (ds_label, values) in enumerate(datasets):
            scaled = [round(v / divisor, 4) if v else 0 for v in values]
            bg, border = _color_for_impl(ds_label, i)
            chart_datasets.append({
                "label": ds_label,
                "data": scaled,
                "backgroundColor": bg,
                "borderColor": border,
                "borderWidth": 1,
            })
        # Linear path: data is already in the chart's display unit, so the
        # humanizer's first branch ("ns" cutoff) wouldn't trigger correctly.
        # Format-in-place: integer when >= 100, one decimal when 1-100, two
        # decimals when < 1. ns is always integer.
        if unit == "ns":
            humanize_js = "(v) => v > 0 ? v.toFixed(0)+' ns' : ''"
        else:
            humanize_js = (
                "(v) => { if (!v) return ''; "
                "if (v >= 100) return v.toFixed(0)+' " + unit + "'; "
                "if (v >= 1) return v.toFixed(1)+' " + unit + "'; "
                "return v.toFixed(2)+' " + unit + "'; }"
            )
        x_axis = {
            "scaleLabel": {
                "display": True,
                "labelString": f"Time ({unit})",
                "fontSize": 9,
            },
            "ticks": {"beginAtZero": True, "fontSize": 8},
        }
        chart_title = f"{title} ({unit})"

    return {
        "type": "horizontalBar",
        "data": {"labels": labels, "datasets": chart_datasets},
        "options": {
            "title": {"display": True, "text": chart_title, "fontSize": 11},
            "legend": {"labels": {"fontSize": 9, "boxWidth": 12}},
            "scales": {
                "xAxes": [x_axis],
                "yAxes": [{"ticks": {"fontSize": 8}}],
            },
            # Per-bar value labels are intentionally suppressed — they crowd
            # the chart and the exact numbers live in the click-to-expand
            # table immediately below each chart.
            "plugins": {
                "datalabels": {"display": False}
            },
        },
    }


# ---------------------------------------------------------------------------
# Rust section renderer (exact original behavior)
# ---------------------------------------------------------------------------

HEAVY_CATEGORIES = {
    "DmaBuf": {"desc": "Zero-copy DMA buffer reference (metadata only)", "size_axis": "Resolution", "unit": "ns"},
    "Image": {"desc": "Camera frame serialization (RGB/YUYV/NV12)", "size_axis": "Format", "unit": "ms"},
    "PointCloud2": {"desc": "LiDAR point cloud data", "size_axis": "Point Count", "unit": "ms"},
    "Mask": {"desc": "Segmentation mask data (uncompressed)", "size_axis": "Size", "unit": "ms"},
    "CompressedMask": {"desc": "Segmentation mask data (zstd compressed)", "size_axis": "Size", "unit": "ms"},
    "RadarCube": {"desc": "SmartMicro DRVEGRD radar cube tensors", "size_axis": "Mode", "unit": "ms"},
    "FoxgloveCompressedVideo": {"desc": "Compressed video frames", "size_axis": "Payload Size", "unit": "ms"},
}


def render_rust_section(f, rust_benchmarks: list):
    basic_msgs = []
    heavy_msgs = []
    for b in rust_benchmarks:
        if any(cat in b["name"] for cat in ["builtin_interfaces", "std_msgs", "geometry_msgs"]):
            basic_msgs.append(b)
        else:
            heavy_msgs.append(b)

    heavy_categories = {k: dict(v, benchmarks=[]) for k, v in HEAVY_CATEGORIES.items()}
    for b in heavy_msgs:
        for cat in heavy_categories:
            if b["name"].startswith(cat + "/"):
                heavy_categories[cat]["benchmarks"].append(b)
                break

    f.write("## \U0001f980 Rust Benchmarks\n\n")

    for cat_name, cat_info in heavy_categories.items():
        cat_benchmarks = cat_info["benchmarks"]
        if not cat_benchmarks:
            continue

        f.write(f"### {cat_name}\n\n")
        f.write(f"*{cat_info['desc']}*\n\n")

        variants = {}
        for b in cat_benchmarks:
            parts = b["name"].split("/")
            op = parts[1] if len(parts) > 1 else "unknown"
            variant = parts[2] if len(parts) >= 3 else "default"
            if variant not in variants:
                variants[variant] = {"serialize": "N/A", "deserialize": "N/A"}
            variants[variant][op] = b["time"]

        sorted_variants = sorted(variants.items(), key=lambda x: SIZE_ORDER.get(x[0], 99))

        chart_unit = cat_info.get("unit", "ms")
        parse_fn = _parse_time_to_ns if chart_unit == "ns" else _parse_time_to_ms

        f.write(f"| {cat_info['size_axis']} | Serialize | Deserialize |\n")
        f.write("|------------|-----------|-------------|\n")

        chart_labels = []
        ser_times = []
        deser_times = []
        for variant, data in sorted_variants:
            label = SIZE_LABELS.get(variant, variant)
            f.write(f"| {label} | {data['serialize']} | {data['deserialize']} |\n")
            chart_labels.append(label)
            ser_times.append(parse_fn(data["serialize"]))
            deser_times.append(parse_fn(data["deserialize"]))

        f.write("\n")

        # Determine unit from max value for smart scaling
        all_ns = [v for v in ser_times + deser_times if v is not None]
        if all_ns:
            chart_unit, _ = pick_unit(max(all_ns))
        chart_config = _make_horizontal_bar_chart(
            f"{cat_name} Latency", chart_labels,
            [("Serialize", ser_times), ("Deserialize", deser_times)],
            chart_unit,
        )
        height = max(200, len(chart_labels) * 50)
        chart_url = _quickchart_url(chart_config, width=600, height=height)
        f.write(f"![{cat_name} Chart]({chart_url})\n\n")

    # Basic messages (collapsed)
    f.write("<details>\n")
    f.write("<summary>Rust Basic Message Types (click to expand)</summary>\n\n")
    f.write("| Message | Serialize | Deserialize |\n")
    f.write("|---------|-----------|-------------|\n")

    msg_data = {}
    for b in basic_msgs:
        parts = b["name"].split("/")
        if len(parts) >= 3:
            msg_type = f"{parts[0]}/{parts[1]}"
            op = parts[2]
            if msg_type not in msg_data:
                msg_data[msg_type] = {"serialize": "N/A", "deserialize": "N/A"}
            msg_data[msg_type][op] = b["time"]

    for msg_type in sorted(msg_data.keys()):
        data = msg_data[msg_type]
        f.write(f"| {msg_type} | {data['serialize']} | {data['deserialize']} |\n")

    f.write("\n</details>\n\n")

    f.write("### Rust Summary\n\n")
    f.write(f"- **Heavy message benchmarks:** {len(heavy_msgs)}\n")
    f.write(f"- **Basic message benchmarks:** {len(basic_msgs)}\n")
    f.write(f"- **Total Rust:** {len(rust_benchmarks)}\n\n")


# ---------------------------------------------------------------------------
# Python section renderer (exact original behavior)
# ---------------------------------------------------------------------------

def render_python_section(f, python_benchmarks: list):
    f.write("## \U0001f40d Python Benchmarks\n\n")

    py_basic = []
    py_heavy = []
    for b in python_benchmarks:
        name = b["name"].lower()
        if any(cat in name for cat in [
            "time", "duration", "header", "color", "vector", "point",
            "quaternion", "pose", "transform", "twist"
        ]):
            py_basic.append(b)
        else:
            py_heavy.append(b)

    if py_heavy:
        f.write("### Heavy Message Types\n\n")
        f.write("| Benchmark | Time |\n")
        f.write("|-----------|------|\n")
        for b in sorted(py_heavy, key=lambda x: x["name"]):
            f.write(f"| {b['name']} | {b['time']} |\n")
        f.write("\n")

    if py_basic:
        f.write("<details>\n")
        f.write("<summary>Python Basic Message Types (click to expand)</summary>\n\n")
        f.write("| Benchmark | Time |\n")
        f.write("|-----------|------|\n")
        for b in sorted(py_basic, key=lambda x: x["name"]):
            f.write(f"| {b['name']} | {b['time']} |\n")
        f.write("\n</details>\n\n")

    f.write("### Python Summary\n\n")
    f.write(f"- **Heavy message benchmarks:** {len(py_heavy)}\n")
    f.write(f"- **Basic message benchmarks:** {len(py_basic)}\n")
    f.write(f"- **Total Python:** {len(python_benchmarks)}\n\n")


# ---------------------------------------------------------------------------
# C++ section renderer
# ---------------------------------------------------------------------------

# Known op groups for C++ benchmarks
_ACCESS_OPS = {"one_field", "half_fields", "all_fields", "payload_iter"}
_WORKFLOW_OPS = {"sub_loop", "sub_modify_pub"}  # pub_loop_* handled by prefix


def _op_group(op: str) -> str:
    if op in _ACCESS_OPS:
        return "access"
    if op in _WORKFLOW_OPS or op.startswith("pub_loop"):
        return "workflow"
    return "core"


def render_cpp_section(f, cpp_benchmarks_by_impl: dict):
    """Render the C++ benchmarks section.

    cpp_benchmarks_by_impl: {impl_name: [benchmark_dicts]}
    Each benchmark dict has: name, time, throughput, impl
    Name format: MessageType/op_group/op_name/variant
    e.g. Header/encode/new/default  or  Header/access/one_field/default
    """
    if not cpp_benchmarks_by_impl:
        return

    impls = sorted(cpp_benchmarks_by_impl.keys())

    f.write("## \U0001f537 C++ Benchmarks\n\n")
    f.write("*EdgeFirst zero-copy CDR vs. eProsima Fast-CDR comparison.*\n\n")

    # Collect all benchmarks flat, then group by (message_type, op_group, op, variant)
    # Name format: MsgType/phase/op/variant  (4 parts)
    # OR:          MsgType/phase/op          (3 parts, variant=default)
    all_rows = {}  # (msg_type, op_group, op, variant) -> {impl: time_str}
    for impl, benches in cpp_benchmarks_by_impl.items():
        for b in benches:
            parts = b["name"].split("/")
            if len(parts) == 4:
                msg_type, phase, op, variant = parts
            elif len(parts) == 3:
                msg_type, phase, op = parts
                variant = "default"
            elif len(parts) == 2:
                msg_type, phase = parts
                op = phase
                variant = "default"
            else:
                continue  # skip malformed
            key = (msg_type, phase, op, variant)
            if key not in all_rows:
                all_rows[key] = {}
            all_rows[key][impl] = b["time"]

    # Group by message_type
    by_msg: dict = {}
    for (msg_type, phase, op, variant), impl_times in all_rows.items():
        by_msg.setdefault(msg_type, []).append((phase, op, variant, impl_times))

    for msg_type in sorted(by_msg.keys()):
        rows = by_msg[msg_type]
        f.write(f"### {msg_type}\n\n")

        # Sub-group by phase (encode, decode, access, workflow)
        by_phase: dict = {}
        for phase, op, variant, impl_times in rows:
            by_phase.setdefault(phase, []).append((op, variant, impl_times))

        for phase in sorted(by_phase.keys()):
            phase_rows = by_phase[phase]

            # Deduplicate: one row per (op, variant)
            seen = {}
            for op, variant, impl_times in phase_rows:
                k = (op, variant)
                if k not in seen:
                    seen[k] = {}
                seen[k].update(impl_times)

            phase_rows_dedup = [(op, variant, impl_times) for (op, variant), impl_times in seen.items()]
            phase_rows_dedup.sort(key=lambda r: r[0])  # sort by op name

            # Lead with the section heading, build chart first, then put the
            # numeric table behind a <details> toggle.
            f.write(f"#### {phase}\n\n")

            # First pass: collect chart datasets and build table rows in memory.
            chart_labels = []
            impl_chart_times = {impl: [] for impl in impls}
            table_rows = []

            for op, variant, impl_times in phase_rows_dedup:
                display = variant if variant != "default" else op
                row = f"| {display} |"
                times_ns = {}
                for impl in impls:
                    t = impl_times.get(impl, "N/A")
                    row += f" {t} |"
                    times_ns[impl] = _parse_time_to_ns(t)

                if len(impls) == 2:
                    t0 = times_ns.get(impls[0])
                    t1 = times_ns.get(impls[1])
                    if t0 and t1 and t0 > 0:
                        speedup = t1 / t0
                        row += f" {speedup:.2f}x |"
                    else:
                        row += " N/A |"
                table_rows.append(row)

                # Chart: skip denylisted ops
                chart_op_key = f"{phase}/{op}"
                if chart_op_key not in CHART_OP_DENYLIST:
                    chart_labels.append(display)
                    for impl in impls:
                        impl_chart_times[impl].append(times_ns.get(impl))

            # Emit the chart first (default-visible).
            if chart_labels:
                all_ns = [v for vals in impl_chart_times.values() for v in vals if v is not None]
                chart_unit, _ = pick_unit(max(all_ns)) if all_ns else ("ns", 1.0)
                datasets = [(impl, impl_chart_times[impl]) for impl in impls]
                chart_config = _make_horizontal_bar_chart(
                    f"{msg_type}/{phase} Latency", chart_labels, datasets, chart_unit
                )
                height = max(200, len(chart_labels) * 60)
                chart_url = _quickchart_url(chart_config, width=600, height=height)
                f.write(f"![{msg_type}/{phase} Chart]({chart_url})\n\n")

            # Tuck the numeric table behind a collapsible toggle.
            f.write("<details>\n")
            f.write(f"<summary>Numeric values for {msg_type}/{phase} (click to expand)</summary>\n\n")
            impl_cols = " | ".join(impl for impl in impls)
            f.write(f"| Variant | {impl_cols} |")
            if len(impls) == 2:
                f.write(" Speedup |")
            f.write("\n")
            sep_cols = " | ".join("-------" for _ in impls)
            f.write(f"| ------- | {sep_cols} |")
            if len(impls) == 2:
                f.write(" ------- |")
            f.write("\n")
            for row in table_rows:
                f.write(row + "\n")
            f.write("\n</details>\n\n")

    # C++ summary
    total_cpp = sum(len(v) for v in cpp_benchmarks_by_impl.values())
    f.write("### C++ Summary\n\n")
    for impl in impls:
        f.write(f"- **{impl}:** {len(cpp_benchmarks_by_impl[impl])} benchmarks\n")
    f.write(f"- **Total C++:** {total_cpp}\n\n")


# ---------------------------------------------------------------------------
# System metadata
# ---------------------------------------------------------------------------

def render_header(f, results_dir: Path, rust_count: int, python_count: int, cpp_count: int):
    f.write("## \U0001f4ca On-Target Benchmark Results\n\n")

    # Try to read system.json for hardware metadata
    system_json = results_dir / "system.json"
    if system_json.exists():
        try:
            sys_info = json.loads(system_json.read_text())
            hw = sys_info.get("hardware", "Unknown")
            arch = sys_info.get("arch", "Unknown")
            f.write(f"**Target Hardware:** {hw}\n")
            f.write(f"**Architecture:** {arch}\n")
        except Exception:
            f.write("**Target Hardware:** NXP i.MX 8M Plus (Cortex-A53 @ 1.8GHz)\n")
            f.write("**Architecture:** aarch64\n")
    else:
        f.write("**Target Hardware:** NXP i.MX 8M Plus (Cortex-A53 @ 1.8GHz)\n")
        f.write("**Architecture:** aarch64\n")

    total = rust_count + python_count + cpp_count
    parts = []
    if rust_count:
        parts.append(f"Rust: {rust_count}")
    if python_count:
        parts.append(f"Python: {python_count}")
    if cpp_count:
        parts.append(f"C++: {cpp_count}")
    detail = ", ".join(parts)
    f.write(f"**Total Benchmarks:** {total}" + (f" ({detail})" if detail else "") + "\n\n")


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        sys.exit(1)

    results_dir = Path(sys.argv[1])
    if not results_dir.is_dir():
        print(f"Error: {results_dir} is not a directory", file=sys.stderr)
        sys.exit(1)

    # --- Parse inputs ---

    rust_benchmarks = []
    criterion_tarball = results_dir / "criterion-data.tar.gz"
    if criterion_tarball.exists():
        print(f"Using Criterion JSON data for Rust benchmarks", file=sys.stderr)
        rust_benchmarks = parse_criterion(criterion_tarball)
        print(f"Parsed {len(rust_benchmarks)} Rust benchmarks", file=sys.stderr)

    python_benchmarks = []
    python_json = results_dir / "benchmark.json"
    if python_json.exists():
        print("Parsing Python benchmark JSON", file=sys.stderr)
        python_benchmarks = parse_pytest_benchmark(python_json)
        print(f"Parsed {len(python_benchmarks)} Python benchmarks", file=sys.stderr)

    cpp_benchmarks_by_impl: dict = {}
    for json_path in sorted(results_dir.glob("*.json")):
        if json_path.name == "benchmark.json":
            continue  # Python pytest-benchmark, already handled
        if json_path.name == "system.json":
            continue  # metadata
        benches = parse_googlebench(json_path)
        if benches:
            impl = json_path.stem
            cpp_benchmarks_by_impl[impl] = benches
            print(f"Parsed {len(benches)} C++ benchmarks from {json_path.name} (impl={impl})", file=sys.stderr)

    cpp_count = sum(len(v) for v in cpp_benchmarks_by_impl.values())

    # --- Render ---
    import io
    buf = io.StringIO()

    render_header(buf, results_dir, len(rust_benchmarks), len(python_benchmarks), cpp_count)

    if rust_benchmarks:
        render_rust_section(buf, rust_benchmarks)

    if python_benchmarks:
        render_python_section(buf, python_benchmarks)

    if cpp_benchmarks_by_impl:
        render_cpp_section(buf, cpp_benchmarks_by_impl)

    print(buf.getvalue(), end="")


if __name__ == "__main__":
    main()
