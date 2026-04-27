#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Render Python CDR benchmark results into BENCHMARKS_PYTHON.md.

Usage:
    render_python_benchmarks.py <results_dir>

Inputs in <results_dir>:
    native.json        -- pytest-benchmark JSON from bench_native.py
    cyclonedds.json    -- pytest-benchmark JSON from bench_cyclonedds.py
    pycdr2.json        -- pytest-benchmark JSON from bench_pycdr2.py
    system.json        -- target metadata (uname/cpuinfo/python_version/git_rev)

Output: markdown to stdout, mirroring the chart-and-table structure of
BENCHMARKS.md (the C++-tier report) but populated with Python implementations.

Test names follow the parametrize convention used by all three benches:
    test_<message>_<op>[<variant>]   for heavy types
    test_<message>_<op>              for light CdrFixed types

The fixture shapes themselves come from `benches/python/shapes.py`, which
mirrors `benches/cpp/common.hpp` exactly — so charts pair Python and C++
results by variant name.
"""

import json
import re
import sys
import urllib.parse
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# Impl identity
# ---------------------------------------------------------------------------

IMPL_ORDER = ["native", "cyclonedds", "pycdr2"]
IMPL_LABELS = {
    "native":     "edgefirst (pyo3)",
    "cyclonedds": "cyclone-dds-py",
    "pycdr2":     "pycdr2",
}
IMPL_COLORS = {
    "native":     ("rgba(59,130,246,0.85)",  "rgba(59,130,246,1)"),  # blue
    "cyclonedds": ("rgba(16,185,129,0.85)",  "rgba(16,185,129,1)"),  # emerald
    "pycdr2":     ("rgba(139,92,246,0.85)",  "rgba(139,92,246,1)"),  # violet
}


# ---------------------------------------------------------------------------
# Heavy-type taxonomy. Order matches benches/cpp/common.hpp.
# Each entry: (msg_python_key, display_name, description, axis_label,
#              [variants], {variant: pretty_label})
# ---------------------------------------------------------------------------


HEAVY_CATEGORIES = [
    {
        "msg": "image",
        "display": "Image",
        "desc": "Camera frame (RGB / YUYV / NV12).",
        "axis": "resolution × encoding",
        "variants": [
            "VGA_rgb8", "VGA_yuyv", "VGA_nv12",
            "HD_rgb8",  "HD_yuyv",  "HD_nv12",
            "FHD_rgb8", "FHD_yuyv", "FHD_nv12",
        ],
        "labels": {
            "VGA_rgb8": "VGA RGB",  "VGA_yuyv": "VGA YUYV", "VGA_nv12": "VGA NV12",
            "HD_rgb8":  "HD RGB",   "HD_yuyv":  "HD YUYV",  "HD_nv12":  "HD NV12",
            "FHD_rgb8": "FHD RGB",  "FHD_yuyv": "FHD YUYV", "FHD_nv12": "FHD NV12",
        },
    },
    {
        "msg": "radar_cube",
        "display": "RadarCube",
        "desc": "SmartMicro DRVEGRD radar cube tensors. DRVEGRD-169 = 12 virtual RX channels (3TX × 4RX); DRVEGRD-171 = 48 virtual RX (6TX × 8RX).",
        "axis": "DRVEGRD range mode",
        "variants": [
            "DRVEGRD169_ultra_short", "DRVEGRD169_short", "DRVEGRD169_long",
            "DRVEGRD171_short", "DRVEGRD171_extra_long",
        ],
        "labels": {
            "DRVEGRD169_ultra_short": "DRVEGRD-169 Ultra-Short",
            "DRVEGRD169_short":       "DRVEGRD-169 Short",
            "DRVEGRD169_long":        "DRVEGRD-169 Long",
            "DRVEGRD171_short":       "DRVEGRD-171 Short",
            "DRVEGRD171_extra_long":  "DRVEGRD-171 Extra-Long",
        },
    },
    {
        "msg": "mask",
        "display": "Mask",
        "desc": "Argmax-applied segmentation masks (1 u8 per pixel, regardless of class count).",
        "axis": "resolution",
        "variants": [
            "160x160_proto", "320x320_8class", "480x480_9class",
            "640x640_8class", "1280x720_hd", "1920x1080_fhd",
        ],
        "labels": {
            "160x160_proto":   "160×160 proto",
            "320x320_8class":  "320×320 (HD model)",
            "480x480_9class":  "480×480 (FHD model)",
            "640x640_8class":  "640×640 (instance mask)",
            "1280x720_hd":     "1280×720 HD",
            "1920x1080_fhd":   "1920×1080 FHD",
        },
    },
    {
        "msg": "point_cloud",
        "display": "PointCloud2",
        "desc": "LiDAR / radar point clouds — point_step=13 = x/y/z f32 + reflect u8; point_step=16 = + fusion_class/vision_class/instance_id.",
        "axis": "sensor mode",
        "variants": [
            "robosense_e1r", "ouster_1024x10_128beam",
            "ouster_2048x10_128beam", "fusion_classes_ouster",
        ],
        "labels": {
            "robosense_e1r":          "Robosense E1R (26K, 13 bpp)",
            "ouster_1024x10_128beam": "Ouster 1024×10×128 (131K, 13 bpp)",
            "ouster_2048x10_128beam": "Ouster 2048×10×128 (262K, 13 bpp)",
            "fusion_classes_ouster":  "Fusion classes (131K, 16 bpp)",
        },
    },
    {
        "msg": "compressed_video",
        "display": "CompressedVideo",
        "desc": "Foxglove H.264/H.265 NAL-unit frames.",
        "axis": "payload size",
        "variants": ["10KB", "100KB", "500KB", "1MB"],
        "labels": {"10KB": "10 KB", "100KB": "100 KB", "500KB": "500 KB", "1MB": "1 MB"},
    },
    {
        "msg": "dmabuf",
        "display": "DmaBuffer",
        "desc": "DMA-buf reference (metadata only; no payload).",
        "axis": "fixture",
        "variants": ["default"],
        "labels": {"default": "default 1280×720 RGBA"},
    },
]

# Light types: order they appear in the table.
LIGHT_TYPES_ORDER = [
    ("time",       "Time"),
    ("duration",   "Duration"),
    ("header",     "Header"),
    ("colorrgba",  "ColorRGBA"),
    ("vector3",    "Vector3"),
    ("point",      "Point"),
    ("point32",    "Point32"),
    ("quaternion", "Quaternion"),
    ("pose",       "Pose"),
    ("pose2d",     "Pose2D"),
    ("transform",  "Transform"),
    ("twist",      "Twist"),
]


# ---------------------------------------------------------------------------
# Test-name parser.
# ---------------------------------------------------------------------------

# Parametrized form:        test_<msg>_<op>[<variant>]
# Plain form (light types): test_<msg>_<op>
_PARAM_RE = re.compile(r"^test_(.+?)_(serialize|deserialize)(?:\[(.+)\])?$")


def parse_test_name(name: str) -> Optional[tuple]:
    """Return (msg_key, op, variant_or_None) or None if name doesn't match."""
    m = _PARAM_RE.match(name)
    if not m:
        return None
    return (m.group(1), m.group(2), m.group(3))


# ---------------------------------------------------------------------------
# JSON parsing
# ---------------------------------------------------------------------------


def parse_pytest_json(path: Path) -> dict:
    """Return {test_name: median_ns} for one impl's JSON."""
    if not path.exists():
        return {}
    with path.open() as f:
        data = json.load(f)
    out = {}
    for b in data.get("benchmarks", []):
        median_s = b["stats"].get("median")
        if median_s is None:
            continue
        out[b["name"]] = median_s * 1e9  # → ns
    return out


# ---------------------------------------------------------------------------
# Chart construction (QuickChart URL + Markdown image embed)
# ---------------------------------------------------------------------------


def _quickchart_url(cfg: dict, width: int = 600, height: int = 300) -> str:
    raw = json.dumps(cfg, separators=(",", ":"))
    return f"https://quickchart.io/chart?c={urllib.parse.quote(raw)}&w={width}&h={height}"


def _format_time(ns: float) -> str:
    if ns is None:
        return "—"
    if ns >= 1e9:
        return f"{ns / 1e9:.2f} s"
    if ns >= 1e6:
        return f"{ns / 1e6:.1f} ms" if ns < 1e8 else f"{ns / 1e6:.0f} ms"
    if ns >= 1e3:
        return f"{ns / 1e3:.1f} µs" if ns < 1e5 else f"{ns / 1e3:.0f} µs"
    return f"{ns:.0f} ns"


def _build_chart(title: str, labels_pretty: list, datasets: list) -> str:
    """datasets: list of (impl_key, [values_in_ns or None]). Returns markdown image."""
    raw = []
    for _, vals in datasets:
        raw.extend(v for v in vals if v)
    if not raw:
        return ""

    use_log = (max(raw) / max(min(raw), 1)) > 100.0

    chart_ds = []
    for impl, vals in datasets:
        bg, border = IMPL_COLORS[impl]
        chart_ds.append({
            "label": IMPL_LABELS[impl],
            "data": [round(v, 2) if v else 0 for v in vals],
            "backgroundColor": bg,
            "borderColor": border,
            "borderWidth": 1,
        })

    if use_log:
        x_axis = {
            "type": "logarithmic",
            "scaleLabel": {"display": True, "labelString": "Time (ns, log)", "fontSize": 9},
            "ticks": {"fontSize": 8},
        }
        chart_title = f"{title} (log scale)"
    else:
        x_axis = {
            "scaleLabel": {"display": True, "labelString": "Time (ns)", "fontSize": 9},
            "ticks": {"beginAtZero": True, "fontSize": 8},
        }
        chart_title = title

    cfg = {
        "type": "horizontalBar",
        "data": {"labels": labels_pretty, "datasets": chart_ds},
        "options": {
            "title": {"display": True, "text": chart_title, "fontSize": 11},
            "legend": {"labels": {"fontSize": 9, "boxWidth": 12}},
            "scales": {
                "xAxes": [x_axis],
                "yAxes": [{"ticks": {"fontSize": 8}}],
            },
            "plugins": {"datalabels": {"display": False}},
        },
    }
    height = max(180, 60 + 36 * len(labels_pretty))
    url = _quickchart_url(cfg, width=620, height=height)
    return f"![{title}]({url})\n\n"


# ---------------------------------------------------------------------------
# Section renderers
# ---------------------------------------------------------------------------


OP_DISPLAY = {"serialize": "encode", "deserialize": "decode"}


def render_heavy_section(f, category: dict, op: str, results: dict):
    """`results`: {variant: {impl: ns}} for this (msg, op)."""
    variants = [v for v in category["variants"] if v in results]
    if not variants:
        return

    pretty = [category["labels"].get(v, v) for v in variants]
    datasets = []
    has_data = False
    for impl in IMPL_ORDER:
        vals = [results[v].get(impl) for v in variants]
        if any(x is not None for x in vals):
            has_data = True
        datasets.append((impl, vals))
    if not has_data:
        return

    op_display = OP_DISPLAY.get(op, op)
    f.write(f"**{category['display']}: {op_display} latency by {category['axis']}**\n\n")
    if op == "deserialize" and category["desc"]:
        f.write(f"_{category['desc']}_\n\n")
    f.write(_build_chart(f"{category['display']}/{op_display}", pretty, datasets))

    # Numeric values
    f.write("<details>\n")
    f.write(f"<summary>Numeric values — {category['display']} {op_display} (click to expand)</summary>\n\n")
    cols = ["Variant"] + [IMPL_LABELS[i] for i in IMPL_ORDER]
    cols += ["cdds / native", "pycdr2 / native"]
    f.write("| " + " | ".join(cols) + " |\n")
    f.write("|" + "|".join(["---"] * len(cols)) + "|\n")
    for v, label in zip(variants, pretty):
        n = results[v].get("native")
        c = results[v].get("cyclonedds")
        p = results[v].get("pycdr2")
        cells = [
            label,
            _format_time(n),
            _format_time(c),
            _format_time(p),
            f"**{c/n:,.0f}×**" if (c and n) else "—",
            f"**{p/n:,.0f}×**" if (p and n) else "—",
        ]
        f.write("| " + " | ".join(cells) + " |\n")
    f.write("\n</details>\n\n")


def render_light_section(f, light_results: dict):
    """`light_results`: {msg_key: {op: {impl: ns}}}."""
    if not light_results:
        return
    f.write("**Light message types: encode / decode round-trip**\n\n")
    f.write(
        "_Small `CdrFixed` structs and `Header` (geometry primitives, time "
        "stamps, color). Costs are dominated by Python attribute access and "
        "per-call interpreter overhead, not the serialization work itself — "
        "speedup ratios are smaller than the heavy types and absolute times "
        "are sub-microsecond._\n\n"
    )
    f.write("<details>\n")
    f.write("<summary>Numeric values — light types (click to expand)</summary>\n\n")
    cols = ["Message", "Op"] + [IMPL_LABELS[i] for i in IMPL_ORDER] + ["cdds / native", "pycdr2 / native"]
    f.write("| " + " | ".join(cols) + " |\n")
    f.write("|" + "|".join(["---"] * len(cols)) + "|\n")
    for msg_key, msg_display in LIGHT_TYPES_ORDER:
        if msg_key not in light_results:
            continue
        for op in ("serialize", "deserialize"):
            r = light_results[msg_key].get(op)
            if not r:
                continue
            n = r.get("native")
            c = r.get("cyclonedds")
            p = r.get("pycdr2")
            row = [
                msg_display, OP_DISPLAY.get(op, op),
                _format_time(n), _format_time(c), _format_time(p),
                f"**{c/n:,.0f}×**" if (c and n) else "—",
                f"**{p/n:,.0f}×**" if (p and n) else "—",
            ]
            f.write("| " + " | ".join(row) + " |\n")
    f.write("\n</details>\n\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    if len(sys.argv) != 2:
        print("Usage: render_python_benchmarks.py <results_dir>", file=sys.stderr)
        sys.exit(1)

    results_dir = Path(sys.argv[1]).resolve()
    if not results_dir.is_dir():
        print(f"error: {results_dir} is not a directory", file=sys.stderr)
        sys.exit(1)

    impl_data = {impl: parse_pytest_json(results_dir / f"{impl}.json") for impl in IMPL_ORDER}

    sys_meta = {}
    sys_path = results_dir / "system.json"
    if sys_path.exists():
        with sys_path.open() as f:
            sys_meta = json.load(f)

    # Index heavy:  results[(msg, op)][variant][impl] = ns
    # Index light:  light[msg][op][impl] = ns
    heavy_msg_keys = {c["msg"] for c in HEAVY_CATEGORIES}
    heavy: dict = {}
    light: dict = {}
    for impl, benches in impl_data.items():
        for name, ns in benches.items():
            parsed = parse_test_name(name)
            if not parsed:
                continue
            msg, op, variant = parsed
            if msg in heavy_msg_keys and variant is not None:
                heavy.setdefault((msg, op), {}).setdefault(variant, {})[impl] = ns
            elif variant is None:
                light.setdefault(msg, {}).setdefault(op, {})[impl] = ns

    # Output
    f = sys.stdout

    f.write("# Python CDR Benchmarks\n\n")
    f.write("## Summary\n\n")
    target_host = sys_meta.get("target_host", "(unspecified)")
    py_version = (sys_meta.get("python_version") or "(unspecified)").strip()
    timestamp = sys_meta.get("timestamp", "(unspecified)")
    git_rev = sys_meta.get("git_rev", "(unspecified)")
    f.write(f"**Target Hardware:** {target_host}  \n")
    f.write(f"**Python:** {py_version}  \n")
    f.write(f"**Last updated:** {timestamp}  \n")
    f.write(f"**Git revision:** `{git_rev[:12]}`\n\n")
    f.write(
        "EdgeFirst Python bindings (pyo3) vs. **pycdr2** (pure Python) and "
        "**cyclonedds-python** (C++-backed bindings). All three "
        "implementations encode/decode identical CDR1 LE messages with the "
        "fixture shapes defined in [`benches/python/shapes.py`](benches/python/shapes.py), "
        "which mirror [`benches/cpp/common.hpp`](benches/cpp/common.hpp) — the C++ "
        "comparison lives in [BENCHMARKS.md](BENCHMARKS.md).\n\n"
    )
    f.write(
        "> Lower is better. Each chart shows three bars per row — "
        "**blue = edgefirst (pyo3)**, **emerald = cyclone-dds-py**, "
        "**violet = pycdr2**. Charts use a logarithmic x-axis when the "
        "dynamic range exceeds ~100×; per-bar labels are suppressed in "
        "favour of the click-to-expand numeric tables. Regenerate locally "
        "with `./benches/python/benchmark.sh --target <ssh-host> --render`.\n\n"
    )

    impls_with_data = [i for i in IMPL_ORDER if impl_data[i]]
    fixture_count = len(impl_data["native"]) if impl_data["native"] else \
        max((len(impl_data[i]) for i in IMPL_ORDER), default=0)
    total = sum(len(impl_data[i]) for i in IMPL_ORDER)
    impls_present = ", ".join(IMPL_LABELS[i] for i in impls_with_data)
    f.write(f"**Coverage:** {fixture_count} fixtures × "
            f"{len(impls_with_data)} implementations ({impls_present}) "
            f"= {total} measurements.\n\n")
    impls_missing = [i for i in IMPL_ORDER if not impl_data[i]]
    if impls_missing:
        missing_lbls = ", ".join(IMPL_LABELS[i] for i in impls_missing)
        f.write(f"> **Note:** {missing_lbls} not measured in this run; "
                f"the corresponding columns are empty.\n>\n"
                f"> `cyclonedds-python` requires the system Cyclone DDS C++ "
                f"library at a version matching the Python bindings' ABI. On "
                f"Debian Trixie aarch64 the apt package "
                f"(`cyclonedds-dev` 0.10.5 with the t64 rebuild) ships a "
                f"slightly different API than every published "
                f"`cyclonedds-python` release: 0.10.2 hits a `const "
                f"dds_typeid_t` signature mismatch, 0.10.4/0.10.5/11.0.1 hit "
                f"`dds/ddsi/ddsi_radmin.h` (an internal header the apt "
                f"package strips). To bench against `cyclonedds-python`, "
                f"build Cyclone DDS C++ from a tag matching one of those "
                f"Python releases (`git checkout 0.10.2` on the C++ side) "
                f"and point `CYCLONEDDS_HOME` at the resulting install "
                f"prefix, then re-run with "
                f"`--impl cyclonedds --run-only --render`.\n\n")

    # Heavy types — one section per (category, op)
    for category in HEAVY_CATEGORIES:
        for op in ("deserialize", "serialize"):
            results = heavy.get((category["msg"], op), {})
            if results:
                render_heavy_section(f, category, op, results)

    # Light types
    render_light_section(f, light)

    # Footer
    f.write("---\n\n")
    f.write("<details>\n<summary>Target system metadata</summary>\n\n")
    f.write("```\n")
    if sys_meta:
        f.write(f"timestamp:      {sys_meta.get('timestamp')}\n")
        f.write(f"git_rev:        {sys_meta.get('git_rev')}\n")
        f.write(f"target_host:    {sys_meta.get('target_host')}\n")
        f.write(f"python_version: {sys_meta.get('python_version')}\n")
        f.write(f"uname:          {sys_meta.get('uname')}\n")
        f.write("--- /proc/cpuinfo (head -30) ---\n")
        f.write(sys_meta.get("cpuinfo_head30", ""))
    f.write("\n```\n\n</details>\n")


if __name__ == "__main__":
    main()
