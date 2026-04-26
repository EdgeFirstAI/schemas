"""Tests for scripts/render_benchmarks.py."""

import subprocess
import sys
import tempfile
from pathlib import Path

SCRIPT = Path(__file__).parent.parent / "render_benchmarks.py"
FIXTURES = Path(__file__).parent / "fixtures"


def test_renders_cpp_results():
    """Golden-output test: cpp_only fixture must match expected.md exactly."""
    in_dir = FIXTURES / "cpp_only"
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), str(in_dir)],
        capture_output=True,
        text=True,
        check=True,
    )
    expected = (in_dir / "expected.md").read_text()
    assert proc.stdout == expected


def test_missing_inputs_no_crash(tmp_path):
    """A results dir with only one cpp JSON should render without crashing."""
    import shutil

    src = FIXTURES / "cpp_only" / "edgefirst.json"
    shutil.copy(src, tmp_path / "edgefirst.json")

    proc = subprocess.run(
        [sys.executable, str(SCRIPT), str(tmp_path)],
        capture_output=True,
        text=True,
        check=True,
    )
    # Should contain C++ section
    assert "C++ Benchmarks" in proc.stdout
    # Should not contain Rust or Python sections
    assert "Rust Benchmarks" not in proc.stdout
    assert "Python Benchmarks" not in proc.stdout


def test_empty_dir_no_crash(tmp_path):
    """A results dir with no recognized inputs should produce a header-only report."""
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), str(tmp_path)],
        capture_output=True,
        text=True,
        check=True,
    )
    assert "On-Target Benchmark Results" in proc.stdout
    assert "C++ Benchmarks" not in proc.stdout


def test_cpp_speedup_column():
    """Speedup column should appear when exactly 2 impls are present."""
    in_dir = FIXTURES / "cpp_only"
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), str(in_dir)],
        capture_output=True,
        text=True,
        check=True,
    )
    assert "Speedup" in proc.stdout


def test_cpp_section_has_charts():
    """Each phase section should include a QuickChart URL."""
    in_dir = FIXTURES / "cpp_only"
    proc = subprocess.run(
        [sys.executable, str(SCRIPT), str(in_dir)],
        capture_output=True,
        text=True,
        check=True,
    )
    assert "quickchart.io/chart" in proc.stdout
