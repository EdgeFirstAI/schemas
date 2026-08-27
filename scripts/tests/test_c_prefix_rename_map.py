"""Tests for scripts/c_prefix_rename_map.py."""

import importlib.util
import sys
from pathlib import Path

SCRIPT = Path(__file__).parent.parent / "c_prefix_rename_map.py"


def _load():
    spec = importlib.util.spec_from_file_location("c_prefix_rename_map", SCRIPT)
    assert spec and spec.loader
    mod = importlib.util.module_from_spec(spec)
    sys.modules["c_prefix_rename_map"] = mod
    spec.loader.exec_module(mod)
    return mod


def test_sensor_vs_foxglove_compressed_image():
    m = _load()
    assert (
        m.rename_identifier("ros_compressed_image_from_cdr")
        == "sensor_msgs_compressed_image_from_cdr"
    )
    assert (
        m.rename_identifier("ros_foxglove_compressed_image_from_cdr")
        == "foxglove_msgs_compressed_image_from_cdr"
    )
    # Must not double the package token.
    assert "foxglove_msgs_foxglove_" not in m.rename_identifier(
        "ros_foxglove_compressed_image_t"
    )


def test_compressed_video_and_mavros_stem_cleanup():
    m = _load()
    assert (
        m.rename_identifier("ros_compressed_video_from_cdr")
        == "foxglove_msgs_compressed_video_from_cdr"
    )
    assert (
        m.rename_identifier("ros_foxglove_compressed_video_builder_new")
        == "foxglove_msgs_compressed_video_builder_new"
    )
    assert (
        m.rename_identifier("ros_mavros_altitude_from_cdr")
        == "mavros_msgs_altitude_from_cdr"
    )


def test_bytes_free_library_prefix():
    m = _load()
    assert m.rename_identifier("ros_bytes_free") == "edgefirst_schemas_bytes_free"


def test_longest_stem_wins():
    m = _load()
    assert (
        m.rename_identifier("ros_accel_with_covariance_stamped_builder_new")
        == "geometry_msgs_accel_with_covariance_stamped_builder_new"
    )
    assert m.rename_identifier("ros_accel_encode") == "geometry_msgs_accel_encode"


def test_rename_text_mixed():
    m = _load()
    text = (
        "ros_image_from_cdr(d, n); "
        "ros_bytes_free(p, n); "
        "ros_foxglove_compressed_image_t *v;"
    )
    out, count, unmapped = m.rename_text(text)
    assert not unmapped
    assert count == 3
    assert "sensor_msgs_image_from_cdr" in out
    assert "edgefirst_schemas_bytes_free" in out
    assert "foxglove_msgs_compressed_image_t" in out
