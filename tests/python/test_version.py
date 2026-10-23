# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for ``edgefirst.schemas.__version__``.

The module reports the PEP 440 spelling of the Cargo version, which is the
version maturin publishes for the wheel, so the attribute and the installed
distribution metadata must always agree.
"""

from importlib.metadata import version

import pytest

import edgefirst.schemas as schemas


def test_version_is_a_string():
    assert isinstance(schemas.__version__, str)


def test_version_matches_distribution_metadata():
    assert schemas.__version__ == version("edgefirst-schemas")


def test_version_is_normalized_pep440():
    packaging_version = pytest.importorskip("packaging.version")
    assert str(packaging_version.Version(schemas.__version__)) == schemas.__version__
