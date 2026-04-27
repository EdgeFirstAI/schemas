# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Bench harness shared config.

Adds `benches/python` to sys.path so legacy pycdr2 modules import as
`legacy.<name>` regardless of how pytest is invoked.
"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
