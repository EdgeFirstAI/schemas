# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tensor ``dtype`` and ``storage_kind`` codes for this repository's Python.

The EdgeFirst HAL Modular Tensor ABI is the only authority for these codes
(``edgefirst-tensor-abi``: ``EfDtype``, ``EfStorageKind``; ``edgefirst/tensor.h``:
``EF_DTYPE_*``, ``EF_STORAGE_KIND_*``). This package carries them and never
interprets them, so it does not declare them; this module is the one place
the Python tests and ``scripts/generate_cdr_testdata.py`` write them out.

``crates/schemas/tests/cdr_golden.rs`` checks every generated golden's codes
against the ABI enums by name, so a wrong value here fails CI.
"""

# EfDtype
DTYPE_U8 = 0
DTYPE_I8 = 1
DTYPE_U16 = 2
DTYPE_I16 = 3

# EfStorageKind
STORAGE_MEM = 0
STORAGE_DMABUF = 2
