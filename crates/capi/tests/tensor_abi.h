/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.
 *
 * Tensor dtype and storage_kind codes for the C and C++ tests.
 *
 * The EdgeFirst HAL Modular Tensor ABI is the only authority for these codes,
 * and the names below are the ones HAL's <edgefirst/tensor.h> declares, so
 * that header can replace this one. The library carries the codes and never
 * interprets them, so it does not declare them.
 *
 * The golden-fixture tests decode testdata/cdr, whose codes
 * crates/schemas/tests/cdr_golden.rs checks against the ABI enums by name,
 * so a wrong value here fails those tests.
 */
#ifndef EDGEFIRST_SCHEMAS_TESTS_TENSOR_ABI_H
#define EDGEFIRST_SCHEMAS_TESTS_TENSOR_ABI_H

#ifndef EF_DTYPE_U8
#define EF_DTYPE_U8 0u
#define EF_DTYPE_I16 3u
#endif

#ifndef EF_STORAGE_KIND_MEM
#define EF_STORAGE_KIND_MEM 0u
#define EF_STORAGE_KIND_DMA_BUF 2u
#endif

#endif /* EDGEFIRST_SCHEMAS_TESTS_TENSOR_ABI_H */
