/**
 * @file test_service.c
 * @brief Criterion tests for service types
 *
 * Note: ServiceHeader type has no FFI bindings in the current v2 buffer-view
 * API. This file is a placeholder that validates the header can be included
 * without errors.
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

// ============================================================================
// ServiceHeader has no v2 FFI bindings
// ============================================================================

Test(service, placeholder) {
    // ServiceHeader is not exported via the v2 FFI.
    // This test validates that the header compiles cleanly.
    cr_assert(1, "Service test placeholder compiles");
}
