/**
 * @file test_mavros_msgs.cpp
 * @brief Catch2 tests for mavros_msgs C++ View classes
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.
 */

#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

namespace ef = edgefirst::schemas;

// ============================================================================
// MavrosAltitudeView
// ============================================================================

TEST_CASE("MavrosAltitudeView from_cdr empty returns error", "[mavros][altitude]") {
    auto result = ef::MavrosAltitudeView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("MavrosAltitudeView from_cdr invalid returns error", "[mavros][altitude]") {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    auto result = ef::MavrosAltitudeView::from_cdr({bad, sizeof(bad)});
    REQUIRE_FALSE(result.has_value());
}

// ============================================================================
// MavrosVfrHudView
// ============================================================================

TEST_CASE("MavrosVfrHudView from_cdr empty returns error", "[mavros][vfrhud]") {
    auto result = ef::MavrosVfrHudView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("MavrosVfrHudView from_cdr invalid returns error", "[mavros][vfrhud]") {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    auto result = ef::MavrosVfrHudView::from_cdr({bad, sizeof(bad)});
    REQUIRE_FALSE(result.has_value());
}

// ============================================================================
// MavrosEstimatorStatusView
// ============================================================================

TEST_CASE("MavrosEstimatorStatusView from_cdr empty returns error", "[mavros][estimator]") {
    auto result = ef::MavrosEstimatorStatusView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("MavrosEstimatorStatusView from_cdr invalid returns error", "[mavros][estimator]") {
    uint8_t bad[] = {0x01, 0x02};
    auto result = ef::MavrosEstimatorStatusView::from_cdr({bad, sizeof(bad)});
    REQUIRE_FALSE(result.has_value());
}

// ============================================================================
// MavrosExtendedStateView
// ============================================================================

TEST_CASE("MavrosExtendedStateView from_cdr empty returns error", "[mavros][extstate]") {
    auto result = ef::MavrosExtendedStateView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("MavrosExtendedStateView constants", "[mavros][extstate]") {
    CHECK(ef::MavrosExtendedStateView::VTOL_STATE_UNDEFINED == 0);
    CHECK(ef::MavrosExtendedStateView::VTOL_STATE_FW == 4);
    CHECK(ef::MavrosExtendedStateView::LANDED_STATE_IN_AIR == 2);
}

// ============================================================================
// MavrosSysStatusView
// ============================================================================

TEST_CASE("MavrosSysStatusView from_cdr empty returns error", "[mavros][sysstatus]") {
    auto result = ef::MavrosSysStatusView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("MavrosSysStatusView from_cdr invalid returns error", "[mavros][sysstatus]") {
    uint8_t bad[] = {0xFF, 0xFF, 0xFF, 0xFF};
    auto result = ef::MavrosSysStatusView::from_cdr({bad, sizeof(bad)});
    REQUIRE_FALSE(result.has_value());
}

// ============================================================================
// MavrosStateView
// ============================================================================

TEST_CASE("MavrosStateView from_cdr empty returns error", "[mavros][state]") {
    auto result = ef::MavrosStateView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("MavrosStateView constants", "[mavros][state]") {
    CHECK(ef::MavrosStateView::MAV_STATE_UNINIT == 0);
    CHECK(ef::MavrosStateView::MAV_STATE_ACTIVE == 4);
    CHECK(ef::MavrosStateView::MAV_STATE_FLIGHT_TERMINATION == 8);
}

// ============================================================================
// MavrosStatusTextView
// ============================================================================

TEST_CASE("MavrosStatusTextView from_cdr empty returns error", "[mavros][statustext]") {
    auto result = ef::MavrosStatusTextView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("MavrosStatusTextView severity constants", "[mavros][statustext]") {
    CHECK(ef::MavrosStatusTextView::SEVERITY_EMERGENCY == 0);
    CHECK(ef::MavrosStatusTextView::SEVERITY_DEBUG == 7);
}

// ============================================================================
// MavrosGpsRawView
// ============================================================================

TEST_CASE("MavrosGpsRawView from_cdr empty returns error", "[mavros][gpsraw]") {
    auto result = ef::MavrosGpsRawView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("MavrosGpsRawView fix type constants", "[mavros][gpsraw]") {
    CHECK(ef::MavrosGpsRawView::GPS_FIX_TYPE_NO_GPS == 0);
    CHECK(ef::MavrosGpsRawView::GPS_FIX_TYPE_FIX_3D == 3);
    CHECK(ef::MavrosGpsRawView::GPS_FIX_TYPE_PPP == 8);
}

// ============================================================================
// MavrosTimesyncStatusView
// ============================================================================

TEST_CASE("MavrosTimesyncStatusView from_cdr empty returns error", "[mavros][timesync]") {
    auto result = ef::MavrosTimesyncStatusView::from_cdr({});
    REQUIRE_FALSE(result.has_value());
}

TEST_CASE("MavrosTimesyncStatusView from_cdr invalid returns error", "[mavros][timesync]") {
    uint8_t bad[] = {0xBA, 0xDC, 0x0D, 0xE0};
    auto result = ef::MavrosTimesyncStatusView::from_cdr({bad, sizeof(bad)});
    REQUIRE_FALSE(result.has_value());
}
