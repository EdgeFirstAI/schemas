# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

# Vibration Message Interface - edgefirst_msgs/msg/Vibration
#
# Per-axis vibration magnitude with explicit measurement semantics
# (RMS/Peak/Peak-to-Peak), physical unit (accel / velocity / displacement),
# and integration band. Generalizes MAVLink VIBRATION, ArduPilot VIBE,
# PX4 vehicle_imu_status, and industrial ISO 10816/20816 broadband sensors.
#
# Producer convention for MAVLink bridges:
#   measurement_type = MEASUREMENT_RMS
#   unit             = UNIT_ACCEL_M_PER_S2
#   band_lower_hz    = NaN   (unknown band)
#   band_upper_hz    = NaN
#   vibration.{x,y,z} = msg.vibration_{x,y,z}
#   clipping         = [clipping_0, clipping_1, clipping_2]
#   header.frame_id  = "imu_link"  (MAVLink --frame-id-imu default)
#
# Rules:
#   - Unknown scalars: publish NaN (never 0.0).
#   - Unknown enums: use the *_UNKNOWN variant (value 0).
#   - Scalar broadband sensors populate vibration.x only and set
#     vibration.y = vibration.z = NaN.
#   - Multi-parameter publishers SHOULD publish one message per
#     parameter on distinct key suffixes (e.g. vibration/velocity,
#     vibration/accel).
#   - Measurement direction (radial vs axial) comes from header.frame_id
#     + TF, not an in-message field.
#   - Publishers without clipping source: clipping = [] (empty).
#     MAVLink publishers emit exactly 3 clipping entries.
#   - ISO 20816-1 integration windows: 10-1000 Hz general,
#     2-1000 Hz for low-speed machinery (<120 RPM).

# Field order is chosen by descending alignment (Vector3→f32→u8→seq)
# so every field after the Header sits at a constant CDR offset relative
# to `vibration`. This eliminates position-dependent internal padding and
# allows single-offset O(1) accessors in the generated bindings.

std_msgs/Header header

# Per-axis vibration magnitude in the chosen unit/statistic.
# Unknown axis components: NaN (not 0.0).
geometry_msgs/Vector3 vibration

# Integration band (inclusive lower, inclusive upper). NaN = unknown.
float32 band_lower_hz
float32 band_upper_hz

# Broadband statistic reported in `vibration`.
uint8 MEASUREMENT_UNKNOWN      = 0
uint8 MEASUREMENT_RMS          = 1  # Broadband RMS (MAVLink, ISO 10816)
uint8 MEASUREMENT_PEAK         = 2
uint8 MEASUREMENT_PEAK_TO_PEAK = 3
uint8 measurement_type

# Physical unit of the values in `vibration`.
uint8 UNIT_UNKNOWN           = 0
uint8 UNIT_ACCEL_M_PER_S2    = 1  # MAVLink VIBRATION
uint8 UNIT_ACCEL_G           = 2  # 1 g = 9.80665 m/s^2
uint8 UNIT_VELOCITY_MM_PER_S = 3  # ISO 10816 broadband velocity
uint8 UNIT_DISPLACEMENT_UM   = 4
uint8 UNIT_VELOCITY_IN_PER_S = 5
uint8 UNIT_DISPLACEMENT_MIL  = 6  # 1 mil = 25.4 um
uint8 unit

# Per-axis accelerometer saturation counters (monotonic). Empty when
# the source does not expose clipping data. MAVLink publishers emit
# exactly 3 entries (clipping_0/1/2).
uint32[] clipping
