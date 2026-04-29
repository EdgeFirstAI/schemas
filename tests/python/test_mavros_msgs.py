# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for `edgefirst.schemas.mavros_msgs`."""

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.mavros_msgs import (
    Altitude,
    VfrHud,
    EstimatorStatus,
    ExtendedState,
    SysStatus,
    State,
    StatusText,
    GpsRaw,
    TimesyncStatus,
)
from edgefirst.schemas.std_msgs import Header


def _header(frame_id: str = "base_link") -> Header:
    return Header(stamp=Time(100, 500), frame_id=frame_id)


class TestAltitude:
    def test_round_trip(self):
        h = _header()
        msg = Altitude(h, monotonic=1.5, amsl=100.0, local=50.0,
                       relative=25.0, terrain=75.0, bottom_clearance=10.0)
        restored = Altitude.from_cdr(msg.to_bytes())
        assert restored.frame_id == "base_link"
        assert restored.stamp.sec == 100
        assert restored.stamp.nanosec == 500
        assert restored.monotonic == 1.5
        assert restored.amsl == 100.0
        assert restored.local == 50.0
        assert restored.relative == 25.0
        assert restored.terrain == 75.0
        assert restored.bottom_clearance == 10.0

    def test_defaults(self):
        msg = Altitude(_header())
        assert msg.monotonic == 0.0
        assert msg.amsl == 0.0


class TestVfrHud:
    def test_round_trip(self):
        msg = VfrHud(_header(), airspeed=25.5, groundspeed=24.0,
                     heading=180, throttle=0.75, altitude=500.0, climb=2.5)
        restored = VfrHud.from_cdr(msg.to_bytes())
        assert restored.airspeed == 25.5
        assert restored.groundspeed == 24.0
        assert restored.heading == 180
        assert restored.throttle == 0.75
        assert restored.altitude == 500.0
        assert restored.climb == 2.5

    def test_negative_heading(self):
        msg = VfrHud(_header(), heading=-90)
        restored = VfrHud.from_cdr(msg.to_bytes())
        assert restored.heading == -90


class TestEstimatorStatus:
    def test_round_trip(self):
        msg = EstimatorStatus(
            _header(),
            attitude_status_flag=True,
            velocity_horiz_status_flag=True,
            pos_horiz_abs_status_flag=True,
            gps_glitch_status_flag=True,
        )
        restored = EstimatorStatus.from_cdr(msg.to_bytes())
        assert restored.attitude_status_flag is True
        assert restored.velocity_horiz_status_flag is True
        assert restored.velocity_vert_status_flag is False
        assert restored.pos_horiz_rel_status_flag is False
        assert restored.pos_horiz_abs_status_flag is True
        assert restored.pos_vert_abs_status_flag is False
        assert restored.pos_vert_agl_status_flag is False
        assert restored.const_pos_mode_status_flag is False
        assert restored.pred_pos_horiz_rel_status_flag is False
        assert restored.pred_pos_horiz_abs_status_flag is False
        assert restored.gps_glitch_status_flag is True
        assert restored.accel_error_status_flag is False

    def test_all_false(self):
        msg = EstimatorStatus(_header())
        restored = EstimatorStatus.from_cdr(msg.to_bytes())
        assert restored.attitude_status_flag is False


class TestExtendedState:
    def test_round_trip(self):
        msg = ExtendedState(_header(), vtol_state=4, landed_state=2)
        restored = ExtendedState.from_cdr(msg.to_bytes())
        assert restored.vtol_state == 4
        assert restored.landed_state == 2

    def test_defaults(self):
        msg = ExtendedState(_header())
        assert msg.vtol_state == 0
        assert msg.landed_state == 0


class TestSysStatus:
    def test_round_trip(self):
        msg = SysStatus(
            _header(),
            sensors_present=0xFFFF0000,
            sensors_enabled=0x00FF00FF,
            sensors_health=0x0F0F0F0F,
            load=500,
            voltage_battery=12600,
            current_battery=-150,
            battery_remaining=75,
            drop_rate_comm=10,
            errors_comm=2,
            errors_count1=1,
            errors_count2=2,
            errors_count3=3,
            errors_count4=4,
        )
        restored = SysStatus.from_cdr(msg.to_bytes())
        assert restored.sensors_present == 0xFFFF0000
        assert restored.sensors_enabled == 0x00FF00FF
        assert restored.sensors_health == 0x0F0F0F0F
        assert restored.load == 500
        assert restored.voltage_battery == 12600
        assert restored.current_battery == -150
        assert restored.battery_remaining == 75
        assert restored.drop_rate_comm == 10
        assert restored.errors_comm == 2
        assert restored.errors_count1 == 1
        assert restored.errors_count2 == 2
        assert restored.errors_count3 == 3
        assert restored.errors_count4 == 4


class TestState:
    def test_round_trip(self):
        msg = State(
            _header(),
            connected=True,
            armed=True,
            guided=False,
            manual_input=True,
            mode="OFFBOARD",
            system_status=4,
        )
        restored = State.from_cdr(msg.to_bytes())
        assert restored.connected is True
        assert restored.armed is True
        assert restored.guided is False
        assert restored.manual_input is True
        assert restored.mode == "OFFBOARD"
        assert restored.system_status == 4

    def test_empty_mode(self):
        msg = State(_header(), mode="")
        restored = State.from_cdr(msg.to_bytes())
        assert restored.mode == ""


class TestStatusText:
    def test_round_trip(self):
        msg = StatusText(_header(), severity=6, text="Disarming motors")
        restored = StatusText.from_cdr(msg.to_bytes())
        assert restored.severity == 6
        assert restored.text == "Disarming motors"

    def test_empty_text(self):
        msg = StatusText(_header(), severity=0, text="")
        restored = StatusText.from_cdr(msg.to_bytes())
        assert restored.text == ""


class TestGpsRaw:
    def test_round_trip(self):
        msg = GpsRaw(
            _header(),
            fix_type=3,
            lat=473977400,
            lon=85455060,
            alt=100000,
            eph=150,
            epv=200,
            vel=500,
            cog=18000,
            satellites_visible=12,
            alt_ellipsoid=100500,
            h_acc=1000,
            v_acc=1500,
            vel_acc=300,
            hdg_acc=100,
            yaw=27000,
            dgps_numch=5,
            dgps_age=3000,
        )
        restored = GpsRaw.from_cdr(msg.to_bytes())
        assert restored.fix_type == 3
        assert restored.lat == 473977400
        assert restored.lon == 85455060
        assert restored.alt == 100000
        assert restored.eph == 150
        assert restored.epv == 200
        assert restored.vel == 500
        assert restored.cog == 18000
        assert restored.satellites_visible == 12
        assert restored.alt_ellipsoid == 100500
        assert restored.h_acc == 1000
        assert restored.v_acc == 1500
        assert restored.vel_acc == 300
        assert restored.hdg_acc == 100
        assert restored.yaw == 27000
        assert restored.dgps_numch == 5
        assert restored.dgps_age == 3000

    def test_no_fix(self):
        msg = GpsRaw(_header(), fix_type=0)
        restored = GpsRaw.from_cdr(msg.to_bytes())
        assert restored.fix_type == 0
        assert restored.satellites_visible == 0


class TestTimesyncStatus:
    def test_round_trip(self):
        msg = TimesyncStatus(
            _header(),
            remote_timestamp_ns=1000000000,
            observed_offset_ns=-500000,
            estimated_offset_ns=-490000,
            round_trip_time_ms=1.5,
        )
        restored = TimesyncStatus.from_cdr(msg.to_bytes())
        assert restored.remote_timestamp_ns == 1000000000
        assert restored.observed_offset_ns == -500000
        assert restored.estimated_offset_ns == -490000
        assert restored.round_trip_time_ms == 1.5

    def test_zero_values(self):
        msg = TimesyncStatus(_header())
        restored = TimesyncStatus.from_cdr(msg.to_bytes())
        assert restored.remote_timestamp_ns == 0
        assert restored.observed_offset_ns == 0
        assert restored.estimated_offset_ns == 0
        assert restored.round_trip_time_ms == 0.0


class TestConstants:
    """Verify class-level constants are accessible at runtime."""

    def test_extended_state_constants(self):
        assert ExtendedState.VTOL_STATE_UNDEFINED == 0
        assert ExtendedState.VTOL_STATE_FW == 4
        assert ExtendedState.LANDED_STATE_ON_GROUND == 1
        assert ExtendedState.LANDED_STATE_IN_AIR == 2
        assert ExtendedState.LANDED_STATE_LANDING == 4

    def test_state_constants(self):
        assert State.MAV_STATE_UNINIT == 0
        assert State.MAV_STATE_STANDBY == 3
        assert State.MAV_STATE_ACTIVE == 4
        assert State.MAV_STATE_EMERGENCY == 6
        assert State.MAV_STATE_FLIGHT_TERMINATION == 8

    def test_status_text_constants(self):
        assert StatusText.EMERGENCY == 0
        assert StatusText.WARNING == 4
        assert StatusText.INFO == 6
        assert StatusText.DEBUG == 7

    def test_gps_raw_constants(self):
        assert GpsRaw.GPS_FIX_TYPE_NO_GPS == 0
        assert GpsRaw.GPS_FIX_TYPE_3D_FIX == 3
        assert GpsRaw.GPS_FIX_TYPE_RTK_FIXED == 6
        assert GpsRaw.GPS_FIX_TYPE_PPP == 8


class TestBufferTypes:
    """Verify from_cdr works with different buffer-like objects."""

    def test_from_cdr_with_memoryview(self):
        msg = Altitude(_header(), monotonic=1.5)
        restored = Altitude.from_cdr(memoryview(msg.to_bytes()))
        assert restored.monotonic == 1.5

    def test_from_cdr_with_bytearray(self):
        msg = State(_header(), mode="STABILIZE", armed=True)
        restored = State.from_cdr(bytearray(msg.to_bytes()))
        assert restored.mode == "STABILIZE"
        assert restored.armed is True
