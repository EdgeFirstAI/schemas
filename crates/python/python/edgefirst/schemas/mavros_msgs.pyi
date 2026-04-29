# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Type stubs for ``edgefirst.schemas.mavros_msgs``."""

from __future__ import annotations

from . import BufferLike
from .builtin_interfaces import Time
from .std_msgs import Header

__all__ = [
    "Altitude",
    "VfrHud",
    "EstimatorStatus",
    "ExtendedState",
    "SysStatus",
    "State",
    "StatusText",
    "GpsRaw",
    "TimesyncStatus",
]


class Altitude:
    """``mavros_msgs.Altitude`` — altitude measurements."""

    def __init__(
        self,
        header: Header,
        monotonic: float = 0.0,
        amsl: float = 0.0,
        local: float = 0.0,
        relative: float = 0.0,
        terrain: float = 0.0,
        bottom_clearance: float = 0.0,
    ) -> None: ...

    @classmethod
    def from_cdr(cls, buf: BufferLike) -> Altitude: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def monotonic(self) -> float: ...
    @property
    def amsl(self) -> float: ...
    @property
    def local(self) -> float: ...
    @property
    def relative(self) -> float: ...
    @property
    def terrain(self) -> float: ...
    @property
    def bottom_clearance(self) -> float: ...
    @property
    def cdr_size(self) -> int: ...
    def to_bytes(self) -> bytes: ...


class VfrHud:
    """``mavros_msgs.VfrHud`` — VFR head-up display telemetry."""

    def __init__(
        self,
        header: Header,
        airspeed: float = 0.0,
        groundspeed: float = 0.0,
        heading: int = 0,
        throttle: float = 0.0,
        altitude: float = 0.0,
        climb: float = 0.0,
    ) -> None: ...

    @classmethod
    def from_cdr(cls, buf: BufferLike) -> VfrHud: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def airspeed(self) -> float: ...
    @property
    def groundspeed(self) -> float: ...
    @property
    def heading(self) -> int: ...
    @property
    def throttle(self) -> float: ...
    @property
    def altitude(self) -> float: ...
    @property
    def climb(self) -> float: ...
    @property
    def cdr_size(self) -> int: ...
    def to_bytes(self) -> bytes: ...


class EstimatorStatus:
    """``mavros_msgs.EstimatorStatus`` — EKF estimator health flags."""

    def __init__(
        self,
        header: Header,
        attitude_status_flag: bool = False,
        velocity_horiz_status_flag: bool = False,
        velocity_vert_status_flag: bool = False,
        pos_horiz_rel_status_flag: bool = False,
        pos_horiz_abs_status_flag: bool = False,
        pos_vert_abs_status_flag: bool = False,
        pos_vert_agl_status_flag: bool = False,
        const_pos_mode_status_flag: bool = False,
        pred_pos_horiz_rel_status_flag: bool = False,
        pred_pos_horiz_abs_status_flag: bool = False,
        gps_glitch_status_flag: bool = False,
        accel_error_status_flag: bool = False,
    ) -> None: ...

    @classmethod
    def from_cdr(cls, buf: BufferLike) -> EstimatorStatus: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def attitude_status_flag(self) -> bool: ...
    @property
    def velocity_horiz_status_flag(self) -> bool: ...
    @property
    def velocity_vert_status_flag(self) -> bool: ...
    @property
    def pos_horiz_rel_status_flag(self) -> bool: ...
    @property
    def pos_horiz_abs_status_flag(self) -> bool: ...
    @property
    def pos_vert_abs_status_flag(self) -> bool: ...
    @property
    def pos_vert_agl_status_flag(self) -> bool: ...
    @property
    def const_pos_mode_status_flag(self) -> bool: ...
    @property
    def pred_pos_horiz_rel_status_flag(self) -> bool: ...
    @property
    def pred_pos_horiz_abs_status_flag(self) -> bool: ...
    @property
    def gps_glitch_status_flag(self) -> bool: ...
    @property
    def accel_error_status_flag(self) -> bool: ...
    @property
    def cdr_size(self) -> int: ...
    def to_bytes(self) -> bytes: ...


class ExtendedState:
    """``mavros_msgs.ExtendedState`` — VTOL and landed state."""

    VTOL_STATE_UNDEFINED: int
    VTOL_STATE_TRANSITION_TO_FW: int
    VTOL_STATE_TRANSITION_TO_MC: int
    VTOL_STATE_MC: int
    VTOL_STATE_FW: int
    LANDED_STATE_UNDEFINED: int
    LANDED_STATE_ON_GROUND: int
    LANDED_STATE_IN_AIR: int
    LANDED_STATE_TAKEOFF: int
    LANDED_STATE_LANDING: int

    def __init__(
        self,
        header: Header,
        vtol_state: int = 0,
        landed_state: int = 0,
    ) -> None: ...

    @classmethod
    def from_cdr(cls, buf: BufferLike) -> ExtendedState: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def vtol_state(self) -> int: ...
    @property
    def landed_state(self) -> int: ...
    @property
    def cdr_size(self) -> int: ...
    def to_bytes(self) -> bytes: ...


class SysStatus:
    """``mavros_msgs.SysStatus`` — system health and power status.

    Field units follow the raw MAVLink SYS_STATUS message — no
    conversions are applied.  See
    `MAVLink SYS_STATUS <https://mavlink.io/en/messages/common.html#SYS_STATUS>`_.
    """

    def __init__(
        self,
        header: Header,
        sensors_present: int = 0,
        sensors_enabled: int = 0,
        sensors_health: int = 0,
        load: int = 0,
        voltage_battery: int = 0,
        current_battery: int = 0,
        battery_remaining: int = 0,
        drop_rate_comm: int = 0,
        errors_comm: int = 0,
        errors_count1: int = 0,
        errors_count2: int = 0,
        errors_count3: int = 0,
        errors_count4: int = 0,
    ) -> None: ...

    @classmethod
    def from_cdr(cls, buf: BufferLike) -> SysStatus: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def sensors_present(self) -> int:
        """Bitmask of onboard sensors present."""
        ...
    @property
    def sensors_enabled(self) -> int:
        """Bitmask of onboard sensors enabled."""
        ...
    @property
    def sensors_health(self) -> int:
        """Bitmask of onboard sensors reporting healthy."""
        ...
    @property
    def load(self) -> int:
        """Maximum CPU/MCU usage in per-mille (0–1000)."""
        ...
    @property
    def voltage_battery(self) -> int:
        """Battery voltage in millivolts (mV). UINT16_MAX if unknown."""
        ...
    @property
    def current_battery(self) -> int:
        """Battery current in centi-amperes (cA, 10 mA steps). -1 if unknown."""
        ...
    @property
    def battery_remaining(self) -> int:
        """Battery remaining, 0–100 (%). -1 if not estimated."""
        ...
    @property
    def drop_rate_comm(self) -> int:
        """Communication drop rate in per-mille."""
        ...
    @property
    def errors_comm(self) -> int:
        """Communication error count."""
        ...
    @property
    def errors_count1(self) -> int: ...
    @property
    def errors_count2(self) -> int: ...
    @property
    def errors_count3(self) -> int: ...
    @property
    def errors_count4(self) -> int: ...
    @property
    def cdr_size(self) -> int: ...
    def to_bytes(self) -> bytes: ...


class State:
    """``mavros_msgs.State`` — autopilot connection and mode state."""

    MAV_STATE_UNINIT: int
    MAV_STATE_BOOT: int
    MAV_STATE_CALIBRATING: int
    MAV_STATE_STANDBY: int
    MAV_STATE_ACTIVE: int
    MAV_STATE_CRITICAL: int
    MAV_STATE_EMERGENCY: int
    MAV_STATE_POWEROFF: int
    MAV_STATE_FLIGHT_TERMINATION: int

    def __init__(
        self,
        header: Header,
        connected: bool = False,
        armed: bool = False,
        guided: bool = False,
        manual_input: bool = False,
        mode: str = "",
        system_status: int = 0,
    ) -> None: ...

    @classmethod
    def from_cdr(cls, buf: BufferLike) -> State: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def connected(self) -> bool: ...
    @property
    def armed(self) -> bool: ...
    @property
    def guided(self) -> bool: ...
    @property
    def manual_input(self) -> bool: ...
    @property
    def mode(self) -> str: ...
    @property
    def system_status(self) -> int: ...
    @property
    def cdr_size(self) -> int: ...
    def to_bytes(self) -> bytes: ...


class StatusText:
    """``mavros_msgs.StatusText`` — status message with severity."""

    EMERGENCY: int
    ALERT: int
    CRITICAL: int
    ERROR: int
    WARNING: int
    NOTICE: int
    INFO: int
    DEBUG: int

    def __init__(
        self,
        header: Header,
        severity: int = 0,
        text: str = "",
    ) -> None: ...

    @classmethod
    def from_cdr(cls, buf: BufferLike) -> StatusText: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def severity(self) -> int: ...
    @property
    def text(self) -> str: ...
    @property
    def cdr_size(self) -> int: ...
    def to_bytes(self) -> bytes: ...


class GpsRaw:
    """``mavros_msgs.GpsRaw`` — raw GPS fix data.

    Field units follow the raw MAVLink GPS_RAW_INT message — no
    conversions are applied.  See
    `MAVLink GPS_RAW_INT <https://mavlink.io/en/messages/common.html#GPS_RAW_INT>`_.
    """

    GPS_FIX_TYPE_NO_GPS: int
    GPS_FIX_TYPE_NO_FIX: int
    GPS_FIX_TYPE_2D_FIX: int
    GPS_FIX_TYPE_3D_FIX: int
    GPS_FIX_TYPE_DGPS: int
    GPS_FIX_TYPE_RTK_FLOAT: int
    GPS_FIX_TYPE_RTK_FIXED: int
    GPS_FIX_TYPE_STATIC: int
    GPS_FIX_TYPE_PPP: int

    def __init__(
        self,
        header: Header,
        fix_type: int = 0,
        lat: int = 0,
        lon: int = 0,
        alt: int = 0,
        eph: int = 0,
        epv: int = 0,
        vel: int = 0,
        cog: int = 0,
        satellites_visible: int = 0,
        alt_ellipsoid: int = 0,
        h_acc: int = 0,
        v_acc: int = 0,
        vel_acc: int = 0,
        hdg_acc: int = 0,
        yaw: int = 0,
        dgps_numch: int = 0,
        dgps_age: int = 0,
    ) -> None: ...

    @classmethod
    def from_cdr(cls, buf: BufferLike) -> GpsRaw: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def fix_type(self) -> int:
        """GPS fix type (see ``GPS_FIX_TYPE_*`` constants)."""
        ...
    @property
    def lat(self) -> int:
        """Latitude in degrees × 10⁷ (degE7)."""
        ...
    @property
    def lon(self) -> int:
        """Longitude in degrees × 10⁷ (degE7)."""
        ...
    @property
    def alt(self) -> int:
        """Altitude above MSL in millimetres (mm)."""
        ...
    @property
    def eph(self) -> int:
        """GPS HDOP in cm. UINT16_MAX if unknown."""
        ...
    @property
    def epv(self) -> int:
        """GPS VDOP in cm. UINT16_MAX if unknown."""
        ...
    @property
    def vel(self) -> int:
        """GPS ground speed in cm/s. UINT16_MAX if unknown."""
        ...
    @property
    def cog(self) -> int:
        """Course over ground in centidegrees (0–35999). UINT16_MAX if unknown."""
        ...
    @property
    def satellites_visible(self) -> int:
        """Number of satellites visible. UINT8_MAX if unknown."""
        ...
    @property
    def alt_ellipsoid(self) -> int:
        """Altitude above WGS84 ellipsoid in millimetres (mm)."""
        ...
    @property
    def h_acc(self) -> int:
        """Horizontal position uncertainty in millimetres (mm)."""
        ...
    @property
    def v_acc(self) -> int:
        """Vertical position uncertainty in millimetres (mm)."""
        ...
    @property
    def vel_acc(self) -> int:
        """Speed uncertainty in mm/s."""
        ...
    @property
    def hdg_acc(self) -> int:
        """Heading uncertainty in centidegrees."""
        ...
    @property
    def yaw(self) -> int:
        """Yaw in centidegrees. 0 if unknown."""
        ...
    @property
    def dgps_numch(self) -> int:
        """Number of DGPS satellites."""
        ...
    @property
    def dgps_age(self) -> int:
        """Age of DGPS correction in milliseconds (ms)."""
        ...
    @property
    def cdr_size(self) -> int: ...
    def to_bytes(self) -> bytes: ...


class TimesyncStatus:
    """``mavros_msgs.TimesyncStatus`` — time synchronization status."""

    def __init__(
        self,
        header: Header,
        remote_timestamp_ns: int = 0,
        observed_offset_ns: int = 0,
        estimated_offset_ns: int = 0,
        round_trip_time_ms: float = 0.0,
    ) -> None: ...

    @classmethod
    def from_cdr(cls, buf: BufferLike) -> TimesyncStatus: ...

    @property
    def stamp(self) -> Time: ...
    @property
    def frame_id(self) -> str: ...
    @property
    def remote_timestamp_ns(self) -> int: ...
    @property
    def observed_offset_ns(self) -> int: ...
    @property
    def estimated_offset_ns(self) -> int: ...
    @property
    def round_trip_time_ms(self) -> float: ...
    @property
    def cdr_size(self) -> int: ...
    def to_bytes(self) -> bytes: ...
