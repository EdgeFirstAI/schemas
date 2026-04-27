# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Tests for `edgefirst.schemas.rosgraph_msgs`."""

from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.rosgraph_msgs import Clock


class TestClock:
    def test_default_is_zero_time(self):
        c = Clock()
        assert c.clock == Time(0, 0)

    def test_round_trip(self):
        c = Clock(clock=Time(sec=1234567890, nanosec=987654321))
        restored = Clock.from_cdr(c.to_bytes())
        assert restored.clock.sec == 1234567890
        assert restored.clock.nanosec == 987654321
