#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl
from testlib.util.common import g_common_obj

SDP = "USB"
CDP = "USB_CDP"
DCP = "USB_DCP"
MOS = "MOS"
COS = "COS"
FULL = 98

class ChargingLongTime(UIATestBase):
    """
    Connecting CDP charger in MOS and wait one night for 12 hours, check battery increase.
    """
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        g_common_obj.close_background_apps()
        super(ChargingLongTime, self).setUp()

    def tearDown(self):
        super(ChargingLongTime, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def check_charging_12h(self, charger_type, device_mode = MOS):
        level = self.emImpl.get_battery_level()
        if device_mode == COS:
            self.emImpl.power_off_device()
        else:
            self.emImpl.set_screen_status("off")
            time.sleep(5)
        # charge 12h
        if charger_type == CDP:
            self.emImpl.enable_cdp_charging()
        elif charger_type == DCP:
            self.emImpl.enable_dcp_charging()
        print "Sleep 12h."
        time.sleep(12 * 3600)
        if charger_type != SDP:
            self.emImpl.enable_sdp_charging()
            time.sleep(30)

        if device_mode == COS:
            self.emImpl.boot_up_device()
            self.emImpl.adb_root()
        level = self.emImpl.get_battery_level()
        assert level >= FULL, "charging for 12 hours is not full."

    def check_charging_1h(self, charger_type):
        level1 = self.emImpl.get_battery_level()
        self.emImpl.set_screen_status("off")
        time.sleep(5)
        if charger_type == CDP:
            self.emImpl.enable_cdp_charging()
        elif charger_type == DCP:
            self.emImpl.enable_dcp_charging()
        print "Sleep 1h."
        time.sleep(36)
        if charger_type != SDP:
            self.emImpl.enable_sdp_charging()
        level2 = self.emImpl.get_battery_level()
        assert level2 >= min(FULL, level1 + 10)

    def test_charging_CDP_MOS_12_hours(self):
        self.check_charging_12h(CDP, MOS)

    def test_charging_CDP_COS_12_hours(self):
        self.check_charging_12h(CDP, COS)

    def test_charging_DCP_MOS_12_hours(self):
        self.check_charging_12h(DCP, MOS)

    def test_charging_DCP_COS_12_hours(self):
        self.check_charging_12h(DCP, COS)

    def test_charging_SDP_MOS_12_hours(self):
        self.check_charging_12h(SDP, MOS)

    def test_charging_SDP_COS_12_hours(self):
        self.check_charging_12h(SDP, COS)

    def test_CDP_charging_1_hour(self):
        self.check_charging_1h(CDP)

    def test_DCP_charging_1_hour(self):
        self.check_charging_1h(DCP)

    def test_SDP_charging_1_hour(self):
        self.check_charging_1h(SDP)

