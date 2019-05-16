#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl

class PowerSupplyCharging(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def get_uevent(self, name = "usb_charger", charging = True):
        if name == "usb_charger" and charging == True:
            uevent = self.emImpl.get_power_supply_uevent(uevent_type = name)
        else:
            uevent_path = "/sys/class/power_supply/%s/uevent" % name
            uevent_copy = "/mnt/sdcard/uevent"
            self.emImpl.copy_file_delay(uevent_path, uevent_copy, delay_time = 10)
            if charging:
                self.emImpl.enable_dcp_charging()
            else:
                self.emImpl.set_three_way_cutter_usb(0)
            time.sleep(12)
            self.emImpl.three_way_cutter_reconnect_sdp()
            uevent = self.emImpl.get_power_supply_uevent(uevent_path = uevent_copy)
        return uevent

    def test_power_supply_sdp(self):
        """
        Test power supply charge discharge sdp
        """
        print "[RunTest]: %s" % self.__str__()

        uevent = self.get_uevent(name = "usb_charger", charging = True)
        assert uevent["POWER_SUPPLY_NAME"] == "usb_charger"
        assert uevent["POWER_SUPPLY_TYPE"] == "USB"

        uevent = self.get_uevent(name = "usb_charger", charging = False)
        assert uevent["POWER_SUPPLY_PRESENT"] == "0"
        assert uevent["POWER_SUPPLY_ONLINE"] == "0"

    def test_power_supply_dcp(self):
        """
        Test power supply charge discharge dcp
        """
        print "[RunTest]: %s" % self.__str__()

        uevent = self.get_uevent(name = "ac_charger", charging = True)
        assert uevent["POWER_SUPPLY_NAME"] == "ac_charger"
        assert uevent["POWER_SUPPLY_TYPE"] == "Mains"

        uevent = self.get_uevent(name = "ac_charger", charging = False)
        assert uevent["POWER_SUPPLY_PRESENT"] == "0"
        assert uevent["POWER_SUPPLY_ONLINE"] == "0"

