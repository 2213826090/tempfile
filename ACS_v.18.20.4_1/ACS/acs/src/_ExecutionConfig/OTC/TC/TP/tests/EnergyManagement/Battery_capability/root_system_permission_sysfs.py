#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.energy import Energy

class root_system_permission_sysfs(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.energy = Energy()
        self.energy.adb_root()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_root_system_permission_sysfs(self):

        print "[RunTest]: %s" % self.__str__()
        cmd1 = "cat /sys/class/power_supply/dollar_cove_battery/uevent"
        cmd2 = "cat /sys/class/power_supply/max170xx_battery/uevent"
        cmd3 = "cat /sys/class/power_supply/intel_fuel_gauge/uevent"
        cmd4 = "cat /sys/class/power_supply/battery/uevent"
        product = self.energy.get_product()
        if "cht_mrd" in product:
            result = self.energy.testDevice.adb_cmd_capture_msg(cmd1)
        elif "cht_ffd" in product or "broxton" in product:
            result = self.energy.testDevice.adb_cmd_capture_msg(cmd2)
        elif "cht_cr" in product:
            result = self.energy.testDevice.adb_cmd_capture_msg(cmd3)
        else:
            result = self.energy.testDevice.adb_cmd_capture_msg(cmd4)
        #print result
        if "POWER_SUPPLY_NAME" not in result:
            assert False, "Root System permission Sysfs Failed."

