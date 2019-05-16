#! /usr/bin/env python
# coding:utf-8

import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.energy import Energy
from testlib.em.tools import get_tmp_dir
from testlib.em.usb_cut import USBCut
from testlib.em.crop_battery_icon import CropBatteryImage
from testlib.em.settings import Settings

class LowBatteryIcon(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.energy = Energy()
        self.energy.adb_root()
        self.energy.set_screen_status("on")
        self.energy.unlock_screen()
        super(LowBatteryIcon, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(LowBatteryIcon, self).tearDown()

    def check_low_battery_icon(self, level):
        tmp_dir = get_tmp_dir()
        screenshot = os.path.join(tmp_dir, "screenshot.png")
        rect = Settings().get_status_bar_rect()
        # try 5 times
        retry = 5
        for i in range(retry):
            try:
                self.energy.set_virtual_battery_level(level)
                self.energy.capture_screen(screenshot)
                crop_battery = CropBatteryImage(screenshot, rect)
                crop_battery.crop_battery()
                crop_battery.check_red_exclamation_in_battery_icon(True)
                break
            except Exception as e:
                print e.message
        else:
            assert False
        USBCut().cut(3)

    def test_battery_icon_0_percent(self):
        """
        Check battery icon 0 percent
        """
        print "[RunTest]: %s" % self.__str__()
        self.check_low_battery_icon(0)

    def test_battery_icon_4_percent(self):
        """
        Check battery icon 4 percent
        """
        print "[RunTest]: %s" % self.__str__()
        self.check_low_battery_icon(4)

