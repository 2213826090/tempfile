#! /usr/bin/env python
# coding:utf-8

import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.constants_def import SDP, CDP, DCP
from testlib.em.energy import Energy
from testlib.em.settings import DisplaySetting, BatterySetting

class Charging(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.energy = Energy()
        self.energy.adb_root()
        self.energy.set_screen_status("on")
        self.energy.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        #self.energy.testDevice.close_background_apps()
        super(Charging, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def check_screen_off_connect_charger(self, each_cycle):
        from testlib.em.apps import EMToolsScreen
        from testlib.em.charging import get_charging_result
        emtools = EMToolsScreen()
        emtools.install()
        hist = get_charging_result(emtools, each_cycle, 1)
        assert hist.count("ON") == 3

    def test_screen_off_connect_SDP_charger(self):
        """
        Screen off connect SDP charger
        """
        print "[RunTest]: %s" % self.__str__()
        each_cycle = ((SDP, 20, 20),)
        self.check_screen_off_connect_charger(each_cycle)

    def test_screen_off_connect_CDP_charger(self):
        """
        Screen off connect CDP charger
        """
        print "[RunTest]: %s" % self.__str__()
        each_cycle = ((CDP, 20, 20),)
        self.check_screen_off_connect_charger(each_cycle)

    def test_screen_off_connect_DCP_charger(self):
        """
        Screen off connect DCP charger
        """
        print "[RunTest]: %s" % self.__str__()
        each_cycle = ((DCP, 20, 20),)
        self.check_screen_off_connect_charger(each_cycle)

    def test_screen_off_connect_hub_charger(self):
        """
        Screen off connect HUB charger(SDP hub)
        """
        print "[RunTest]: %s" % self.__str__()
        each_cycle = ((SDP, 20, 20),)
        self.check_screen_off_connect_charger(each_cycle)

    def test_screen_wake_after_usb_plugged(self):
        """
        Screen wake after usb plugged
        """
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.basic_ui import LockScreenUI
        from testlib.em.crop_battery_icon import CropBatteryImage
        from testlib.em.tools import get_tmp_dir, remove_tmp_dir
        from testlib.em.usb_cut import USBCut
        lockscreen = LockScreenUI()
        lockscreen.lock_screen()
        USBCut().cut(20)
        assert self.energy.get_screen_status()
        tmp_dir = get_tmp_dir()
        screenshot = os.path.join(tmp_dir, "screenshot.png")
        rect = lockscreen.get_battery_icon_rect()
        self.energy.set_screen_status("on")
        time.sleep(2)
        self.energy.capture_screen(screenshot)
        crop_battery = CropBatteryImage(screenshot, rect)
        crop_battery.check_charging_status_by_icon(True)
        remove_tmp_dir(tmp_dir)
        time.sleep(20)
        self.energy.verify_screen_status("off")

    def test_sdp_charger_insertion_when_device_on(self):
        """
        SDP charger insertion when device is on
        """
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.tools import get_tmp_dir, remove_tmp_dir
        from testlib.em.crop_battery_icon import CropBatteryImage
        rect = BatterySetting().get_status_bar_rect()
        tmp_dir = get_tmp_dir()
        screenshot = os.path.join(tmp_dir, "screenshot.png")
        self.energy.capture_screen(screenshot)
        crop_battery = CropBatteryImage(screenshot, rect)
        crop_battery.check_charging_status_by_icon(True)
        remove_tmp_dir(tmp_dir)

    def test_charged_by_usb_cable(self):
        """
        Test charged by usb cable
        """
        print "[RunTest]: %s" % self.__str__()
        battery = BatterySetting()
        battery.launch()
        status = battery.get_battery_status()
        print status
        assert "USB" in status

    def test_power_show_charging_icon(self):
        """
        Test power show charging icon
        """
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.tools import get_tmp_dir, remove_tmp_dir
        from testlib.em.crop_battery_icon import CropBatteryImage
        import os
        tmp_dir = get_tmp_dir()
        screenshot = os.path.join(tmp_dir, "screenshot.png")
        rect = DisplaySetting().get_status_bar_rect()
        self.energy.capture_screen(screenshot)
        crop_battery = CropBatteryImage(screenshot, rect)
        crop_battery.crop_battery()
        crop_battery.check_charging_status_by_icon(True)
        remove_tmp_dir(tmp_dir)

    def test_power_show_charging_status(self):
        """
        Test power show charging status
        """
        print "[RunTest]: %s" % self.__str__()
        battery = BatterySetting()
        status = battery.get_battery_status()
        if "Full" in status:
            return
        assert "Charging" in status

    def test_power_show_charging_percentage(self):
        """
        Test power show charging percentage
        """
        print "[RunTest]: %s" % self.__str__()

        level1 = self.energy.get_battery_info()["level"]
        self.energy.set_screen_status("off")
        for i in range(100):
            print "cycle:", i + 1
            level2 = self.energy.get_battery_info()["level"]
            if level2 > level1:
                return
            time.sleep(60)
        self.energy.set_screen_status("on")
        self.energy.unlock_screen()
        level2 = self.energy.get_battery_info()["level"]
        assert level2 > level1

    def test_SDP_charging(self):
        """
        SDP charging
        """
        print "[RunTest]: %s" % self.__str__()
        battery = BatterySetting()
        status = battery.get_battery_status()
        if "Full" in status:
            return
        assert "USB" in status

    def test_DCP_charging(self):
        """
        DCP charging
        """
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import EMToolsCharger
        from testlib.em.charging import get_charging_result
        each_cycle = ((DCP, 2, 2),)
        hist = get_charging_result(EMToolsCharger(), each_cycle, 1)
        print hist
        assert DCP in hist

    def test_CDP_charging(self):
        """
        CDP charging
        """
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import EMToolsCharger
        from testlib.em.charging import get_charging_result
        each_cycle = ((CDP, 2, 2),)
        hist = get_charging_result(EMToolsCharger(), each_cycle, 1)
        print hist
        assert CDP in hist

    def test_check_AC_charger(self):
        """
        to check AC charger/ Wall charger current value
        """
        print "[RunTest]: %s" % self.__str__()
        from testlib.em.apps import EMToolsCharger
        from testlib.em.charging import get_charging_result
        each_cycle = ((DCP, 2, 2),)
        hist = get_charging_result(EMToolsCharger(), each_cycle, 1)
        print hist
        assert DCP in hist

