import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.energy import Energy
from testlib.em.settings import DisplaySetting
from testlib.em.tools import get_tmp_dir, remove_tmp_dir
from testlib.em.crop_battery_icon import CropBatteryImage

class PowerShowBatteryIcon(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.energy = Energy()
        self.tmp_dir = get_tmp_dir()
        self.energy.set_screen_status("on")
        self.energy.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        super(PowerShowBatteryIcon, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        remove_tmp_dir(self.tmp_dir)
        super(PowerShowBatteryIcon, self).tearDown()

    def test_power_show_battery_icon(self):
        """
        Test power show battery icon
        """
        print "[RunTest]: %s" % self.__str__()
        screenshot = os.path.join(self.tmp_dir, "screenshot.png")
        rect = DisplaySetting().get_status_bar_rect()
        self.energy.capture_screen_not_charging(screenshot)
        crop_battery = CropBatteryImage(screenshot, rect)
        crop_battery.crop_battery()
        crop_battery.check_charging_status_by_icon(False)

