import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.energy import Energy
from testlib.em.settings import DisplaySetting

class BacklightMitigation(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.energy = Energy()
        self.energy.set_screen_status("on")
        self.energy.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        super(BacklightMitigation, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(BacklightMitigation, self).tearDown()

    def test_backlight_mitigation_low_battery(self):
        """
        Test backlight mitigation low battery
        """
        print "[RunTest]: %s" % self.__str__()

        backlight1 = self.energy.get_actual_backlight()
        backlight2 = self.energy.get_actual_backlight_battery_saver()
        assert backlight1 > backlight2

