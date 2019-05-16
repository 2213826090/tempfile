import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.energy import Energy
from testlib.em.settings import BatterySetting, DisplaySetting
from testlib.em.tools import get_tmp_dir, remove_tmp_dir

class PowerShowBatteryStatus(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.energy = Energy()
        self.energy.adb_root()
        self.tmp_dir = get_tmp_dir()
        self.energy.set_screen_status("on")
        self.energy.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        super(PowerShowBatteryStatus, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        remove_tmp_dir(self.tmp_dir)
        super(PowerShowBatteryStatus, self).tearDown()

    def test_power_show_battery_status(self):
        """
        Test power show battery status
        """
        print "[RunTest]: %s" % self.__str__()


        uidump = os.path.join(self.tmp_dir, "uidump.xml")
        BatterySetting().launch()
        self.energy.capture_uidump_with_charging(uidump_file = uidump, charger_type = None)
        status = self.energy.get_battery_status_from_uidump(uidump)
        print status
        assert not ("Charging over" in status or "Full" in status)

