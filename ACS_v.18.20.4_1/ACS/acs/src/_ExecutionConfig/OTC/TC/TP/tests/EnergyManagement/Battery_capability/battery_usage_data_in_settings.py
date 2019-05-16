import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.settings import BatterySetting, DisplaySetting
from testlib.em.usb_cut import USBCut

class BatteryUsageData(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.battery = BatterySetting()
        self.battery.set_screen_status("on")
        self.battery.unlock_screen()
        DisplaySetting().set_sleep_mode("30 minutes")
        super(BatteryUsageData, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(BatteryUsageData, self).tearDown()

    def test_battery_usage_data(self):
        """
        Test battery usage data in settings
        """
        print "[RunTest]: %s" % self.__str__()
        for i in range(20):
            if self.battery.check_battery_usage_data():
                break
            USBCut().cut(30)
        assert self.battery.check_battery_usage_data()

