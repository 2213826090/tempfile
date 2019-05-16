import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.settings import BatterySetting
from testlib.em.basic_ui import Notification

class BatteryShowPercentage(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.battery = BatterySetting()
        self.battery.set_screen_status("on")
        self.battery.unlock_screen()
        super(BatteryShowPercentage, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(BatteryShowPercentage, self).tearDown()

    def test_battery_show_percentage_in_settings(self):
        """
        Test battery show percentage in settings
        """
        print "[RunTest]: %s" % self.__str__()
        level_setting = self.battery.get_battery_level()
        level_drop_menu = Notification().get_battery_level()
        assert abs(level_drop_menu - level_setting) < 2

