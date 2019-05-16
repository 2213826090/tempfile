import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.basic_ui import Notification

class BatteryShowPercentage(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.noti = Notification()
        self.noti.set_screen_status("on")
        self.noti.unlock_screen()
        super(BatteryShowPercentage, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(BatteryShowPercentage, self).tearDown()

    def test_battery_show_percentage_on_home(self):
        """
        Test battery show percentage on home
        """
        print "[RunTest]: %s" % self.__str__()
        level = self.noti.get_battery_level()
        print level

