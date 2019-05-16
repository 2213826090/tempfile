from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch



class IconLongPress2Min(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        print
        print "[Setup]: %s" % self._test_name
        self.touch.enable_devoption_touch_location()
        super(IconLongPress2Min, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(IconLongPress2Min, self).tearDown()

    def test_icon_long_press_2min(self):
        print "[RunTest]: %s" % self.__str__()
        press_2min = 1000 * 5 #6
        self.touch.check_icon_long_press_build_comm("Settings", press_2min)
