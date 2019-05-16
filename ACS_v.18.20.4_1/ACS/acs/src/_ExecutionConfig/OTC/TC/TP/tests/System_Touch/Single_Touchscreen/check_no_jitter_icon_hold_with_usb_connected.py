from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch



class IconHoldWithUsb(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        print
        print "[Setup]: %s" % self._test_name
        self.touch.enable_devoption_touch_location()
        super(IconHoldWithUsb, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(IconHoldWithUsb, self).tearDown()

    def test_icon_hold_with_usb(self):
        print "[RunTest]: %s" % self.__str__()
        hold = 1000 * 1  # 6
        self.touch.check_icon_long_press_build_comm("Settings", hold)
