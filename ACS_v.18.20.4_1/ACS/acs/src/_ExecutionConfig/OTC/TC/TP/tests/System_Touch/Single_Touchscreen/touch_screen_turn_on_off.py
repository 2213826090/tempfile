from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch
import sys


class TouchScreenTurnOnOff(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        print
        print "[Setup]: %s" % self._test_name
        self.touch.check_home_ui_skip_got_it()
        #self.touch.start_RPCServer()
        super(TouchScreenTurnOnOff, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.touch.boot_up_completed_skip_boot_ui()
        super(TouchScreenTurnOnOff, self).tearDown()

    def test_touch_screen_cannot_wake_up_when_screen_off(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.check_touch_cannot_wake_up_when_screen_off()

    def test_touch_screen_turn_on_off_screen_response_10_times(self):
        print "[RunTest]: %s" % self.__str__()
        # get method self name
        fileName = sys._getframe().f_code.co_name
        for loop in range(10):
            self.touch.logger.info("")
            self.touch.logger.info("Loop %d/10 " % (loop + 1) + fileName)
            self.touch.check_touch_screen_response_comm()
            self.touch.screen_turn_on_off_comm()

    def test_touch_operation_after_screen_off_on_getevent_no_output(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.screen_on_off_getevent_no_output()

    def test_touch_screen_turn_on_off_with_fingers(self):
        #after screen off, hold your fingers on touch screen
        print "[RunTest]: %s" % self.__str__()
        self.touch.check_touch_screen_response_comm()
        self.touch.screen_turn_on_off_comm()
        product_name = self.touch.check_product()
        if "cht_mrd" in product_name:
            self.touch.check_touch_screen_response_comm()
        else:
            self.touch.check_single_touch_sendevent_screen_response_comm()
        self.touch.check_icon_long_press_build_comm("Settings", 2000)
        self.touch.check_swipe_vertical()
