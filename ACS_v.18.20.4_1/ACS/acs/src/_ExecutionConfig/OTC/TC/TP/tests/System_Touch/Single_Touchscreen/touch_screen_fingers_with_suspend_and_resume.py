from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch
from testlib.system_touch.multiple_touchscreen import MultiTouchScreen
import sys


class TouchScreenFingersWithSuspend(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.touch = SystemTouch()
        self.multi_ts = MultiTouchScreen()
        super(TouchScreenFingersWithSuspend, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.touch.boot_up_completed_skip_boot_ui()
        super(TouchScreenFingersWithSuspend, self).tearDown()

    def test_check_touch_point_before_enter_S3_and_resume_S3(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.enable_devoption_touch_location()
        self.touch.check_touch_screen_response_comm()
        self.multi_ts.check_touch_gesture_swipe_vertical_down_up_comm()
        self.touch.screen_turn_on_off_comm()
        self.touch.check_single_touch_sendevent_screen_response_comm()

    def test_touch_screen_suspend_and_resume_10_times(self):
        print "[RunTest]: %s" % self.__str__()
        # get method self name
        fileName = sys._getframe().f_code.co_name
        for loop in range(10):
            self.touch.logger.info("")
            self.touch.logger.info("Loop %d/10 " % (loop + 1) + fileName)
            self.touch.check_touch_screen_response_comm()
            self.touch.check_swipe_vertical()
            self.touch.screen_turn_on_off_comm()
