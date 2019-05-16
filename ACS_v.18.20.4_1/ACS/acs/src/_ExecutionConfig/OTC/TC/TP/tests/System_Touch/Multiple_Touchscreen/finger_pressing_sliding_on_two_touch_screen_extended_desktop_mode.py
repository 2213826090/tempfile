from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch
from testlib.system_touch.multiple_touchscreen import MultiTouchScreen


class FingerPressingSlidingOnExtendedDesktopMode(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        self.multi_ts = MultiTouchScreen()
        print
        print "[Setup]: %s" % self._test_name
        self.multi_ts.check_multi_touch_sp_connected_num(2)
        self.multi_ts.enter_to_mosaic_mode_display_touch_work_comm()
        super(FingerPressingSlidingOnExtendedDesktopMode, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.multi_ts.exit_to_mosaic_mode_display_touch_work_comm()
        super(FingerPressingSlidingOnExtendedDesktopMode, self).tearDown()

    def test_finger_pressing_sliding_on_extended_desktop_mode(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.logger.info("")
        self.touch.enable_devoption_touch_location()
        self.multi_ts.check_mosaic_single_touch_sendevent_screen_comm(2)
        self.multi_ts.check_touch_gesture_swipe_vertical_down_up_comm()
        self.multi_ts.check_touch_gesture_swipe_vertical_down_up_comm("up")
