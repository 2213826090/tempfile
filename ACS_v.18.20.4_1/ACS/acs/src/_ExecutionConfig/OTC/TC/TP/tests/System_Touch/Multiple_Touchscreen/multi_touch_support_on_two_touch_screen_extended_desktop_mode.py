from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch
from testlib.system_touch.multiple_touchscreen import MultiTouchScreen

class MultiTouchSupportExtendedDesktopMode(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        self.multi_ts = MultiTouchScreen()
        print
        print "[Setup]: %s" % self._test_name
        self.multi_ts.check_multi_touch_sp_connected_num(2)
        self.multi_ts.enter_to_mosaic_mode_display_touch_work_comm()
        super(MultiTouchSupportExtendedDesktopMode, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.multi_ts.exit_to_mosaic_mode_display_touch_work_comm()
        super(MultiTouchSupportExtendedDesktopMode, self).tearDown()

    def test_multi_touch_support_extended_desktop_mode(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.logger.info("")
        self.touch.enable_devoption_touch_location()
        self.touch.push_send_event_script("multi_touch_max_support.sh")
        self.touch.push_send_event_script("multi_touch_event_untouch.sh")
        self.touch.check_maximum_fingers_support()
