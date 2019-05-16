from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch
from testlib.system_touch.multiple_touchscreen import MultiTouchScreen

class SingleTouchSupportExtendedDesktopMode(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        self.multi_ts = MultiTouchScreen()
        print
        print "[Setup]: %s" % self._test_name
        self.multi_ts.check_multi_touch_sp_connected_num(2)
        self.multi_ts.enter_to_mosaic_mode_display_touch_work_comm()
        super(SingleTouchSupportExtendedDesktopMode, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.multi_ts.exit_to_mosaic_mode_display_touch_work_comm()
        super(SingleTouchSupportExtendedDesktopMode, self).tearDown()

    def test_single_touch_support_extended_desktop_mode(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.logger.info("")
        self.touch.enable_devoption_touch_location()
        self.touch.check_touch_screen_response_comm()
        self.touch.single_touch_settings_search_check_coordinates("Settings")
        self.multi_ts.check_mosaic_single_touch_sendevent_screen_comm(2)
