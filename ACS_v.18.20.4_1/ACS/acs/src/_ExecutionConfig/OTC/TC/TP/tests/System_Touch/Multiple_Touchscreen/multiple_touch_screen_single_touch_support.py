from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch
from testlib.system_touch.multiple_touchscreen import MultiTouchScreen


class MultipleTSSingleTouch(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        self.multi_ts = MultiTouchScreen()
        print
        print "[Setup]: %s" % self._test_name
        super(MultipleTSSingleTouch, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(MultipleTSSingleTouch, self).tearDown()

    def test_single_touch_support_on_two_touch_sp(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.logger.info("")
        self.multi_ts.check_multi_touch_sp_connected_num(2)
        self.multi_ts.check_car_mode_exists()
        self.touch.enable_devoption_touch_location()
        self.touch.check_single_touch_sendevent_screen_response_comm()

    def test_single_touch_support_on_three_touch_sp(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.logger.info("")
        self.multi_ts.check_multi_touch_sp_connected_num(3)
        self.multi_ts.check_car_mode_exists()
        self.touch.enable_devoption_touch_location()
        self.touch.check_single_touch_sendevent_screen_response_comm()
