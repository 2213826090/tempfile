from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch



class TouchNoLineInterruptionSlidingScreen(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        print
        print "[Setup]: %s" % self._test_name
        self.touch.enable_devoption_touch_location()
        super(TouchNoLineInterruptionSlidingScreen, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.touch.remove_screen_lock_pattern()
        super(TouchNoLineInterruptionSlidingScreen, self).tearDown()

    def test_no_line_interruption_sliding_screen(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.check_no_line_interruption_when_sliding()

