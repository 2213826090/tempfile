from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch



class SingleTouchSupport(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        print
        print "[Setup]: %s" % self._test_name
        self.touch.enable_devoption_touch_location()
        super(SingleTouchSupport, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SingleTouchSupport, self).tearDown()

    def test_single_touch_support_with_coordinates(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.single_touch_settings_search_check_coordinates("Settings")
