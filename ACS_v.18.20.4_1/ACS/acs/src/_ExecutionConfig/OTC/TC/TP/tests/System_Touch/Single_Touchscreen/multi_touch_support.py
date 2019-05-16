from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch
from testlib.util.common import g_common_obj


class MultiTouchSupport(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        print
        print "[Setup]: %s" % self._test_name
        self.touch.enable_devoption_touch_location()
        self.touch.push_send_event_script("multi_touch_max_support.sh")
        self.touch.push_send_event_script("multi_touch_event_untouch.sh")
        super(MultiTouchSupport, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        #g_common_obj.close_background_apps()
        super(MultiTouchSupport, self).tearDown()

    def test_maximum_fingers_support(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.check_maximum_fingers_support()

    def test_multitouch_support_with_coordinates(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.check_multi_touch_with_coordinates()
