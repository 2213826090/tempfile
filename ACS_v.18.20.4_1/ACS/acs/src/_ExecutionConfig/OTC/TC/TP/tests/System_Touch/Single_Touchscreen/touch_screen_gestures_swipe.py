from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch

touch = SystemTouch()

class GestureSwipe(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        touch.install_artifactory_app("clock_google", "com.google.android.deskclock")
        super(GestureSwipe, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(GestureSwipe, self).tearDown()

    def test_touch_screen_gestures_swipe(self):
        print "[RunTest]: %s" % self.__str__()
        touch.check_swipe_vertical()
        touch.check_swipe_horizontal()

