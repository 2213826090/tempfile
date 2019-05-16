from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch
from testlib.util.common import g_common_obj

touch = SystemTouch()

class GestureZoom(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        g_common_obj.root_on_device()
        touch.install_artifactory_app("chrome_google", "com.android.chrome")
        touch.launch_chrome_skip_accept()
        g_common_obj.close_background_apps()
        super(GestureZoom, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(GestureZoom, self).tearDown()

    def test_touch_screen_gestures_zoom(self):
        print "[RunTest]: %s" % self.__str__()
        touch.grant_permissions_for_chrome_app()
        touch.close_chrome_tabs()
        touch.open_webpage("file:///mnt/sdcard/")
        touch.check_gesture_zoom()

