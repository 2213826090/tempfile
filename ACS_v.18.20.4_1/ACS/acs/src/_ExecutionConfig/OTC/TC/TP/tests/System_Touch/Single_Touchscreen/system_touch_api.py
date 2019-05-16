from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch

touch = SystemTouch()
touch.install_artifactory_app("api_test", "com.intel.test.apitests")

class SystemTouchAPI(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_get_fake_touch_status(self):
        print "[RunTest]: %s" % self.__str__()
        assert touch.get_fake_touch_status()

    def test_touch_screen_api_features(self):
        print "[RunTest]: %s" % self.__str__()
        assert touch.touch_screen_api_features()

    def test_get_touch_screen_presence(self):
        print "[RunTest]: %s" % self.__str__()
        assert touch.get_touch_screen_presence()

    def test_get_touch_screen_type(self):
        print "[RunTest]: %s" % self.__str__()
        assert touch.get_touch_screen_type()

