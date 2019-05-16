from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch
from testlib.util.common import g_common_obj


class DeactivateScreenPinning(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        print
        print "[Setup]: %s" % self._test_name
        g_common_obj.close_background_apps()
        super(DeactivateScreenPinning, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.touch.open_screen_pinning(enable=False)
        super(DeactivateScreenPinning, self).tearDown()

    def test_deactivate_screen_pinning(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.touch_deactivate_screen_pinning_mode()
