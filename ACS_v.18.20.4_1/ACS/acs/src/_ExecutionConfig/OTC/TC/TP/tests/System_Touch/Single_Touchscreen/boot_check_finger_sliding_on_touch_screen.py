from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch


class BootCheckFingerSlidingOnTouchScreen(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        print
        print "[Setup]: %s" % self._test_name
        super(BootCheckFingerSlidingOnTouchScreen, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(BootCheckFingerSlidingOnTouchScreen, self).tearDown()

    def test_boot_check_finger_sliding(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.logger.info("")
        self.touch.reboot_devices()
        product_name = self.touch.check_product()
        if "cht_mrd" in product_name or "androidia" in product_name or "celadon" in product_name:
            self.touch.check_touch_screen_response_comm()
        else:
            self.touch.check_single_touch_sendevent_screen_response_comm()
        self.touch.check_swipe_vertical()
