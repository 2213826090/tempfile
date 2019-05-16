import sys
from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch



class BootCheckTouchScreenResopnse10Times(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        print
        print "[Setup]: %s" % self._test_name
        super(BootCheckTouchScreenResopnse10Times, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.touch.boot_up_completed_skip_boot_ui()
        super(BootCheckTouchScreenResopnse10Times, self).tearDown()

    def test_boot_check_touch_screen_response_10_times(self):
        print "[RunTest]: %s" % self.__str__()
        fileName = sys._getframe().f_code.co_name
        for loop in range(10):
            self.touch.logger.info("")
            self.touch.logger.info("Loop %d/10 " %(loop+1) + fileName)
            self.touch.reboot_devices()
            self.touch.check_touch_screen_response_step()
