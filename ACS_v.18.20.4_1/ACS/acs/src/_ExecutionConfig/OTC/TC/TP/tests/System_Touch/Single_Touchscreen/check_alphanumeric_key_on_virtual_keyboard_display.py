from testlib.util.uiatestbase import UIATestBase
from testlib.system_touch.system_touch import SystemTouch



class CheckAlphanumericKeyOnVirtualKeyboardDisplay(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.touch = SystemTouch()
        print
        print "[Setup]: %s" % self._test_name
        self.touch.enable_devoption_touch_location()
        super(CheckAlphanumericKeyOnVirtualKeyboardDisplay, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckAlphanumericKeyOnVirtualKeyboardDisplay, self).tearDown()

    def test_check_alphanumeric_key_display(self):
        print "[RunTest]: %s" % self.__str__()
        self.touch.logger.info("")
        self.touch.check_virtual_keyboard_display()
