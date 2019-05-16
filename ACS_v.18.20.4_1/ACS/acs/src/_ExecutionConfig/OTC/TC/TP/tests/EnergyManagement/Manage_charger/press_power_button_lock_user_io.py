#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl

class LockUserIO(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_press_power_button_lock_user_io(self):
        """
        press power button lock user IO
        """
        print "[RunTest]: %s" % self.__str__()

        self.emImpl.input_keyevent("KEYCODE_POWER")
        time.sleep(2)
        self.emImpl.verify_screen_status("off")

        self.emImpl.input_keyevent("KEYCODE_VOLUME_UP")
        self.emImpl.verify_screen_status("off")

        self.emImpl.input_keyevent("KEYCODE_VOLUME_DOWN")
        self.emImpl.verify_screen_status("off")

        self.emImpl.input_keyevent("KEYCODE_VOLUME_HOME")
        self.emImpl.verify_screen_status("off")

