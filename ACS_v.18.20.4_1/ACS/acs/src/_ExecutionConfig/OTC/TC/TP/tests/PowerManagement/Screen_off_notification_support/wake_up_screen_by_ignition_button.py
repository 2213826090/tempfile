#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.relay08 import get_relay_obj
from testlib.em.s0i3 import S0i3

class WakeUpScreen(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.s0i3 = S0i3()
        self.relay08 = get_relay_obj()
        super(WakeUpScreen, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(WakeUpScreen, self).tearDown()

    def test_wake_up_screen_by_ignition_button(self):
        print "[RunTest]: %s" % self.__str__()
        self.relay08.press_power_key(0.5)
        time.sleep(5)
        #assert not self.s0i3.check_adb()
        self.relay08.press_power_key(0.5)
        time.sleep(10)
        self.s0i3.verify_screen_status("on")

