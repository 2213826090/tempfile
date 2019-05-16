#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.energy import Energy
from testlib.em.tools import Logcat
from testlib.em.usb_cut import USBCut

class SleepTimeInLockScreen(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.energy = Energy()
        self.logcat = Logcat(filter_tag = "PowerManagerService", grep_opt = "grep -c 'Going to sleep'")

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        time.sleep(2)
        super(SleepTimeInLockScreen, self).tearDown()

    def test_sleep_time_in_lock_screen(self):
        """
        Test_Check_the_sleep_time_in_lock_screen_after_USB_unplugged
        """
        print "[RunTest]: %s" % self.__str__()

        self.energy.set_screen_status("off")
        time.sleep(2)
        self.energy.set_screen_status("on")
        time.sleep(1)
        self.logcat.set_start_time()
        log = self.logcat.get_log()
        assert int(log) == 0
        USBCut().cut(15)
        log = self.logcat.get_log()
        assert int(log) == 1

