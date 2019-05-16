#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
#from testlib.util.common import g_common_obj
from testlib.em.constants_def import *
from testlib.em.s0i3 import S0i3
from testlib.em.power import get_power_obj


class AutomotivePowerStates(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.s0i3 = S0i3()
        self.s0i3.adb_root()
        self.s0i3.set_screen_status("on")
        self.s0i3.unlock_screen()
        #g_common_obj.close_background_apps()
        super(AutomotivePowerStates, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(AutomotivePowerStates, self).tearDown()

    def test_automotive_resume_vehicle_from_deep_sleep(self):
        assert self.s0i3.suspend_resume(retry = 2), "Not enter S3"
        assert self.s0i3.get_screen_status()

    def test_automotive_power_on_display_on(self):
        get_power_obj().reboot()
        self.s0i3.adb_root()
        s3 = self.s0i3.get_s0i3_suspend_stat()
        assert s3 == 0
        assert self.s0i3.get_screen_status()

    def test_automotive_vehicle_in_s3(self):
        assert self.s0i3.suspend_resume(retry = 2), "Not enter S3"

    def test_automotive_vehicle_in_s5(self):
        get_power_obj().reboot()
        self.s0i3.adb_root()
        s3 = self.s0i3.get_s0i3_suspend_stat()
        assert s3 == 0

    def test_automotive_vehicle_prepare_shutdown(self):
        for _ in range(2):
            logcat_start = self.s0i3.get_logcat_format_time()
            self.s0i3.suspend_resume(retry = 1)
            cmd = "logcat -s CAR.POWER -t '%s' | grep 'send sleep entry'" % logcat_start
            msg = self.s0i3.testDevice.adb_cmd_common(cmd)
            if msg:
                return
        assert False, "Not found 'send sleep entry' in log"

