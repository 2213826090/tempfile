#! /usr/bin/env python
# coding:utf-8

import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0i3 import S0i3
from testlib.em.ioc_log import IOC
from testlib.em.settings import DateSetting
from testlib.em.relay08 import get_relay_obj

class RealTimeClock(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        #self.s0i3 = S0i3()
        self.relay = get_relay_obj()
        self.ioc = IOC()
        self.ds = DateSetting()
        self.ds.set_property()
        #super(RealTimeClock, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.ioc.stop_reading()
        #super(RealTimeClock, self).tearDown()

    def test_s3_wake_up_via_real_time_clock(self):
        self.ds.set_time("23:59")
        self.ioc.start()
        self.relay.press_power_key(0.5)
        time.sleep(70)
        self.ioc.stop_reading()
        wakeup = self.ioc.check_string_in_log("-> on state")
        time.sleep(20)
        self.relay.press_power_key(0.5)
        time.sleep(10)
        assert wakeup, "Not wake at 00:00"

    def test_s5_wake_up_via_real_time_clock(self):
        self.ds.adb_root()
        msg = self.ds.testDevice.adb_cmd_capture_msg("cat /proc/uptime")
        uptime = float(msg.split()[0])
        if uptime < 200:
            time.sleep(200 - uptime)
        self.ds.set_time("23:59")
        self.ds.testDevice.adb_cmd("setprop dbg.sleepshutdown.dummy_cmd 1")
        self.ioc.start()
        self.relay.press_power_key(0.5)
        time.sleep(70)
        self.ioc.stop_reading()
        wakeup = self.ioc.check_string_in_log("-> on state")
        time.sleep(30)
        self.relay.press_power_key(0.5)
        time.sleep(10)
        assert wakeup, "Not wake at 00:00"
        msg = self.ds.testDevice.adb_cmd_capture_msg("cat /proc/uptime")
        uptime = float(msg.split()[0])
        assert uptime < 100

