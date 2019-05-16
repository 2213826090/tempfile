# -*- coding: utf-8 -*-
import os
import time

from constants_def import *
from tools import *
from relay08 import get_relay_obj


class S0i3(UIBase):

    def __init__(self, serial = None):
        UIBase.__init__(self, serial)
        self.relay08 = get_relay_obj()

    def get_s0i3_suspend_stat(self):
        cmd_str = "cat /d/suspend_stats  | grep success"
        for _ in range(3):
            msg = self.testDevice.adb_cmd_capture_msg(cmd_str)
            if msg:
                success = msg.split()[1]
                print "[info]--- S0i3 suspend stat:", success
                return int(success)
            time.sleep(5)
        assert False, "Get S0i3 failed."

    def check_enter_s0i3_state_ivi(self, sleep_time):
        s3_pre = self.get_s0i3_suspend_stat()
        self.relay08.press_power_key(0.5)
        print "[Info]--- Sleep %ss" % sleep_time
        time.sleep(sleep_time)
        self.relay08.press_power_key(0.5)
        time.sleep(10)
        s3_post = self.get_s0i3_suspend_stat()
        if self.get_product() == BXT_O:
            if self.d(text="Owner").exists:
                self.d(text="Owner").click()
            #self.d.click(320, 950)
        if s3_pre < s3_post:
            return True
        return False

    def suspend_resume(self, sleep_time = None, retry = 1):
        if sleep_time == None:
            sleep_time = int(get_config_value("s0i3", "ivi"))
        for i in range(1, 1 + retry):
            enter_s3 = self.check_enter_s0i3_state_ivi(sleep_time * i)
            if enter_s3:
                return True
        return False

