# -*- coding: utf-8 -*-
import os
import time

#from constants_def import *
from relay08 import get_relay_obj
from tools import UIBase


class AS0ix(UIBase):

    def __init__(self, serial = None, retry = 1):
        UIBase.__init__(self, serial)
        self.relay08 = get_relay_obj()
        self.conf_name = ""
        self.retry = retry

    def get_suspend_stat(self):
        cmd_str = "cat /d/suspend_stats  | grep success"
        for _ in range(3):
            msg = self.testDevice.adb_cmd_capture_msg(cmd_str)
            if msg:
                success = msg.split()[1]
                print "[info]--- S0i3 suspend stat:", success
                return int(success)
            time.sleep(5)
        assert False, "Get S0i3 failed."

    def get_sleep_time_from_conf(self):
        assert self.conf_name, "Confiure name error"
        from tools import get_config_value
        sleep_time = get_config_value("s0i3", self.conf_name)
        return int(sleep_time)

    def sleep(self, sleep_time):
        assert False, "Needs be implemented in subclass"

    def suspend_resume(self, sleep_time = None, retry = 0):
        if not sleep_time:
            sleep_time = self.get_sleep_time_from_conf()
        if not retry:
            retry = self.retry
        pre = self.get_suspend_stat()
        for i in range(1, retry + 1):
            self.sleep(sleep_time * i)
            post = self.get_suspend_stat()
            if pre < post:
                break
        return post - pre

    def reconnect_usb(self):
        pass


class CS0ixNormal(AS0ix):

    IDLE_SCREEN_OFF = 1
    #CHECK_WAKE = 2
    RESUME_BY_POWER = 4
    #RESUME_BY_CHARGER = 8

    def __init__(self, serial = None):
        AS0ix.__init__(self, serial)
        self.conf_name = "normal"
        self.retry = 1
        from usb_cut import USBCut
        self.usbcut = USBCut()

    def reconnect_usb(self):
        self.usbcut.connect_usb_retry()

    def sleep(self, sleep_time, flags = 0):
        if flags & CS0ixNormal.IDLE_SCREEN_OFF == 0:
            self.set_screen_status("off")
            time.sleep(2)
        self.relay08.usb_disconnect()
        print "[Info]--- Sleep %ss" % sleep_time
        time.sleep(sleep_time)
        # check screen on event from log
        if flags & CS0ixNormal.RESUME_BY_POWER:
            self.relay08.press_power_key(1)
            time.sleep(20)
        self.usbcut.connect_usb_retry()
        self.set_screen_status("on")
        self.unlock_screen()

    def suspend_resume_special(self, sleep_time = None, sleep_flags = 0):
        if not sleep_time:
            sleep_time = self.get_sleep_time_from_conf()
        pre = self.get_suspend_stat()
        self.sleep(sleep_time, sleep_flags)
        post = self.get_suspend_stat()
        return post - pre


class CS0ixIVI(AS0ix):

    def __init__(self, serial = None):
        AS0ix.__init__(self, serial)
        self.conf_name = "ivi"
        self.retry = 2

    def sleep(self, sleep_time):
        self.relay08.press_power_key(0.5)
        print "[Info]--- Sleep %ss" % sleep_time
        time.sleep(sleep_time)
        self.relay08.press_power_key(0.5)
        time.sleep(10)
        #if self.get_product() == BXT_O:
        #    if self.d(text="Owner").exists:
        #        self.d(text="Owner").click()


def get_s0ix_obj(serial = None):
    from constants_def import BXT_M, BXT_O
    product = UIBase().get_product()
    if product in [BXT_M, BXT_O]:
        return CS0ixIVI(serial)
    else:
        return CS0ixNormal(serial)

