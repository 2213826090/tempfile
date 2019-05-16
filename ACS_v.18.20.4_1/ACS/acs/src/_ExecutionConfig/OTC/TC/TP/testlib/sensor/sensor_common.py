#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import time
import re
import fileinput
import logging
import ConfigParser
import math
import csv

from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.log import Logger
import relay08

class SensorCommon(object):
    """
    Sensor Common Resource
    """

    def __init__(self):
        formatter = "[%(asctime)s - %(levelname)s] %(message)s"
        self.testDevice = g_common_obj.get_test_device()
        self.d = g_common_obj.get_device()
        self.dsn = self.d.server.adb.device_serial()
        self.logger = Logger("Sensor", formatter)
        self.conf = ConfigParser.ConfigParser()
        # self.conf_file = self.match_configuration()
        self.relay = None
        self.conf_file = "tests.tablet.sensor.conf"
        self.conf_path = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), self.conf_file)

    def check_adb_connection(self):
        '''
        check adb connection status.
        '''
        result = g_common_obj.adb_cmd_capture_msg("echo hello").find("hello")
        if (result != -1):
            self.logger.info("%s : adb connected" % self.dsn)
            return True
        else:
            self.logger.error("%s : adb disconnected" % self.dsn)
            #assert False, "Test Result : Failed"
            return False

    def setting_ON_OFF(self, _text, enable=True):
        widget = self.d(text=_text).right(className="android.widget.Switch")
        if widget:
            if enable:
                if not widget.checked:
                    widget.click()
            else:
                if widget.checked:
                    widget.click()
        status = widget.checked
        return status

    def set_screen_status(self, status):
        self.d.screen(status)

    def check_product(self):
        cmd = "getprop ro.product.name"
        product = g_common_obj.adb_cmd_capture_msg(cmd)
        return product

    def check_version(self):
        cmd = "getprop ro.build.version.incremental"
        version = g_common_obj.adb_cmd_capture_msg(cmd)
        return version

    def check_build(self):
        cmd = "getprop ro.build.version.release"
        return g_common_obj.adb_cmd_capture_msg(cmd)

    def unlock_screen(self):
        if self.d(description="Unlock").exists:
            self.d.press("menu")

    def unlock_screen_single_swipe(self):
        info = self.d.info
        x = info["displayWidth"] / 2
        y = info["displayHeight"]
        self.d.swipe(x, y, x, 0)
        time.sleep(2)

    def setSleepMode(self, Mode):
        modelist = Mode.split()
        t = int(modelist[0]) * 1000
        if modelist[1].startswith('minute'):
            t *= 60
        cmd = "dumpsys power | grep 'mScreenOffTimeoutSetting='"
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        msglist =msg.split('=')
        if int(msglist[1]) == t:
            return
        self.launch_settings("Display")
        for i in range(5):
            if self.d(text="Sleep").exists:
                self.d(text = "Sleep").click()
                break
            time.sleep(2)
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(text=Mode)
        for i in range(5):
            if self.d(text = Mode).exists:
                self.d(text = Mode).click()
                break
            time.sleep(2)

    def clear_app(self, app_package):
        g_common_obj.adb_cmd("pm clear %s"% app_package)

    def launch_settings(self, sub_setting=None):
        #build = self.check_build()
        self.set_screen_status("on")
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        # assert self.d(resourceId="com.android.settings:id/icon").exists
        if sub_setting:
            for i in range(5):
                if self.d(scrollable=True).exists:
                    self.d(scrollable=True).scroll.vert.to(textStartsWith=sub_setting)
                if self.d(textStartsWith=sub_setting).exists:
                    self.d(textStartsWith=sub_setting).click()
                    break
                time.sleep(2)
        time.sleep(5)

    def scrollable_select_text(self, sub_text=None):
        time.sleep(2)
        if self.d(textContains=sub_text).exists:
            return True
        if not self.d(textContains=sub_text).exists:
            self.d().swipe.up(steps=2)  #resolve N build issue for app list
            self.d(scrollable=True).scroll.vert.to(textContains=sub_text)
        assert self.d(scrollable=True).scroll.vert.to(textContains=sub_text)

    def enable_developer_option(self):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        if not self.d(textContains="About ").exists:
            self.d(scrollable=True).scroll.vert.to(textContains="About ")
        if self.d(textContains="Developer options").exists:
            return
        if self.d(textContains="About ").exists:
            self.d(textContains="About ").click.wait()
        for _ in range(8):
            self.d(textContains="Build number").click()
            time.sleep(.5)
        self.d.press.back()
        if not self.d(textContains="Developer options").exists:
            self.d(scrollable=True).scroll.vert.to(textContains="Developer options")
        assert self.d(textContains="Developer options").exists

    def keep_awake(self):
        success = True
        self.set_screen_status("on")
        self.unlock_screen()
        self.enable_developer_option()
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self.scrollable_select_text("Developer options")
        if not self.d(textContains="Developer options").exists:
            self.d(scrollable=True).scroll.vert.to(textContains="Developer options")
        self.d(textContains="Developer options").click.wait()
        self.setting_ON_OFF("Stay awake", enable=True)
        assert success

    def check_screen_response_comm(self):
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        for i in range(3):
            self.scrollable_select_text("Display")
            if self.d(textContains="Display").exists:
                self.d(textContains="Display").click.wait()
                assert self.d(textContains="Sleep").exists
            self.d.press.back()
            self.scrollable_select_text("Security")
            if self.d(textContains="Security").exists:
                self.d(textContains="Security").click.wait()
                assert self.d(textContains="Encryption").exists
                self.d.press.back()
        if self.d(resourceId="com.android.settings:id/search").exists:
            self.d(resourceId="com.android.settings:id/search").click.wait()
            time.sleep(2)
            if not self.d(resourceId="android:id/search_src_text").exists:
                assert False, self.d(resourceId="android:id/search_src_text").exists

    def check_home_ui_user_safely_O(self):
        if self.d(text="Drive safely").exists or self.d(text="Owner").exists:
            if self.d(text="Owner").exists:
                self.d(text="Owner").click.wait()
            if self.d(textContains="START DRIVING").exists:
                self.d(textContains="START DRIVING").click.wait()
            assert not self.d(text="Owner").exists
        else:
            if self.d(textContains="START DRIVING").exists:
                self.d(textContains="START DRIVING").click.wait()
            g_common_obj.adb_cmd_capture_msg("input tap 310 1007")
            #assert not self.d(text="Owner").exists

    def adb_root(self):
        g_common_obj.root_on_device()

    def root(self):
        g_common_obj.adb_cmd_common("root")
        for i in range(5):
            message = g_common_obj.adb_cmd_common("root")
            if "adbd is already running as root" in message:
                print "[info]--- Root success"
                return
            time.sleep(2)
        assert False, "Root failed!"

    def check_boot_completed(self):
        g_common_obj.adb_cmd_capture_msg("getprop sys.boot_completed")
        for i in range(30):
            time.sleep(3)
            message = g_common_obj.adb_cmd_capture_msg("getprop sys.boot_completed")
            if '1' == message:
                return True
        else:
            return False

    def reboot_cycle(self):
        for i in range(5):
            g_common_obj.adb_cmd_common("reboot")
            time.sleep(20)
            if self.check_boot_completed() is True:
                print "[info]--- reboot completed"
                time.sleep(5)
                self.boot_up_completed_skip_boot_ui()
                self.root()
                return
        assert False, "reboot failed!"

    def press_power_button(self):
        power_key_cmd = "input keyevent 26"
        g_common_obj.adb_cmd_capture_msg(power_key_cmd)

    def enter_screen_off(self):
        g_common_obj.adb_cmd_common("logcat -c")
        self.press_power_button()
        time.sleep(5)
        log_str = "logcat -v threadtime -d |grep 'Display device changed state:' "
        state_info = g_common_obj.adb_cmd_common(log_str)
        print state_info
        if "ON" in state_info:
            self.press_power_button()
        if "OFF" in state_info:
            print "Devices screen is Off"

    def long_press_power_buttou(self):
        power_key_cmd = "input keyevent --longpress 26"
        for i in range(10):
            time.sleep(2)
            g_common_obj.adb_cmd_capture_msg(power_key_cmd)
            if self.d(text="Power off").exists:
                break

    def power_off(self):
        self.long_press_power_buttou()
        if self.d(text="Power off").exists:
            self.d(text="Power off").click.wait()
        if self.d(text="Power off").exists:
            self.d(text="Power off").click.wait()
            print "[info]--- power off devices"
            time.sleep(30)
            return True
        return False

    def check_cos_mode(self):
        time.sleep(5)
        if self.root() is False:
            return False
        else:
            print "[info]--- cos mode"

    def power_on(self):
        for i in range(5):
            g_common_obj.adb_cmd_common("reboot")
            time.sleep(20)
            if self.check_boot_completed() is True:
                print "[info]--- power on completed"
                time.sleep(5)
                self.unlock_screen_single_swipe()
                self.root()
                return
        assert False, "power on failed!"

    def boot_up_completed_skip_boot_ui(self):
        self.check_boot_completed()
        product_name = self.check_product()
        if "gordon_peak" in product_name:
            time.sleep(20)
            self.check_home_ui_user_safely_O()
        else:
            self.unlock_screen_single_swipe()

    def get_relay08_obj(self):
        self.relay = relay08.get_relay_obj()

    def press_power_key(self, duration=4):
        if not self.relay:
            self.relay = relay08.get_relay_obj()
        self.relay.press_power_key(duration)

    def get_s0i3_suspend_stat(self):
        self.adb_root()
        cmd_str = "cat /d/suspend_stats  | grep success"
        msg = self.testDevice.adb_cmd_capture_msg(cmd_str)
        success = msg.split()[1]
        print "[INFO]--- S0i3 suspend stat:", success
        return int(success)

    def check_screen_off_or_power_off_for_ivi(self):
        #no devices, adb connection failed for ivi
        time.sleep(80)
        if self.check_adb_connection() is False:
            print "[INFO]---press ignition key success"
        else:
            #skip first enter to S3 to long time, develop take for this not issue
            self.press_power_key(0.5)
            time.sleep(15)
            self.press_power_key(0.5)
            time.sleep(100)
            if self.check_adb_connection() is False:
                print "[INFO]---press ignition key success"
            else:
                assert False, "[INFO]---press ignition key Failed, Screen is ON"

    def check_enter_s0i3_state_for_ivi(self, sleep_time=20):
        s3_pre = self.get_s0i3_suspend_stat()
        self.press_power_key(0.5)
        time.sleep(15)
        try:
            self.check_screen_off_or_power_off_for_ivi()
        finally:
            self.press_power_key(0.5)
            time.sleep(3)
            self.boot_up_completed_skip_boot_ui()
        s3_post = self.get_s0i3_suspend_stat()
        assert s3_pre < s3_post
        if s3_pre < s3_post:
            return True
        return False

    def power_off_on_from_os_by_ignition(self):
        self.press_power_key(10)
        time.sleep(10)
        try:
            self.check_screen_off_or_power_off_for_ivi()
        finally:
            self.press_power_key(0.5)
            time.sleep(10)
            self.boot_up_completed_skip_boot_ui()
        s3_post = self.get_s0i3_suspend_stat()
        assert s3_post == 0

