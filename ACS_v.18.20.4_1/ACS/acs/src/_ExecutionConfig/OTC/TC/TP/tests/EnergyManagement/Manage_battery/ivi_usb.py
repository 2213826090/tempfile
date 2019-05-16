#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.tools import ADBTools

class IVI_USB(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.adb = ADBTools()
        self.adb.adb_root()
        super(IVI_USB, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(IVI_USB, self).tearDown()

    def test_connecting_keyboard_mouse(self):
        cmd = "cat /proc/bus/input/devices | grep -E '(Mouse|Keyboard)'"
        msg = self.adb.testDevice.adb_cmd_capture_msg(cmd)
        assert msg

    def test_connecting_usb_flash_disk(self):
        mount_path = self.adb.get_external_disk_mount_path("USB")
        assert mount_path

    def test_connecting_usb_charger(self):
        import re
        pattern = re.compile(".+,mSerialNumber=(\w+),.+")
        cmd = "dumpsys usb | grep /dev/bus/usb/"
        msg = self.adb.testDevice.adb_cmd_capture_msg(cmd)
        for line in msg.splitlines():
            result = pattern.match(line)
            if result and result.group(1) != "null":
                break
        else:
            assert False

    def test_connecting_usb_otg(self):
        # Test by using adb via tcp
        # Enable OTG
        cmd = "/vendor/bin/usb_otg_switch.sh h"
        self.adb.testDevice.adb_cmd_capture_msg(cmd)
        time.sleep(2)
        # Check OTG
        cmd = "cat /proc/bus/input/devices | grep -E '(Mouse|Keyboard)'"
        msg = self.adb.testDevice.adb_cmd_capture_msg(cmd)
        # Disable OTG
        cmd = "/vendor/bin/usb_otg_switch.sh p"
        self.adb.testDevice.adb_cmd_capture_msg(cmd)
        assert msg

