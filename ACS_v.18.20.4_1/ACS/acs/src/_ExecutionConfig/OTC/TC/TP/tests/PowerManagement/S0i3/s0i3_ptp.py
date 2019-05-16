#! /usr/bin/env python
# coding:utf-8

import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0i3 import S0i3
from testlib.em.tools import get_ptp_mount_dir
from testlib.em.constants_def import O_MR1

class S0i3PTP(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.s0i3 = S0i3()
        self.s0i3.adb_root()
        super(S0i3PTP, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        if self.s0i3.get_build_version() == O_MR1:
            self.dev.select_usb_option("Charging")
        else:
            self.s0i3.select_usb_option("charge")
        super(S0i3PTP, self).tearDown()

    def test_s0i3_resume_ptp_transfer(self):
        enter_s3 = S0i3().suspend_resume(retry = 2)
        assert enter_s3
        self.s0i3.adb_usb()
        time.sleep(5)
        if self.s0i3.get_build_version() == O_MR1:
            from testlib.em.settings import DeveloperSetting
            self.dev = DeveloperSetting()
            self.dev.select_usb_option("PTP")
        else:
            self.s0i3.select_usb_option("PTP")
        photo = "Pictures/1.png"
        self.s0i3.testDevice.adb_cmd("screencap -p /sdcard/%s" % photo)
        ser = self.s0i3.get_serialno()
        ptp_mount_dir = get_ptp_mount_dir(ser)
        photo_on_host = os.path.join(ptp_mount_dir, photo)
        print photo_on_host
        assert os.path.isfile(photo_on_host)

