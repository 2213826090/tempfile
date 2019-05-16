#! /usr/bin/env python
# coding:utf-8

import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0i3 import S0i3
from testlib.em.tools import get_tmp_dir, mount_mtp, umount_mtp
from testlib.em.constants_def import O_MR1

class S0i3MTP(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.s0i3 = S0i3()
        self.s0i3.adb_root()
        #super(S0i3MTP, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        umount_mtp()
        if self.s0i3.get_build_version() == O_MR1:
            self.dev.select_usb_option("Charging")
        else:
            self.s0i3.select_usb_option("charge")
        #super(S0i3MTP, self).tearDown()

    def test_s0i3_resume_mtp_transfer(self):
        assert S0i3().suspend_resume(retry = 2), "Not enter S3"
        self.s0i3.adb_usb()
        time.sleep(5)
        if self.s0i3.get_build_version() == O_MR1:
            from testlib.em.settings import DeveloperSetting
            self.dev = DeveloperSetting()
            self.dev.select_usb_option("MTP")
        else:
            self.s0i3.select_usb_option("MTP")
        mount_status = mount_mtp()
        assert mount_status

