# -*- coding: utf-8 -*-
"""
Created on 2014-11-5

@author: yusux
"""
import re
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.zedgeImpl import ZEDGEImpl
from testlib.apk.apk_install_uninstall_impl import ApkInstallUninstallImpl
from testlib.graphics.common import Logcat


class TestZEDGE(UIATestBase):
    def setUp(self):
        super(TestZEDGE, self).setUp()
        self.logcat = Logcat()
        ZEDGEImpl().install_zedge()

    def defaultTestResult(self):
        """
        """
        super(TestZEDGE, self).tearDown()
        ApkInstallUninstallImpl().apk_uninstall("Zedge")

    def test_zedge_wallpapper(self):
        """
        test_3rdParty_Apk:Zedge
        """
        for x in range(3):
            mark_time = self.logcat.get_device_time_mark()
            ZEDGEImpl().init_zedge()
            ZEDGEImpl().set_static_wallpapper()
            ZEDGEImpl().set_live_wallpaper()
            ZEDGEImpl().set_ringtone()
            dut_log = self.logcat.get_device_log(mark_time, filters="*:F |grep net.zedge.android")
            # fatal_log = re.findall(
            #     '.+FATAL EXCEPTION.+', dut_log)
            print '=======================\n'
            print dut_log+'\n'
            print '-' * 60 + '\n'
            # print fatal_log
            print '-' * 60 + '\n'
            assert not dut_log, "there is been FATAL EXCEPTION in 3rdParty apk:'Zedge'"
