# -*- coding: utf-8 -*-
'''
Created on 06/15/2015
@author: Zhao, XiangyiX
'''

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase


class HWCVersionCheck(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(HWCVersionCheck, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(HWCVersionCheck, self).setUp()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(HWCVersionCheck, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(HWCVersionCheck, cls).tearDownClass()

    def test_HWC_VersionCheck(self):
        """
        test_HWC_VersionCheck

        Steps:
        1. adb shell dumpsys SurfaceFlinger | grep "Hardware Composer state"
           Check if return Hardware Composer state (version  1010000) or Hardware Composer state (version  1030000) or
            Hardware Composer state (version  1040000) or Hardware Composer state (version  1050000)
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. adb shell dumpsys SurfaceFlinger | grep "Hardware Composer state"""""
        cmd = "dumpsys SurfaceFlinger | grep 'Hardware Composer state'"
        output = g_common_obj.adb_cmd_capture_msg(cmd)
        print "output:" + output
        assert output == "Hardware Composer state (version 01010000):" or output == "Hardware Composer state (version 01030000):" or \
        output == "Hardware Composer state (version 01040000):" or output == "Hardware Composer state (version 01050000):", "HWC version not right"
