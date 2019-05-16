# -*- coding: utf-8 -*-
'''
Created on 04/07/2015
@author: Ding, JunnanX
'''

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.apitest_impl import APITestImpl
from testlib.graphics.common import AdbExtension


class ScreenSizeReportingTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(ScreenSizeReportingTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(ScreenSizeReportingTest, self).setUp()
        AdbExtension().screen_rotation(0)
        self.apitest = APITestImpl()
        self.systemui = SystemUiExtendImpl()
        self.apitest.setup()
        self.systemui.unlock_screen()
        self.d.screen.on()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.apitest.clean()
        super(ScreenSizeReportingTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(ScreenSizeReportingTest, cls).tearDownClass()

    def test_Screen_Size_Reporting(self):
        """
        test_Screen_Size_Reporting

        Steps:
        1. Execute DisplayMetricsTestDriver InstrumentationTestRunner
            adb shell am instrument
                -e class com.intel.test.apitests.tests.DisplayMetricsTestDriver#testDisplayAllFields
                -w com.intel.test.apitests/com.intel.test.apitests.runners.DisplayMetricsTestRunner
        2. adb logcat -d | grep -Ei "widthPixels | heightPixels"
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Execute DisplayMetricsTestDriver InstrumentationTestRunner."""
        report_logs = self.apitest.run_DisplayMetricsTest()
        self.apitest.verify_screen_size(report_logs)
