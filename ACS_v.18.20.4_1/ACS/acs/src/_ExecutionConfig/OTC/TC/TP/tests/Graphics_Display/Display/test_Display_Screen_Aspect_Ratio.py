# -*- coding: utf-8 -*-
'''
@summary: Test display ScreenRatio
@since: 05/29/2015
@author: Zhang,RongX Z(rongx.z.zhang@intel.com)
'''
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.display_metrics_report_impl import DisplayMetricsReportImpl
from testlib.graphics.common import windows_info

class ScreenRatio(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(ScreenRatio, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(ScreenRatio, self).setUp()
        self.displaymetrics=DisplayMetricsReportImpl()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(ScreenRatio, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(ScreenRatio, cls).tearDownClass()

    def test_Display_Screen_Aspect_Ratio(self):
        print "[RunTest]: %s" % self.__str__()
        windows_info.get_wm_size()
        self.displaymetrics.calculate_screen_aspect_ratio()