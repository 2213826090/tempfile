# -*- coding: utf-8 -*-
'''
@summary: Test display metricsReport
@since: 05/29/2015
@author: Zhang,RongX Z(rongx.z.zhang@intel.com)
'''
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.display_metrics_report_impl import DisplayMetricsReportImpl


class DisplayMetricsReportTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(DisplayMetricsReportTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(DisplayMetricsReportTest, self).setUp()
        self.displaymetrics = DisplayMetricsReportImpl()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(DisplayMetricsReportTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(DisplayMetricsReportTest, cls).tearDownClass()

    def test_Display_MetricsReport(self):
        print "[RunTest]: %s" % self.__str__()
        print "1.[Debug] compare dumpsys size with real size"
        self.displaymetrics.compare_dumpsys_size_with_real_size()
        print "2.[Debug] compare dumpsys density with real density"
        self.displaymetrics.compare_dumpsys_density_with_real_density()
        print "3.[Debug] judge if dumpsys density is in [120,160,213,240,280,320,360,400,480,560,640]"
        self.displaymetrics.judge_density_in_range()
#         print "4.[Debug] judge if dumpsys density is the  closest to dumpsys dpi"
#         self.displaymetrics.judge_dpi()
