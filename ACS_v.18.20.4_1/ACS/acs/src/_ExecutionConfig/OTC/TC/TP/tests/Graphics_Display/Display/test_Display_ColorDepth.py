# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 11/20/2015
@author: Xiangyi Zhao
'''

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.display_metrics_report_impl import DisplayMetricsReportImpl


class DisplayColorDepth(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(DisplayColorDepth, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(DisplayColorDepth, self).setUp()
        self.displaymetrics = DisplayMetricsReportImpl()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(DisplayColorDepth, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(DisplayColorDepth, cls).tearDownClass()

    def test_Display_ColorDepth(self):
        self.displaymetrics.check_color_depth()

