# -*- coding: utf-8 -*-
'''
@summary: Test display metricsReport
@since: 09/14/2015
@author: Zhang,RongX Z(rongx.z.zhang@intel.com)
'''
import re
from testlib.graphics.common import windows_info
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.display_metrics_report_impl import DisplayMetricsReportImpl


class DalvikTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(DalvikTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(DalvikTest, self).setUp()
        self.displaymetrics = DisplayMetricsReportImpl()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(DalvikTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(DalvikTest, cls).tearDownClass()

    def test_xhdpi_DalvikVMHeapgrowthlimit_GreaterThan_128m(self):
        print "[RunTest]: %s" % self.__str__()
        density=windows_info.get_wm_density()
        if density >=320:
            output=g_common_obj.adb_cmd_capture_msg("getprop | grep dalvik.vm.heapgrowthlimit")
            if output != None:
                heapgrowthlimit=int(re.search("\d+",output).group())
                assert heapgrowthlimit >=128,"check alvik.vm.heapgrowthlimit failed,heapgrowthlimit is %s" %heapgrowthlimit
        else:
            print "density is %s"% density