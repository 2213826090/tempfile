# -*- coding: utf-8 -*-
'''
Created on 11/25/2015
@author: Zhang,RongX Z
'''
import time
from testlib.util.common import g_common_obj
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.extend_glbenckmark_impl import GLBenchmarkExtendImpl
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.common import close_all_tasks


class GLBenchmarkEgyptHD(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(GLBenchmarkEgyptHD, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(GLBenchmarkEgyptHD, self).setUp()

        self.benchmark = GLBenchmarkExtendImpl()
        self.systemui = SystemUiExtendImpl()
        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.press.menu()
        self.benchmark.clean()
        self.benchmark.setup()
        close_all_tasks()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.benchmark.clean()

        g_common_obj.set_vertical_screen()
        super(GLBenchmarkEgyptHD, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(GLBenchmarkEgyptHD, cls).tearDownClass()

    def test_GLBenchmarkEgyptHD_MSAA(self):
        print "[RunTest]: %s" % self.__str__()
        self.benchmark.launch()
        self.benchmark.run_performance_test("test25")
        self.benchmark.check_all_results()
        g_common_obj.assert_exp_happens()
