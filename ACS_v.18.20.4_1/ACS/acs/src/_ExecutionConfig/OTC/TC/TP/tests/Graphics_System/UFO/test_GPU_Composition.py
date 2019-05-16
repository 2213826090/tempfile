# -*- coding: utf-8 -*-
'''
Created on 03/23/2015
@author: Ding, JunnanX
'''
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.extend_glbenckmark_impl import GLBenchmarkExtendImpl
from testlib.graphics.development_settings_impl import DevelopmentSettingsImpl
from testlib.graphics.common import logcat


class GPUCompositionTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(GPUCompositionTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(GPUCompositionTest, self).setUp()

        self.benchmark = GLBenchmarkExtendImpl()
        self.systemui = SystemUiExtendImpl()
        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.press.menu()
        self.benchmark.clean()
        self.benchmark.setup()
        self.develop_settings = DevelopmentSettingsImpl()
        self.develop_settings.set_disable_hw_overlays('ON')

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(GPUCompositionTest, self).tearDown()
        self.benchmark.clean()
        self.develop_settings.set_disable_hw_overlays('OFF')

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(GPUCompositionTest, cls).tearDownClass()

    def test_GPU_Composition(self):
        """
        test_GPU_Composition

        Steps:
        1. Start GLBenchmark 2.7.0 and run the Egypt HD ETC1 Onscreen test.
            The test runs without any errors and the workload images are properly displayed on the screen and we don't have artifacts.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Start GLBenchmark 2.7 and run the Egypt HD ETC1 Onscreen test.
            The test runs without any errors and the workload images are properly displayed on the screen and we don't have artifacts."""
        self.benchmark.launch()
        self.benchmark.run_performance_test("test25")
        self.benchmark.check_all_results()
        g_common_obj.assert_exp_happens()

    def test_GPU_Composition_Mesa(self):
        assert logcat.check_dumpsys_SurfaceFlinger_info(True, 'GLES.*Mesa', 'Mesa'), "It's not mesa."
        self.benchmark.launch()
        self.benchmark.run_performance_test("test25")
        self.benchmark.check_all_results()
        g_common_obj.assert_exp_happens()
