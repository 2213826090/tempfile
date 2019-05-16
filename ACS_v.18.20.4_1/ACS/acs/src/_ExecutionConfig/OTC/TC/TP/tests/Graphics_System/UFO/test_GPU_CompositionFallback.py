# -*- coding: utf-8 -*-
'''
Created on 03/23/2015
@author: Ding, JunnanX
'''

import re
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.extend_glbenckmark_impl import GLBenchmarkExtendImpl
from testlib.graphics.development_settings_impl import DevelopmentSettingsImpl
from testlib.graphics.common import logcat


class GPUCompositionFallbackTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(GPUCompositionFallbackTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(GPUCompositionFallbackTest, self).setUp()

        self.benchmark = GLBenchmarkExtendImpl()
        self.systemui = SystemUiExtendImpl()
        self.develop_settings = DevelopmentSettingsImpl()

        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.press.menu()
        self.benchmark.clean()
        self.benchmark.setup()
        self.develop_settings.set_disable_hw_overlays(switch='ON')

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(GPUCompositionFallbackTest, self).tearDown()
        self.benchmark.clean()
        self.develop_settings.set_disable_hw_overlays('OFF')
        g_common_obj.set_vertical_screen()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(GPUCompositionFallbackTest, cls).tearDownClass()

    def test_GPU_CompositionFallback(self):
        """
        test_GPU_CompositionFallback

        Steps:
        1. Enable "Disable HW overlays" in developer options
        2. adb shell dumpsys SurfaceFlinger | grep -wi "GLES "
            GLES | xxxx | xxxx | xxxx | xx | xxxx |
        3. Start GLBenchmark 2.7 and run the Egypt HD ETC1 Onscreen test.
            The test runs without any errors and the workload images are properly displayed on the screen and we don't have artifacts.
        """
        assert logcat.check_dumpsys_SurfaceFlinger_info(True, 'GLES.*', 'GLES'), "Not found GLES info"

        print """[Step] 3. Start GLBenchmark 2.7 and run the Egypt HD ETC1 Onscreen test.
            The test runs without any errors and the workload images are properly displayed on the screen and we don't have artifacts."""
        self.benchmark.launch()
        self.benchmark.run_performance_test("test25")
        self.benchmark.check_all_results()
        g_common_obj.assert_exp_happens()

    def test_GPU_CompositionFallback_Mesa(self):
        assert logcat.check_dumpsys_SurfaceFlinger_info(True, 'GLES.*Mesa', 'Mesa'), "Not found GLES info"
        self.benchmark.launch()
        self.benchmark.run_performance_test("test25")
        self.benchmark.check_all_results()
        g_common_obj.assert_exp_happens()