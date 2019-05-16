# -*- coding: utf-8 -*-
'''
Created on 06/11/2015
@author: Zhang,RongX Z
'''

from testlib.util.common import g_common_obj
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.gfxbench_impl import Gfxbench
from testlib.graphics.common import rotate_screen


class GFXBenchGLTest(RenderAppTestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(GFXBenchGLTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(GFXBenchGLTest, self).setUp()
        self.gfxbench = Gfxbench()
        self.gfxbench.setup()
        self.timeout = 600

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.gfxbench.stop_gfxbenchmark()
        super(GFXBenchGLTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(GFXBenchGLTest, cls).tearDownClass()

    def test_GFXBenchGL_HighLevelTest_Manhattan_Onscreen(self):
        """
        test_GFXBenchGL_HighLevelTest_Manhattan_Onscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gfxbench.run_test("Manhattan", self.timeout)

    def test_GFXBenchGL_HighLevelTest_Manhattan_Offscreen(self):
        """
        test_GFXBenchGL_HighLevelTest_Manhattan_Offscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gfxbench.run_test("1080p Manhattan Offscreen", self.timeout)

    def test_GFXBenchGL_HighLevelTest_TRex_Onscreen(self):
        """
        test_GFXBenchGL_HighLevelTest_TRex_Onscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gfxbench.run_test("T-Rex", self.timeout)

    def test_GFXBenchGL_HighLevelTest_TRex_Offscreen(self):
        """
        test_GFXBenchGL_HighLevelTest_TRex_Offscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gfxbench.run_test("1080p T-Rex Offscreen", self.timeout)

    def test_GFXBenchGL_SpecialTests_RenderQuality_HighPrecision(self):
        """
        test_GFXBenchGL_SpecialTests_RenderQuality_HighPrecision
        """
        self.gfxbench.run_test("Render Quality (high precision)", self.timeout)

    def test_GFXBenchGL_SpecialTests_RenderQuality(self):
        """
        test_GFXBenchGL_SpecialTests_RenderQuality
        """
        self.gfxbench.run_test("Render Quality", self.timeout)

    def test_GFXBenchGL_LowLevelTest_DriverOverhead_Onscreen(self):
        """
        test_GFXBenchGL_LowLevelTest_DriverOverhead_Onscreen
        """
        self.gfxbench.run_test("gl_driver", self.timeout)

    def test_GFXBenchGL_LowLevelTest_DriverOverhead_Offscreen(self):
        """
        test_GFXBenchGL_LowLevelTest_DriverOverhead_Offscreen
        """
        self.gfxbench.run_test("gl_driver_off", self.timeout)

    def test_GFXBenchGL_LowLevelTest_ALU_ONscreen(self):
        """
        test_GFXBenchGL_LowLevelTest_ALU_ONscreen
        """
        self.gfxbench.run_test("gl_alu", self.timeout)

    def test_GFXBenchGL_LowLevelTest_ALU_Offscreen(self):
        """
        test_GFXBenchGL_LowLevelTest_ALU_Offscreen
        """
        self.gfxbench.run_test("gl_alu_off", self.timeout)

    def test_GFXBenchGL_LowLevelTest_AlphaBlending_Onscreen(self):
        """
        test_GFXBenchGL_LowLevelTest_AlphaBlending_Onscreen
        """
        self.gfxbench.run_test("gl_blending", self.timeout)

    def test_GFXBenchGL_LowLevelTest_AlphaBlending_Offscreen(self):
        """
        test_GFXBenchGL_LowLevelTest_AlphaBlending_Offscreen
        """
        self.gfxbench.run_test("gl_blending_off", self.timeout)

    def test_GFXBenchGL_LowLevelTest_Fill_Onscreen(self):
        """
        test_GFXBenchGL_LowLevelTest_Fill_Onscreen
        """
        self.gfxbench.run_test("gl_fill", self.timeout)

    def test_GFXBenchGL_LowLevelTest_Fill_Offscreen(self):
        """
        test_GFXBenchGL_LowLevelTest_Fill_Offscreen
        """
        self.gfxbench.run_test("gl_fill_off", self.timeout)

    def test_GFXBenchGL_LowLevelTest_Tessellation_Offscreen(self):
        self.gfxbench.run_test("1080p Tessellation Offscreen", self.timeout)

    def test_GFXBenchGL_LowLevelTest_Tessellation_Onscreen(self):
        self.gfxbench.run_test("Tessellation", self.timeout)

    def test_GFXBenchGL_LowLevelTest_ALU2_Offscreen(self):
        self.gfxbench.run_test("1080p ALU 2 Offscreen", self.timeout)

    def test_GFXBenchGL_LowLevelTest_ALU2_ONscreen(self):
        self.gfxbench.run_test("ALU 2", self.timeout)

    def test_GFXBenchGL_LowLevelTest_DriverOverhead2_Offscreen(self):
        self.gfxbench.run_test("1080p Driver Overhead 2 Offscreen", self.timeout)

    def test_GFXBenchGL_LowLevelTest_DriverOverhead2_Onscreen(self):
        self.gfxbench.run_test("Driver Overhead 2", self.timeout)

    def test_GFXBenchGL_LowLevelTest_Texturing_Offscreen(self):
        self.gfxbench.run_test("1080p Texturing Offscreen", self.timeout)

    def test_GFXBenchGL_LowLevelTest_Texturing_Onscreen(self):
        self.gfxbench.run_test("Texturing", self.timeout)

    def test_GFXBenchGL_HighLevelTest_ManhattanES31_Offscreen(self):
        self.gfxbench.run_test("1080p Manhattan 3.1 Offscreen", self.timeout)

    def test_GFXBenchGL_HighLevelTest_ManhattanES31_Onscreen(self):
        self.gfxbench.run_test("Manhattan 3.1", self.timeout)

    def test_GFXBenchGL_HighLevelTest_Car_Chase_Offscreen(self):
        self.gfxbench.run_test("1080p Car Chase Offscreen", self.timeout)

    def test_GFXBenchGL_HighLevelTest_Car_Chase_Onscreen(self):
        self.gfxbench.run_test("Car Chase", self.timeout)

