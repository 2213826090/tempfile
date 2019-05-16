# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/20/2015
@author: Yingjun Jin
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.gfxbench_impl import GFXBench


class RunGFXBench(RenderAppTestBase):

    def setUp(self):
        super(RunGFXBench, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._gfx = GFXBench()
        self._gfx.set_environment()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RunGFXBench, self).tearDown()
        self._gfx.uninstall_gfxbench()

    def test_gfxbenchgl_specialtests_renderquality_highprecision(self):
        ''' refer TC test_GFXBenchGL_SpecialTests_RenderQuality_HighPrecision
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gfx.launch_gfxbench_am()
        self._gfx.run_renderquality_highprecision()
        self._gfx.stop_gfxbench_am()

    def test_gfxbenchgl_specialtests_renderquality(self):
        ''' refer TC test_GFXBenchGL_SpecialTests_RenderQuality
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gfx.launch_gfxbench_am()
        self._gfx.run_renderquality()
        self._gfx.stop_gfxbench_am()

    def test_gfxbenchgl_lowleveltest_texturing_onscreen(self):
        ''' refer TC test_GFXBenchGL_LowLevelTest_Texturing_Onscreen
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gfx.launch_gfxbench_am()
        self._gfx.run_texturing_onscreen()
        self._gfx.stop_gfxbench_am()

    def test_gfxbenchgl_lowleveltest_texturing_offscreen(self):
        ''' refer TC test_GFXBenchGL_LowLevelTest_Texturing_Offscreen
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gfx.launch_gfxbench_am()
        self._gfx.run_texturing_offscreen()
        self._gfx.stop_gfxbench_am()

    def test_gfxbenchgl_lowleveltest_driveroverhead2_onscreen(self):
        ''' refer TC test_GFXBenchGL_LowLevelTest_DriverOverhead2_Onscreen
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gfx.launch_gfxbench_am()
        self._gfx.run_driveroverhead2_onscreen()
        self._gfx.stop_gfxbench_am()

    def test_gfxbenchgl_lowleveltest_driveroverhead2_offscreen(self):
        ''' refer TC test_GFXBenchGL_LowLevelTest_DriverOverhead2_Offscreen
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gfx.launch_gfxbench_am()
        self._gfx.run_driveroverhead2_offscreen()
        self._gfx.stop_gfxbench_am()

    def test_gfxbenchgl_lowleveltest_alu2_onscreen(self):
        ''' refer TC test_GFXBenchGL_LowLevelTest_ALU2_Onscreen
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gfx.launch_gfxbench_am()
        self._gfx.run_alu2_onscreen()
        self._gfx.stop_gfxbench_am()

    def test_gfxbenchgl_lowleveltest_alu2_offscreen(self):
        ''' refer TC test_GFXBenchGL_LowLevelTest_ALU2_Offscreen
        '''
        print "[RunTest]: %s" % self.__str__()
        self._gfx.launch_gfxbench_am()
        self._gfx.run_alu2_offscreen()
        self._gfx.stop_gfxbench_am()
