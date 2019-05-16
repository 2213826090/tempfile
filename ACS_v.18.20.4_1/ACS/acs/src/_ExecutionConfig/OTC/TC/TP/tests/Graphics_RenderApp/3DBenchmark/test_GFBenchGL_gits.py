# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 12/11/2015
@author: Zhao Xiangyi
'''

import os
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.util.config import TestConfig
from testlib.graphics.gits_impl import GitsImpl


class GFXBenchGLTest(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(GFXBenchGLTest, self).setUpClass()
        self.config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        self.case_cfg = 'tests.tablet.gits.conf'
        GitsImpl.setup_enviroment(self.config.read(cfg_file, "content_gits"))

    def setUp(self):
        super(GFXBenchGLTest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.gits = GitsImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(GFXBenchGLTest, self).tearDown()
        self.gits.remove_file()

    def test_GFXBenchGL_LowLevelTest_AlphaBlending_Offscreen(self):
        """
        refer TC test_GFXBenchGL_LowLevelTest_AlphaBlending_Offscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'AlphaBlendingOffscreen')

    def test_GFXBenchGL_LowLevelTest_AlphaBlending_Onscreen(self):
        """
        refer TC test_GFXBenchGL_LowLevelTest_AlphaBlending_Onscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'AlphaBlendingOnscreen')

    def test_GFXBenchGL_HighLevelTest_Manhattan_Onscreen(self):
        """
        refer TC test_GFXBenchGL_HighLevelTest_Manhattan_Onscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'ManhattanOnScreen')

    def test_GFXBenchGL_HighLevelTest_Manhattan_Offscreen(self):
        """
        refer TC test_GFXBenchGL_HighLevelTest_Manhattan_Offscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'ManhattanOffScreen')

    def test_GFXBenchGL_HighLevelTest_TRex_Onscreen(self):
        """
        refer TC test_GFXBenchGL_HighLevelTest_TRex_Onscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'TRexOnscreen')

    def test_GFXBenchGL_HighLevelTest_TRex_Offscreen(self):
        """
        refer TC test_GFXBenchGL_HighLevelTest_TRex_Offscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'TRexOffscreen')

    def test_GFXBenchGL_LowLevelTest_ALU_ONscreen(self):
        """
        refer TC test_GFXBenchGL_LowLevelTest_ALU_ONscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'ALUONscreen')

    def test_GFXBenchGL_LowLevelTest_ALU_Offscreen(self):
        """
        refer TC test_GFXBenchGL_LowLevelTest_ALU_Offscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'ALUOffscreen')

    def test_GFXBenchGL_LowLevelTest_DriverOverhead_Offscreen(self):
        """
        refer TC test_GFXBenchGL_LowLevelTest_DriverOverhead_Offscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'DriverOverheadOffscreen')

    def test_GFXBenchGL_LowLevelTest_DriverOverhead_Onscreen(self):
        """
        refer TC test_GFXBenchGL_LowLevelTest_DriverOverhead_Onscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'DriverOverheadOnscreen')

    def test_GFXBenchGL_LowLevelTest_Fill_Offscreen(self):
        """
        refer TC test_GFXBenchGL_LowLevelTest_Fill_Offscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'FillOffscreen')

    def test_GFXBenchGL_LowLevelTest_Fill_Onscreen(self):
        """
        refer TC test_GFXBenchGL_LowLevelTest_Fill_Onscreen
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'FillOnscreen')

    def test_GFXBenchGL_SpecialTests_RenderQuality(self):
        """
        refer TC test_GFXBenchGL_SpecialTests_RenderQuality
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'RenderQuality')

    def test_GFXBenchGL_SpecialTests_RenderQuality_HighPrecision(self):
        """
        refer TC test_GFXBenchGL_SpecialTests_RenderQuality_HighPrecision
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'RenderQualityHighPrecision')

