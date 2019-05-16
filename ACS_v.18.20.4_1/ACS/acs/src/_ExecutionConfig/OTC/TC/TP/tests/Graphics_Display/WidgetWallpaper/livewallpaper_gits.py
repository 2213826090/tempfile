# -*- coding: utf-8 -*-
'''
@summary: This module contains live wallpaper tests implemented with GITS, GLES buffer that captured by GITS in png format 
when rendering live wallpaper will be compared with referential images in each test.
@since: 05/24/2016
@author: Lijin Xiong
'''

import os
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig
from testlib.graphics.gits_impl import GitsImpl

class LiveWallpaper(UIATestBase):
    @classmethod
    def setUpClass(self):
        super(LiveWallpaper, self).setUpClass()
        self.config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        self.case_cfg = 'tests.tablet.gits.conf'
        GitsImpl.setup_enviroment(self.config.read(cfg_file, "content_gits"))

    def setUp(self):
        super(LiveWallpaper, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.gits = GitsImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(LiveWallpaper, self).tearDown()
        self.gits.remove_file()


    def test_LiveWallpaper_BlackHole(self):
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'blackhole')

    def test_LiveWallpaper_Bubble(self):
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'bubble')

    def test_LiveWallpaper_HoloSpiral(self):
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'holospiral')

    def test_LiveWallpaper_PhaseBeam(self):
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'phasebeam')
