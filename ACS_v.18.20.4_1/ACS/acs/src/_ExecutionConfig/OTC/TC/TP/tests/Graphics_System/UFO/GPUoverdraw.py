# -*- coding: utf-8 -*-

from testlib.graphics.development_settings_impl import Debug_GPUoverdraw
from testlib.graphics.compare_pic_impl import compare_pic
from testlib.common.uiatestbase import UIATestBase


class GPUoverdraw(UIATestBase):

    def setUp(self):
        super(GPUoverdraw, self).setUp()
        self.dgo = Debug_GPUoverdraw()
        self.img_default = self.dgo.turn_off_GPUoverdraw_get_screenshot()

    def tearDown(self):
        self.dgo.turn_off_GPUoverdraw()

    def test_DebugMode_GPUOverdraw(self):
        img1 = self.dgo.enable_show_overdraw_areas_get_screenshot()
        img2 = self.dgo.enable_show_areas_for_deuteranomaly_get_screenshot()
        rms1 = compare_pic.compare_pic(self.img_default, img1)
        rms2 = compare_pic.compare_pic(self.img_default, img2)
        assert rms1 > 0 and rms2 > 0, "Fail to set GPU overdraw."

    def test_DebugMode_ShowGPUViewUpdates(self):
        '''
        Covered by previous test.
        '''
        self.test_DebugMode_GPUOverdraw()
