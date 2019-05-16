# -*- coding: utf-8 -*-
'''
Created on 03/10/2016
@author:ZhangroX
'''

import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.composition_fallback_impl import CompositionFallbackImpl
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
import testlib.graphics.common as common

class GPUCompositionFallbackMoreLayers(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(GPUCompositionFallbackMoreLayers, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(GPUCompositionFallbackMoreLayers, self).setUp()
        self.compositionImpl=CompositionFallbackImpl()
        self.systemui = SystemUiExtendImpl()
        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.press.menu()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.d.press.home()
        common.launch_settings_am()
#         self.compositionImpl.enable_disable_other_input_methods("off")
        super(GPUCompositionFallbackMoreLayers, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(GPUCompositionFallbackMoreLayers, cls).tearDownClass()

    def test_GPU_CompositionFallback_MoreLayers(self):
        """
        test_GPU_CompositionFallback_MoreLayers

        """
        print "[RunTest]: %s" % self.__str__()
        common.launch_settings_am()
        output=self.compositionImpl.dumpsys_surface_flinger()
        assert output=='',"dumpsys_surface failed"
        time.sleep(1)
#         self.compositionImpl.enable_disable_other_input_methods("on")
#         time.sleep(1)
#         self.d.press.home()
#         time.sleep(1)
#         self.compositionImpl.touch_goolge_search_bar()
        self.compositionImpl.open_reset_window()
        output=self.compositionImpl.dumpsys_surface_flinger()
        assert output.find("DimLayer"),"dumpsys SurfaceFlinger | grep 'GLES |'  failed"
