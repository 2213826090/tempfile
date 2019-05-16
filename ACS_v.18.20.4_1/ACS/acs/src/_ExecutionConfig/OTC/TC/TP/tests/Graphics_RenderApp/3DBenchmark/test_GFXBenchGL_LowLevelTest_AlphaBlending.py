# -*- coding: utf-8 -*-
'''
Created on 04/22/2015
@author: Ding, JunnanX
'''

from testlib.util.common import g_common_obj
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.gfxbench30_impl import Gfxbench30Impl
from testlib.graphics.common import wifi_ctrl
from testlib.graphics.common import rotate_screen


class GFXBenchGLLowLevelTestAlphaBlendingTest(RenderAppTestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(GFXBenchGLLowLevelTestAlphaBlendingTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(GFXBenchGLLowLevelTestAlphaBlendingTest, self).setUp()

        self.gfxbench = Gfxbench30Impl()
        self.systemui = SystemUiExtendImpl()

        self.gfxbench.clean()
        self.gfxbench.setup()

        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.press.menu()
        self.natural_orientation = self.d.info['naturalOrientation']
        self.orientation = self.d.orientation
        rotate_screen(mode='wide')
        wifi_ctrl.turn_on()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.gfxbench.clean()
        self.d.orientation = self.orientation
        self.d.freeze_rotation(self.natural_orientation)
        super(GFXBenchGLLowLevelTestAlphaBlendingTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(GFXBenchGLLowLevelTestAlphaBlendingTest, cls).tearDownClass()

    def test_GFXBenchGL_LowLevelTest_AlphaBlending(self):
        """
        test_GFXBenchGL_LowLevelTest_AlphaBlending

        Steps:
        1. Launch GFXBench and select "Alpha Blending", then touch "Start" to run..
            The application starts and run without any issue. Check test result, it should be reported, not N/A.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch GFXBench and select "Alpha Blending", then touch "Start" to run..
            The application starts and run without any issue. Check test result, it should be reported, not N/A."""
        self.gfxbench.launch()

        self.gfxbench.run_test_alpha_blending()
