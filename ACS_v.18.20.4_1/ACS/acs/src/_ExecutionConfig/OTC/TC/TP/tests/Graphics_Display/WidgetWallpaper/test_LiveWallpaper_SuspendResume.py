# -*- coding: utf-8 -*-
'''
Created on 11/17/2015
@author: Zhang,RongX Z
'''
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.set_wallpaper_impl import WallpaperImpl
from testlib.util.common import g_common_obj
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.composeui_impl import ComposeUiImpl


class SetLiveWallpaperSuspendResume(UIATestBase):


    def setUp(self):
        super(SetLiveWallpaperSuspendResume, self).setUp()
        self._test_name = __name__
        self._device = g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
        self._composeui=ComposeUiImpl()
        self._liveWallpaper = WallpaperImpl()
#         self._liveWallpaper.reset_wallpaper()
        config_handle = ConfigHandle()
        result = config_handle.check_apps("com.android.galaxy4")
        if result == 0:
            self._composeui.install_apk('Galaxy4')

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SetLiveWallpaperSuspendResume, self).tearDown()
#         self._liveWallpaper.reset_wallpaper()
        self._liveWallpaper.stop_settings_am()

    def test_liveWallpaper_suspendResume(self):
        ''' refer TC test_LiveWallpaper_SuspendResume
        '''
        print "[RunTest]: %s" % self.__str__()
#         self._liveWallpaper.set_livewallpaper_by_desktop("Black Hole")
        self._liveWallpaper.set_livewallpaper_by_Settings()
        self._device.press.home()
        time.sleep(5)
        self._device.screen.off()
        time.sleep(3)
        self._device.screen.on()
        g_common_obj.assert_exp_happens()