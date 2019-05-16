# -*- coding: utf-8 -*-
'''
Created on 11/19/2015
@author: Zhang,RongX Z
'''
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.set_wallpaper_impl import WallpaperImpl
from testlib.util.common import g_common_obj
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.composeui_impl import ComposeUiImpl

class LiveWallpaperPreview(UIATestBase):

    def setUp(self):
        super(LiveWallpaperPreview, self).setUp()
        self._test_name = __name__
        self._device = g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
        self._liveWallpaper = WallpaperImpl()
        self._composeui = ComposeUiImpl()
#         self._liveWallpaper.reset_wallpaper()
        self._device = g_common_obj.get_device()
        config_handle = ConfigHandle()
        result = config_handle.check_apps("com.android.galaxy4")
        if result == 0:
            self._composeui.install_apk('Galaxy4')
        self.livewallpaper = ["Black Hole"]

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(LiveWallpaperPreview, self).tearDown()
#         self._liveWallpaper.reset_wallpaper()
        self._liveWallpaper.stop_settings_am()

    def test_LiveWallpaper_PreviewAndSetAs(self):
        ''' refer TC test_LiveWallpaper_PreviewAndSetAs
        '''
        print "[RunTest]: %s" % self.__str__()
        self._liveWallpaper.launch_set_livewallpaper_by_Settings()
        time.sleep(10)
#         self._device(text="Wallpapers").click.wait()
#         for i in range (0,len(self.livewallpaper)):
#             time.sleep(1)
#             try:
#                 self._device(scrollable=True).scroll.horiz.to(text=self.livewallpaper[i])
#                 time.sleep(1)
#                 self._device(text=self.livewallpaper[i]).click.wait()
#                 time.sleep(4)
#             except:
#                 print "Can't find text: " + self.livewallpaper[i]
#             g_common_obj.assert_exp_happens()
#             if i !=len(self.livewallpaper):
#                 self._device.press.back()
#             else:
#                 self._device(text="Set wallpaper").click.wait()
#                 time.sleep(2)
#                 self._device.press.home()
#                 time.sleep(1)
#                 g_common_obj.assert_exp_happens()
        g_common_obj.assert_exp_happens()