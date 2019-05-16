# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 12/28/2015
@author: Xiangyi Zhao
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.composeui_impl import ComposeUiImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.set_wallpaper_impl import WallpaperImpl
from testlib.common.base import clearTmpDir


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
#         self._liveWallpaper = WallpaperImpl()
#         self._liveWallpaper.reset_wallpaper()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        clearTmpDir()

    def test_ScreenCapture_screencap(self):
        ''' refer TC test_ScreenCapture_screencap
        '''
        print "[RunTest]: %s" % self.__str__()
        self._composeui.capture_screenshot_compare_screenshot()
