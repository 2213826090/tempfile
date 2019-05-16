# -*- coding: utf-8 -*-
'''
Created on 09/18/2015
@author: Zhang,RongX Z
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.set_wallpaper_impl import WallpaperImpl
from testlib.util.common import g_common_obj


class SetLiveWallpaperSpectrum(UIATestBase):


    def setUp(self):
        super(SetLiveWallpaperSpectrum, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._liveWallpaper = WallpaperImpl()
        self._liveWallpaper.reset_wallpaper()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SetLiveWallpaperSpectrum, self).tearDown()
        self._liveWallpaper.reset_wallpaper()
        self._liveWallpaper.stop_settings_am()

    def test_LiveWallpaper_Spectrum(self):
        ''' refer TC test_LiveWallpaper_PhaseBeam
        '''
        print "[RunTest]: %s" % self.__str__()
        self._liveWallpaper.set_livewallpaper_by_desktop("Spectrum")
        g_common_obj.assert_exp_happens()