# -*- coding: utf-8 -*-
'''
Created on 07/06/2015
@author: Zhang,RongX Z
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.set_wallpaper_impl import WallpaperImpl



class SetLiveWallpaperPhaseBeam(UIATestBase):


    def setUp(self):
        super(SetLiveWallpaperPhaseBeam, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._liveWallpaper = WallpaperImpl()
        self._liveWallpaper.reset_wallpaper()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SetLiveWallpaperPhaseBeam, self).tearDown()
        self._liveWallpaper.reset_wallpaper()
        self._liveWallpaper.stop_settings_am()

    def test_LiveWallpaper_PhaseBeam(self):
        ''' refer TC test_LiveWallpaper_PhaseBeam
        '''
        print "[RunTest]: %s" % self.__str__()
        self._liveWallpaper.set_livewallpaper_by_settings("Phase Beam")
