# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/16/2015
@author: Yingjun Jin
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.fit_impl import FitImpl
from testlib.graphics.set_wallpaper_impl import WallpaperImpl


class Fit(RenderAppTestBase):

    def setUp(self):
        super(Fit, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._fit = FitImpl()
        self._fit.set_password_lock()
        self.wallpaper = WallpaperImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Fit, self).tearDown()
        self._fit.set_none_lock_from_password()
        self.wallpaper.reset_wallpaper()

    def test_passwordunlock_livewallpaper(self):
        ''' refer TC test_PasswordUnlock_LiveWallpaper
        '''
        print "[RunTest]: %s" % self.__str__()
        self._fit.set_live_wallpaper_phasebeam()
        self._fit.test_password_lock()
