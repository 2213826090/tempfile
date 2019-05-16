# -*- coding: utf-8 -*-
'''
@summary: Early Splash Test
@since: 09/12/2017
@author: Rui
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.graphics.splash_impl import SplashImpl


class SplashScreen(UIATestBase):

    def setUp(self):
        super(SplashScreen, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.common = g_common_obj
        self.d = self.common.get_device()
        self.sImpl = SplashImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SplashScreen, self).tearDown()

    def test_Alternative_Splash_Screen_without_BIOS(self):
        print "[RunTest]: %s" % self.__str__()
        self.sImpl.check_earlyEvs_init()
        self.sImpl.check_static_splashfile()
        self.sImpl.check_animation_splashfile()
