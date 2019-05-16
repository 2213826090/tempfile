# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 05/04/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.composeui_impl import ComposeUiImpl
from testlib.graphics.imagedetails_impl import ImageDetails


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        ImageDetails.delete_picture()
        ImageDetails.set_workaround()

    def test_screenshot_homescreen_staticwallpaper(self):
        ''' refer TC test_ScreenShot_HomeScreen_StaticWallpaper
        '''
        print "[RunTest]: %s" % self.__str__()
        self._composeui.capture_screen_shot()