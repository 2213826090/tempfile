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
from testlib.graphics.photos_impl import get_photo_implement
from testlib.util.common import g_common_obj


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()
        self.photos = get_photo_implement()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        ImageDetails.delete_picture()
        ImageDetails.set_workaround()
        g_common_obj.set_vertical_screen()

    def test_screenshot_approtate(self):
        ''' refer TC test_ScreenShot_AppRotate
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.launch_photos_am()
        self._composeui.rotate_to_left()
        self._composeui.capture_screen_shot()
