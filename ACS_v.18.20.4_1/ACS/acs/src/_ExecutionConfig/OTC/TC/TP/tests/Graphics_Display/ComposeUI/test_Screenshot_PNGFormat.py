# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 05/03/2015
@author: Yingjun Jin
'''

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.composeui_impl import ComposeUiImpl
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.graphics.photos_impl import get_photo_implement


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()

    def test_screenshot_pngformat(self):
        ''' refer TC test_Screenshot_PNGFormat
        '''
        print "[RunTest]: %s" % self.__str__()
        self._composeui.capture_screen_shot()
        time.sleep(2)
#         ImageDetails.set_workaround()
#         self.photos.launch_photos_am()
        self.photos.open_image_command("/sdcard/Pictures/screen.png")
        time.sleep(2)
        self._composeui.check_photos_format()
        self.photos.stop_photos_am()