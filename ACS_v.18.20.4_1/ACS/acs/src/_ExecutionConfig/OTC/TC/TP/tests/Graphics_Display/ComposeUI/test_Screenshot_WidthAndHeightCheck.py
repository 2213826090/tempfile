# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/30/2015
@author: Yingjun Jin
'''

import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.composeui_impl import ComposeUiImpl
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.graphics.photos_impl import get_photo_implement


class ComposeUI(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        init test environment
        """
        super(ComposeUI, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_picture')
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg.get("png")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures')
        self.pic_path = '/sdcard/Pictures/' + os.path.basename(file_path)
        ImageDetails.set_workaround()

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

    def test_screenshot_widthandheightcheck(self):
        ''' refer TC test_Screenshot_WidthAndHeightCheck
        '''
        print "[RunTest]: %s" % self.__str__()
#         self.photos.launch_photos_am()
        self.photos.open_image_command(self.pic_path)
        time.sleep(5)
        self._composeui.capture_screen_shot_with_png_and_check()
        self.photos.stop_photos_am()
