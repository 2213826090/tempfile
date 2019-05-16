# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 06/04/2015
@author: Xiangyi Zhao
'''

import time
import os
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.composeui_impl import ComposeUiImpl
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.graphics.extend_photos_impl import PhotosExtendImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.QRcode_impl import QRcode
from testlib.graphics.special_actions_impl import special_actions


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()
        self.photos = get_photo_implement()
        self.qrcodeImpl = QRcode()
        self.photosImpl = get_photo_implement()
        self.photosImpl.rm_delete_photos()
        self.photosImpl.refresh_sdcard()
#         self.photos.clean()

        self.qrcode = "GIFPICTURE"
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')

        cfg = config.read(cfg_file, 'qrcode_marked_image')
        self.qrcode = cfg.get("screenshot_imageview_qrcode")
        print "[Debug] qrcode:%s" % (self.qrcode)
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg.get("screenshot_imageview_image")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures')
        self.pic_path = '/sdcard/Pictures/' + os.path.basename(file_path)
        self.photosImpl.refresh_sdcard()
        special_actions.setup()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        self.photosImpl.rm_delete_photos()
        self.photosImpl.refresh_sdcard()

    def test_screenshot_imageviewing(self):
        ''' refer TC test_screenshot_imageviewing
        '''
        print "[RunTest]: %s" % self.__str__()
#         self.photosImpl.launch_photos_am()
#         self.photosImpl.open_a_picture()
        self.photos.open_image_command(self.pic_path)
        time.sleep(3)
        self.qrcodeImpl.verify_qrcode_marked_image(self.qrcode, set_wallpaper=False)
        self.photosImpl.stop_photos_am()
