# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 06/08/2015
@author: Xiangyi Zhao
'''
import os, time
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
from testlib.util.common import g_common_obj
from testlib.common.base import clearTmpDir


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        self.device = g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()
        self._imageviewer = ImageDetails()
        self.photos = PhotosExtendImpl()
        self.photosImpl = get_photo_implement()
        self.qrcodeImpl = QRcode()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        self.photosImpl.rm_delete_photos()
        self.photosImpl.refresh_sdcard()
        self.photos.clean()

        self.qrcode = "GIFPICTURE"
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')

        cfg = config.read(cfg_file, 'qrcode_marked_image')
        self.qrcode = cfg.get("screenshot_edit_qrcode")
        print "[Debug] qrcode:%s" % (self.qrcode)
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg.get("screenshot_edit_image")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures')
        self.photosImpl.refresh_sdcard()
        special_actions.setup()
        self.photos.setup()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        self.photosImpl.rm_delete_photos()
        self.photosImpl.refresh_sdcard()
        clearTmpDir()

    def test_screenshot_imageedit(self):
        ''' refer TC test_screenshot_imageedit
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photosImpl.launch_photos_am()
        self._composeui.addeffects_check_screenshot()
        self.device.press.back()
        time.sleep(2)
        self.photosImpl.open_a_picture()
        self.qrcodeImpl.verify_qrcode_marked_image(self.qrcode, set_wallpaper=False)
        self.photosImpl.stop_photos_am()
