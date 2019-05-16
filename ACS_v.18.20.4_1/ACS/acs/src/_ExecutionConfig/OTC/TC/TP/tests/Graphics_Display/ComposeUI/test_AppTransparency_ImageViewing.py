# -*- coding: utf-8 -*-
'''
@since: 08/11/2015
@author: ZhangroX
'''
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.QRcode_impl import QRcode
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.set_wallpaper_impl import WallpaperImpl


class ImageViewing(UIATestBase):

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(ImageViewing, self).setUp()
        self.device = g_common_obj.get_device()
        self.photos = get_photo_implement()
        self.qrcodeImpl = QRcode()
        self.wallpaper = WallpaperImpl()

        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()

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
        self.photos.refresh_sdcard()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        super(ImageViewing, self).tearDown()

    def test_AppTransparency_ImageViewing(self):
        ''' refer TC test_AppTransparency_ImageViewing
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.launch_photos_am()
        self.photos.open_a_picture()
        if self.device(resourceId="com.google.android.apps.photos:id/photos_overflow_icon"):
            self.device(resourceId="com.google.android.apps.photos:id/photos_overflow_icon").click.wait()
        if self.device(resourceId="com.google.android.apps.photos:id/actionbar_overflow"):
            self.device(resourceId="com.google.android.apps.photos:id/actionbar_overflow").click.wait()
        time.sleep(1)
        self.qrcodeImpl.verify_qrcode_marked_image(self.qrcode, set_wallpaper=False)
        g_common_obj.assert_exp_happens()
        self.photos.stop_photos_am()
