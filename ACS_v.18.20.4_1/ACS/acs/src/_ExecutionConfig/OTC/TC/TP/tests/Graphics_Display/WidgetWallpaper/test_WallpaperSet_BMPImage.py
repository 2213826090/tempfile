# -*- coding: utf-8 -*-
'''
Created on 04/01/2015
@author: Ding, JunnanX
'''

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.common import logcat
from testlib.graphics.extend_camera_impl import CameraExtendImpl
from testlib.graphics.extend_photos_impl import PhotosExtendImpl
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.set_wallpaper_impl import WallpaperImpl
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.QRcode_impl import QRcode
from testlib.graphics.special_actions_impl import special_actions


class WallpaperSetBMPImageTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(WallpaperSetBMPImageTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
#         g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(WallpaperSetBMPImageTest, self).setUp()

        self.photos = PhotosExtendImpl()
        self.camera = CameraExtendImpl()
        self.systemui = SystemUiExtendImpl()
        self.photosImpl = get_photo_implement()
        self.qrcodeImpl = QRcode()
#         self.wallpaper = WallpaperImpl()
#         self.systemui.unlock_screen()
#         self.d.screen.on()
#         self.d.press.menu()

        self.photos.clean()
        self.camera.clean()
#         self.photosImpl.rm_delete_photos()
#         self.photosImpl.refresh_sdcard()
#         self.wallpaper.reset_wallpaper()

        self.qrcode = "GIFPICTURE"
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')

        cfg = config.read(cfg_file, 'qrcode_marked_image')
        self.qrcode = cfg.get("wallpaperset_bmp_qrcode")
        print "[Debug] qrcode:%s" % (self.qrcode)
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg.get("wallpaperset_bmp_image")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures')
        self.photosImpl.refresh_sdcard()
        special_actions.setup()

        self.mark_time = logcat.get_device_time_mark()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.photosImpl.rm_delete_photos()
        self.photosImpl.refresh_sdcard()
#         fatal_msg = logcat.get_device_log(self.mark_time, filters='*:F')
#         assert len(fatal_msg) == 0,\
#             "occurred Fatal error during testing:\n%s" % (fatal_msg)
        super(WallpaperSetBMPImageTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(WallpaperSetBMPImageTest, cls).tearDownClass()

    def test_WallpaperSet_BMPImage(self):
        """
        test_WallpaperSet_BMPImage

        Steps:
        1. Open a BMP picture by Gallery app
        BMP image display is OK.
        2. Select "Set as..." Wallpaper option and crop the picture
        Select wallpaper option and crop the picture are OK.
        3. Back the home UI to check the wallpaer after setting
        The BMP picture can set as wallpaper successfully. DUT has no crash or freeze.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Open a BMP picture by Gallery app
            BMP image display is OK."""
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        print """[Step] 2. Select "Set as..." Wallpaper option and crop the picture
            Select wallpaper option and crop the picture are OK."""
        self.photosImpl.set_picture_as_wallpaper()
        print """[Step] 3. Back the home UI to check the wallpaer after setting
            The BMP picture can set as wallpaper successfully. DUT has no crash or freeze."""
        self.qrcodeImpl.verify_qrcode_marked_image(self.qrcode)
