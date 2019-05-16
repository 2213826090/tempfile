# -*- coding: utf-8 -*-
'''
Created on 05/06/2015
@author: Ding, JunnanX
'''

import time
import os
from testlib.util.common import g_common_obj
from testlib.graphics.test_template.phototestbase import PhotoTestBase
from testlib.graphics.common import logcat, is_device_screen_on
# from testlib.graphics.extend_camera_impl import CameraExtendImpl
from testlib.graphics.extend_photos_impl import PhotosExtendImpl
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.QRcode_impl import QRcode
from testlib.graphics.special_actions_impl import special_actions


class ImageViewingSuspendResumeByCommandTest(PhotoTestBase):


    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(ImageViewingSuspendResumeByCommandTest, self).setUp()
        self.photos = PhotosExtendImpl()
#         self.camera = CameraExtendImpl()
        self.systemui = SystemUiExtendImpl()
        self.photosImpl = get_photo_implement()
        self.qrcodeImpl = QRcode()
        self.device = g_common_obj.get_device()
        self.systemui.unlock_screen()
        self.device.screen.on()
#         self.device.press.menu()
        self.photosImpl.rm_delete_photos()
        self.photosImpl.refresh_sdcard()
        self.photos.clean()
#         self.camera.clean()

        self.qrcode = "QRCODE_TEST_STRING"
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')

        cfg = config.read(cfg_file, 'content_picture')
        self.qrcode = cfg.get("imageviewing_qrcode")
        print "[Debug] qrcode:%s" % (self.qrcode)
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg.get("qr_image")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures')
        self.photosImpl.refresh_sdcard()
        special_actions.setup()
        self.photosImpl.launch_photos_am()
        self.mark_time = logcat.get_device_time_mark()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.photosImpl.rm_delete_photos()
        self.photosImpl.refresh_sdcard()
#         fatal_msg = logcat.get_device_log(self.mark_time, filters='*:F')
#         assert len(fatal_msg) == 0,\
#             "occurred Fatal error during testing:\n%s" % (fatal_msg)
        super(ImageViewingSuspendResumeByCommandTest, self).tearDown()
        try:
            os.remove(".tmp.png")
        except OSError:
            pass

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(ImageViewingSuspendResumeByCommandTest, cls).tearDownClass()

    def test_ImageViewing_SuspendResumeByCommand_20times(self):
        """
        test_ImageViewing_SuspendResumeByCommand_20times

        Steps:
        1. Launch Photos app to view an image.
        Image viewed in Photos app.
        2. Launch a terminal/CMD and input command "adb shell input keyevent KEYCODE_POWER" to suspend/resume the devices for 20 times.
        No error & hung up & panic occur.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch Photos app to view an image.
        Image viewed in Photos app."""
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
#         self.qrcodeImpl.verify_qrcode_marked_image(self.qrcode,set_wallpaper=False)
        print """[Step] 2. Launch a terminal/CMD and input command "adb shell input keyevent KEYCODE_POWER" to suspend/resume the devices for 20 times.
        No error & hung up & panic occur."""
#         cmd_power = "input keyevent KEYCODE_POWER"
#         cmd_menu = "input keyevent KEYCODE_MENU"
        for i in range(1, 21):
            print "[Debug] %s times" % (i)
#             g_common_obj.adb_cmd(cmd_power)
#             time.sleep(1)
#             assert not is_device_screen_on(), "[FAILURE] Failed suspend device"
            self.device.wakeup()
            time.sleep(3)
#             g_common_obj.adb_cmd(cmd_power)
#             g_common_obj.adb_cmd(cmd_menu)
#             time.sleep(1)
#             assert is_device_screen_on(), "[FAILURE] Failed resume device"
#             self.qrcodeImpl.verify_qrcode_marked_image(self.qrcode,set_wallpaper=False)
            assert self.qrcodeImpl.decode_image_qrcode(self.device.screenshot(".tmp.png"))[1] == "QRCODE_TEST_STRING",\
                "Imageviewing failed after resuming DUT in test cycle %s" % (i)
            time.sleep(1)
            self.device.sleep()
            time.sleep(3)
#             g_common_obj.adb_cmd(cmd_power)
