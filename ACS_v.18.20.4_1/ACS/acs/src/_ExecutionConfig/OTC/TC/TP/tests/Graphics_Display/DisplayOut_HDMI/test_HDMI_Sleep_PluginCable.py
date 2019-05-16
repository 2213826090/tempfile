# -*- coding: utf-8 -*-
'''
Created on 11/9/2015
@author: Zhao, XiangyiX
'''

import os
import time

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.HDMI_switch_card_impl import HDMISwitchCardImpl
from testlib.graphics.HDMI_capture_decode_impl import HDMICaptureDecodeImpl
from testlib.graphics.common import is_device_screen_on


class HDMITest(UIATestBase):

    @classmethod
    def setUpClass(self):
        print "[setUpClass]: %s" % self.__name__
        super(HDMITest, self).setUpClass()
        self.photosImpl = get_photo_implement()
        self.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(HDMITest, self).setUp()
        self.photosImpl.rm_delete_photos()
        self.qrcode = "bmppicture"
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')

        self.retry_num = self.config.read(cfg_file, 'rerun_times').get("hdmi_imagedisplay_png")
        self.retry_num = int(self.retry_num)
        cfg = config.read(cfg_file, 'qrcode_marked_image')
        print self.qrcode
        self.qrcode = cfg.get("hdmi_png_qrcode")
        print "[Debug] qrcode:%s" % (self.qrcode)
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg.get("hdmi_png_image")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures')
        self.photosImpl.refresh_sdcard()

        self.systemui = SystemUiExtendImpl()
        self.HDMI_switch_card = HDMISwitchCardImpl()
        self.HDMI_capture_decode = HDMICaptureDecodeImpl()
        self.HDMI_switch_card.setup()
        self.HDMI_switch_card.switch_off()
        self.HDMI_capture_decode.setup()

        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.orientation = 'natural'

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(HDMITest, self).tearDown()
        self.photosImpl.rm_delete_photos()
        self.photosImpl.refresh_sdcard()

    def test_HDMI_Sleep_PluginCable(self):
        """
        test_HDMI_Sleep_PluginCable
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Launch photos and check HDMI by capture decode QRcodepicture."""
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        self.photosImpl.view_a_picture_fullscreen()

        print """[Step] 2. Press PWR key to Suspend DUT."""
        print "press power key to Suspend DUT."
        g_common_obj.adb_cmd("input keyevent 26")
        time.sleep(2)
        assert not is_device_screen_on(), \
            "[FAILURE] Failed DUT screen turned Off."
        print "press power key to Resume DUT."
        g_common_obj.adb_cmd("input keyevent 26")
        time.sleep(2)

        print """[Step] 3. Plugin HDMI cable."""
        print "plug HDMI."
        self.HDMI_switch_card.switch_on()
        time.sleep(2)
        print "check HDMI by capture decode QRcode picture."
        success = False
        self.error_string = ""
        for i in range(self.retry_num):
            print "[capture_decode]:%s times" % (i + 1)
            success, self.error_string = self.HDMI_capture_decode.capture_decode(self.qrcode)
            print success, self.error_string
            if success == True:
                break
            else:
                try:
                    picname = g_common_obj.globalcontext.user_log_dir + "/hdmipicture.png"
                    cmd = "mv " + self.HDMI_capture_decode.get_path() + "saveimages/fail_0_0.png %s" % (picname)
                    print os.system(cmd)
                except Exception as e:
                    print e.message, e.args
        assert success, self.error_string
        self.photosImpl.stop_photos_am()
        print "unplug HDMI."
        self.HDMI_switch_card.switch_off()
