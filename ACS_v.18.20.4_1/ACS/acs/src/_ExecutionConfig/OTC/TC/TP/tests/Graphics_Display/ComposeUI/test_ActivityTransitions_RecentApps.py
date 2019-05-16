# -*- coding: utf-8 -*-
'''
@since: 06/24/2015
@author: ZhangroX
'''
import time
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.photos_impl import get_photo_implement
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.graphics.QRcode_impl import QRcode


class ActivityTransitions(UIATestBase):

    def setUp(self):
        super(ActivityTransitions, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.device = g_common_obj.get_device()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        self._qr = QRcode()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_picture')
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg.get("qr_image_up")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures')
        self.pic_path = '/sdcard/Pictures/' + os.path.basename(file_path)
        self.photos.refresh_sdcard()
        if self.device.orientation != 'natural':
            self.device.orientation = 'natural'
        if self.device.productName == "r0_bxtp_abl":
            self.device.orientation = "l"

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        if self.device.orientation != "natural":
            self.device.orientation = "n"
        super(ActivityTransitions, self).tearDown()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        try:
            os.remove(".tmp.png")
        except OSError:
            pass

    def test_ActivityTransitions_RecentApps(self):
        ''' refer TC test_ActivityTransitions_RecentApps
        '''
        print "[RunTest]: %s" % self.__str__()
#         self.photos.launch_photos_am()
#         self.photos.open_a_picture()
        self.photos.open_image_command(self.pic_path)
        time.sleep(3)
#         assert self._qr.decode_image_qrcode(self.device.screenshot("tmp.png"))[1] == "QRCODE_TEST_STRING","Photos app imageviewing failed!"
#         time.sleep(3)
        self.device.press.recent()
        time.sleep(3)
        x = self.device.info["displayWidth"]
        y = self.device.info["displayHeight"]
        self.device.swipe(x / 2, y * 2 / 3, x / 2, 0)
        # self.device(className="com.android.systemui.recents.views.TaskStackView").swipe.up(steps=5)
        time.sleep(3)
        if self._qr.decode_image_qrcode(self.device.screenshot("tmp.png"))[1] != "QRCODE_TEST_STRING":
            print "QR code not found, try portrait mode."
            self.device.press.back()
            self.device.orientation = "l"
            time.sleep(3)
            self.device.press.recent()
            time.sleep(2)
            self.device.swipe(x / 2, y * 2 / 3, x / 2, 0)
            assert self._qr.decode_image_qrcode(self.device.screenshot("tmp.png"))[1] == "QRCODE_TEST_STRING", \
                    "Recents task_view_thumbnail failed!"
        else:
            assert self._qr.decode_image_qrcode(self.device.screenshot("tmp.png"))[1] == "QRCODE_TEST_STRING", \
                    "Recents task_view_thumbnail failed!"
#         g_common_obj.assert_exp_happens()
#         time.sleep(1)
#         assert self.device(text="Device folders").exists and self.device(resourceId="com.android.systemui:id/task_view_thumbnail").exists,\
#          "test_ActivityTransitions_RecentApps failed"
#         self.device(text="Device folders").click.wait()
#         g_common_obj.assert_exp_happens()
        self.photos.stop_photos_am()
