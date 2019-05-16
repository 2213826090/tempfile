# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 11/20/2015
@author: Xiangyi Zhao
'''
import time
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.extend_photos_impl import PhotosExtendImpl
from testlib.util.common import g_common_obj
from testlib.graphics.QRcode_impl import QRcode
from testlib.graphics.composeui_impl import ComposeUiImpl
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.common import adb32


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.d = g_common_obj.get_device()
#         self.photos = get_photo_implement()
        self._composeui = ComposeUiImpl()
        config_handle = ConfigHandle()
        result = config_handle.check_apps("com.mxtech.videoplayer.ad")
        if result == 0:
            self._composeui.install_apk('Mxplayer')
        self.d.screen.on()
        self._qr = QRcode()
        self.extend_photos = PhotosExtendImpl()
        self.remote_path = self.extend_photos.push_videos(count=1, like='test_video_qr_middle', exts='.mp4')[0]
        adb32.screen_rotation(0)

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        os.remove("tmp.png")
        adb32.screen_rotation(0)

    def test_scaling_rotation_Video(self):
        ''' refer TC test_Scaling_Rotation_Video
        '''
        print "[RunTest]: %s" % self.__str__()
        self._composeui.play_video_command_mx(self.remote_path)
        time.sleep(2)
        if self.d(textContains="START OVER").exists:
            self.d(textContains="START OVER").click.wait()
        for i in [2, 1, 3, 0]:
            adb32.screen_rotation(i)
            time.sleep(1)
            if self.d(resourceId="com.mxtech.videoplayer.ad:id/ui_layout").exists:
                self.d(resourceId="com.mxtech.videoplayer.ad:id/ui_layout").pinch.Out(percent=120)
                time.sleep(2)
                assert self._qr.decode_image_qrcode(self.d.screenshot("tmp.png"))[1] == "QRCODE_TEST_STRING", "The scaling of video playback failed!"
                self.d(resourceId="com.mxtech.videoplayer.ad:id/ui_layout").pinch.In(percent=70, steps=5)
                time.sleep(2)
                assert self._qr.decode_image_qrcode(self.d.screenshot("tmp.png"))[1] == "QRCODE_TEST_STRING", "The scaling of video playback failed!"
#         self._composeui.launch_mxtech_am()
#         self._composeui.check_scaling_rotation_during_playing()
#         self._composeui.stop_mxtech_am()
