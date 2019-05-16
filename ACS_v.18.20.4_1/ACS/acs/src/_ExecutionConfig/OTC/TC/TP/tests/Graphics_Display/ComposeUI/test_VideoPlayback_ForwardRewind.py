# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/01/2015
@author: Yingjun Jin
'''
import os
from testlib.util.uiatestbase import UIATestBase
# from testlib.graphics.composeui_impl import ComposeUiImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.util.common import g_common_obj
from testlib.graphics.QRcode_impl import QRcode
from testlib.graphics.extend_photos_impl import PhotosExtendImpl


class ComposeUI(UIATestBase):


    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        self.d = g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
        self.photos = get_photo_implement()
        self._qr = QRcode()
#         self.photos.rm_delete_photos()
#         self.photos.refresh_sdcard()
#         self._composeui = ComposeUiImpl()
#         self._composeui.init_local_video()
        self.extend_photos = PhotosExtendImpl()
        self.remote_path = self.extend_photos.push_videos(like='test_video_qr', exts='.mp4')[0]

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
#         super(ComposeUI, self).tearDown()
#         self._composeui.delete_local_video()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        g_common_obj.stop_app_am("com.google.android.apps.photos")
        try:
            os.remove(".tmp.png")
        except OSError:
            pass

    def test_videoplayback_forwardrewind(self):
        ''' refer TC test_VideoPlayback_ForwardRewind
        '''
        print "[RunTest]: %s" % self.__str__()
#         self.photos.launch_photos_am()
        print "---------------GFX_forwardrewind--------------"
#         from testlib.graphics.screenshot_for_liverpt import take_screenshot_for_liverpt
#         take_screenshot_for_liverpt()
        self.photos.play_video_command(self.remote_path)
#         take_screenshot_for_liverpt()
        self.photos.slide_video_processbar_forward()
#         take_screenshot_for_liverpt()
        assert self._qr.decode_image_qrcode(self.d.screenshot(".tmp.png"))[1] == "QRCODE_TEST_STRING", "Video playback failed!"
        self.photos.slide_video_processbar_backward()
        assert self._qr.decode_image_qrcode(self.d.screenshot(".tmp.png"))[1] == "QRCODE_TEST_STRING", "Video playback failed!"
#         g_common_obj.assert_exp_happens()
#         self._composeui.stop_photos_am()
