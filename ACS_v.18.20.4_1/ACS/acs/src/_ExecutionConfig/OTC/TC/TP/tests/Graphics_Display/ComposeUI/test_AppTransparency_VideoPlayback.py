# -*- coding: utf-8 -*-
'''
@since: 08/11/2015
@author: ZhangroX
'''
import time
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.extend_photos_impl import PhotosExtendImpl
from testlib.util.common import g_common_obj
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.special_actions_impl import special_actions
from testlib.graphics.QRcode_impl import QRcode


class VideoPlayback(UIATestBase):

    def setUp(self):
        super(VideoPlayback, self).setUp()
        self._test_name = __name__
        self.device = g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
#         g_common_obj.root_on_device()
#         g_common_obj.remount_device()
        self.photos = get_photo_implement()
        self.photosImpl = PhotosExtendImpl()
        self._qr = QRcode()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        self.remote_path = self.photosImpl.push_videos(count=1, like='test_video_qr_bottom', exts='.mp4')[0]
        special_actions.setup()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(VideoPlayback, self).tearDown()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        try:
            os.remove(".tmp.png")
        except OSError:
            pass

    def test_AppTransparency_VideoPlayback(self):
        ''' refer TC test_AppTransparency_VideoPlayback
        '''
        print "[RunTest]: %s" % self.__str__()
        x, y = 0, self.device.info['displayHeight'] / 2 # touch point for full screen playback.
        self.photos.play_video_command(self.remote_path)
#         self.photos.launch_photos_am()
#         self.photos.play_video()
        time.sleep(5)
        self.photos.pause_play_video()
        g_common_obj.adb_cmd_capture_msg("input touchscreen tap %s %s" % (x, y))
        time.sleep(5)
        assert self._qr.decode_image_qrcode(self.device.screenshot(".tmp.png"))[1] == "QRCODE_TEST_STRING",\
            "Video playback transparency test failed!"
