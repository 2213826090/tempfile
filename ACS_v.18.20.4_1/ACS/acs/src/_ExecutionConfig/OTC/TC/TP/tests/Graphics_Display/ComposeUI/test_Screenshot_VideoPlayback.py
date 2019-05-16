# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 05/05/2015
@author: Yingjun Jin
'''

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.composeui_impl import ComposeUiImpl
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.extend_photos_impl import PhotosExtendImpl


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        self._composeui.init_local_video()
        self.extend_photos = PhotosExtendImpl()
        self.remote_path = self.extend_photos.push_videos(like='test_video_qr', exts='.mp4')[0]

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        self._composeui.delete_local_video()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        ImageDetails.delete_picture()
        ImageDetails.set_workaround()

    def test_screenshot_videoplayback(self):
        ''' refer TC test_Screenshot_VideoPlayback
        '''
        print "[RunTest]: %s" % self.__str__()
#         self.photos.launch_photos_am()
        self.photos.play_video_command(self.remote_path)
        time.sleep(5)
        self._composeui.capture_screen_shot_during_video_playback()
        self.photos.stop_photos_am()
