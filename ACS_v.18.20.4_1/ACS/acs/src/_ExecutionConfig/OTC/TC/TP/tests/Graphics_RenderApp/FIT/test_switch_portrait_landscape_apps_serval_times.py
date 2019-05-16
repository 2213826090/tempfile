# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/23/2015
@author: Yingjun Jin
'''
import time
import os
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.extend_photos_impl import PhotosExtendImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.util.common import g_common_obj
from testlib.graphics.QRcode_impl import QRcode
from testlib.graphics.common import adb32


class Fit(RenderAppTestBase):

    def setUp(self):
        super(Fit, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.photos = get_photo_implement()
        self.d = g_common_obj.get_device()
        self.d.screen.on()
        self._qr = QRcode()
        self.extend_photos = PhotosExtendImpl()
        self.remote_path = self.extend_photos.push_videos(count=1, like='test_video_qr', exts='.mp4')[0]
        adb32.screen_rotation(0)

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Fit, self).tearDown()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        g_common_obj.stop_app_am("com.google.android.apps.photos")
        try:
            os.remove(".tmp.png")
        except OSError:
            pass
        adb32.screen_rotation(0)

    def test_switch_portrait_landscape_apps_serval_times(self):
        ''' refer TC test_switch_portrait_landscape_apps_serval_times
        '''
        print "[RunTest]: %s" % self.__str__()
        n = 5
        self.photos.play_video_command(self.remote_path)
        while n > 0:
            for i in [2, 1, 3, 0]:
                adb32.screen_rotation(i)
                time.sleep(10)
                assert self._qr.decode_image_qrcode(self.d.screenshot(".tmp.png"))[1] == "QRCODE_TEST_STRING", "Video playback failed!"
            n = n - 1

#         self._photos.launch_photos_am()
#         self._photos.play_video()
#         time.sleep(10)
#         self._game.launch_airattack_am()
#         self._fit.switch_portrait_landscape()
#         g_common_obj.assert_exp_happens()
