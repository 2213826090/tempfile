# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/25/2015
@author: Yingjun Jin
'''
import time
from testlib.util.common import g_common_obj
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.extend_camera_impl import CameraExtendImpl
from testlib.graphics.photos_impl import get_photo_implement


class Render(RenderAppTestBase):

    def setUp(self):
        super(Render, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._extendcamera = CameraExtendImpl()
        self._photos = get_photo_implement()
        self._photos.rm_delete_photos()
        self._photos.refresh_sdcard()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Render, self).tearDown()
        self._photos.rm_delete_photos()
        self._photos.refresh_sdcard()

    def test_videorecording_severalclips(self):
        ''' refer TC test_VideoRecording_SeveralClips
        '''
        print "[RunTest]: %s" % self.__str__()
        self._extendcamera.launch_camera_am()
        self._extendcamera.capture_video("1")
        self._extendcamera.stop_camera_am()
        self._photos.launch_photos_am()
        self._photos.play_video("Camera")
        time.sleep(10)
        g_common_obj.assert_exp_happens()
        self._photos.stop_photos_am()
