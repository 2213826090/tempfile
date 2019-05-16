# -*- coding: utf-8 -*-
'''
@since: 05/17/2017
@author: rui
'''
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.extend_camera_impl import CameraExtendImpl
from testlib.util.common import g_common_obj


class VideoModeViewFinder(UIATestBase):

    def setUp(self):
        super(VideoModeViewFinder, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.device = g_common_obj.get_device()
        self._extendcamera = CameraExtendImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self._extendcamera.cameraSwitchTo("Camera")
        self._extendcamera.cameraSwitchBackFront("back")
        self._extendcamera.stop_camera_am()
        super(VideoModeViewFinder, self).tearDown()

    def test_viewfinderdisplays_videomode_primarycamera(self):
        '''
        Run test with uiautomator instead of GITS.
        '''
        print "[RunTest]: %s" % self.__str__()
        self._extendcamera.launch_camera_am()
        self._extendcamera.cameraSwitchTo("Video")
        time.sleep(3)
        assert not self.device(text="Camera error").exists, "Camera popups 'Camera error'"
        g_common_obj.assert_exp_happens()
