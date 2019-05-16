# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/16/2015
@author: Yingjun Jin
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.extend_camera_impl import CameraExtendImpl


class Fit(RenderAppTestBase):

    def setUp(self):
        super(Fit, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._extendcamera = CameraExtendImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Fit, self).tearDown()
        self._extendcamera.set_camera_as_back_camera()

    def switch_rear_front_camera_100times(self):
        ''' refer TC test_Switch_Camera
        '''
        print "[RunTest]: %s" % self.__str__()
        self._extendcamera.launch_camera_am()
        self._extendcamera.switch_rear_front_camera_100times()
        # self._extendcamera.switch_rear_front_camera()
        self._extendcamera.stop_camera_am()
