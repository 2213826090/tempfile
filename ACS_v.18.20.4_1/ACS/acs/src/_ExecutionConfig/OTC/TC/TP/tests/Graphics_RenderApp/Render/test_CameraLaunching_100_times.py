# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/27/2015
@author: Yingjun Jin
'''
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.extend_camera_impl import CameraExtendImpl


class Render(RenderAppTestBase):

    def setUp(self):
        super(Render, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._extendcamera = CameraExtendImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Render, self).tearDown()
        self._extendcamera.delete_capture_pictures()

    def test_cameralaunching_100_times(self):
        ''' refer TC test_CameraLaunching_100_times
        '''
        print "[RunTest]: %s" % self.__str__()
        self._extendcamera.switch_back_home_serveraltimes()

