# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 03/25/2015
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
        self._extendcamera.install_apk("PreviewRS")

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Render, self).tearDown()

    def test_previewactivity_frontrearcamera(self):
        ''' refer TC test_PreviewActivity_FrontRearCamera
        '''
        print "[RunTest]: %s" % self.__str__()
        self._extendcamera.launch_previewrs_am()
        self._extendcamera.switch_to_front_camera_rs()
        self._extendcamera.switch_to_back_camera_rs()
        self._extendcamera.stop_previewrs_am()
