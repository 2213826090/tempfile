# -*- coding: utf-8 -*-
'''
Created on Dec 5, 2014

@author: yusux
'''
from testlib.graphics.extend_camera_impl import CameraExtendImpl
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase


class SwitchBetweenHome_Camera(RenderAppTestBase):

    def setUp(self):
        super(SwitchBetweenHome_Camera, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._extendcamera = CameraExtendImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SwitchBetweenHome_Camera, self).tearDown()
        self._extendcamera = None

    def testSwitch_back_home_serveraltimes(self):
        self._extendcamera.switch_back_home_serveraltimes(20)
