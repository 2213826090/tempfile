# -*- coding: utf-8 -*-
'''
@author: yusux
'''
from testlib.graphics.sample_apidemo import SampleApiDemoImpl
from testlib.systemui.systemui_impl import SystemUI
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase


class SwitchCameraPreview(RenderAppTestBase):

    @classmethod
    def setUpClass(self):
        super(SwitchCameraPreview, self).setUpClass()
        self._demoInstance = SampleApiDemoImpl()
        self._demoInstance.install_apk()

    def setUp(self):
        super(SwitchCameraPreview, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SwitchCameraPreview, self).tearDown()
        self._demoInstance = None

    def test_Switch_Camera(self):
        SystemUI().unlock_screen()
        self._demoInstance.switch_camera_preview()
