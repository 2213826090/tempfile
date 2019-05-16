# -*- coding: utf-8 -*-
'''
@since: 09/14/2015
@author: ZhangroX
'''
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.extend_camera_impl import CameraExtendImpl

class CameraHorizontalMirroring(UIATestBase):

    def setUp(self):
        super(CameraHorizontalMirroring, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.device = g_common_obj.get_device()
        self._extendcamera = CameraExtendImpl()
        config_handle = ConfigHandle()
        result = config_handle.check_apps("com.android.rs.livepreview")
        if result == 0:
            self._extendcamera.install_apk("PreviewRS")

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self._extendcamera.uninstall_apk()
        super(CameraHorizontalMirroring, self).tearDown()

    def test_Viewfinder_HorizontalMirroring_SecondaryCamera(self):
        ''' refer TC test_Viewfinder_HorizontalMirroring_SecondaryCamera
        '''
        print "[RunTest]: %s" % self.__str__()
        self._extendcamera.launch_previewrs_am()
        self._extendcamera.switch_to_front_camera_rs()
        self._extendcamera.stop_previewrs_am()