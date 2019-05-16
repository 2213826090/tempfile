import sys
import os
import unittest

from testlib.util.common import g_common_obj
from testlib.camera.camera_log import CameraLogger
from testlib.util.uiatestbase import UIATestBase
from testlib.camera.mum_camera_impl import CameraImpl

class CameraTestBase(UIATestBase):
    """
    Camera UI Automataed Test Case Implementation

    """

    def setUp(self):
        super(CameraTestBase, self).setUp()
        self.logger = CameraLogger.instance()
        self.logger.addFileHandler()
        # CameraLogger(g_common_obj.get_user_log_dir()+os.sep+CameraLogger.logFileName)
        self.camera_dir = "/sdcard/DCIM/Camera/"

    def tearDown(self):
        super(CameraTestBase, self).tearDown()
        # self.logger.debug("===== Set CameraLogger instance to None====")
        self.logger.removeHanlders()
        CameraLogger.destory()

    def assertFalse(self, expr, msg=None):
        """Check that the expression is false."""
        camera = CameraImpl()
        if expr == True:
            self.logger.debug("=====Screen capture====")
            camera.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert (not expr), msg
        # super(CameraTestBase, self).assertFalse(expr, msg)

    def assertTrue(self, expr, msg=None):
        """Check that the expression is true."""
        camera = CameraImpl()
        if expr == False:
            self.logger.debug("=====Screen capture====")
            camera.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert expr, msg
        # super(CameraTestBase, self).assertTrue(expr, msg)

