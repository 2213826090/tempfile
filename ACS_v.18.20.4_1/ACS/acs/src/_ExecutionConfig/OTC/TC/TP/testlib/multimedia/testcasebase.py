import sys
import os
import unittest

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.multimedia.multimedia_log import MultimediaLogger


class TestCaseBase(UIATestBase):
    """
    Camera UI Automataed Test Case Implementation

    """

    def setUp(self):
        super(TestCaseBase, self).setUp()
        self.logger = MultimediaLogger.instance()
        self.logger.addFileHandler()
        #MultimediaLogger(g_common_obj.get_user_log_dir()+os.sep+MultimediaLogger.logFileName)
        #self.camera_dir = "/sdcard/DCIM/Camera/"

    def tearDown(self):
        super(TestCaseBase, self).tearDown()
        self.logger.debug("===== Set MultimediaLogger instance to None====")
        self.logger.removeHanlders()
        MultimediaLogger.destory()

    # def assertFalse(self, expr, msg=None):
    #     """Check that the expression is false."""
    #     camera = CameraImpl()
    #     if expr == True:
    #         self.logger.debug("=====Screen capture====")
    #         camera.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
    #     assert (not expr), msg
    #     # super(CameraTestBase, self).assertFalse(expr, msg)
    #
    # def assertTrue(self, expr, msg=None):
    #     """Check that the expression is true."""
    #     camera = CameraImpl()
    #     if expr == False:
    #         self.logger.debug("=====Screen capture====")
    #         camera.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
    #     assert expr, msg
    #     # super(CameraTestBase, self).assertTrue(expr, msg)

