#! /usr/bin/env python
# coding:utf-8

#import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.s0i3 import S0i3
from testlib.em.apps import Camera
from testlib.em.constants_def import O_MR0, O_MR1

class S0i3Camera(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.camera = Camera()
        self.camera.grant_permissions()
        self.camera.adb_root()
        #super(S0i3Camera, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.camera.stop()
        super(S0i3Camera, self).tearDown()

    def test_enter_s0i3_aosp_camera(self):
        if self.camera.get_build_version() == O_MR1:
            self.camera.launch()
        elif self.camera.get_build_version() == O_MR0:
            assert False, "Not applicable for this platform"
        enter_s3 = S0i3().suspend_resume(retry = 2)
        assert enter_s3, "Not enter S0i3"

