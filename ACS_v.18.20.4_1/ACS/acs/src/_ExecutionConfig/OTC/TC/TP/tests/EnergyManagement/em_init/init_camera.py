#! /usr/bin/env python
# coding:utf-8

import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl
from testlib.util.common import g_common_obj

class InitCamera(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        super(InitCamera, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(InitCamera, self).tearDown()

    def test_init_camera(self):
        self.emImpl.grant_permissions_for_camera_app()
        for i in range(3):
            try:
                self.emImpl.launch_camera()
                self.emImpl.stop_focused_activity()
                break
            except Exception as e:
                print e.message
        else:
            assert False

