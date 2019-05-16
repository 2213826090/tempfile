# -*- coding: utf-8 -*-
'''
@summary: Rotate 180
@since: 09/13/2017
@author: Rui
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.graphics.common import AdbExtension, get_current_focus_window


class RotateScreen(UIATestBase):

    def setUp(self):
        super(RotateScreen, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.common = g_common_obj
        self.adbExt = AdbExtension()
        self.d = self.common.get_device()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RotateScreen, self).tearDown()

    def test_Rotate_180Degree(self):
        print "[RunTest]: %s" % self.__str__()
        self.common.launch_app_am("com.android.settings", ".Settings")
        self.adbExt.change_automatic_rotation(0) # Turn off auto rotation
        self.adbExt.screen_rotation(2) # Rotate 180
        self.adbExt.screen_rotation(0) # Rotate back to default
        packagename = get_current_focus_window()[0]
        assert packagename == "com.android.settings", "Not focus on settings window."
