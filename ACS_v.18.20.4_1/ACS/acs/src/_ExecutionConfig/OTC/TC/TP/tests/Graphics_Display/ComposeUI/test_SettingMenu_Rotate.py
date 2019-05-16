# -*- coding: utf-8 -*-
'''
@summary: test_DisplayRotate_UnprotectedContent
@since: 08/31/2015
@author: Zhang,RongX Z(rongx.z.zhang@intel.com)
'''
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
import testlib.graphics.common as common


class SettingMenuRotate(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(SettingMenuRotate, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(SettingMenuRotate, self).setUp()
        self.device = g_common_obj.get_device()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(SettingMenuRotate, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(SettingMenuRotate, cls).tearDownClass()
        g_common_obj.set_vertical_screen()

    def test_SettingMenu_Rotate(self):
        print "[RunTest]: %s" % self.__str__()
        common.launch_settings_am()
        self.device.orientation = "l"
        time.sleep(1)
        self.device.orientation = "r"
        time.sleep(1)
        self.device.orientation = "n"
        time.sleep(1)
        g_common_obj.assert_exp_happens()
        common.stop_settings_am()
