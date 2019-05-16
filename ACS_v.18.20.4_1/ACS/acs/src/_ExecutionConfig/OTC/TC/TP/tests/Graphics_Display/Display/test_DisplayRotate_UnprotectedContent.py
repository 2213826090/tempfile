# -*- coding: utf-8 -*-
'''
@summary: test_DisplayRotate_UnprotectedContent
@since: 08/31/2015
@author: Zhang,RongX Z(rongx.z.zhang@intel.com)
'''
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase


class DisplayRotate(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(DisplayRotate, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(DisplayRotate, self).setUp()
        self.device = g_common_obj.get_device()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(DisplayRotate, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(DisplayRotate, cls).tearDownClass()
        g_common_obj.set_vertical_screen()

    def test_DisplayRotate_UnprotectedContent(self):
        print "[RunTest]: %s" % self.__str__()
        self.device.orientation = "l"
        time.sleep(1)
        self.device.orientation = "r"
        time.sleep(1)
        self.device.orientation = "n"
        time.sleep(1)
        g_common_obj.assert_exp_happens()