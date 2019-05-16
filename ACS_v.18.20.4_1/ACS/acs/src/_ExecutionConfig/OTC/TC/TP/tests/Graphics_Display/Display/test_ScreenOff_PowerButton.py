# -*- coding: utf-8 -*-
'''
Created on 04/07/2015
@author: Ding, JunnanX
'''

from testlib.graphics.common import is_device_screen_on
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase


class ScreenOffPowerButtonTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(ScreenOffPowerButtonTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(ScreenOffPowerButtonTest, self).setUp()

        self.systemui = SystemUiExtendImpl()
        self.systemui.unlock_screen()
        self.d.screen.on()
        assert is_device_screen_on(),\
            "[FAILURE] Failed DUT screen turned On."

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(ScreenOffPowerButtonTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(ScreenOffPowerButtonTest, cls).tearDownClass()

    def test_ScreenOff_PowerButton(self):
        """
        test_ScreenOff_PowerButton

        Steps:
        1. Press Power button.
            DUT screen turned Off.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. Press Power button.
            DUT screen turned Off."""
        g_common_obj.adb_cmd("input keyevent 26")
        assert not is_device_screen_on(),\
            "[FAILURE] Failed DUT screen turned Off."
