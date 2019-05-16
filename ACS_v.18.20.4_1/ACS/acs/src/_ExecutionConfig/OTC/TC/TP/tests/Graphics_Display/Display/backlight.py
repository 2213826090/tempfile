# -*- coding: utf-8 -*-
'''
Created on Oct 30, 2014

@author: yusux
'''

from testlib.systemui.systemui_impl import SystemUI
from testlib.util.uiatestbase import UIATestBase


class TestBackLight(UIATestBase):

    def setUp(self):
        super(TestBackLight, self).setUp()
        self.systemui = SystemUI()

    def tearDown(self):
        super(TestBackLight, self).tearDown()
        self.systemui = None

    def testBackLightSwitch(self):
        self.systemui.suspend_wakeup()

    def test_Backlight_SuspendResume(self):
        self.systemui.power_off_device(sleepTime=3)
        self.systemui.power_on_device(sleepTime=3)