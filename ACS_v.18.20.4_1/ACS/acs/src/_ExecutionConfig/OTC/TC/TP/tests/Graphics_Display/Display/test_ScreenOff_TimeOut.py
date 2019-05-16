# -*- coding: utf-8 -*-
'''
Created on 04/07/2015
@author: Ding, JunnanX
'''

import time
from testlib.graphics.common import DBSettingsSetGet
from testlib.graphics.common import is_device_screen_on
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.system_settings_impl import SystemSettingsImpl
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase


class ScreenOffTimeOutTest(UIATestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(ScreenOffTimeOutTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(ScreenOffTimeOutTest, self).setUp()

        self.sys_setting = SystemSettingsImpl()
        self.systemui = SystemUiExtendImpl()
        self.dbsettings = DBSettingsSetGet()
        self.systemui.unlock_screen()
        self.d.screen.on()
        self.sleep_time = 30

        self.org_screen_brightness =\
            self.sys_setting.get_system_setting('screen_brightness')
        self.org_screen_off_timeout = \
            self.sys_setting.get_system_setting('screen_off_timeout')

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        self.dbsettings.set_status_stay_awake(True)
        self.sys_setting.put_system_setting('screen_brightness',
                                            self.org_screen_brightness)
        self.sys_setting.put_system_setting('screen_off_timeout',
                                            self.org_screen_off_timeout)
        super(ScreenOffTimeOutTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(ScreenOffTimeOutTest, cls).tearDownClass()

    def test_ScreenOff_TimeOut(self):
        """
        test_ScreenOff_TimeOut

        Steps:
        1. set brightness to maximum.
            DUT brightness is changed to brightest.
        2. set Sleep time to 30 seconds.
            DUT sleep time is set to 5 minutes successfully.
        3. Idle the DUT.
            DUT screen turned Off automatically after 5 minutes.
        """
        print "[RunTest]: %s" % self.__str__()

        print """[Step] 1. set brightness to maximum.
            DUT brightness is changed to brightest."""
        self.sys_setting.put_system_setting('screen_brightness', '255')

        print """[Step] 2. set Sleep time to 30 seconds.
            DUT sleep time is set to 30 seconds successfully."""
        self.dbsettings.set_status_stay_awake(False)
        self.sys_setting.put_system_setting('screen_off_timeout',
                                            str(self.sleep_time * 1000))
        self.d.screen.off()
        self.systemui.unlock_screen()
        self.d.screen.on()

        print """[Step] 3. Idle the DUT.
            DUT screen turned Off automatically after 30 seconds."""
        from testlib.gps.common import GPS_Common
        GPS_Common().kill_uiautomator()
        # start_time = time.time()
        # steep_time = 0.5
        # while time.time() - start_time <= self.sleep_time - steep_time:
        #     current_time = time.time()
        #     print "[Debug] second %.3f" % (current_time - start_time)
        #     assert is_device_screen_on(),\
        #         "[FAILURE] go to screen off early! start_time:%s current_time:%s"\
        #         % (start_time, current_time)
        #     time.sleep(steep_time)
        time.sleep(self.sleep_time + 5)
        print "-----------GFX_test_ScreenOff_TimeOut------------------"
        # from testlib.graphics.screenshot_for_liverpt import take_screenshot_for_liverpt
        # take_screenshot_for_liverpt()
        assert not is_device_screen_on(),\
            "[FAILURE] Failed automatically turned off"
