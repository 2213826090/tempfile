#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.basic_ui import LockScreenUI
from testlib.em.energy import Energy

class SDP_Charging_Lockscreen(UIATestBase):
    """
    Battery percentage - Less than 15% - Lock screen - Password mode
    """
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.screen = LockScreenUI()
        self.screen.set_screen_status("on")
        self.screen.unlock_screen()
        self.encrypt = None
        #super(SDP_Charging_Lockscreen, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        #self.encrypt.remove_lock()
        #super(SDP_Charging_Lockscreen, self).tearDown()

    def verify_charging_info_in_lockscreen(self):
        # set screen lock
        self.encrypt.set_lock()
        # check charging in lock screen
        self.screen.lock_screen()
        status = self.screen.get_charging_status()
        print status
        # unlock screen
        self.encrypt.unlock_screen()
        # remove screen lock
        self.encrypt.remove_lock()

        assert status

    def test_SDP_less_than_15_password_lockscreen(self):
        from testlib.em.screen_lock import CPassword
        self.encrypt = CPassword()
        info = Energy().get_battery_info()
        assert info["level"] <= 15
        self.verify_charging_info_in_lockscreen()

    def test_SDP_less_than_15_pattern_lockscreen(self):
        from testlib.em.screen_lock import CPattern
        self.encrypt = CPattern()
        info = Energy().get_battery_info()
        assert info["level"] <= 15
        self.verify_charging_info_in_lockscreen()

    def test_SDP_less_than_15_pin_lockscreen(self):
        from testlib.em.screen_lock import CPIN
        self.encrypt = CPIN()
        info = Energy().get_battery_info()
        assert info["level"] <= 15
        self.verify_charging_info_in_lockscreen()

    def test_SDP_less_than_15_swipe_lockscreen(self):
        info = Energy().get_battery_info()
        assert info["level"] <= 15
        self.screen.lock_screen()
        status = self.screen.get_charging_status()
        assert status

    def test_SDP_more_than_15_password_lockscreen(self):
        from testlib.em.screen_lock import CPassword
        self.encrypt = CPassword()
        info = Energy().get_battery_info()
        assert info["level"] > 15
        self.verify_charging_info_in_lockscreen()

    def test_SDP_more_than_15_pattern_lockscreen(self):
        from testlib.em.screen_lock import CPattern
        self.encrypt = CPattern()
        info = Energy().get_battery_info()
        assert info["level"] > 15
        self.verify_charging_info_in_lockscreen()

    def test_SDP_more_than_15_pin_lockscreen(self):
        from testlib.em.screen_lock import CPIN
        self.encrypt = CPIN()
        info = Energy().get_battery_info()
        assert info["level"] > 15
        self.verify_charging_info_in_lockscreen()

    def test_SDP_more_than_15_swipe_lockscreen(self):
        info = Energy().get_battery_info()
        assert info["level"] > 15
        self.screen.lock_screen()
        status = self.screen.get_charging_status()
        assert status

