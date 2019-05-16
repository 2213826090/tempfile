#! /usr/bin/env python
# coding:utf-8

import time

from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl

class long_time_idle_overnight(UIATestBase):
    """
    DUT idle overnight without any radio connected, check battery level normally or not.
    """
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.emImpl.change_lock_screen_status("Swipe")
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.setSleepMode("30 seconds")
        g_common_obj.close_background_apps()
        super(long_time_idle_overnight, self).setUp()

    def tearDown(self):
        super(long_time_idle_overnight, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def test_long_time_idle_overnight(self):

        level1 = self.emImpl.get_battery_level()
        self.emImpl.set_screen_status("off")
        self.emImpl.set_three_way_cutter_usb(0)
        time.sleep(12*3600)
        self.emImpl.three_way_cutter_reconnect_sdp(3, 2, 5)
        level2 = self.emImpl.get_battery_level()
        assert level1 >= level2
        assert level1 - level2 > 5

