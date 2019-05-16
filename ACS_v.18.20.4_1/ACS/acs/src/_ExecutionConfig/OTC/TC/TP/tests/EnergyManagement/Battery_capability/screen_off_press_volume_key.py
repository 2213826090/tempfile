#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.energy import Energy

class ScreenOffPressVolumeKey(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.energy = Energy()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_screen_off_press_volume_key(self):
        """
        Screen off press volume key
        """
        print "[RunTest]: %s" % self.__str__()

        self.energy.set_screen_status("off")
        self.energy.input_keyevent("KEYCODE_VOLUME_UP")
        self.energy.verify_screen_status("off")
        self.energy.input_keyevent("KEYCODE_VOLUME_DOWN")
        self.energy.verify_screen_status("off")

