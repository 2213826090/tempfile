#! /usr/bin/env python
# coding:utf-8

import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.em.em_impl import EMImpl

class DCPPlay3DGameTemperature(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.app_package = "com.imangi.templerun2"
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()
        self.emImpl.install_app(self.config.read(self.cfg_file, "artifactory"), "location", "templerun", self.app_package)
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        self.emImpl.setSleepMode("30 minutes")
        g_common_obj.close_background_apps()
        super(DCPPlay3DGameTemperature, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.emImpl.three_way_cutter_reconnect_sdp(3, 2, 5)
        self.emImpl.set_screen_status("on")
        self.emImpl.unlock_screen()
        g_common_obj.close_background_apps()
        super(DCPPlay3DGameTemperature, self).tearDown()

    def test_DCP_play_3D_game_temperature(self):
        """
        DCP charging play 3D game temperature
        """
        print "[RunTest]: %s" % self.__str__()

        # play 3D game
        self.emImpl.launch_templerun()
        time.sleep(60)
        self.emImpl.play_templerun()
        # DCP charge for 30min
        self.emImpl.enable_dcp_charging(2, 2)
        time.sleep(1800)
        self.emImpl.three_way_cutter_reconnect_sdp(3, 2, 5)
        assert self.emImpl.get_power_temperature() / 10 < 55

