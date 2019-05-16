# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 05/18/2015
@author: Xiangyi Zhao
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.debugmode_impl import DebugModeImpl
from testlib.graphics.common import osversion


class DebugModeSimulateSecondaryDisplay(UIATestBase):

    def setUp(self):
        super(DebugModeSimulateSecondaryDisplay, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._debugmode = DebugModeImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DebugModeSimulateSecondaryDisplay, self).tearDown()
        self._debugmode.set_secondary_displays_none()

    def test_DebugMode_SimulateSecondaryDisplay(self):
        ''' refer TC test_DebugMode_SimulateSecondaryDisplay
        '''
        print "[RunTest]: %s" % self.__str__()
        version_array = osversion.get_android_version()
        androidversion = version_array[0]
        if androidversion in (5, 6, 7, 8):
            print "osversion is M, N, O"
            self._debugmode.launch_settings_am()
            self._debugmode.choose_simulate_secondary_displays()
            self._debugmode.change_simulate_720p_1080p_dualscreen()
            self._debugmode.change_simulate_4K_upscaled_secure()
            self._debugmode.change_simulate_4K_upscaled()
            self._debugmode.change_simulate_4K_secure()
            self._debugmode.change_simulate_4K()
            self._debugmode.change_simulate_1080p_secure()
            self._debugmode.change_simulate_1080p()
            self._debugmode.change_simulate_720p_secure()
            self._debugmode.change_simulate_720p()
            self._debugmode.change_simulate_480p_secure()
            self._debugmode.change_simulate_480p()
            self._debugmode.change_simulate_None()
            self._debugmode.stop_settings_am()
        else:
            print "osversion is %s" % (androidversion)
            self._debugmode.launch_settings_am()
            self._debugmode.choose_simulate_secondary_displays()
            self._debugmode.change_simulate_1280x720_and_1920x1080()
            self._debugmode.change_simulate_1920x1080_secure()
            self._debugmode.change_simulate_1920x1080()
            self._debugmode.change_simulate_1280x720_secure()
            self._debugmode.change_simulate_1280x720()
            self._debugmode.change_simulate_720x480_secure()
            self._debugmode.change_simulate_720x480()
            self._debugmode.change_simulate_None()
            self._debugmode.stop_settings_am()
