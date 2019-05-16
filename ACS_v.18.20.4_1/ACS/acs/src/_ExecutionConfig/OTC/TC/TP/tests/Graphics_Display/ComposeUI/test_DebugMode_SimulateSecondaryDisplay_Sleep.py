# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 06/04/2015
@author: Zhang RongX,Z
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.debugmode_sleep_impl import DebugModeSleepImpl
from testlib.graphics.debugmode_impl import DebugModeImpl
from testlib.util.common import g_common_obj
from testlib.graphics.common import osversion

class DebugModeSimulateSecondaryDisplaySleep(UIATestBase):

    def setUp(self):
        super(DebugModeSimulateSecondaryDisplaySleep, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._debugmodeSleep = DebugModeSleepImpl()
        self._debugmode = DebugModeImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DebugModeSimulateSecondaryDisplaySleep, self).tearDown()
        self._debugmode.set_secondary_displays_none()

    def test_DebugMode_SimulateSecondaryDisplay_Sleep(self):
        ''' refer TC test_DebugMode_SimulateSecondaryDisplay
        '''
        print "[RunTest]: %s" % self.__str__()
        version_array = osversion.get_android_version()
        androidversion = version_array[0]
        if androidversion == 7:
            print "osversion is N"
            self._debugmodeSleep.launch_settings_am()
            self._debugmodeSleep.change_to_vertical()
            self._debugmodeSleep.choose_simulate_secondary_displays()
            self._debugmodeSleep.change_simulate_720p_1080p_dualscreen()
            self._debugmodeSleep.change_simulate_4K_upscaled_secure()
            self._debugmodeSleep.change_simulate_4K_upscaled()
            self._debugmodeSleep.change_simulate_4K_secure()
            self._debugmodeSleep.change_simulate_4K()
            self._debugmodeSleep.change_simulate_1080p_secure()
            self._debugmodeSleep.change_simulate_1080p()
            self._debugmodeSleep.change_simulate_720p_secure()
            self._debugmodeSleep.change_simulate_720p()
            self._debugmodeSleep.change_simulate_480p_secure()
            self._debugmodeSleep.change_simulate_480p()
            self._debugmodeSleep.change_simulate_None()
            self._debugmodeSleep.stop_settings_am()
        elif androidversion == 6:
            print "osversion is M"
            self._debugmodeSleep.launch_settings_am()
            self._debugmodeSleep.change_to_vertical()
            self._debugmodeSleep.choose_simulate_secondary_displays()
            self._debugmodeSleep.change_simulate_720p_1080p_dualscreen()
            self._debugmodeSleep.change_simulate_4K_upscaled_secure()
            self._debugmodeSleep.change_simulate_4K_upscaled()
            self._debugmodeSleep.change_simulate_4K_secure()
            self._debugmodeSleep.change_simulate_4K()
            self._debugmodeSleep.change_simulate_1080p_secure()
            self._debugmodeSleep.change_simulate_1080p()
            self._debugmodeSleep.change_simulate_720p_secure()
            self._debugmodeSleep.change_simulate_720p()
            self._debugmodeSleep.change_simulate_480p_secure()
            self._debugmodeSleep.change_simulate_480p()
            self._debugmodeSleep.change_simulate_None()
            self._debugmodeSleep.stop_settings_am()
        elif androidversion == 5:
            print "osversion is L"
            self._debugmodeSleep.launch_settings_am()
            self._debugmodeSleep.change_to_vertical()
            self._debugmodeSleep.choose_simulate_secondary_displays()
            self._debugmodeSleep.change_simulate_720x480()
            self._debugmodeSleep.change_simulate_1280x720()
            self._debugmodeSleep.change_simulate_1280x720_secure()
            self._debugmodeSleep.change_simulate_1920x1080()
            self._debugmodeSleep.change_simulate_1920x1080_secure()
            self._debugmodeSleep.change_simulate_1280x720_and_1920x1080()
            self._debugmodeSleep.change_simulate_720x480_secure()
            self._debugmodeSleep.change_simulate_None()
            self._debugmodeSleep.stop_settings_am()
        else:
            print "osversion is %s" % (androidversion)
            self._debugmodeSleep.launch_settings_am()
            self._debugmodeSleep.change_to_vertical()
            self._debugmodeSleep.choose_simulate_secondary_displays()
            self._debugmodeSleep.change_simulate_720x480()
            self._debugmodeSleep.change_simulate_1280x720()
            self._debugmodeSleep.change_simulate_1280x720_secure()
            self._debugmodeSleep.change_simulate_1920x1080()
            self._debugmodeSleep.change_simulate_1920x1080_secure()
            self._debugmodeSleep.change_simulate_1280x720_and_1920x1080()
            self._debugmodeSleep.change_simulate_720x480_secure()
            self._debugmodeSleep.change_simulate_None()
            self._debugmodeSleep.stop_settings_am()