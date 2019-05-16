# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 06/08/2015
@author: Zhang RongX,Z
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.debugmode_drag_impl import DebugModeDragImpl
from testlib.graphics.debugmode_impl import DebugModeImpl
from testlib.graphics.common import osversion, adb32


class DebugModeSimulateSecondaryDisplayRotation(UIATestBase):

    def setUp(self):
        super(DebugModeSimulateSecondaryDisplayRotation, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._debugmodeDrag = DebugModeDragImpl()
        self._debugmode = DebugModeImpl()
        adb32.screen_rotation(3)

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DebugModeSimulateSecondaryDisplayRotation, self).tearDown()
        self._debugmode.set_secondary_displays_none()
        adb32.screen_rotation(0)

    def test_DebugMode_SimulateSecondaryDisplay_Rotation(self):
        ''' refer TC test_DebugMode_SimulateSecondaryDisplay_Rotation
        '''
        print "[RunTest]: %s" % self.__str__()
        version_array = osversion.get_android_version()
        androidversion = version_array[0]
        if androidversion in (5, 6, 7, 8):
            print "osversion is M,N,O"
            self._debugmodeDrag.launch_settings_am()
            self._debugmodeDrag.choose_simulate_secondary_displays()
            self._debugmodeDrag.change_simulate_720p_1080p_dualscreen()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_4K_upscaled_secure()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_4K_upscaled()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_4K_secure()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_4K()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_1080p_secure()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_1080p()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_720p_secure()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_720p()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_480p_secure()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_480p()
            self._debugmodeDrag.change_simulate_None()
            self._debugmodeDrag.stop_settings_am()
        else:
            print "osversion is %s" % (androidversion)
            self._debugmodeDrag.launch_settings_am()
            self._debugmodeDrag.choose_simulate_secondary_displays()
            self._debugmodeDrag.change_simulate_1280x720_and_1920x1080()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_1920x1080_secure()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_1920x1080()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_1280x720_secure()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_1280x720()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_720x480_secure()
            self._debugmodeDrag.drag_simulated_secondary_display()
            self._debugmodeDrag.change_simulate_720x480()
            self._debugmodeDrag.change_simulate_None()
            self._debugmodeDrag.stop_settings_am()