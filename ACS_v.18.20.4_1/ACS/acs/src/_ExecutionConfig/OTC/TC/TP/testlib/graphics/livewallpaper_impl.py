# Copyright (C) 2015
# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

'''
@summary: Class for LiveWallpaperImpl
@since: 07/08/2015
@author: Zhang,RongX Z
'''


import time
from testlib.util.common import g_common_obj

class Locator(object):

    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=5):
            return uiobj.wait("exists", timeout * 500)
        return _wait

    @property
    def performance_tests(self):
        return self._device(text="Performance Tests")

class LiveWallpaperImpl(object):

    """ test the graphics liveWallpaper related cases
    """
    def __init__(self):
        self._device = g_common_obj.get_device()
        self._root = g_common_obj.root_on_device
        self._locator = Locator(self._device)

    def set_livewallpaper_Bubble(self):
        """ set live wallpaper with Bubble
        """
        print "Set live wallpaper Bubble"
        self.launch_settings_am()
        self._device(text="Display").click.wait()
        time.sleep(2)
        self._device(text="Wallpaper").click.wait()
        time.sleep(2)
        self._device(text="Live Wallpapers").click.wait()
        time.sleep(2)
        self._device(text="Bubbles").click.wait()
        time.sleep(2)
        self._device(text="Set wallpaper").click.wait()
        time.sleep(2)
        self._device.press.home()
        time.sleep(1)
        g_common_obj.assert_exp_happens()

    def set_livewallpaper_HoloSpiral(self):
        """ set live wallpaper with HoloSpiral
        """
        self._device.press.home()
        x = self._device.info["displayWidth"]
        y = self._device.info["displayHeight"]
        for _ in range(0,3):
            self._device.long_click(x/2, y*7/8)
            if self._device(text="Wallpapers").exists:
                break
            else:
                self._device.long_click(x/4, y*7/8)
                if self._device(text="Wallpapers").exists:
                    break
        self._device(text="Wallpapers").click.wait()
        time.sleep(1)
        self._device(scrollable=True).scroll.horiz.toEnd()
        time.sleep(1)
        self._device(text="Holo Spiral").click.wait()
        time.sleep(4)
        self._device(text="Set wallpaper").click.wait()
        time.sleep(2)
        self._device.press.home()
        time.sleep(1)
        g_common_obj.assert_exp_happens()

    def set_livewallpaper_PhaseBeam(self):
        """ set live wallpaper with Phase Beam
        """
        print "Set live wallpaper with Phase Beam"
        self.launch_settings_am()
        self._device(text="Display").click.wait()
        time.sleep(2)
        self._device(text="Wallpaper").click.wait()
        time.sleep(2)
        self._device(text="Live Wallpapers").click.wait()
        time.sleep(2)
        self._device(text="Phase Beam").click.wait()
        time.sleep(2)
        self._device(text="Set wallpaper").click.wait()
        time.sleep(2)
        self._device.press.home()
        time.sleep(1)
        g_common_obj.assert_exp_happens()

    def set_livewallpaper_BlackHole(self):
        """ set live wallpaper with BlackHole
        """
        print "Set live wallpaper with BlackHole"
        self._device.press.home()
        x = self._device.info["displayWidth"]
        y = self._device.info["displayHeight"]
        for _ in range(0,3):
            self._device.long_click(x/2, y*7/8)
            if self._device(text="Wallpapers").exists:
                break
            else:
                self._device.long_click(x/4, y*7/8)
                if self._device(text="Wallpapers").exists:
                    break
        self._device(text="Wallpapers").click.wait()
        time.sleep(1)
        self._device(scrollable=True).scroll.horiz.toEnd()
        time.sleep(1)
        self._device(text="Black Hole").click.wait()
        time.sleep(4)
        self._device(text="Set wallpaper").click.wait()
        time.sleep(2)
        self._device.press.home()
        time.sleep(1)
        self.change_to_landscape()
        time.sleep(4)
        g_common_obj.assert_exp_happens()
        self.change_to_vertical()
        self.suspend_resume_device()
        g_common_obj.assert_exp_happens()

    def change_orientation(self,orientation):
        for _ in range(0,3):
            self._device.orientation = orientation
            if self._device.orientation == orientation:
                break

    def change_to_vertical(self):
        """ judge wether is vertical,change to vertical if it not
        """
        print "change to vertical"
        width = self._device.info["displayWidth"]
        height = self._device.info["displayHeight"]
        orientation = self._device.info["displayRotation"]
        if width > height and orientation == 0:
            self.change_orientation("r")
        elif width > height and orientation > 0:
            self.change_orientation("n")
        self._device.freeze_rotation()

    def change_to_landscape(self):
        """ judge wether is landscape,change to landscape if it not
        """
        print "change to landscape"
        y = self._device.info["displayHeight"]
        x = self._device.info["displayWidth"]
        orientation = self._device.info["displayRotation"]
        if y > x and orientation == 0:
            self.change_orientation("l")
        elif y > x and orientation > 0:
            self.change_orientation("n")
        self._device.freeze_rotation()

    def suspend_resume_device(self):
        self._device.sleep()
        time.sleep(5)
        self._device.wakeup()
        if self._device(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").exists:
            self._device(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").swipe.down()
        if self._device(description="Unlock").exists:
            self._device(description="Unlock").drag.to(resourceId = "com.android.systemui:id/clock_view")
        time.sleep(3)

    def launch_settings_am(self):
        """ Launch Settings via adb am command
        """
        print "Launch Settings by adb am"
        g_common_obj.launch_app_am(
            "com.android.settings", "com.android.settings.Settings")
        self._locator.wait_exist(self._locator.performance_tests)

    def set_wallpaper(self):
        """ set wallpaper with
        """
        print "Set wallpaper"
        self.launch_settings_am()
        self._device(text="Display").click()
        time.sleep(2)
        self._device(text="Wallpaper").click()
        time.sleep(2)
        self._device(text="Wallpapers").click()
        time.sleep(2)
        self._device(className="android.widget.FrameLayout",index=2).click()
        time.sleep(2)
        self._device(text="Set wallpaper").click()
        time.sleep(2)

    @staticmethod
    def stop_settings_am():
        """ Stop Settings via adb am command
        """
        print "Stop Settings by adb am"
        g_common_obj.stop_app_am("com.android.settings")