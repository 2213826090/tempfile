# -*- coding: utf-8 -*-
# Copyright (C) 2015  Jin, YingjunX <yingjunx.jin@intel.com>
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
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

'''
@summary: Class for set contact photo Operation
@since: 08/31/2015
@author: Xiangyi Zhao
'''


import time
from testlib.util.common import g_common_obj
from testlib.graphics.common import osversion


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


class WallpaperImpl(object):

    """ test the graphics liveWallpaper related cases
    """

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._root = g_common_obj.root_on_device
        self._locator = Locator(self._device)

    def launch_settings_am(self):
        """ Launch Settings via adb am command
        """
        print "Launch Settings by adb am"
        g_common_obj.launch_app_am(
            "com.android.settings", "com.android.settings.Settings")
        self._locator.wait_exist(self._locator.performance_tests)

    def launch_Settings_Display(self):
        """ 
        Launch Settings Display via adb am command
        """
        g_common_obj.launch_app_am("com.android.settings", "com.android.settings.DisplaySettings")

    def launch_set_wallpaper_by_desktop(self):
        """ launch set wallpaper by desktop
        """
        print "launch set wallpaper by desktop"
        self._device.press.home()
        x = self._device.info["displayWidth"]
        y = self._device.info["displayHeight"]
        for _ in range(0, 3):
            self._device.long_click(5, y * 7 / 8)
            if self._device(text="Wallpapers").exists:
                break
            else:
                self._device.long_click(x / 4, y * 3 / 4)
                if self._device(text="Wallpapers").exists:
                    break

    def launch_set_wallpaper_by_settings(self):
        """ launch set wallpaper by settings
        """
        print "Launch set wallpaper by settings"
        self.launch_settings_am()
        self._device(text="Display").click.wait()
        time.sleep(2)
        self._device(text="Wallpaper").click.wait()
        time.sleep(2)

    def launch_set_livewallpaper_by_Settings(self):
        self.launch_Settings_Display()
#         self._device(text="Display").click.wait()
#         time.sleep(2)
        self._device(text="Wallpaper").click.wait()
        time.sleep(2)
        if not self._device(text="Wallpapers").exists:
            if self._device(text="Google Now Launcher").exists:
                self._device(text="Google Now Launcher").click.wait()
        else:
            self._device(text="Live Wallpapers").click.wait()
        time.sleep(2)
        if self._device(className="android.widget.RelativeLayout", index=0).exists:
            self._device(className="android.widget.RelativeLayout", index=0).click.wait()

    @staticmethod
    def stop_settings_am():
        """ Stop Settings via adb am command
        """
        print "Stop Settings by adb am"
        g_common_obj.stop_app_am("com.android.settings")

    def reset_wallpaper(self):
        """ reset wallpaper
        """
        print "Reset wallpaper"
        self.launch_settings_am()
        self._device(text="Display").click.wait()
        time.sleep(2)
        self._device(text="Wallpaper").click.wait()
        time.sleep(2)
        if not self._device(text="Wallpapers").exists:
            if self._device(text="Google Now Launcher").exists:
                self._device(text="Google Now Launcher").click.wait()
        elif self._device(descriptionContains="Wallpaper").exists:
            self._device(descriptionContains="Wallpaper").click.wait()
        else:
            self._device(text="Wallpapers").click.wait()
        time.sleep(2)
        if self._device(className="android.widget.FrameLayout", index=2).exists:
            self._device(className="android.widget.FrameLayout", index=2).click.wait()
        time.sleep(2)
        self._device(textContains="Set wallpaper").click()
        time.sleep(2)
        self._device.press.home()
        time.sleep(1)

    def set_livewallpaper_by_desktop(self, wallpaper):
        """ set live wallpaper by desktop
        """
        print "Set live wallpaper by desktop"
        self.launch_set_wallpaper_by_desktop()
        try:
            self._device(text="Wallpapers").click.wait()
            time.sleep(1)
        except:
            print "can't find text 'Wallpapers'"
        if not self._device(text=wallpaper).exists:
            self._device().scroll.horiz.to(text=wallpaper)
            time.sleep(1)
            self._device(text=wallpaper).click.wait()
            time.sleep(4)
        else:
            self._device(text=wallpaper).click.wait()
            time.sleep(4)
        try:
            self._device(textContains="Set wallpaper").click.wait()
        except:
            print "Can't find text 'Set wallpaper'"
        time.sleep(2)
        self._device.press.home()
        time.sleep(1)
        g_common_obj.assert_exp_happens()

    def set_livewallpaper_by_Settings(self):
        self.launch_settings_am()
        self._device(text="Display").click.wait()
        time.sleep(2)
        self._device(text="Wallpaper").click.wait()
        time.sleep(2)
        if not self._device(text="Wallpapers").exists:
            if self._device(text="Google Now Launcher").exists:
                self._device(text="Google Now Launcher").click.wait()
        else:
            self._device(text="Live Wallpapers").click.wait()
        time.sleep(2)
        if self._device(className="android.widget.RelativeLayout", index=0).exists:
            self._device(className="android.widget.RelativeLayout", index=0).click.wait()
        if self._device(text="Set wallpaper").exists:
            self._device(text="Set wallpaper").click.wait()
        if self._device(text="Home screen and lock screen").exists:
            self._device(text="Home screen and lock screen").click.wait()

    def preview_livewallpaper_by_desktop(self, wallpaper):
        """ preview live wallpaper by desktop
        """
        print "Preview live wallpaper by desktop"
        self._device(text="Wallpapers").click.wait()
        time.sleep(1)
        self._device(scrollable=True).scroll.horiz.to(text=wallpaper)
        time.sleep(1)
        self._device(text=wallpaper).click.wait()
        time.sleep(4)

    def set_wallpaper_by_desktop(self, wallpaper):
        """ set wallpaper by desktop
            The wallpaper parameter just need the number of description from uiautomator,
            Eg:description=Wallpaper 7 of 10;you just need imput 7 as parameter;
            To distinguish total number is 10 or 11,it depends on android M or android L.
        """
        print "Set wallpaper by desktop"
        version_array = osversion.get_android_version()
        androidversion = version_array[0]
        if androidversion == 7:
            print "osversion is N"
            wallpaper = "Wallpaper %s of 10" % (wallpaper)
        elif androidversion == 6:
            print "osversion is M"
            wallpaper = "Wallpaper %s of 10" % (wallpaper)
        elif androidversion == 5:
            print "osversion is L"
            wallpaper = "Wallpaper %s of 11" % (wallpaper)
        else:
            print "osversion is %s" % (androidversion)
            wallpaper = "Wallpaper %s of 11" % (wallpaper)
        self.launch_set_wallpaper_by_desktop()
        self._device(text="Wallpapers").click.wait()
        time.sleep(1)
        self._device(scrollable=True).scroll.horiz.to(description=wallpaper)
        time.sleep(1)
        self._device(description=wallpaper).click.wait()
        time.sleep(4)
        self._device(textContains="Set wallpaper").click.wait()
        time.sleep(2)
        self._device.press.home()
        time.sleep(1)
        g_common_obj.assert_exp_happens()

    def set_livewallpaper_by_settings(self, wallpaper):
        """ set live wallpaper by settings
        """
        print "Set live wallpaper by settings"
        self.launch_set_wallpaper_by_settings()
        self._device(text="Live Wallpapers").click.wait()
        time.sleep(2)
        self._device(text=wallpaper).click.wait()
        time.sleep(2)
        self._device(textContains="Set wallpaper").click.wait()
        time.sleep(2)
        self._device.press.home()
        time.sleep(1)
        g_common_obj.assert_exp_happens()

    def set_wallpaper_by_settings(self, wallpaper):
        """ set wallpaper by settings
            The wallpaper parameter just need the number of description from uiautomator,
            Eg:description=Wallpaper 7 of 10;you just need imput 7 as parameter;
            To distinguish total number is 10 or 11,it depends on android M or android L.
        """
        print "Set wallpaper by settings"
        version_array = osversion.get_android_version()
        androidversion = version_array[0]
        if androidversion == 7:
            print "osversion is N"
            wallpaper = "Wallpaper %s of 10" % (wallpaper)
        elif androidversion == 6:
            print "osversion is M"
            wallpaper = "Wallpaper %s of 10" % (wallpaper)
        elif androidversion == 5:
            print "osversion is L"
            wallpaper = "Wallpaper %s of 11" % (wallpaper)
        else:
            print "osversion is %s" % (androidversion)
            wallpaper = "Wallpaper %s of 11" % (wallpaper)
        self.launch_set_wallpaper_by_settings()
        if not self._device(text="Wallpapers").exists:
            self._device(text="Google Now Launcher").click.wait()
        else:
            self._device(text="Wallpapers").click.wait()
        time.sleep(1)
        self._device(scrollable=True).scroll.horiz.to(text=wallpaper)
        time.sleep(1)
        self._device(text=wallpaper).click.wait()
        time.sleep(4)
        self._device(textContains="Set wallpaper").click.wait()
        time.sleep(2)
        self._device.press.home()
        time.sleep(1)
        g_common_obj.assert_exp_happens()
