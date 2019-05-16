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
@summary: Class for Memtrack operation
@since: 02/06/2015
@author: Yingjun Jin
'''


import time
import os
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle


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


class FitImpl(object):

    """ test the graphics fit related cases
    """

    pkg_name = "com.google.android.apps.plus"
    activity_name = "com.google.android.apps.photos.phone.PhotosHomeActivity"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._root = g_common_obj.root_on_device
        self._locator = Locator(self._device)
        self._serial = self._device.server.adb.device_serial()

    @staticmethod
    def set_environment():
        """ init the test environment
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.fit.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_png = config.read(cfg_file, 'png_source')
        png_name = cfg_png.get('name')
        png_path = arti.get(png_name)
        # unzip
        print os.system("cd " + os.path.dirname(png_path) +
                        ";tar xzf " + png_name[-10:])
        # push pictures to tar path
        png_folder = os.path.join(os.path.dirname(png_path
                                                  ) + png_name[-11:].rstrip(".tar.gz"))
        print g_common_obj.adb_cmd_common("push " +
                                          png_folder + " " + ' /sdcard/Pictures', time_out=900)
        FitImpl.set_workaround()

    @staticmethod
    def set_workaround():
        """ Reset the workaround to let the picture display after push
        """
        cmd = 'am broadcast -a android.intent.action.MEDIA_MOUNTED \
               -d file:///sdcard/Pictures'
        g_common_obj.adb_cmd(cmd)

    def sync_time_with_host(self):
        """ sync device time with host
        """
        g_common_obj.launch_app_am("com.android.settings", ".Settings")
        self._device(scrollable=True).scroll.vert.to(text="Date & time")
        self._device(text="Date & time").click()
        # set 24 hour format
        if not self._device(text="Use 24-hour format").right(
                resourceId="android:id/checkbox").checked:
            self._device(text="Use 24-hour format").right(
                resourceId="android:id/checkbox").click.wait()
        time.sleep(1)
        assert self._device(text="Use 24-hour format").right(
            resourceId="android:id/checkbox").checked
        # set timezone
        if not self._device(text="GMT+08:00 China Standard Time").exists:
            self._device(text="Select time zone").click()
            self._device(scrollable=True).scroll.vert.to(text="Shanghai")
            self._device(text="Shanghai").click()
        # set date and time
        date_time = time.strftime('%Y%m%d.%H%M%S')
        self._root()
        print "[info]--- Set time to %s" % date_time
        g_common_obj.adb_cmd('date -s "%s"' % date_time)

    def get_delay_time(self, delay):
        """ get the delay time
        """
        print "[info]--- current time %s" % time.strftime("%H:%M:%S")
        current_time = time.time()
        st = time.localtime(current_time + delay)
        print st
        print "[info]--- %ds later is %02d:%02d:%02d" % (
            delay, st.tm_hour, st.tm_min, st.tm_sec)
        self.sleep_time = (60 - st.tm_sec) + 25
        print self.sleep_time
        return st.tm_hour, st.tm_min

    def launch_alarm(self):
        """launch the alarm
        """
        g_common_obj.launch_app_am(
            "com.google.android.deskclock", "com.android.deskclock.DeskClock")
        time.sleep(2)
        self._device(description="Alarm").click()

    def delete_all_alarms(self):
        """ delete all alarm
        """
        print "[info]--- Clear all the alarms"
        if not self._device(description="Alarm").exists:
            self.launch_alarm()
        while not self._device(text="No Alarms").exists:
            if not self._device(description="Delete alarm").exists:
                self._device(description="Expand alarm").click()
            self._device(description="Delete alarm").click()
            time.sleep(1)

    def get_circle_data(self):
        """ get the circle of alarm's data
        """
        rect = self._device(
            resourceId="android:id/radial_picker").info["bounds"]
        left = int(rect["left"])
        right = int(rect["right"])
        top = int(rect["top"])
        bottom = int(rect["bottom"])
        center_x = (left + right) / 2
        center_y = (top + bottom) / 2
        r = (right - left) / 2
        return center_x, center_y, r

    def add_alarm(self, hour, minite):
        """ add alarm
        """
        print "[info]--- Set alarm time %02d:%02d" % (hour, minite)

        self._device(description="Add alarm").click()
        time.sleep(1)
#        self._device(resourceId="android:id/hours").click()
        import math
        center_x, center_y, r = self.get_circle_data()
        angle = (hour % 12) * 3.14159 / 6
        if hour > 12 or hour == 0:
            r_hour = r * 3 / 4
        else:
            r_hour = r / 2

        x = int(center_x + r_hour * math.sin(angle))
        y = int(center_y - r_hour * math.cos(angle))
        self._device.click(x, y)

        time.sleep(1)
        adjust_list = [0, 0.5, -0.5, -1, 1]
        for i in range(len(adjust_list)):
            adjust_min = (minite + adjust_list[i]) % 60
            angle = adjust_min * 3.14159 / 30
            r_minite = r * 2.0 / 3

            x = int(center_x + r_minite * math.sin(angle))
            y = int(center_y - r_minite * math.cos(angle))
            self._device.click(x, y)
            if self._device(
                    resourceId="android:id/minutes", text="%02d" % minite).exists:
                break
        self._device(text="OK").click()
        time.sleep(2)
        assert self._device(text="%02d:%02d" % (hour, minite)).exists

    def add_alarm_by_delay_time(self, delay):
        """ use delay time to run add alarm
        """
        hour, minite = self.get_delay_time(delay)
        self.add_alarm(hour, minite)

    def dismiss_alarm(self):
        """ dismiss the alarm
        """
        print "[info]--- Dismiss alarm"
        self._device.screen.off()
        time.sleep(2)
        self._device.screen.on()
        time.sleep(2)
        self._device.open.notification()
        time.sleep(2)
        self._device(text="Dismiss").click()
        time.sleep(2)
        self._device.press.back()
        if self._device(description="Unlock").exists:
            self._device.press("menu")

    def slide_view(self):
        """ view pictures as slide mode
        """
        self.launch_app_am()
        time.sleep(2)
        self._device(resourceId="com.google.android.apps.plus:id/tile_row")\
            .child(className="android.view.View").click()
        time.sleep(2)
        self._device(descriptionMatches="More options").click()
        time.sleep(2)
        self._device(text="Slideshow").click()

    def launch_app_am(self):
        """ Launch Photos via adb am command
        """
        print "Launch Photos by adb am"
        g_common_obj.launch_app_am(
            FitImpl.pkg_name, FitImpl.activity_name)
        time.sleep(2)
        if self._device(text="Later").exists:
            self._device(text="Later").click()
            time.sleep(2)
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_app_am():
        """ Stop Photos via adb am command
        """
        print "Stop Photos by adb am"
        g_common_obj.stop_app_am(FitImpl.pkg_name)

    def launch_settings_am(self):
        """ Launch Settings via adb am command
        """
        print "Launch Settings by adb am"
        g_common_obj.launch_app_am(
            "com.android.settings", "com.android.settings.Settings")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_settings_am():
        """ Stop Settings via adb am command
        """
        print "Stop Settings by adb am"
        g_common_obj.stop_app_am("com.android.settings")

    def set_pin_lock(self):
        """ Set security as pin lock
        """
        self.launch_settings_am()
        if not self._device(text="Security").exists:
            self._device(scrollable=True).scroll.to(text="Security")
        self._device(text="Security").click()
        self._device(text="Screen lock").click()
        self._device(text="PIN").click()
        if self._device(text="No thanks").exists:
            self._device(text="No thanks").click()
        if self._device(text="Continue").exists:
            self._device(text="Continue").click()
        self._device(
            resourceId="com.android.settings:id/password_entry").set_text("1234")
        time.sleep(2)
        self._device(text="Continue").click()
        self._device(
            resourceId="com.android.settings:id/password_entry").set_text("1234")
        time.sleep(2)
        self._device(text="OK").click()
        self._device(text="Done").click()
        self.stop_settings_am()

    def set_none_lock_from_pin(self):
        """ Set security as none lock
        """
        self.launch_settings_am()
        if not self._device(text="Security").exists:
            self._device(scrollable=True).scroll.to(text="Security")
        self._device(text="Security").click()
        self._device(text="Screen lock").click()
        self._device(
            resourceId="com.android.settings:id/password_entry").set_text("1234")
        time.sleep(2)
        if self._device(text="Continue").exists:
            self._device(text="Continue").click()
        if self._device(text="Next").exists:
            self._device(text="Next").click()
        g_common_obj.adb_cmd("input keyevent 66")
        self._device(text="None").click.wait()
        if self._device(text="OK").exists:
            self._device(text="OK").click.wait()
        if self._device(resourceId="android:id/button1").exists:
            self._device(resourceId="android:id/button1").click.wait()
        self.stop_settings_am()

    def suspend_and_resume(self):
        """ suspend and resume the DUTs
        """
        g_common_obj.adb_cmd("input keyevent 26")
        time.sleep(5)
        g_common_obj.adb_cmd("input keyevent 26")
        time.sleep(2)

    def unlock_pin(self):
        """ unlock pin screen lock
        """
        for _ in range(0, 3):
            if self._device(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").exists:
                self._device(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").swipe.down()
            if self._device(description="Unlock").exists:
                self._device(description="Unlock").drag.to(resourceId="com.android.systemui:id/clock_view")
            time.sleep(3)
            if self._device(resourceId="com.android.systemui:id/pinEntry").exists:
                break
        time.sleep(1)
        self._device(
            resourceId="com.android.systemui:id/pinEntry").set_text("1234")
        time.sleep(2)
        self._device(resourceId="com.android.systemui:id/key_enter").click()

    def set_live_wallpaper_bubbles(self):
        """ set live wallpaper with Bubbles
        """
        print "Set live wallpaper"
        self.launch_settings_am()
        self._device(text="Display").click()
        time.sleep(2)
        self._device(text="Wallpaper").click()
        time.sleep(2)
        self._device(text="Live Wallpapers").click()
        time.sleep(2)
        self._device(text="Bubbles").click()
        time.sleep(2)
        self._device(text="Set wallpaper").click()
        time.sleep(2)
        self.stop_settings_am()

    def test_pin_lock(self):
        """ test unlock screen pin lock
        """
        print "Test pin unlock"
        for _ in range(0, 5):
            self.suspend_and_resume()
            self.unlock_pin()

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
        self._device(description="Wallpaper 1 of 11").click()
        time.sleep(2)
        self._device(text="Set wallpaper").click()
        time.sleep(2)
        self.stop_settings_am()

    def set_password_lock(self):
        """ Set security as password lock
        """
        self.launch_settings_am()
        if not self._device(text="Security").exists:
            self._device(scrollable=True).scroll.to(text="Security")
        self._device(text="Security").click()
        self._device(text="Screen lock").click()
        self._device(text="Password").click()
        if self._device(text="No thanks").exists:
            self._device(text="No thanks").click()
        if self._device(text="Continue").exists:
            self._device(text="Continue").click()
        self._device(
            resourceId="com.android.settings:id/password_entry").set_text("1234a")
        time.sleep(2)
        self._device(text="Continue").click()
        self._device(
            resourceId="com.android.settings:id/password_entry").set_text("1234a")
        time.sleep(2)
        self._device(text="OK").click()
        self._device(text="Done").click()
        self.stop_settings_am()

    def unlock_password(self):
        """ unlock password screen lock
        """
        for _ in range(0, 3):
            if self._device(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").exists:
                self._device(resourceId="com.android.keyguard:id/keyguard_selector_view_frame").swipe.down()
            if self._device(description="Unlock").exists:
                self._device(description="Unlock").drag.to(resourceId="com.android.systemui:id/clock_view")
            time.sleep(3)
            if self._device(resourceId="com.android.systemui:id/passwordEntry").exists:
                break
        self._device(
            resourceId="com.android.systemui:id/passwordEntry").set_text("1234a")
        time.sleep(2)
        self._device.press.enter()

    def set_live_wallpaper_phasebeam(self):
        """ set live wallpaper with Phase Beam
        """
        print "Set live wallpaper"
        self.launch_settings_am()
        self._device(text="Display").click()
        time.sleep(2)
        self._device(text="Wallpaper").click()
        time.sleep(2)
        self._device(text="Live Wallpapers").click()
        time.sleep(2)
        self._device(text="Phase Beam").click()
        time.sleep(2)
        self._device(text="Set wallpaper").click()
        time.sleep(2)
        self.stop_settings_am()

    def test_password_lock(self):
        """ test unlock screen password lock
        """
        print "Test password unlock"
        for _ in range(0, 5):
            self.suspend_and_resume()
            self.unlock_password()

    def set_none_lock_from_password(self):
        """ Set security as none lock
        """
        self.launch_settings_am()
        if not self._device(text="Security").exists:
            self._device(scrollable=True).scroll.to(text="Security")
        self._device(text="Security").click()
        self._device(text="Screen lock").click()
        self._device(
            resourceId="com.android.settings:id/password_entry").set_text("1234a")
        time.sleep(2)
        if self._device(text="Continue").exists:
            self._device(text="Continue").click()
        if self._device(text="Next").exists:
            self._device(text="Next").click()
        g_common_obj.adb_cmd("input keyevent 66")
        self._device(text="None").click.wait()
        if self._device(text="OK").exists:
            self._device(text="OK").click.wait()
        if self._device(resourceId="android:id/button1").exists:
            self._device(resourceId="android:id/button1").click.wait()
        self.stop_settings_am()

    def set_airplane(self):
        """ set airplane mode
        """
        cmd = "settings put global airplane_mode_on 1"
        g_common_obj.adb_cmd(cmd)

    def unset_airplane(self):
        """ unset airplane mode
        """
        cmd = "settings put global airplane_mode_on 1"
        g_common_obj.adb_cmd(cmd)

    def enable_wifi(self):
        """ enable_wifi
        """
        self.launch_settings_am()
        self._device(textContains="Fi").click()
        if self._device(text="OFF").exists:
            self._device(
                resourceId="com.android.settings:id/switch_widget").click()
            time.sleep(120)
        self.stop_settings_am()

    def switch_portrait_landscape(self):
        """ Swith protrait and landscape 10 times
        """
        for _ in range(0, 10):
            self._device.press.recent()
            if self._device().scroll.vert.to(text="Photos"):
                self._device(text="Photos").click()
                time.sleep(3)
                self._device.press.recent()
                assert self._device().scroll.vert.to(text="Air Attack (Ads)"), "Air Attack (Ads) not exists in recent"
                self._device(text="Air Attack (Ads)").click()
                time.sleep(3)
            elif self._device().scroll.vert.to(text="Device folders"):
                self._device(text="Device folders").click()
                time.sleep(3)
                self._device.press.recent()
                assert self._device().scroll.vert.to(text="Air Attack (Ads)"), "Air Attack (Ads) not exists in recent"
                self._device(text="Air Attack (Ads)").click()
                time.sleep(3)
            else:
                assert False, "Can not found Photos or Device folders in recent"

    def launch_playstore_am(self):
        """ Launch playstore via adb am command
        """
        print "Launch playstore by adb am"
        g_common_obj.launch_app_am(
            "com.android.vending", "com.google.android.finsky.activities.MainActivity")
        time.sleep(4)
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_playstore_am():
        """ Stop playstore via adb am command
        """
        print "Stop playstore by adb am"
        g_common_obj.stop_app_am("com.android.vending")

    def close_wifi_during_download_airattack(self):
        """ Download air attack
        """

        for _ in range(0, 3):
            if self._device(resourceId="com.android.vending:id/search_box_idle_text"):
                print "----------GFX_test_progressivedownload_networkconnectionlost---------"
                try:
                    self._device(resourceId="com.android.vending:id/search_box_idle_text").set_text("Angry Birds")
                except Exception, e:
                    print "----------2GFX_test_progressivedownload_networkconnectionlost---------"
                    self._device(resourceId="com.android.vending:id/search_box_text_input").set_text("Angry Birds")
                    print e
                break
            elif self._device(resourceId="com.android.vending:id/search_button"):
                self._device(resourceId="com.android.vending:id/search_src_text").set_text("Angry Birds")
                break
            else:
                self.launch_playstore_am()
        time.sleep(2)
        self._device.press.enter()
        self._device(text="Angry Birds", resourceId="com.android.vending:id/li_title").wait.exists(timeout=15000)
        time.sleep(1)
        self._device(text="Angry Birds", resourceId="com.android.vending:id/li_title").click.wait()
        if self._device(text="GET STARTED").exists:
            self._device(text="GET STARTED").click.wait()
        time.sleep(1)
        if self._device(text="UNINSTALL"):
            self._device(text="UNINSTALL").click.wait()
            time.sleep(1)
        if self._device(text="OK"):
            self._device(text="OK").click.wait()
            time.sleep(1)
        if self._device(text="SKIP"):
            self._device(text="SKIP").click.wait()
            time.sleep(1)
        self._device(text="INSTALL").click.wait()
        time.sleep(2)
        if self._device(text="CONTINUE"):
            self._device(text="CONTINUE").click.wait()
        time.sleep(1)
        for _ in range(0, 5):
            if self._device(text="ACCEPT").exists:
                self._device(text="ACCEPT").click.wait()
                break
        if self._device(text="Continue").exists:
            self._device(text="Continue").click.wait()
            time.sleep(2)
            self._device.press.back()
            time.sleep(1)
        if self._device(textContains="%").wait.exists(timeout=10000):
            self.close_wifi()
            time.sleep(2)
        if self._device(textContains="Waiting for network").wait.exists(timeout=30000):
            assert self._device(textContains="Waiting for network"), "The download not stuck"
            self._device(resourceId="com.android.vending:id/cancel_download").click()

    def close_wifi(self):
        """ Close wifi
        """
        try:
            cmd = 'svc wifi disable'
            g_common_obj.adb_cmd_capture_msg(repr(cmd))
        except:
            pass

    def open_wifi(self):
        """ Open wifi
        """
        try:
            cmd = 'svc wifi enable'
            g_common_obj.adb_cmd_capture_msg(repr(cmd))
        except:
            pass

    def open_google_web_and_disconnect_wifi(self):
        """ open google web
        """
        #self._device(resourceId="com.android.chrome:id/url_bar").clear_text()
        self._device.press("recent")
        time.sleep(2)
        self._device.press("back")
        time.sleep(2)
        self._device(resourceId="com.android.chrome:id/url_bar").set_text('www.bing.com')
        self._device.press.enter()
        time.sleep(20)
        g_common_obj.adb_cmd_capture_msg("svc wifi disable")
        time.sleep(1)

    def check_web_after_disconnect_wifi(self):
        """ check web after disconnect wifi
        """
        print "check web after disconnect wifi"
        key = "bing"
        self._device.press.menu()
        time.sleep(3)
        if not self._device(text="Find in page").exists:
            assert self._device(scrollable=True).scroll.to(text="Find in page"),\
                    "UIobj not found in viewList."
            self._device(text="Find in page").click()
        else:
            self._device(text="Find in page").click()
        time.sleep(3)
        self._device(resourceId="com.android.chrome:id/find_query"
                     ).set_text(key)
        time.sleep(3)
        assert self._device(text="1/1"
                            ).exists, "The test case failed or timeout"
