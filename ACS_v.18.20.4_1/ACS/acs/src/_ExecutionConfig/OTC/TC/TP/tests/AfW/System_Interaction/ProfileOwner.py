# -*- coding: utf-8 -*-

from testlib.common.common import g_common_obj2
from testlib.AfW.api_impl import ApiImpl
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
import time
import os

class MultiUserSupport(UIATestBase):
    """
    @summary: Test cases for multi user support
    """

    def setUp(self):
        super(MultiUserSupport, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.d.watchers.run()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(MultiUserSupport, self).tearDown()
        # reboot device since there is a chance that we get error before switch back to owner user
        # that will impact some later testing
        g_common_obj2.system_reboot(90)
        for _ in range(20):
            self.api = ApiImpl()
            self.api.d.wakeup()
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                break
            time.sleep(5)
        g_common_obj.set_vertical_screen()
        print "[Teardown]: %s" % self._test_name

    def testAdd_Shift_Remove_User(self):
        """
        verify that able to add and remove user under profile owner
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.USER_SETTINGS", False)  # launch settings
        if not self.api.check_ui_exists("text", "Users", 5):
            self.api.settings_sub_launch("Users")
        if self.api.check_ui_exists("text", "New user"):  # remove "New user" if it already exist
            if self.api.check_ui_exists("resourceId", "com.android.settings:id/trash_user"):
                self.api.d(text="New user").right(resourceId="com.android.settings:id/trash_user").click()
                self.api.click_with_timeout("text", "Delete")
            elif self.api.check_ui_exists("resourceId", "com.android.settings:id/manage_user"):
                self.api.d(text="New user").right(resourceId="com.android.settings:id/manage_user").click()
                self.api.click_with_timeout("text", "Remove user")
                self.api.click_with_timeout("text", "Delete")
                self.api.clean_tasks()
                self.api.launch_app_by_intents("android.settings.USER_SETTINGS", False)  # launch settings
                if not self.api.check_ui_exists("text", "Users", 5):
                    self.api.settings_sub_launch("Users")
        for _ in range(10):
            if self.api.check_ui_exists("text", "Set up now"):
                break
            if self.api.check_ui_exists("textContains", "Add user"):
                self.api.click_with_timeout("textContains", "Add user")
                self.api.click_with_timeout("text", "User")
                self.api.click_with_timeout("text", "OK")
        assert self.api.check_ui_exists("text", "Set up now"), "[ERROR]: fail to create new user"
        self.api.click_with_timeout("text", "Set up now")
        time.sleep(90)  # try to sleep 90 seconds since usb device will disconnect when switching user
        for _ in range(10):
            self.api.d.wakeup()
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                assert self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view), \
                    "[ERROR]: fail to switch to new user"
                self.api.unlock_screen()
                break
            time.sleep(5)
        self.api.launch_app_by_intents("android.settings.USER_SETTINGS", False)
        assert self.api.check_ui_exists("textContains", "You (New User)", 5), "[ERROR]: fail to switch to new user"
        self.api.click_with_timeout("textContains", "Owner", 5)
        time.sleep(90)  # try to sleep 90 seconds since usb device will disconnect when switching user
        for _ in range(10):
            self.api.d.wakeup()
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                assert self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view), \
                    "[ERROR]: fail to switch back to owner"
                self.api.unlock_screen()
                break
            time.sleep(5)
        self.api.launch_app_by_intents("android.settings.USER_SETTINGS", False)  # launch settings
        if not self.api.check_ui_exists("text", "Users"):
            self.api.settings_sub_launch("Users")
        assert self.api.check_ui_exists("text", "New user"), "[ERROR]: New user doesn't exist"
        if self.api.check_ui_exists("text", "New user"):  # remove "New user"
            if self.api.check_ui_exists("resourceId", "com.android.settings:id/trash_user"):
                self.api.d(text="New user").right(resourceId="com.android.settings:id/trash_user").click()
                self.api.click_with_timeout("text", "Delete")
            elif self.api.check_ui_exists("resourceId", "com.android.settings:id/manage_user"):
                self.api.d(text="New user").right(resourceId="com.android.settings:id/manage_user").click()
                self.api.click_with_timeout("text", "Remove user")
                self.api.click_with_timeout("text", "Delete")

    def testGuest_Shift(self):
        """
        verify that able to switch to guest user
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.USER_SETTINGS", False)  # launch settings
        self.api.d(text="Users").wait.exists(timeout=5000)
        if not self.api.check_ui_exists("text", "Users"):
            self.api.settings_sub_launch("Users")
        self.api.click_with_timeout("text", "Guest")
        time.sleep(90)  # try to sleep 90 seconds since usb device will disconnect when switching user
        if self.api.check_ui_exists("textContains", "Welcome back", 5):
            self.api.click_with_timeout("text", "Yes, continue")
            self.api.d.sleep()
        for _ in range(10):
            self.api.d.wakeup()
            if self.api.check_ui_exists("description", "Apps"):
                break
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                self.api.unlock_screen()
                break
            time.sleep(5)
        self.api.launch_app_by_intents("android.settings.USER_SETTINGS", False)  # launch settings
        self.api.d(text="Users").wait.exists(timeout=10000)
        if not self.api.check_ui_exists("text", "Users"):
            self.api.settings_sub_launch("Users")
        assert self.api.check_ui_exists("text", "Remove guest"), "[ERROR]: fail to switch to guest"
        self.api.click_with_timeout("text", "Remove guest", 5)
        self.api.click_with_timeout("text", "Remove", 5)
        time.sleep(90)  # try to sleep 90 seconds since usb device will disconnect when switching user
        for _ in range(10):
            self.api.d.wakeup()
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                self.api.unlock_screen()
                break
            time.sleep(5)
        assert self.api.check_ui_exists("text", "Guest"), "[ERROR]: fail to switch back"


class Settings(UIATestBase):
    """
    @summary: Test cases for settings
    """

    def setUp(self):
        super(Settings, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(Settings, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testWorkProfile_Change(self):
        self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS", False)
        if not self.api.check_ui_exists("text", "Security", 5):
            self.api.settings_sub_launch("Security")
        if not self.api.check_ui_exists("text", "Device administrators"):
            self.api.d(scrollable=True).scroll.vert.to(text="Device administrators")
        self.api.click_with_timeout("text", "Device administrators")
        assert self.api.check_ui_exists("text", "Work"), "[ERROR]: fail to detect Work profile"
        assert self.api.d(text="Work").down(text="Sample MDM").right(resourceId="com.android.settings:id/checkbox").checked, \
            "[ERROR]: Sample MDM for work profile doesn't checked in security"
        self.api.d(text="Work").down(text="Sample MDM").right(resourceId="com.android.settings:id/checkbox").click()
        assert self.api.check_ui_exists("textContains", "To stop Sample MDM from accessing your work"), \
            "[ERROR]: warning message for remove work profile doesn't pop"
        self.api.click_with_timeout("text", "OK")


class DeviceControl(UIATestBase):
    """
    @summary: Test cases for device control
    """

    def setUp(self):
        super(DeviceControl, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(DeviceControl, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testReboot_OS(self):
        """
        reboot device
        :return: None
        """
        g_common_obj2.system_reboot(90)
        for _ in range(20):
            self.api = ApiImpl()
            self.api.d.wakeup()
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                break
            time.sleep(5)
        g_common_obj.set_vertical_screen()
        self.api.unlock_screen()
        if self.api.check_ui_exists("textContains", "isn't responding", 5):
            self.api.click_with_timeout("text", "OK")
        assert self.api.check_ui_exists("description", "Apps", 5), "[ERROR]: fail to detect Apps launcher"

    def testWiFi_Relaunch(self):
        """
        on/off wifi
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.WIFI_SETTINGS", False)
        if not self.api.check_ui_exists("textMatches", "Wi.Fi", 5):
            self.api.settings_sub_launch("Wi.Fi")
        try:
            for _ in range(2):
                if self.api.check_ui_exists("text", "On"):
                    self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
                    self.api.d(text="Off").wait.exists(timeout=5000)
                    wifi_state = ""
                    for i in range(10):
                        wifi_state = os.popen(
                            "adb -s {0} shell dumpsys wifi".format(self.api.serial)).read().strip()
                        if wifi_state.find("Wi-Fi is disabled") != -1:
                            break
                        time.sleep(2)
                    assert wifi_state.find("Wi-Fi is disabled") != -1, "Fail to turn off wifi: {0}".format(wifi_state)
                else:
                    self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
                    self.api.d(text="On").wait.exists(timeout=5000)
                    wifi_state = ""
                    for i in range(10):
                        wifi_state = os.popen(
                            "adb -s {0} shell dumpsys wifi".format(self.api.serial)).read().strip()
                        if wifi_state.find("Wi-Fi is enabled") != -1:
                            break
                        time.sleep(2)
                    assert wifi_state.find("Wi-Fi is enabled") != -1, "Fail to turn on wifi {0}".format(wifi_state)
        finally:
            if self.api.check_ui_exists("text", "Off"):
                self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")

    def testBT_Relaunch(self):
        """
        on/off bt
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.BLUETOOTH_SETTINGS", False)
        self.api.d(text="Bluetooth").wait.exists(timeout=5000)
        if not self.api.check_ui_exists("text", "Bluetooth"):
            self.api.settings_sub_launch("Bluetooth")
        # for i in range(2):
        #     if self.api.check_ui_exists("text", "On"):
        #         self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
        #         self.api.d(text="Off").wait.exists(timeout=5000)
        #     else:
        #         self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
        #         self.api.d(text="On").wait.exists(timeout=5000)
        #     time.sleep(5)
        # for _ in range(2):
        #     if self.api.check_ui_exists("text", "On"):
        #         self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
        #         self.api.d(text="Off").wait.exists(timeout=5000)
        #         bt_state = ""
        #         for i in range(10):
        #             bt_state = os.popen(
        #                 "adb -s {0} shell dumpsys bluetooth_manager".format(self.api.serial)).read().strip()
        #             if bt_state.find("enabled: false") != -1:
        #                 break
        #             time.sleep(2)
        #         assert bt_state.find("enabled: false") != -1, "Fail to turn off bleutooth: {0}".format(bt_state)
        #     else:
        #         self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
        #         self.api.d(text="On").wait.exists(timeout=5000)
        #         bt_state = ""
        #         for i in range(10):
        #             bt_state = os.popen(
        #                 "adb -s {0} shell dumpsys bluetooth_manager".format(self.api.serial)).read().strip()
        #             if bt_state.find("enabled: true") != -1:
        #                 break
        #             time.sleep(2)
        #         assert bt_state.find("enabled: true") != -1, "Fail to turn on bleutooth {0}".format(bt_state)
        for _ in range(2):
            if self.api.check_ui_exists("text", "On"):
                self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
                self.api.d(text="Off").wait.exists(timeout=5000)
                for i in range(3):
                    if self.api.check_ui_exists("text", "Off"):
                        break
                    time.sleep(5)
                assert self.api.check_ui_exists("text", "Off"), "fail to turn off BT"
            else:
                self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
                self.api.d(text="On").wait.exists(timeout=5000)
                for i in range(3):
                    if self.api.check_ui_exists("text", "On"):
                        break
                    time.sleep(5)
                assert self.api.check_ui_exists("text", "On"), "fail to turn on BT"
            time.sleep(5)

    def testVolume_Increase_Decrease(self):
        """
        verify vol +/- after setup managed profile
        :return: None
        """
        for i in range(3):
            time.sleep(1)
            self.api.d.press.volume_down()
        time.sleep(5)
        for i in range(3):
            time.sleep(1)
            self.api.d.press.volume_up()
        vol_first = self.api.get_notification_volume_by_dumpsys()
        for i in range(3):
            time.sleep(1)
            self.api.d.press.volume_down()
        vol_second = self.api.get_notification_volume_by_dumpsys()
        assert vol_first != vol_second, "[ERROR]: fail to decrease volume"
        for i in range(3):
            time.sleep(1)
            self.api.d.press.volume_up()
        vol_third = self.api.get_notification_volume_by_dumpsys()
        assert vol_second != vol_third, "[ERROR]: fail to increase volume"

    def testGPS_Relaunch(self):
        """
        verify that able to turn on/off GPS
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.LOCATION_SOURCE_SETTINGS", False)
        self.api.click_with_timeout("text", "Agree", 3)
        if not self.api.check_ui_exists("text", "Location"):
            self.api.settings_sub_launch("Location")
            self.api.click_with_timeout("text", "Agree", 3)
        for _ in range(2):
            if self.api.check_ui_exists("text", "On"):
                off_state = False
                for i in range(3):
                    if self.api.check_ui_exists("text", "On"):
                        self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
                    if self.api.check_ui_exists("text", "Off"):
                        off_state = True
                        break
                assert off_state, "fail to turn off GPS"
            else:
                on_state = False
                for j in range(3):
                    if self.api.check_ui_exists("text", "Off"):
                        self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
                        self.api.click_with_timeout("text", "Agree", 3)
                    if self.api.check_ui_exists("text", "On"):
                        on_state = True
                        break
                assert on_state, "fail to turn on GPS"
        if self.api.check_ui_exists("text", "Off"):
            self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
            self.api.click_with_timeout("text", "Agree", 3)

    def testNFC_Relaunch(self):
        """
        verify relaunching NFC after profile owner provisioning
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.SETTINGS", True)
        if not self.api.check_ui_exists("text", "More"):
            self.api.d(scrollable=True).scroll.vert.to(text="More")
        self.api.click_with_timeout("text", "More")
        assert self.api.check_ui_exists("text", "Android Beam"), "not support NFC"
        self.api.click_with_timeout("text", "Android Beam")
        for _ in range(2):
            if self.api.check_ui_exists("text", "On"):
                self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_widget")
                assert self.api.check_ui_exists("text", "Off"), "fail to turn off Android Beam"
            else:
                self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_widget")
                assert self.api.check_ui_exists("text", "On"), "fail to turn on Android Beam"
            time.sleep(2)
