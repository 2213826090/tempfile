# -*- coding: utf-8 -*-

from testlib.AfW.api_impl import ApiImpl
from testlib.util.uiatestbase import UIATestBase
import time
import os


class IterationControl(UIATestBase):
    """
    @summary: Profile Owner Provisioning
    """

    def setUp(self):
        super(IterationControl, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(IterationControl, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testCreateNotification_Remove_20_times(self):
        """
        verify iteration control by create/remove notifications for 20 times
        :return: None
        """
        self.api.uninstall_apps_by_adb("com.android.trustagent.testsample")
        self.api.unknown_source_control(True)
        for _ in range(10):
            self.api.api_demo_launch()
            self.api.click_with_timeout("resourceId", self.api.ui.app_provision)
            assert self.api.check_ui_exists("text", "Install An App Silently"), \
                "fail to detect install app silently button at {0}".format(_)
            self.api.click_with_timeout("text", "Install An App Silently")
            self.api.click_with_timeout("text", "OK", 10)
            self.api.d.sleep()
            time.sleep(2)
            self.api.d.wakeup()
            assert self.api.check_ui_exists("text", "Sample Trust Agent"), \
                "fail to detect the install notification in lock screen at {0}".format(_)
            self.api.d(text="Sample Trust Agent").swipe.right()
            self.api.d.wakeup()
            assert not self.api.check_ui_exists("text", "Sample Trust Agent"), \
                "fail to remove the installed notification at {0}".format(_)
            self.api.unlock_screen()
            assert self.api.check_ui_exists("text", "Uninstall An App Silently"), \
                "fail to detect uninstall button at {0}".format(_)
            self.api.click_with_timeout("text", "Uninstall An App Silently")
            self.api.click_with_timeout("text", "OK", 10)
            self.api.d.sleep()
            time.sleep(2)
            self.api.d.wakeup()
            assert self.api.check_ui_exists("text", "Sample Trust Agent"), \
                "fail to detect the remove notification in lock screen at {0}".format(_)
            self.api.d(text="Sample Trust Agent").swipe.right()
            self.api.d.wakeup()
            assert not self.api.check_ui_exists("text", "Sample Trust Agent"), \
                "fail to clear the removed notification at {0}".format(_)
            self.api.unlock_screen()
            self.api.clean_tasks()

    def testLaunchBT_FightMode_WorkProfile_50_times(self):
        """
        On/off BT in fight mode
        :return: None
        """
        self.api.settings_sub_launch("More")
        if self.api.d(text="Airplane mode").right(resourceId="android:id/switchWidget").checked:
            self.api.d(text="Airplane mode").right(resourceId="android:id/switchWidget").click()
        self.api.launch_app_by_intents("android.settings.BLUETOOTH_SETTINGS", False)
        if not self.api.check_ui_exists("text", "Bluetooth"):
            self.api.clean_tasks()
            self.api.settings_sub_launch("Bluetooth")
        if self.api.check_ui_exists("text", "Off"):
            self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
            time.sleep(5)
        assert self.api.check_ui_exists("text", "On"), "fail to turn bleutooth on"
        self.api.clean_tasks()
        for _ in range(50):
            self.api.settings_sub_launch("More")
            assert self.api.check_ui_exists("text", "Airplane mode"), \
                "fail to detect airplane mode at {0}".format(_)
            if not self.api.d(text="Airplane mode").right(resourceId="android:id/switchWidget").checked:
                self.api.d(text="Airplane mode").right(resourceId="android:id/switchWidget").click()
                time.sleep(5)
                self.api.launch_app_by_intents("android.settings.BLUETOOTH_SETTINGS", False)
                if not self.api.check_ui_exists("text", "Bluetooth"):
                    self.api.clean_tasks()
                    self.api.settings_sub_launch("Bluetooth")
                assert self.api.check_ui_exists("text", "Bluetooth"), \
                    "fail to detect bluetooth after enable fight mode at {0}".format(_)
                assert self.api.check_ui_exists("text", "Off"), \
                    "BT wasn't off after enable fight mode at {0}".format(_)
            else:
                self.api.d(text="Airplane mode").right(resourceId="android:id/switchWidget").click()
                time.sleep(10)
                self.api.launch_app_by_intents("android.settings.BLUETOOTH_SETTINGS", False)
                if not self.api.check_ui_exists("text", "Bluetooth"):
                    self.api.clean_tasks()
                    self.api.settings_sub_launch("Bluetooth")
                assert self.api.check_ui_exists("text", "Bluetooth"), \
                    "fail to detect bluetooth after enable fight mode at {0}".format(_)
                assert self.api.check_ui_exists("text", "On"), \
                    "BT wasn't on after enable fight mode at {0}".format(_)
            self.api.clean_tasks()

    def testLaunchWiFi_FlightMode_WorkProfile_50_times(self):
        """
        On/off WiFi in fight mode
        :return: None
        """
        self.api.settings_sub_launch("More")
        if self.api.d(text="Airplane mode").right(resourceId="android:id/switchWidget").checked:
            self.api.d(text="Airplane mode").right(resourceId="android:id/switchWidget").click()
        self.api.launch_app_by_intents("android.settings.WIFI_SETTINGS", False)
        if not self.api.check_ui_exists("text", u'Wi\u2011Fi'):
            self.api.clean_tasks()
            self.api.settings_sub_launch(u'Wi\u2011Fi')
        if self.api.check_ui_exists("text", "Off"):
            self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
            time.sleep(5)
        assert self.api.check_ui_exists("text", "On"), "fail to turn wifi on"
        self.api.clean_tasks()
        for _ in range(50):
            self.api.settings_sub_launch("More")
            assert self.api.check_ui_exists("text", "Airplane mode"), \
                "fail to detect airplane mode at {0}".format(_)
            if not self.api.d(text="Airplane mode").right(resourceId="android:id/switchWidget").checked:
                self.api.d(text="Airplane mode").right(resourceId="android:id/switchWidget").click()
                time.sleep(5)
                self.api.launch_app_by_intents("android.settings.WIFI_SETTINGS", False)
                if not self.api.check_ui_exists("text", u'Wi\u2011Fi'):
                    self.api.clean_tasks()
                    self.api.settings_sub_launch(u'Wi\u2011Fi')
                assert self.api.check_ui_exists("text", u'Wi\u2011Fi'), \
                    "fail to detect Wi-Fi after enable fight mode at {0}".format(_)
                assert self.api.check_ui_exists("text", "Off"), \
                    "WiFi wasn't off after enable fight mode at {0}".format(_)
            else:
                self.api.d(text="Airplane mode").right(resourceId="android:id/switchWidget").click()
                time.sleep(10)
                self.api.launch_app_by_intents("android.settings.WIFI_SETTINGS", False)
                if not self.api.check_ui_exists("text", u'Wi\u2011Fi'):
                    self.api.clean_tasks()
                    self.api.settings_sub_launch(u'Wi\u2011Fi')
                assert self.api.check_ui_exists("text", u'Wi\u2011Fi'), \
                    "fail to detect Wi-Fi after enable fight mode at {0}".format(_)
                assert self.api.check_ui_exists("text", "On"), \
                    "WiFi wasn't on after enable fight mode at {0}".format(_)
            self.api.clean_tasks()
