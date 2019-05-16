# -*- coding: utf-8 -*-

from testlib.AfW.api_impl import ApiImpl
from testlib.util.uiatestbase import UIATestBase
import time
import os


class AppData(UIATestBase):
    """
    @summary: check app data
    """

    def setUp(self):
        super(AppData, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(AppData, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testCrossProfileWidgetInfo(self):
        """
        verify that widgets for primary user and work profile are both exists
        :return: None
        """
        try:
            self.api.add_x_profile_widget_provider(True)
            if self.api.is_android_L_build():
                self.api.d(text="Analog clock").wait.exists(timeout=5000)
                assert self.api.check_ui_exists("text", "Analog clock"), "fail to detect Analog clock widget"
                assert self.api.d(text="Analog clock").right(text="Analog clock").exists, \
                    "fail to detect two clock widget"
            else:
                work_widget = False
                self.api.d(text="Work Clock").wait.exists(timeout=5000)
                for _ in range(3):
                    if self.api.check_ui_exists(
                            "description", "Work Calendar") or self.api.check_ui_exists("description", "Work Clock"):
                        work_widget = True
                        break
                assert work_widget, "fail to detect work calendar/clock widget"
        finally:
            self.api.clean_tasks()
            self.api.remove_x_profile_widget_provider(True)
            self.api.d.press.home()

    def testCrossProfileAppInfo(self):
        """
        verify that app info between primary user and work profile are different
        :return: None
        """
        if not self.api.launch_app_by_activity(self.api.ui.chrome, True):
            self.api.launch_app("Work Chrome")
        # for i in range(5):
        #     if not self.api.check_ui_exists("resourceId", "com.android.packageinstaller:id/permission_allow_button"):
        #         break
        #     self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/permission_allow_button")
        # time.sleep(5)
        # self.api.click_with_timeout("text", "NEXT")
        self.api.d(packageName="com.android.chrome").wait.exists(timeout=5000)
        for _ in range(3):
            self.api.unlock_screen()
            if self.api.check_ui_exists("text", "Welcome to Chrome"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/terms_accept")
                self.api.click_with_timeout("text", "Next")
                self.api.click_with_timeout("text", "No thanks")
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/got_it_button")
                self.api.d.wait.update(timeout=3000)
                break
        self.api.launch_app_by_intents(
            "android.settings.APPLICATION_DETAILS_SETTINGS -d package:com.android.chrome", False)
        self.api.d(packageName="com.android.settings").wait.exists(timeout=5000)
        # if not self.api.check_ui_exists("text", "App info", 5):
        #     # self.api.launch_app_by_intents("android.settings.APPLICATION_SETTINGS", False)
        #     self.api.settings_sub_launch("Apps")
        #     if self.api.is_android_L_build():
        #         self.api.click_with_timeout("text", "Running")
        #         self.api.click_with_timeout("text", "All")
        #     else:
        #         if self.api.check_ui_exists("text", "All apps"):
        #             self.api.click_with_timeout("text", "All apps")
        #             self.api.click_with_timeout("text", "Personal")
        #         elif self.api.check_ui_exists("text", "Work"):
        #             self.api.click_with_timeout("text", "Work")
        #             self.api.click_with_timeout("text", "Personal")
        #     if not self.api.check_ui_exists("text", "Camera", 5):
        #         self.api.d(scrollable=True).scroll.vert.to(text="Camera")
        #     self.api.click_with_timeout("text", "Camera")
        # time.sleep(5)
        if self.api.is_android_L_build():
            self.api.click_with_timeout("text", "Clear data")
            self.api.click_with_timeout("text", "OK")
            data_size = self.api.d(resourceId="com.android.settings:id/data_size_text").text
        else:
            self.api.click_with_timeout("text", "Storage")
            assert self.api.check_ui_exists("text", "Data"), "fail to detect Cache"
            self.api.click_with_timeout("text", "Clear data")
            self.api.click_with_timeout("text", "OK")
            data_size = self.api.d(text="Data").right(resourceId="android:id/summary").text

        self.api.launch_app_by_intents(
            "android.settings.APPLICATION_DETAILS_SETTINGS -d package:com.android.chrome", True)
        self.api.d(packageName="com.android.settings").wait.exists(timeout=5000)
        # if not self.api.check_ui_exists("text", "App info", 5):
        #     # self.api.launch_app_by_intents("android.settings.APPLICATION_SETTINGS", True)
        #     self.api.settings_sub_launch("Apps")
        #     if self.api.is_android_L_build():
        #         self.api.click_with_timeout("text", "Running")
        #         self.api.click_with_timeout("text", "All")
        #     else:
        #         if self.api.check_ui_exists("text", "All apps"):
        #             self.api.click_with_timeout("text", "All apps")
        #             self.api.click_with_timeout("text", "Work")
        #         elif self.api.check_ui_exists("text", "Personal"):
        #             self.api.click_with_timeout("text", "Personal")
        #             self.api.click_with_timeout("text", "Work")
        #     if not self.api.check_ui_exists("text", "Camera", 5):
        #         self.api.d(scrollable=True).scroll.vert.to(text="Camera")
        #     self.api.click_with_timeout("text", "Camera")
        # time.sleep(5)
        if self.api.is_android_L_build():
            data_size_work = self.api.d(resourceId="com.android.settings:id/data_size_text").text
        else:
            self.api.click_with_timeout("text", "Storage")
            assert self.api.check_ui_exists("text", "Data"), "fail to detect Cache under work profile"
            data_size_work = self.api.d(text="Data").right(resourceId="android:id/summary").text
        assert data_size.encode('ascii', 'ignore') != data_size_work.encode('ascii', 'ignore'), \
            "the cache size are same between primary user and work profile"

    def testCrossProfileIsolation(self):
        """
        verify cross profile isolation when work profile is enabled
        :return: None
        """
        if not self.api.locate_apps("Work Clock"):
            self.api.enable_system_applications(True)
        set_alarm = "android.intent.action.SET_ALARM " \
                    "--ei android.intent.extra.alarm.HOUR {1} " \
                    "--ei android.intent.extra.alarm.MINUTES {2}"
        show_alarm = "android.intent.action.SHOW_ALARMS"
        self.api.launch_app_by_intents(show_alarm, True)
        if self.api.check_ui_exists("text", "Complete action using"):
            self.api.click_with_timeout("text", "Clock")
            self.api.click_with_timeout("resourceId", "android:id/button_always")
        for _ in range(3):
            if not self.api.check_ui_exists("text", u'11:11\u200aAM'):
                break
            self.api.click_with_timeout("text", u'11:11\u200aAM')
            self.api.click_with_timeout("text", "Cancel")
            self.api.click_with_timeout("resourceId", "com.android.deskclock:id/delete")
        self.api.launch_app_by_intents(set_alarm.format(self.api.serial, '11', '11'), True)
        if self.api.check_ui_exists("text", "Complete action using"):
            self.api.click_with_timeout("text", "Clock")
            self.api.click_with_timeout("resourceId", "android:id/button_always")
        assert self.api.check_ui_exists("text", u'11:11\u200aAM'), "fail to create new alarm for work profile"
        self.api.launch_app_by_intents(show_alarm, False)
        if self.api.check_ui_exists("text", "Complete action using"):
            self.api.click_with_timeout("text", "Clock")
            self.api.click_with_timeout("resourceId", "android:id/button_always")
        assert not self.api.check_ui_exists("text", u'11:11\u200aAM'), "detect the alarm which set in work profile"
        self.api.clean_tasks()
        self.api.launch_app_by_intents(show_alarm, False)
        for _ in range(3):
            if not self.api.check_ui_exists("text", u'10:10\u200aAM'):
                break
            self.api.click_with_timeout("text", u'10:10\u200aAM')
            self.api.click_with_timeout("text", "Cancel")
            self.api.click_with_timeout("resourceId", "com.android.deskclock:id/delete")
        self.api.launch_app_by_intents(set_alarm.format(self.api.serial, '10', '10'), False)
        if self.api.check_ui_exists("text", "Complete action using"):
            self.api.click_with_timeout("text", "Clock")
            self.api.click_with_timeout("resourceId", "android:id/button_always")
        assert self.api.check_ui_exists("text", u'10:10\u200aAM'), "fail to create new alarm"
        self.api.launch_app_by_intents(show_alarm, True)
        assert not self.api.check_ui_exists("text", u'10:10\u200aAM'), "detect the alarm which set in primary"


class AppBehavior(UIATestBase):
    """
    @summary: check app behavior under work profile
    """

    def setUp(self):
        super(AppBehavior, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(AppBehavior, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testRecentTasks_WorkProfile(self):
        """
        verify that apps under work profile will exist in recent tasks after launched
        :return: None
        """
        self.api.launch_app("Work Clock")
        assert self.api.check_ui_exists("resourceId", "com.android.deskclock:id/desk_clock_pager", 10), \
            "fail to launch work clock"
        self.api.launch_app("Work Camera")
        for i in range(5):
            if not self.api.check_ui_exists("resourceId", "com.android.packageinstaller:id/permission_allow_button"):
                break
            self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/permission_allow_button")
        self.api.click_with_timeout("text", "NEXT", 5)
        # assert self.api.check_ui_exists("resourceId", "com.android.camera2:id/shutter_button", 10), \
            # "fail to launch work camera"
        self.api.d.press.home()
        time.sleep(2)
        self.api.d.press.recent()
        assert self.api.check_ui_exists("resourceId", "com.android.systemui:id/dismiss_task", 10), \
            "fail to activate recent tasks"
        if self.api.is_android_L_build():
            assert self.api.check_ui_exists("description", "Clock"), "fail to detect clock in recent tasks"
            assert self.api.check_ui_exists("description", "Camera"), "fail to detect camera in recent tasks"
        else:
            assert self.api.check_ui_exists("description", "Work Clock"), "fail to detect work clock in recent tasks"
            assert self.api.check_ui_exists("description", "Work Camera"), "fail to detect work camera in recent tasks"

    def testLaunchChrome_WorkProfile(self):
        if not self.api.launch_app_by_activity(self.api.ui.chrome, True):
            self.api.launch_app("Work Chrome")
        for _ in range(5):
            if self.api.check_ui_exists("text", "Welcome to Chrome"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/terms_accept")
                self.api.click_with_timeout("text", "Next")
                self.api.click_with_timeout("text", "No thanks")
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/got_it_button")
                self.api.d.wait.update(timeout=3000)
                break
        assert self.api.check_ui_exists("packageName", "com.android.chrome", 10), "fail to detect google chrome"


class GMSBehavior(UIATestBase):
    """
    @summary: check app behavior under work profile
    """

    def setUp(self):
        super(GMSBehavior, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(GMSBehavior, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testGMS_Account_WorkProfile_Add(self):
        """
        verify that able to add google account in profile owner
        :return: None
        """
        assert os.path.isfile("/etc/oat/sys.conf"), "/etc/oat/sys.conf doesn't exists"
        account = self.config.read("/etc/oat/sys.conf", "google_account").get("user_name")
        password = self.config.read("/etc/oat/sys.conf", "google_account").get("password")
        self.api.launch_app_by_intents("android.settings.SYNC_SETTINGS", False)
        self.api.d(packageName="com.android.settings").wait.exists(timeout=5000)
        if not self.api.check_ui_exists("text", "Accounts"):
            self.api.settings_launch()
            if not self.api.check_ui_exists("text", "Accounts") and self.api.d(scrollable=True).exists:
                self.api.d(scrollable=True).scroll.vert.to(textContains="Accounts")
            self.api.click_with_timeout("text", "Accounts")
        for _ in range(5):
            if not self.api.check_ui_exists("text", "Google"):
                break
            self.api.click_with_timeout("text", "Google")
            time.sleep(2)
            self.api.d.press.menu()
            self.api.click_with_timeout("text", "Remove account")
            self.api.click_with_timeout("text", "Remove account")
            time.sleep(5)
        self.api.clean_tasks()
        self.api.add_google_account(account, password, True)
        self.api.d.press.home()
        self.api.clean_tasks()
        self.api.settings_sub_launch("Accounts")
        assert self.api.check_ui_exists("text", "Google"), "fail to detect google account"

    def testGMS_Account_WorkProfile_Remove(self):
        """
        verify that able to remove google account in profile owner
        :return: None
        """
        account = self.config.read("/etc/oat/sys.conf", "google_account").get("user_name")
        password = self.config.read("/etc/oat/sys.conf", "google_account").get("password")
        self.api.settings_sub_launch("Accounts")
        if not self.api.check_ui_exists("text", "Google"):
            self.api.clean_tasks()
            self.api.add_google_account(account, password, True)
            self.api.clean_tasks()
            self.api.settings_sub_launch("Accounts")
        self.api.click_with_timeout("text", "Google")
        time.sleep(5)
        self.api.d.press.menu()
        self.api.click_with_timeout("text", "Remove account")
        self.api.click_with_timeout("text", "Remove account")
        time.sleep(5)
        assert not self.api.check_ui_exists("text", "Google"), "fail to remove google account"
