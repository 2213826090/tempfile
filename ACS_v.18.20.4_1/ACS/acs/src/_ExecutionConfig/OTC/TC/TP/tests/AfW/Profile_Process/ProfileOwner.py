# -*- coding: utf-8 -*-

from testlib.AfW.api_impl import ApiImpl
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
import time
import os


class IterationControl(UIATestBase):
    """
    @summary: Profile Owner Provisioning
    """

    def setUp(self):
        super(IterationControl, self).setUp()
        self._test_name = __name__
        g_common_obj2.system_reboot(90)
        for _ in range(20):
            self.api = ApiImpl()
            self.api.d = g_common_obj.get_device()
            self.api.d.wakeup()
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                break
            time.sleep(5)
        self.api.unlock_screen()
        # self.api = ApiImpl()
        # self.api.unlock_screen()
        self.api.clean_tasks()
        if not self.api.locate_apps("Work Sample MDM"):
            self.api.setup_managed_profile(True)
            self.api.enable_system_applications(True)
            time.sleep(10)
            self.api.clean_tasks()
        if not self.api.locate_apps("Work Downloads"):
            self.api.unhide_applications(True)
            time.sleep(10)
            self.api.clean_tasks()
        if not self.api.locate_apps("Work Chrome"):
            self.api.enable_system_applications(True)
            time.sleep(10)
            self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(IterationControl, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testAddAllSystemApp_Hide_20_times(self):
        """
        hide and un-hide system apps for 20 times
        :return: None
        """
        for _ in range(20):
            self.api.hide_applications(True)
            self.api.d.press.home()
            self.api.d(description="Apps").click.wait()
            self.api.d.wait.update(timeout=10000)
            assert not self.api.locate_apps("Work Downloads"), "Still able to detect Work Downloads at {0}".format(_)
            self.api.clean_tasks()
            self.api.unhide_applications(True)
            self.api.d.press.home()
            self.api.d(description="Apps").click.wait()
            self.api.d.wait.update(timeout=15000)
            self.api.click_with_timeout("text", "OK")
            assert self.api.locate_apps("Work Downloads", 5), "Fail to detect Work Downloads at {0}".format(_)
            self.api.clean_tasks()

    def testCamera_WorkProfile_Launch_Close_50times(self):
        """
        launch and close google camera under managed profile
        :return: None
        """
        # self.api.launch_app("Work Camera")
        self.api.launch_app_by_activity("com.android.camera2/com.android.camera.CameraLauncher", True)
        self.api.d(packageName="com.android.camera2").wait.exists(timeout=5000)
        for i in range(5):
            if not self.api.check_ui_exists("resourceId", "com.android.packageinstaller:id/permission_allow_button"):
                break
            self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/permission_allow_button")
        self.api.click_with_timeout("text", "NEXT")
        if self.api.check_ui_exists("packageName", "com.google.android.GoogleCamera"):
            assert self.api.check_ui_exists("packageName", "com.google.android.GoogleCamera"), \
                "fail to launch work camera at first time"
        else:
            assert self.api.check_ui_exists("packageName", "com.android.camera2"), \
                "fail to launch work camera2 at first time"
        self.api.clean_tasks()
        for _ in range(50):
            # self.api.launch_app("Work Camera")
            self.api.launch_app_by_activity("com.android.camera2/com.android.camera.CameraLauncher", True)
            self.api.d(packageName="com.android.camera2").wait.exists(timeout=5000)
            assert not self.api.check_ui_exists("text", "Camera error"), "Detect camera error at {0}".format(_)
            if self.api.check_ui_exists("packageName", "com.google.android.GoogleCamera"):
                assert self.api.check_ui_exists("packageName", "com.google.android.GoogleCamera"), \
                    "fail to launch work camera at {0}".format(_)
            else:
                assert self.api.check_ui_exists("packageName", "com.android.camera2"), \
                    "fail to launch work camera2 at {0}".format(_)
            self.api.clean_tasks()

    def testRemoveRecentTasks_WorkProfile_50_times(self):
        """
        remove recent tasks for 50 times under managed profile
        :return: None
        """
        for _ in range(50):
            self.api.api_demo_po_launch()
            self.api.clean_tasks()
            self.api.d.press.recent()
            time.sleep(5)
            assert not self.api.check_ui_exists("resourceId", self.api.ui.dismiss_task), \
                "Detect dismiss task at {0}".format(_)
            self.api.d.press.home()

    def testChrome_WorkProfile_10MSNTabs_Close_50times(self):
        """
        verify iteration control by close 10 tabs in badged chrome app for 50 times
        :return: None
        """
        self.api.launch_app_by_intents(
            "android.intent.action.VIEW -c android.intent.category.BROWSABLE -d http://www.bing.com", True)
        for i in range(5):
            if self.api.check_ui_exists("text", "Welcome to Chrome"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/terms_accept")
                self.api.click_with_timeout("text", "Next")
                self.api.click_with_timeout("text", "No thanks")
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/got_it_button")
                break
        self.api.clean_tasks()
        model = os.popen("adb -s {0} shell getprop ro.product.model".format(self.api.serial)).read().strip()
        if model.find("sltmrd") != -1 or model.find("s3gr") != -1:
            for i in range(50):
                for j in range(10):
                    self.api.launch_app_by_intents(
                        "android.intent.action.VIEW -c android.intent.category.BROWSABLE -d http://www.bing.com", True)
                    time.sleep(5)
                    self.api.d.press.home()
                self.api.clean_tasks()
                self.api.d.press("recent")
                assert not self.api.check_ui_exists("resourceId", self.api.ui.dismiss_task), \
                    "fail to close chrome at {0}".format(i)
                self.api.d.press.home()
        else:
            for _ in range(50):
                self.api.launch_app_by_intents(
                    "android.intent.action.VIEW -c android.intent.category.BROWSABLE -d http://www.bing.com", True)
                for j in range(9):
                    self.api.d.press.menu()
                    if self.api.check_ui_exists("text", "New tab", 5):
                        self.api.click_with_timeout("text", "New tab")
                    else:
                        self.api.click_with_timeout("text", "New incognito tab")
                    if self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar"):
                        self.api.d(resourceId="com.android.chrome:id/url_bar").set_text("www.bing.com")
                    else:
                        self.api.d(resourceId="com.android.chrome:id/search_box_text").set_text("www.bing.com")
                    self.api.d.press.enter()
                    time.sleep(5)
                self.api.clean_tasks()
                self.api.d.press("recent")
                assert not self.api.check_ui_exists("resourceId", self.api.ui.dismiss_task), \
                    "fail to close multi tabs chrome at {0}".format(_)
                self.api.d.press.home()

    def testCreateNotification_Remove_20_times(self):
        """
        verify iteration control by create/remove notifications for 20 times
        :return: None
        """
        self.api.api_demo_po_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.vpn_management)
        for _ in range(20):
            self.api.unlock_screen()
            self.api.click_with_timeout("resourceId", self.api.ui.install_ca_cert)
            self.api.click_with_timeout("text", "Select All", 5)
            self.api.click_with_timeout("resourceId", "android:id/button1", 5)
            self.api.d.sleep()
            time.sleep(3)
            network_notification = False
            for i in range(5):
                self.api.d.wakeup()
                if self.api.check_ui_exists("text", "Network may be monitored"):
                    network_notification = True
                    break
            assert network_notification, \
                "Fail to detect Network may be monitored in lock screen at {0}".format(_)
            self.api.unlock_screen()
            self.api.click_with_timeout("resourceId", self.api.ui.uninstall_all_user_ca_cert, 5)
            self.api.click_with_timeout("resourceId", "android:id/button1", 5)
            self.api.d.sleep()
            time.sleep(3)
            network_notification = True
            for i in range(3):
                self.api.d.wakeup()
                if not self.api.check_ui_exists("text", "Network may be monitored"):
                    network_notification = False
                    break
            assert not network_notification, \
                "Still detect Network may be monitored in lock screen at {0}".format(_)
            self.api.unlock_screen()

    def testAddCrossProfileWidget_Remove_50_times(self):
        """
        verify iteration control by add/remove badged widgets for 50 times
        :return: None
        """
        for _ in range(50):
            self.api.clean_tasks()
            self.api.add_x_profile_widget_provider(True)
            work_widget = False
            self.api.d(text="Work Clock").wait.exists(timeout=5000)
            for i in range(3):
                if self.api.check_ui_exists(
                        "description", "Work Calendar") or self.api.check_ui_exists("description", "Work Clock"):
                    work_widget = True
                    break
            assert work_widget, "fail to detect work calendar/clock widget at {0}".format(_)
            self.api.clean_tasks()
            self.api.remove_x_profile_widget_provider(True)
            work_widget = True
            for i in range(3):
                if not self.api.check_ui_exists(
                        "description", "Work Calendar") and not self.api.check_ui_exists("description", "Work Clock"):
                    work_widget = False
                    break
            assert not work_widget, "detect work calendar/clock widget at {0}".format(_)
