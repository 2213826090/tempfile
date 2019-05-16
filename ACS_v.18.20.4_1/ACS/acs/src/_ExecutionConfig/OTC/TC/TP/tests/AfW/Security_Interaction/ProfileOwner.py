# -*- coding: utf-8 -*-

from testlib.AfW.api_impl import ApiImpl
from testlib.util.uiatestbase import UIATestBase
import time
import os


class GMSDataRemove(UIATestBase):
    """
    @summary: Test cases for apps control with GMS app in work profile
    """

    def setUp(self):
        super(GMSDataRemove, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(GMSDataRemove, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testGMSApp_WorkProfile_Clear_Cache(self):
        # if not self.api.launch_app_by_activity(self.api.ui.camera, True):  # launch camera to generate cache
        #     self.api.launch_app("Work Camera")
        # for i in range(5):
        #     if not self.api.check_ui_exists("resourceId", "com.android.packageinstaller:id/permission_allow_button"):
        #         break
        #     self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/permission_allow_button")
        # time.sleep(5)
        # self.api.click_with_timeout("text", "NEXT")
        # # launch app info for Google Camera
        # self.api.launch_app_by_intents(
        #     "android.settings.APPLICATION_DETAILS_SETTINGS -d package:com.google.android.GoogleCamera", True)
        # # launch app info from Apps Settings if fail to launch it by intent directly
        # if not self.api.check_ui_exists("text", "App info", 5):
        #     self.api.launch_app_by_intents("android.settings.APPLICATION_SETTINGS", True)
        #     if self.api.is_android_L_build():
        #         self.api.click_with_timeout("text", "Running")
        #         self.api.click_with_timeout("text", "All")
        #     else:
        #         if self.api.check_ui_exists("text", "All apps"):
        #             self.api.click_with_timeout("text", "All apps")
        #             self.api.click_with_timeout("text", "Work")
        #     if not self.api.check_ui_exists("text", "Camera", 5):
        #         self.api.d(scrollable=True).scroll.vert.to(text="Camera")
        #     self.api.click_with_timeout("text", "Camera")
        # time.sleep(10)
        self.api.launch_app_by_activity(self.api.ui.chrome, True)
        for _ in range(5):
            if self.api.check_ui_exists("text", "Welcome to Chrome"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/terms_accept")
                self.api.click_with_timeout("text", "Next")
                self.api.click_with_timeout("text", "No thanks")
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/got_it_button")
                self.api.d.wait.update(timeout=3000)
                break
        self.api.launch_app_by_intents(
            "android.settings.APPLICATION_DETAILS_SETTINGS -d package:com.android.chrome", True)
        self.api.d.wait.update(timeout=5000)
        if self.api.is_android_L_build():
            assert self.api.check_ui_exists("text", "Clear cache", 5), \
                "[ERROR]: fail to launch apps control for Google Chrome"
            cache_size = self.api.d(resourceId="com.android.settings:id/cache_size_text").text
            assert cache_size.encode('ascii', 'ignore') != "12.00KB", \
                "[ERROR]: cache size is equal to default size 12.00KB"
            self.api.click_with_timeout("text", "Clear cache")
            cache_size = self.api.d(resourceId="com.android.settings:id/cache_size_text").text
            assert cache_size.encode('ascii', 'ignore') == "12.00KB", \
                "[ERROR]: cache size is not equal to 12.00KB after clear"
        else:
            assert self.api.check_ui_exists("text", "Storage", 5), "fail to detect storage"
            self.api.click_with_timeout("text", "Storage")
            cache_size = self.api.d(text="Cache").right(resourceId="android:id/summary").text
            assert cache_size.encode('ascii', 'ignore') != "12.00 KB", \
                "[ERROR]: cache size is equal to default size 12.00 KB"
            self.api.click_with_timeout("text", "Clear cache")
            cache_size = self.api.d(text="Cache").right(resourceId="android:id/summary").text
            assert cache_size.encode('ascii', 'ignore') == "12.00 KB", \
                "[ERROR]: cache size is not equal to 12.00 KB after clear"

    def testGMSApp_WorkProfile_Clear_Data(self):
        # if not self.api.launch_app_by_activity(self.api.ui.camera, True):
        #     self.api.launch_app("Work Camera")
        # for i in range(5):
        #     if not self.api.check_ui_exists("resourceId", "com.android.packageinstaller:id/permission_allow_button"):
        #         break
        #     self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/permission_allow_button")
        # time.sleep(5)
        # self.api.click_with_timeout("text", "NEXT")
        # self.api.launch_app_by_intents(
        #     "android.settings.APPLICATION_DETAILS_SETTINGS -d package:com.google.android.GoogleCamera", True)
        # if not self.api.check_ui_exists("text", "App info", 5):
        #     self.api.launch_app_by_intents("android.settings.APPLICATION_SETTINGS", True)
        #     if self.api.is_android_L_build():
        #         self.api.click_with_timeout("text", "Running")
        #         self.api.click_with_timeout("text", "All")
        #     else:
        #         if self.api.check_ui_exists("text", "All apps"):
        #             self.api.click_with_timeout("text", "All apps")
        #             self.api.click_with_timeout("text", "Work")
        #     if not self.api.check_ui_exists("text", "Camera", 5):
        #         self.api.d(scrollable=True).scroll.vert.to(text="Camera")
        #     self.api.click_with_timeout("text", "Camera")
        # time.sleep(10)
        self.api.launch_app_by_activity(self.api.ui.chrome, True)
        for _ in range(5):
            if self.api.check_ui_exists("text", "Welcome to Chrome"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/terms_accept")
                self.api.click_with_timeout("text", "Next")
                self.api.click_with_timeout("text", "No thanks")
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/got_it_button")
                self.api.d.wait.update(timeout=3000)
                break
        self.api.launch_app_by_intents(
            "android.settings.APPLICATION_DETAILS_SETTINGS -d package:com.android.chrome", True)
        self.api.d.wait.update(timeout=5000)
        if self.api.is_android_L_build():
            assert self.api.check_ui_exists("text", "Clear data", 5), \
                "[ERROR]: fail to launch apps control for Google Chrome"
            cache_size = self.api.d(resourceId="com.android.settings:id/cache_size_text").text
            data_size = self.api.d(resourceId="com.android.settings:id/data_size_text").text
            assert cache_size.encode('ascii', 'ignore') != "0.00B", "[ERROR]: cache size is 0.00B before clear data"
            assert data_size.encode('ascii', 'ignore') != "0.00B", "[ERROR]: data size is 0.00B before clear data"
            self.api.click_with_timeout("text", "Clear data")
            self.api.click_with_timeout("text", "OK")
            cache_size = self.api.d(resourceId="com.android.settings:id/cache_size_text").text
            data_size = self.api.d(resourceId="com.android.settings:id/data_size_text").text
            assert cache_size.encode('ascii', 'ignore') == "0.00B", "[ERROR]: cache size is not 0.00B after clear data"
            assert data_size.encode('ascii', 'ignore') == "0.00B", "[ERROR]: data size is not 0.00B after clear data"
        else:
            assert self.api.check_ui_exists("text", "Storage", 5), "fail to detect storage"
            self.api.click_with_timeout("text", "Storage")
            cache_size = self.api.d(text="Cache").right(resourceId="android:id/summary").text
            data_size = self.api.d(text="Data").right(resourceId="android:id/summary").text
            assert cache_size.encode('ascii', 'ignore') != "0.00 B", "[ERROR]: cache size is 0.00 B before clear data"
            assert data_size.encode('ascii', 'ignore') != "0.00 B", "[ERROR]: data size is 0.00 B before clear data"
            self.api.click_with_timeout("text", "Clear data")
            self.api.click_with_timeout("text", "OK")
            cache_size = self.api.d(text="Cache").right(resourceId="android:id/summary").text
            data_size = self.api.d(text="Data").right(resourceId="android:id/summary").text
            assert cache_size.encode('ascii', 'ignore') == "0.00 B", \
                "[ERROR]: cache size is not 0.00 B after clear data"
            assert data_size.encode('ascii', 'ignore') == "0.00 B", \
                "[ERROR]: data size is not 0.00 B after clear data"


class ScreenLockChange(UIATestBase):
    """
    @summary: Test cases for apps control with GMS app in work profile
    """

    def setUp(self):
        super(ScreenLockChange, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(ScreenLockChange, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testUpdateSwipeLock(self):
        self.api.set_lock_swipe()
        self.api.d.press.home()
        self.api.d.sleep()
        time.sleep(2)
        for _ in range(5):
            self.api.d.wakeup()
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    self.api.d(resourceId=self.api.ui.lock_screen_scroll_view).swipe.up()
            if self.api.check_ui_exists("description", "Apps"):
                break
        assert self.api.check_ui_exists("description", "Apps"), "[ERROR]: fail to detect apps launcher "
        assert not self.api.check_ui_exists("resourceId", self.api.ui.lock_pin_pad), \
            "[ERROR]: lock pin pad was detected"

    def testUpdatePINLock(self):
        self.api.set_lock_pin()
        self.api.d.press.home()
        self.api.d.sleep()
        time.sleep(2)
        for _ in range(5):
            self.api.d.wakeup()
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    self.api.d(resourceId=self.api.ui.lock_screen_scroll_view).swipe.up()
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_pin_pad):
                break
        assert not self.api.check_ui_exists("description", "Apps"), "[ERROR]: apps launcher is found"
        assert self.api.check_ui_exists("resourceId", self.api.ui.lock_pin_pad), \
            "[ERROR]: fail to change screen lock to pin code"


class IntentForwarding(UIATestBase):
    """
    @summary: Test cases for apps control with GMS app in work profile
    """

    def setUp(self):
        super(IntentForwarding, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(IntentForwarding, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testLaunchPhotosAlbum_Work(self):
        self.api.launch_app_by_intents("android.intent.action.GET_CONTENT -d 'file:///null.jpg' -t 'image/*'", True)
        assert self.api.check_ui_exists("text", "Recent"), "fail to detect recent item"
        if not self.api.check_ui_exists("text", "Photos"):
            x = self.api.d.info['displayWidth']
            y = self.api.d.info['displayHeight']
            self.api.d.swipe(0, y/2, x/2, y/2, steps=10)
        assert self.api.check_ui_exists("text", "Photos"), "fail to detect photos item"
        assert self.api.check_ui_exists("textContains", "Personal"), "fail to detect personal apps"


class Cert(UIATestBase):
    """
    @summary: Test cases for apps control with GMS app in work profile
    """

    def setUp(self):
        super(Cert, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(Cert, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testAddCert(self):
        """
        verify adding certs in work profile
        :return: None
        """
        self.api.api_demo_po_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.vpn_management)
        self.api.d(resourceId=self.api.ui.install_ca_cert).wait.exists(timeout=5000)
        self.api.click_with_timeout("resourceId", self.api.ui.install_ca_cert)
        self.api.d(text="mytestcert5.cer").wait.exists(timeout=5000)
        self.api.click_with_timeout("text", "Select All")
        self.api.click_with_timeout("text", "Install CA Cert")
        self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS", False)
        self.api.d(packageName="com.android.settings").wait.exists(timeout=5000)
        if not self.api.check_ui_exists("text", "Security"):
            self.api.settings_sub_launch("Security")
        if not self.api.check_ui_exists("text", "Trusted credentials"):
            self.api.d(scrollable=True).scroll.vert.to(text="Trusted credentials")
        self.api.click_with_timeout("text", "Trusted credentials")
        self.api.d(text="User").wait.exists(timeout=5000)
        self.api.click_with_timeout("text", "User")
        self.api.d(text="Work").wait.exists(timeout=5000)
        self.api.click_with_timeout("text", "Work")
        assert self.api.check_ui_exists("text", "intel5"), "fail to detect installed cert in work profile"

    def testRemoveCert(self):
        """
        verify that able to remove installed cert file from settings
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS", False)
        self.api.d(packageName="com.android.settings").wait.exists(timeout=5000)
        if not self.api.check_ui_exists("text", "Security"):
            self.api.settings_sub_launch("Security")
        if not self.api.check_ui_exists("text", "Trusted credentials"):
            self.api.d(scrollable=True).scroll.vert.to(text="Trusted credentials")
        self.api.click_with_timeout("text", "Trusted credentials")
        self.api.d(text="User").wait.exists(timeout=5000)
        self.api.click_with_timeout("text", "User")
        self.api.d(text="Work").wait.exists(timeout=5000)
        self.api.click_with_timeout("text", "Work")
        if not self.api.check_ui_exists("text", "intel5"):
            self.api.click_with_timeout("text", "Work")
        self.api.click_with_timeout("text", "intel5")
        if not self.api.check_ui_exists("text", "Remove"):
            self.api.d(scrollable=True).scroll.vert.to(text="Remove")
        self.api.click_with_timeout("text", "Remove")
        self.api.click_with_timeout("text", "OK")
        try:
            assert not self.api.check_ui_exists("text", "intel5"), \
                "fail to remove installed cert under work profile"
        finally:
            self.api.api_demo_po_launch()
            self.api.click_with_timeout("resourceId", self.api.ui.vpn_management)
            self.api.d(resourceId=self.api.ui.uninstall_all_user_ca_cert).wait.exists(timeout=5000)
            self.api.click_with_timeout("resourceId", self.api.ui.uninstall_all_user_ca_cert)
            self.api.d(text="Yes").wait.exists(timeout=5000)
            self.api.click_with_timeout("text", "Yes")

    def testClearCredentials(self):
        """
        verify removing certs in work profile
        :return: None
        """
        cert_file = self.api.download_file_from_artifactory(self.api.remote.cert_file['sub_path'],
                                                            self.api.remote.cert_file['name'])
        assert cert_file is not None, "[ERROR]: fail to download CA Cert file"
        self.api.set_lock_pin()
        if self.api.is_android_L_build():
            os.popen("adb -s {0} push {1} /mnt/shell/emulated/0/Download/".format(self.api.serial, cert_file))
        else:
            os.popen("adb -s {0} push {1} /storage/emulated/0/Download/".format(self.api.serial, cert_file))
        self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS", False)
        self.api.d(packageName="com.android.settings").wait.exists(timeout=5000)
        if not self.api.check_ui_exists("text", "Security"):
            self.api.settings_sub_launch("Security")
        if not self.api.check_ui_exists("text", "Install from SD card"):
            self.api.d(scrollable=True).scroll.vert.to(text="Install from SD card")
        self.api.click_with_timeout("text", "Install from SD card")
        if not self.api.check_ui_exists("text", "Internal storage"):
            x = self.api.d.info['displayWidth']
            y = self.api.d.info['displayHeight']
            self.api.d.swipe(0, y/2, x/2, y/2, steps=10)
        self.api.click_with_timeout("text", "Internal storage")
        self.api.click_with_timeout("text", "Download")
        assert self.api.check_ui_exists("text", "key_alpha.cer"), "fail to detect cert file in Download"
        self.api.click_with_timeout("text", "key_alpha.cer")
        self.api.d(resourceId="com.android.certinstaller:id/credential_name").set_text("hello")
        self.api.click_with_timeout("text", "OK")
        if not self.api.check_ui_exists("text", "Clear credentials"):
            self.api.d(scrollable=True).scroll.vert.to(text="Clear credentials")
        try:
            assert self.api.d(text="Clear credentials").enabled, "there isn't any cert file exist"
            self.api.click_with_timeout("text", "Clear credentials")
            self.api.click_with_timeout("text", "OK")
            assert not self.api.d(text="Clear credentials").enabled, "fail to remove certs"
        finally:
            self.api.set_lock_swipe()
