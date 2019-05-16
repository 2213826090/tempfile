# -*- coding: utf-8 -*-

from testlib.AfW.api_impl import ApiImpl
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
from testlib.dut_init.dut_init_impl import Function
import time
import os


class ProvisionDevice(UIATestBase):
    """
    @summary: Test cases for provision device
    """

    def setUp(self):
        super(ProvisionDevice, self).setUp()
        self._test_name = __name__
        self.function = Function()
        self.api = ApiImpl()
        self.api.unlock_screen()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(ProvisionDevice, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testProvision_Device_with_Device_Owner(self):
        """
        provision device with device owner during OOBE
        :return: None
        """
        self.api.d.wakeup()
        self.function.push_uiautomator_jar()
        g_common_obj.set_vertical_screen()
        if not self.api.check_ui_exists("packageName", "com.google.android.setupwizard"):
            self.api.d.press.home()
            if self.api.check_ui_exists("description", "Apps"):
                self.api.clean_tasks()
                self.api.factory_reset()
                time.sleep(1200)
                self.api = ApiImpl()
                self.api.d.wakeup()
                g_common_obj.set_vertical_screen()
        else:
            while not self.api.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"):
                self.api.d.press.back()
        assert self.api.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"), \
            "[ERROR]: Please run test cases at the beginning of OOBE"
        # for unencrypted device, just bypass encryption when provision device
        # the encryption test cases will raise this problem
        if os.popen("adb -s {0} shell getprop ro.crypto.state".format(self.api.serial)).read().strip() == "unencrypted":
            os.popen("adb -s {0} root".format(self.api.serial)).read().strip()
            time.sleep(5)
            os.popen("adb -s {0} shell setprop persist.sys.no_req_encrypt true".format(self.api.serial)).read().strip()
            time.sleep(5)
        self.api.oobe_setup(True)
        self.api.unlock_screen()
        self.api.clean_tasks()
        self.api.d.press.home()
        time.sleep(5)
        self.api.click_with_timeout("text", "GOT IT")
        try:
            self.api.api_demo_launch()
            assert self.api.check_ui_exists("textContains", "Device Owner", 5), \
                "[ERROR]: fail to provision device as device owner"
            g_common_obj.set_vertical_screen()
        finally:
            self.api.keep_awake()

    def testWiFi_Connection_Pretest(self):
        """ Check WiFi connection in pretest

        :return: None

        """
        # read wifi info from /etc/oat/sys.conf by default if it exist
        assert os.path.isfile("/etc/oat/sys.conf"), "[ERROR]: Missing config file /etc/oat/sys.config in host"
        cfg_file = "/etc/oat/sys.conf"
        self.ssid = self.config.read(cfg_file, 'wifisetting').get("ssid")
        self.password = self.config.read(cfg_file, 'wifisetting').get("passwd")
        self.security = self.config.read(cfg_file, 'wifisetting').get("security")
        self.retry_wifi = self.config.read('tests.tablet.dut_init.conf', 'init_list').get("connect_wifi")
        # check ui element "Apps" to make sure we are at oobe stage
        self.api.d.press.home()
        if self.api.check_ui_exists("description", "Apps"):
            self.api.set_lock_swipe()
            self.api.clean_tasks()
            self.api.settings_sub_launch("Backup & reset")
            assert self.api.check_ui_exists("text", "Factory data reset", 5), "fail to detect factory data reset"
            self.api.click_with_timeout("text", "Factory data reset")
            self.api.click_with_timeout("resourceId", "com.android.settings:id/initiate_master_clear")
            assert self.api.check_ui_exists("resourceId", "com.android.settings:id/execute_master_clear", 5), \
                "fail to detect erase everything"
            self.api.click_with_timeout("resourceId", "com.android.settings:id/execute_master_clear")
            time.sleep(600)
            # reboot device
            g_common_obj2.system_reboot(90)
            for _ in range(30):
                self.api = ApiImpl()
                self.api.d = g_common_obj.get_device()
                self.api.d.wakeup()
                if self.api.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"):
                    break
                time.sleep(10)
            self.api.d.wakeup()
        g_common_obj.set_vertical_screen()
        # keep device awake
        # self.api.keep_awake()
        # enable unknown sources
        self.api.unknown_source_control(True)
        # will check for 10 times and will back to welcome page if it's not
        for _ in range(10):
            if self.api.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"):
                break
            # just back to welcome page
            self.api.d.press.back()
        # launch WiFi Settings by intent
        self.api.launch_app_by_intents("android.settings.WIFI_SETTINGS", False)
        assert self.api.check_ui_exists("textMatches", "Wi.Fi"), "[ERROR]: fail to launch wifi settings"
        # try to connect wifi
        for retry in range(int(self.retry_wifi)):
            # turn on wifi if it is off
            if self.api.check_ui_exists("text", "Off"):
                self.api.d(text="Off").right(text="OFF").click()
                time.sleep(10)
            if self.api.d(resourceId="android:id/list").scrollable:
                self.api.d(scrollable=True).scroll.vert.to(text=self.ssid)  # scroll to ssid
            if self.api.check_ui_exists("text", self.ssid):  # if wifi ap is not hidden
                if self.api.check_ui_exists("text", "Connected"):  # break if wifi is connected
                    break
                self.api.click_with_timeout("text", self.ssid)
                # input password if needed
                if self.api.check_ui_exists("resourceId", "com.android.settings:id/password"):
                    self.api.d(resourceId="com.android.settings:id/password").set_text(self.password)
                # click "Connect": a)first time to connect b)reconnect to wifi if failed before
                if self.api.check_ui_exists("text", "Connect"):
                    self.api.click_with_timeout("text", "Connect")
                # click "Done" if wifi is connecting
                if self.api.check_ui_exists("text", "Done"):
                    self.api.click_with_timeout("text", "Done")
            else:  # connect to hidden wifi ap
                self.api.d.press.menu()
                self.api.d(text="Add network").click.wait()
                self.api.d(text="Enter the SSID").set_text(self.ssid)
                self.api.d(resourceId="com.android.settings:id/security", index=1).click.wait()
                self.api.d(text=self.security).click.wait()
                self.api.d(resourceId="com.android.settings:id/password").set_text(self.password)
                self.api.d(text="Save").click.wait()
            time.sleep(10)
        # wifi should be connected now
        assert self.api.check_ui_exists("text", self.ssid), "[ERROR]: WiFi SSID does't exist"
        assert self.api.check_ui_exists("text", "Connected"), "[ERROR]: Fail to connect WiFi"
        # back to oobe
        self.api.d.press.back()

    def testPretest_for_reliability(self):
        """
        provision device if needed
        """
        # read wifi info from /etc/oat/sys.conf by default if it exist
        assert os.path.isfile("/etc/oat/sys.conf"), "[ERROR]: Missing config file /etc/oat/sys.config in host"
        cfg_file = "/etc/oat/sys.conf"
        self.ssid = self.config.read(cfg_file, 'wifisetting').get("ssid")
        self.password = self.config.read(cfg_file, 'wifisetting').get("passwd")
        self.security = self.config.read(cfg_file, 'wifisetting').get("security")
        self.retry_wifi = self.config.read('tests.tablet.dut_init.conf', 'init_list').get("connect_wifi")
        # check ui element "Apps" to make sure we are at oobe stage
        self.api.d.press.home()
        self.api.d.wait.update(timeout=5000)
        if self.api.check_ui_exists("description", "Apps"):
            # self.api.set_lock_swipe()
            # self.api.clean_tasks()
            # self.api.settings_sub_launch("Backup & reset")
            # assert self.api.check_ui_exists("text", "Factory data reset", 5), "fail to detect factory data reset"
            # self.api.click_with_timeout("text", "Factory data reset")
            # self.api.click_with_timeout("resourceId", "com.android.settings:id/initiate_master_clear")
            # assert self.api.check_ui_exists("resourceId", "com.android.settings:id/execute_master_clear", 5), \
            #     "fail to detect erase everything"
            # self.api.click_with_timeout("resourceId", "com.android.settings:id/execute_master_clear")
            self.api.factory_reset()
            time.sleep(600)
            # reboot device
            g_common_obj2.system_reboot(90)
            for _ in range(30):
                # self.api = ApiImpl()
                self.api.d = g_common_obj.get_device()
                self.api.d.wakeup()
                if self.api.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"):
                    break
                time.sleep(10)
            self.api.d.wakeup()
        g_common_obj.set_vertical_screen()
        # keep device awake
        # self.api.keep_awake()
        # enable unknown sources
        self.api.unknown_source_control(True)
        # will check for 10 times and will back to welcome page if it's not
        for _ in range(10):
            if self.api.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"):
                break
            # just back to welcome page
            self.api.d.press.back()
        # launch WiFi Settings by intent
        self.api.launch_app_by_intents("android.settings.WIFI_SETTINGS", False)
        assert self.api.check_ui_exists("textMatches", "Wi.Fi"), "[ERROR]: fail to launch wifi settings"
        # try to connect wifi
        for retry in range(int(self.retry_wifi)):
            # turn on wifi if it is off
            if self.api.check_ui_exists("text", "Off"):
                self.api.d(text="Off").right(text="OFF").click()
                time.sleep(10)
            if self.api.d(resourceId="android:id/list").scrollable:
                self.api.d(scrollable=True).scroll.vert.to(text=self.ssid)  # scroll to ssid
            if self.api.check_ui_exists("text", self.ssid):  # if wifi ap is not hidden
                if self.api.check_ui_exists("text", "Connected"):  # break if wifi is connected
                    break
                self.api.click_with_timeout("text", self.ssid)
                # input password if needed
                if self.api.check_ui_exists("resourceId", "com.android.settings:id/password"):
                    self.api.d(resourceId="com.android.settings:id/password").set_text(self.password)
                # click "Connect": a)first time to connect b)reconnect to wifi if failed before
                if self.api.check_ui_exists("text", "Connect"):
                    self.api.click_with_timeout("text", "Connect")
                # click "Done" if wifi is connecting
                if self.api.check_ui_exists("text", "Done"):
                    self.api.click_with_timeout("text", "Done")
            else:  # connect to hidden wifi ap
                self.api.d.press.menu()
                self.api.d(text="Add network").click.wait()
                self.api.d(text="Enter the SSID").set_text(self.ssid)
                self.api.d(resourceId="com.android.settings:id/security", index=1).click.wait()
                self.api.d(text=self.security).click.wait()
                self.api.d(resourceId="com.android.settings:id/password").set_text(self.password)
                self.api.d(text="Save").click.wait()
            time.sleep(10)
        # wifi should be connected now
        assert self.api.check_ui_exists("text", self.ssid), "[ERROR]: WiFi SSID does't exist"
        assert self.api.check_ui_exists("text", "Connected"), "[ERROR]: Fail to connect WiFi"
        # back to oobe
        self.api.d.wakeup()
        self.function.push_uiautomator_jar()
        g_common_obj.set_vertical_screen()
        while not self.api.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"):
            self.api.d.wakeup()
            self.api.d.press.back()
        assert self.api.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"), \
            "[ERROR]: Please run test cases at the beginning of OOBE"
        # for unencrypted device, just bypass encryption when provision device
        # the encryption test cases will raise this problem
        if os.popen("adb -s {0} shell getprop ro.crypto.state".format(self.api.serial)).read().strip() == "unencrypted":
            os.popen("adb -s {0} root".format(self.api.serial)).read().strip()
            time.sleep(5)
            os.popen("adb -s {0} shell setprop persist.sys.no_req_encrypt true".format(self.api.serial)).read().strip()
            time.sleep(5)
            self.api.d.wakeup()
        self.api.oobe_setup(True)
        self.api.unlock_screen()
        self.api.clean_tasks()
        self.api.d.press.home()
        time.sleep(5)
        self.api.click_with_timeout("text", "GOT IT")
        try:
            self.api.unlock_screen()
            self.api.launch_app("Sample MDM")
            assert self.api.check_ui_exists("textContains", "Device Owner", 5), \
                "[ERROR]: fail to provision device as device owner"
            g_common_obj.set_vertical_screen()
        finally:
            self.api.keep_awake()
        self.api.clean_tasks()
        self.api.enable_system_applications(False)


class OpenTheTrustAgentSample(UIATestBase):
    """
    @summary: Test cases for open the trust agent sample
    """

    def setUp(self):
        super(OpenTheTrustAgentSample, self).setUp()
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(OpenTheTrustAgentSample, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testInstall_Trust_Agent_Sample(self):
        """
        install trust agent sample
        :return: None
        """
        self.api.uninstall_apps_by_adb("package:com.android.trustagent.testsample")
        self.api.set_lock_pin()
        if not self.api.locate_apps("Work Sample MDM"):
            self.api.setup_managed_profile()
        self.api.launch_app("Sample MDM")
        assert self.api.check_ui_exists("text", self.api.ui.install_a_trust_agent), \
            "[ERROR]: Install A Trust Agent Sample doesn't exist"
        self.api.click_with_timeout("text", self.api.ui.install_a_trust_agent)
        self.api.click_with_timeout("text", "Install")
        self.api.click_with_timeout("text", "ACCEPT", 10)
        self.api.check_ui_exists("text", "Done", 30)
        self.api.click_with_timeout("text", "Done")
        for _ in range(5):
            if self.api.locate_apps("Sample Trust Agent"):
                break
            time.sleep(5)
        assert self.api.locate_apps("Sample Trust Agent"), "[ERROR]: fail to install Sample Trust Agent"
        self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS", False)
        if not self.api.check_ui_exists("text", "Trust agents"):
            self.api.d(scrollable=True).scroll.vert.to(text="Trust agents")
        self.api.click_with_timeout("text", "Trust agents")
        assert self.api.check_ui_exists("text", "Sample Trust Agent"), \
            "[ERROR]: Sample Trust Agent doesn't exist in settings"
        if not self.api.d(text="Sample Trust Agent").right(resourceId="android:id/switchWidget").checked:
            self.api.d(text="Sample Trust Agent").right(resourceId="android:id/switchWidget").click.wait()

    def testGrant_Trust_to_This_device(self):
        """
        grant trust agent to this device
        :return: None
        """
        self.api.d.sleep()
        self.api.d.wait.update(timeout=3000)
        self.api.d.wakeup()
        for _ in range(5):
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                self.api.d(resourceId=self.api.ui.lock_screen_scroll_view).swipe.up()
                if not self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    break
        assert self.api.check_ui_exists("resourceId", self.api.ui.lock_pin_pad), \
            "[ERROR]: fail to lock screen by pin code"
        self.api.d.sleep()
        self.api.unlock_screen()
        self.api.launch_app("Sample Trust Agent")
        self.api.click_with_timeout("resourceId", self.api.ui.grant_trust)
        self.api.d.sleep()
        self.api.d.wait.update(timeout=3000)
        self.api.d.wakeup()
        for _ in range(5):
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                self.api.d(resourceId=self.api.ui.lock_screen_scroll_view).swipe.up()
                if not self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    break
        assert not self.api.check_ui_exists("resourceId", self.api.ui.lock_pin_pad), "[ERROR]: unable to grant trust"
        self.api.clean_tasks()
        # sleep more than 10min
        self.api.d.sleep()
        time.sleep(800)
        self.api.d.wait.update(timeout=3000)
        os.popen("adb -s {0} shell input keyevent 82".format(self.api.serial)).read().strip()
        self.api.d.wait.update(timeout=3000)
        assert self.api.check_ui_exists("resourceId", "com.android.systemui:id/pinEntry"), \
            "fail to detect pinEntry after 10mins"
        self.api.d.sleep()
        self.api.unlock_screen()

    def testRevoke_Trust_from_This_Device(self):
        """
        revoke trust agent from this device
        :return: None
        """
        self.api.launch_app("Sample Trust Agent")
        self.api.click_with_timeout("resourceId", self.api.ui.grant_trust)
        self.api.d.sleep()
        self.api.d.wait.update(timeout=3000)
        self.api.d.wakeup()
        for _ in range(5):
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                self.api.d(resourceId=self.api.ui.lock_screen_scroll_view).swipe.up()
                if not self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    break
        assert not self.api.check_ui_exists("resourceId", self.api.ui.lock_pin_pad), "[ERROR]: unable to grant trust"
        self.api.d.sleep()
        self.api.unlock_screen()
        self.api.clean_tasks()
        self.api.launch_app("Sample Trust Agent")
        self.api.click_with_timeout("resourceId", self.api.ui.revoke_trust)
        self.api.d.sleep()
        self.api.d.wait.update(timeout=3000)
        self.api.d.wakeup()
        for _ in range(5):
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                self.api.d(resourceId=self.api.ui.lock_screen_scroll_view).swipe.up()
                if not self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    break
        assert self.api.check_ui_exists("resourceId", self.api.ui.lock_pin_pad), \
            "[ERROR]: fail to lock screen by pin code"
        self.api.d.sleep()
        self.api.unlock_screen()
        self.api.set_lock_swipe()
        self.api.uninstall_apps_by_adb("com.android.trustagent.testsample")


class SetupManagedProfile(UIATestBase):
    """
    @summary: Test cases for setup managed profile
    """

    def setUp(self):
        super(SetupManagedProfile, self).setUp()
        self.api = ApiImpl()
        self.function = Function()
        self.api.unlock_screen()
        self.api.clean_tasks()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(SetupManagedProfile, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSetup_Managed_Profile(self):
        """
        set up manged profile
        :return: None
        """
        self.api.settings_sub_launch("Accounts")
        if self.api.check_ui_exists("text", "Work", 5):
            self.api.click_with_timeout("text", "Remove work profile")
            self.api.click_with_timeout("text", "Delete", 10)
            time.sleep(5)
        self.api.remove_other_users_by_id()
        self.api.setup_managed_profile()
        for _ in range(5):
            self.api.d.press.home()
            self.api.api_demo_po_launch()
            if self.api.check_ui_exists("textContains", "Profile Owner"):
                break
            time.sleep(3)
        assert self.api.check_ui_exists("textContains", "Profile Owner"), "[ERROR]: fail to launch Work Sample MDM"

    def testDisable_Managed_Profile(self):
        """
        disable manage profile
        :return: None
        """
        self.api.remove_managed_profile()
        all_users = None
        for _ in range(5):
            all_users = repr(os.popen("adb -s {0} shell pm list users".format(self.api.serial)).read().strip())
            if all_users.find("Work profile") == -1 and all_users.find("HiProfileOwner") == -1:
                break
            time.sleep(3)
        assert all_users is not None, "[ERROR]: fail to get user info"
        assert all_users.find("Work profile") == -1, "[ERROR]: still able to find Work profile"
        assert all_users.find("HiProfileOwner") == -1, "[ERROR]: still able to find HiProfileOwner"

    def testAccount_Migrate(self):
        """
        migrate account
        :return: None
        """
        assert os.path.isfile("/etc/oat/sys.conf"), "[ERROR]: Missing config file /etc/oat/sys.config in host"
        cfg_file = "/etc/oat/sys.conf"
        account = self.config.read(cfg_file, "google_account").get("user_name")
        password = self.config.read(cfg_file, "google_account").get("password")
        # if self.api.locate_apps("Work Sample MDM"):
        if self.api.is_work_profile_enabled():
            self.api.remove_managed_profile()
        if self.api.is_android_L_build():
            self.function.add_google_account_mr1(account, password)
        else:
            self.api.add_google_account(account, password, False)
        self.api.clean_tasks()
        self.api.launch_app_by_intents("android.settings.SETTINGS", False)
        if not self.api.check_ui_exists("text", "Accounts"):
            self.api.d(scrollable=True).scroll.vert.to(text="Accounts")
        self.api.click_with_timeout("text", "Accounts")
        assert self.api.check_ui_exists("text", "Google"), "[ERROR]: fail to add google account"
        assert not self.api.check_ui_exists("text", "Work"), "[ERROR]: Work profile already created"
        self.api.api_demo_launch()
        if self.api.is_android_L_build():  # TODO: change the entry if sample mdm got update for L
            self.api.click_with_timeout("resourceId", self.api.ui.create_and_delete_profile)
        else:
            self.api.click_with_timeout("resourceId", self.api.ui.setup_managed_profile)
            if self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/content_edit", 10):
                self.api.d(resourceId="com.intel.afw.mdm:id/content_edit").set_text(account)
        self.api.click_with_timeout("text", "Yes")
        if self.api.is_android_L_build():
            assert self.api.check_ui_exists("textContains", account), "[ERROR]: fail to detect google account"
            self.api.click_with_timeout("textContains", account)
            self.api.click_with_timeout("text", "Create Managed Profile")
        self.api.click_with_timeout("text", "Set up")
        self.api.click_with_timeout("text", "OK")
        self.api.check_ui_exists("text", "Setup all done", 30)
        for _ in range(5):
            self.api.d.press.home()
            self.api.api_demo_po_launch()
            if self.api.check_ui_exists("textContains", "Profile Owner"):
                break
            time.sleep(3)
        assert self.api.check_ui_exists("textContains", "Profile Owner"), "[ERROR]: fail to launch Work Sample MDM"
        self.api.clean_tasks()
        time.sleep(15)
        for _ in range(5):
            self.api.clean_tasks()
            self.api.launch_app_by_intents("android.settings.SETTINGS", False)
            if not self.api.check_ui_exists("text", "Accounts"):
                self.api.d(scrollable=True).scroll.vert.to(text="Accounts")
            self.api.click_with_timeout("text", "Accounts")
            if self.api.check_ui_exists("text", "Work") and self.api.check_ui_exists("text", "Google"):
                break
            time.sleep(5)
        assert self.api.check_ui_exists("text", "Work"), "[ERROR]: fail to detect work profile"
        assert self.api.check_ui_exists("text", "Google"), "[ERROR]: fail to detect google account"
        assert self.api.d(text="Remove work profile").up(text="Google").exists, "[ERROR]: fail to migrate account"
