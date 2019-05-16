# -*- coding: utf-8 -*-

from igascomparator import igascomparator
from PIL import Image
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
from testlib.AfW.api_impl import ApiImpl
from testlib.util.uiatestbase import UIATestBase
from testlib.common.base import getTmpDir
import time
import os
import random


class CreateOrDeleteAProfile(UIATestBase):
    """
    @summary: Test cases for create or delete profile under device owner
    """

    def setUp(self):
        super(CreateOrDeleteAProfile, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(CreateOrDeleteAProfile, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testProfile_Setup_Wizard(self):
        """
        create or delete profile for device owner
        :return: None
        """
        if self.api.check_managed_user_by_adb():
            self.api.remove_managed_profile(False)
            self.api.clean_tasks()
        self.api.setup_managed_profile(False)
        assert self.api.check_managed_user_by_adb(), "[ERROR]: fail to create profile"
        self.api.clean_tasks()
        self.api.remove_managed_profile(False)
        assert not self.api.check_managed_user_by_adb(), "[ERROR]: fail to remove profile"


class DeviceProvisioning(UIATestBase):
    """
    @summary: Test cases for device provision under device owner
    """

    def setUp(self):
        super(DeviceProvisioning, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        # self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(DeviceProvisioning, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSet_Profile_Name(self):
        """
        set profile name
        :return: None
        """
        profile_name = "HiDeviceOwner"
        if self.api.check_ui_exists("text", "GOT IT"):
            self.api.click_with_timeout("text", "GOT IT")
        self.api.set_profile_name(profile_name, False)
        user_string = os.popen("adb -s {0} shell pm list users".format(self.api.serial)).read().strip()
        assert user_string.find(profile_name) != -1, "[ERROR]: fail to set profile name"

    def testCheck_Profile_Owner_App(self):
        """
        check profile owner
        :return: None
        """
        assert not self.api.check_profile_owner(False), "[ERROR]: intel mdm is profile owner"

    def testCheck_Device_Owner_App(self):
        """
        check device owner
        :return: None
        """
        assert self.api.check_device_owner(False), "[ERROR]: intel mdm is not device owner"

    def testCreate_User(self):
        """
        create user
        :return: None
        """
        users = ""
        self.api.remove_other_users_by_id()
        self.api.create_user_in_mdm("HiCreateUser")
        try:
            for _ in range(3):
                users = os.popen("adb -s {0} shell pm list users".format(self.api.serial)).read().strip()
                if users.find("HiCreateUser") != -1:
                    break
            assert users.find("HiCreateUser") != -1, "fail to detect created HiCreateUser"
        finally:
            self.api.remove_other_users_by_id()

    def testRemove_User(self):
        """
        remove user
        :return: None
        """
        users = ""
        self.api.remove_other_users_by_id()
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.device_provision)
        self.api.click_with_timeout("resourceId", self.api.ui.remove_user)
        while self.api.check_ui_exists("resourceId", "android:id/list"):
            self.api.click_with_timeout("textContains", "Hi")
            self.api.click_with_timeout("text", "Remove User")
            self.api.click_with_timeout("text", "Yes")
            self.api.click_with_timeout("resourceId", self.api.ui.remove_user)
        self.api.click_with_timeout("text", "OK")
        self.api.create_user_in_mdm("HiRemoveUser")
        try:
            for i in range(3):
                users = os.popen("adb -s {0} shell pm list users".format(self.api.serial)).read().strip()
                if users.find("HiRemoveUser") != -1:
                    break
                else:
                    self.api.create_user_in_mdm("HiRemoveUser")
            assert users.find("HiRemoveUser") != -1, "fail to create new user by mdm"
            self.api.clean_tasks()
            self.api.remove_user_in_mdm("HiRemoveUser")
            for j in range(3):
                users = os.popen("adb -s {0} shell pm list users".format(self.api.serial)).read().strip()
                if users.find("HiRemoveUser") == -1:
                    break
            assert users.find("HiRemoveUser") == -1, "fail to remove created user"
        finally:
            self.api.remove_other_users_by_id()

    def testCreate_and_Initialize_User(self):
        """
        create and initialize user
        :return: None
        """
        users = ""
        self.api.remove_other_users_by_id()
        self.api.create_and_initialize_user("HiInitializeUser")
        try:
            for i in range(3):
                users = os.popen("adb -s {0} shell pm list users".format(self.api.serial)).read().strip()
                if users.find("HiInitializeUser") != -1:
                    break
            assert users.find("HiInitializeUser") != -1, "fail to create HiInitializeUser"
            self.api.launch_app_by_intents("android.settings.USER_SETTINGS", False)
            assert not self.api.check_ui_exists("text", "Not set up"), "still detect Not set up"
        finally:
            self.api.remove_other_users_by_id()

    def testSwitch_User(self):
        """
        switch to another user and back
        :return: None
        """
        self.api.remove_other_users_by_id()
        self.api.launch_app_by_intents("android.settings.SYNC_SETTINGS", False)
        if not self.api.check_ui_exists("text", "Accounts"):
            self.api.clean_tasks()
            self.api.settings_sub_launch("Accounts")
        if self.api.check_ui_exists("text", "Work", 5):
            self.api.click_with_timeout("text", "Remove work profile")
            self.api.click_with_timeout("text", "Delete", 10)
        user_name = "HiTest" + str(random.randrange(10, 99, 2))
        self.api.create_and_initialize_user(user_name)
        self.api.click_with_timeout("resourceId", self.api.ui.switch_user)
        try:
            self.api.click_with_timeout("text", user_name)
            self.api.click_with_timeout("text", "Switch User")
            time.sleep(90)  # try to sleep 90 seconds since usb device will disconnect when switching user
            for _ in range(10):
                self.api.d.wakeup()
                if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    assert self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view), \
                        "[ERROR]: fail to switch to new user {0}".format(user_name)
                    self.api.unlock_screen()
                    break
                time.sleep(5)
            self.api.launch_app_by_intents("android.settings.USER_SETTINGS", False)
            assert self.api.check_ui_exists("textContains", user_name, 5), \
                "[ERROR]: fail to switch to new user {0}".format(user_name)
            self.api.click_with_timeout("textContains", "Owner", 5)
            time.sleep(90)  # try to sleep 90 seconds since usb device will disconnect when switching user
            for _ in range(10):
                self.api.d.wakeup()
                if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    assert self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view), \
                        "[ERROR]: fail to switch back to owner"
                    break
                time.sleep(5)
        finally:
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
            self.api.unlock_screen()
            if self.api.check_ui_exists("textContains", "isn't responding", 5):
                self.api.click_with_timeout("text", "OK")
            self.api.clean_tasks()
            self.api.remove_other_users_by_id()

    def testClear_Device_Owner_App(self):
        """
        clear device owner
        :return: None
        """
        # self.api.launch_app_by_intents("android.settings.USER_SETTINGS", False)  # launch settings
        # if not self.api.check_ui_exists("text", "Users"):
        #     self.api.settings_sub_launch("Users")
        # if self.api.check_ui_exists("text", "New user"):  # remove "New user"
        #     if self.api.check_ui_exists("resourceId", "com.android.settings:id/trash_user"):
        #         self.api.d(text="New user").right(resourceId="com.android.settings:id/trash_user").click()
        #         self.api.click_with_timeout("text", "Delete")
        #     elif self.api.check_ui_exists("resourceId", "com.android.settings:id/manage_user"):
        #         self.api.d(text="New user").right(resourceId="com.android.settings:id/manage_user").click()
        #         self.api.click_with_timeout("text", "Remove user")
        #         self.api.click_with_timeout("text", "Delete")
        self.api.remove_other_users_by_id()
        try:
            self.api.clear_device_owner_app()
        finally:
            g_common_obj2.system_reboot(90)
            for _ in range(20):
                self.api = ApiImpl()
                self.api.d = g_common_obj.get_device()
                self.api.d.wakeup()
                if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    break
                time.sleep(5)
            g_common_obj.set_vertical_screen()
            self.api.unlock_screen()
            if self.api.check_ui_exists("textContains", "isn't responding", 5):
                self.api.click_with_timeout("text", "OK")
            self.api.clean_tasks()
            self.api.api_demo_launch()
            assert not self.api.check_ui_exists("textContains", "Device Owner", 10), \
                "[ERROR]: fail to clear device owner"
            assert not self.api.is_device_owner_enabled(), "fail to clear device owner"


class AppProvisioning(UIATestBase):
    """
    @summary: Test cases for applications provisioning under device owner
    """

    def setUp(self):
        super(AppProvisioning, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(AppProvisioning, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testEnable_System_Applications(self):
        """
        enable system applications for device owner
        :return: None
        """
        assert not self.api.locate_apps("Gmail"), "[INFO]: system applications have been enabled"
        self.api.enable_system_applications(False)
        self.api.d.press.home()
        self.api.click_with_timeout("description", "Apps")
        time.sleep(5)
        assert self.api.locate_apps("Gmail", 5), "[ERROR]: system apps have not been enabled"

    def testSelect_Applications_to_Hide(self):
        """
        hide applications for device owner
        :return: None
        """
        assert self.api.locate_apps("Downloads", 5), "[INFO]: Downloads app are already hide"
        self.api.hide_applications(False)
        self.api.d.press.home()
        self.api.click_with_timeout("description", "Apps")
        time.sleep(5)
        assert not self.api.locate_apps("Downloads"), "[ERROR]: fail to hide applications"

    def testList_Hidden_Applications(self):
        """
        list hidden applications for device owner
        :return: None
        """
        assert not self.api.locate_apps("Downloads"), "[INFO]: Downloads app are already exist"
        self.api.unhide_applications(False)
        self.api.d.press.home()
        self.api.click_with_timeout("description", "Apps")
        time.sleep(5)
        assert self.api.locate_apps("Downloads", 5), "[ERROR]: fail to unhide applications"

    def testBlock_Uninstalling_of_Applications(self):
        """
        block uninstalling applications for device owner
        :return: None
        """
        # install_package = self.api.download_file_from_artifactory(self.api.remote.qq_ime['sub_path'],
        #                                                           self.api.remote.qq_ime['name'])
        # self.api.unknown_source_control(True)
        # assert install_package is not None, "[ERROR]: fail to download QQ_IME.apk"
        # old_dir = os.getcwd()
        # os.chdir(os.path.split(install_package)[0])
        # self.api.install_apps(self.api.remote.qq_ime['pkg_name'], self.api.remote.qq_ime['name'].split('.apk')[0])
        # os.chdir(old_dir)
        # self.api.block_uninstall_apps(False)
        # self.api.launch_app_by_intents("android.settings.APPLICATION_DETAILS_SETTINGS -d " +
        #                                self.api.remote.qq_ime['pkg_name'], False)
        # self.api.click_with_timeout("text", "Uninstall")
        # self.api.click_with_timeout("text", "OK")
        # assert self.api.check_ui_exists("resourceId", "com.android.packageinstaller:id/device_manager_button", 10), \
        #     "[ERROR]: fail to block uninstall apps"
        self.api.unknown_source_control(True)
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.app_provision)
        for _ in range(3):
            app_info = os.popen("adb -s {0} shell pm list packages com.example.jizhenlo.runtimepermissiontest".format(
                self.api.serial)).read().strip()
            if app_info.find("com.example.jizhenlo.runtimepermissiontest") != -1:
                break
            self.api.click_with_timeout("text", "Install A Runtime Permission Test Sample")
            self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/ok_button")
            self.api.click_with_timeout("text", "ACCEPT")
            self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/done_button")
        self.api.block_uninstall_apps(False)
        self.api.launch_app_by_intents(
            "android.settings.APPLICATION_DETAILS_SETTINGS -d package:com.example.jizhenlo.runtimepermissiontest", False)
        self.api.click_with_timeout("text", "Uninstall")
        self.api.click_with_timeout("text", "OK")
        assert self.api.check_ui_exists("resourceId", "com.android.packageinstaller:id/device_manager_button", 5), \
            "[ERROR]: fail to block uninstall apps"

    def testBlocked_Uninstalling_Applications(self):
        """
        unblock uninstalling applications for device owner
        :return: None
        """
        # self.api.unblock_uninstall_apps(False)
        # self.api.launch_app_by_intents("android.settings.APPLICATION_DETAILS_SETTINGS -d " +
        #                                self.api.remote.qq_ime['pkg_name'], False)
        # self.api.click_with_timeout("text", "Uninstall")
        # self.api.click_with_timeout("text", "OK")
        # assert not self.api.check_ui_exists("resourceId", "com.android.packageinstaller:id/device_manager_button", 5), \
        #     "[ERROR]: fail to uninstall apps"
        # self.api.uninstall_apps_by_adb(self.api.remote.qq_ime['pkg_name'])
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.app_provision)
        for _ in range(3):
            app_info = os.popen("adb -s {0} shell pm list packages com.example.jizhenlo.runtimepermissiontest".format(
                self.api.serial)).read().strip()
            if app_info.find("com.example.jizhenlo.runtimepermissiontest") != -1:
                break
            self.api.click_with_timeout("text", "Install A Runtime Permission Test Sample")
            self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/ok_button")
            self.api.click_with_timeout("text", "ACCEPT")
            self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/done_button")
        self.api.unblock_uninstall_apps(False)
        self.api.launch_app_by_intents(
            "android.settings.APPLICATION_DETAILS_SETTINGS -d package:com.example.jizhenlo.runtimepermissiontest", False)
        self.api.click_with_timeout("text", "Uninstall")
        self.api.click_with_timeout("text", "OK")
        app_uninstalled = False
        for _ in range(3):
            app_info = os.popen("adb -s {0} shell pm list packages com.example.jizhenlo.runtimepermissiontest".format(
                self.api.serial)).read().strip()
            if app_info.find("com.example.jizhenlo.runtimepermissiontest") == -1:
                app_uninstalled = True
                break
        assert app_uninstalled, "fail to uninstall app after unblock"


class IntentsDataSharing(UIATestBase):
    """
    @summary: Test cases for intents and data sharing under device owner1
    """

    def setUp(self):
        super(IntentsDataSharing, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(IntentsDataSharing, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSend_Intent_toHandle_Text_Plain(self):
        """
        send intent to handle text/plain for device owner
        :return: None
        """
        self.api.send_intent_to_handle(False)
        self.api.d(resourceId="android:id/resolver_list").wait.exists(timeout=3000)
        assert self.api.check_ui_exists("resourceId", "android:id/resolver_list"), \
            "[ERROR]: share window doesn't pop up"

    def testAdd_PersistentPreferredActivity_for_Text_Plain(self):
        """
        add persistent preferred activity for text plain
        :return: None
        """
        self.api.add_persistent_activities(False)
        self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_handle)
        assert not self.api.check_ui_exists("resourceId", "android:id/resolver_list"), \
            "[ERROR]: share window still pop up"

    def testClear_Persistent_Preferred_Activities(self):
        """
        clear persistent preferred activity for device owner
        :return: None
        """
        self.api.clear_persistent_activities(False)
        self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_handle)
        self.api.d(resourceId="android:id/resolver_list").wait.exists(timeout=3000)
        assert self.api.check_ui_exists("resourceId", "android:id/resolver_list"), \
            "[ERROR]: share window doesn't pop up"


class AppConfigAndPolicy(UIATestBase):
    """
    @summary: Test cases for app config and policy under device owner
    """

    def setUp(self):
        super(AppConfigAndPolicy, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(AppConfigAndPolicy, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSet_Application_Restriction_Chrome(self):
        """
        set application restriction for chrome under device owner
        :return: None
        """
        if not self.api.locate_apps("Chrome"):
            self.api.enable_system_applications(False)
            self.api.clean_tasks()
        self.api.set_apps_restriction_chrome(False)
        if "afwHandleChromeInvitedWindow" not in self.api.d.watchers:
            self.api.d.watcher("afwHandleChromeInvitedWindow").when(
                text="No Thanks").click(text="No Thanks")
        if not self.api.launch_app_by_activity(self.api.ui.chrome, False):
            self.api.launch_app("Chrome")
        self.api.d(packageName="com.android.chrome").wait.exists(timeout=5000)
        for _ in range(3):
            if self.api.check_ui_exists("text", "Welcome to Chrome"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/terms_accept")
                self.api.click_with_timeout("text", "Next")
                self.api.click_with_timeout("text", "No thanks")
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/got_it_button")
                break
        for i in range(5):
            if self.api.check_ui_exists("textContains", "incognito"):
                break
            self.api.d.press.menu()
            self.api.d(textContains="incognito").wait.exists(timeout=3000)
        assert self.api.check_ui_exists("textContains", "incognito", 5), "[ERROR]: menu window doesn't pop up"
        assert not self.api.d(textContains="incognito").enabled, "[ERROR]: fail to set restriction for chrome"

    def testGet_Application_Restriction_Chrome(self):
        """
        get application restriction for chrome under device owner
        :return: None
        """
        self.api.get_apps_restriction_chrome(False)
        assert self.api.check_ui_exists("text", "Restrictions"), "[ERROR]: restrictions window doesn't pop up"
        assert self.api.check_ui_exists("textContains", "ManagedBookmarks"), "[ERROR]: fail to get restrictions content"
        self.api.click_with_timeout("text", "OK")

    def testClear_Application_Restriction_Chrome(self):
        """
        clear application restriction for chrome under device owner
        :return: None
        """
        self.api.clear_apps_restriction_chrome(False)
        if not self.api.launch_app_by_activity(self.api.ui.chrome, False):
            self.api.launch_app("Chrome")
        self.api.d(packageName="com.android.chrome").wait.exists(timeout=5000)
        for _ in range(3):
            if self.api.check_ui_exists("text", "Welcome to Chrome"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/terms_accept")
                self.api.click_with_timeout("text", "Next")
                self.api.click_with_timeout("text", "No thanks")
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/got_it_button")
                self.api.d.wait.update(timeout=3000)
                break
        for i in range(5):
            if self.api.check_ui_exists("textContains", "incognito"):
                break
            self.api.d.press.menu()
            self.api.d(textContains="incognito").wait.exists(timeout=3000)
        assert self.api.check_ui_exists("textContains", "incognito", 5), "[ERROR]: menu window doesn't pop up"
        assert self.api.d(textContains="incognito").enabled, "[ERROR]: fail to clear restriction for chrome"
        if "afwHandleChromeInvitedWindow" in self.api.d.watchers:
            self.api.d.watchers.remove("afwHandleChromeInvitedWindow")

    def testSetRestrictionProvider_SetTheDefaultRestrictionProvider(self):
        """
        set restriction provider for device owner
        :return: None
        """
        try:
            self.api.set_clear_restriction_provider(True, False)
            assert self.api.check_ui_exists("text", "Set com.intel.afw.mdm.BasicRestrictionsReceiver as the restrictions provider", 5), \
                "[ERROR]: fail to set restriction provider"
        finally:
            self.api.clean_tasks()
            self.api.set_clear_restriction_provider(False, False)
            self.api.click_toast_window()

    def testSetRestrictionProvider_ClearTheDefaultRestrictionProvider(self):
        """
        clear restriction provider for device owner
        :return: None
        """
        try:
            self.api.set_clear_restriction_provider(False, False)
            assert self.api.check_ui_exists("text", "Delete the restrictions provider", 5), \
                "[ERROR]: fail to clear restriction provider"
        finally:
            self.api.clean_tasks()
            self.api.set_clear_restriction_provider(False, False)
            self.api.click_toast_window()

    def testCheck_Restrictions_Provider(self):
        """
        check restriction provider for device owner
        :return: None
        """
        try:
            self.api.set_clear_restriction_provider(True, False)
            self.api.clean_tasks()
            self.api.check_restriction_provider(False)
            assert self.api.check_ui_exists("text", "A restrictions provider has been set for this profile"), \
                "[ERROR]: fail to check restriction provider"
        finally:
            self.api.clean_tasks()
            self.api.set_clear_restriction_provider(False, False)

    def testInstall_AppRestrictionSchema(self):
        """
        install app restriction schema for device owner
        :return: None
        """
        self.api.unknown_source_control(True)
        self.api.install_apprestriction_schema(False)
        if not self.api.locate_apps("AppRestrictionSchema"):
            g_common_obj2.system_reboot(90)
            for i in range(20):
                self.api.d = g_common_obj.get_device()
                self.api.d.wakeup()
                if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    break
                time.sleep(5)
            g_common_obj.set_vertical_screen()
            self.api.unlock_screen()
        ret = os.popen("adb -s {0} shell pm list packages com.example.android.apprestrictionschema".format(
            self.api.serial)).read().strip()
        assert ret.find("apprestrictionschema") != -1, "fail to detect installed restriction schema"

    def testSet_Restrictions_Policy_For_AppRestrictionSchema(self):
        """
        set restriction schema for device owner
        :return: None
        """
        try:
            if not self.api.locate_apps("AppRestrictionSchema"):
                self.api.install_apprestriction_schema(False)
                self.api.clean_tasks()
            self.api.set_clear_restriction_provider(True, False)
            self.api.click_toast_window()
            self.api.click_with_timeout("text", self.api.ui.set_apprestriction_schema)
            self.api.click_with_timeout("text", "Approve")
            # time.sleep(5)
            # self.api.launch_app("AppRestrictionSchema")
            for i in range(2):
                if not self.api.launch_app_by_activity(self.api.ui.app_schema, False):
                    self.api.launch_app("AppRestrictionSchema")
                self.api.d(resourceId="com.example.android.apprestrictionschema:id/say_hello").wait.exists(timeout=5000)
                if self.api.d(resourceId="com.example.android.apprestrictionschema:id/say_hello").enabled:
                    break
                for j in range(5):
                    if self.api.d(resourceId="com.example.android.apprestrictionschema:id/say_hello").enabled:
                        break
                    self.api.click_with_timeout("resourceId",
                                                "com.example.android.apprestrictionschema:id/request_configuration")
                    time.sleep(2)
                self.api.clean_tasks()
            assert self.api.d(resourceId="com.example.android.apprestrictionschema:id/say_hello").enabled, \
                "[ERROR]: fail to request permission"
            assert self.api.check_ui_exists("text", "I can say hello to you."), "[ERROR]: fail to check approve string"
            self.api.clean_tasks()
            self.api.set_clear_restriction_provider(True, False)
            self.api.click_toast_window()
            self.api.click_with_timeout("text", self.api.ui.set_apprestriction_schema)
            self.api.click_with_timeout("text", "Deny")
            time.sleep(5)
            # self.api.launch_app("AppRestrictionSchema")
            if not self.api.launch_app_by_activity(self.api.ui.app_schema, False):
                self.api.launch_app("AppRestrictionSchema")
            self.api.click_with_timeout("resourceId",
                                        "com.example.android.apprestrictionschema:id/request_configuration", 10)
            assert not self.api.d(resourceId="com.example.android.apprestrictionschema:id/say_hello").enabled, \
                "[ERROR]: fail to deny request"
            assert self.api.check_ui_exists("text", "I am restricted from saying hello to you."), \
                "[ERROR] fail to check deny string"
        finally:
            self.api.clean_tasks()
            self.api.set_clear_restriction_provider(False, False)
            self.api.click_toast_window()
            self.api.uninstall_apps_by_adb(self.api.remote.restriction_schema['pkg_name'])

    def testSilentlyInstallApp(self):
        """
        verify that able to install apk silently under device owner
        """
        res = os.popen("adb -s {0} shell pm list packages com.android.trustagent.testsample".format(
            self.api.serial)).read().strip()
        if res.find("trustagent.testsample") != -1:
            self.api.uninstall_apps_by_adb("com.android.trustagent.testsample")
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.app_provision)
        self.api.click_with_timeout("text", "Install An App Silently")
        # assert self.api.check_ui_exists("text", "Install Sample Trust Agent App Successfully", 10), \
        #     "fail to install app silently"
        self.api.click_with_timeout("text", "OK")
        res = os.popen("adb -s {0} shell pm list packages com.android.trustagent.testsample".format(
            self.api.serial)).read().strip()
        assert res.find("trustagent.testsample") != -1, "fail to detect installed test sample"

    def testSilentlyUninstallApp(self):
        """
        verify that able to uninstall apk silently under device owner
        """
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.app_provision)
        self.api.click_with_timeout("text", "Uninstall An App Silently")
        # assert self.api.check_ui_exists("text", "Uninstall Sample Trust Agent App Successfully", 10), \
        #     "fail to uninstall app silently"
        self.api.click_with_timeout("text", "OK")
        res = os.popen("adb -s {0} shell pm list packages com.android.trustagent.testsample".format(
            self.api.serial)).read().strip()
        assert res.find("trustagent.testsample") == -1, "still able to detect uninstalled test sample"


class DeviceProfileManagementPolicies(UIATestBase):
    """
    @summary: Test cases for device profile management and policies under device owner
    """

    def setUp(self):
        super(DeviceProfileManagementPolicies, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(DeviceProfileManagementPolicies, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSetGet_Master_VolumeMuteState(self):
        """
        verify mute/unmute dut
        :return: None
        """
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.device_management_policy)
        self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetOrGetMasterVol")
        if self.api.check_ui_exists("text", "Unmute Master Volume"):
            self.api.click_with_timeout("text", "Unmute Master Volume")
        dump_master_mute = os.popen("adb -s {0} shell dumpsys activity broadcasts".format(self.api.serial)).read()
        locate_mute_string = dump_master_mute.find("android.media.EXTRA_MASTER_VOLUME_MUTED")
        assert locate_mute_string != -1, "fail to locate android.media.EXTRA_MASTER_VOLUME_MUTED in dumpsys"
        locate_end = dump_master_mute[locate_mute_string:].find("}]")
        assert locate_end != -1, "fail to find the end string"
        mute_state_before_set = dump_master_mute[locate_mute_string:][:locate_end].split("=")[-1]
        assert mute_state_before_set == "false", "master already muted"
        if self.api.check_ui_exists("text", "Mute Master Volume"):
            self.api.click_with_timeout("text", "Mute Master Volume")
        else:
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetOrGetMasterVol")
            self.api.click_with_timeout("text", "Mute Master Volume")
        dump_master_mute = os.popen("adb -s {0} shell dumpsys activity broadcasts".format(self.api.serial)).read()
        locate_mute_string = dump_master_mute.find("android.media.EXTRA_MASTER_VOLUME_MUTED")
        assert locate_mute_string != -1, "fail to locate android.media.EXTRA_MASTER_VOLUME_MUTED in dumpsys"
        locate_end = dump_master_mute[locate_mute_string:].find("}]")
        assert locate_end != -1, "fail to find the end string"
        mute_state_after_set = dump_master_mute[locate_mute_string:][:locate_end].split("=")[-1]
        assert mute_state_after_set == "true", "fail to mute master volume"
        self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetOrGetMasterVol")
        self.api.click_with_timeout("text", "Unmute Master Volume")

    def testSetGet_ScreenCaptureState(self):
        """
        set or get screen capture state for device owner
        :return: None
        """
        temp_path = getTmpDir()
        self.api.set_get_screen_capture(False, False)
        self.api.d.press("home")
        time.sleep(5)
        assert self.api.d.screenshot(os.path.join(temp_path, "screenshotDisabled.png")) is None, \
            "[ERROR]: still able to take a screenshot"
        self.api.clean_tasks()
        self.api.set_get_screen_capture(True, False)
        self.api.d.press("home")
        time.sleep(5)
        assert self.api.d.screenshot(os.path.join(temp_path, "screenshotEnabled.png")) is not None, \
            "[ERROR]: unable to take a screenshot"

    def testDisable_Account_Management_Add(self):
        """
        add new account in disable account management list for device owner
        :return: None
        """
        self.api.disable_account_management(True, False)
        self.api.settings_sub_launch("Accounts")
        self.api.click_with_timeout("text", "Add account")
        self.api.click_with_timeout("text", "Google")
        assert self.api.check_ui_exists("text", "This change isn't allowed by your administrator", 5), \
            "[ERROR]: still allow to add google account"

    def testDisable_Account_Management_Del(self):
        """
        delete exist account in disable account management list for device owner
        :return: None
        """
        self.api.disable_account_management(False, False)
        self.api.settings_sub_launch("Accounts")
        self.api.click_with_timeout("text", "Add account")
        self.api.click_with_timeout("text", "Google")
        assert not self.api.check_ui_exists("text", "This change isn't allowed by your administrator", 5), \
            "[ERROR]: fail to add google account"

    def testSetUserRestrictions_Disallow_modify_accounts(self):
        """
        disallow modify accounts for device owner
        :return: None
        """
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_modify_account + "OFF", False)
            self.api.settings_sub_launch("Accounts")
            assert not self.api.check_ui_exists("text", "Add account"), "ERROR: still able to modify account"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_modify_account + "ON", False)

    def testSetUserRestrictions_Disallow_share_location(self):
        """
        disallow share location for device owner
        :return: None
        """
        for _ in range(3):  # seems like sometimes settings crashed
            self.api.launch_app_by_intents("android.settings.LOCATION_SOURCE_SETTINGS", False)
            self.api.click_with_timeout("text", "Agree", 5)
            if not self.api.check_ui_exists("text", "Location"):
                self.api.settings_sub_launch("Location")
                self.api.click_with_timeout("text", "Agree", 5)
            if self.api.check_ui_exists("text", "Location"):
                break
        assert self.api.check_ui_exists("text", "Location"), "fail to launch location settings"
        if self.api.check_ui_exists("text", "Off"):
            self.api.click_with_timeout("text", "Off")
            self.api.click_with_timeout("text", "Agree", 5)
        assert self.api.check_ui_exists("text", "On"), "location sharing already turned off"
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_share_location + "OFF", False)
            self.api.launch_app_by_intents("android.settings.LOCATION_SOURCE_SETTINGS", False)
            self.api.click_with_timeout("text", "Agree", 5)
            if not self.api.check_ui_exists("text", "Location"):
                self.api.settings_sub_launch("Location")
                self.api.click_with_timeout("text", "Agree", 5)
            assert self.api.check_ui_exists("text", "Off", 5), "fail to detect off option"
            location_enabled = True
            for _ in range(3):
                if self.api.check_ui_exists("text", "Mode"):
                    if not self.api.d(text="Mode").enabled:
                        location_enabled = False
                        break
            assert not location_enabled, "[ERROR]: fail to disallow share location"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_share_location + "ON", False)

    def testSetUserRestrictions_Disallow_apps_control(self):
        """
        disallow apps control for device owner
        :return: None
        """
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_apps_control + "OFF", False)
            self.api.launch_app_by_intents(
                "android.settings.APPLICATION_DETAILS_SETTINGS -d package:com.android.contacts", False)
            if not self.api.check_ui_exists("text", "App info", 5):
                self.api.launch_app_by_intents("android.settings.APPLICATION_SETTINGS", False)
                self.api.click_with_timeout("text", "Running")
                self.api.click_with_timeout("text", "All")
                if not self.api.check_ui_exists("text", "Contacts", 5):
                    self.api.d(scrollable=True).scroll.vert.to(text="Contacts")
                self.api.click_with_timeout("text", "Contacts")
            self.api.check_ui_exists("text", "Force stop", 5)
            assert not self.api.d(text="Force stop").enabled, "[ERROR]: fail to disallow apps control"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_apps_control + "ON", False)

    def testSetUserRestrictions_Disallow_install_apps(self):
        """
        disallow install apps for device owner
        :return: None
        """
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_install_apps + "OFF", False)
            os.popen("adb -s {0} shell am start -a android.settings.SECURITY_SETTINGS".format(self.api.serial)).read()
            if not self.api.check_ui_exists("text", "Unknown sources"):
                self.api.d(scrollable=True).scroll.vert.to(text="Unknown sources")
            assert not self.api.d(text="Unknown sources").enabled, "[ERROR]: fail to disallow install apps"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_install_apps + "ON", False)

    def testSetUserRestrictions_Disallow_install_unknown_sources(self):
        """
        disallow install unknown sources for device owner
        :return: None
        """
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_unknown_source + "OFF", False)
            os.popen("adb -s {0} shell am start -a android.settings.SECURITY_SETTINGS".format(self.api.serial)).read()
            if not self.api.check_ui_exists("text", "Unknown sources"):
                self.api.d(scrollable=True).scroll.vert.to(text="Unknown sources")
            assert not self.api.d(text="Unknown sources").enabled, "[ERROR]: fail to disallow unknown source"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_unknown_source + "ON", False)

    def testSetUserRestrictions_Disallow_remove_user(self):
        """
        disallow remove user for device owner
        :return: None
        """
        self.api.remove_other_users_by_id()
        self.api.launch_app_by_intents("android.settings.USER_SETTINGS", False)
        if not self.api.check_ui_exists("text", "New user"):
            self.api.click_with_timeout("text", "Add user")
            self.api.click_with_timeout("text", "OK")
            self.api.click_with_timeout("text", "Not now", 5)
        try:
            if self.api.check_ui_exists("resourceId", "com.android.settings:id/manage_user"):
                self.api.d(text="New user").right(resourceId="com.android.settings:id/manage_user").click.wait()
                assert self.api.check_ui_exists("text", "Remove user"), "[ERROR]: Remove user doesn't exist"
            else:
                assert self.api.check_ui_exists("resourceId", "com.android.settings:id/trash_user"), \
                    "[ERROR]: unable to remove user"
            self.api.set_user_restrictions(self.api.ui.disallow_remove_users + "OFF", False)
            self.api.launch_app_by_intents("android.settings.USER_SETTINGS", False)
            if self.api.check_ui_exists("resourceId", "com.android.settings:id/manage_user"):
                self.api.d(text="New user").right(resourceId="com.android.settings:id/manage_user").click.wait()
                assert not self.api.check_ui_exists("text", "Remove user"), "[ERROR]: Remove user still exist"
            else:
                assert not self.api.check_ui_exists("resourceId", "com.android.settings:id/trash_user"), \
                    "[ERROR]: still able to remove users"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_remove_users + "ON", False)
            self.api.remove_other_users_by_id()

    def testSetUserRestrictions_Disallow_config_VPN(self):
        """
        disallow config vpn for device owner
        :return: None
        """
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_config_vpn + "OFF", False)
            self.api.launch_app_by_intents("android.settings.SETTINGS", False)
            self.api.click_with_timeout("text", "More")
            assert not self.api.check_ui_exists("text", "VPN"), "[ERROR]: VPN item still exists"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_config_vpn + "ON", False)

    def testSetUserRestrictions_Disallow_factory_reset(self):
        """
        disallow factory reset for device owner
        :return: None
        """
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_factory_reset + "OFF", False)
            self.api.settings_sub_launch("Backup & reset")
            assert not self.api.check_ui_exists("text", "Factory data reset"), \
                "[ERROR]: factory data reset still exists"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_factory_reset + "ON", False)

    def testSetUserRestrictions_Disallow_config_credentials(self):
        """
        disallow config credentials for device owner
        :return: None
        """
        cert_file = self.api.download_file_from_artifactory(self.api.remote.cert_file['sub_path'],
                                                            self.api.remote.cert_file['name'])
        assert cert_file is not None, "[ERROR]: fail to download CA Cert file"
        if os.popen('adb -s {0} shell ls /mnt'.format(self.api.serial)).read().strip().find('shell') != -1:
            os.popen("adb -s {0} push {1} /mnt/shell/emulated/0/Download/".format(self.api.serial, cert_file))
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_config_credentials + "OFF", False)
            self.api.clean_tasks()
            self.api.install_ca_cert(False)
            assert self.api.check_ui_exists("textContains", "DISALLOW_CONFIG_CREDENTIALS"), \
                "[ERROR]: still able to install ca cert"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_config_credentials + "ON", False)

    def testSetUserRestrictions_Disallow_add_user(self):
        """
        disallow add user for device owner
        :return: None
        """
        self.api.remove_other_users_by_id()
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_add_user + "OFF", False)
            os.popen("adb -s {0} shell am start -a android.settings.USER_SETTINGS".format(self.api.serial)).read()
            assert not self.api.check_ui_exists("text", "Add user"), "[ERROR]: add user item still exists"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_add_user + "ON", False)

    def testSetUserRestrictions_Ensure_verify_apps(self):
        """
        ensure verify apps for device owner
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.APPLICATION_DEVELOPMENT_SETTINGS", False)
        if self.api.d(resourceId="android:id/list").scrollable:
            self.api.d(resourceId="android:id/list").scroll.to(text="Verify apps over USB")
        if self.api.d(text="Verify apps over USB").right(resourceId="android:id/switchWidget").text == u'ON':
            self.api.d(text="Verify apps over USB").click()
        try:
            self.api.set_user_restrictions(self.api.ui.ensure_verify_apps + "OFF", False)
            self.api.launch_app_by_intents("android.settings.APPLICATION_DEVELOPMENT_SETTINGS", False)
            if self.api.d(resourceId="android:id/list").scrollable:
                self.api.d(resourceId="android:id/list").scroll.vert.to(text="Verify apps over USB")
            assert self.api.d(text="Verify apps over USB").right(resourceId="android:id/switchWidget").text == u'ON',\
                "[ERROR]: still unable to verify apps over usb"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.ensure_verify_apps + "ON", False)
            self.api.launch_app_by_intents("android.settings.APPLICATION_DEVELOPMENT_SETTINGS", False)
            if self.api.d(resourceId="android:id/list").scrollable:
                self.api.d(resourceId="android:id/list").scroll.to(text="Verify apps over USB")
            if self.api.d(text="Verify apps over USB").right(resourceId="android:id/switchWidget").text == u'ON':
                self.api.d(text="Verify apps over USB").click()

    def testSetUserRestrictions_Disallow_config_tethering(self):
        """
        disallow config tethering for device owner
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.SETTINGS", False)
        self.api.click_with_timeout("text", "More")
        if self.api.check_ui_exists("text", "Tethering"):
            self.api.click_with_timeout("text", "Tethering")
        if self.api.check_ui_exists("text", "Bluetooth tethering"):
            assert self.api.check_ui_exists("text", "Bluetooth tethering"), "[ERROR]: BT tethering already disabled"
        elif self.api.check_ui_exists("text", "Tethering & portable hotspot"):
            assert self.api.check_ui_exists("text", "Tethering & portable hotspot"), \
                "[ERROR]: Tethering & portable hotspot already disabled"
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_config_tethering + "OFF", False)
            self.api.launch_app_by_intents("android.settings.SETTINGS", False)
            self.api.click_with_timeout("text", "More")
            assert not self.api.check_ui_exists("text", "Tethering"), "[ERROR]: Tethering still exist"
            assert not self.api.check_ui_exists("text", "Bluetooth tethering"), "[ERROR]: BT tethering still exist"
            assert not self.api.check_ui_exists("text", "Tethering & portable hotspot"), \
                "[ERROR]: Tethering & portable hotspot still exist"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_config_tethering + "ON", False)

    def testSetUserRestrictions_Disallow_config_WiFi(self):
        """
        disallow config wifi for device owner
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.WIFI_SETTINGS", False)
        assert not self.api.check_ui_exists("textContains", "have permission"), "[ERROR]: WiFi has been disallowed"
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_config_wifi + "OFF", False)
            self.api.clean_tasks()
            self.api.launch_app_by_intents("android.settings.WIFI_SETTINGS", False)
            assert self.api.check_ui_exists("textContains", "have permission"), "[ERROR]: fail to disallow config wifi"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_config_wifi + "ON", False)

    def testSet_or_Clear_AutoTimeRequired(self):
        """
        set and clear auto time required
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.DATE_SETTINGS", False)
        assert self.api.check_ui_exists("text", "Automatic date & time"), "[ERROR]: fail to launch Date & time"
        assert self.api.d(text="Automatic date & time").enabled, "[ERROR]: auto time have been disabled"
        self.api.set_clear_auto_time_required(True)
        self.api.clean_tasks()
        self.api.launch_app_by_intents("android.settings.DATE_SETTINGS", False)
        assert self.api.check_ui_exists("text", "Automatic date & time"), "[ERROR]: fail to launch Date & time"
        assert not self.api.d(text="Automatic date & time").enabled, "[ERROR]: fail to disable auto time"
        self.api.clean_tasks()
        self.api.set_clear_auto_time_required(False)
        self.api.launch_app_by_intents("android.settings.DATE_SETTINGS", False)
        assert self.api.check_ui_exists("text", "Automatic date & time"), "[ERROR]: fail to launch Date & time"
        assert self.api.d(text="Automatic date & time").enabled, "[ERROR]: auto time have been disabled"

    def testSetLockTask_AddEnable_MDM_to_TaskLocking(self):
        """
        Add Sample MDM to task locking mode
        :return: None
        """
        self.api.enable_mdm_to_task_lock()
        self.api.d.press.home()
        assert not self.api.check_ui_exists("description", "Apps"), "[ERROR]: fail to lock tasks"
        if self.api.d(text="Sample MDM").right(resourceId="com.intel.afw.mdm:id/task_lock_state").text == u'ON':
            self.api.click_with_timeout("text", "Sample MDM")
        self.api.click_with_timeout("resourceId", self.api.ui.lock_task_mode)
        self.api.d.press.home()
        assert self.api.check_ui_exists("description", "Apps"), "[ERROR]: unable to exit from lock task mode"

    def testSetLockTask_RemoveDisable_MDM_to_TaskLocking(self):
        """
        Remove Sample MDM from task locking mode
        :return: None
        """
        self.api.enable_mdm_to_task_lock()
        self.api.d.press.home()
        assert not self.api.check_ui_exists("description", "Apps"), "[ERROR]: fail to lock tasks"
        if self.api.d(text="Sample MDM").right(resourceId="com.intel.afw.mdm:id/task_lock_state").text == u'ON':
            self.api.click_with_timeout("text", "Sample MDM")
        self.api.click_with_timeout("resourceId", self.api.ui.lock_task_mode)
        self.api.d.press.home()
        assert self.api.check_ui_exists("description", "Apps"), "[ERROR]: unable to exit from lock task mode"

    def testSetGlobalOrSecureSettings_Default_input_method(self):
        """
        Change default input method by set global or secure settings in Intel Sample MDM
        :return: None
        """
        self.api.settings_sub_launch("Language & input")
        assert self.api.check_ui_exists("text", "Current Keyboard"), "[ERROR]: fail to launch Language & input"
        assert self.api.d(text="Current Keyboard").down(textContains="Google Keyboard").exists, \
            "[ERROR]: the default input method is not Google Keyboard"
        self.api.change_global_spinner_settings(self.api.ui.global_spinner_input)
        self.api.click_with_timeout("text", "com.google.android.googlequicksearchbox")
        self.api.click_with_timeout("text", "OK")
        self.api.clean_tasks()
        self.api.settings_sub_launch("Language & input")
        assert self.api.check_ui_exists("text", "Current Keyboard"), "[ERROR]: fail to launch Language & input"
        assert self.api.d(text="Current Keyboard").down(textContains="Google voice typing").exists, \
            "[ERROR]: fail to set voice typing as default input method"
        self.api.clean_tasks()
        self.api.change_global_spinner_settings(self.api.ui.global_spinner_input)
        self.api.click_with_timeout("text", "com.google.android.inputmethod.latin")
        self.api.click_with_timeout("text", "OK")

    def testSetGlobalOrSecureSettings_Auto_time_enable(self):
        """
        enable or disable auto time settings in set global or secure settings page
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.DATE_SETTINGS", False)
        assert self.api.check_ui_exists("text", "Set date"), "[ERROR]: fail to launch date & time"
        assert not self.api.d(text="Set date").enabled, "[ERROR]: set date is enabled"
        assert not self.api.d(text="Set time").enabled, "[ERROR]: set time is enabled"
        self.api.set_global_secure_settings(self.api.ui.global_auto_time + "ON")
        self.api.clean_tasks()
        self.api.launch_app_by_intents("android.settings.DATE_SETTINGS", False)
        assert self.api.check_ui_exists("text", "Set date"), "[ERROR]: fail to launch date & time"
        assert self.api.d(text="Set date").enabled, "[ERROR]: set date is disabled"
        assert self.api.d(text="Set time").enabled, "[ERROR]: set time is disabled"
        self.api.set_global_secure_settings(self.api.ui.global_auto_time + "OFF")

    def testSetGlobalOrSecureSettings_WiFi(self):
        """
        disable wifi from Sample MDM
        :return: None
        """
        # make sure wifi is enabled
        self.api.set_wifi_status(True)
        # launch Sample MDM to disable wifi
        self.api.set_global_secure_settings(self.api.ui.global_wifi + "ON")
        self.api.click_with_timeout("text", "OK")
        # reboot device
        g_common_obj2.system_reboot(90)
        for _ in range(20):
            self.api = ApiImpl()
            self.api.d = g_common_obj.get_device()
            self.api.d.wakeup()
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                break
            time.sleep(5)
        g_common_obj.set_vertical_screen()
        self.api.unlock_screen()
        if self.api.check_ui_exists("textContains", "isn't responding", 5):
            self.api.click_with_timeout("text", "OK")
        # verify that wifi is disabled now
        self.api.clean_tasks()
        self.api.launch_app_by_intents("android.settings.WIFI_SETTINGS", False)
        if not self.api.check_ui_exists("textMatches", "Wi.Fi"):
            self.api.settings_sub_launch("Wi.Fi")
        assert self.api.check_ui_exists("textMatches", "Wi.Fi"), "[ERROR]: fail to launch WiFi"
        assert self.api.check_ui_exists("text", "Off"), "[ERROR]: fail to disable WiFi"
        # turn on wifi
        self.api.d(text="Off").right(resourceId="com.android.settings:id/switch_widget").click()

    def testSetGlobalOrSecureSettings_Bluetooth_enable(self):
        """
        enable bluetooth from Sample MDM
        :return: None
        """
        # make sure bt is disabled
        self.api.set_bluetooth_status(False)
        # launch Sample MDM to enable bt
        self.api.set_global_secure_settings(self.api.ui.global_bluetooth + "OFF")
        self.api.click_with_timeout("text", "OK")
        # reboot device
        g_common_obj2.system_reboot(90)
        for _ in range(20):
            self.api = ApiImpl()
            self.api.d = g_common_obj.get_device()
            self.api.d.wakeup()
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                break
            time.sleep(5)
        g_common_obj.set_vertical_screen()
        self.api.unlock_screen()
        if self.api.check_ui_exists("textContains", "isn't responding", 5):
            self.api.click_with_timeout("text", "OK")
        self.api.clean_tasks()
        self.api.launch_app_by_intents("android.settings.BLUETOOTH_SETTINGS", False)
        if not self.api.check_ui_exists("text", "Bluetooth"):
            self.api.settings_sub_launch("Bluetooth")
        assert self.api.check_ui_exists("text", "Bluetooth"), "[ERROR]: fail to launch BT"
        for _ in range(5):
            if self.api.check_ui_exists("text", "On", 5):
                break
            time.sleep(5)
        assert self.api.check_ui_exists("text", "On"), "[ERROR]: fail to enable BT"
        self.api.d(text="On").right(resourceId="com.android.settings:id/switch_widget").click()

    def testSetGlobalOrSecureSettings_WiFi_sleep_policy(self):
        """
        change wifi sleep policy in set global or secure settings page
        :return: None
        """
        self.api.change_global_spinner_settings(self.api.ui.global_spinner_wifi)
        self.api.click_with_timeout("text", "never")
        self.api.click_with_timeout("text", "OK")
        self.api.launch_app_by_intents("android.settings.WIFI_SETTINGS", False)
        assert self.api.check_ui_exists("resourceId", "com.android.settings:id/switch_widget"), \
            "[ERROR]: fail to launch wifi settings"
        self.api.d.press.menu()
        assert self.api.check_ui_exists("text", "Advanced"), "[ERROR]: menu window in wifi settings doesn't pop up"
        self.api.click_with_timeout("text", "Advanced")
        assert self.api.check_ui_exists("textContains", "on during sleep"), \
            "[ERROR]: Keep Wi-Fi on during sleep doesn't exist"
        assert self.api.d(textContains="on during sleep").down(resourceId="android:id/summary").text == u'Always', \
            "[ERROR]: wifi is not always on"
        self.api.clean_tasks()
        self.api.change_global_spinner_settings(self.api.ui.global_spinner_wifi)
        self.api.click_with_timeout("text", "default")
        self.api.click_with_timeout("text", "OK")

    def testSetUserRestrictions_Disallow_config_bluetooth(self):
        """
        disallow config bluetooth for device owner
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.BLUETOOTH_SETTINGS", False)
        assert self.api.check_ui_exists("text", "Bluetooth"), "[ERROR]: fail to launch bluetooth settings"
        for _ in range(10):
            if self.api.check_ui_exists("text", "Off"):
                self.api.d(text="Off").right(resourceId="com.android.settings:id/switch_widget").click()
            if self.api.check_ui_exists("text", "Available devices"):
                break
            time.sleep(5)
        assert self.api.check_ui_exists("text", "Available devices"), "[ERROR]: fail to check available device string"
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_config_bluetooth + "OFF", False)
            self.api.launch_app_by_intents("android.settings.BLUETOOTH_SETTINGS", False)
            assert self.api.check_ui_exists("textContains", "have permission"), \
                "[ERROR]: fail to disallow config bluetooth"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_config_bluetooth + "ON", False)

    def testSetUserRestrictions_Disallow_adjust_volume(self):
        """
         disallow adjust volume for device owner
        :return: None
        """
        if self.api.is_android_L_build():
            volume_first = self.api.get_notification_volume_by_dumpsys()
            if volume_first > 0:
                self.api.d.press.volume_down()
                self.api.d.press.volume_down()
                self.api.d.press.volume_down()
                time.sleep(2)
            volume_second = self.api.get_notification_volume_by_dumpsys()
            assert volume_first >= volume_second, "[ERROR]: fail to lower the volume of notification"
            try:
                self.api.set_user_restrictions(self.api.ui.disallow_adjust_volume + "OFF", False)
                time.sleep(2)
                self.api.d.press.volume_up()
                self.api.d.press.volume_up()
                self.api.d.press.volume_up()
                volume_third = self.api.get_notification_volume_by_dumpsys()
                assert volume_third == volume_second, "[ERROR]: fail to disallow adjust volume"
            finally:
                self.api.d.press.volume_up()
                self.api.clean_tasks()
                self.api.set_user_restrictions(self.api.ui.disallow_adjust_volume + "ON", False)
        else:
            self.api.launch_app_by_intents("android.settings.SOUND_SETTINGS", False)
            if not self.api.check_ui_exists("text", "Sound & notification", 5):
                self.api.clean_tasks()
                self.api.settings_sub_launch("Sound & notification")
            if self.api.check_ui_exists("text", "Ring volume"):
                assert self.api.check_ui_exists("text", "Ring volume"), "[ERROR]: fail to detect ring volume"
                assert self.api.d(text="Ring volume").enabled, "[ERROR]: ring volume already disabled"
            else:
                assert self.api.check_ui_exists("text", "Notification volume"), \
                    "[ERROR]: fail to detect Notification volume"
                assert self.api.d(text="Notification volume").enabled, "[ERROR]: Notification volume already disabled"
            try:
                self.api.set_user_restrictions(self.api.ui.disallow_adjust_volume + "OFF", False)
                self.api.launch_app_by_intents("android.settings.SOUND_SETTINGS", False)
                if not self.api.check_ui_exists("text", "Sound & notification", 5):
                    self.api.clean_tasks()
                    self.api.settings_sub_launch("Sound & notification")
                if self.api.check_ui_exists("text", "Ring volume"):
                    assert self.api.check_ui_exists("text", "Ring volume"), "[ERROR]: fail to detect ring volume"
                    assert not self.api.d(text="Ring volume").enabled, "[ERROR]: ring volume still enabled"
                else:
                    assert self.api.check_ui_exists("text", "Notification volume"), \
                        "[ERROR]: fail to detect Notification volume"
                    assert not self.api.d(text="Notification volume").enabled, \
                        "[ERROR]: Notification volume still enabled"
            finally:
                self.api.clean_tasks()
                self.api.set_user_restrictions(self.api.ui.disallow_adjust_volume + "ON", False)

    def testSetUserRestrictions_Disallow_create_windows(self):
        """
         disallow create window for device owner
        :return: None
        """
        igas_comp = igascomparator()
        if self.api.is_android_L_build():
            self.api.api_demo_launch()
            self.api.click_with_timeout("resourceId", self.api.ui.intents_sharing)
            self.api.click_with_timeout("resourceId", self.api.ui.add_persistent_preferred)
            self.api.click_with_timeout("text", "Copy to clipboard")
            self.api.click_with_timeout("text", "Pick APP")
        else:
            if not self.api.launch_app_by_activity(self.api.ui.app_schema, False):
                self.api.launch_app("AppRestrictionSchema")
        box = (0, (self.api.d.info["displayHeight"]*3)/4,
               self.api.d.info["displayWidth"], self.api.d.info["displayHeight"])
        temp_path = getTmpDir()
        self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
        image = Image.open(os.path.join(temp_path, "screenshot.png"))
        image.crop(box).save(os.path.join(temp_path, "not_trigger.png"))
        os.remove(os.path.join(temp_path, "screenshot.png"))
        assert os.path.isfile(os.path.join(temp_path, "not_trigger.png")), "[ERROR]: fail to save not_trigger.png"
        result = 1
        for _ in range(5):
            if self.api.is_android_L_build():
                for i in range(5):
                    self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_handle)
            else:
                assert self.api.check_ui_exists("resourceId", "com.example.android.apprestrictionschema:id/request_configuration"), \
                    "[ERROR]: fail to detect request restrictions provider button"
                button_bounds = self.api.d(
                    resourceId="com.example.android.apprestrictionschema:id/request_configuration").bounds
                x = (button_bounds["left"] + button_bounds["right"]) / 2
                y = (button_bounds["top"] + button_bounds["bottom"]) / 2
                for i in range(5):
                    os.popen("adb -s {0} shell input tap {1} {2}".format(self.api.serial, x, y)).read()
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "trigger_toast.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            assert os.path.isfile(os.path.join(temp_path, "trigger_toast.png")), \
                "[ERROR]: fail to save trigger_toast.png"
            result = float(igas_comp.getsimilarityrate(os.path.join(temp_path, "not_trigger.png"),
                                                       os.path.join(temp_path, "trigger_toast.png")))
            if result < 1.0:
                break
            time.sleep(2)
        assert result < 1.0, "[ERROR]: fail to capture screen with toast message"
        self.api.clean_tasks()
        self.api.set_user_restrictions("disallow create windows OFF", False)
        if self.api.is_android_L_build():
            self.api.d.press.back()
            self.api.click_with_timeout("resourceId", self.api.ui.intents_sharing)
        else:
            if not self.api.launch_app_by_activity(self.api.ui.app_schema, False):
                self.api.launch_app("AppRestrictionSchema")
        result = 0
        for _ in range(5):
            if self.api.is_android_L_build():
                for i in range(5):
                    self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_handle)
            else:
                assert self.api.check_ui_exists("resourceId", "com.example.android.apprestrictionschema:id/request_configuration"), \
                    "[ERROR]: fail to detect request restrictions provider button"
                button_bounds = self.api.d(
                    resourceId="com.example.android.apprestrictionschema:id/request_configuration").bounds
                x = (button_bounds["left"] + button_bounds["right"]) / 2
                y = (button_bounds["top"] + button_bounds["bottom"]) / 2
                for i in range(5):
                    os.popen("adb -s {0} shell input tap {1} {2}".format(self.api.serial, x, y)).read()
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "trigger_without_toast.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            assert os.path.isfile(os.path.join(temp_path, "trigger_without_toast.png")), \
                "[ERROR]: fail to save trigger_without_toast.png"
            result = float(igas_comp.getsimilarityrate(os.path.join(temp_path, "not_trigger.png"),
                                                       os.path.join(temp_path, "trigger_without_toast.png")))
            if result == 1.0:
                break
            time.sleep(2)
        assert result == 1.0, "[ERROR]: fail to disallow create window"
        if self.api.is_android_L_build():
            self.api.click_with_timeout("resourceId", self.api.ui.clear_persistent_preferred)
            self.api.click_with_timeout("text", "OK")
        self.api.clean_tasks()
        self.api.set_user_restrictions("disallow create windows ON", False)

    def testSetUserRestrictions_Disallow_cross_profile_CopyPaste(self):
        """
         disallow cross copy paste for device owner
        :return: None
        """
        if "afwHandleChromeInvitedWindow" not in self.api.d.watchers:
            self.api.d.watcher("afwHandleChromeInvitedWindow").when(
                text="No Thanks").click(text="No Thanks")
        # enable system applications under device owner if Chrome doesn't exist
        if not self.api.locate_apps("Chrome"):
            self.api.enable_system_applications(False)
            self.api.clean_tasks()
        # create manage profile if needed
        if not self.api.locate_apps("Work Sample MDM"):
            self.api.setup_managed_profile(False)
            self.api.clean_tasks()
        # enable system applications under managed profile if Work Chrome doesn't exist
        if not self.api.locate_apps("Work Chrome"):
            self.api.enable_system_applications()
            self.api.clean_tasks()
        # handle Welcome page for Chrome
        # self.api.launch_app("Chrome")
        if not self.api.launch_app_by_activity(self.api.ui.chrome, False):
            self.api.launch_app("Chrome")
        for _ in range(5):
            self.api.unlock_screen()
            if self.api.check_ui_exists("text", "Welcome to Chrome"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/terms_accept")
                self.api.click_with_timeout("text", "Next")
                self.api.click_with_timeout("text", "No thanks")
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/got_it_button")
                break
        # handle Welcome page for Work Chrome
        # self.api.launch_app("Work Chrome")
        if not self.api.launch_app_by_activity(self.api.ui.chrome, True):
            self.api.launch_app("Work Chrome")
        for _ in range(5):
            self.api.unlock_screen()
            if self.api.check_ui_exists("text", "Welcome to Chrome"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/terms_accept")
                self.api.click_with_timeout("text", "Next")
                self.api.click_with_timeout("text", "No thanks")
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/got_it_button")
                break
        self.api.clean_tasks()
        # self.api.launch_app("Chrome")
        if not self.api.launch_app_by_activity(self.api.ui.chrome, False):
            self.api.launch_app("Chrome")
        # assert self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar", 10), \
        #     "[ERROR]: fail to detect url bar for chrome"
        assert self.api.check_ui_exists("packageName", "com.android.chrome", 10), "fail to detect chrome"
        time.sleep(5)
        for i in range(3):
            if self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar"):
                self.api.d(resourceId="com.android.chrome:id/url_bar").set_text("DeviceOwner")
            else:
                self.api.d(resourceId="com.android.chrome:id/search_box_text").set_text("DeviceOwner")
            time.sleep(3)
            # self.api.d(resourceId="com.android.chrome:id/url_bar").long_click.topleft()
            self.api.d.press(0x1d, 0x7000)  # select all by Ctrl + A
            self.api.d.press(0x1f, 0x7000)  # copy content by key event Ctrl + C
        if not self.api.launch_app_by_activity(self.api.ui.chrome, True):
            self.api.launch_app("Work Chrome")
        # assert self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar", 10), \
        #     "[ERROR]: fail to detect url bar for work chrome"
        assert self.api.check_ui_exists("packageName", "com.android.chrome", 10), "fail to detect work chrome"
        time.sleep(5)
        managed_profile = False
        for i in range(3):
            if self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar"):
                self.api.d(resourceId="com.android.chrome:id/url_bar").clear_text()
            else:
                self.api.d(resourceId="com.android.chrome:id/search_box_text").clear_text()
                self.api.d(resourceId="com.android.chrome:id/search_box_text").set_text("hello")
            time.sleep(3)
            self.api.d.press(0x32, 0x7000)  # paste content by key event Ctrl + V
            if self.api.check_ui_exists("textContains", "DeviceOwner"):
                managed_profile = True
        assert managed_profile, "[ERROR]: unable to paste content from DO to PO"

        # unable to copy content from managed profile to primary user
        self.api.clean_tasks()
        # set restrictions
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_cross_copy_paste + "OFF", False)
            # self.api.launch_app("Chrome")
            if not self.api.launch_app_by_activity(self.api.ui.chrome, False):
                self.api.launch_app("Chrome")
            # assert self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar", 10), \
            #     "[ERROR]: fail to detect url bar for chrome"
            assert self.api.check_ui_exists("packageName", "com.android.chrome", 10), "fail to detect chrome"
            time.sleep(5)
            for i in range(3):
                if self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar"):
                    self.api.d(resourceId="com.android.chrome:id/url_bar").set_text("DeviceOwner")
                else:
                    self.api.d(resourceId="com.android.chrome:id/search_box_text").set_text("DeviceOwner")
                time.sleep(3)
                # self.api.d(resourceId="com.android.chrome:id/url_bar").long_click.topleft()
                self.api.d.press(0x1d, 0x7000)  # select all by Ctrl + A
                self.api.d.press(0x1f, 0x7000)  # copy content by key event Ctrl + C
            if not self.api.launch_app_by_activity(self.api.ui.chrome, True):
                self.api.launch_app("Work Chrome")
            # assert self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar", 10), \
            #     "[ERROR]: fail to detect url bar for work chrome"
            assert self.api.check_ui_exists("packageName", "com.android.chrome", 10), "fail to detect work chrome"
            time.sleep(5)
            managed_profile = False
            for i in range(3):
                if self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar"):
                    self.api.d(resourceId="com.android.chrome:id/url_bar").clear_text()
                else:
                    self.api.d(resourceId="com.android.chrome:id/search_box_text").clear_text()
                    self.api.d(resourceId="com.android.chrome:id/search_box_text").set_text("hello")
                time.sleep(3)
                self.api.d.press(0x32, 0x7000)  # paste content by key event Ctrl + V
                if self.api.check_ui_exists("textContains", "DeviceOwner"):
                    managed_profile = True
            assert not managed_profile, "[ERROR]: able to paste content from DO to PO"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_cross_copy_paste + "ON", False)
            self.api.clean_tasks()
            self.api.remove_managed_profile(False)
            if "afwHandleChromeInvitedWindow" in self.api.d.watchers:
                self.api.d.watchers.remove("afwHandleChromeInvitedWindow")

    def testSetGlobalOrSecureSettings_Skip_first_use_hints(self):
        """
        skip first hints for device owner
        :return: None
        """
        launcher_package = self.api.download_file_from_artifactory(self.api.remote.app_launcher['sub_path'],
                                                                   self.api.remote.app_launcher['name'])
        assert launcher_package is not None, "[ERROR]: fail to download Launcher"
        try:
            old_dir = os.getcwd()
            os.chdir(os.path.split(launcher_package)[0])
            self.api.install_apps(self.api.remote.app_launcher['pkg_name'],
                                  self.api.remote.app_launcher['name'].split('.apk')[0])
            os.chdir(old_dir)
            # switch to new installed launcher
            self.api.d.press.home()
            if self.api.check_ui_exists("text", "Use Google Now Launcher as Home"):
                self.api.click_with_timeout("text", "Launcher")
            elif self.api.check_ui_exists("text", "Use Launcher as Home"):
                self.api.click_with_timeout("text", "Just once")
            else:
                self.api.click_with_timeout("text", "Launcher")
                self.api.click_with_timeout("text", "Just once")
            time.sleep(10)
            # hints must be existed at this time
            assert self.api.check_ui_exists("text", "Make yourself at home"), "[ERROR]: hints doesn't exist"
            assert self.api.check_ui_exists("text", "To see all your apps, touch the circle."), \
                "[ERROR]: hints doesn't exist"
            # switch to google now launcher
            self.api.d.press.home()
            if self.api.check_ui_exists("text", "Use Launcher as Home"):
                self.api.click_with_timeout("text", "Google Now Launcher")
            elif self.api.check_ui_exists("text", "Use Google Now Launcher as Home"):
                self.api.click_with_timeout("text", "Just once")
            else:
                self.api.click_with_timeout("text", "Google Now Launcher")
                self.api.click_with_timeout("text", "Just once")
            self.api.uninstall_apps_by_adb(self.api.remote.app_launcher['pkg_name'])
            self.api.set_global_secure_settings(self.api.ui.global_skip_first_hint + "OFF")
            old_dir = os.getcwd()
            os.chdir(os.path.split(launcher_package)[0])
            self.api.install_apps(self.api.remote.app_launcher['pkg_name'],
                                  self.api.remote.app_launcher['name'].split('.apk')[0])
            os.chdir(old_dir)
            self.api.d.press.home()
            if self.api.check_ui_exists("text", "Use Google Now Launcher as Home"):
                self.api.click_with_timeout("text", "Launcher")
            elif self.api.check_ui_exists("text", "Use Launcher as Home"):
                self.api.click_with_timeout("text", "Just once")
            else:
                self.api.click_with_timeout("text", "Launcher")
                self.api.click_with_timeout("text", "Just once")
            time.sleep(5)
            assert not self.api.check_ui_exists("text", "Make yourself at home"), "[ERROR]: hints still exist"
            assert not self.api.check_ui_exists("text", "To see all your apps, touch the circle."), \
                "[ERROR]: hints still exist"
            # switch to google now launcher
            self.api.d.press.home()
            if self.api.check_ui_exists("text", "Use Launcher as Home"):
                self.api.click_with_timeout("text", "Google Now Launcher")
            elif self.api.check_ui_exists("text", "Use Google Now Launcher as Home"):
                self.api.click_with_timeout("text", "Always")
            else:
                self.api.click_with_timeout("text", "Google Now Launcher")
                self.api.click_with_timeout("text", "Always")
        finally:
            self.api.uninstall_apps_by_adb(self.api.remote.app_launcher['pkg_name'])
            self.api.clean_tasks()
            self.api.set_global_secure_settings(self.api.ui.global_skip_first_hint + "ON")

    def testEnable_Pre_Populated_Email_Address(self):
        """
        populate email address for device owner
        :return: None
        """
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.device_management_policy)
        self.api.click_with_timeout("resourceId", self.api.ui.pre_populated_email)
        assert self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/content_edit", 10), \
            "[ERROR]: input box for populate email address doesn't exist"
        self.api.d(resourceId="com.intel.afw.mdm:id/content_edit").set_text("mzhou")
        self.api.click_with_timeout("text", "OK")
        if self.api.check_ui_exists("resourceId", "com.google.android.gms:id/title", 30):
            assert self.api.check_ui_exists("resourceId", "com.google.android.gms:id/title"), \
                "[ERROR]: fail to check account page"
        elif self.api.check_ui_exists("resourceId", "com.google.android.gms:id/auth_setup_wizard_navbar_next"):
            assert self.api.check_ui_exists("resourceId", "com.google.android.gms:id/auth_setup_wizard_navbar_next"), \
                "[ERROR]: accout login page doesn't pop up"

    def testAutoAcceptSystemUpdate(self):
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.device_management_policy)
        if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/btnSetSystemUpdateAutoInstallPolicy"):
            self.api.d(scrollable=True).scroll.vert.to(
                resourceId="com.intel.afw.mdm:id/btnSetSystemUpdateAutoInstallPolicy")
        self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetSystemUpdateAutoInstallPolicy")
        self.api.click_with_timeout("text", "OK")
        if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/btnGetSystemUpdateAutoInstallPolicy"):
            self.api.d(scrollable=True).scroll.vert.to(
                resourceId="com.intel.afw.mdm:id/btnGetSystemUpdateAutoInstallPolicy")
        get_auto_policy = False
        for _ in range(3):
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnGetSystemUpdateAutoInstallPolicy")
            if self.api.check_ui_exists("text", "get SystemUpdateAutoInstallPolicy success"):
                get_auto_policy = True
                self.api.click_with_timeout("text", "OK")
                break
        assert get_auto_policy, "fail to set auto policy for system update"

    def testAutoPostponeSystemUpdate(self):
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.device_management_policy)
        if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/btnSetSystemUpdatePostponeInstallPolicy"):
            self.api.d(scrollable=True).scroll.vert.to(
                resourceId="com.intel.afw.mdm:id/btnSetSystemUpdatePostponeInstallPolicy")
        self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetSystemUpdatePostponeInstallPolicy")
        self.api.click_with_timeout("text", "OK")
        if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/btnGetSystemUpdatePostponeInstallPolicy"):
            self.api.d(scrollable=True).scroll.vert.to(
                resourceId="com.intel.afw.mdm:id/btnGetSystemUpdatePostponeInstallPolicy")
        get_auto_policy = False
        for _ in range(3):
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnGetSystemUpdatePostponeInstallPolicy")
            if self.api.check_ui_exists("text", "get setSystemUpdatePostponeInstallPolicy success"):
                get_auto_policy = True
                self.api.click_with_timeout("text", "OK")
                break
        assert get_auto_policy, "fail to postpone system update"

    def testSetUserRestrictions_Disallow_outgoing_calls(self):
        """
        verify disallow call dialed out
        :return: None
        """
        if not self.api.locate_apps("Phone"):
            self.api.enable_system_applications(False)
        self.api.launch_app_by_activity("com.android.dialer/com.android.dialer.DialtactsActivity", False)
        if not self.api.check_ui_exists("packageName", "com.android.dialer"):
            self.api.launch_app("Phone")
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/floating_action_button")
        self.api.d(resourceId="com.android.dialer:id/one").wait.exists(timeout=5000)
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/one")
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/zero")
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/zero")
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/eight")
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/six")
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/dialpad_floating_action_button")
        assert not self.api.check_ui_exists("text", "Only emergency calls are allowed by the device owner."), \
            "Only emergency calls are allowed by the device owner"
        if self.api.check_ui_exists("text", "Cellular network not available."):
            self.api.click_with_timeout("text", "OK")
        self.api.set_user_restrictions("disallow outgoing calls OFF", False)
        self.api.launch_app_by_activity("com.android.dialer/com.android.dialer.DialtactsActivity", False)
        if not self.api.check_ui_exists("packageName", "com.android.dialer"):
            self.api.launch_app("Phone")
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/floating_action_button")
        self.api.d(resourceId="com.android.dialer:id/one").wait.exists(timeout=5000)
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/one")
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/zero")
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/zero")
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/eight")
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/six")
        self.api.click_with_timeout("resourceId", "com.android.dialer:id/dialpad_floating_action_button")
        try:
            assert self.api.check_ui_exists("text", "Only emergency calls are allowed by the device owner."), \
                "fail to detect Only emergency calls are allowed by the device owner"
            self.api.click_with_timeout("text", "OK")
        finally:
            self.api.set_user_restrictions("disallow outgoing calls ON", False)

    def testSetUserRestrictions_Disallow_SMS(self):
        """
        verify disallow to use sms app
        :return: None
        """
        if not self.api.locate_apps("Messenger"):
            self.api.enable_system_applications(False)
        self.api.launch_app_by_activity(
            "com.google.android.apps.messaging/com.google.android.apps.messaging.ui.ConversationListActivity", False)
        if not self.api.check_ui_exists("packageName", "com.google.android.apps.messaging"):
            self.api.launch_app("Messenger")
        self.api.click_with_timeout("text", "Skip")
        assert not self.api.check_ui_exists("text", "This app isn't allowed by the device owner."), \
            "This app isn't allowed by the device owner."
        self.api.set_user_restrictions("disallow sms OFF", False)
        self.api.launch_app_by_activity(
            "com.google.android.apps.messaging/com.google.android.apps.messaging.ui.ConversationListActivity", False)
        if not self.api.check_ui_exists("packageName", "com.google.android.apps.messaging"):
            self.api.launch_app("Messenger")
        self.api.click_with_timeout("text", "Skip")
        try:
            assert self.api.check_ui_exists("text", "This app isn't allowed by the device owner."), \
                "fail to detect This app isn't allowed by the device owner."
            self.api.click_with_timeout("text", "OK")
        finally:
            self.api.set_user_restrictions("disallow sms ON", False)

    def testSetUserRestrictions_Disallow_config_cell_broadcasts(self):
        self.api.set_user_restrictions("disallow config cell broadcasts ON", False)
        self.api.settings_sub_launch("More")
        self.api.d(text="More").wait.exists(timeout=5000)
        assert self.api.check_ui_exists("text", "More"), "fail to launch Settings/More"
        assert self.api.check_ui_exists("text", "Emergency broadcasts"), "fail to detect Emergency broadcatsts"
        self.api.set_user_restrictions("disallow config cell broadcasts OFF", False)
        self.api.settings_sub_launch("More")
        self.api.d(text="More").wait.exists(timeout=5000)
        try:
            assert self.api.check_ui_exists("text", "More"), "fail to launch Settings/More"
            assert not self.api.check_ui_exists("text", "Emergency broadcasts"), "still detect Emergency broadcasts"
        finally:
            self.api.set_user_restrictions("disallow config cell broadcasts ON", False)

    def testSetUserRestrictions_Disallow_config_mobile_networks(self):
        self.api.set_user_restrictions("disallow config mobile networks ON", False)
        self.api.settings_sub_launch("More")
        self.api.d(text="More").wait.exists(timeout=5000)
        assert self.api.check_ui_exists("text", "More"), "fail to launch Settings/More"
        assert self.api.check_ui_exists("text", "Cellular networks"), "fail to detect Cellular networks"
        assert self.api.check_ui_exists("text", "Mobile plan"), "fail to detect Mobile plan"
        self.api.set_user_restrictions("disallow config mobile networks OFF", False)
        self.api.settings_sub_launch("More")
        self.api.d(text="More").wait.exists(timeout=5000)
        try:
            assert self.api.check_ui_exists("text", "More"), "fail to launch Settings/More"
            assert not self.api.check_ui_exists("text", "Cellular networks"), "still detect Cellular networks"
            assert not self.api.check_ui_exists("text", "Mobile plan"), "still detect Mobile plan"
        finally:
            self.api.set_user_restrictions("disallow config mobile networks ON", False)

    def testSetGlobalOrSecureSettings_Data_roaming_enable(self):
        self.api.set_global_secure_settings("data roaming OFF")
        self.api.settings_sub_launch("More")
        self.api.d(text="More").wait.exists(timeout=5000)
        assert self.api.check_ui_exists("text", "More"), "fail to launch Settings/More"
        self.api.click_with_timeout("text", "Cellular networks")
        self.api.d(text="Data roaming").wait.exists(timeout=5000)
        assert self.api.d(text="Data roaming").right(resourceId="android:id/switchWidget").checked, \
            "data roaming is OFF"
        self.api.set_global_secure_settings("data roaming ON")
        self.api.settings_sub_launch("More")
        self.api.d(text="More").wait.exists(timeout=5000)
        assert self.api.check_ui_exists("text", "More"), "fail to launch Settings/More"
        self.api.click_with_timeout("text", "Cellular networks")
        self.api.d(text="Data roaming").wait.exists(timeout=5000)
        try:
            assert not self.api.d(text="Data roaming").right(resourceId="android:id/switchWidget").checked, \
                "data roaming is ON"
        finally:
            self.api.set_global_secure_settings("data roaming OFF")


class VPNCertificateManagement(UIATestBase):
    """
    @summary: Test cases for VPN certificate management under device owner
    """

    def setUp(self):
        super(VPNCertificateManagement, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(VPNCertificateManagement, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSetCertInstallerPackage(self):
        """
        allow 3rd party cert app to install cert file
        :return: None
        """
        self.api.uninstall_apps_by_adb("com.intel.afw.certsInstallation")
        cert_3rd_apk = self.api.download_file_from_artifactory("AfW/Apks/", "Cert_Verification_M.apk")
        assert cert_3rd_apk is not None, "[ERROR]: fail to download 3rd cert apk"
        old_dir = os.getcwd()
        os.chdir(os.path.split(cert_3rd_apk)[0])
        self.api.install_apps(self.api.remote.cert_3rd_m['pkg_name'],
                              self.api.remote.cert_3rd_m['name'].split('.apk')[0])
        os.chdir(old_dir)
        for i in range(5):
            ret = os.popen("adb -s {0} shell pm list packages com.intel.afw.certsInstallation".format(
                self.api.serial)).read().strip()
            if ret.find("com.intel.afw.certsInstallation") != -1:
                break
        if not self.api.locate_apps("Certs Management"):
            g_common_obj2.system_reboot(90)
            for i in range(20):
                self.api.d = g_common_obj.get_device()
                self.api.d.wakeup()
                if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    break
                time.sleep(5)
            g_common_obj.set_vertical_screen()
            self.api.unlock_screen()
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.vpn_management)
        for _ in range(3):
            if self.api.check_ui_exists("textContains", "com.intel.afw.certsInstallation will be gaven access to"):
                break
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetCertInstallerPackage")
        assert self.api.check_ui_exists("textContains", "com.intel.afw.certsInstallation will be gaven access to"), \
            "fail to give access to 3rd party app"
        self.api.click_with_timeout("text", "OK")
        if self.api.launch_app_by_activity(
                "com.intel.afw.certsInstallation/com.intel.afw.certsInstallation.MainWindow", False):
            self.api.launch_app("Certs Management")
        for _ in range(3):
            if self.api.check_ui_exists("text", "Select All"):
                break
            self.api.click_with_timeout("resourceId", "com.intel.afw.certsInstallation:id/btnInstallCaCert")
        self.api.click_with_timeout("text", "Select All", 5)
        self.api.click_with_timeout("text", "Install CA Cert")
        self.api.click_with_timeout("resourceId", "com.intel.afw.certsInstallation:id/btnGetInstalledCaCerts", 5)
        assert self.api.check_ui_exists("text", "mytestcert2.cer"), "fail to install cert"

    def testGetCertInstallerPackage(self):
        """
        get allowed 3rd party cert app
        :return: None
        """
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.vpn_management)
        for _ in range(3):
            if self.api.check_ui_exists("textContains", "com.intel.afw.certsInstallation had been gaven access to"):
                break
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnGetCertInstallerPackage")
        assert self.api.check_ui_exists("textContains", "com.intel.afw.certsInstallation had been gaven access to"), \
            "fail to get allowed 3rd app"
        self.api.click_with_timeout("text", "OK")

    def testSilentlyAccessCert(self):
        """
        verify that able to access cert silently
        :return: None
        """
        self.api.set_lock_pin()
        if self.api.launch_app_by_activity(
                "com.intel.afw.certsInstallation/com.intel.afw.certsInstallation.MainWindow", False):
            self.api.launch_app("Certs Management")
        self.api.click_with_timeout("resourceId", "com.intel.afw.certsInstallation:id/btnInstallKeyPair")
        self.api.click_with_timeout("text", "OK")
        self.api.click_with_timeout("resourceId", "com.intel.afw.certsInstallation:id/btnSilentCertAccess")
        self.api.d(resourceId="com.intel.afw.certsInstallation:id/content_edit").set_text(
            "com.intel.afw.certsInstallation")
        self.api.click_with_timeout("text", "OK")
        if self.api.check_ui_exists("text", "Choose certificate", 5):
            assert self.api.check_ui_exists("resourceId", "com.android.keychain:id/cert_item_selected"), \
                "fail to detect selector"
            assert self.api.d(resourceId="com.android.keychain:id/cert_item_alias").right(
                resourceId="com.android.keychain:id/cert_item_selected").checked, "fail to access cert silently"
        else:
            assert not self.api.check_ui_exists("text", "Choose certificate"), "fail to access cert silently"
        self.api.set_lock_swipe()

    def testClearCertInstallerPackage(self):
        """
        clear allowed 3rd party cert app
        :return: None
        """

        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.vpn_management)
        for _ in range(3):
            if self.api.check_ui_exists("textContains", "com.intel.afw.certsInstallation will not be gaven access to"):
                break
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnClearCertInstallerPackage")
        assert self.api.check_ui_exists("textContains", "com.intel.afw.certsInstallation will not be gaven access to"), \
            "fail to clear allowed 3rd app"
        self.api.click_with_timeout("text", "OK")
        if self.api.launch_app_by_activity(
                "com.intel.afw.certsInstallation/com.intel.afw.certsInstallation.MainWindow", False):
            self.api.launch_app("Certs Management")
        self.api.click_with_timeout("resourceId", "com.intel.afw.certsInstallation:id/btnUninstallCaCert")
        self.api.click_with_timeout("text", "Select All")
        self.api.d.watchers.remove()
        self.api.click_with_timeout("text", "Uninstall CA Cert")
        assert self.api.check_ui_exists("textContains", "Unfortunately"), "fail to clear 3rd cert installer"
        self.api.click_with_timeout("text", "OK")
        self.api.uninstall_apps_by_adb("com.intel.afw.certsInstallation")

    def testInstall_CA_Cert(self):
        """
        install CA Cert for device owner
        :return: None
        """
        if self.api.is_android_L_build():
            cert_file = self.api.download_file_from_artifactory(self.api.remote.cert_file['sub_path'],
                                                                self.api.remote.cert_file['name'])
            assert cert_file is not None, "[ERROR]: fail to download CA Cert file"
            if os.popen("adb -s {0} shell ls /mnt".format(self.api.serial)).read().strip().find('shell') != -1:
                os.popen("adb -s {0} push {1} /mnt/shell/emulated/0/Download/".format(self.api.serial, cert_file))
        self.api.install_ca_cert(False)
        self.api.click_with_timeout("resourceId", self.api.ui.get_installed_ca_cert)
        if self.api.check_ui_exists("text", "key_alpha.cer"):
            assert self.api.check_ui_exists("text", "key_alpha.cer"), "[ERROR]: fail to install key_alpha.cer"
        else:
            assert self.api.check_ui_exists("text", "mytestcert3.cer"), "[ERROR]: fail to install mytestcert3.cer"

    def testUninstall_CA_Cert(self):
        """
        uninstall CA Cert for device owner
        :return: None
        """
        self.api.uninstall_ca_cert(False)
        self.api.click_with_timeout("resourceId", self.api.ui.get_installed_ca_cert)
        assert self.api.check_ui_exists("text", "Attention"), "[ERROR]: fail to uninstall ca cert"

    def testCheck_CA_Cert_Installed(self):
        """
        check CA Cert whether is installed
        :return: None
        """
        self.api.check_ca_cert(False)
        assert self.api.check_ui_exists("textContains", "has been installed"), "[ERROR]: fail to check ca cert"

    def testGet_Installed_CA_Cert(self):
        """
        list all installed CA Cert files
        :return: None
        """
        self.api.get_installed_ca_cert(False)
        if self.api.check_ui_exists("text", "key_alpha.cer"):
            assert self.api.check_ui_exists("text", "key_alpha.cer"), "[ERROR]: fail to get installed key_alpha.cer"
        else:
            assert self.api.check_ui_exists("text", "mytestcert3.cer"), "[ERROR]: fail to get installed mytestcert3.cer"

    def testUninstall_All_User_CA_Certs(self):
        """
        uninstall all installed CA Cert file
        :return:
        """
        self.api.uninstall_all_ca_cert(False)
        self.api.click_with_timeout("resourceId", self.api.ui.get_installed_ca_cert)
        assert self.api.check_ui_exists("text", "Attention", 5), "[ERROR]: fail to uninstall ca cert"

    def testSet_Recommended_Global_Proxy(self):
        """
        set recommended proxy for device owner
        :return: None
        """
        if not self.api.locate_apps("Chrome"):
            self.api.enable_system_applications(False)
        if not self.api.launch_app_by_activity(self.api.ui.chrome, False):
            self.api.launch_app("Chrome")
        self.api.d(packageName="com.android.chrome").wait.exists(timeout=5000)
        if "afwHandleChromeInvitedWindow" not in self.api.d.watchers:
            self.api.d.watcher("afwHandleChromeInvitedWindow").when(
                text="No Thanks").click(text="No Thanks")
        for _ in range(3):
            self.api.unlock_screen()
            if self.api.check_ui_exists("text", "Welcome to Chrome"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/terms_accept")
                self.api.click_with_timeout("text", "Next")
                self.api.click_with_timeout("text", "No thanks")
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/got_it_button")
                break
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.vpn_management)
        self.api.click_with_timeout("resourceId", self.api.ui.set_global_proxy)
        assert self.api.check_ui_exists("resourceId", self.api.ui.proxy_edit), \
            "[ERROR]: proxy settings window doesn't activated"
        if self.api.check_ui_exists("resourceId", self.api.ui.proxy_edit):
            self.api.d(resourceId=self.api.ui.proxy_edit).set_text("proxy.intel.com")
        if self.api.check_ui_exists("resourceId", self.api.ui.proxy_port_edit):
            self.api.d(resourceId=self.api.ui.proxy_port_edit).set_text("999")
        if self.api.check_ui_exists("resourceId", self.api.ui.proxy_exclude_list):
            self.api.d(resourceId=self.api.ui.proxy_exclude_list).set_text("*.intel.com")
        self.api.click_with_timeout("text", "OK")
        self.api.click_with_timeout("text", "OK")
        time.sleep(5)
        # if not self.api.launch_app_by_activity(self.api.ui.chrome, False):
        #     self.api.launch_app("Chrome")
        # if self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar", 5):
        #     self.api.d(resourceId="com.android.chrome:id/url_bar").set_text("chrome://net-internals/#proxy")
        # else:
        #     self.api.d(resourceId="com.android.chrome:id/search_box_text").set_text("chrome://net-internals/#proxy")
        # self.api.d.press.enter()
        # time.sleep(5)
        # result = None
        # for j in range(5):
        #     result = None
        #     if self.api.check_ui_exists("resourceId", "com.android.chrome:id/menu_button"):
        #         self.api.click_with_timeout("resourceId", "com.android.chrome:id/menu_button")
        #     else:
        #         self.api.d.press.menu()
        #     self.api.d(text="Find in page").wait.exists(timeout=3000)
        #     self.api.click_with_timeout("text", "Find in page")
        #     if self.api.check_ui_exists("resourceId", "com.android.chrome:id/find_query"):
        #         self.api.d(resourceId="com.android.chrome:id/find_query").set_text("Proxy server")
        #     if self.api.check_ui_exists("resourceId", "com.android.chrome:id/find_status"):
        #         result = self.api.d(resourceId="com.android.chrome:id/find_status").info["contentDescription"]
        #     if result is not None:
        #         break
        result = None
        for j in range(5):
            result = None
            if not self.api.launch_app_by_activity(self.api.ui.chrome, False):
                self.api.launch_app("Chrome")
            if self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar", 5):
                self.api.d(resourceId="com.android.chrome:id/url_bar").set_text("chrome://net-internals/#proxy")
            else:
                self.api.d(resourceId="com.android.chrome:id/search_box_text").set_text("chrome://net-internals/#proxy")
            self.api.d.press.enter()
            self.api.d.wait.update(timeout=5000)
            if self.api.check_ui_exists("resourceId", "com.android.chrome:id/menu_button"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/menu_button")
            else:
                self.api.d.press.menu()
            self.api.d(text="Find in page").wait.exists(timeout=3000)
            self.api.click_with_timeout("text", "Find in page")
            if self.api.check_ui_exists("resourceId", "com.android.chrome:id/find_query"):
                self.api.d(resourceId="com.android.chrome:id/find_query").set_text("Proxy server")
            if self.api.check_ui_exists("resourceId", "com.android.chrome:id/find_status"):
                result = self.api.d(resourceId="com.android.chrome:id/find_status").info["contentDescription"]
            if result is not None:
                break
            self.api.clean_tasks()
        assert result is not None, "[ERROR]: fail to get content description"
        assert repr(result).find("No results") == -1, "[ERROR]: fail to query DIRECT in chrome page"

    def testClear_Recommended_Global_Proxy(self):
        """
        clear recommended proxy for device owner
        :return: None
        """
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.vpn_management)
        self.api.click_with_timeout("resourceId", self.api.ui.clear_global_proxy)
        assert self.api.check_ui_exists("text", "Clear Recommended Global Proxy"), \
            "[ERROR]: there isn't any proxy available or window doesn't pop up"
        self.api.click_with_timeout("text", "OK")
        # if not self.api.launch_app_by_activity(self.api.ui.chrome, False):
        #     self.api.launch_app("Chrome")
        # self.api.d(packageName="com.android.chrome").wait.exists(timeout=5000)
        # if self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar", 5):
        #     self.api.d(resourceId="com.android.chrome:id/url_bar").set_text("chrome://net-internals/#proxy")
        # else:
        #     self.api.d(resourceId="com.android.chrome:id/search_box_text").set_text("chrome://net-internals/#proxy")
        # self.api.d.press.enter()
        # time.sleep(5)
        # result = None
        # for j in range(5):
        #     result = None
        #     if self.api.check_ui_exists("resourceId", "com.android.chrome:id/menu_button"):
        #         self.api.click_with_timeout("resourceId", "com.android.chrome:id/menu_button")
        #     else:
        #         self.api.d.press.menu()
        #     self.api.click_with_timeout("text", "Find in page")
        #     if self.api.check_ui_exists("resourceId", "com.android.chrome:id/find_query"):
        #         self.api.d(resourceId="com.android.chrome:id/find_query").set_text("DIRECT")
        #     if self.api.check_ui_exists("resourceId", "com.android.chrome:id/find_status"):
        #         result = self.api.d(resourceId="com.android.chrome:id/find_status").info["contentDescription"]
        #     if result is not None:
        #         break
        result = None
        for j in range(5):
            result = None
            if not self.api.launch_app_by_activity(self.api.ui.chrome, False):
                self.api.launch_app("Chrome")
            self.api.d(packageName="com.android.chrome").wait.exists(timeout=5000)
            if self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar", 5):
                self.api.d(resourceId="com.android.chrome:id/url_bar").set_text("chrome://net-internals/#proxy")
            else:
                self.api.d(resourceId="com.android.chrome:id/search_box_text").set_text("chrome://net-internals/#proxy")
            self.api.d.press.enter()
            self.api.d.wait.update(timeout=5000)
            if self.api.check_ui_exists("resourceId", "com.android.chrome:id/menu_button"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/menu_button")
            else:
                self.api.d.press.menu()
            self.api.d(text="Find in page").wait.exists(timeout=3000)
            self.api.click_with_timeout("text", "Find in page")
            if self.api.check_ui_exists("resourceId", "com.android.chrome:id/find_query"):
                self.api.d(resourceId="com.android.chrome:id/find_query").set_text("DIRECT")
            if self.api.check_ui_exists("resourceId", "com.android.chrome:id/find_status"):
                result = self.api.d(resourceId="com.android.chrome:id/find_status").info["contentDescription"]
            if result is not None:
                break
            self.api.clean_tasks()
        assert result is not None, "[ERROR]: fail to get content description"
        assert repr(result).find("No results") == -1, "[ERROR]: fail to query proxy server in chrome page"
        if "afwHandleChromeInvitedWindow" in self.api.d.watchers:
            self.api.d.watcher("afwHandleChromeInvitedWindow").remove()


class LegacyDevicePolicies(UIATestBase):
    """
    @summary: Test cases for Legacy device and policies under device owner
    """

    def setUp(self):
        super(LegacyDevicePolicies, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(LegacyDevicePolicies, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testGET_Password_Info(self):
        """
        get inputting password information in lock screen for device owner
        :return: None
        """
        self.api.set_lock_pin()
        self.api.d.sleep()
        time.sleep(2)
        for _ in range(10):
            self.api.d.wakeup()
            if self.api.check_ui_exists("resourceId", self.api.ui.lock_pin_pad):
                break
            self.api.d(resourceId=self.api.ui.lock_screen_scroll_view).swipe.up()
        # self.api.input_unlock_pin("1233")  #comment this due to OAM-7510, it will block other cases
        self.api.input_unlock_pin("1234")
        self.api.get_password_info(False)
        assert self.api.check_ui_exists("text", "Password relevant info"), "[ERROR]: fail to get password info"
        rel_info = str(self.api.d(resourceId="android:id/message").text).strip().split('\ncurrent')
        # failed_time = int(rel_info[1][-1])
        # assert failed_time == 1, "fail to detect failed times"
        assert rel_info[0].find("Device screen has been successfully unlocked") != 1, "fail to detect successfully info"
        self.api.click_with_timeout("text", "OK")
        self.api.set_lock_swipe()

    def testGet_Set_Camera_State(self):
        """
        set and get camera state for device owner
        :return: None
        """
        self.api.set_get_camera_state(False, False)
        # self.api.d.press("home")
        # self.api.launch_app("Camera")
        if not self.api.launch_app_by_activity("com.android.camera2/com.android.camera.CameraLauncher", False):
            self.api.launch_app("Camera")
        self.api.d(packageName="com.android.camera2").wait.exists(timeout=5000)
        for i in range(5):
            if not self.api.check_ui_exists("resourceId", "com.android.packageinstaller:id/permission_allow_button"):
                break
            self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/permission_allow_button")
        camera_state = False
        for _ in range(5):  # try more times to check
            self.api.click_with_timeout("text", "NEXT")  # remember photo locations page
            if self.api.check_ui_exists("text", "Camera error"):
                camera_state = True
                break
            time.sleep(5)
        assert camera_state, "[ERROR]: fail to disable camera"
        self.api.clean_tasks()
        self.api.set_get_camera_state(True, False)
        # self.api.d.press("home")
        # self.api.launch_app("Camera")
        if not self.api.launch_app_by_activity("com.android.camera2/com.android.camera.CameraLauncher", False):
            self.api.launch_app("Camera")
        self.api.d(packageName="com.android.camera2").wait.exists(timeout=5000)
        self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/permission_allow_button")
        self.api.click_with_timeout("text", "NEXT")
        assert not self.api.check_ui_exists("text", "Camera error"), "[ERROR]: fail to enable camera"

    def testLock_Now(self):
        """
        lock screen in Intel Sample MDM for device owner
        :return: None
        """
        self.api.lock_now_from_mdm(False)
        time.sleep(2)
        self.api.d.wakeup()
        assert self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view), "[ERROR]: fail to lock"

    def testGET_Encryption_State(self):
        """
        get encryption state of device for device owner
        :return: None
        """
        ret = os.popen("adb -s {0} shell getprop ro.crypto.state".format(self.api.serial)).read().strip()
        for _ in range(3):
            if ret == "encrypted":
                break
            ret = os.popen("adb -s {0} shell getprop ro.crypto.state".format(self.api.serial)).read().strip()
        assert ret == "encrypted", "device wasn't encrypted"

    def testSET_Encryption_State(self):
        """
        set encryption state of device for device owner
        just verify that the device is encrypted
        :return: None
        """
        ret = os.popen("adb -s {0} shell getprop ro.crypto.state".format(self.api.serial)).read().strip()
        if ret != "encrypted":
            self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS", False)
            if not self.api.check_ui_exists("text", "Security"):
                self.api.clean_tasks()
                self.api.settings_sub_launch("Security")
            assert self.api.check_ui_exists("text", "Encryption", 5), "Fail to detect Encryption option"
            if self.api.check_ui_exists("text", "Encrypt phone"):
                self.api.click_with_timeout("text", "Encrypt phone")
            else:
                self.api.click_with_timeout("text", "Encrypt tablet")
            assert not self.api.check_ui_exists("resourceId", "com.android.settings:id/warning_low_charge"), \
                "fail to encrypt device due to low battery"
            assert not self.api.check_ui_exists("resourceId", "com.android.settings:id/warning_unplugged"), \
                "fail to encrypt due to need to replug in charger"
            self.api.click_with_timeout("resourceId", "com.android.settings:id/initiate_encrypt")
            self.api.click_with_timeout("resourceId", "com.android.settings:id/execute_encrypt")
            for _ in range(3):
                self.api.d.server.stop()
            time.sleep(600)
            g_common_obj2.system_reboot(90)
            for i in range(20):
                self.api = ApiImpl()
                self.api.d = g_common_obj.get_device()
                self.api.d.wakeup()
                if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    break
                time.sleep(5)
            g_common_obj.set_vertical_screen()
            self.api.unlock_screen()
            ret = os.popen("adb -s {0} shell getprop ro.crypto.state".format(self.api.serial)).read().strip()
        assert ret == "encrypted", "fail to encrypt device"

    def testSetKeyguardDisabledFeatures_donot_show_camera(self):
        """
        don't show camera for device owner
        :return: None
        """
        self.api.set_lock_pin()
        self.api.clean_tasks()
        try:
            self.api.set_keyguard_disabled_features("don't show camera OFF")
            self.api.d.sleep()
            time.sleep(2)
            self.api.d.wakeup()
            assert not self.api.check_ui_exists("resourceId", "com.android.systemui:id/camera_button"), \
                "[ERROR]: camera still exists"
        finally:
            self.api.unlock_screen()
            self.api.clean_tasks()
            self.api.set_lock_swipe()
            self.api.set_keyguard_disabled_features("don't show camera ON")

    def testSetKeyguardDisabledFeatures_donot_show_any_notifications(self):
        """
        don't show any notification for device owner
        :return: None
        """
        self.api.set_lock_pin()
        self.api.clean_tasks()
        try:
            self.api.set_keyguard_disabled_features("don't show any notifications OFF")
            self.api.d.sleep()
            time.sleep(2)
            self.api.d.wakeup()
            assert not self.api.check_ui_exists("text", "USB debugging connected"), "[ERROR]: still show notifications"
        finally:
            self.api.unlock_screen()
            self.api.clean_tasks()
            self.api.set_lock_swipe()
            self.api.set_keyguard_disabled_features("don't show any notifications ON")

    def testSetKeyguardDisabledFeatures_donot_show_unredacted_notifications(self):
        """
        don't show unredacted notification for device owner
        :return: None
        """
        self.api.set_lock_pin()
        self.api.clean_tasks()
        try:
            self.api.set_keyguard_disabled_features("don't show unredacted notifications OFF")
            self.api.launch_app_by_intents(
                "android.intent.action.VIEW -c android.intent.category.BROWSABLE -d "
                "http://mars.androidapksfree.com/files/moon/com.liquidum.hexlock-v1.4.2-25-Android-4.0.3.apk",
                False)
            self.api.d(packageName="com.android.chrome").wait.exists(timeout=5000)
            if self.api.check_ui_exists("text", "Welcome to Chrome"):
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/terms_accept")
                self.api.click_with_timeout("text", "Next")
                self.api.click_with_timeout("text", "No thanks")
                self.api.click_with_timeout("resourceId", "com.android.chrome:id/got_it_button")
            for _ in range(10):
                self.api.click_with_timeout("text", "Update permissions")
                self.api.click_with_timeout("text", "Allow")
                if self.api.check_ui_exists("text", "OK"):
                    break
                time.sleep(3)
            assert self.api.check_ui_exists("text", "OK"), "[ERROR]: fail to download file from internet"
            self.api.click_with_timeout("text", "OK")
            self.api.click_with_timeout("text", "Replace file")
            self.api.d.sleep()
            time.sleep(3)
            self.api.d.wakeup()
            self.api.d.press.back()
            assert self.api.check_ui_exists("text", "Download Manager", 15), \
                "[ERROR]: fail to detect Download Manager in lock screen"
            assert self.api.check_ui_exists("text", "Contents hidden"), \
                "[ERROR]: fail to detect Contents hidden in lock screen"
        finally:
            self.api.unlock_screen()
            self.api.clean_tasks()
            self.api.set_lock_swipe()
            self.api.set_keyguard_disabled_features("don't show unredacted notifications ON")

    def testSetKeyguardDisabledFeatures_disable_trust_agent(self):
        """
        disable trust agent for device owner
        :return: None
        """
        self.api.set_lock_pin()
        self.api.set_keyguard_disabled_features("disable trust agent ON")
        self.api.clean_tasks()
        self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS", False)
        self.api.click_with_timeout("text", "Smart Lock")
        if self.api.check_ui_exists("resourceId", "com.android.settings:id/password_entry"):
            self.api.d(resourceId="com.android.settings:id/password_entry").set_text("1234")
            if self.api.check_ui_exists("text", "Next"):
                self.api.click_with_timeout("text", "Next")
            elif self.api.check_ui_exists("text", "Continue"):
                self.api.click_with_timeout("text", "Continue")
            elif self.api.check_ui_exists("text", "Confirm your PIN"):
                self.api.d.press.enter()
        if self.api.check_ui_exists("text", "Trusted devices"):
            assert self.api.d(text="Trusted devices").enabled, "[ERROR]: trusted devices already disabled by default"
        else:
            assert not self.api.check_ui_exists("text", "Disabled by administrator"), \
                "[ERROR]: Disable by administrator already exists before we disable trust agent"
        try:
            self.api.set_keyguard_disabled_features("disable trust agent OFF")
            self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS", False)
            self.api.click_with_timeout("text", "Smart Lock")
            if self.api.check_ui_exists("resourceId", "com.android.settings:id/password_entry"):
                self.api.d(resourceId="com.android.settings:id/password_entry").set_text("1234")
                if self.api.check_ui_exists("text", "Next"):
                    self.api.click_with_timeout("text", "Next")
                elif self.api.check_ui_exists("text", "Continue"):
                    self.api.click_with_timeout("text", "Continue")
                elif self.api.check_ui_exists("text", "Confirm your PIN"):
                    self.api.d.press.enter()
            if self.api.check_ui_exists("text", "Trusted devices"):
                assert not self.api.d(text="Trusted devices").enabled, "[ERROR]: fail to disable trust agent"
            else:
                assert self.api.check_ui_exists("textContains", "Disabled by administrator"), \
                    "[ERROR]: fail to disable trust agent"
        finally:
            self.api.clean_tasks()
            self.api.set_keyguard_disabled_features("disable trust agent ON")
            self.api.set_lock_swipe()

    def testSetKeyguardDisabledFeatures_disable_features_all(self):
        """
        disable all features for device owner
        :return: None
        """
        self.api.set_lock_pin()
        self.api.clean_tasks()
        try:
            self.api.set_keyguard_disabled_features("disable features all OFF")
            self.api.d.sleep()
            time.sleep(2)
            self.api.d.wakeup()
            assert not self.api.check_ui_exists("text", "USB debugging connected"), "[ERROR]: still show notifications"
            assert not self.api.check_ui_exists("resourceId", "com.android.systemui:id/camera_button"), \
                "[ERROR]: camera still exists"
            self.api.d.sleep()
        finally:
            self.api.unlock_screen()
            self.api.clean_tasks()
            self.api.set_keyguard_disabled_features("disable features all ON")
            self.api.set_lock_swipe()

    def testSet_Password_Expiration_Timeout(self):
        """
        set password expiration time for device owner
        :return: None
        """
        # verify string(Password expired) in "get password info"
        self.api.set_password_expiration_timeout(False)
        time.sleep(75)
        self.api.click_with_timeout("resourceId", self.api.ui.get_password_info)
        assert self.api.check_ui_exists("resourceId", "android:id/message"), "[ERROR]: fail to get password info"
        password_info = self.api.d(resourceId="android:id/message").text.strip()
        self.api.click_with_timeout("text", "OK")
        assert password_info.find("Password expired") != -1, "[ERROR]: fail to set expire timeout for password"

    def testWipe_Data_with_FRP(self):
        """
        wipe data with FRP for device owner
        :return: None
        """
        self.api.wipe_all_data_for_current_user("withFRP", False)
        time.sleep(600)
        self.api.d.wakeup()
        assert self.api.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"), \
            "[ERROR]: fail to check oobe welcome page"

    def testGetDataUsageStatistics(self):
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.device_management_policy)
        if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/btnGetNetworkDataUsage"):
            self.api.d(scrollable=True).scroll.vert.to(resourceId="com.intel.afw.mdm:id/btnGetNetworkDataUsage")
        network_data = False
        for _ in range(3):
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnGetNetworkDataUsage")
            if self.api.check_ui_exists("textContains", "Newtwork Data Usage for current user"):
                network_data = True
                self.api.click_with_timeout("text", "OK")
                break
        assert network_data, "fail to get network data usage"


class ThirdPartyAppService(UIATestBase):
    """
    @summary: Test cases for 3rd party apps and services under device owner
    """

    def setUp(self):
        super(ThirdPartyAppService, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(ThirdPartyAppService, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSet_Permitted_Input_Method(self):
        """
        set permitted input method for device owner
        :return: None
        """
        ime_package = self.api.download_file_from_artifactory(self.api.remote.qq_ime['sub_path'],
                                                              self.api.remote.qq_ime['name'])
        assert ime_package is not None, "[ERROR]: fail to download QQ input method"
        old_dir = os.getcwd()
        os.chdir(os.path.split(ime_package)[0])
        self.api.install_apps(self.api.remote.qq_ime['pkg_name'],
                              self.api.remote.qq_ime['name'].split('.apk')[0])
        os.chdir(old_dir)
        self.api.third_party_app_action("SetInput", False)
        if not self.api.check_ui_exists("text", "com.tencent.qqpinyin", 3):
            g_common_obj2.system_reboot(90)
            for i in range(20):
                self.api = ApiImpl()
                self.api.d = g_common_obj.get_device()
                self.api.d.wakeup()
                if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    break
                time.sleep(5)
            g_common_obj.set_vertical_screen()
            self.api.unlock_screen()
            self.api.third_party_app_action("SetInput", False)
        # assert self.api.d(resourceId="android:id/list").child(index="1").exists, \
        #     "[ERROR]: there is no any input method exists"
        self.api.click_with_timeout("text", self.api.ui.select_all)
        self.api.click_with_timeout("text", "set selected apps")
        assert self.api.check_ui_exists("textContains", "Set successfully with packages"), \
            "[ERROR]: fail to set input method"
        self.api.click_with_timeout("text", "OK")
        self.api.launch_app_by_intents("android.settings.INPUT_METHOD_SETTINGS", False)
        assert self.api.check_ui_exists("text", u"QQ输入法") and self.api.d(text=u"QQ输入法").enabled, \
            "[ERROR]: input method is not exist or disabled"

        self.api.clean_tasks()
        self.api.third_party_app_action("SetInput", False)
        assert self.api.d(resourceId="android:id/list").child(index="1").exists, \
            "[ERROR]: there is no any input method exists"
        self.api.click_with_timeout("text", "set selected apps")
        assert self.api.check_ui_exists("textContains", "Set successfully with empty packages"), \
            "[ERROR]: fail to set with empty packages"
        self.api.click_with_timeout("text", "OK")
        self.api.launch_app_by_intents("android.settings.INPUT_METHOD_SETTINGS", False)
        assert self.api.check_ui_exists("text", u"QQ输入法") and not self.api.d(text=u"QQ输入法").enabled, \
            "[ERROR]: input method doesn't exist or be enabled"

        self.api.clean_tasks()
        self.api.third_party_app_action("SetInput", False)
        assert self.api.d(resourceId="android:id/list").child(index="1").exists, \
            "[ERROR]: there is no any input method exists"
        self.api.click_with_timeout("text", "set null apps")
        assert self.api.check_ui_exists("textContains", "Set successfully with NULL packages"), \
            "[ERROR]: fail to set with NULL packages"
        self.api.click_with_timeout("text", "OK")
        self.api.launch_app_by_intents("android.settings.INPUT_METHOD_SETTINGS", False)
        assert self.api.check_ui_exists("text", u"QQ输入法") and self.api.d(text=u"QQ输入法").enabled, \
            "[ERROR]: input method doesn't exist or be disabled"

    def testGet_Permitted_Input_Method(self):
        """
        get permitted input method for device owner
        :return: None
        """
        self.api.third_party_app_action("SetInput", False)
        assert self.api.d(resourceId="android:id/list").child(index="1").exists, \
            "[ERROR]: there is no any input method exists"
        self.api.click_with_timeout("text", "set selected apps")
        assert self.api.check_ui_exists("textContains", "Set successfully with empty packages"), \
            "[ERROR]: fail to set input method with empty package"
        self.api.click_with_timeout("text", "OK")
        self.api.click_with_timeout("resourceId", self.api.ui.get_input_method)
        assert self.api.check_ui_exists("textContains", "No packages are in whitelist"), \
            "[ERROR]: fail to get input method settings with empty package"
        self.api.click_with_timeout("text", "OK")

        self.api.clean_tasks()
        self.api.third_party_app_action("SetInput", False)
        assert self.api.d(resourceId="android:id/list").child(index="1").exists, \
            "[ERROR]: there is no any input method exists"
        self.api.click_with_timeout("text", "set null apps")
        assert self.api.check_ui_exists("textContains", "Set successfully with NULL packages"), \
            "[ERROR]: fail to set input method with NULL package"
        self.api.click_with_timeout("text", "OK")
        self.api.click_with_timeout("resourceId", self.api.ui.get_input_method)
        assert self.api.check_ui_exists("textContains", "all packages are in whitelist"), \
            "[ERROR]: fail to get input method settings with empty package"
        self.api.click_with_timeout("text", "OK")
        self.api.uninstall_apps_by_adb(self.api.remote.qq_ime['pkg_name'])

    def testSet_Permitted_Accessibility_Services(self):
        """
        set permitted accessibility services for device owner
        :return: None
        """
        test_back_package = self.api.download_file_from_artifactory(self.api.remote.test_back['sub_path'],
                                                                    self.api.remote.test_back['name'])
        assert test_back_package is not None, "[ERROR]: fail to download test back pakcage"
        old_dir = os.getcwd()
        os.chdir(os.path.split(test_back_package)[0])
        self.api.install_apps(self.api.remote.test_back['pkg_name'],
                              self.api.remote.test_back['name'].split('.apk')[0])
        os.chdir(old_dir)
        # sometimes all installed accessibility services will disappear just after install test back package
        # so add code to check and reboot os
        for _ in range(5):  # try to check TestBack service again after reboot
            self.api.clean_tasks()
            self.api.launch_app_by_intents("android.settings.ACCESSIBILITY_SETTINGS", False)
            if self.api.check_ui_exists("text", "TestBack", 5):
                break
            if self.api.check_ui_exists("text", "Services") and \
                    self.api.check_ui_exists("text", "No services installed"):
                # reboot device
                g_common_obj2.system_reboot(90)
                for i in range(20):
                    self.api = ApiImpl()
                    self.api.d = g_common_obj.get_device()
                    self.api.d.wakeup()
                    if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                        break
                    time.sleep(5)
                g_common_obj.set_vertical_screen()
                self.api.unlock_screen()
            if self.api.check_ui_exists("textContains", "isn't responding", 5):
                self.api.click_with_timeout("text", "OK")
        assert self.api.check_ui_exists("text", "TestBack"), "[ERROR]: TestBack still doesn't exist in Accessibility"
        self.api.third_party_app_action("SetAccessibility", False)
        for _ in range(5):  # try more times to verify that foo.bar.testback is detected by Sample MDM
            if self.api.check_ui_exists("text", self.api.remote.test_back['pkg_name'][8:]):
                break
            self.api.d.press.back()
            self.api.click_with_timeout("resourceId", self.api.ui.set_accessibility_service)
        self.api.click_with_timeout("text", self.api.ui.select_all)
        self.api.click_with_timeout("text", "set selected apps")
        assert self.api.check_ui_exists("textContains", "Set successfully with packages"), \
            "[ERROR]: fail to set accessibility service"
        self.api.click_with_timeout("text", "OK")
        self.api.launch_app_by_intents("android.settings.ACCESSIBILITY_SETTINGS", False)
        assert self.api.check_ui_exists("text", "TestBack") and self.api.d(text="TestBack").enabled, \
            "[ERROR]: testback service is not exist or disabled"

        self.api.clean_tasks()
        self.api.third_party_app_action("SetAccessibility", False)
        for _ in range(5):  # try more times to verify that foo.bar.testback is detected by Sample MDM
            if self.api.check_ui_exists("text", self.api.remote.test_back['pkg_name'][8:]):
                break
            self.api.d.press.back()
            self.api.click_with_timeout("resourceId", self.api.ui.set_accessibility_service)
        self.api.click_with_timeout("text", "set selected apps")
        assert self.api.check_ui_exists("textContains", "Set successfully with empty packages"), \
            "[ERROR]: fail to set accessibility service with empty package"
        self.api.click_with_timeout("text", "OK")
        self.api.launch_app_by_intents("android.settings.ACCESSIBILITY_SETTINGS", False)
        assert self.api.check_ui_exists("text", "TestBack") and not self.api.d(text="TestBack").enabled, \
            "[ERROR]: testback service is not exist or be enabled"

        self.api.clean_tasks()
        self.api.third_party_app_action("SetAccessibility", False)
        for _ in range(5):  # try more times to verify that foo.bar.testback is detected by Sample MDM
            if self.api.check_ui_exists("text", self.api.remote.test_back['pkg_name'][8:]):
                break
            self.api.d.press.back()
            self.api.click_with_timeout("resourceId", self.api.ui.set_accessibility_service)
        self.api.click_with_timeout("text", "set null apps")
        assert self.api.check_ui_exists("textContains", "Set successfully with NULL packages"), \
            "[ERROR]: fail to set accessibility service with NULL package"
        self.api.click_with_timeout("text", "OK")
        self.api.launch_app_by_intents("android.settings.ACCESSIBILITY_SETTINGS", False)
        assert self.api.check_ui_exists("text", "TestBack") and self.api.d(text="TestBack").enabled, \
            "[ERROR]: testback service is not exist or disabled"

    def testGet_Permitted_Accessibility_Services(self):
        """
        get permitted accessibility services for device owner
        :return: None
        """
        self.api.third_party_app_action("SetAccessibility", False)
        for _ in range(5):  # try more times to verify that foo.bar.testback is detected by Sample MDM
            if self.api.check_ui_exists("text", self.api.remote.test_back['pkg_name'][8:]):
                break
            self.api.d.press.back()
            self.api.click_with_timeout("resourceId", self.api.ui.set_accessibility_service)
        self.api.click_with_timeout("text", "set selected apps")
        assert self.api.check_ui_exists("textContains", "Set successfully with empty packages"), \
            "[ERROR]: fail to set accessibility service with empty package"
        self.api.click_with_timeout("text", "OK")
        self.api.click_with_timeout("resourceId", self.api.ui.get_accessibility_service)
        assert self.api.check_ui_exists("textContains", "No packages are in whitelist"), \
            "[ERROR]: fail to get accessibility service with empty package"
        self.api.click_with_timeout("text", "OK")

        self.api.clean_tasks()
        self.api.third_party_app_action("SetAccessibility", False)
        for _ in range(5):  # try more times to verify that foo.bar.testback is detected by Sample MDM
            if self.api.check_ui_exists("text", self.api.remote.test_back['pkg_name'][8:]):
                break
            self.api.d.press.back()
            self.api.click_with_timeout("resourceId", self.api.ui.set_accessibility_service)
        self.api.click_with_timeout("text", "set null apps")
        assert self.api.check_ui_exists("textContains", "Set successfully with NULL packages"), \
            "[ERROR]: fail to set accessibility service with NULL package"
        self.api.click_with_timeout("text", "OK")
        self.api.click_with_timeout("resourceId", self.api.ui.get_accessibility_service)
        assert self.api.check_ui_exists("textContains", "all packages are in whitelist"), \
            "[ERROR]: fail to get accessibility service with empty package"
        self.api.click_with_timeout("text", "OK")


class RuntimePermission(UIATestBase):
    """
    @summary: Test cases for 3rd party apps and services under device owner
    """

    def setUp(self):
        super(RuntimePermission, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(RuntimePermission, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    # location
    def testSetDefault_Location_RuntimepermissionState(self):
        self.api.set_runtime_permission("LOCATION", "Default", False)
        enabled, checked = self.api.check_runtime_permission("Location", False)
        assert enabled, "fail to set location to Default"

    def testSetGrant_Location_RuntimepermissionState(self):
        self.api.set_runtime_permission("LOCATION", "Grant", False)
        enabled, checked = self.api.check_runtime_permission("Location", False)
        assert not enabled, "location still enabled"
        assert checked, "fail to set location to Grant"

    def testSetDeny_Location_RuntimepermissionState(self):
        self.api.set_runtime_permission("LOCATION", "Deny", False)
        enabled, checked = self.api.check_runtime_permission("Location", False)
        assert not enabled, "location still enabled"
        assert not checked, "fail to set location to Deny"

    # contact
    def testSetGrant_Contact_RuntimepermissionState(self):
        self.api.uninstall_apps_by_adb("com.example.jizhenlo.runtimepermissiontest")
        self.api.set_runtime_permission("CONTACT", "Grant", False)
        enabled, checked = self.api.check_runtime_permission("Contacts", False)
        assert not enabled, "contact still enabled"
        assert checked, "fail to set contact to Grant"

    def testSetDeny_Contact_RuntimepermissionState(self):
        self.api.set_runtime_permission("CONTACT", "Deny", False)
        enabled, checked = self.api.check_runtime_permission("Contacts", False)
        assert not enabled, "contact still enabled"
        assert not checked, "fail to set contact to Deny"

    def testSetDefault_Contact_RuntimepermissionState(self):
        self.api.set_runtime_permission("CONTACT", "Default", False)
        enabled, checked = self.api.check_runtime_permission("Contacts", False)
        assert enabled, "fail to set contact to Default"

    # camera
    def testSetGrant_Camera_RuntimepermissionState(self):
        self.api.set_runtime_permission("CAMERA", "Grant", False)
        enabled, checked = self.api.check_runtime_permission("Camera", False)
        assert not enabled, "camera still enabled"
        assert checked, "fail to set camera to Grant"

    def testSetDeny_Camera_RuntimepermissionState(self):
        self.api.set_runtime_permission("CAMERA", "Deny", False)
        enabled, checked = self.api.check_runtime_permission("Camera", False)
        assert not enabled, "camera still enabled"
        assert not checked, "fail to set camera to Deny"

    def testSetDefault_Camera_RuntimepermissionState(self):
        self.api.set_runtime_permission("CAMERA", "Default", False)
        enabled, checked = self.api.check_runtime_permission("Camera", False)
        assert enabled, "fail to set camera to Default"

    # calendar
    def testSetGrant_Calendar_RuntimepermissionState(self):
        self.api.set_runtime_permission("CALENDAR", "Grant", False)
        enabled, checked = self.api.check_runtime_permission("Calendar", False)
        assert not enabled, "calendar still enabled"
        assert checked, "fail to set calendar to Grant"

    def testSetDeny_Calendar_RuntimepermissionState(self):
        self.api.set_runtime_permission("CALENDAR", "Deny", False)
        enabled, checked = self.api.check_runtime_permission("Calendar", False)
        assert not enabled, "calendar still enabled"
        assert not checked, "fail to set calendar to Deny"

    def testSetDefault_Calendar_RuntimepermissionState(self):
        self.api.set_runtime_permission("CALENDAR", "Default", False)
        enabled, checked = self.api.check_runtime_permission("Calendar", False)
        assert enabled, "fail to set calendar to Default"

    # audio
    def testSetGrant_Audio_RuntimepermissionState(self):
        self.api.set_runtime_permission("AUDIO", "Grant", False)
        enabled, checked = self.api.check_runtime_permission("Microphone", False)
        assert not enabled, "audio still enabled"
        assert checked, "fail to set audio to Grant"

    def testSetDeny_Audio_RuntimepermissionState(self):
        self.api.set_runtime_permission("AUDIO", "Deny", False)
        enabled, checked = self.api.check_runtime_permission("Microphone", False)
        assert not enabled, "audio still enabled"
        assert not checked, "fail to set audio to Deny"

    def testSetDefault_Audio_RuntimepermissionState(self):
        self.api.set_runtime_permission("AUDIO", "Default", False)
        enabled, checked = self.api.check_runtime_permission("Microphone", False)
        assert enabled, "fail to set audio to Default"

    # phone
    def testSetGrant_Phone_RuntimepermissionState(self):
        self.api.set_runtime_permission("PHONE", "Grant", False)
        enabled, checked = self.api.check_runtime_permission("Phone", False)
        assert not enabled, "phone still enabled"
        assert checked, "fail to set phone to Grant"

    def testSetDeny_Phone_RuntimepermissionState(self):
        self.api.set_runtime_permission("PHONE", "Deny", False)
        enabled, checked = self.api.check_runtime_permission("Phone", False)
        assert not enabled, "Phone still enabled"
        assert not checked, "fail to set phone to Deny"

    def testSetDefault_Phone_RuntimepermissionState(self):
        self.api.set_runtime_permission("PHONE", "Default", False)
        enabled, checked = self.api.check_runtime_permission("Phone", False)
        assert enabled, "fail to set phone to Default"

    # body_sensor
    def testSetGrant_Body_sensor_RuntimepermissionState(self):
        self.api.set_runtime_permission("BODY_SENSOR", "Grant", False)
        enabled, checked = self.api.check_runtime_permission("Body Sensors", False)
        assert not enabled, "body sensors still enabled"
        assert checked, "fail to set body sensors to Grant"

    def testSetDeny_Body_sensor_RuntimepermissionState(self):
        self.api.set_runtime_permission("BODY_SENSOR", "Deny", False)
        enabled, checked = self.api.check_runtime_permission("Body Sensors", False)
        assert not enabled, "body sensors still enabled"
        assert not checked, "fail to set body sensors to Deny"

    def testSetDefault_Body_sensor_RuntimepermissionState(self):
        self.api.set_runtime_permission("BODY_SENSOR", "Default", False)
        enabled, checked = self.api.check_runtime_permission("Body Sensors", False)
        assert enabled, "fail to set body sensors to Default"

    # sms
    def testSetGrant_SMS_RuntimepermissionState(self):
        self.api.set_runtime_permission("SMS", "Grant", False)
        enabled, checked = self.api.check_runtime_permission("SMS", False)
        assert not enabled, "sms still enabled"
        assert checked, "fail to set sms to Grant"

    def testSetDeny_SMS_RuntimepermissionState(self):
        self.api.set_runtime_permission("SMS", "Deny", False)
        enabled, checked = self.api.check_runtime_permission("SMS", False)
        assert not enabled, "sms still enabled"
        assert not checked, "fail to set sms to Deny"

    def testSetDefault_SMS_RuntimepermissionState(self):
        self.api.set_runtime_permission("SMS", "Default", False)
        enabled, checked = self.api.check_runtime_permission("SMS", False)
        assert enabled, "fail to set sms to Default"

    # storage
    def testSetGrant_Storage_RuntimepermissionState(self):
        self.api.set_runtime_permission("STORAGE", "Grant", False)
        enabled, checked = self.api.check_runtime_permission("Storage", False)
        assert not enabled, "storage still enabled"
        assert checked, "fail to set storage to Grant"

    def testSetDeny_Storage_RuntimepermissionState(self):
        self.api.set_runtime_permission("STORAGE", "Deny", False)
        enabled, checked = self.api.check_runtime_permission("Storage", False)
        assert not enabled, "storage still enabled"
        assert not checked, "fail to set storage to Deny"

    def testSetDefault_Storage_RuntimepermissionState(self):
        self.api.set_runtime_permission("STORAGE", "Default", False)
        enabled, checked = self.api.check_runtime_permission("Storage", False)
        assert enabled, "fail to set storage to Default"
        self.api.uninstall_apps_by_adb("com.example.jizhenlo.runtimepermissiontest")


class COSU(UIATestBase):
    """
    @summary: COSU under device owner
    """

    def setUp(self):
        super(COSU, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(COSU, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testAllowScreenOff_Plugged_In(self):
        """
        verify device not stay awake while device is in charging
        :return: None
        """
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.device_management_policy)
        self.api.click_with_timeout("resourceId", self.api.ui.global_setting)
        if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/stay_on_while_plugged_in"):
            self.api.d(scrollable=True).scroll.vert.to(resourceId="com.intel.afw.mdm:id/stay_on_while_plugged_in")
        if self.api.d(resourceId="com.intel.afw.mdm:id/stay_on_while_plugged_in").checked:
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/stay_on_while_plugged_in")
            self.api.click_with_timeout("text", "OK")
        self.api.launch_app_by_intents("android.settings.APPLICATION_DEVELOPMENT_SETTINGS", False)
        if not self.api.check_ui_exists("text", "Stay awake"):
            self.api.d(scrollable=True).scroll.vert.to(text="Stay awake")
        awake_off = False
        for i in range(3):
            if not self.api.d(text="Stay awake").right(resourceId="android:id/switchWidget").checked:
                awake_off = True
                break
        assert awake_off, "fail to turn off stay awake option"

    def testPreventScreenOff_Plugged_In(self):
        """
        verify stay awake while device is in charging
        :return: None
        """
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.device_management_policy)
        self.api.click_with_timeout("resourceId", self.api.ui.global_setting)
        if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/stay_on_while_plugged_in"):
            self.api.d(scrollable=True).scroll.vert.to(resourceId="com.intel.afw.mdm:id/stay_on_while_plugged_in")
        if not self.api.d(resourceId="com.intel.afw.mdm:id/stay_on_while_plugged_in").checked:
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/stay_on_while_plugged_in")
            self.api.click_with_timeout("text", "OK")
        self.api.launch_app_by_intents("android.settings.APPLICATION_DEVELOPMENT_SETTINGS", False)
        if not self.api.check_ui_exists("text", "Stay awake"):
            self.api.d(scrollable=True).scroll.vert.to(text="Stay awake")
        awake_on = False
        for i in range(3):
            if self.api.d(text="Stay awake").right(resourceId="android:id/switchWidget").checked:
                awake_on = True
                break
        assert awake_on, "fail to turn on stay awake option"

    def testDisableKeyguard(self):
        """
        verify disable keyguard in device owner mode
        :return: None
        """
        self.api.set_lock_swipe()
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.device_management_policy)
        if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/btnSetKeyguardState"):
            self.api.d(scrollable=True).scroll.vert.to(resourceId="com.intel.afw.mdm:id/btnSetKeyguardState")
        self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetKeyguardState")
        try:
            self.api.click_with_timeout("text", "Disable")
            self.api.d.sleep()
            time.sleep(2)
            self.api.d.wakeup()
            time.sleep(2)
            self.api.d.press.home()
            assert self.api.check_ui_exists("description", "Apps"), "fail to disable keyguard"
        finally:
            self.api.api_demo_launch()
            if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/btnSetKeyguardState"):
                self.api.clean_tasks()
                self.api.api_demo_launch()
                self.api.click_with_timeout("resourceId", self.api.ui.device_management_policy)
                if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/btnSetKeyguardState"):
                    self.api.d(scrollable=True).scroll.vert.to(resourceId="com.intel.afw.mdm:id/btnSetKeyguardState")
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetKeyguardState")
            self.api.click_with_timeout("text", "Enable")

    def testEnableKeyguard(self):
        """
        verify enable keyguard in device owner
        :return: None
        """
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.device_management_policy)
        if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/btnSetKeyguardState"):
            self.api.d(scrollable=True).scroll.vert.to(resourceId="com.intel.afw.mdm:id/btnSetKeyguardState")
        self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetKeyguardState")
        self.api.click_with_timeout("text", "Enable")
        self.api.d.sleep()
        time.sleep(2)
        self.api.d.wakeup()
        time.sleep(2)
        self.api.d.press.home()
        assert not self.api.check_ui_exists("description", "Apps"), "fail to enable keyguard"

    def testDisable_StatusNotificationBar(self):
        """
        verify disable notification bar in device owner
        :return:
        """
        igas_comp = igascomparator()
        temp_path = getTmpDir()
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.device_management_policy)
        if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/btnSetStatusBarState"):
            self.api.d(scrollable=True).scroll.vert.to(resourceId="com.intel.afw.mdm:id/btnSetStatusBarState")
        x = self.api.d(resourceId="android:id/statusBarBackground").info['visibleBounds']['right'] / 2
        y = self.api.d(resourceId="android:id/statusBarBackground").info['visibleBounds']['bottom']
        box = (0, 0, x, y)
        result = 1.0
        for _ in range(3):
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetStatusBarState")
            self.api.click_with_timeout("text", "Enable")
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "enable_status_bar.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetStatusBarState")
            self.api.click_with_timeout("text", "Disable")
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "disable_status_bar.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            result = float(igas_comp.getsimilarityrate(
                os.path.join(temp_path, "enable_status_bar.png"), os.path.join(temp_path, "disable_status_bar.png")))
            if result < 1.0:
                break
        self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetStatusBarState")
        self.api.click_with_timeout("text", "Enable")
        assert result < 1.0, "fail to disable status notification bar"

    def testEnable_StatusNotificationBar(self):
        """
        verify enable notification bar in device owner
        :return: None
        """
        igas_comp = igascomparator()
        temp_path = getTmpDir()
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.device_management_policy)
        if not self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/btnSetStatusBarState"):
            self.api.d(scrollable=True).scroll.vert.to(resourceId="com.intel.afw.mdm:id/btnSetStatusBarState")
        x = self.api.d(resourceId="android:id/statusBarBackground").info['visibleBounds']['right'] / 2
        y = self.api.d(resourceId="android:id/statusBarBackground").info['visibleBounds']['bottom']
        box = (0, 0, x, y)
        result = 1.0
        for _ in range(3):
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetStatusBarState")
            self.api.click_with_timeout("text", "Disable")
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "disable_status_bar.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetStatusBarState")
            self.api.click_with_timeout("text", "Enable")
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "enable_status_bar.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            result = float(igas_comp.getsimilarityrate(
                os.path.join(temp_path, "enable_status_bar.png"), os.path.join(temp_path, "disable_status_bar.png")))
            if result < 1.0:
                break
        assert result < 1.0, "fail to enable status notification bar"
