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


class DeviceProvisioning(UIATestBase):
    """
    @summary: Test cases for device provisioning under managed profile
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
        set profile name for managed profile
        :return: None
        """
        profile_name = "HiProfileOwner"
        self.api.set_profile_name(profile_name)
        user_string = os.popen("adb -s {0} shell pm list users".format(self.api.serial)).read().strip()
        assert user_string.find(profile_name) != -1, "[ERROR]: fail to set profile name"
        # self.api.clean_tasks()
        self.api.set_profile_name("Work profile")

    def testCheck_Profile_Owner_App(self):
        """
        check profile owner for managed profile
        :return: None
        """
        assert self.api.check_profile_owner(), "[ERROR]: intel mdm is not profile owner"

    def testCheck_Device_Owner_App(self):
        """
        check device owner for managed profile
        :return: None
        """
        assert not self.api.check_device_owner(), "[ERROR]: intel mdm is device owner"


class AppProvisioning(UIATestBase):
    """
    @summary: Test cases for app provisioning under managed profile
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
        enable system applications for managed profile
        :return: None
        """
        assert not self.api.locate_apps("Work Gmail"), "[INFO]: system applications have been enabled"
        self.api.enable_system_applications()
        self.api.d.press.home()
        self.api.click_with_timeout("description", "Apps")
        time.sleep(5)
        assert self.api.locate_apps("Work Gmail", 5), "[ERROR]: system apps have not been enabled"

    def testSelect_Applications_to_Hide(self):
        """
        hide applications for managed profile
        :return: None
        """
        assert self.api.locate_apps("Work Downloads", 5), "[INFO]: Work Downloads app are already hide"
        self.api.hide_applications()
        self.api.d.press.home()
        self.api.click_with_timeout("description", "Apps")
        time.sleep(5)
        assert not self.api.locate_apps("Work Downloads"), "[ERROR]: fail to hide applications"

    def testList_Hidden_Applications(self):
        """
        list hidden application for managed profile
        :return: None
        """
        assert not self.api.locate_apps("Work Downloads"), "[INFO]: Work Downloads app are already exist"
        self.api.unhide_applications()
        self.api.d.press.home()
        self.api.click_with_timeout("description", "Apps")
        time.sleep(5)
        assert self.api.locate_apps("Work Downloads", 5), "[ERROR]: fail to unhide applications"

    def testBlock_Uninstalling_of_Applications(self):
        """
        block uninstalling of applications for managed profile
        :return: None
        """
        install_package = self.api.download_file_from_artifactory(self.api.remote.qq_ime['sub_path'],
                                                                  self.api.remote.qq_ime['name'])
        self.api.unknown_source_control(True)
        assert install_package is not None, "[ERROR]: fail to download QQshurufa_1337.apk"
        old_dir = os.getcwd()
        os.chdir(os.path.split(install_package)[0])
        self.api.install_apps(self.api.remote.qq_ime['pkg_name'], self.api.remote.qq_ime['name'].split('.apk')[0])
        os.chdir(old_dir)
        self.api.block_uninstall_apps()
        intent_string = "android.settings.APPLICATION_DETAILS_SETTINGS -d " + self.api.remote.qq_ime['pkg_name']
        self.api.launch_app_by_intents(intent_string)
        assert self.api.check_ui_exists("text", "Uninstall"), "[ERROR]: fail to launch app info for QQ input"
        self.api.click_with_timeout("text", "Uninstall")
        self.api.click_with_timeout("text", "OK")
        assert self.api.check_ui_exists("resourceId", "com.android.packageinstaller:id/device_manager_button", 10), \
            "[ERROR]: fail to block uninstall apps"

    def testBlocked_Uninstalling_Applications(self):
        """
        unblock uninstalling applications for managed profile
        :return: None
        """
        self.api.unblock_uninstall_apps()
        intent_string = "android.settings.APPLICATION_DETAILS_SETTINGS -d " + self.api.remote.qq_ime['pkg_name']
        self.api.launch_app_by_intents(intent_string)
        self.api.click_with_timeout("text", "Uninstall")
        self.api.click_with_timeout("text", "OK")
        assert not self.api.check_ui_exists("resourceId", "com.android.packageinstaller:id/device_manager_button", 5), \
            "[ERROR]: fail to uninstall apps"
        self.api.uninstall_apps_by_adb(self.api.remote.qq_ime['pkg_name'])


class IntentsDataSharing(UIATestBase):
    """
    @summary: Test cases for intent and data sharing under managed profile
    """

    def setUp(self):
        super(IntentsDataSharing, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        if not self.api.locate_apps("Work Chrome"):
            self.api.enable_system_applications(True)
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(IntentsDataSharing, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSend_Intent_toHandle_Text_Plain(self):
        """
        send intent to handle text/plain for managed profile
        :return: None
        """
        self.api.send_intent_to_handle()
        self.api.d(resourceId="android:id/resolver_list").wait.exists(timeout=3000)
        assert self.api.check_ui_exists("resourceId", "android:id/resolver_list"), \
            "[ERROR]: share window doesn't pop up"

    def testAdd_PersistentPreferredActivity_for_Text_Plain(self):
        """
        add persistent preferred activity for managed profile
        :return: None
        """
        self.api.add_persistent_activities()
        self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_handle)
        assert not self.api.check_ui_exists("resourceId", "android:id/resolver_list"), \
            "[ERROR]: share window still pop up"

    def testClear_Persistent_Preferred_Activities(self):
        """
        clear persistent preferred activities for managed profile
        :return: None
        """
        self.api.clear_persistent_activities()
        self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_handle)
        self.api.d(resourceId="android:id/resolver_list").wait.exists(timeout=3000)
        assert self.api.check_ui_exists("resourceId", "android:id/resolver_list"), \
            "[ERROR]: share window doesn't pop up"

    def testAdd_XProfileIntentFilter_for_Text_Plain(self):
        """
        add cross X profile intent filter for managed profile
        :return: None
        """
        self.api.add_x_intent_filter()
        self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_handle)
        if self.api.is_android_L_build():
            self.api.d(resourceId="android:id/resolver_list").wait.exists(timeout=3000)
            assert self.api.check_ui_exists("text", "Personal apps"), "[ERROR]: fail to add X profile filter"
        else:
            self.api.d(resourceId="android:id/resolver_list").wait.exists(timeout=3000)
            assert self.api.check_ui_exists("text", "Personal"), "[ERROR]: fail to add X profile filter"

    def testClear_XProfile_Intent_Filters(self):
        """
        clear cross profile intent filter for managed profile
        :return: None
        """
        self.api.clear_x_intent_filter()
        self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_handle)
        self.api.d(resourceId="android:id/resolver_list").wait.exists(timeout=3000)
        assert not self.api.check_ui_exists("textContains", "Personal"), "[ERROR]: fail to clear X profile filter"

    def testAdd_XProfile_Widget_Provider(self):
        """
        add cross X profile widget provider for managed profile
        :return: None
        """
        self.api.add_x_profile_widget_provider()
        if self.api.is_android_L_build():
            self.api.d(text="Analog clock").wait.exists(timeout=5000)
            assert self.api.check_ui_exists("text", "Analog clock"), "[ERROR]: Analog clock doesn't exist"
            assert self.api.d(text="Analog clock").right(text="Analog clock").exists, \
                "[ERROR]: badged widget doesn't exist"
        else:
            work_widget = False
            self.api.d(text="Work Clock").wait.exists(timeout=5000)
            for _ in range(3):
                if self.api.check_ui_exists(
                        "description", "Work Calendar") or self.api.check_ui_exists("description", "Work Clock"):
                    work_widget = True
                    break
            assert work_widget, "fail to detect work calendar/clock widget"

    def testRemove_XProfile_Widget_Provider(self):
        """
        remove cross X profile widget provider for managed profile
        :return: None
        """
        self.api.remove_x_profile_widget_provider()
        if self.api.is_android_L_build():
            assert self.api.check_ui_exists("text", "Analog clock"), "[ERROR]: Analog clock doesn't exist"
            if self.api.check_ui_exists("text", "AwareHub"):
                assert self.api.d(text="Analog clock").right(text="AwareHub").exists, \
                    "[ERROR]: AwareHub widget doesn't exist at right of analog clock"
            elif self.api.check_ui_exists("text", "Book"):
                assert self.api.d(text="Analog clock").right(text="Book").exists, \
                    "[ERROR]: Book widget doesn't exist at right of analog clock"
            elif self.api.check_ui_exists("text", "Bookmark"):
                assert self.api.d(text="Analog clock").right(text="Bookmark").exists, \
                    "[ERROR]: Bookmark widget doesn't exist at right of analog clock"
        else:
            work_widget = True
            for _ in range(3):
                if not self.api.check_ui_exists("description", "Work Calendar") and not self.api.check_ui_exists(
                        "description", "Work Clock"):
                    work_widget = False
                    break
            assert not work_widget, "able to detect Work widget after remove cross profile widget"

    def testGet_XProfile_Widget_Provider(self):
        """
        get cross X profile widget provider for managed profile
        :return: None
        """
        self.api.get_x_profile_widget_provider()
        if self.api.d(resourceId="android:id/list").scrollable:
            assert self.api.d(resourceId="android:id/list").scrollable, \
                "[ERROR]: there are no any widget providers exist"
        else:
            assert self.api.check_ui_exists("text", "com.google.android.youtube"), \
                "[ERROR]: there are no any widget providers exist"

    def testSend_Intent_to_Display_Audio_Effect_Control_Panel(self):
        """
        launch audio effect control panel by sending intent for managed profile
        :return: None
        """
        self.api.send_intent_to_audio_effect()
        self.api.d(text="Equalizer").wait.exists(timeout=3000)
        assert self.api.check_ui_exists("text", "Equalizer"), "[ERROR]: fail to launch Audio Equalizer"

    def testSend_Intent_to_Display_Recognize_Speech(self):
        """
        launch voice speech recognize by sending intent for managed profile
        :return: None
        """
        self.api.send_intent_to_speech_recognize()
        if self.api.check_ui_exists("text", "Google App"):
            self.api.click_with_timeout("text", "Google App")
        if self.api.check_ui_exists("text", "Just once"):
            self.api.click_with_timeout("text", "Just once")
        assert self.api.check_ui_exists("resourceId", "com.google.android.googlequicksearchbox:id/intent_api_recognizer"), \
            "[ERROR]: fail to launch speech recognize"

    def testSend_Intent_to_Display_Account_Sync_Settings(self):
        """
        send intent to display account sync settings for managed profile
        :return: None
        """
        assert os.path.isfile("/etc/oat/sys.conf"), "[ERROR]: Missing config file /etc/oat/sys.config in host"
        cfg_file = "/etc/oat/sys.conf"
        account = self.config.read(cfg_file, "google_account").get("user_name")
        password = self.config.read(cfg_file, "google_account").get("password")
        if not self.api.locate_apps("Work Sample MDM"):
            self.api.setup_managed_profile()
        self.api.settings_launch()
        if not self.api.check_ui_exists("text", "Accounts") and self.api.d(scrollable=True).exists:
            self.api.d(scrollable=True).scroll.vert.to(textContains="Accounts")
        self.api.click_with_timeout("text", "Accounts")
        assert self.api.check_ui_exists("text", "Work", 5), "[ERROR]: Work section doesn't exist"
        if not self.api.check_ui_exists("text", "Google", 5):
            self.api.clean_tasks()
            self.api.add_google_account(account, password, True)
        self.api.api_demo_po_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.intents_sharing)
        if not self.api.check_ui_exists("resourceId", self.api.ui.send_intent_to_account_sync):
            self.api.d(scrollable=True).scroll.vert.to(resourceId=self.api.ui.send_intent_to_account_sync)
        self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_account_sync)
        if self.api.is_android_L_build():
            self.api.click_with_timeout("textContains", "com.google")
            self.api.click_with_timeout("text", "Send Intent")
            self.api.check_ui_exists("text", "Sync", 10)
        else:
            self.api.check_ui_exists("resourceId", "com.intel.afw.mdm:id/content_edit")
            self.api.d(resourceId="com.intel.afw.mdm:id/content_edit").set_text(account)
            self.api.click_with_timeout("text", "Yes")
        assert self.api.check_ui_exists("text", "Google"), "[ERROR]: google account doesn't exist"
        assert self.api.check_ui_exists("text", "App Data"), "[ERROR]: app data doesn't exist in sync page"


class AppConfigAndPolicy(UIATestBase):
    """
    @summary: Test cases for app config and policy under managed profile
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
        set application restriction for chrome under managed profile
        :return: None
        """
        if not self.api.locate_apps("Work Chrome"):
            self.api.enable_system_applications()
            self.api.clean_tasks()
        self.api.set_apps_restriction_chrome()
        if "afwHandleChromeInvitedWindow" not in self.api.d.watchers:
            self.api.d.watcher("afwHandleChromeInvitedWindow").when(
                text="No Thanks").click(text="No Thanks")
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
                self.api.d.wait.update(timeout=3000)
                break
        # self.api.click_with_timeout("resourceId", "com.android.chrome:id/menu_button")
        for i in range(5):
            self.api.d.press.menu()
            if self.api.check_ui_exists("textContains", "incognito"):
                break
        assert self.api.check_ui_exists("textContains", "incognito", 5), "[ERROR]: menu window doesn't pop up"
        assert not self.api.d(textContains="incognito").enabled, "[ERROR]: fail to set restriction for chrome"
        # assert self.api.check_ui_exists("text", "Bookmarks"), "[ERROR]: Bookmarks item doesn't exist"
        # self.api.click_with_timeout("text", "Bookmarks")
        # if self.api.check_ui_exists("text", "Mobile bookmarks") and self.api.d(text="Bookmarks").clickable:
        #     self.api.click_with_timeout("text", "Bookmarks")
        # assert self.api.check_ui_exists("text", "Managed bookmarks"), "[ERROR]: Managed bookmarks item doesn't exist"
        # if self.api.d(text="Managed bookmarks").clickable:
        #     self.api.click_with_timeout("text", "Managed bookmarks")
        # assert self.api.check_ui_exists("text", "Chromium"), "[ERROR]: bookmark Chromium doesn't exist"
        # assert self.api.check_ui_exists("text", "Google"), "[ERROR]: bookmark Google doesn't exist"

    def testGet_Application_Restriction_Chrome(self):
        """
        get application restriction for chrome under managed profile
        :return: None
        """
        self.api.get_apps_restriction_chrome()
        assert self.api.check_ui_exists("text", "Restrictions"), "[ERROR]: restrictions window doesn't pop up"
        assert self.api.check_ui_exists("textContains", "ManagedBookmarks"), "[ERROR]: fail to get restrictions content"
        self.api.click_with_timeout("text", "OK")

    def testClear_Application_Restriction_Chrome(self):
        """
        clear application restriction for chrome under managed profile
        :return: None
        """
        self.api.clear_apps_restriction_chrome()
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
                self.api.d.wait.update(timeout=3000)
                break
        for i in range(5):
            self.api.d.press.menu()
            if self.api.check_ui_exists("textContains", "incognito"):
                break
        assert self.api.check_ui_exists("textContains", "incognito", 5), "[ERROR]: menu window doesn't pop up"
        assert self.api.d(textContains="incognito").enabled, "[ERROR]: fail to clear restriction for chrome"
        # assert self.api.check_ui_exists("text", "Bookmarks"), "[ERROR]: Bookmarks item doesn't exist"
        # self.api.click_with_timeout("text", "Bookmarks")
        # if self.api.check_ui_exists("text", "Mobile bookmarks") and self.api.d(text="Bookmarks").clickable:
        #     self.api.click_with_timeout("text", "Bookmarks")
        # assert not self.api.check_ui_exists("text", "Managed bookmarks"), "[ERROR]: Managed bookmarks still exist"
        # assert self.api.check_ui_exists("text", "Mobile bookmarks"), "[ERROR]: Mobile bookmarks doesn't exist"
        if "afwHandleChromeInvitedWindow" in self.api.d.watchers:
            self.api.d.watchers.remove("afwHandleChromeInvitedWindow")

    def testSetRestrictionProvider_SetTheDefaultRestrictionProvider(self):
        """
        set restriction provider for profile manager
        :return: None
        """
        try:
            self.api.set_clear_restriction_provider(True, True)
            assert self.api.check_ui_exists("text", "Set com.intel.afw.mdm.BasicRestrictionsReceiver as the restrictions provider"), \
                "[ERROR]: fail to set restriction provider"
        finally:
            self.api.clean_tasks()
            self.api.set_clear_restriction_provider(False, True)
            self.api.click_toast_window()

    def testSetRestrictionProvider_ClearTheDefaultRestrictionProvider(self):
        """
        clear restriction provider for profile manager
        :return: None
        """
        try:
            self.api.set_clear_restriction_provider(False, True)
            assert self.api.check_ui_exists("text", "Delete the restrictions provider"), \
                "[ERROR]: fail to clear restriction provider"
        finally:
            self.api.clean_tasks()
            self.api.set_clear_restriction_provider(False, True)
            self.api.click_toast_window()

    def testCheck_Restrictions_Provider(self):
        """
        check restriction provider for profile manager
        :return: None
        """
        try:
            self.api.set_clear_restriction_provider(True, True)
            self.api.clean_tasks()
            self.api.check_restriction_provider(True)
            assert self.api.check_ui_exists("text", "A restrictions provider has been set for this profile"), \
                "[ERROR]: fail to check restriction provider"
        finally:
            self.api.clean_tasks()
            self.api.set_clear_restriction_provider(False, True)
            self.api.click_toast_window()

    def testInstall_AppRestrictionSchema(self):
        """
        install app restriction schema for device owner
        :return: None
        """
        self.api.install_apprestriction_schema(True)
        if not self.api.locate_apps("Work AppRestrictionSchema"):
            g_common_obj2.system_reboot(90)
            for i in range(20):
                self.api.d = g_common_obj.get_device()
                self.api.d.wakeup()
                if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
                    break
                time.sleep(5)
            g_common_obj.set_vertical_screen()
            self.api.unlock_screen()
        # assert self.api.check_ui_exists("text", self.api.ui.set_apprestriction_schema), \
        #     "[ERROR]: fail to find set restrictions policy string"
        assert self.api.locate_apps("Work AppRestrictionSchema", 5), "[ERROR]: fail to locate apprestrictionschema"

    def testSet_Restrictions_Policy_For_AppRestrictionSchema(self):
        """
        set restriction schema for profile manager
        :return: None
        """
        try:
            if not self.api.locate_apps("Work AppRestrictionSchema"):
                self.api.install_apprestriction_schema(True)
                self.api.clean_tasks()
            self.api.set_clear_restriction_provider(True, True)
            self.api.click_toast_window()
            self.api.click_with_timeout("text", self.api.ui.set_apprestriction_schema)
            self.api.click_with_timeout("text", "Approve")
            # self.api.launch_app("Work AppRestrictionSchema")
            for i in range(2):
                if not self.api.launch_app_by_activity(self.api.ui.app_schema, True):
                    self.api.launch_app("Work AppRestrictionSchema")
                self.api.d(resourceId="com.example.android.apprestrictionschema:id/say_hello").wait.exists(timeout=5000)
                if self.api.d(resourceId="com.example.android.apprestrictionschema:id/say_hello").enabled:
                    break
                for _ in range(5):
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
            self.api.set_clear_restriction_provider(True, True)
            self.api.click_toast_window()
            self.api.click_with_timeout("text", self.api.ui.set_apprestriction_schema)
            self.api.click_with_timeout("text", "Deny")
            # self.api.launch_app("Work AppRestrictionSchema")
            if not self.api.launch_app_by_activity(self.api.ui.app_schema, True):
                self.api.launch_app("Work AppRestrictionSchema")
            self.api.click_with_timeout("resourceId",
                                        "com.example.android.apprestrictionschema:id/request_configuration")
            time.sleep(5)
            assert not self.api.d(resourceId="com.example.android.apprestrictionschema:id/say_hello").enabled, \
                "[ERROR]: fail to deny request"
            assert self.api.check_ui_exists("text", "I am restricted from saying hello to you."), \
                "[ERROR] fail to check deny string"
        finally:
            self.api.clean_tasks()
            self.api.set_clear_restriction_provider(False, True)
            self.api.click_toast_window()
            self.api.uninstall_apps_by_adb(self.api.remote.restriction_schema['pkg_name'])


class DeviceProfileManagementPolicies(UIATestBase):
    """
    @summary: Test cases for device profile management and policies under managed profile
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
        self.api.api_demo_po_launch()
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
        set and get screen capture state for managed profile
        :return: None
        """
        self.api.set_get_screen_capture(False, True)
        self.api.d.press("home")
        # self.api.launch_app("Work Downloads")
        if not self.api.launch_app_by_activity(self.api.ui.downloads, True):
            self.api.launch_app("Work Downloads")
        time.sleep(5)
        self.api.d.press(0x78)
        time.sleep(5)
        self.api.d.sleep()
        time.sleep(2)
        self.api.d.wakeup()
        assert self.api.check_ui_exists("textContains", "Couldn't capture screenshot"), \
            "[ERROR]: still able to take a screenshot"
        self.api.d(textContains="Couldn't capture screenshot").swipe.right()
        self.api.unlock_screen()
        self.api.clean_tasks()
        self.api.set_get_screen_capture(True, True)
        self.api.d.press("home")
        # self.api.launch_app("Work Downloads")
        if not self.api.launch_app_by_activity(self.api.ui.downloads, True):
            self.api.launch_app("Work Downloads")
        time.sleep(5)
        self.api.d.press(0x78)
        time.sleep(5)
        self.api.d.sleep()
        time.sleep(2)
        self.api.d.wakeup()
        assert self.api.check_ui_exists("textContains", "Screenshot captured"), \
            "[ERROR]: still unable to take a screenshot"
        self.api.d(textContains="Screenshot captured").swipe.right()
        self.api.unlock_screen()

    def testSetUserRestrictions_Disallow_modify_accounts(self):
        """
        disallow modify accounts for managed profile
        :return: None
        """
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_modify_account + "OFF")
            self.api.settings_sub_launch("Accounts")
            assert self.api.check_ui_exists("text", "Work"), "ERROR: Work section doesn't exist"
            assert self.api.d(text="Work").down(text="Add account") is None, "[ERROR]: still able to add account"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_modify_account + "ON")

    def testSetUserRestrictions_Disallow_share_location(self):
        """
        disallow share location for managed profile
        :return: None
        """
        for _ in range(3):  # seems like sometimes settings crashed
            self.api.launch_app_by_intents("android.settings.LOCATION_SOURCE_SETTINGS", True)
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
        assert self.api.check_ui_exists("text", "On"), "location sharing has been turned off"
        assert self.api.check_ui_exists("text", "Work profile"), "fail to detect work profile option"
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_share_location + "OFF", True)
            self.api.launch_app_by_intents("android.settings.LOCATION_SOURCE_SETTINGS", True)
            self.api.click_with_timeout("text", "Agree", 5)
            if not self.api.check_ui_exists("text", "Location"):
                self.api.settings_sub_launch("Location")
                self.api.click_with_timeout("text", "Agree", 5)
            assert self.api.check_ui_exists("text", "Location for work profile"), "fail to detect location for work"
            assert self.api.check_ui_exists("text", "Turned off by your company"), "fail to detect off statement"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_share_location + "ON", True)

    def testSetUserRestrictions_Disallow_config_credentials(self):
        """
        disallow config credentials for managed profile
        :return: None
        """
        cert_file = self.api.download_file_from_artifactory(self.api.remote.cert_file['sub_path'],
                                                            self.api.remote.cert_file['name'])
        assert cert_file is not None, "[ERROR]: fail to download CA Cert file"
        user_id = self.api.get_userid_for_work_profile()
        if os.popen("adb -s {0} shell ls /mnt".format(self.api.serial)).read().strip().find("shell") != -1:
            os.popen("adb -s {0} push {1} /mnt/shell/emulated/{2}/Download/".format(
                self.api.serial, cert_file, str(user_id)))
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_config_credentials + "OFF")
            self.api.clean_tasks()
            self.api.install_ca_cert()
            assert self.api.check_ui_exists("textContains", "DISALLOW_CONFIG_CREDENTIALS"), \
                "[ERROR]: still able to install ca cert"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_config_credentials + "ON")

    def testSetUserRestrictions_Disallow_remove_user(self):
        """
        disallow remove user for managed profile
        :return: None
        """
        sample_mdm_check = self.api.download_file_from_artifactory(self.api.remote.mdm_check['sub_path'],
                                                                   self.api.remote.mdm_check['name'])
        assert sample_mdm_check is not None, "[ERROR]: fail to download test package AFW_Test.apk"
        old_dir = os.getcwd()
        os.chdir(os.path.split(sample_mdm_check)[0])
        self.api.install_apps(self.api.remote.mdm_check['pkg_name'],
                              self.api.remote.mdm_check['name'].split('.apk')[0])
        os.chdir(old_dir)
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_remove_users + "OFF")
            intent_string = "android.settings.USER_SETTINGS"
            self.api.launch_app_by_intents(intent_string)
            if not self.api.check_ui_exists("text", "New user"):
                self.api.click_with_timeout("text", "Add user or profile")
                self.api.click_with_timeout("text", "Add user")
                self.api.click_with_timeout("text", "User")
                self.api.click_with_timeout("text", "OK")
                self.api.click_with_timeout("text", "Not now", 5)
            # self.api.launch_app("Work Sample MDM for Check")
            self.api.launch_app_by_activity("com.intel.afw.mdmforcheck/com.intel.afw.mdmforcheck.MainWindow", True)
            self.api.click_with_timeout("text", "Remove User")
            self.api.click_with_timeout("textContains", "New user")
            self.api.click_with_timeout("text", "Remove User")
            assert self.api.check_ui_exists("textContains", "has not been removed"), "[ERROR]: able to remove user"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_remove_users + "ON")
        # self.api.launch_app("Work Sample MDM for Check")
        self.api.launch_app_by_activity("com.intel.afw.mdmforcheck/com.intel.afw.mdmforcheck.MainWindow", True)
        self.api.click_with_timeout("text", "Remove User")
        self.api.click_with_timeout("textContains", "New user")
        self.api.click_with_timeout("text", "Remove User")
        self.api.uninstall_apps_by_adb(self.api.remote.mdm_check['pkg_name'])

    def testSetUserRestrictions_Disallow_apps_control(self):
        """
        disallow apps control for managed profile
        :return: None
        """
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_apps_control + "OFF")
            intent_string = "android.settings.APPLICATION_DETAILS_SETTINGS -d package:com.android.contacts"
            self.api.launch_app_by_intents(intent_string)
            if not self.api.check_ui_exists("text", "App info", 5):
                self.api.launch_app_by_intents("android.settings.APPLICATION_SETTINGS", True)
                self.api.click_with_timeout("text", "Running")
                self.api.click_with_timeout("text", "All")
                if not self.api.check_ui_exists("text", "Contacts", 5):
                    self.api.d(scrollable=True).scroll.vert.to(text="Contacts")
                self.api.click_with_timeout("text", "Contacts")
            assert self.api.check_ui_exists("text", "Force stop"), "[ERROR]: Force stop doesn't exist"
            assert not self.api.d(text="Force stop").enabled, "[ERROR]: fail to disallow apps control"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_apps_control + "ON")

    def testSetUserRestrictions_Disallow_cross_profile_CopyPaste(self):
        """
         disallow cross copy paste for managed profile
        :return: None
        """
        if "afwHandleChromeInvitedWindow" not in self.api.d.watchers:
            self.api.d.watcher("afwHandleChromeInvitedWindow").when(
                text="No Thanks").click(text="No Thanks")
        # create manage profile if needed
        if not self.api.locate_apps("Work Sample MDM"):
            self.api.setup_managed_profile()
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
        # able to copy content from managed profile to primary user
        self.api.clean_tasks()
        # self.api.launch_app("Work Chrome")
        if not self.api.launch_app_by_activity(self.api.ui.chrome, True):
            self.api.launch_app("Work Chrome")
        # assert self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar", 10), \
        #     "[ERROR]: fail to detect url bar for work chrome"
        assert self.api.check_ui_exists("packageName", "com.android.chrome", 10), "fail to detect work chrome"
        time.sleep(5)
        for i in range(3):
            if self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar"):
                self.api.d(resourceId="com.android.chrome:id/url_bar").set_text("ManagedProfile")
            else:
                self.api.d(resourceId="com.android.chrome:id/search_box_text").set_text("ManagedProfile")
            time.sleep(3)
            # self.api.d(resourceId="com.android.chrome:id/url_bar").long_click.topleft()
            self.api.d.press(0x1d, 0x7000)  # select all by Ctrl + A
            self.api.d.press(0x1f, 0x7000)  # copy content by key event Ctrl + C
        if not self.api.launch_app_by_activity(self.api.ui.chrome, False):
            self.api.launch_app("Chrome")
        # assert self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar", 10), \
        #     "[ERROR]: fail to detect url bar for chrome"
        assert self.api.check_ui_exists("packageName", "com.android.chrome", 10), "fail to detect chrome"
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
            if self.api.check_ui_exists("textContains", "ManagedProfile"):
                managed_profile = True
        assert managed_profile, "[ERROR]: unable to paste content from managed profile to primary user"

        # unable to copy content from managed profile to primary user
        self.api.clean_tasks()
        # set restrictions
        try:
            self.api.set_user_restrictions(self.api.ui.disallow_cross_copy_paste + "OFF", True)
            # self.api.launch_app("Work Chrome")
            if not self.api.launch_app_by_activity(self.api.ui.chrome, True):
                self.api.launch_app("Work Chrome")
            # assert self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar", 10), \
            #     "[ERROR]: fail to detect url bar for work chrome"
            assert self.api.check_ui_exists("packageName", "com.android.chrome", 10), "fail to detect work chrome"
            time.sleep(5)
            for i in range(3):
                if self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar"):
                    self.api.d(resourceId="com.android.chrome:id/url_bar").set_text("ManagedProfile")
                else:
                    self.api.d(resourceId="com.android.chrome:id/search_box_text").set_text("ManagedProfile")
                time.sleep(3)
                # self.api.d(resourceId="com.android.chrome:id/url_bar").long_click.topleft()
                self.api.d.press(0x1d, 0x7000)  # select all by Ctrl + A
                self.api.d.press(0x1f, 0x7000)  # copy content by key event Ctrl + C
            if not self.api.launch_app_by_activity(self.api.ui.chrome, False):
                self.api.launch_app("Chrome")
            # assert self.api.check_ui_exists("resourceId", "com.android.chrome:id/url_bar", 10), \
            #     "[ERROR]: fail to detect url bar for chrome"
            assert self.api.check_ui_exists("packageName", "com.android.chrome", 10), "fail to detect chrome"
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
                if self.api.check_ui_exists("textContains", "ManagedProfile"):
                    managed_profile = True
            assert not managed_profile, "[ERROR]: able to paste content from managed profile to primary user"
        finally:
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_cross_copy_paste + "ON", True)
            if "afwHandleChromeInvitedWindow" in self.api.d.watchers:
                self.api.d.watchers.remove("afwHandleChromeInvitedWindow")

    def testDisable_Account_Management_Add(self):
        """
        add account into disable account management list for managed profile
        :return: None
        """
        self.api.disable_account_management(True)
        self.api.settings_sub_launch("Accounts")
        assert self.api.check_ui_exists("text", "Work"), "[ERROR]: work section doesn't exist"
        self.api.d(text="Remove work profile").up(text="Add account").click()
        self.api.click_with_timeout("text", "Google")
        assert self.api.check_ui_exists("text", "This change isn't allowed by your administrator"), \
            "[ERROR]: still allow to add google account"

    def testDisable_Account_Management_Del(self):
        """
        delete account from disable account management list for managed profile
        :return: None
        """
        self.api.disable_account_management(False)
        self.api.settings_sub_launch("Accounts")
        assert self.api.check_ui_exists("text", "Work"), "[ERROR]: work section doesn't exist"
        self.api.d(text="Remove work profile").up(text="Add account").click()
        self.api.click_with_timeout("text", "Google")
        assert not self.api.check_ui_exists("text", "This change isn't allowed by your administrator"), \
            "[ERROR]: fail to add google account"

    def testEnable_Pre_Populated_Email_Address(self):
        """
        populate email address for managed profile
        :return: None
        """
        self.api.api_demo_po_launch()
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

    def testSetUserRestrictions_Disallow_install_apps(self):
        """
        disallow install apps from google play store
        :return: None
        """
        if not self.api.locate_apps("Work Sample MDM"):
            self.api.setup_managed_profile()
        self.api.settings_launch()
        if not self.api.check_ui_exists("text", "Accounts") and self.api.d(scrollable=True).exists:
            self.api.d(scrollable=True).scroll.vert.to(textContains="Accounts")
        self.api.click_with_timeout("text", "Accounts")
        assert self.api.check_ui_exists("text", "Work", 5), "[ERROR]: Work section doesn't exist"
        if not self.api.check_ui_exists("text", "Google", 5):
            assert os.path.isfile("/etc/oat/sys.conf"), "[ERROR]: Missing config file /etc/oat/sys.config in host"
            cfg_file = "/etc/oat/sys.conf"
            account = self.config.read(cfg_file, "google_account").get("user_name")
            password = self.config.read(cfg_file, "google_account").get("password")
            self.api.clean_tasks()
            self.api.add_google_account(account, password, True)
        if "afwHandleUnfortunately" in self.api.d.watchers:
            self.api.d.watchers.remove("afwHandleUnfortunately")
        try:
            # suppose run this test case just after NotProvision.SetupManagedProfile.testAccount_Migrate
            # we need google account in work profile before launch google play store
            self.api.set_user_restrictions(self.api.ui.disallow_install_apps + "OFF")
            self.api.launch_app_by_intents(
                "android.intent.action.VIEW -c android.intent.category.BROWSABLE "
                "-d market://details?id=com.surpax.ledflashlight.panel")
            self.api.click_with_timeout("text", "ACCEPT", 30)
            assert self.api.check_ui_exists("text", "INSTALL", 60), "[ERROR]: fail to find chrome in play store"
            self.api.click_with_timeout("text", "INSTALL")
            self.api.click_with_timeout("text", "ACCEPT", 30)
            self.api.click_with_timeout("text", "Proceed", 15)
            self.api.click_with_timeout("text", "CONTINUE", 5)
            assert self.api.check_ui_exists("textContains", "Unfortunately", 10), \
                "[ERROR]: fail to disallow install apps from play store"
        finally:
            for _ in range(3):
                self.api.click_with_timeout("text", "OK")
            self.api.clean_tasks()
            self.api.set_user_restrictions(self.api.ui.disallow_install_apps + "ON")
            if self.api.is_android_L_build():
                self.api.remove_managed_profile(False)

    def testSetUserRestrictions_Disallow_create_windows(self):
        """
         disallow create window for device owner
        :return: None
        """
        igas_comp = igascomparator()
        self.api.api_demo_po_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.intents_sharing)
        self.api.click_with_timeout("resourceId", self.api.ui.add_persistent_preferred)
        self.api.click_with_timeout("text", "Copy to clipboard")
        self.api.click_with_timeout("text", "Pick APP")
        box = (0, (self.api.d.info["displayHeight"]*3)/4,
               self.api.d.info["displayWidth"], self.api.d.info["displayHeight"])
        temp_path = getTmpDir()
        self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
        image = Image.open(os.path.join(temp_path, "screenshot.png"))
        image.crop(box).save(os.path.join(temp_path, "not_trigger.png"))
        os.remove(os.path.join(temp_path, "screenshot.png"))
        assert os.path.isfile(os.path.join(temp_path, "not_trigger.png")), "[ERROR]: fail to save not_trigger.png"
        result = 1
        for _ in range(5):  # try 5 times to capture screen with toast message and compare it with previous screen
            self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_handle)
            self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_handle)
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
        self.api.set_user_restrictions("disallow create windows OFF")
        self.api.d.press.back()
        self.api.click_with_timeout("resourceId", self.api.ui.intents_sharing)
        result = 0
        for _ in range(5):  # try 5 times to capture screen without toast message and compare it with previous screen
            self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_handle)
            self.api.click_with_timeout("resourceId", self.api.ui.send_intent_to_handle)
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
        self.api.click_with_timeout("resourceId", self.api.ui.clear_persistent_preferred)
        self.api.click_with_timeout("text", "OK")
        self.api.clean_tasks()
        self.api.set_user_restrictions("disallow create windows ON")

    def testSetUserRestrictions_Disallow_outgoing_calls(self):
        """
        verify disallow call dialed out
        :return: None
        """
        if not self.api.locate_apps("Work Phone"):
            self.api.enable_system_applications(True)
        self.api.launch_app_by_activity("com.android.dialer/com.android.dialer.DialtactsActivity", True)
        if not self.api.check_ui_exists("packageName", "com.android.dialer"):
            self.api.launch_app("Work Phone")
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
        self.api.set_user_restrictions("disallow outgoing calls OFF", True)
        self.api.launch_app_by_activity("com.android.dialer/com.android.dialer.DialtactsActivity", True)
        if not self.api.check_ui_exists("packageName", "com.android.dialer"):
            self.api.launch_app("Work Phone")
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
            self.api.set_user_restrictions("disallow outgoing calls ON", True)


class VPNCertificateManagement(UIATestBase):
    """
    Test cases for VPN certificate management under managed profile
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
        time.sleep(5)
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
        self.api.api_demo_po_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.vpn_management)
        for _ in range(3):
            if self.api.check_ui_exists("textContains", "com.intel.afw.certsInstallation will be gaven access to"):
                break
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnSetCertInstallerPackage")
        assert self.api.check_ui_exists("textContains", "com.intel.afw.certsInstallation will be gaven access to"), \
            "fail to give access to 3rd party app"
        self.api.click_with_timeout("text", "OK")
        if self.api.launch_app_by_activity(
                "com.intel.afw.certsInstallation/com.intel.afw.certsInstallation.MainWindow", True):
            self.api.launch_app("Work Certs Management")
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
        self.api.api_demo_po_launch()
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
                "com.intel.afw.certsInstallation/com.intel.afw.certsInstallation.MainWindow", True):
            self.api.launch_app("Work Certs Management")
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

        self.api.api_demo_po_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.vpn_management)
        for _ in range(3):
            if self.api.check_ui_exists("textContains", "com.intel.afw.certsInstallation will not be gaven access to"):
                break
            self.api.click_with_timeout("resourceId", "com.intel.afw.mdm:id/btnClearCertInstallerPackage")
        assert self.api.check_ui_exists("textContains", "com.intel.afw.certsInstallation will not be gaven access to"), \
            "fail to clear allowed 3rd app"
        self.api.click_with_timeout("text", "OK")
        if self.api.launch_app_by_activity(
                "com.intel.afw.certsInstallation/com.intel.afw.certsInstallation.MainWindow", True):
            self.api.launch_app("Work Certs Management")
        self.api.click_with_timeout("resourceId", "com.intel.afw.certsInstallation:id/btnUninstallCaCert")
        self.api.click_with_timeout("text", "Select All", 5)
        self.api.d.watchers.remove()
        self.api.click_with_timeout("text", "Uninstall CA Cert")
        assert self.api.check_ui_exists("textContains", "Unfortunately"), "fail to clear 3rd cert installer"
        self.api.click_with_timeout("text", "OK")
        self.api.uninstall_apps_by_adb("com.intel.afw.certsInstallation")

    def testInstall_CA_Cert(self):
        """
        install CA Cert for managed profile
        :return: None
        """
        if self.api.is_android_L_build():
            cert_file = self.api.download_file_from_artifactory(self.api.remote.cert_file['sub_path'],
                                                                self.api.remote.cert_file['name'])
            assert cert_file is not None, "[ERROR]: fail to download CA Cert file"
            user_id = self.api.get_userid_for_work_profile()
            if os.popen("adb -s {0} shell ls /mnt".format(self.api.serial)).read().strip().find("shell") != -1:
                os.popen("adb -s {0} push {1} /mnt/shell/emulated/{2}/Download/".format(
                    self.api.serial, cert_file, str(user_id)))
        self.api.install_ca_cert()
        self.api.click_with_timeout("resourceId", self.api.ui.get_installed_ca_cert)
        if self.api.check_ui_exists("text", "key_alpha.cer"):
            assert self.api.check_ui_exists("text", "key_alpha.cer"), "[ERROR]: fail to install key_alpha.cer"
        else:
            assert self.api.check_ui_exists("text", "mytestcert3.cer"), "[ERROR]: fail to install mytestcert3.cer"

    def testUninstall_CA_Cert(self):
        """
        uninstall CA Cert for managed profile
        :return: None
        """
        self.api.uninstall_ca_cert()
        self.api.click_with_timeout("resourceId", self.api.ui.get_installed_ca_cert)
        assert self.api.check_ui_exists("text", "Attention"), "[ERROR]: fail to uninstall ca cert"

    def testCheck_CA_Cert_Installed(self):
        """
        check CA Cert whether is installed for managed profile
        :return: None
        """
        self.api.check_ca_cert()
        assert self.api.check_ui_exists("textContains", "has been installed"), "[ERROR]: fail to check ca cert"

    def testGet_Installed_CA_Cert(self):
        """
        get all installed CA Cert for managed profile
        :return: None
        """
        self.api.get_installed_ca_cert()
        if self.api.check_ui_exists("text", "key_alpha.cer"):
            assert self.api.check_ui_exists("text", "key_alpha.cer"), "[ERROR]: fail to get installed key_alpha.cer"
        else:
            assert self.api.check_ui_exists("text", "mytestcert3.cer"), "[ERROR]: fail to get installed mytestcert3.cer"

    def testUninstall_All_User_CA_Certs(self):
        """
        uninstall all installed CA Cert for managed profile
        :return: None
        """
        self.api.uninstall_all_ca_cert()
        self.api.click_with_timeout("resourceId", self.api.ui.get_installed_ca_cert)
        assert self.api.check_ui_exists("text", "Attention"), "[ERROR]: fail to uninstall ca cert"


class LegacyDevicePolicies(UIATestBase):
    """
    @summary: Test cases for legacy device and policies under managed profile
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

    def testGet_Password_Info(self):
        """
        get inputting password information in lock screen for managed profile
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
        self.api.get_password_info()
        assert self.api.check_ui_exists("text", "Password relevant info"), "[ERROR]: fail to get password info"
        rel_info = str(self.api.d(resourceId="android:id/message").text).strip().split('\ncurrent')
        # failed_time = int(rel_info[1][-1])
        # assert failed_time == 1, "fail to detect failed times"
        assert rel_info[0].find("Device screen has been successfully unlocked") != 1, "fail to detect successfully info"
        self.api.click_with_timeout("text", "OK")
        self.api.set_lock_swipe()

    def testGet_Set_Camera_State(self):
        """
        get and set camera state for managed profile
        :return: None
        """
        self.api.set_get_camera_state(False)
        # self.api.d.press("home")
        # self.api.launch_app("Camera")
        if not self.api.launch_app_by_activity(self.api.ui.camera, True):
            self.api.launch_app("Work Camera")
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
        self.api.set_get_camera_state(True)
        # self.api.d.press("home")
        # self.api.launch_app("Camera")
        if not self.api.launch_app_by_activity(self.api.ui.camera, True):
            self.api.launch_app("Work Camera")
        time.sleep(2)
        self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/permission_allow_button")
        self.api.click_with_timeout("text", "NEXT")
        assert not self.api.check_ui_exists("text", "Camera error"), "[ERROR]: fail to enable camera"

    def testGet_Set_Camera_State_NotAffect(self):
        """
        Verify the primary camera is still able to launch after disable it in work profile
        :return: None
        """
        self.api.set_get_camera_state(False, True)
        # self.api.d.press("home")
        # self.api.launch_app("Camera")
        if not self.api.launch_app_by_activity(self.api.ui.camera, True):
            self.api.launch_app("Work Camera")
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
        assert camera_state, "[ERROR]: fail to disable camera in work profile"

        if not self.api.launch_app_by_activity(self.api.ui.camera, False):
            self.api.launch_app("Camera")
        for i in range(5):
            if not self.api.check_ui_exists("resourceId", "com.android.packageinstaller:id/permission_allow_button"):
                break
            self.api.click_with_timeout("resourceId", "com.android.packageinstaller:id/permission_allow_button")
        camera_state = False
        for _ in range(3):  # try more times to check
            self.api.click_with_timeout("text", "NEXT")  # remember photo locations page
            if self.api.check_ui_exists("text", "Camera error"):
                camera_state = True
                break
        assert not camera_state, "[ERROR]: camera in primary user also be disabled"
        self.api.clean_tasks()
        self.api.set_get_camera_state(True, True)

    def testLock_Now(self):
        """
        lock screen in Intel Sample MDM for managed profile
        :return: None
        """
        self.api.lock_now_from_mdm()
        time.sleep(2)
        self.api.d.wakeup()
        assert self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view), "[ERROR]: fail to lock"

    def testSet_Password_Expiration_Timeout(self):
        """
        set password expiration time for managed profile
        :return: None
        """
        # verify string(Password expired) in "get password info"
        self.api.set_password_expiration_timeout()
        time.sleep(75)
        self.api.click_with_timeout("resourceId", self.api.ui.get_password_info)
        assert self.api.check_ui_exists("resourceId", "android:id/message"), "[ERROR]: fail to get password info"
        password_info = self.api.d(resourceId="android:id/message").text.strip()
        self.api.click_with_timeout("text", "OK")
        assert password_info.find("Password expired") != -1, "[ERROR]: fail to set expire timeout for password"

    def testWipe_Data_with_FRP(self):
        """
        wipe data with frp for managed profile
        :return: None
        """
        assert self.api.locate_apps("Work Sample MDM", 5), "[ERROR]: fail to locate Work Sample MDM"
        self.api.api_demo_po_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.legacy_policy)
        self.api.click_with_timeout("resourceId", self.api.ui.wipe_with_frp)
        self.api.click_with_timeout("text", "OK")
        # assert self.api.check_ui_exists("textContains", "Success", 10), "[ERROR]: fail to wipe data"
        self.api.click_with_timeout("text", "OK")
        assert self.api.locate_apps("Work Sample MDM", 5), "[ERROR]: Work Sample MDM doesn't exist"

    def testGetDataUsageStatistics(self):
        self.api.api_demo_po_launch()
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

    def testSetKeyguardDisabledFeatures_donot_show_unredacted_notifications(self):
        """
        don't show unredacted notification for profile owner
        :return: None
        """
        self.api.set_lock_pin()
        self.api.clean_tasks()
        try:
            self.api.set_keyguard_disabled_features("don't show unredacted notifications OFF", True)
            self.api.launch_app_by_intents(
                "android.intent.action.VIEW -c android.intent.category.BROWSABLE -d "
                "http://mars.androidapksfree.com/files/moon/com.liquidum.hexlock-v1.4.2-25-Android-4.0.3.apk")
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
            assert self.api.check_ui_exists("text", "Download Manager", 15), \
                "[ERROR]: fail to detect Download Manager in lock screen"
            assert self.api.check_ui_exists("text", "Contents hidden"), \
                "[ERROR]: fail to detect Contents hidden in lock screen"
        finally:
            self.api.unlock_screen()
            self.api.clean_tasks()
            self.api.set_lock_swipe()
            self.api.set_keyguard_disabled_features("don't show unredacted notifications ON", True)

    def testSetKeyguardDisabledFeatures_disable_trust_agent(self):
        """
        disable trust agent for device owner
        :return: None
        """
        self.api.set_lock_pin()
        self.api.set_keyguard_disabled_features("disable trust agent ON", True)
        self.api.clean_tasks()
        self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS")
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
            self.api.set_keyguard_disabled_features("disable trust agent OFF", True)
            self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS")
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
            self.api.set_keyguard_disabled_features("disable trust agent ON", True)
            self.api.set_lock_swipe()

    def testSetKeyguardDisabledFeatures_disable_features_all(self):
        """
        disable all available keyguard feature for profile owner
        :return: None
        """
        self.api.set_lock_pin()
        self.api.api_demo_po_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.legacy_policy)
        self.api.click_with_timeout("resourceId", self.api.ui.set_keyguard_disable_feature)
        if self.api.check_ui_exists("text", "disable trust agent ON"):
            self.api.click_with_timeout("text", "disable trust agent ON")
        if self.api.check_ui_exists("text", "don't show unredacted notifications ON"):
            self.api.click_with_timeout("text", "don't show unredacted notifications ON")
        self.api.click_with_timeout("text", "OK")
        self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS", True)
        if not self.api.check_ui_exists("text", "Smart Lock"):
            self.api.d(scrollable=True).scroll.vert.to(text="Smart Lock")
        assert self.api.check_ui_exists("text", "Smart Lock"), "fail to detect smart lock"
        assert not self.api.check_ui_exists("text", "Disabled by administrator"), "smart lock was already disabled"
        self.api.clean_tasks()
        self.api.api_demo_po_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.legacy_policy)
        self.api.click_with_timeout("resourceId", self.api.ui.set_keyguard_disable_feature)
        self.api.click_with_timeout("text", "disable features all OFF")
        self.api.click_with_timeout("text", "OK")
        self.api.launch_app_by_intents(
            "android.intent.action.VIEW -c android.intent.category.BROWSABLE -d "
            "http://mars.androidapksfree.com/files/moon/com.liquidum.hexlock-v1.4.2-25-Android-4.0.3.apk")
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
        assert self.api.check_ui_exists("text", "Download Manager", 15), \
            "[ERROR]: fail to detect Download Manager in lock screen"
        assert self.api.check_ui_exists("text", "Contents hidden"), \
            "[ERROR]: fail to detect Contents hidden in lock screen"
        self.api.unlock_screen()
        self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS", True)
        if not self.api.check_ui_exists("text", "Smart Lock"):
            self.api.d(scrollable=True).scroll.vert.to(text="Smart Lock")
        assert self.api.check_ui_exists("text", "Smart Lock"), "fail to detect smart lock"
        assert self.api.check_ui_exists("text", "Disabled by administrator"), "fail to disable smart lock"
        self.api.clean_tasks()
        self.api.set_lock_swipe()


class ThirdPartyAppService(UIATestBase):
    """
    @summary: Test cases for 3rd party apps and services under managed profile
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
        set permitted input method for managed profile
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
        self.api.third_party_app_action("SetInput")
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
            self.api.third_party_app_action("SetInput")
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
        self.api.third_party_app_action("SetInput")
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
        self.api.third_party_app_action("SetInput")
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
        get permitted input method for managed profile
        :return: None
        """
        self.api.third_party_app_action("SetInput")
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
        self.api.third_party_app_action("SetInput")
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
        set permitted accessibility service for managed profile
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
        self.api.third_party_app_action("SetAccessibility")
        for i in range(5):  # try more times to verify that foo.bar.testback is detected by Sample MDM
            if self.api.check_ui_exists("text", self.api.remote.test_back['pkg_name'][8:]):
                break
            self.api.d.press.back()
            time.sleep(5)
            self.api.click_with_timeout("resourceId", self.api.ui.set_accessibility_service)
        assert self.api.check_ui_exists("text", self.api.remote.test_back['pkg_name'][8:]), \
            "[ERROR]: fail to detect foo.bar.testback in Sample MDM"
        self.api.click_with_timeout("text", self.api.ui.select_all)
        self.api.click_with_timeout("text", "set selected apps")
        assert self.api.check_ui_exists("textContains", "Set successfully with packages"), \
            "[ERROR]: fail to set accessibility service"
        self.api.click_with_timeout("text", "OK")
        self.api.launch_app_by_intents("android.settings.ACCESSIBILITY_SETTINGS", False)
        assert self.api.check_ui_exists("text", "TestBack") and self.api.d(text="TestBack").enabled, \
            "[ERROR]: testback service is not exist or disabled"

        self.api.clean_tasks()
        self.api.third_party_app_action("SetAccessibility")
        for i in range(5):  # try more times to verify that foo.bar.testback is detected by Sample MDM
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
        self.api.third_party_app_action("SetAccessibility")
        for i in range(5):  # try more times to verify that foo.bar.testback is detected by Sample MDM
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
        get permitted accessibility service for managed profile
        :return: None
        """
        self.api.third_party_app_action("SetAccessibility")
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
        self.api.third_party_app_action("SetAccessibility")
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
        self.api.uninstall_apps_by_adb(self.api.remote.test_back['pkg_name'])


class RuntimePermission(UIATestBase):
    """
    @summary: Test cases for 3rd party apps and services under profile owner
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
        self.api.set_runtime_permission("LOCATION", "Default", True)
        enabled, checked = self.api.check_runtime_permission("Location", True)
        assert enabled, "fail to set location to Default"

    def testSetGrant_Location_RuntimepermissionState(self):
        self.api.set_runtime_permission("LOCATION", "Grant", True)
        enabled, checked = self.api.check_runtime_permission("Location", True)
        assert not enabled, "location still enabled"
        assert checked, "fail to set location to Grant"

    def testSetDeny_Location_RuntimepermissionState(self):
        self.api.set_runtime_permission("LOCATION", "Deny", True)
        enabled, checked = self.api.check_runtime_permission("Location", True)
        assert not enabled, "location still enabled"
        assert not checked, "fail to set location to Deny"

    # contact
    def testSetGrant_Contact_RuntimepermissionState(self):
        self.api.uninstall_apps_by_adb("com.example.jizhenlo.runtimepermissiontest")
        self.api.set_runtime_permission("CONTACT", "Grant", True)
        enabled, checked = self.api.check_runtime_permission("Contacts", True)
        assert not enabled, "contact still enabled"
        assert checked, "fail to set contact to Grant"

    def testSetDeny_Contact_RuntimepermissionState(self):
        self.api.set_runtime_permission("CONTACT", "Deny", True)
        enabled, checked = self.api.check_runtime_permission("Contacts", True)
        assert not enabled, "contact still enabled"
        assert not checked, "fail to set contact to Deny"

    def testSetDefault_Contact_RuntimepermissionState(self):
        self.api.set_runtime_permission("CONTACT", "Default", True)
        enabled, checked = self.api.check_runtime_permission("Contacts", True)
        assert enabled, "fail to set contact to Default"

    # camera
    def testSetGrant_Camera_RuntimepermissionState(self):
        self.api.set_runtime_permission("CAMERA", "Grant", True)
        enabled, checked = self.api.check_runtime_permission("Camera", True)
        assert not enabled, "camera still enabled"
        assert checked, "fail to set camera to Grant"

    def testSetDeny_Camera_RuntimepermissionState(self):
        self.api.set_runtime_permission("CAMERA", "Deny", True)
        enabled, checked = self.api.check_runtime_permission("Camera", True)
        assert not enabled, "camera still enabled"
        assert not checked, "fail to set camera to Deny"

    def testSetDefault_Camera_RuntimepermissionState(self):
        self.api.set_runtime_permission("CAMERA", "Default", True)
        enabled, checked = self.api.check_runtime_permission("Camera", True)
        assert enabled, "fail to set camera to Default"

    # calendar
    def testSetGrant_Calendar_RuntimepermissionState(self):
        self.api.set_runtime_permission("CALENDAR", "Grant", True)
        enabled, checked = self.api.check_runtime_permission("Calendar", True)
        assert not enabled, "calendar still enabled"
        assert checked, "fail to set calendar to Grant"

    def testSetDeny_Calendar_RuntimepermissionState(self):
        self.api.set_runtime_permission("CALENDAR", "Deny", True)
        enabled, checked = self.api.check_runtime_permission("Calendar", True)
        assert not enabled, "calendar still enabled"
        assert not checked, "fail to set calendar to Deny"

    def testSetDefault_Calendar_RuntimepermissionState(self):
        self.api.set_runtime_permission("CALENDAR", "Default", True)
        enabled, checked = self.api.check_runtime_permission("Calendar", True)
        assert enabled, "fail to set calendar to Default"

    # audio
    def testSetGrant_Audio_RuntimepermissionState(self):
        self.api.set_runtime_permission("AUDIO", "Grant", True)
        enabled, checked = self.api.check_runtime_permission("Microphone", True)
        assert not enabled, "audio still enabled"
        assert checked, "fail to set audio to Grant"

    def testSetDeny_Audio_RuntimepermissionState(self):
        self.api.set_runtime_permission("AUDIO", "Deny", True)
        enabled, checked = self.api.check_runtime_permission("Microphone", True)
        assert not enabled, "audio still enabled"
        assert not checked, "fail to set audio to Deny"

    def testSetDefault_Audio_RuntimepermissionState(self):
        self.api.set_runtime_permission("AUDIO", "Default", True)
        enabled, checked = self.api.check_runtime_permission("Microphone", True)
        assert enabled, "fail to set audio to Default"

    # phone
    def testSetGrant_Phone_RuntimepermissionState(self):
        self.api.set_runtime_permission("PHONE", "Grant", True)
        enabled, checked = self.api.check_runtime_permission("Phone", True)
        assert not enabled, "phone still enabled"
        assert checked, "fail to set phone to Grant"

    def testSetDeny_Phone_RuntimepermissionState(self):
        self.api.set_runtime_permission("PHONE", "Deny", True)
        enabled, checked = self.api.check_runtime_permission("Phone", True)
        assert not enabled, "Phone still enabled"
        assert not checked, "fail to set phone to Deny"

    def testSetDefault_Phone_RuntimepermissionState(self):
        self.api.set_runtime_permission("PHONE", "Default", True)
        enabled, checked = self.api.check_runtime_permission("Phone", True)
        assert enabled, "fail to set phone to Default"

    # body_sensor
    def testSetGrant_Body_sensor_RuntimepermissionState(self):
        self.api.set_runtime_permission("BODY_SENSOR", "Grant", True)
        enabled, checked = self.api.check_runtime_permission("Body Sensors", True)
        assert not enabled, "body sensors still enabled"
        assert checked, "fail to set body sensors to Grant"

    def testSetDeny_Body_sensor_RuntimepermissionState(self):
        self.api.set_runtime_permission("BODY_SENSOR", "Deny", True)
        enabled, checked = self.api.check_runtime_permission("Body Sensors", True)
        assert not enabled, "body sensors still enabled"
        assert not checked, "fail to set body sensors to Deny"

    def testSetDefault_Body_sensor_RuntimepermissionState(self):
        self.api.set_runtime_permission("BODY_SENSOR", "Default", True)
        enabled, checked = self.api.check_runtime_permission("Body Sensors", True)
        assert enabled, "fail to set body sensors to Default"

    # sms
    def testSetGrant_SMS_RuntimepermissionState(self):
        self.api.set_runtime_permission("SMS", "Grant", True)
        enabled, checked = self.api.check_runtime_permission("SMS", True)
        assert not enabled, "sms still enabled"
        assert checked, "fail to set sms to Grant"

    def testSetDeny_SMS_RuntimepermissionState(self):
        self.api.set_runtime_permission("SMS", "Deny", True)
        enabled, checked = self.api.check_runtime_permission("SMS", True)
        assert not enabled, "sms still enabled"
        assert not checked, "fail to set sms to Deny"

    def testSetDefault_SMS_RuntimepermissionState(self):
        self.api.set_runtime_permission("SMS", "Default", True)
        enabled, checked = self.api.check_runtime_permission("SMS", True)
        assert enabled, "fail to set sms to Default"

    # storage
    def testSetGrant_Storage_RuntimepermissionState(self):
        self.api.set_runtime_permission("STORAGE", "Grant", True)
        enabled, checked = self.api.check_runtime_permission("Storage", True)
        assert not enabled, "storage still enabled"
        assert checked, "fail to set storage to Grant"

    def testSetDeny_Storage_RuntimepermissionState(self):
        self.api.set_runtime_permission("STORAGE", "Deny", True)
        enabled, checked = self.api.check_runtime_permission("Storage", True)
        assert not enabled, "storage still enabled"
        assert not checked, "fail to set storage to Deny"

    def testSetDefault_Storage_RuntimepermissionState(self):
        self.api.set_runtime_permission("STORAGE", "Default", True)
        enabled, checked = self.api.check_runtime_permission("Storage", True)
        assert enabled, "fail to set storage to Default"
        self.api.uninstall_apps_by_adb("com.example.jizhenlo.runtimepermissiontest")


class Notification(UIATestBase):
    """
    @summary: test notification for profile owner
    """

    def setUp(self):
        super(Notification, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(Notification, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testPersonalProfileIndicatorDisplay(self):
        """
        verify personal profile icon will not show on notification bar
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.BLUETOOTH_SETTINGS", False)
        if self.api.check_ui_exists("text", "On"):
            self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
        igas_comp = igascomparator()
        temp_path = getTmpDir()
        result = 1.0
        xrate = [0.7, 0.75, 0.8, 0.85, 0.9]
        for _ in range(5):
            self.api.api_demo_po_launch()
            self.api.d(packageName="com.intel.afw.mdm").wait.exists(timeout=5000)
            x = self.api.d(resourceId="android:id/statusBarBackground").info['visibleBounds']['right']
            y = self.api.d(resourceId="android:id/statusBarBackground").info['visibleBounds']['bottom']
            box = (int(x * xrate[_]), 0, x, y)
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "with_indicator.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            self.api.api_demo_launch()
            self.api.d(packageName="com.intel.afw.mdm").wait.exists(timeout=5000)
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "without_indicator.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            result = float(igas_comp.getsimilarityrate(
                os.path.join(temp_path, "with_indicator.png"), os.path.join(temp_path, "without_indicator.png")))
            if result < 1.0:
                break
        assert result < 1.0, "fail to detect indicator icon"

    def testPersonalProfileUnlockToastDisplay(self):
        """
        verify personal profile toast doesn't display after unlock screen
        :return: None
        """
        igas_comp = igascomparator()
        temp_path = getTmpDir()
        result = 1.0
        self.api.set_lock_swipe()
        for _ in range(3):
            self.api.unlock_screen()
            self.api.launch_app_by_activity("com.google.android.deskclock/com.android.deskclock.DeskClock", True)
            if not self.api.check_ui_exists("packageName", "com.google.android.deskclock"):
                self.api.launch_app("Work Clock")
            self.api.d(packageName="com.google.android.deskclock").wait.exists(timeout=5000)
            x = self.api.d.info['displayWidth']
            y = self.api.d.info['displayHeight']
            box = (0, (y * 3 / 4), x, y)
            self.api.d.sleep()
            time.sleep(2)
            self.api.d.wakeup()
            # if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
            #     self.api.d(resourceId="com.android.systemui:id/scroll_view").swipe.up()
            # time.sleep(2)
            os.popen("adb -s {0} shell input keyevent 82".format(self.api.serial)).read().strip()
            self.api.d(packageName="com.google.android.deskclock").wait.exists(timeout=5000)
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "with_toast.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            time.sleep(5)
            self.api.launch_app_by_activity("com.google.android.deskclock/com.android.deskclock.DeskClock", False)
            if not self.api.check_ui_exists("packageName", "com.google.android.deskclock"):
                self.api.launch_app("Clock")
            self.api.d(packageName="com.google.android.deskclock").wait.exists(timeout=5000)
            self.api.d.sleep()
            time.sleep(2)
            self.api.d.wakeup()
            # if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
            #     self.api.d(resourceId="com.android.systemui:id/scroll_view").swipe.up()
            # time.sleep(2)
            os.popen("adb -s {0} shell input keyevent 82".format(self.api.serial)).read().strip()
            self.api.d(packageName="com.google.android.deskclock").wait.exists(timeout=5000)
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "without_toast.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            result = float(igas_comp.getsimilarityrate(
                os.path.join(temp_path, "with_toast.png"), os.path.join(temp_path, "without_toast.png")))
            if result < 1.0:
                break
        assert result < 1.0, "fail to detect toast message"

    def testWorkProfileIndicatorDisplay(self):
        """
        verify work profile icon will show on notification bar
        :return: None
        """
        self.api.launch_app_by_intents("android.settings.BLUETOOTH_SETTINGS", False)
        if self.api.check_ui_exists("text", "On"):
            self.api.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
        igas_comp = igascomparator()
        temp_path = getTmpDir()
        result = 1.0
        xrate = [0.7, 0.75, 0.8, 0.85, 0.9]
        for _ in range(5):
            self.api.api_demo_po_launch()
            self.api.d(packageName="com.intel.afw.mdm").wait.exists(timeout=5000)
            x = self.api.d(resourceId="android:id/statusBarBackground").info['visibleBounds']['right']
            y = self.api.d(resourceId="android:id/statusBarBackground").info['visibleBounds']['bottom']
            box = (int(x * xrate[_]), 0, x, y)
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "with_indicator.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            self.api.api_demo_launch()
            self.api.d(packageName="com.intel.afw.mdm").wait.exists(timeout=5000)
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "without_indicator.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            result = float(igas_comp.getsimilarityrate(
                os.path.join(temp_path, "with_indicator.png"), os.path.join(temp_path, "without_indicator.png")))
            if result < 1.0:
                break
        assert result < 1.0, "fail to detect indicator icon"

    def testWorkProfileUnlockToastDisplay(self):
        """
        verify work profile toast display after unlock screen
        :return: None
        """
        igas_comp = igascomparator()
        temp_path = getTmpDir()
        result = 1.0
        self.api.set_lock_swipe()
        for _ in range(3):
            self.api.unlock_screen()
            self.api.launch_app_by_activity("com.google.android.deskclock/com.android.deskclock.DeskClock", True)
            if not self.api.check_ui_exists("packageName", "com.google.android.deskclock"):
                self.api.launch_app("Work Clock")
            self.api.d(packageName="com.google.android.deskclock").wait.exists(timeout=5000)
            x = self.api.d.info['displayWidth']
            y = self.api.d.info['displayHeight']
            box = (0, (y * 3 / 4), x, y)
            self.api.d.sleep()
            time.sleep(2)
            self.api.d.wakeup()
            # if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
            #     self.api.d(resourceId="com.android.systemui:id/scroll_view").swipe.up()
            # time.sleep(2)
            os.popen("adb -s {0} shell input keyevent 82".format(self.api.serial)).read().strip()
            self.api.d(packageName="com.google.android.deskclock").wait.exists(timeout=5000)
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "with_toast.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            time.sleep(5)
            self.api.launch_app_by_activity("com.google.android.deskclock/com.android.deskclock.DeskClock", False)
            if not self.api.check_ui_exists("packageName", "com.google.android.deskclock"):
                self.api.launch_app("Clock")
            self.api.d(packageName="com.google.android.deskclock").wait.exists(timeout=5000)
            self.api.d.sleep()
            time.sleep(2)
            self.api.d.wakeup()
            # if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view):
            #     self.api.d(resourceId="com.android.systemui:id/scroll_view").swipe.up()
            # time.sleep(2)
            os.popen("adb -s {0} shell input keyevent 82".format(self.api.serial)).read().strip()
            self.api.d(packageName="com.google.android.deskclock").wait.exists(timeout=5000)
            self.api.d.screenshot(os.path.join(temp_path, "screenshot.png"))
            image = Image.open(os.path.join(temp_path, "screenshot.png"))
            image.crop(box).save(os.path.join(temp_path, "without_toast.png"))
            os.remove(os.path.join(temp_path, "screenshot.png"))
            result = float(igas_comp.getsimilarityrate(
                os.path.join(temp_path, "with_toast.png"), os.path.join(temp_path, "without_toast.png")))
            if result < 1.0:
                break
        assert result < 1.0, "fail to detect toast message"


