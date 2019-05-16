# -*- coding: utf-8 -*-

from testlib.common.common import g_common_obj2
from testlib.util.common import g_common_obj
from testlib.util.repo import Artifactory
from testlib.AfW.entity import MDM_Entity
from testlib.AfW.entity import Settings
from testlib.AfW.entity import Remote
from testlib.util.config import TestConfig
from testlib.dut_init.dut_init_impl import Function
import ConfigParser
import os
import time


class ApiImpl(object):
    """
    @summary: Intel Sample MDM API Implementation
    """

    def __init__(self):
        function = Function()
        function.push_uiautomator_jar()
        self.d = g_common_obj.get_device()
        self.serial = self.d.server.adb.device_serial()
        self.ui = MDM_Entity()
        self.settings = Settings()
        self.remote = Remote()
        self.config = TestConfig()
        if "afwHandleUnfortunately" not in self.d.watchers:
            self.d.watcher("afwHandleUnfortunately").when(textStartsWith="Unfortunately").click(text="OK")
        if "afwHandleResponding" not in self.d.watchers:
            self.d.watcher("afwHandleResponding").when(textContains="isn't responding").click(text="OK")
        self.d.watchers.run()

    def click_with_timeout(self, selector_key, selector_string, timeout=2):
        """
        wrapper for click action in uiautomator with timeout
        :param selector_key: key for ui selector
        :param selector_string: value for ui selector
        :param timeout: seconds for timeout, default value is 2s
        :return: True or False
        """
        kwargs = {selector_key: selector_string}
        for _ in range(timeout):  # watcher list will be invoked if uiselector fail to match one target
            if self.d(**kwargs).exists:  # so try more times
                self.d(**kwargs).click()
                # self.d.wait.update(timeout=5000)
                break
            time.sleep(1)

    def check_ui_exists(self, selector_key, selector_string, timeout=3):
        """
        wrapper for wait.exists in uiautomator with timeout(3s)
        :param selector_key: key for ui selector
        :param selector_string: value for ui selector
        :param timeout: default value is 3s
        :return: True or False
        """
        kwargs = {selector_key: selector_string}
        ret = False
        try:
            for _ in range(timeout):  # watcher list will be invoked if uiselector fail to match one target
                if self.d(**kwargs).exists:  # so try more times
                    ret = True
                    break
                time.sleep(1)
        finally:
            return ret

    def locate_apps(self, app_desc, retry=2):
        """
        locate application in apps launcher
        :param app_desc: description for app in apps launcher
        :param retry: retry several times to locate apps in launcher
        :return: True or False
        """
        if self.is_android_L_build():
            for i in range(retry):
                self.d.press.recent()
                self.d.wait.update(timeout=5000)
                self.d.press.home()
                self.d.wait.update(timeout=5000)
                self.click_with_timeout("description", "Apps")
                self.d.wait.update(timeout=5000)
                for _ in range(4):
                    if self.check_ui_exists("description", app_desc):
                        return True
                    self.d().swipe.left()
                    self.d.wait.update(timeout=5000)
            return False
        else:
            for i in range(retry):
                self.d.press.home()
                self.click_with_timeout("description", "Apps")
                self.d.wait.update(timeout=5000)
                if self.check_ui_exists("resourceId", "com.google.android.googlequicksearchbox:id/search_box_container"):
                    # os.popen("adb -s {0} shell input text {1}".format(
                    #     self.serial, app_desc[5:] if app_desc.startswith("Work ") else app_desc)).read().strip()
                    self.d(resourceId="com.google.android.googlequicksearchbox:id/search_box_container").set_text(
                        app_desc[5:] if app_desc.startswith("Work ") else app_desc)
                    if self.check_ui_exists("description", app_desc):
                        return True
            return False

    def launch_app(self, app_desc):
        """
        launch app from apps launcher
        :param app_desc: description for app in apps launcher
        :return: None
        """
        if self.locate_apps(app_desc):
            self.click_with_timeout("description", app_desc, 5)

    def clean_tasks(self):
        """
        clear recent tasks
        :return: None
        """
        self.d.press("home")
        self.d.wait.update(timeout=2000)
        self.d.press("recent")
        self.d.wait.update(timeout=2000)
        # wait for ~10 seconds and will quit if still doesn't have any recent apps
        if not self.check_ui_exists("resourceId", self.ui.dismiss_task, 5):
            self.d.press.home()
            return
        while self.check_ui_exists("resourceId", self.ui.dismiss_task):
            self.click_with_timeout("resourceId", self.ui.dismiss_task, 1)
            # just quit if back to home page
            if self.check_ui_exists("description", "Apps", 1):
                break
        self.d.press("home")

    def input_unlock_pin(self, pin_code):
        """
        input unlock pin code in lock screen ui
        :param pin_code: pin code to unlock screen
        :return: None
        """
        if self.check_ui_exists("resourceId", self.ui.lock_pin_pad):
            self.d(resourceId=self.ui.lock_pin_entry).set_text(str(pin_code))
            time.sleep(3)
            self.click_with_timeout("resourceId", self.ui.lock_pin_pad)

    def unlock_screen(self):
        """
        unlock the screen by key event
        :return: None
        """
        for i in range(3):
            self.d.wakeup()
            if self.check_ui_exists("resourceId", "com.android.systemui:id/lock_icon"):
                os.popen("adb -s {0} shell input keyevent 82".format(self.serial)).read().strip()
                self.d.wait.update(timeout=3000)
                if not self.check_ui_exists("resourceId", "com.android.systemui:id/pinEntry"):
                    g_common_obj.set_vertical_screen()
                    break
                os.popen("adb -s {0} shell input text 1234".format(self.serial)).read().strip()
                self.click_with_timeout("resourceId", "com.android.systemui:id/key_enter")
            else:
                break

    def get_userid_for_work_profile(self):
        """
        get user id for work profile owner
        :return: user's id for work profile owner
        """
        all_users = os.popen("adb -s {0} shell pm list users".format(self.serial)).read().strip()
        if repr(all_users).find("Work profile") != -1:
            for user in all_users.replace(" ", "").split():
                if user.find("Workprofile") != -1:
                    return int(user.split(":")[0][9:])
        elif repr(all_users).find("HiProfileOwner") != -1:
            for user in all_users.replace(" ", "").split():
                if user.find("HiProfileOwner") != -1:
                    return int(user.split(":")[0][9:])
        else:
            return 10

    def launch_app_by_intents(self, intent_string, profile=True):
        """
        launch apps by intent string via adb command
        :param intent_string: intent string for activities
        :param profile: True or False, default is True to indicate that apps under control of work profile
        :return: None
        """
        os.popen("adb -s {0} root".format(self.serial)).read().strip()
        time.sleep(5)
        if profile:
            user_id = self.get_userid_for_work_profile()
            command = "adb -s {0} shell am start -a {1} --user {2}".format(self.serial, intent_string, str(user_id))
        else:
            command = "adb -s {0} shell am start -a {1}".format(self.serial, intent_string)
        return_string = repr(os.popen(command).read().strip())
        assert return_string.find("Error") == -1, "[ERROR]: fail to launch app by intent :" + return_string
        self.d.wait.update(timeout=3000)

    def launch_app_by_activity(self, activity, profile=True):
        """
        launch apps by activity via adb command
        :param activity: activity string
        :param profile: True or False, default is True to indicate that apps under control of work profile
        :return: None
        """
        os.popen("adb -s {0} root".format(self.serial)).read().strip()
        if profile:
            user_id = self.get_userid_for_work_profile()
            command = "adb -s {0} shell am start --user {1} {2}".format(self.serial, str(user_id), activity)
        else:
            command = "adb -s {0} shell am start {1}".format(self.serial, activity)
        for _ in range(3):
            return_string = repr(os.popen(command).read().strip())
            if return_string.find("Error") == -1:
                return True
            time.sleep(5)
        return False

    def set_lock_pin(self):
        """
        set pin code for lock screen, the default pin code is "1234"
        :return: None
        """
        self.launch_app_by_intents("android.settings.SECURITY_SETTINGS", False)
        # return if PIN code was already set
        if self.check_ui_exists("text", self.settings.screen_lock) and self.check_ui_exists("text", "PIN"):
            return
        self.click_with_timeout("text", self.settings.screen_lock)
        self.click_with_timeout("text", self.settings.lock_pin)
        self.click_with_timeout("text", "Continue")
        if self.check_ui_exists("resourceId", self.settings.password_entry):
            self.d(resourceId=self.settings.password_entry).set_text("1234")
            time.sleep(3)
            self.click_with_timeout("text", "Continue")
            self.d(resourceId=self.settings.password_entry).set_text("1234")
            time.sleep(3)
            self.click_with_timeout("text", "OK")
            for _ in range(5):  # for some device, sometimes VKB respond little slowly, block Done button
                if self.check_ui_exists("text", "Done"):
                    break
                time.sleep(3)
            self.click_with_timeout("text", "Done")
        self.d.press.home()

    def set_lock_swipe(self):
        """
        change lock screen from pin to swipe
        :return: None
        """
        self.launch_app_by_intents("android.settings.SECURITY_SETTINGS", False)
        if not self.check_ui_exists("text", self.settings.screen_lock):
            self.clean_tasks()
            self.settings_sub_launch("Security")
            self.d(scrollable=True).scroll.vert.to(text="Screen lock")
        if self.check_ui_exists("text", "PIN"):
            self.click_with_timeout("text", self.settings.screen_lock)
            if self.check_ui_exists("text", "Confirm your PIN"):
                self.d(resourceId=self.settings.password_entry).set_text("1234")
                time.sleep(3)
                if self.check_ui_exists("text", "Next"):
                    self.click_with_timeout("text", "Next")
                elif self.check_ui_exists("text", "Continue"):
                    self.click_with_timeout("text", "Continue")
                elif self.check_ui_exists("text", "Confirm your PIN"):
                    self.d.press.enter()
            time.sleep(3)
            self.click_with_timeout("text", "Swipe")
            self.click_with_timeout("text", "OK")
            self.click_with_timeout("textContains", "Yes")
            self.d.press.home()

    def unknown_source_control(self, action=True):
        """
        turn on or off the unknown source control in settings
        :param action: True or False, default is True to turn on unknown source control
        :return: None
        """
        self.launch_app_by_intents("android.settings.SECURITY_SETTINGS", False)
        if not self.check_ui_exists("text", self.settings.unknown_source):
            self.d(scrollable=True).scroll.vert.to(text="Unknown sources")
        if action:
            if self.d(text=self.settings.unknown_source).right(resourceId=self.settings.switch_widget).checked:
                print "[INFO]: Already turned on"
            else:
                self.d(text=self.settings.unknown_source).right(resourceId=self.settings.switch_widget).click()
                self.click_with_timeout("text", "OK")
        else:
            if self.d(text=self.settings.unknown_source).right(resourceId=self.settings.switch_widget).checked:
                self.d(text=self.settings.unknown_source).right(resourceId=self.settings.switch_widget).click()
            else:
                print "[INFO]: Already turned off"
        self.d.press("home")

    def install_apps(self, package_name, app_name):
        """
        install apps by adb command
        :param package_name: package string for apps, e.g. package:foo.bar.testback
        :param app_name: apps name string, e.g. TestBack2
        :return: None
        """
        if package_name.startswith("package:"):
            pkg_name = package_name[8:]
        else:
            pkg_name = package_name
        check_result = os.popen("adb -s {0} shell pm list packages {1}".format(self.serial, pkg_name)).read().strip()
        if check_result == package_name:
            self.uninstall_apps_by_adb(package_name)
        os.popen("adb -s {0} install -r {1} & > /dev/null 2>&1".format(self.serial, (app_name + ".apk")))
        for _ in range(20):
            self.unlock_screen()
            check_result = os.popen("adb -s {0} shell pm list packages {1}".format(self.serial, pkg_name)).read().strip()
            if check_result == package_name:
                print "[INFO]: install package successfully"
                break
            if self.check_ui_exists("text", "ACCEPT"):
                self.click_with_timeout("text", "ACCEPT")

    def uninstall_apps_by_adb(self, package_name):
        """
        uninstall apps by adb command
        :param package_name: package string for apps, e.g. package:foo.bar.testback
        :return: None
        """
        if package_name.startswith("package:"):
            pkg_name = package_name[8:]
        else:
            pkg_name = package_name
        return_string = os.popen("adb -s {0} uninstall {1}".format(self.serial, pkg_name)).read().strip()
        if return_string == "Success":
            print "[INFO]: uninstall apps successfully"
        else:
            print "[ERROR]: fail to uninstall apps: " + return_string

    def download_file_from_artifactory(self, sub_path=None, download_file=None):
        """
        download file from artifcatory server
        :param sub_path: sub location for download file in server
        :param download_file: file to download
        :return: local location of downloaded file or None
        """
        if os.path.isfile("/etc/oat/sys.conf"):
            common_url = self.config.read(section='artifactory').get("location")
            remote_server = Artifactory(common_url + sub_path)
            for _ in range(5):
                return_file = remote_server.get(download_file)
                if return_file is not None and os.path.isfile(return_file):
                    return return_file
                time.sleep(2)
        # try to download file from mirror server
        # mirror_url = self.config.read('tests.tablet.dut_init.conf', 'download_server').get('mirror_url')
        # mirror_server = Artifactory(mirror_url + sub_path)
        # for _ in range(5):
        #     return_file = mirror_server.get(download_file)
        #     if return_file is not None and os.path.isfile(return_file):
        #         return return_file
        #     time.sleep(2)
        return None

    def api_demo_launch(self):
        """
        launch Intel Sample MDM
        :return: None
        """
        os.popen("adb -s {0} shell am force-stop com.intel.afw.mdm".format(self.serial)).read()
        self.launch_app_by_activity(self.ui.intel_mdm, False)
        self.d(packageName="com.intel.afw.mdm").wait.exists(timeout=5000)
        if not self.check_ui_exists("textContains", "Sample MDM"):
            self.clean_tasks()
            self.launch_app("Sample MDM")

    def api_demo_po_launch(self):
        """
        launch Intel Sample MDM which under work profile
        :return: None
        """
        os.popen("adb -s {0} shell am force-stop com.intel.afw.mdm".format(self.serial)).read()
        self.launch_app_by_activity(self.ui.intel_mdm, True)
        self.d(packageName="com.intel.afw.mdm").wait.exists(timeout=5000)
        if not self.check_ui_exists("textContains", "Sample MDM"):
            self.clean_tasks()
            self.launch_app("Work Sample MDM")

    def settings_launch(self):
        """
        launch settings
        :return: None
        """
        os.popen("adb -s {0} shell am force-stop com.android.settings".format(self.serial)).read()
        self.launch_app_by_activity(self.ui.settings, False)
        self.d(packageName="com.android.settings").wait.exists(timeout=5000)
        if not self.check_ui_exists("text", "Settings"):
            self.clean_tasks()
            self.launch_app("Settings")

    def settings_sub_launch(self, sub_settings):
        """
        launch sub item in settings
        :param sub_settings: string for sub settings
        :return: None
        """
        self.settings_launch()
        if self.check_ui_exists("text", sub_settings):
            self.click_with_timeout("text", sub_settings)
        # to launch Wi-Fi
        elif self.check_ui_exists("textMatches", sub_settings):
            self.click_with_timeout("textMatches", sub_settings)
        elif not self.check_ui_exists("text", sub_settings):
            self.d(scrollable=True).scroll.vert.to(text=sub_settings)
            self.click_with_timeout("text", sub_settings)
        
    def setup_managed_profile(self, profile=True):
        """
        Setup a work profile by Intel Sample MDM
        :param profile: True for setup profile under not provision mode, False for create profile under device owner
        :return: None
        """
        self.api_demo_launch()
        if profile:
            self.click_with_timeout("resourceId", self.ui.setup_managed_profile)
        else:
            self.click_with_timeout("resourceId", self.ui.create_and_delete_profile)
        # if not self.is_android_L_build():
        self.click_with_timeout("text", "No", 5)
        self.click_with_timeout("text", "Set up", 10)
        if self.check_ui_exists("text", "Remove"):
            print "[INFO] Managed profile has been provisioned."
            self.clean_tasks()
        else:
            self.click_with_timeout("text", "OK")
            self.click_with_timeout("text", "OK")
            self.check_ui_exists("text", "Setup all done", 30)
            self.d.press("home")

    def remove_managed_profile(self, profile=True):
        """
        Remove work profile by Intel Sample MDM
        :param profile: True for remove managed profile under not provision mode, False to delete profile
        :return: None
        """
        if self.is_android_L_build():
            self.api_demo_launch()
            if profile:
                self.click_with_timeout("resourceId", self.ui.setup_managed_profile)
            else:
                self.click_with_timeout("resourceId", self.ui.create_and_delete_profile)
            # assert self.check_ui_exists("text", "Remove"), "Device is not managed profile provisioned"
            self.click_with_timeout("text", "Remove")
            self.click_with_timeout("text", "Remove")
        else:
            # self.launch_app_by_intents("android.settings.SETTINGS", False)
            # if not self.check_ui_exists("text", "Accounts"):
            #     self.d(scrollable=True).scroll.vert.to(text="Accounts")
            # self.click_with_timeout("text", "Accounts")
            self.launch_app_by_intents("android.settings.SYNC_SETTINGS", False)
            if not self.check_ui_exists("text", "Accounts"):
                self.settings_sub_launch("Accounts")
            if self.check_ui_exists("text", "Work", 5):
                self.click_with_timeout("text", "Remove work profile")
                self.click_with_timeout("text", "Delete", 10)

    def set_profile_name(self, profile_name, profile=True):
        """
        Set profile name by Intel Sample MDM
        :param profile_name: profile name string
        :param profile: True or False, default is True to indicate that set the profile name for Profile Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_provision)
        self.click_with_timeout("resourceId", self.ui.set_profile_name)
        if self.check_ui_exists("resourceId", self.ui.profile_input_box, 5):
            self.d(resourceId=self.ui.profile_input_box).set_text(profile_name)
            self.click_with_timeout("text", "OK")

    def check_profile_owner(self, profile=True):
        """
        check Intel Sample MDM whether is the profile owner
        :param profile: True or False, default is True to indicate that check it under profile owner
        :return: True or False
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_provision)
        self.click_with_timeout("resourceId", self.ui.check_profile_app)
        self.click_with_timeout("text", "com.intel.afw.mdm")
        self.click_with_timeout("text", "Yes")
        if self.check_ui_exists("textContains", "is Profile Owner"):
            return True
        else:
            return False

    def check_device_owner(self, profile=True):
        """
        check Intel Sample MDM whether is the device owner
        :param profile: True or False, default is True to indicate that check it under device owner
        :return: True or False
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_provision)
        self.click_with_timeout("resourceId", self.ui.check_device_app)
        self.click_with_timeout("text", "com.intel.afw.mdm")
        self.click_with_timeout("text", "Yes")
        if self.check_ui_exists("textContains", "is Device Owner"):
            return True
        else:
            return False

    def create_user_in_mdm(self, user_name):
        """
        create user in Intel Sample MDM
        :param user_name: user name string
        :return: None
        """
        self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_provision)
        self.click_with_timeout("resourceId", self.ui.create_user)
        if self.check_ui_exists("resourceId", self.ui.user_name_input):
            self.d(resourceId=self.ui.user_name_input).set_text(user_name)
            self.click_with_timeout("text", "OK")
            self.click_with_timeout("text", "Yes")
        if self.check_ui_exists("text", "Yes"):
            self.d.press.back()

    def create_and_initialize_user(self, user_name):
        """
        create and initialize user in Intel Sample MDM
        :param user_name: user name string
        :return: None
        """
        self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_provision)
        self.click_with_timeout("resourceId", self.ui.create_and_init_user)
        if self.check_ui_exists("resourceId", self.ui.user_name_input):
            self.d(resourceId=self.ui.user_name_input).set_text(user_name)
            self.click_with_timeout("text", "OK")
            self.click_with_timeout("text", "Yes")
        if self.check_ui_exists("text", "Yes"):
            self.d.press.back()

    def remove_user_in_mdm(self, user_name):
        """
        remove user in Intel Sample MDM
        :param user_name: user name string
        :return: None
        """
        self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_provision)
        self.click_with_timeout("resourceId", self.ui.remove_user)
        if self.check_ui_exists("text", user_name):
            self.click_with_timeout("text", user_name)
            self.click_with_timeout("text", "Remove User")
            self.click_with_timeout("text", "Yes")
        if self.check_ui_exists("text", "Yes"):
            self.d.press.back()

    def clear_device_owner_app(self):
        """
        clear device owner provision
        :return: None
        """
        self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_provision)
        self.click_with_timeout("resourceId", self.ui.clear_device_owner_app)
        self.click_with_timeout("text", "Yes")

    def enable_system_applications(self, profile=True):
        """
        enable system applications by Intel Sample MDM
        :param profile: True or False, default is True to indicate that enable apps for profile owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.app_provision)
        self.click_with_timeout("resourceId", self.ui.enable_system_apps)
        if self.check_ui_exists("text", "Pick Applications to Unhide"):
            self.click_with_timeout("text", self.ui.select_all)
            self.click_with_timeout("text", "Enable System Applications to Profile")
            time.sleep(5)
        else:
            print "[INFO]: system applications have been enabled"

    def hide_applications(self, profile=True):
        """
        hide applications by Intel Sample MDM
        :param profile: True or False, default is True to indicate that hide apps for profile owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.app_provision)
        self.d(resourceId=self.ui.hide_apps).wait.exists(timeout=5000)
        self.click_with_timeout("resourceId", self.ui.hide_apps)
        self.click_with_timeout("text", "OK")
        self.d(text="Select All").wait.exists(timeout=5000)
        self.click_with_timeout("text", self.ui.select_all)
        self.click_with_timeout("text", "Hide Apps")

    def unhide_applications(self, profile=True):
        """
        list hidden applications by Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.app_provision)
        self.d(resourceId=self.ui.list_hidden_apps).wait.exists(timeout=5000)
        self.click_with_timeout("resourceId", self.ui.list_hidden_apps)
        self.d(text="Select All").wait.exists(timeout=5000)
        self.click_with_timeout("text", self.ui.select_all)
        self.click_with_timeout("text", "Unhide Apps")

    def block_uninstall_apps(self, profile=True):
        """
        block uninstalling applications by Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.app_provision)
        self.click_with_timeout("resourceId", self.ui.block_uninstall_apps)
        if self.check_ui_exists("text", self.ui.select_all):
            self.click_with_timeout("text", self.ui.select_all)
            self.click_with_timeout("text", "Block Uninstalling")
        else:
            print "[INFO]: Cannot find proper installed apps"

    def unblock_uninstall_apps(self, profile=True):
        """
        unblock uninstalling applications by Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.app_provision)
        self.click_with_timeout("resourceId", self.ui.blocked_uninstall_apps)
        if self.check_ui_exists("text", self.ui.select_all):
            self.click_with_timeout("text", self.ui.select_all)
            self.click_with_timeout("text", "Disable Block Uninstalling")
        else:
            print "[INFO]: Cannot find proper installed apps"

    def send_intent_to_handle(self, profile=True):
        """
        send intent to handle text/plain in Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.intents_sharing)
        self.click_with_timeout("resourceId", self.ui.send_intent_to_handle)

    def add_x_intent_filter(self, profile=True):
        """
        add cross X profile intent filter in Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.intents_sharing)
        self.click_with_timeout("resourceId", self.ui.add_x_intent_filter)
        self.click_toast_window()

    def clear_x_intent_filter(self, profile=True):
        """
        clear cross X profile intent filter in Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.intents_sharing)
        self.click_with_timeout("resourceId", self.ui.clear_x_intent_filter)
        self.click_toast_window()

    def add_persistent_activities(self, profile=True):
        """
        add persistent activities in Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.intents_sharing)
        self.click_with_timeout("resourceId", self.ui.add_persistent_preferred)
        self.d(text="Copy to clipboard").wait.exists(timeout=3000)
        self.click_with_timeout("text", "Copy to clipboard")
        self.click_with_timeout("text", "Pick APP")

    def clear_persistent_activities(self, profile=True):
        """
        clear persistent activities in Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.intents_sharing)
        self.click_with_timeout("resourceId", self.ui.clear_persistent_preferred)
        self.click_toast_window()

    def add_x_profile_widget_provider(self, profile=True):
        """
        add cross X profile widget provider by Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        self.hide_applications(profile)
        self.d.press("back")
        self.click_with_timeout("resourceId", self.ui.intents_sharing)
        self.click_with_timeout("resourceId", self.ui.add_x_widget)
        self.d(text="Select All").wait.exists(timeout=5000)
        self.click_with_timeout("text", self.ui.select_all)
        self.click_with_timeout("text", "process")
        self.d.press("back")
        self.unhide_applications(profile)
        self.d.press("home")
        time.sleep(5)
        self.d.press.menu()
        self.d(text="Widgets").wait.exists(timeout=5000)
        if not self.check_ui_exists("text", "Widgets"):
            os.popen("adb -s {0} shell input keyevent 82".format(self.serial)).read().strip()
            self.d(text="Widgets").wait.exists(timeout=5000)
        self.click_with_timeout("text", "Widgets")
        # if self.is_android_L_build():
        #     assert self.check_ui_exists("resourceId", self.ui.widget_pages), "[ERROR]: fail to launch widget page"
        # else:
        #     assert self.check_ui_exists("resourceId", "com.google.android.googlequicksearchbox:id/widgets_list_view"), \
        #         "[ERROR]: fail to detect widget page in launcher"

    def remove_x_profile_widget_provider(self, profile=True):
        """
        clear cross X profile widget provider by Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        self.hide_applications(profile)
        self.d.press("back")
        self.click_with_timeout("resourceId", self.ui.intents_sharing)
        self.click_with_timeout("resourceId", self.ui.remove_x_widget)
        self.d(text="Select All").wait.exists(timeout=5000)
        self.click_with_timeout("text", self.ui.select_all)
        self.click_with_timeout("text", "process")
        self.d.press("back")
        self.unhide_applications(profile)
        self.d.press("home")
        time.sleep(5)
        self.d.press.menu()
        self.d(text="Widgets").wait.exists(timeout=5000)
        if not self.check_ui_exists("text", "Widgets"):
            os.popen("adb -s {0} shell input keyevent 82".format(self.serial)).read().strip()
            self.d(text="Widgets").wait.exists(timeout=5000)
        self.click_with_timeout("text", "Widgets")
        # if self.is_android_L_build():
        #     assert self.check_ui_exists("resourceId", self.ui.widget_pages), \
        #         "[ERROR]: fail to launch widget page"
        # else:
        #     assert self.check_ui_exists("resourceId", "com.google.android.googlequicksearchbox:id/widgets_list_view"), \
        #         "[ERROR]: fail to detect widget page in launcher"

    def get_x_profile_widget_provider(self):
        """
        get current enabled cross X profile widget provider by Intel Sample MDM
        :return: None
        """
        self.api_demo_po_launch()
        self.click_with_timeout("resourceId", self.ui.intents_sharing)
        if not self.check_ui_exists("resourceId", self.ui.get_x_widget):
            self.d(scrollable=True).scroll.vert.to(resourceId=self.ui.get_x_widget)
        self.click_with_timeout("resourceId", self.ui.get_x_widget)
        assert self.check_ui_exists("text", "Pick APP"), "[ERROR]: pick app window doesn't pop up"

    def send_intent_to_audio_effect(self, profile=True):
        """
        launch audio effect control by sending intent in Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.intents_sharing)
        if not self.check_ui_exists("resourceId", self.ui.send_intent_to_audio_effect):
            self.d(scrollable=True).scroll.vert.to(resourceId=self.ui.send_intent_to_audio_effect)
        self.click_with_timeout("resourceId", self.ui.send_intent_to_audio_effect)

    def send_intent_to_speech_recognize(self, profile=True):
        """
        launch speech recognize by sending intent in Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.intents_sharing)
        self.click_with_timeout("resourceId", self.ui.send_intent_to_speech)

    def clear_apps_restriction_chrome(self, profile=True):
        """
        clear application restriction policy for chrome
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.app_config_policy)
        for _ in range(3):
            if self.check_ui_exists("textContains", self.ui.set_chrome_restriction):
                self.click_with_timeout("textContains", self.ui.set_chrome_restriction)
                self.click_with_timeout("text", "OK")
            if self.check_ui_exists("textContains", self.ui.clear_chrome_restriction):
                self.click_with_timeout("textContains", self.ui.clear_chrome_restriction)
                self.click_with_timeout("text", "OK")
                break

    def set_apps_restriction_chrome(self, profile=True):
        """
        set application restriction policy for chrome
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.app_config_policy)
        if self.check_ui_exists("textContains", self.ui.clear_chrome_restriction):
            self.click_with_timeout("textContains", self.ui.clear_chrome_restriction)
            self.click_toast_window()
        self.click_with_timeout("textContains", self.ui.set_chrome_restriction)
        self.click_toast_window()

    def get_apps_restriction_chrome(self, profile=True):
        """
        get application restriction policy for chrome
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.app_config_policy)
        for _ in range(5):
            self.click_with_timeout("resourceId", self.ui.get_chrome_restriction)
            if self.check_ui_exists("text", "Restrictions"):
                break
            time.sleep(5)

    def set_get_screen_capture(self, state, profile=True):
        """
        set or get screen capture state by Intel Sample MDM
        :param state: True or False, True for enable and False for disable
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_management_policy)
        self.click_with_timeout("resourceId", self.ui.screen_capture_state)
        if self.check_ui_exists("text", self.ui.disable_screen_capture, 2) and not state:
            self.click_with_timeout("text", self.ui.disable_screen_capture)
        elif self.check_ui_exists("text", self.ui.enable_screen_capture, 2) and state:
            self.click_with_timeout("text", self.ui.enable_screen_capture)

    def set_user_restrictions(self, restriction_string, profile=True):
        """
        set user restrictions in Intel Sample MDM
        :param restriction_string: restriction string
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_management_policy)
        self.click_with_timeout("resourceId", self.ui.set_user_restrictions)
        if not self.check_ui_exists("text", restriction_string) and \
                self.d(className="android.widget.ScrollView").scrollable:
            self.d(className="android.widget.ScrollView").scroll.to(text=restriction_string)
        self.click_with_timeout("text", restriction_string)
        self.click_with_timeout("text", "OK")

    def disable_account_management(self, is_add, profile=True):
        """
        disable account management by Intel Sample MDM
        :param is_add: True or False, True for add account and False for delete exist account
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_management_policy)
        self.click_with_timeout("resourceId", self.ui.disable_account_management)
        if is_add:
            self.click_with_timeout("text", "ADD")
            if self.check_ui_exists("className", self.ui.account_type_input):
                self.d(className=self.ui.account_type_input).set_text("com.google")
                self.click_with_timeout("text", "OK")
            self.click_with_timeout("text", "Exit")
        else:
            self.click_with_timeout("text", self.ui.select_all)
            self.click_with_timeout("text", "DEL")
            self.click_with_timeout("text", "Exit")

    def set_clear_auto_time_required(self, is_set):
        """
        set or clear auto time required in Intel Sample MDM
        :param is_set: True for set and False for clear
        :return: None
        """
        self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_management_policy)
        self.click_with_timeout("resourceId", self.ui.set_clear_auto_time)
        if self.check_ui_exists("text", self.ui.set_auto_time) and is_set:
            self.click_with_timeout("text", self.ui.set_auto_time)
        elif self.check_ui_exists("text", self.ui.clear_auto_time) and not is_set:
            self.click_with_timeout("text", self.ui.clear_auto_time)

    def enable_mdm_to_task_lock(self):
        """
        enable sample mdm into task lock mode
        :return: None
        """
        self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_management_policy)
        self.click_with_timeout("resourceId", self.ui.set_lock_task)
        if self.d(resourceId="android:id/list").scrollable:
            self.d(resourceId="android:id/list").scroll.vert.to(text="Sample MDM")
            # if not self.check_ui_exists("text", "Contacts"):
            #     self.d(resourceId="android:id/list").scroll.vert.to(text="Contacts")
        assert self.check_ui_exists("text", "Sample MDM"), "[ERROR]: Sample MDM doesn't exist in list of lock task"
        if self.d(text="Sample MDM").right(resourceId="com.intel.afw.mdm:id/task_lock_state").text == u'OFF':
            self.click_with_timeout("text", "Sample MDM")
        self.click_with_timeout("resourceId", self.ui.lock_task_mode)
        self.click_with_timeout("text", "OK")

    def set_global_secure_settings(self, global_settings):
        """
        change global and secure settings in Intel Sample MDM
        :param global_settings: string for global settings
        :return: None
        """
        self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_management_policy)
        self.click_with_timeout("resourceId", self.ui.global_setting)
        if not self.check_ui_exists("text", global_settings):
            self.d(scrollable=True).scroll.vert.to(text=global_settings)
        self.click_with_timeout("text", global_settings)
        self.click_with_timeout("text", "OK")

    def change_global_spinner_settings(self, spinner):
        """
        change the settings for spinner in set global or secure settings page
        :param spinner: resource id for spinner
        :return: None
        """
        self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.device_management_policy)
        self.click_with_timeout("resourceId", self.ui.global_setting)
        if not self.check_ui_exists("resourceId", spinner):
            self.d(scrollable=True).scroll.vert.to(resourceId=spinner)
        self.click_with_timeout("resourceId", spinner)

    def install_ca_cert(self, profile=True):
        """
        install exist CA Cert file
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.vpn_management)
        for _ in range(5):
            self.click_with_timeout("resourceId", self.ui.install_ca_cert)
            if self.check_ui_exists("text", "OK"):
                self.click_with_timeout("text", "OK")
                break
            if self.is_android_L_build():
                self.d(text="key_alpha.cer").wait.exists(timeout=5000)
            else:
                self.d(text="mytestcert3.cer").wait.exists(timeout=5000)
            self.click_with_timeout("text", self.ui.select_all)
            self.click_with_timeout("text", "Install CA Cert")

    def uninstall_ca_cert(self, profile=True):
        """
        uninstall exist CA Cert file
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.vpn_management)
        for _ in range(5):
            self.click_with_timeout("resourceId", self.ui.uninstall_ca_cert)
            if self.check_ui_exists("text", "Attention"):
                self.click_with_timeout("text", "OK")
                self.clean_tasks()
                self.install_ca_cert(profile)
                self.click_with_timeout("resourceId", self.ui.uninstall_ca_cert)
            self.d(text="Select All").wait.exists(timeout=5000)
            self.click_with_timeout("text", "Select All")
            if self.check_ui_exists("text", "Uninstall CA Cert"):
                self.click_with_timeout("text", "Uninstall CA Cert")
                break

    def check_ca_cert(self, profile=True):
        """
        check a CA cert whether is installed
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.vpn_management)
        for _ in range(5):
            self.click_with_timeout("resourceId", self.ui.check_ca_cert)
            self.d(text="Select a CA certs for check").wait.exists(timeout=5000)
            if self.is_android_L_build():
                self.click_with_timeout("text", "key_alpha.cer")
            else:
                self.click_with_timeout("text", "mytestcert3.cer")
            self.click_with_timeout("text", "Installed?")
            if self.check_ui_exists("text", "Check result"):
                break

    def get_installed_ca_cert(self, profile=True):
        """
        list all installed CA Cert files
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.vpn_management)
        self.click_with_timeout("resourceId", self.ui.get_installed_ca_cert)

    def uninstall_all_ca_cert(self, profile=True):
        """
        uninstall all installed CA Cert files
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.vpn_management)
        for _ in range(5):
            self.click_with_timeout("resourceId", self.ui.uninstall_all_user_ca_cert)
            if self.check_ui_exists("text", "Attention"):
                self.click_with_timeout("text", "OK")
                self.clean_tasks()
                self.install_ca_cert(profile)
                self.click_with_timeout("resourceId", self.ui.uninstall_all_user_ca_cert)
            self.d(textContains="Do You Want To Uninstall All User").wait.exists(timeout=5000)
            if self.check_ui_exists("text", "Yes"):
                self.click_with_timeout("text", "Yes")
                break

    def third_party_app_action(self, action, profile=True):
        """
        set or get input method/accessibility service in Intel Sample MDM
        :param action: action string: SetInput/GetInput/SetAccessibility/GetAccessibility
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.third_party)
        if action == "SetInput":
            self.click_with_timeout("resourceId", self.ui.set_input_method)
            self.d.wait.update(timeout=3000)
        elif action == "GetInput":
            self.click_with_timeout("resourceId", self.ui.get_input_method)
            self.d.wait.update(timeout=3000)
        elif action == "SetAccessibility":
            self.click_with_timeout("resourceId", self.ui.set_accessibility_service)
            self.d.wait.update(timeout=3000)
        elif action == "GetAccessibility":
            self.click_with_timeout("resourceId", self.ui.get_accessibility_service)
            self.d.wait.update(timeout=3000)
        else:
            print "[INFO]: wrong action"

    def wipe_all_data_for_current_user(self, action, profile=True):
        """
        wipe all data for current user in Intel Sample MDM
        :param action: action string, includeSD/excludeSD/withFRP
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.legacy_policy)
        if action == "includeSD":
            self.click_with_timeout("resourceId", self.ui.wipe_include_sd)
        elif action == "excludeSD":
            self.click_with_timeout("resourceId", self.ui.wipe_exclude_sd)
        elif action == "withFRP":
            self.click_with_timeout("resourceId", self.ui.wipe_with_frp)
        else:
            print "[INFO]: wrong action"
            return
        assert self.check_ui_exists("textContains", "This action will wipe all data"), \
            "[ERROR]: caution window doesn't pop up"
        self.click_with_timeout("text", "OK")

    def set_get_camera_state(self, state, profile=True):
        """
        set or get camera state in Intel Sample MDM
        :param state: True or False, False for disable and True for enable
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.legacy_policy)
        self.click_with_timeout("resourceId", self.ui.set_camera_state)
        if self.check_ui_exists("text", "Disable") and not state:
            self.click_with_timeout("text", "Disable")
        elif self.check_ui_exists("text", "Enable") and state:
            self.click_with_timeout("text", "Enable")

    def lock_now_from_mdm(self, profile=True):
        """
        just lock screen from Intel Sample MDM
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.legacy_policy)
        self.click_with_timeout("text", "LOCK NOW")

    def get_password_info(self, profile=True):
        """
        get inputting password information in lock screen
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.legacy_policy)
        self.click_with_timeout("resourceId", self.ui.get_password_info)

    def set_keyguard_disabled_features(self, disabled_feature, profile=False):
        """
        set key guard disabled feature by Intel Sample MDM
        :param disabled_feature: disable feature string
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()

        self.click_with_timeout("resourceId", self.ui.legacy_policy)
        self.click_with_timeout("resourceId", self.ui.set_keyguard_disable_feature)
        self.click_with_timeout("text", disabled_feature)
        self.click_with_timeout("text", "OK")

    def oobe_setup(self, provision=False):
        """
        setup device owner provision during OOBE
        :param provision: True or False, True for provision
        :return: None
        """
        if self.is_android_L_build():
            mdm_apk = self.download_file_from_artifactory(self.remote.sample_mdm['sub_path'],
                                                          self.remote.sample_mdm['name'])
        else:
            mdm_apk = self.download_file_from_artifactory(self.remote.sample_mdm['sub_path'],
                                                          self.remote.sample_mdm['name_m'])
        ADB_DEVICE_OWNER_CMD="adb -s {0} shell am start -a com.android.managedprovisioning.ACTION_PROVISION_MANAGED_DEVICE" \
                             " --es android.app.extra.PROVISIONING_DEVICE_ADMIN_PACKAGE_NAME com.intel.afw.mdm"

        if self.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"):
            if not self.check_ui_exists("textContains", "English (United States)"):
                self.d(scrollable=True).scroll.vert.to(textContains="English (United States)")
            if provision and mdm_apk is not None:
                old_dir = os.getcwd()
                os.chdir(os.path.split(mdm_apk)[0])
                if self.is_android_L_build():
                    self.install_apps(self.remote.sample_mdm['pkg_name'],
                                      self.remote.sample_mdm['name'].split('.apk')[0])
                else:
                    self.install_apps(self.remote.sample_mdm['pkg_name'],
                                      self.remote.sample_mdm['name_m'].split('.apk')[0])
                self.click_with_timeout("text", "ACCEPT")
                os.chdir(old_dir)
                os.popen(ADB_DEVICE_OWNER_CMD.format(self.serial)).read()
                self.check_ui_exists("resourceId", "com.android.managedprovisioning:id/learn_more_text1", 10)
                self.click_with_timeout("text", "Set up")
                self.click_with_timeout("text", "OK")
                self.click_with_timeout("text", "OK")
                time.sleep(20)
                if self.check_ui_exists("text", "Set up phone", 5):
                    g_common_obj2.system_reboot(90)
                    for i in range(20):
                        self.d = g_common_obj.get_device()
                        self.d.wakeup()
                        if self.check_ui_exists("resourceId", self.ui.lock_clock_view) or \
                                self.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"):
                            break
                        time.sleep(10)
                    g_common_obj.set_vertical_screen()
                    self.unlock_screen()
            else:
                self.click_with_timeout("resourceId", "com.google.android.setupwizard:id/start")
        if not provision:
            if self.check_ui_exists("textMatches", "Select Wi.*Fi network"):
                self.click_with_timeout("text", "Skip")
                self.click_with_timeout("text", "Skip anyway")
            if self.check_ui_exists("text", "Date & time"):
                self.click_with_timeout("text", "Next")
            if self.check_ui_exists("text", "Name"):
                self.click_with_timeout("text", "Next")
            if self.check_ui_exists("text", "Protect your phone"):
                self.click_with_timeout("text", "Skip")
                self.click_with_timeout("text", "Skip anyway")
            if self.check_ui_exists("text", "Google services"):
                self.click_with_timeout("text", "More")
                self.click_with_timeout("text", "Next")
        for _ in range(5):
            self.unlock_screen()
            if self.check_ui_exists("text", "Data Reporting"):
                self.click_with_timeout("text", "Allow")
                break
            time.sleep(2)
        self.d.press.home()
        for _ in range(5):
            self.unlock_screen()
            if self.check_ui_exists("text", "GOT IT", 5):
                self.click_with_timeout("text", "GOT IT")
                break
            time.sleep(2)
        # if not self.is_android_L_build():  # handle hints for old launcher
        #     self.d.press.home()
        #     self.click_with_timeout("text", "OK", 5)
        #     self.click_with_timeout("description", "Apps", 5)
        #     self.click_with_timeout("text", "OK", 5)

    def factory_reset(self):
        """
        factory reset device
        :return: None
        """
        self.set_lock_swipe()
        self.settings_sub_launch("Backup & reset")
        self.click_with_timeout("text", "Factory data reset")
        self.click_with_timeout("resourceId", "com.android.settings:id/initiate_master_clear")
        self.click_with_timeout("resourceId", "com.android.settings:id/execute_master_clear")

    def handle_exception_by_watcher(self):
        """
        force to run watchers to handle unfortunately window
        :return: None
        """
        if "afwHandleUnfortunately" not in self.d.watchers:
            self.d.watcher("afwHandleUnfortunately").when(textContains="Unfortunately").click(text="OK")
        if "afwHandleResponding" not in self.d.watchers:
            self.d.watcher("afwHandleResponding").when(textContains="isn't responding").click(text="OK")
        self.d.watchers.run()

    def click_toast_window(self):
        """
        click ok on toast window when it pops up
        :return: None
        """
        if self.check_ui_exists("text", "Toast"):
            self.click_with_timeout("text", "OK")

    def keep_awake(self):
        """
        keep dut awake
        :return: None
        """
        self.launch_app_by_intents("android.settings.APPLICATION_DEVELOPMENT_SETTINGS", False)
        if self.check_ui_exists("text", "Developer options"):
            if not self.check_ui_exists("text", "Stay awake"):
                self.d(scrollable=True).scroll.vert.to(text="Stay awake")
            if not self.d(text="Stay awake").right(resourceId="android:id/switchWidget").checked:
                self.click_with_timeout("text", "Stay awake")
            if not self.check_ui_exists("text", "Verify apps over USB"):
                self.d(resourceId="android:id/list").scroll.to(text="Verify apps over USB")
            if self.d(text="Verify apps over USB").enabled:
                if self.d(text="Verify apps over USB").right(resourceId="android:id/switchWidget").checked:
                    self.click_with_timeout("text", "Verify apps over USB")
        self.launch_app_by_intents("android.settings.DISPLAY_SETTINGS", False)
        if not self.check_ui_exists("text", "After 30 minutes of inactivity"):
            self.click_with_timeout("text", "Sleep")
            self.click_with_timeout("text", "30 minutes")

    def install_apprestriction_schema(self, profile=True):
        """
        install app restriction schema
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
            self.click_with_timeout("resourceId", self.ui.device_management_policy)
            self.click_with_timeout("resourceId", self.ui.global_setting)
            if self.check_ui_exists("text", "install non market apps OFF", 5):
                self.click_with_timeout("resourceId", "com.intel.afw.mdm:id/install_non_market_apps")
            self.click_with_timeout("text", "OK")
            schema_package = self.download_file_from_artifactory(self.remote.restriction_schema['sub_path'],
                                                                 self.remote.restriction_schema['name'])
            assert schema_package is not None, "[ERROR]: fail to download appRestrictionschema"
            old_dir = os.getcwd()
            os.chdir(os.path.split(schema_package)[0])
            self.install_apps(self.remote.restriction_schema['pkg_name'],
                              self.remote.restriction_schema['name'].split('.apk')[0])
            os.chdir(old_dir)
            self.api_demo_po_launch()
            self.click_with_timeout("resourceId", self.ui.app_config_policy)
        else:
            self.api_demo_launch()
            self.click_with_timeout("resourceId", self.ui.app_config_policy)
            self.click_with_timeout("text", self.ui.install_restriction_schema)
            self.click_with_timeout("text", "Install", 10)
            if self.check_ui_exists("text", "ACCEPT", 10):
                self.click_with_timeout("text", "ACCEPT")
            assert not self.check_ui_exists("text", "App not installed."), "fail to install apprestriction schema"
            self.click_with_timeout("text", "Done", 15)

    def set_clear_restriction_provider(self, is_set, profile=True):
        """
        set or clear restriction provider
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.app_config_policy)
        if is_set:
            if self.check_ui_exists("text", "Clear the default restrictions provider"):
                self.click_with_timeout("text", "Clear the default restrictions provider")
                self.click_with_timeout("text", "OK")
            self.click_with_timeout("text", "Set the default restrictions provider")
        else:
            if self.check_ui_exists("text", "Set the default restrictions provider"):
                self.click_with_timeout("text", "Set the default restrictions provider")
                self.click_with_timeout("text", "OK")
            self.click_with_timeout("text", "Clear the default restrictions provider")

    def check_restriction_provider(self, profile=True):
        """
        check restriction provider
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.app_config_policy)
        self.click_with_timeout("resourceId", self.ui.check_restriction_provider)

    def set_wifi_status(self, is_on):
        """
        turn on/off wifi
        :param is_on: True or False, True for on, False for off
        :return: None
        """
        self.launch_app_by_intents("android.settings.WIFI_SETTINGS", False)
        if not self.check_ui_exists("textMatches", "Wi.Fi"):
            self.settings_sub_launch("Wi.Fi")
        if is_on:
            if self.check_ui_exists("text", "Off"):
                self.d(text="Off").right(resourceId="com.android.settings:id/switch_widget").click()
                time.sleep(5)
            assert self.check_ui_exists("text", "On"), "[ERROR]: fail to turn on WiFi"
        else:
            if self.check_ui_exists("text", "On"):
                self.d(text="On").right(resourceId="com.android.settings:id/switch_widget").click()
                time.sleep(5)
            assert self.check_ui_exists("text", "Off"), "[ERROR]: fail to turn off WiFi"

    def set_bluetooth_status(self, is_on):
        """
        turn on/off bluetooth
        :param is_on: True or False, True for on, False for off
        :return: None
        """
        self.launch_app_by_intents("android.settings.BLUETOOTH_SETTINGS", False)
        if not self.check_ui_exists("text", "Bluetooth"):
            self.settings_sub_launch("Bluetooth")
        if is_on:
            if self.check_ui_exists("text", "Off"):
                self.d(text="Off").right(resourceId="com.android.settings:id/switch_widget").click()
            assert self.check_ui_exists("text", "On"), "[ERROR]: fail to turn on BT"
        else:
            if self.check_ui_exists("text", "On"):
                self.d(text="On").right(resourceId="com.android.settings:id/switch_widget").click()
            assert self.check_ui_exists("text", "Off"), "[ERROR]: fail to turn off BT"

    def get_notification_volume_by_dumpsys(self):
        """
        get volume of notification by dumpsys
        :return: current volume of notification
        """
        # get audo information by dumpsys command
        dumpsys_audio = os.popen("adb -s {0} shell dumpsys audio".format(self.serial)).read().strip()
        # strip the content and store them into a list
        audio_content_list = [x.strip() for x in dumpsys_audio.split("\r\n") if x.strip() != '']
        # locate the index of '- STREAM_NOTIFICATION:'
        notification_index = audio_content_list.index("- STREAM_NOTIFICATION:")
        if self.is_android_L_build():
            volume_list = audio_content_list[notification_index+3].split(":")
        else:
            for i in range(5):
                if audio_content_list[notification_index+4].find("speaker") != -1:
                    break
                time.sleep(5)
                dumpsys_audio = os.popen("adb -s {0} shell dumpsys audio".format(self.serial)).read().strip()
                audio_content_list = [x.strip() for x in dumpsys_audio.split("\r\n") if x.strip() != '']
                notification_index = audio_content_list.index("- STREAM_NOTIFICATION:")
            start = audio_content_list[notification_index+4].index("speaker")
            end = audio_content_list[notification_index+4].index(",")
            assert start < end, "[ERROR]: start >= end"
            volume_list = audio_content_list[notification_index+4][start:end].split(":")
        return int(volume_list[-1].strip())

    def set_password_expiration_timeout(self, profile=True):
        """
        set password expiration time
        :param profile: True or False, True for Profile Owner, False for Device Owner
        :return: None
        """
        if profile:
            self.api_demo_po_launch()
        else:
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.legacy_policy)
        self.click_with_timeout("resourceId", self.ui.set_password_expiration_time)
        self.d(resourceId="com.intel.afw.mdm:id/content_edit").set_text("1")
        self.click_with_timeout("text", "OK")

    def check_managed_user_by_adb(self):
        """
        verify that managed user is exist
        :return: True for managed user created
        """
        for _ in range(3):
            all_users = repr(os.popen("adb -s {0} shell pm list users".format(self.serial)).read().strip())
            if all_users.find("Users") == -1:
                continue
            if all_users.find("Work profile") != -1 or all_users.find("HiProfileOwner") != -1:
                return True
        return False

    def is_android_L_build(self):
        """
        verify current android version whether is L
        :return: True for L
        """
        # currently, for L the value is '5.x.x'
        # if os.popen("adb -s {0} shell getprop ro.build.version.release".format(self.serial)).read().startswith("5"):
        # switch to common api
        if g_common_obj2.getAndroidVersion() == "L":
            return True
        return False

    def add_google_account(self, username, password, profile=False):
        """
        add google account from settings, copy and modify code from dut_init_impl.py: add_google_account_mr1
        :param username: google account
        :param password: password for google account
        :param profile: true for adding google account in managed profile
        :return: None
        """
        self.launch_app_by_intents("android.settings.SYNC_SETTINGS", False)
        self.d(packageName="com.android.settings").wait.exists(timeout=10000)
        if not self.check_ui_exists("text", "Accounts"):
            self.settings_launch()
            if not self.check_ui_exists("text", "Accounts") and self.d(scrollable=True).exists:
                self.d(scrollable=True).scroll.vert.to(textContains="Accounts")
            self.click_with_timeout("text", "Accounts")
        if self.check_ui_exists("text", "Work"):
            if profile:
                self.d(text="Remove work profile").up(text="Add account").click.wait()
            else:
                self.d(text="Personal").down(text="Add account").click.wait()
        else:
            self.click_with_timeout("text", "Add account")
        self.d(text="Google").wait.exists(timeout=10000)
        self.click_with_timeout("text", "Google")
        self.d(description="Add your account").wait.exists(timeout=10000)
        assert not self.check_ui_exists("text", "Couldn't sign in"), \
            "[ERROR]: Couldn't sign in due to network"
        for _ in range(5):
            self.d(resourceId="identifierId").set_text(username)
            self.d(description=username).wait.exists(timeout=10000)
            if self.check_ui_exists("description", username):
                break
            self.d(resourceId="identifierId").clear_text()
            time.sleep(2)
        self.click_with_timeout("resourceId", "identifierNext")
        self.d(resourceId="password").wait.exists(timeout=10000)
        for _ in range(5):
            self.d(resourceId="password").set_text(password)
            self.d(resourceId="password-label").wait.exists(timeout=10000)
            if self.check_ui_exists("resourceId", "password-label"):
                break
            time.sleep(2)
        self.click_with_timeout("resourceId", "next")
        self.d(resourceId="tosText").wait.exists(timeout=10000)
        self.click_with_timeout("resourceId", "next")
        self.d(resourceId="com.google.android.gms:id/agree_backup").wait.exists(timeout=30000)
        self.click_with_timeout("resourceId", "com.google.android.gms:id/agree_backup")
        self.click_with_timeout("text", "Next")
        self.d(text="Set up payment info").wait.exists(timeout=20000)
        if self.check_ui_exists("text", "No thanks"):
            self.click_with_timeout("text", "No thanks")
        elif self.check_ui_exists("text", "Remind me later"):
            self.click_with_timeout("text", "Remind me later")
        self.click_with_timeout("resourceId", "com.android.vending:id/positive_button")
        self.d(packageName="com.android.settings").wait.exists(timeout=3000)

    def check_runtime_permission(self, item, profile=True):
        """
        check runtime permission
        :param item: items need to check
        :param profile: work profile or not
        :return: True False None
        """
        self.launch_app_by_intents(
            "android.settings.APPLICATION_DETAILS_SETTINGS -d package:com.example.jizhenlo.runtimepermissiontest", profile)
        self.d(text="App info").wait.exists(timeout=5000)
        if not self.check_ui_exists("text", "App info"):
            self.launch_app_by_intents("android.settings.APPLICATION_SETTINGS", profile)
            if profile and self.check_ui_exists("text", "All apps"):
                self.click_with_timeout("text", "All apps")
                self.click_with_timeout("text", "Work")
            if not self.check_ui_exists("text", "RuntimePermissionTest", 5):
                self.d(scrollable=True).scroll.vert.to(text="RuntimePermissionTest")
            self.click_with_timeout("text", "RuntimePermissionTest")
        assert self.check_ui_exists("text", "Permissions"), "Permissions doesn't exist"
        assert self.d(text="Permissions").enabled, "Permissions disabled"
        self.click_with_timeout("text", "Permissions")
        self.check_ui_exists("text", item, 10)
        for _ in range(3):
            if not self.d(text=item).enabled:
                if self.check_ui_exists("resourceId", "com.android.packageinstaller:id/switchWidget"):
                    if self.d(text=item).right(resourceId="com.android.packageinstaller:id/switchWidget").checked:
                        return False, True
                    else:
                        return False, False
                else:
                    if self.d(text=item).right(resourceId="android:id/switchWidget").checked:
                        return False, True
                    else:
                        return False, False
        return True, None

    def set_runtime_permission(self, item, value, profile=True):
        """
        set runtime permission state
        :param item: string, the item want to setup
        :param value: string, Default, Grant, Deny
        :param profile: True of False
        :return: None
        """
        value_dict = {
            "Default": "com.intel.afw.mdm:id/radioDefault",
            "Grant": "com.intel.afw.mdm:id/radioGrant",
            "Deny": "com.intel.afw.mdm:id/radioDeny"
        }
        res = os.popen("adb -s {0} shell pm list packages com.example.jizhenlo.runtimepermissiontest".format(
            self.serial)).read().strip()
        not_installed = False
        if profile:
            if not self.locate_apps("Work RuntimePermissionTest"):
                not_installed = True
        else:
            if res.find("runtimepermissiontest") == -1:
                not_installed = True
        if not_installed:
            self.unknown_source_control(True)
            if profile:
                # currently, fail to install apk under work profile due to selinux policy
                for _ in range(5):
                    os.popen("adb -s {0} root".format(self.serial)).read()
                    selinux = os.popen("adb -s {0} shell getenforce".format(self.serial)).read().strip()
                    if selinux == "Permissive":
                        break
                    os.popen("adb -s {0} shell setenforce 0".format(self.serial)).read().strip()
                    time.sleep(2)
                self.api_demo_po_launch()
                self.click_with_timeout("resourceId", self.ui.device_management_policy)
                self.click_with_timeout("resourceId", self.ui.global_setting)
                assert self.check_ui_exists("text", "Set global or secure settings"), "fail to detect global window"
                if self.check_ui_exists("text", "install non market apps OFF"):
                    self.click_with_timeout("resourceId", "com.intel.afw.mdm:id/install_non_market_apps")
                self.click_with_timeout("text", "OK")
                self.d.press.back()
                if not self.check_ui_exists("resourceId", self.ui.app_provision):
                    self.clean_tasks()
                    self.api_demo_po_launch()
            else:
                self.api_demo_launch()
            self.click_with_timeout("resourceId", self.ui.app_provision)
            for _ in range(10):
                self.click_with_timeout("text", "Install A Runtime Permission Test Sample")
                self.click_with_timeout("resourceId", "com.android.packageinstaller:id/ok_button")
                self.click_with_timeout("text", "ACCEPT", 5)
                if self.check_ui_exists("resourceId", "com.android.packageinstaller:id/done_button", 10):
                    self.click_with_timeout("resourceId", "com.android.packageinstaller:id/done_button")
                    break
            self.clean_tasks()
            if profile:
                os.popen("adb -s {0} shell setenforce 1".format(self.serial)).read().strip()
            time.sleep(5)
        installed = False
        if profile:
            if self.locate_apps("Work RuntimePermissionTest"):
                installed = True
            assert installed, "fail to detect installed package"
            self.api_demo_po_launch()
        else:
            for i in range(5):
                res = os.popen("adb -s {0} shell pm list packages com.example.jizhenlo.runtimepermissiontest".format(
                    self.serial)).read().strip()
                if res.find("runtimepermissiontest") != -1:
                    installed = True
                    break
                time.sleep(2)
            if not installed and not self.locate_apps("RuntimePermissionTest"):
                g_common_obj2.system_reboot(90)
                for i in range(20):
                    self.d = g_common_obj.get_device()
                    self.d.wakeup()
                    if self.check_ui_exists("resourceId", self.ui.lock_clock_view):
                        break
                    time.sleep(5)
                g_common_obj.set_vertical_screen()
                self.unlock_screen()
            self.api_demo_launch()
        self.click_with_timeout("resourceId", self.ui.app_provision)
        self.click_with_timeout("text", "Set RuntimePermissionTest Permission State")
        self.click_with_timeout("resourceId", "com.intel.afw.mdm:id/SpinnerPermissionGrp")
        self.click_with_timeout("text", item)
        self.click_with_timeout("resourceId", value_dict[value])
        self.click_with_timeout("text", "OK")

    def remove_other_users_by_id(self):
        """
        remove all users except the owner by id
        :return: None
        """
        users_list = ""
        os.popen("adb -s {0} root".format(self.serial)).read().strip()
        for _ in range(5):
            users_list = os.popen("adb -s {0} shell pm list users".format(self.serial)).read().strip()
            if users_list.find("UserInfo") != -1:
                break
        while users_list.find("UserInfo{") != -1:
            index = users_list.find("UserInfo{") + 9
            users_list = users_list[index:]
            id_index = users_list.find(":")
            user_id = int(users_list[:id_index])
            if user_id != 0:
                os.popen("adb -s {0} shell pm remove-user {1}".format(self.serial, user_id)).read().strip()

    def is_work_profile_enabled(self):
        """
        check whether work profile is enabled in Account Settings
        :return: True/False
        """
        self.launch_app_by_intents("android.settings.SYNC_SETTINGS", False)
        if not self.check_ui_exists("text", "Accounts"):
            self.settings_sub_launch("Accounts")
        if self.check_ui_exists("text", "Work", 5):
            return True
        return False

    def is_device_owner_enabled(self):
        """
        check whether device_owner is enabled
        :return: True/False
        """
        self.d = g_common_obj.get_device()
        os.popen("adb -s {0} root".format(self.serial)).read().strip()
        mdm_package = os.popen("adb -s {0} shell pm list packages com.intel.afw.mdm".format(self.serial)).read().strip()
        if mdm_package.find("com.intel.afw.mdm") == -1:
            return False
        owner_file_exist = os.popen("adb -s {0} shell ls /data/system/device_owner.xml".format(self.serial)).read()
        if owner_file_exist.find("No such file or directory") != -1:
            return False
        owner_file = os.popen("adb -s {0} shell cat /data/system/device_owner.xml".format(self.serial)).read().strip()
        if owner_file.find("device-owner") != -1:
            return True
        return False

    def connect_wifi(self):
        """
        connect wifi
        :return: None
        """
        if not os.path.isfile("/etc/oat/sys.conf"):
            return None
        ssid = self.config.read("/etc/oat/sys.conf", 'wifisetting').get("ssid")
        password = self.config.read("/etc/oat/sys.conf", 'wifisetting').get("passwd")
        security = self.config.read("/etc/oat/sys.conf", 'wifisetting').get("security")
        self.launch_app_by_intents("android.settings.WIFI_SETTINGS", False)
        self.d(packageName="com.android.settings").wait.exists(timeout=5000)
        if self.check_ui_exists("text", "Off"):
            self.click_with_timeout("resourceId", "com.android.settings:id/switch_bar")
            self.d(text="On").wait.exists(timeout=5000)
        self.d.press.menu()
        self.d(text="Add network").wait.exists(timeout=5000)
        self.d(text="Add network").click.wait()
        self.d(resourceId="com.android.settings:id/ssid").wait.exists(timeout=5000)
        self.d(resourceId="com.android.settings:id/ssid").set_text(ssid)
        self.d(resourceId="com.android.settings:id/security").click.wait()
        self.d(text=security).click.wait()
        self.d(resourceId="com.android.settings:id/password").set_text(password)
        self.d(text="Save").click.wait()
        for _ in range(10):
            if self.d(resourceId="android:id/list").scrollable:
                self.d(resourceId="android:id/list").fling.vert.toBeginning()
            if self.check_ui_exists("text", "Connected"):
                break
            time.sleep(2)
        self.d.press.home()

    def provision_device_owner(self):
        """
        provision device as device owner
        :return: None
        """
        # function = Function()
        # function.push_uiautomator_jar()
        self.d.wakeup()
        g_common_obj.set_vertical_screen()
        self.d.press.home()
        # self.d("packageName", "com.google.android.setupwizard").wait.exists(timeout=5000)
        while not self.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"):
            self.d.press.back()
        if os.popen("adb -s {0} shell getprop ro.crypto.state".format(self.serial)).read().strip() == "unencrypted":
            os.popen("adb -s {0} root".format(self.serial)).read().strip()
            time.sleep(5)
            os.popen("adb -s {0} shell setprop persist.sys.no_req_encrypt true".format(self.serial)).read().strip()
            time.sleep(5)
        self.oobe_setup(True)
        self.unlock_screen()
        self.d.press.home()
        self.click_with_timeout("text", "GOT IT")
        self.keep_awake()


class ResultConfig(object):
    """
    File for temporary result
    """
    def __init__(self, group, domain=".AfW"):
        self.domain = domain
        self.group = group
        self.result = ConfigParser.RawConfigParser()
        if not os.path.exists("/tmp/{0}".format(self.domain)):
            os.mkdir("/tmp/{0}".format(self.domain))
        if not os.path.exists("/tmp/{0}/{1}".format(self.domain, self.group)):
            os.mkdir("/tmp/{0}/{1}".format(self.domain, self.group))
        self.result_file = "/tmp/{0}/{1}/{2}.ini".format(self.domain, self.group, self.group)

    def remove_result_file(self):
        """
        remove result file
        :return: None
        """
        if os.path.isfile(self.result_file):
            self.result = None
            os.remove(self.result_file)

    def query_result(self, section, option):
        """
        query result from result file
        :return: True/False
        """
        if os.path.isfile(self.result_file):
            self.result.read(self.result_file)
            if self.result.has_option(section, option):
                return self.result.getboolean(section, option)
        return False

    def update_result(self, section, option, result):
        """
        update result in file
        :return: None
        """
        if os.path.isfile(self.result_file):
            self.result.read(self.result_file)
            if self.result.has_option(section, option):
                self.result.remove_option(section, option)
            if not self.result.has_section(section):
                self.result.add_section(section)
        else:
            self.result.add_section(section)
        self.result.set(section, option, result)
        with open(self.result_file, "wb") as f:
            self.result.write(f)
