# -*- coding: utf-8 -*-

from testlib.AfW.api_impl import ApiImpl
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
import time
import os


class System(UIATestBase):
    """
    @summary: system status
    """

    def setUp(self):
        super(System, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        # self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(System, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSystemFlag_LowRam(self):
        """
        verify low ram flag
        :return: None
        """
        # device_model = os.popen("adb -s {0} shell getprop ro.product.model".format(self.api.serial)).read().strip()
        ret = os.popen("adb -s {0} shell getprop ro.config.low_ram".format(self.api.serial)).read().strip()
        assert ret == '', "detect low_ram flag"

    def testFeatureFlag_BYOD(self):
        """
        verify BYOD provisioning is supported
        :return: None
        """
        # device_model = os.popen("adb -s {0} shell getprop ro.product.model".format(self.api.serial)).read().strip()
        ret = os.popen("adb -s {0} shell pm list features |grep android.software.managed_users".format(
            self.api.serial)).read().strip()
        assert ret.find("android.software.managed_users") != -1, "fail to detect managed_users flag for M"

    def testFeatureFlag_COPE(self):
        """
        verify COPE provisioning is supported
        :return: None
        """
        cope_flag = False
        for _ in range(3):
            ret = os.popen("adb -s {0} shell pm list features |grep android.software.device_admin".format(
                self.api.serial)).read().strip()
            if ret.find("android.software.device_admin") != -1:
                cope_flag = True
                break
        assert cope_flag, "fail to detect device_admin flag"

    def testMultiUserSupport(self):
        """
        verify multi user is supported
        :return: None
        """
        # device_model = os.popen("adb -s {0} shell getprop ro.product.model".format(self.api.serial)).read().strip()
        ret = os.popen("adb -s {0} shell pm get-max-users".format(self.api.serial)).read().strip().split()[-1]
        assert int(ret) >= 2, "multi supported user < 2"

    def testFactoryResetSupport(self):
        """
        verify that able to factory reset device successfully
        :return: None
        """
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
        assert self.api.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"), \
            "fail to detect welcome title in setup wizard"
        # try to skip setup wizard by click four corner
        for i in range(5):
            self.api.d.wakeup()
            x = self.api.d.info['displayWidth']
            y = self.api.d.info['displayHeight']
            self.api.click_with_timeout("text", "Allow")
            if self.api.check_ui_exists("text", "GOT IT"):
                self.api.click_with_timeout("text", "GOT IT")
                break
            self.api.d.click(50, 50)
            self.api.d.click(x - 50, 50)
            self.api.d.click(x - 50, y - 50)
            self.api.d.click(50, y - 50)
            time.sleep(5)
        self.api.keep_awake()

    def testDiskEncryptionStatus(self):
        """
        verify the encryption status of device
        :return:
        """
        ret = os.popen("adb -s {0} shell getprop ro.crypto.state".format(self.api.serial)).read().strip()
        if ret != 'encrypted':
            try:
                self.api.keep_awake()
                self.api.launch_app_by_intents("android.settings.SECURITY_SETTINGS", False)
                assert self.api.check_ui_exists("text", "Encryption", 5), "Fail to detect Encryption option"
                if not self.api.check_ui_exists("text", "Encrypted", 5):
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
            finally:
                time.sleep(600)
                self.api = ApiImpl()
                g_common_obj2.system_reboot(90)
                for i in range(30):
                    self.api.d = g_common_obj.get_device()
                    self.api.d.wakeup()
                    if self.api.check_ui_exists("resourceId", self.api.ui.lock_clock_view) or \
                            self.api.check_ui_exists("resourceId", "com.google.android.setupwizard:id/welcome_title"):
                        break
                    time.sleep(10)
                g_common_obj.set_vertical_screen()
                self.api.unlock_screen()
                ret = os.popen("adb -s {0} shell getprop ro.crypto.state".format(self.api.serial)).read().strip()
        assert ret == 'encrypted', "fail to encrypt device"
