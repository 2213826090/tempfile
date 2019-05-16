# -*- coding: utf-8 -*-

from testlib.AfW.api_impl import ApiImpl
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
import time


class ProfileOwnerProvisioning(UIATestBase):
    """
    @summary: Profile Owner Provisioning
    """

    def setUp(self):
        super(ProfileOwnerProvisioning, self).setUp()
        self._test_name = __name__
        self.api = ApiImpl()
        self.api.unlock_screen()
        self.api.clean_tasks()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        super(ProfileOwnerProvisioning, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testRebootOSADB(self):
        """
        reboot device by 'adb reboot' while setting up work profile
        :return: None
        """
        if self.api.locate_apps("Work Sample MDM"):
            self.api.remove_managed_profile(True)
            self.api.clean_tasks()
        self.api.api_demo_launch()
        self.api.click_with_timeout("resourceId", self.api.ui.setup_managed_profile)
        if not self.api.is_android_L_build():
            self.api.click_with_timeout("text", "No", 5)
        self.api.click_with_timeout("text", "Set up", 10)
        self.api.click_with_timeout("text", "OK")
        self.api.click_with_timeout("text", "OK")
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
        assert self.api.check_ui_exists("description", "Apps"), "Fail to detect Apps Launcher"
