# -*- coding: utf-8 -*-

from testlib.AfW.api_impl import ApiImpl
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
import time


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

    def testCreateWorkProfile_Remove_50_times(self):
        """
        create and delete work profile for 50 times
        :return: None
        """
        self.api.api_demo_launch()
        if self.api.check_ui_exists("textContains", "Device Owner", 5):
            self.api.click_with_timeout("resourceId", self.api.ui.device_provision)
            self.api.click_with_timeout("resourceId", self.api.ui.clear_device_owner_app)
            self.api.click_with_timeout("text", "Yes")
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
        for _ in range(50):
            if not self.api.locate_apps("Work Sample MDM"):
                self.api.setup_managed_profile(True)
            # time.sleep(5)
            # self.api.settings_sub_launch("Accounts")
            # assert self.api.check_ui_exists("text", "Accounts"),
            # "fail to launch settings at {0}".format(_)
            # assert self.api.check_ui_exists("text", "Work"),
            # "fail to detect work section in accounts at {0}".format(_)
            assert self.api.is_work_profile_enabled(), "fail to detect work section in accounts at {0}".format(_)
            self.api.click_with_timeout("text", "Remove work profile")
            self.api.d.wait.update(timeout=3000)
            self.api.click_with_timeout("text", "Delete")
            self.api.d.wait.update(timeout=3000)
            assert self.api.check_ui_exists("text", "Accounts"), \
                "fail to detect accounts after remove work profile at {0}".format(_)
            assert not self.api.check_ui_exists("text", "Work"), \
                "fail to detect work section in accounts at {0}".format(_)
            self.api.clean_tasks()
