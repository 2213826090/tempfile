# -*- coding: utf-8 -*-

from testlib.AfW.api_impl import ApiImpl
from testlib.util.uiatestbase import UIATestBase
import time
import os


class GMSBehavior(UIATestBase):
    """
    @summary: test cases for GMSBehavior
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

    def testGMS_Account_Add(self):
        """
        verify that able to add google account in device owner
        :return: None
        """
        assert os.path.isfile("/etc/oat/sys.conf"), "/etc/oat/sys.conf doesn't exists"
        account = self.config.read("/etc/oat/sys.conf", "google_account").get("user_name")
        password = self.config.read("/etc/oat/sys.conf", "google_account").get("password")
        self.api.settings_sub_launch("Accounts")
        if self.api.check_ui_exists("text", "Google"):
            self.api.click_with_timeout("text", "Google")
            time.sleep(2)
            self.api.d.press.menu()
            self.api.click_with_timeout("text", "Remove account")
            self.api.click_with_timeout("text", "Remove account")
            time.sleep(5)
        self.api.clean_tasks()
        self.api.add_google_account(account, password, False)
        self.api.d.press.home()
        self.api.clean_tasks()
        self.api.settings_sub_launch("Accounts")
        assert self.api.check_ui_exists("text", "Google"), "fail to detect google account"

    def testGMS_Account_Remove(self):
        """
        verify that able to remove google account in device owner
        :return: None
        """
        account = self.config.read("/etc/oat/sys.conf", "google_account").get("user_name")
        password = self.config.read("/etc/oat/sys.conf", "google_account").get("password")
        self.api.settings_sub_launch("Accounts")
        if not self.api.check_ui_exists("text", "Google"):
            self.api.clean_tasks()
            self.api.add_google_account(account, password, False)
            self.api.clean_tasks()
            self.api.settings_sub_launch("Accounts")
        self.api.click_with_timeout("text", "Google")
        time.sleep(5)
        self.api.d.press.menu()
        self.api.click_with_timeout("text", "Remove account")
        self.api.click_with_timeout("text", "Remove account")
        time.sleep(5)
        assert not self.api.check_ui_exists("text", "Google"), "fail to remove google account"
