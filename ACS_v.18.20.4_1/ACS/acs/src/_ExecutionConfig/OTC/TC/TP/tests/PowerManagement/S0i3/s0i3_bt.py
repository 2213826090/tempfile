#! /usr/bin/env python
# coding:utf-8

from testlib.util.uiatestbase import UIATestBase
import time
from testlib.em.settings import BTSetting
from testlib.em.s0i3 import S0i3

class S0i3_BT(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.bt = BTSetting()
        self.bt.adb_root()
        #super(S0i3_BT, self).setUp()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.bt.host_teardown()
        self.bt.switch_bt("OFF")
        #super(S0i3_BT, self).tearDown()

    def bt_transfer(self):
        alias = self.bt.get_host_alias()
        self.bt.host_setup()
        self.bt.switch_bt("ON")
        self.bt.bt_search(alias)
        self.bt.bt_pair(alias)
        self.bt.host_send_file_to_device()
        self.bt.device_receive_file()
        time.sleep(10)
        assert self.bt.get_file_transfer_status(), "BT transfor Failed"

    def test_s0i3_bt_transfer(self):
        print "[RunTest]: %s" % self.__str__()
        self.bt_transfer()
        assert S0i3().suspend_resume(retry = 2), "Not enter S3"

    def test_s0i3_resume_bt_on(self):
        print "[RunTest]: %s" % self.__str__()
        self.bt.switch_bt("ON")
        time.sleep(5)
        assert S0i3().suspend_resume(retry = 2), "Not enter S3"
        assert self.bt.get_bt_status(), "BT is OFF"

    def test_s0i3_resume_bt_transfer(self):
        print "[RunTest]: %s" % self.__str__()
        self.bt.switch_bt("ON")
        time.sleep(5)
        assert S0i3().suspend_resume(retry = 2), "Not enter S3"
        time.sleep(5)
        self.bt_transfer()

