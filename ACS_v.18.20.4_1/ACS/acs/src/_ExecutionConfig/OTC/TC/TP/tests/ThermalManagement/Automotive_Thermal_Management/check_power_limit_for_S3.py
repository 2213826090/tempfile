#! /usr/bin/env python
# coding:utf-8

from testlib.util.uiatestbase import UIATestBase
from testlib.em.thermal import Thermal
from testlib.em.s0i3 import S0i3

class CheckPowerLimit(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        self.s0i3 = S0i3()
        self.s0i3.adb_root()
        self.s0i3.set_screen_status("on")
        self.s0i3.unlock_screen()
        print "[Setup]: %s" % self._test_name
        self.thermal = Thermal()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_check_power_limit_for_S3(self):
        """
        test check Power limit
        """
        print "[RunTest]: %s" % self.__str__()
        power_p = self.thermal.ivi_get_power()
        power_pre = power_p / float(10 ** 6)
        enter_s3 = self.s0i3.suspend_resume(retry=2)
        assert enter_s3, "Not enter to S3"
        power_a = self.thermal.ivi_get_power()
        power_post = power_a / float(10 ** 6)
        if power_pre == power_post:
            print "[Info]--- check Power limit: pre:{}, post:{}".format(power_pre,power_post)
        else:
            assert False, "[Info]--- check Power limit: pre:{}, post:{}".format(power_pre,power_post)

