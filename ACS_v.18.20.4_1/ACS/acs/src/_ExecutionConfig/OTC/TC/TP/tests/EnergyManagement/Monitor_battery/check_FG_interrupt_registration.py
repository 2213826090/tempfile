#! /usr/bin/env python
# coding:utf-8

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.em.em_impl import EMImpl
from testlib.util.common import g_common_obj

class check_FG_interrupt_registration(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.emImpl = EMImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_check_FG_interrupt_registration(self):

        print "[RunTest]: %s" % self.__str__()
        self.emImpl.adb_root()
        time.sleep(2)
        self.emImpl.adb_root()
        time.sleep(5)
        product = self.emImpl.product
        if "cht_ffd" in product:
            cmd = "cat /proc/interrupts | grep max17"
            result = g_common_obj.adb_cmd_capture_msg(cmd)
            if not result:
                assert False, "Fuel Gauger Interrupt Registration Not Exist."
        else:
            assert False, "Not applicable for this DUT"

