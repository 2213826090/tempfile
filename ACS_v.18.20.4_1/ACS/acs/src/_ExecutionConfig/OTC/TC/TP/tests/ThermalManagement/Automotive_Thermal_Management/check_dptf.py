#! /usr/bin/env python
# coding:utf-8

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.em.thermal import Thermal

class CheckDPTF(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.thermal = Thermal()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_check_DPTF(self):
        """
        test check DPTF
        """
        print "[RunTest]: %s" % self.__str__()
        pid = self.thermal.get_process_pid("esif_ufd")
        assert pid, "No esif_ufd process"

