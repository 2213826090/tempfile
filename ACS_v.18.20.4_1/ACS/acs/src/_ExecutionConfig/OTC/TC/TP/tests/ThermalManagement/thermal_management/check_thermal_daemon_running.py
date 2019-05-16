#! /usr/bin/env python
# coding:utf-8

from testlib.util.uiatestbase import UIATestBase
from testlib.em.tools import UIBase

class CheckThermalDaemonRunning(UIATestBase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def test_check_thermal_daemon_running(self):
        """
        Check thermal daemon running
        """
        print "[RunTest]: %s" % self.__str__()
        pid = UIBase().get_process_pid("thermal")

