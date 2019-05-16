import os
import unittest
#from testlib.util.uiatestbase import UIATestBase
#from testlib.util.repo import Artifactory
from testlib.dut_init.dut_init_impl import Function

class StartRPCServer(unittest.TestCase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.func = Function()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def testStartRPCServer(self):
        """
        Start RPC server
        """
        print "[RunTest]: %s" % self.__str__()

        self.func.push_uiautomator_jar()

