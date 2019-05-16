import os
from testlib.util.uiatestbase import UIATestBase
#from testlib.util.repo import Artifactory
from testlib.dut_init.dut_init_impl import Function

class AdbRoot(UIATestBase):

    def setUp(self):
        super(AdbRoot, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.retry_num = self.config.read(cfg_file,'init_list').get("switch_to_adb_root_mode")
        self.retry_num = int(self.retry_num)
        self.func = Function()

    def tearDown(self):
        super(AdbRoot, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testAdbRoot(self):
        """
        Switch to adb root mode
        """
        print "[RunTest]: %s" % self.__str__()

        succeed = False
        for i in range(self.retry_num):
            try:
                self.func.root_DUT()
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

