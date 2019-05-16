import os
from testlib.util.uiatestbase import UIATestBase
#from testlib.util.repo import Artifactory
from testlib.dut_init.dut_init_impl import Function

class FillInternalStorage(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.percent = self.config.read(cfg_file,'fill_internal_storage').get("percent")
        self.percent = int(self.percent)
        self.retry_num = self.config.read(cfg_file,'init_list').get("fill_internal_storage")
        self.retry_num = int(self.retry_num)
        self.func = Function()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def testFillInternalStorage(self):
        """
        Fill internal storage
        """
        print "[RunTest]: %s" % self.__str__()

        succeed = False
        for i in range(self.retry_num):
            try:
                self.func.fill_internal_storage(self.percent)
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

