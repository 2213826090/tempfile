import os
from testlib.util.uiatestbase import UIATestBase
#from testlib.util.repo import Artifactory
from testlib.dut_init.dut_init_impl import Function

class SetTimeDate(UIATestBase):

    def setUp(self):
        super(SetTimeDate, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.retry_num = self.config.read(cfg_file,'init_list').get("set_system_date_and_time")
        self.retry_num = int(self.retry_num)
        self.func = Function()

    def tearDown(self):
        super(SetTimeDate, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSetTimeDate(self):
        """
        Set system date and time
        """
        print "[RunTest]: %s" % self.__str__()

        succeed = False
        for i in range(self.retry_num):
            try:
                self.func.set_timedate()
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

