import os
from testlib.util.uiatestbase import UIATestBase
#from testlib.util.repo import Artifactory
from testlib.dut_init.dut_init_impl import Function

class SetLanguageUS(UIATestBase):

    def setUp(self):
        super(SetLanguageUS, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.retry_num = self.config.read(cfg_file,'init_list').get("set_language_to_united_states")
        if self.retry_num is None:
            self.retry_num = 3
        self.retry_num = int(self.retry_num)
        self.func = Function()

    def tearDown(self):
        super(SetLanguageUS, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSetLanguageUS(self):
        """
        Set language to United States
        """
        print "[RunTest]: %s" % self.__str__()

        succeed = False
        for i in range(self.retry_num):
            try:
                self.func.set_language()
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

