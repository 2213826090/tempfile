import os
from testlib.util.uiatestbase import UIATestBase
#from testlib.util.repo import Artifactory
from testlib.dut_init.dut_init_impl import Function

class EnableDeveloperOption(UIATestBase):

    def setUp(self):
        super(EnableDeveloperOption, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.retry_num = self.config.read(cfg_file,'init_list').get("enable_developer_option")
        self.retry_num = int(self.retry_num)
        self.func = Function()

    def tearDown(self):
        super(EnableDeveloperOption, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testEnableDeveloperOption(self):
        """
        Enable developer option
        """
        print "[RunTest]: %s" % self.__str__()

        succeed = False
        for i in range(self.retry_num):
            try:
                os.system("adb shell input keyevent 82;adb shell input keyevent 3")
                self.func.enable_developer_option()
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

