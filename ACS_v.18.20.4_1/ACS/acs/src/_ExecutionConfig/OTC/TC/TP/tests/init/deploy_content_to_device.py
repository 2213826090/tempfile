import os
from testlib.util.uiatestbase import UIATestBase
#from testlib.util.repo import Artifactory
from testlib.dut_init.dut_init_impl import Function

class DeployContent(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.location = self.config.read(cfg_file,'artifactory').get("location")
        self.content = self.config.read(cfg_file,'artifactory').get("content")
        self.script = self.config.read(cfg_file,'deploy_script').get("script_file")
        self.retry_num = self.config.read(cfg_file,'init_list').get("deploy_content_to_device")
        self.retry_num = int(self.retry_num)
        self.func = Function()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def testDeployContent(self):
        """
        Deploy content files to device
        """
        print "[RunTest]: %s" % self.__str__()

        succeed = False
        for i in range(self.retry_num):
            try:
                self.func.deploy_content(self.location, self.content, self.script)
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

