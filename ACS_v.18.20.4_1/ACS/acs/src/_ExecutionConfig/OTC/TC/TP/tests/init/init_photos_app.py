import os
from testlib.util.uiatestbase import UIATestBase
#from testlib.util.repo import Artifactory
from testlib.dut_init.dut_init_impl import Function

class InitPhotos(UIATestBase):

    def setUp(self):
        super(InitPhotos, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.retry_num = self.config.read(cfg_file,'init_list').get("init_photo_app")
        self.retry_num = int(self.retry_num)
        self.func = Function()

    def tearDown(self):
        super(InitPhotos, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testInitPhotos(self):
        """
        Init Photos App
        """
        print "[RunTest]: %s" % self.__str__()

        succeed = False
        for i in range(self.retry_num):
            try:
                self.func.init_photo()
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

