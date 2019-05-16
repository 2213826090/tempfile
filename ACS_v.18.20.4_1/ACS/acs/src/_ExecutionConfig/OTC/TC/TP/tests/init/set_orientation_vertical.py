import os
from testlib.util.uiatestbase import UIATestBase
#from testlib.util.repo import Artifactory
from testlib.util.common import Common

class SetOrientationVertical(UIATestBase):

    def setUp(self):
        super(SetOrientationVertical, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.retry_num = self.config.read(cfg_file,'init_list').get("set_orientation_vertical")
        self.retry_num = int(self.retry_num)
        self.func = Common()

    def tearDown(self):
        super(SetOrientationVertical, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSetOrientationVertical(self):
        """
        Set orientation to vertical
        """
        print "[RunTest]: %s" % self.__str__()

        succeed = False
        for i in range(self.retry_num):
            try:
                self.func.set_vertical_screen()
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

