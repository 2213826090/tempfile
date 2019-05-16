import os
from testlib.util.uiatestbase import UIATestBase
#from testlib.util.repo import Artifactory
from testlib.dut_init.dut_init_impl import Function

class DisableLockScreen(UIATestBase):

    def setUp(self):
        super(DisableLockScreen, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.retry_num = self.config.read(cfg_file,'init_list').get("disable_screen_lock")
        self.retry_num = int(self.retry_num)
        self.func = Function()

    def tearDown(self):
        super(DisableLockScreen, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testDisableLockScreen(self):
        """
        Disable screen lock
        """
        print "[RunTest]: %s" % self.__str__()

        succeed = False
        for i in range(self.retry_num):
            try:
                os.system("adb shell input keyevent 82;adb shell input keyevent 3")
                self.func.close_lock_screen()
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

