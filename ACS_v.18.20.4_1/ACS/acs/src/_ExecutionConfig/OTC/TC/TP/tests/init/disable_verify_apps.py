import os
from testlib.util.uiatestbase import UIATestBase
#from testlib.util.repo import Artifactory
from testlib.dut_init.dut_init_impl import Function
from testlib.util.common import g_common_obj

class DisableVerifyApp(UIATestBase):

    def setUp(self):
        super(DisableVerifyApp, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.retry_num = self.config.read(cfg_file,'init_list').get("disable_verify_apps")
        self.retry_num = int(self.retry_num)
        self.func = Function()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DisableVerifyApp, self).tearDown()

    def testDisableVerifyApp(self):
        """
        Disable verify apps
        """
        print "[RunTest]: %s" % self.__str__()

        succeed = False
        for i in range(self.retry_num):
            try:
                os.system("adb shell input keyevent 82;adb shell input keyevent 3")
                android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
                if (android_version_release == '[8.0.0]') or (android_version_release == '[8.1.0]'):
                    self.func.disable_verify_apps()
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

