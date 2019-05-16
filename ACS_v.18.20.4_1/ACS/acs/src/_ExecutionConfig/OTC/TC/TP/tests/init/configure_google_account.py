from testlib.util.uiatestbase import UIATestBase
from testlib.dut_init.dut_init_impl import Function
from testlib.util.common import g_common_obj

class ConfigGoogleAccount(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.username = self.config.read(cfg_file,'google_account').get("user_name")
        self.passwd = self.config.read(cfg_file,'google_account').get("password")
        self.retry_num = self.config.read(cfg_file,'init_list').get("configure_google_account")
        self.retry_num = int(self.retry_num)
        self.d = g_common_obj.get_device()
        self.func = Function()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def testConfigGoogleAccount(self):
        """
        Configure google account
        """
        print "[RunTest]: %s" % self.__str__()

        self.d.wakeup()
        if self.d(resourceId="com.android.systemui:id/lock_icon"):
            self.func.close_lock_screen()

        succeed = False
        for i in range(self.retry_num):
            try:
                android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
                if (android_version_release == '[8.0.0]') or (android_version_release == '[8.1.0]'):
                    self.func.add_google_account_mr1(self.username, self.passwd)
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

