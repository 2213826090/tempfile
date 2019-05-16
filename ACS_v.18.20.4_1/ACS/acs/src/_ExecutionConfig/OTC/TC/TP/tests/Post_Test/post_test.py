import os
from testlib.util.uiatestbase import UIATestBase
from testlib.wifi.wifi_impl import WifiSettingImpl
from testlib.dut_init.dut_init_impl import Function
from testlib.util.common import g_common_obj

class PostTestWifiTearDown(UIATestBase):

    def setUp(self):
        super(PostTestWifiTearDown, self).setUp()
        cfg_file = 'tests.tablet.wifi.conf'
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.wifi = WifiSettingImpl({})

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(PostTestWifiTearDown, self).tearDown()
        self.wifi = None

    def postTestWifiOff(self):
        """
        post test wifi off
        """
        print "[RunTest]: %s" % self.__str__()
        self.wifi.launch_from_am()
        self.wifi.off()

    def postTestRemoveGoogleAccount(self):
        print "Remove Google Account"
        if g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip() == '[O]':
            return
        Function().remove_google_accout()
