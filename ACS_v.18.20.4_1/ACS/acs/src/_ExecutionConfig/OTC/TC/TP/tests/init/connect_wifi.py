import os
from testlib.util.uiatestbase import UIATestBase
#from testlib.util.repo import Artifactory
from testlib.dut_init.dut_init_impl import Function

class ConnectWifi(UIATestBase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.conf = self.config.read(cfg_file,'wifisetting')
        self.ssid = self.conf.get("ssid")
        self.passwd = self.conf.get("passwd")
        self.security = self.conf.get("security")
        self.retry_num = self.config.read(cfg_file,'init_list').get("connect_wifi")
        self.retry_num = int(self.retry_num)
        self.func = Function()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def testConnectWifi(self):
        """
        Connect wifi
        """
        print "[RunTest]: %s" % self.__str__()

        succeed = False
        for i in range(self.retry_num):
            try:
                os.system("adb shell input keyevent 82;adb shell input keyevent 3")
                self.func.connect_AP(self.ssid, self.passwd, self.security)
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

