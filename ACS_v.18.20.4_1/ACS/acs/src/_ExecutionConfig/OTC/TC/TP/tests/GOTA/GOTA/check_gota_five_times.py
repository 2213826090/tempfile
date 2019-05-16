'''
@summary: Check GOTA update five times
@since: 8/6/2015
@author: Yang, NaX <nax.yang@intel.com>
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.common.common import g_common_obj
from testlib.common.common import g_common_obj2
from testlib.util.device import TestDevice

class CheckGotaFiveTimes(UIATestBase):
    """
    @summary: reliability test
    """

    def setUp(self):
        super(CheckGotaFiveTimes, self).setUp()
        self._test_name = __name__
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''),'tests.tablet.gota.conf')
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota = gotaImpl(self.cfg)
        print "[Setup]: %s" % self._test_name
        self.ssid = self.config.read(cfg_file, 'wifisetting').get("ssid")
        self.passwd = self.config.read(cfg_file, 'wifisetting').get("passwd")
        self.serial = g_common_obj2.getSerialNumber()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckGotaFiveTimes, self).tearDown()
        self.cfg = None

    def testGotaFiveTimes(self):
        print "[RunTest]: %s" % self.__str__()
        for i in range(5):
            #flash DUT and skip boot the wizard
            base_build = self.cfg.get("base_build")
            self.gota.phone_flash_tool_build_zip_file(base_build)
            self.gota.push_uiautomator_jar()
            g_common_obj.set_vertical_screen()
            TestDevice().skip_initial_screen_after_factory_reset()
            self.gota.enable_developer_option()
            self.gota.keep_awake()
            self.gota.close_lock_screen()
            self.gota.accept_unknow_resource()
            self.gota.unverify_apps_over_USB()
            #check the fingerprint before gota
            Fingerprint_before_GOTA = os.popen("adb -s %s shell getprop ro.build.fingerprint" %self.serial).read()
            Fingerprint_before_GOTA = Fingerprint_before_GOTA.replace("\n","",1)
            self.gota.insert_info("Fingerprint before GOTA", Fingerprint_before_GOTA)
            print "[INFO] --- check the fingerprint before gota is: %s" %Fingerprint_before_GOTA
            #connect AP
            print "self.ssid: %s" %self.ssid
            print "self.passwd: %s" %self.passwd
            if self.ssid == None:
                print "using default ap setting"
                self.gota.connect_AP("SHZ13F-OTC-irda101", "zxcvbnm!?")
            else:
                print "using config ap setting"
                self.gota.connect_AP(self.ssid, self.passwd)
            print "[Info] --- Connect ap before gota"
            #download and install GOTA package
            self.gota.download_package_and_auto_retry()
            self.gota.restart_and_install_package()
            #check the fingerprint after gota
            Fingerprint_after_GOTA = os.popen("adb -s %s shell getprop ro.build.fingerprint" %self.serial).read()
            Fingerprint_after_GOTA = Fingerprint_after_GOTA.replace("\n","",1)
            self.gota.insert_info("Fingerprint after GOTA", Fingerprint_after_GOTA)
            print "[INFO] --- check the fingerprint before gota is: %s" %Fingerprint_after_GOTA