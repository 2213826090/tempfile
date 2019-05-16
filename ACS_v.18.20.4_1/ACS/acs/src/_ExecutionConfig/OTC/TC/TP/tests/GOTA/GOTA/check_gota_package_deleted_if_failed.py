'''
@summary: Check gota package is deleted automatically if gota fails
@since: 8/6/2015
@author: Yang, NaX <nax.yang@intel.com>
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.common.common import g_common_obj
from testlib.util.device import TestDevice

class Gota_package_deleted_if_failed(UIATestBase):
    """
    @summary: check if the package was deleted
    """

    def setUp(self):
        super(Gota_package_deleted_if_failed, self).setUp()
        self._test_name = __name__
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), 'tests.tablet.gota.conf')
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota = gotaImpl(self.cfg)
        print "[Setup]: %s" % self._test_name
        self.ssid = self.config.read(cfg_file,'wifisetting').get("ssid")
        self.passwd = self.config.read(cfg_file,'wifisetting').get("passwd")

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Gota_package_deleted_if_failed, self).tearDown()
        self.cfg = None

    def testGota_package_deleted_if_failed(self):
        """
        This test case is to test basic GOTA update function

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
        1    Goto Settings->About tablet->System updates, click button "Check now" and download GOTA update package
        2    Turn off Wi-Fi to interrrupt GOTA package downloading
        3    adb root, adb shell, and cd /cache or cd /data to check update.zip package

        Expect Result:
        1    There is a new package can detect and downloading
        2    The GOTA downloading is interrupted
        3    The update.zip package is automatically deleted when the GOTA package download fails

        The real implementation will be in SystemImpl class.
        """

        print "[RunTest]: %s" % self.__str__()
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
        #remount DUT
        self.gota.remount_device()
        #download and install GOTA package
        self.gota.download_package_and_auto_retry()
        self.gota.restart_and_install_package_after_remount()
        #check gota package after gota
        self.gota.check_gota_package_deleted()