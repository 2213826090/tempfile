'''
@summary: test device owner work Sample MDM is successful after gota
@since: 1/6/2015
@author: Yang, NaX <nax.yang@intel.com>
'''

import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.util.common import g_common_obj
from testlib.AfW.api_impl import ApiImpl

class CheckDeviceOwnerAfterGOTA(UIATestBase):

    def setUp(self):
        super(CheckDeviceOwnerAfterGOTA, self).setUp()
        self._test_name = __name__
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), 'tests.tablet.gota.conf')
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
        self.api = ApiImpl()
        self.d = g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
        self.ssid = self.config.read(cfg_file,'wifisetting').get("ssid")
        self.passwd = self.config.read(cfg_file,'wifisetting').get("passwd")

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckDeviceOwnerAfterGOTA, self).tearDown()
        self.cfg = None

    def testCheckDeviceOwnerAfterGOTA(self):
        print "[RunTest]: %s" % self.__str__()
        #flash DUT
        base_build = self.cfg.get("base_build")
        self.gota.phone_flash_tool_build_zip_file(base_build)
        self.gota.push_uiautomator_jar()
        g_common_obj.set_vertical_screen()
        #check Connect AP before gota
        self.gota.launch_settings()
        print "self.ssid: %s" %self.ssid
        print "self.passwd: %s" %self.passwd
        if self.ssid == None:
            print "using default ap setting"
            self.gota.connect_AP("SHZ13F-OTC-irda101", "zxcvbnm!?")
        else:
            print "using config ap setting"
            self.gota.connect_AP(self.ssid, self.passwd)
        print "[Info] --- Connect ap before gota"
        #setup init DUT
        self.gota.enable_developer_option()
        self.gota.keep_awake()
        self.gota.close_lock_screen()
        self.gota.accept_unknow_resource()
        self.gota.unverify_apps_over_USB()
        self.d.press.back()
        self.d.press.back()
        #setup Sample MDM in oobe
        self.api.oobe_setup(True)
        #check Sample MDM(Profile Owner) before gota
        self.api.launch_app("Sample MDM")
        self.assertFalse(self.d(text="Sample MDM(Profile Owner)").exists, "[ERROR]:Fail to provision device as device owner")
        #check data in personal profile before gota
        self.gota.check_AFW_data_in_personal_profile_before_GOTA()
        #download and install GOTA package
        self.gota.download_package_and_auto_retry()
        self.gota.restart_and_install_package()
        #check Sample MDM(Profile Owner) after gota
        self.gota.check_AFW_devices_owner_after_GOTA()