#Copyright (C) 2014  Lan, SamX <samx.lan@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

'''
@summary: test different builds can Update same gota update package
@since: 01/08/2014
@author: Sam Lan(samx.lan@intel.com)
'''
import time
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.system_domains.system_impl import SystemImpl
from testlib.system.system_impl import SystemImpl as SystemImpl2
from testlib.apk.apk_install_uninstall_impl import ApkInstallUninstallImpl
#from testlib.domains.settings_impl import SettingsImpl
class SameDeviceUpdateSameGOTAImage(UIATestBase):
    """
    @summary: Test basic GOTA update
    """

    def setUp(self):
        super(SameDeviceUpdateSameGOTAImage, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.system= SystemImpl(self.cfg)
        self.system2=SystemImpl2(self.cfg)
        self.apk=ApkInstallUninstallImpl(self.cfg)
        self.ssid = self.config.read(cfg_file,'wifisetting').get("ssid")
        self.passwd = self.config.read(cfg_file,'wifisetting').get("passwd")
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SameDeviceUpdateSameGOTAImage, self).tearDown()
        self.cfg = None

    def testSameDeviceUpdateSameGOTAImage(self):
        """
        This test case is to test different builds can Update sameGOTA update function

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
        1. flash base build1
        2. build1 can Update correct gota package
        3. flash base build2
        4. build2 can Update correct gota package as step2

        Expect Result:
        1. flashed successfully.
        2. correct package is Updateed
        3. flashed successfully/
        4. Updated correct package as step2

        The real implementation will be in SystemImpl class.
        """

        print "[RunTest]: %s" % self.__str__()


        base1=self.cfg.get("base1")
        base2=self.cfg.get("base2")
        target=self.cfg.get("target")
        base_build1=self.cfg.get("base_build_1")
        base_build2=self.cfg.get("base_build_2")

        #phone flash base build, and init screen
        self.system.phone_flash_tool_build_zip_file2(base_build1, "1")
        self.system.push_uiautomator_jar()
        self.system.skip_initial_screen_after_flash()
        self.system.enable_developer_option()
        self.system.keep_awake()
        self.system.close_lock_screen()
        self.system.accept_unknow_resource()
        self.system.unverify_apps_over_USB()
        self.system.connect_AP(self.ssid, self.passwd)
        #process basic gota update
	self.system.wait_gotastatus_to_downloaded()
	self.system.basic_gota_update()
	Fingerprint_after_GOTA = os.popen("adb shell getprop ro.build.fingerprint").read()
	Fingerprint_after_GOTA=Fingerprint_after_GOTA.replace("\n","",1)
	fingerprint=self.system.get_info("Fingerprint after GOTA")
	print "fingerprint %s" % fingerprint
	print "Fingerprint_after_GOTA %s" % Fingerprint_after_GOTA
	assert cmp(fingerprint,Fingerprint_after_GOTA)
        #phone flash base build, and init screen
        #self.system.phone_flash_tool_build_zip_file2(base_build2, "2")
        #self.system.push_uiautomator_jar()
        #self.system.skip_initial_screen_after_flash()
        #self.system.enable_developer_option()
        #self.system.keep_awake()
        #self.system.close_lock_screen()
        #self.system.accept_unknow_resource()
        #self.system.unverify_apps_over_USB()
        #self.system.connect_AP(self.ssid, self.passwd)
        #process basic gota update
        #self.system.basic_gota_update(base2, target)

