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
@summary: Test singed package can update to singed package
@since: 11/12/2014
@author: Sam Lan(samx.lan@intel.com)
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.system_domains.system_impl import SystemImpl
from testlib.system.system_impl import SystemImpl as SystemImpl2

#from testlib.domains.settings_impl import SettingsImpl
class SignedPackageUpdateAfterGOTA(UIATestBase):
    """
    @summary: Test data is not erased after GOTA update
    """

    def setUp(self):
        super(SignedPackageUpdateAfterGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.system= SystemImpl(self.cfg)
        self.system2=SystemImpl2(self.cfg)
        self.ssid = self.config.read(cfg_file,'wifisetting').get("ssid")
        self.passwd = self.config.read(cfg_file,'wifisetting').get("passwd")
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SignedPackageUpdateAfterGOTA, self).tearDown()
        self.cfg = None

    def testSignedPackageUpdateAfterGOTA(self):
        """
        This test case is to test basic GOTA update function

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
        1. check the base package is signed package 
        2. process GOTA update
        3. check the DUT can update to signed version

        Expect Result:
        1. signed checks successfully
        2. DUT can GOTA update successfully
        3. The DUT can update to the corresponding signed version

        The real implementation will be in SystemImpl class.
        """

        print "[RunTest]: %s" % self.__str__()
        #base="COHOL00315"
        #target="COHOL00355"t
        base=self.cfg.get("base")
        target=self.cfg.get("target")
        base_bios=self.cfg.get("base_signed_fingerprint")
        target_bios=self.cfg.get("target_signed_fingerprint")
        base_build=self.cfg.get("base_build")
        #phone flash the base build
        self.system.phone_flash_tool_build_zip_file(base_build)
        #push uiautomator jar and skill inital screen after flash
        self.system.push_uiautomator_jar()
        self.system.skip_initial_screen_after_flash()
        #enable developer option, keep awake, clock lock screen, connect Ap for gota
        self.system.enable_developer_option()
        self.system.keep_awake()
        self.system.close_lock_screen()
        self.system.connect_AP(self.ssid, self.passwd)
        #process gota
        self.system.check_fingerprint(base_bios)
        self.system.basic_gota_update(base, target)
        self.system.check_fingerprint(target_bios)
