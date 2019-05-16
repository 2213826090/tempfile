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
@summary: test data is not erased after GOTA update
@since: 11/12/2014
@author: Sam Lan(samx.lan@intel.com)
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.system.system_impl import SystemImpl as SystemImpl
from testlib.apk.apk_install_uninstall_impl import ApkInstallUninstallImpl
from testlib.util.common import g_common_obj
#from testlib.domains.settings_impl import SettingsImpl
class InstalledAPKUnerasedAfterGOTA(UIATestBase):
    """
    @summary: Test installed apk is not erased after GOTA update
    """

    def setUp(self):
        super(InstalledAPKUnerasedAfterGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
        self.system=SystemImpl(self.cfg)
        self.apk=ApkInstallUninstallImpl(self.cfg)
        self.ssid = self.config.read(cfg_file,'wifisetting').get("ssid")
        self.passwd = self.config.read(cfg_file,'wifisetting').get("passwd")
	self.d=g_common_obj.get_device()
	self.locator=ApkInstallUninstallImpl.Locator(self.d)
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(InstalledAPKUnerasedAfterGOTA, self).tearDown()
        self.cfg = None

    def testInstalledAPKUnerasedAfterGOTA(self):
        """
        This test case is to test installed apk is not erased after GOTA update

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
        1. install apk
        2. process GOTA update
        3. check installed apk is not erased after GOTA 

        Expect Result:
        1. apk is installed successfully
        2. DUT can GOTA update successfully
        3. installed apk is not erased after GOTA update

        """

        print "[RunTest]: %s" % self.__str__()

        apk_name=self.cfg.get("apk_name")
        apk_file=self.cfg.get("apk_file")
	self.locator.btn_apps.click()
        assert self.apk.apk_check(apk_name)
        #self.apk.apk_uninstall(apk_name)

