#Copyright (C) 2015 haley.han
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
@summary: test apk install successfull after GOTA update
@since: 10/3/2015
@author: haley.han(yi.a.han@intel.com)
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.system.system_impl import SystemImpl as SystemImpl
from testlib.domains.settings_impl import SettingsImpl
from testlib.apk.apk_install_uninstall_impl import ApkInstallUninstallImpl
from testlib.util.common import g_common_obj
class ApkInstallAfterGOTA(UIATestBase):
    """
    @summary: Test data is not erased after GOTA update
    """

    def setUp(self):
        super(ApkInstallAfterGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
        self.system2=SystemImpl(self.cfg)
        self.apk=ApkInstallUninstallImpl(self.cfg)
        self.ssid = self.config.read(cfg_file,'wifisetting').get("ssid")
        self.passwd = self.config.read(cfg_file,'wifisetting').get("passwd")
        self.d=g_common_obj.get_device()
        self.locator=ApkInstallUninstallImpl.Locator(self.d)
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ApkInstallAfterGOTA, self).tearDown()
        self.cfg = None

    def testApkInstallAfterGOTA(self):
        """
        This test case is to test basic GOTA update function

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
        1. process GOTA update


        Expect Result:
        1. DUT can install apk successfully

        """

        print "[RunTest]: %s" % self.__str__()

        apk_name=self.cfg.get("apk_name")
        apk_file=self.cfg.get("apk_file")
        apk_website=self.cfg.get("apk_website")
        base_build=self.cfg.get("base_build")
        print "apk name %s" %apk_name
        print "apk apk_file %s" %apk_file
        self.gota.unlock_screen()
        self.locator.btn_apps.click()
        if self.apk.apk_check(apk_name):
            print "1"
            self.apk.apk_uninstall(apk_name)
            self.apk.apk_install(apk_file,apk_name)

