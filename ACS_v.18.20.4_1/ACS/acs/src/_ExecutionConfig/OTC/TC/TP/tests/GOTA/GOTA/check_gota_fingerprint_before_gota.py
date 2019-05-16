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
@summary: test fingerprint before GOTA update
@since: 1/20/2015
@author: Sam Lan(samx.lan@intel.com)
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.system.system_impl import SystemImpl
from testlib.apk.apk_install_uninstall_impl import ApkInstallUninstallImpl

class CheckFingerprintoBeforeGOTA(UIATestBase):
    """
    @summary: Test fingerprint before GOTA update
    """

    def setUp(self):
        super(CheckFingerprintoBeforeGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''),'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
        self.system=SystemImpl(self.cfg)

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckFingerprintoBeforeGOTA, self).tearDown()
        self.cfg = None

    def testCheckFingerprintoBeforeGOTA(self):
        """
        This test case is to test basic GOTA update function

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
        1. test fingerprint before GOTA update

        Expect Result:
        1. DUT can fingerprit before GOTA update successfully

        """

        print "[RunTest]: %s" % self.__str__()
        #base_fingerprint=self.cfg.get_info("base_fingerprint")
        base_fingerprint=self.gota.get_info("Fingerprint before GOTA")
        #check the fingerprint before gota
        assert base_fingerprint
        self.gota.check_fingerprint(base_fingerprint)
        print "[INFO]check the fingerprint before gota is: %s" %base_fingerprint

