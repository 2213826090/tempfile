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
@summary: Test encryption status correctly after GOTA update
@since: 12/10/2014
@author: Sam Lan(samx.lan@intel.com)
'''
import os
import string
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.system.system_impl import SystemImpl

#from testlib.domains.settings_impl import SettingsImpl
class FingerprintUpdateAfterGOTA(UIATestBase):
    """
    @summary: Test encryption status correctly after GOTA update
    """

    def setUp(self):
        super(FingerprintUpdateAfterGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
        self.system=SystemImpl(self.cfg)
        self.ssid = self.config.read(cfg_file,'wifisetting').get("ssid")
        self.passwd = self.config.read(cfg_file,'wifisetting').get("passwd")
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(FingerprintUpdateAfterGOTA, self).tearDown()
        self.cfg = None

    def testFingerprintUpdateAfterGOTA(self):
        """
        This test case is to test encryption status correctly after GOTA update

        Test Case Precondition:
        N/A

        Test Case Step:
        1. get the encryption status
        2. process GOTA update
        3. check the encryption status

        Expect Result:
        1. the encryption status is correct
        2. DUT can GOTA update successfully
        3. check the encryption status is correct

        The real implementation will be in SystemImpl class.
        """

        print "[RunTest]: %s" % self.__str__()
        #base="COHOL00315"
        #target="COHOL00355"t
        base=self.cfg.get("base")
        target=self.cfg.get("target")
        base_fingerprint=self.cfg.get("base_fingerprint")
        target_fingerprint=self.cfg.get("target_fingerprint")
        base_build=self.cfg.get("base_build")

        base_fingerprint=self.gota.get_info("Fingerprint before GOTA")
        target_fingerprint=self.gota.get_info("Fingerprint after GOTA")
        assert target_fingerprint
        assert cmp(base_fingerprint,target_fingerprint)
        self.gota.check_fingerprint(target_fingerprint)
        fplist=target_fingerprint.split('/')
        version_str=fplist[4].encode('utf-8')
        version=filter(str.isdigit,version_str)
        self.gota.insert_info("Target image",version)
        self.gota.update_info(self.gota.get_filename())
        print "[INFO]check the fingerprint before gota is: %s" %base_fingerprint
