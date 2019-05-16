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
@summary: Check GOTA package include title and description
@since: 12/08/2014
@author: yuhui.xu(yuhuix.xu@intel.com)
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.system.system_impl import SystemImpl as SystemImpl

#from testlib.domains.settings_impl import SettingsImpl
class Check_Gota_upgrade_package_size(UIATestBase):
    """
    @summary: Test fingerprint is update correctly after GOTA update
    """

    def setUp(self):
        super(Check_Gota_upgrade_package_size, self).setUp()
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
        super(Check_Gota_upgrade_package_size, self).tearDown()
        self.cfg = None

    def testCheck_Gota_upgrade_package_size(self):
        """
        This test case is to Check GOTA package include title and description

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
        1    Goto Settings->About tablet->System updates, click button "Check now" and download GOTA update package
        Expect Result:
        There is a new package can detect and  GOTA package  must smaller than 700M
        
        eg:(KK1290->L495)Downloaded and verified -633.2MB


        The real implementation will be in SystemImpl class.
        """

        print "[RunTest]: %s" % self.__str__()
        self.gota.check_gota_upgrade_package()
