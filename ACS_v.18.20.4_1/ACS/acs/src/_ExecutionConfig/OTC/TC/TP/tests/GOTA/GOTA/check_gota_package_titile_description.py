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
@since: 12/04/2014
@author: yuhui.xu(yuhuix.xu@intel.com)
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.system.system_impl import SystemImpl

#from testlib.domains.settings_impl import SettingsImpl
class Check_GOTA_package_titile_description(UIATestBase):
    """
    @summary: Test fingerprint is update correctly after GOTA update
    """

    def setUp(self):
        super(Check_GOTA_package_titile_description, self).setUp()
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
        super(Check_GOTA_package_titile_description, self).tearDown()
        self.cfg = None

    def testCheck_titile_description(self):
        """
        This test case is to Check GOTA package include title and description

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
        1    Flash N version
        2    Goto Settings->About tablet->System updates, click button "Check now" and download GOTA update package
        3    Check the GOTA package's interface description

        Expect Result:
        1    The N version can flash successfully
        2    There is a new package can detect and download
        3    "The GOTA package include title and description
        eg:
        title: Update to COHO build L00535
        description: Thiscrelease contains Lollipop enhancements and fixes for your device. For more information visit http://android.com/whatsnew/#lollipop"


        The real implementation will be in SystemImpl class.
        """

        print "[RunTest]: %s" % self.__str__()
        #base="COHOL00315"
        #target="COHOL00355"t
        #description_keyword=self.cfg.get("description_keyword")
        base=self.cfg.get("base")
        target=self.cfg.get("target")
        base_build=self.cfg.get("base_build")
        gota_test_run_by_testplan=self.cfg.get("gota_test_run_by_testplan")
        self.gota.check_gota_title_description()
