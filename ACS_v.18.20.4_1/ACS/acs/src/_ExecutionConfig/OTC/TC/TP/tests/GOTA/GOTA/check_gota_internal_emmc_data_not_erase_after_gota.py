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
from testlib.system.system_impl import SystemImpl
#from testlib.domains.settings_impl import SettingsImpl
class InternalEmmcDataNotEraseAfterGOTA(UIATestBase):
    """
    @summary: Test data is not erased after GOTA update
    """

    def setUp(self):
        super(InternalEmmcDataNotEraseAfterGOTA, self).setUp()
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
        super(InternalEmmcDataNotEraseAfterGOTA, self).tearDown()
        self.cfg = None

    def testInternalEmmcDataNotErase(self):
        """
        This test case is to test basic GOTA update function

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
        1. push some file into data
        2. process GOTA update
        3. check thee file is not erased after GOTA

        Expect Result:
        1. file is pushed successfully
        2. DUT can GOTA update successfully
        3. file exist after GOTA update

        The real implementation will be in SystemImpl class.
        """

        print "[RunTest]: %s" % self.__str__()
        #base="COHOL00315"
        #target="COHOL00355"t
        #set the screen lock to be None
        #self.setting.launch_settings()
        #self.setting.set_screen_lock("None")
        #do gota update from base to target

        #host_file="/home/sam/ota/aaa.txt"
        #client_file="/data/aaa.txt"
        base=self.cfg.get("base")
        target=self.cfg.get("target")
        base_build=self.cfg.get("base_build")
        host_file=self.cfg.get("host_file")
        client_file=self.cfg.get("client_file")

        assert self.system.file_exists(client_file, "DEVICE"), "[ERROR]: The client file %s does not exist" % client_file
        #assert os.popen("rm %s" %host_file), "[ERROR]: host_file %s is removed" %host_file

