#Copyright (C) 2015  Song, GuimeiX Z
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
@summary: test downloading apk during gota
@since: 1/7/2015
@author:  Song, GuimeiX Z < guimeix.z.song@intel.com>
'''
import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.dut_init.dut_init_impl import Function

#from testlib.domains.settings_impl import SettingsImpl
class DownloadingDuringGOTA(UIATestBase):
    """
    @summary: Test downloading apk during gota
    """

    def setUp(self):
        super(DownloadingDuringGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DownloadingDuringGOTA, self).tearDown()
        self.cfg = None

    def testDownloadingDuringGOTA(self):
        """
        This test case is to test downloading apk during gota

        Test Case Precondition:

        Test Case Step:
        1. downloading apk in google market store
        2. during downloading, gota

        Expect Result:
        1. download apk in google market store
        3. gota successfully

        """

        print "[RunTest]: %s" % self.__str__()

        base=self.cfg.get("base")
        target=self.cfg.get("target")
        base_build=self.cfg.get("base_build")
        download_webside=self.cfg.get("website")
        print download_webside

        #self.gota.download_content(download_webside, "video-app.apk")
        self.gota.download_content(download_webside, "sample.apk")
        self.gota.insert_info("Download app status","start")
