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
@summary: test playing music during gota
@since: 29/6/2014
@author: Song, GuimeiX Z < guimeix.z.song@intel.com>
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
class PlayingMusicDuringGOTA(UIATestBase):
    """
    @summary: Test playing music during gota
    """

    def setUp(self):
        super(PlayingMusicDuringGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(PlayingMusicDuringGOTA, self).tearDown()
        self.cfg = None

    def testPlayingMusicDuringGOTA(self):
        """
        This test case is to test playing music during gota

        Test Case Precondition:

        Test Case Step:
        1. download and push music file into dut
        2. playing music
        3. connect wifi to gota

        Expect Result:
        1. music file is downloaded and pushed into dut successfully
        2. music is played
        3. gota successfully

        The real implementation will be in SystemImpl class.
        """

        print "[RunTest]: %s" % self.__str__()

        base=self.cfg.get("base")
        target=self.cfg.get("target")
        base_build=self.cfg.get("base_build")
        self.gota.audioPlayback()
        self.gota.insert_info("Play music status","start")