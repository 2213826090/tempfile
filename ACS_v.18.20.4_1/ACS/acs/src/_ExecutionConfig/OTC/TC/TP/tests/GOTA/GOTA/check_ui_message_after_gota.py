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
@summary: Test encryption status correctly after GOTA update
@since: 10/3/2015
@author: haley han(yi.a.han@intel.com)
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
class CheckUIMessageAfterGOTA(UIATestBase):
    """
    @summary: Test encryption status correctly after GOTA update
    """

    def setUp(self):
        super(CheckUIMessageAfterGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckUIMessageAfterGOTA, self).tearDown()
        self.cfg = None

    def testCheckUIMessageAfterGOTA(self):
        """
        This test case is to test is there a UI message to end user gota success

        Test Case Precondition:
        N/A

        Test Case Step:
        1. process GOTA update
        2. check is there a UI message to end user gota success

        Expect Result:
        1. DUT can GOTA update successfully
        2. there is a UI message to end user gota success

        """

        print "[RunTest]: %s" % self.__str__()
        result=self.gota.get_info("GOTA update prompt")
        assert result=='Successfully'
        print "[INFO]is there a UI message to end user gota success: %s" %result

