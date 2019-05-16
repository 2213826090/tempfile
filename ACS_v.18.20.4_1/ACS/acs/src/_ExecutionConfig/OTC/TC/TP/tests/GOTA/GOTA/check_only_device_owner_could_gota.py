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
@summary: Test only device owner could gota
@since: 10/3/2015
@author: haley han(yi.a.han@intel.com)
'''
import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.util.common import g_common_obj
class CheckOnlyDeviceOwnerCouldGOTA(UIATestBase):
    """
    @summary: Test only device owner could gota
    """

    def setUp(self):
        super(CheckOnlyDeviceOwnerCouldGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''),'tests.tablet.gota.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
        self.d=g_common_obj.get_device()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.gota.switch_to_owner()
        super(CheckOnlyDeviceOwnerCouldGOTA, self).tearDown()
        self.cfg = None

    def testCheckOnlyDeviceOwnerCouldGOTA(self):
        """
        This test case is to Test only device owner could gota

        Test Case Precondition:
        N/A

        Test Case Step:
        1. change to another user
        2. check gota update

        Expect Result:
        1. change success
        2. couldn't gota
        """

        print "[RunTest]: %s" % self.__str__()
        self.gota.unlock_screen()
        self.gota.switch_to_Guest()
        self.gota.unlock_screen()
        self.gota.only_device_owner_could_GOTA()
