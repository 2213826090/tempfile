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

from testlib.util.uiatestbase import UIATestBase
from testlib.systemui.systemui_impl import SystemUI

class Suspend(UIATestBase):
    """
    @summary: Test suspend and wakeup
    """
    def setUp(self):
        super(Suspend, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.systemui = SystemUI()

    def tearDown(self):
        super(Suspend, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testSuspend(self):
        """
        This test case is to test suspend and wakeup

        Test Case Step:
        1. Make the device sleep for 60s
        2. Make the device wakeup
        """
        print "[RunTest]: %s" % self.__str__()

        self.systemui.suspend_wakeup()
