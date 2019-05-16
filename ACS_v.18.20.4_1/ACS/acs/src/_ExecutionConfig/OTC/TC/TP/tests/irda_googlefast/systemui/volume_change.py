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

class Volume(UIATestBase):
    """
    @summary: Test change volume
    """
    def setUp(self):
        super(Volume, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.systemui = SystemUI()

    def tearDown(self):
        super(Volume, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testVolume(self):
        """
        This test case is to change volume

        Test Case Step:
        1. Launch settings app, select "Sound & notification"
        2. Change the volume
        """
        print "[RunTest]: %s" % self.__str__()

        self.systemui.change_volume_silent_settings()
