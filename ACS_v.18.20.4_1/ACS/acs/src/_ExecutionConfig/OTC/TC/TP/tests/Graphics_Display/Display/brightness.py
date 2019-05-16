#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
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

"""
@summary: Test brightness
@since: 09/20/2014
@author: Grace Yi (gracex.yi@intel.com)
"""

import time
from testlib.brightness.brightness import BrightnessBSPImpl
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj

class BrightnessBSPTest(UIATestBase):
    """
    @summary: Test backlight funcitonalities
    """
    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(BrightnessBSPTest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.brightness = BrightnessBSPImpl()

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(BrightnessBSPTest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.back_home()
        self.brightness.reset()
        self.brightness = None

    def testBrightness(self):
        """
        This test used to test brightness

        The test case spec is following:
        1. get brightness level
        2. set brightness level
        3. check brightness
        """
        print "[RunTest]: %s" % self.__str__()
        s_level = self.brightness.get_brightness_level()
        if s_level == 0:
            self.brightness.set_brightness_level(255)
        elif s_level == 255:
            self.brightness.set_brightness_level(0)
        else:
            self.brightness.set_brightness_level(0)
            self.brightness.set_brightness_level(255)
        e_level = self.brightness.get_brightness_level()
        assert not s_level == e_level, "Error Brightness not changed."