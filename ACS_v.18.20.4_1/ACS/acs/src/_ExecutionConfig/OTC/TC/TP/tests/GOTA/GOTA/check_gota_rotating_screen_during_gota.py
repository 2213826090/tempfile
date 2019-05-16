#Copyright (C) 2015  haley,han
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
@summary: test rotating screen during gota
@since: 10/3/2014
@author: haley(yi.a.han@intel.com)
'''
#import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj

class RotatingScreenDuringGOTA(UIATestBase):
    """
    @summary: Test rotating screen during gota
    """

    def setUp(self):
        super(RotatingScreenDuringGOTA, self).setUp()
        self._test_name = __name__
	self.d=g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RotatingScreenDuringGOTA, self).tearDown()


    def testRotatingScreenDuringGOTA(self):       
        """
        This test case is to test rorating screen during gota

        Test Case Precondition:

        Test Case Step:
        1. rotating screen when gota downloading

        Expect Result:
        1. rotating successfully
        3. gota successfully

        The real implementation will be in gota_Impl class.
        """
	print "[RunTest]: %s" % self.__str__()
        self.d.orientation = "l"
        time.sleep(1)
        self.d.orientation = "u"
        time.sleep(1)
        self.d.orientation = "r"
        time.sleep(1)
        self.d.orientation = "n"
        g_common_obj.set_vertical_screen()
