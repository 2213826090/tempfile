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
@summary: test interacting app during gota
@since: 10/3/2015
@author: haley(yi.a.han@intel.com)
'''
#import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj

class InteractingWithAppDuringGOTA(UIATestBase):
    """
    @summary: Test interacting app during gota
    """

    def setUp(self):
        super(InteractingWithAppDuringGOTA, self).setUp()
        self._test_name = __name__
	self.d=g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(InteractingWithAppDuringGOTA, self).tearDown()


    def testInteractingWithAppDuringGOTA(self):       
        """
        This test case is to test interacting app during gota

        Test Case Precondition:

        Test Case Step:
        1. cpen chrome when gota downloading

        Expect Result:
        1. app open successfully
        3. gota successfully

        The real implementation will be in gota_Impl class.
        """
	print "[RunTest]: %s" % self.__str__()
	g_common_obj.launch_app_from_home_sc("Chrome")
    time.sleep(2)
    g_common_obj.back_home()



