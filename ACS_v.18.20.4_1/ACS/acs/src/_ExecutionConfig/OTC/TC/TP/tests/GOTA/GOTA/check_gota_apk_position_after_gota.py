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
@summary: test app position after GOTA update
@since: 10/3/2015
@author: haley.han(yi.a.han@intel.com)
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.util.common import g_common_obj
from testlib.systemui.systemui_impl import SystemUI
class ApkPositionAfterGOTA(UIATestBase):
    """
    @summary: test app position after GOTA update
    """

    def setUp(self):
        super(ApkPositionAfterGOTA, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ApkPositionAfterGOTA, self).tearDown()
        self.cfg = None

    def testApkPositionAfterGOTA(self):
        """
        This test case is to test app position after GOTA update

        Test Case Precondition:
        set screen lock as none, and sleep time to be more than 10 minutes

        Test Case Step:
        1. process GOTA update
	2. test app position after GOTA update

        Expect Result:
        1. app postion nochange after GOTA

        """

        print "[RunTest]: %s" % self.__str__()
	from igascomparator import igascomparator
	comp=igascomparator()
	rate= comp.getsimilarityrate("/tmp/gota/applistbefore.png","/tmp/gota/applistafter.png")
	print "rate %s" %rate
	assert rate>0.8

