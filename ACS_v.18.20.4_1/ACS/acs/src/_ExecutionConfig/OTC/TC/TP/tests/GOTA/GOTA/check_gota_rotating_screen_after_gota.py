
'''
@summary: test rotating screen after gota
@since: 9/10/2014
@author: Su Chaonan(chaonanx.su@intel.com)
'''
#import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj

class RotatingScreenAfterGOTA(UIATestBase):
    """
    @summary: Test rotating screen after gota
    """

    def setUp(self):
        super(RotatingScreenAfterGOTA, self).setUp()
        self._test_name = __name__
	self.d=g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RotatingScreenAfterGOTA, self).tearDown()


    def testRotatingScreenAfterGOTA(self):
        """
        This test case is to test rorating screen after gota

        Test Case Precondition:

        Test Case Step:
        1. rotating screen after gota
        Expect Result:
        rotating successfully

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
