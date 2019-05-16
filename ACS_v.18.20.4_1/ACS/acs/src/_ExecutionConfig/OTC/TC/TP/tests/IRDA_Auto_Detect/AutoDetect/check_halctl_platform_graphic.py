"""
@summary: check
@casename:/System_IRDA_Feature/Platform - Graphics
@since: 7/8/2015
@author: Song, GuimeiX Z <guimeix.z.song@intel.com>
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase
import time

class Check_Platform_Graphics(UIATestBase):
    """
    check haclctl -l
    """

    def setUp(self):
        super(Check_Platform_Graphics, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.hal = AutodetectImpl()
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_Platform_Graphics, self).tearDown()
        self.hal =None
    def testCheck_Platform_Graphics(self):
        """
        This test case is to check : adb shell haclctl -l

        Test Case Precondition:
        None

        Test Case Step:
1    adb root -->adb shell
2    Type halctl -i gralloc



        Expect Result:
1    Connect device and root device successful
2    Bindings list is displayed
        """

        print "[RunTest]: %s" % self.__str__()
        halctl_keyword1='graphic'
        halctl_keyword2='gralloc'
        res = g_common_obj2.root_on_device()
        print res
        cmdstr1="adb shell halctl -i %s"%halctl_keyword1
        cmdstr2="adb shell halctl -i %s"%halctl_keyword2
        halctl_cmd1=os.popen(cmdstr1).read()
        print halctl_cmd1
        time.sleep(2)
        search_result=halctl_cmd1.count('Binding')
        time.sleep(1)
        if search_result == 0:
            halctl_cmd2=os.popen(cmdstr2).read()
            search_result=halctl_cmd2.count('Binding')
        assert search_result>0,"[ERROR]: halctl -i %s  is fail"%halctl_keyword2
        print "[INFO]: halctl -i %s is success"%halctl_keyword2
