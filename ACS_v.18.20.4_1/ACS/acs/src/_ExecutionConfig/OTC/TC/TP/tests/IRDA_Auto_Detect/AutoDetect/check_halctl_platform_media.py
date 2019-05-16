"""
@summary: check
@casename:/System_IRDA_Feature/Platform - Graphics
@since: 7/8/2015
@author: Song, GuimeiX Z <guimeix.z.song@intel.com>
"""
import os
from testlib.common.common import g_common_obj2
from testlib.util.uiatestbase import UIATestBase


class Check_Platform_Media(UIATestBase):
    """
    check haclctl -l
    """

    def setUp(self):
        super(Check_Platform_Media, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_Platform_Media, self).tearDown()
    def testCheck_Platform_Media(self):
        """
        This test case is to check : adb shell haclctl -i

        Test Case Precondition:
        None

        Test Case Step:
1    adb root -->adb shell
2    Type halctl -i media



        Expect Result:
1    Connect device and root device successful
2    Bindings list is displayed
       """
        print "[RunTest]: %s" % self.__str__()
        halctl_keyword1='video'
        halctl_keyword2='media'
        res = g_common_obj2.root_on_device()
        print res
        cmdstr1="adb shell halctl -i %s"%halctl_keyword1
        cmdstr2="adb shell halctl -i %s"%halctl_keyword2
        print cmdstr1
        halctl_cmd=os.popen(cmdstr1).read()
        print halctl_cmd
        search_result=halctl_cmd.count('Binding')
        if search_result == 0:
            halctl_cmd2=os.popen(cmdstr2).read()
            print halctl_cmd2
            search_result=halctl_cmd2.count('Binding')
        assert search_result>0,"[ERROR]: halctl -i %s  is fail"%halctl_keyword2
        print "[INFO]: halctl -i %s is success"%halctl_keyword2
