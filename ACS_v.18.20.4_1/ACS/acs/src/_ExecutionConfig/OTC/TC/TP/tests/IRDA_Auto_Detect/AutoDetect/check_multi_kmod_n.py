"""
@summary: check haclctl -l at other user
@casename:/System_IRDA_Feature/
@since: 7/17/2015
@author: Song, GuimeiX Z <guimeix.z.song@intel.com>
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase


class Check_Multi_Kmod_N(UIATestBase):
    """
    check haclctl -l
    """

    def setUp(self):
        super(Check_Multi_Kmod_N, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.hal = AutodetectImpl()
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_Multi_Kmod_N, self).tearDown()
        self.hal=None
    def testCheck_Multi_Kmod_N(self):
        """
        This test case is to check : adb shell haclctl -l

        Test Case Precondition:
        None

        Test Case Step:
        1. Type adb shell halctl -l | grep kmod
        2. Type kmod -n kmod_model

        Expect Result:
        1    Get kmod information.
        2    It will display very verbose information about coretemp module

        """

        print "[RunTest]: %s" % self.__str__()
        self.hal.wake_up()
        self.hal.add_one_multi_user()
        res = g_common_obj2.root_on_device()
        print res
        cmdstr="adb shell halctl -l | grep kmod"
        halctl_cmd=os.popen(cmdstr).read().strip()
        print halctl_cmd
        kmod_model=halctl_cmd.replace('\r\n\t', ' ').split()[0].split(':')[1]
        cmdstr="adb shell kmod -n %s"%kmod_model
        halctl_cmd=os.popen(cmdstr).read()
        print halctl_cmd
        search_result=halctl_cmd.count('alias')
        assert search_result>0,"[Error]:kmod -n  coretemp  is fail"
        print "[INFO]: kmod -n  coretemp  is success"
        self.hal.switch_to_owner()
