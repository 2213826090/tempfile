"""
@summary: check kmod -n
@casename:/System_IRDA_Feature/Auto_detect kmod -n
@since: 7/8/2015
@author: Song, GuimeiX Z <guimeix.z.song@intel.com>
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase

class CheckKmod_N(UIATestBase):
    """
    check kmod -n
    """

    def setUp(self):
        super(CheckKmod_N, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.hal = AutodetectImpl()
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckKmod_N, self).tearDown()
        self.hal=None
    def testCheckKmod_N(self):
        """
        This test case is to check kmod -n

        Test Case Precondition:
        None

        Test Case Step:
1    Connect to the device using adb for root session
2    Type adb shell halctl -l | grep kmod
3    Type kmod -n kmod_model



        Expect Result:
1    Connection is successful. Root session is available.
2    Get kmod information.
3    It will display very verbose information about coretemp module



        """

        print "[RunTest]: %s" % self.__str__()
        res = g_common_obj2.root_on_device()
        print res
        cmdstr="adb shell halctl -l | grep kmod"
        print cmdstr
        halctl_cmd=os.popen(cmdstr).read().strip()
        print halctl_cmd
        kmod_model=halctl_cmd.replace('\r\n\t', ' ').split()[0].split(':')[1]
        print kmod_model
        cmdstr="adb shell kmod -n %s"%kmod_model
        print cmdstr
        halctl_cmd=os.popen(cmdstr).read()
        print halctl_cmd
        search_result=halctl_cmd.count('alias')
        assert search_result>0,"[Error]:kmod -n  coretemp  is fail"
        print "[INFO]: kmod -n  coretemp  is success"