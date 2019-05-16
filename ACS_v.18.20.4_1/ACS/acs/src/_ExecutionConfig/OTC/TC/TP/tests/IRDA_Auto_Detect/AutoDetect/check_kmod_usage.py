"""
@summary: check kmod usage
@casename:/System_IRDA_Feature/Auto_detect kmod
@since: 7/8/2015
@author: Song, GuimeiX Z <guimeix.z.song@intel.com>
"""
import os
from testlib.common.common import g_common_obj2
from testlib.util.uiatestbase import UIATestBase


class CheckKmod_Usage(UIATestBase):
    """
    check kmod usage
    """

    def setUp(self):
        super(CheckKmod_Usage, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckKmod_Usage, self).tearDown()
    def testCheckKmod_Usage(self):
        """
        This test case is to check : adb shell kmod usage

        Test Case Precondition:
        None

        Test Case Step:
        1. Run "adb shell kmod  /-h/--help"

        Expect Result:
        1. It shows usage of kmod --help

        """

        print "[RunTest]: %s" % self.__str__()
        res = g_common_obj2.root_on_device()
        print res
        cmdstr="adb shell kmod"
        print cmdstr
        halctl_cmd=os.popen(cmdstr).read()
        print halctl_cmd
        search_result=halctl_cmd.count('Usage')
        assert search_result>0,"[ERROR]: adb shell kmod  is fail"
        print "[INFO]: adb shell kmod is success"
