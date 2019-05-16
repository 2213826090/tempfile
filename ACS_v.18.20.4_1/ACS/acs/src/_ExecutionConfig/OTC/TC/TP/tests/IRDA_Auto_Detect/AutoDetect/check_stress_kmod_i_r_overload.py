"""
@summary: Stress - kmod -i/-r
@casename:/System_IRDA_Feature/Stress - kmod -i/-r
@since: 7/17/2015
@author: Song, GuimeiX Z <guimeix.z.song@intel.com>
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase
class Check_Stress_kmod_i_r_overload(UIATestBase):
    """
    Stress - halctl --add/--del
    """
    def setUp(self):
        super(Check_Stress_kmod_i_r_overload, self).setUp()
        self._test_name = __name__
        self.hal = AutodetectImpl()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_Stress_kmod_i_r_overload, self).tearDown()
        self.hal = None
        self.hal=None

    def testCheck_Stress_kmod_i_r_overload(self):
        """
        This test case is to check : Stress - kmod -i/-r
        
        Test Case Precondition:
        NoneSystemImpl

        Test Case Step:
1    Type kmod -r <module>
2    Check the logs
3    Type kmod -i <module>
4    Check the logs
5    Repeat steps 1 to 4 ten times

        Expect Result:
1    No error occurs
2    Module was unloaded
3    No error occurs
4    Module was loaded
5    Each step has the expected behavior no matter the iteration
        """
        print "[RunTest]: %s" % self.__str__()
        res = g_common_obj2.root_on_device()
        print res
        self.hal.make_device_cpu_overload()
        cmdstr="adb shell halctl -l | grep kmod"
        print cmdstr
        halctl_cmd=os.popen(cmdstr).read().strip()
        print halctl_cmd
        kmod_model=halctl_cmd.replace('\r\n\t', ' ').split()[0].split(':')[1]
        print kmod_model
        i=5
        while i>0:
            print "loop times: %s"%i
            self.hal.kmod_r(kmod_model)
            self.hal.kmod_i(kmod_model)
            i=i-1
        g_common_obj2.system_reboot()