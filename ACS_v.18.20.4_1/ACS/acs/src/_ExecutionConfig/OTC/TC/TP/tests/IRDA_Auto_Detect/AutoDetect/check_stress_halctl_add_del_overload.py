"""
@summary: Stress - halctl --add/--del
@casename:/System_IRDA_Feature/Stress - halctl --add/--del
@since: 12/23/2014
@author: Yuhui Xu(yuhuix.xu@intel.com)
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase
import time
class Check_stress_halctl_add_del_overload(UIATestBase):
    """
    Stress - halctl --add/--del
    """
    def setUp(self):
        super(Check_stress_halctl_add_del_overload, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        self.cfg=self.config.read(cfg_file, 'system_domain')
        self.hal = AutodetectImpl()
        print "[Setup]: %s" % self._test_name
#         self.cfg = self.config.read(cfg_file, 'system_domain')

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_stress_halctl_add_del_overload, self).tearDown()
        self.hal = None
        self.hal=None

    def testCheck_stress_halctl_add_del_overload(self):
        """
        This test case is to check : Stress - halctl --add/--del
        Test Case Precondition:
        NoneSystemImpl

        Test Case Step:
1.   Make DUT overload via typing follow commdl:
adb shell "cat /dev/urandom > /dev/null & cat /dev/urandom > /dev/null & cat /dev/urandom > /dev/null & cat /dev/urandom > /dev/null & cat /dev/urandom > /dev/null & cat /dev/urandom > /dev/null"

2    Type halctl --add new_entries
3    Halctl -l
4    Type halctl --del new_entries
5    Halctl -l
6    Repeat steps 1 to 4 ten times
7    Close the terminal the overload command line running in,if that doesn't work,please reboot device.

        Expect Result:
1    DUT's cpu  is overload.
2    No error occurs.
3    No new entry was added. refcount was incremented.
4    No error occurs
5    Sensors module is not listed anymore
6    No error uccurs
7    Terminal is closed,make DUT back to normal
        """
        print "[RunTest]: %s" % self.__str__()
        res = g_common_obj2.root_on_device()
        print res
        self.hal.make_device_cpu_overload()
        modalias2='usbbbbb'
        i=5
        while i>0:
            print "loop times: %s"%i
            self.hal.halctl_a(modalias2)
            time.sleep(3)
            self.hal.halctl_s(modalias2)
            i=i-1
        g_common_obj2.system_reboot()
