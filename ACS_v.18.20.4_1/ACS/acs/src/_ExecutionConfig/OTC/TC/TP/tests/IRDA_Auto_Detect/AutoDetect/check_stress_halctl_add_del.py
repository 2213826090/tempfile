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
class Check_stress_halctl_add_del(UIATestBase):
    """
    Stress - halctl --add/--del
    """
    def setUp(self):
        super(Check_stress_halctl_add_del, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        self.cfg=self.config.read(cfg_file, 'system_domain')
        self.hal = AutodetectImpl()
        print "[Setup]: %s" % self._test_name
#         self.cfg = self.config.read(cfg_file, 'system_domain')

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_stress_halctl_add_del, self).tearDown()
        self.hal = None
        self.hal=None

    def testCheck_stress_halctl_add_del(self):
        """
        This test case is to check : Stress - halctl --add/--del
        Test Case Precondition:
        NoneSystemImpl

        Test Case Step:
1    Type halctl --add new_entries
2    Halctl -l
3    Type halctl --del new_entries
4    Halctl -l
5    Repeat steps 1 to 4 ten times

        Expect Result:
1    No error occurs.
2    No new entry was added. refcount was incremented.
3    No error occurs
4    New_entries is not listed anymore
5    No error uccurs
        """
        print "[RunTest]: %s" % self.__str__()
        res = g_common_obj2.root_on_device()
        print res
        #modalias=self.cfg.get("sensors_module_alias")
        modalias2='usbbbbb'
        i=5
        while i>0:
            print "loop times: %s"%i
            self.hal.halctl_a(modalias2)
            time.sleep(3)
            self.hal.halctl_s(modalias2)
            i=i-1