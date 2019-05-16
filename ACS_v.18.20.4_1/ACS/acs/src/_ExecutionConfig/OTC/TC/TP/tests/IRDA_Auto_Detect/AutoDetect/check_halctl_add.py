"""
@summary: check haclctl --add
@casename:/System_IRDA_Feature/Auto_detect halctl --add
@since: 7/13/2015
@author: Song, GuimeiX Z <guimeix.z.song@intel.com>
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase


class CheckHalctl_add(UIATestBase):
    """
    check haclctl -l
    """

    def setUp(self):
        super(CheckHalctl_add, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        self.hal = AutodetectImpl(self.config.read(cfg_file, 'system_domain'))
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'system_domain')

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckHalctl_add, self).tearDown()
        self.cfg = None
        self.hal=None

    def testCheckHalctl_add(self):
        """
        This test case is to check : adb shell haclctl -add

        Test Case Precondition:
        NoneSystemImpl

        Test Case Step:
1    adb root
2    adb shell
3    Type halctl --add new_module_alias
4    Type halctl -l press return return and check the




        Expect Result:
1    No error occurs.
2    No new entry was added. refcount was incremented.
3    No error occurs
4    New Sensors module is  listed anymore



        """

        print "[RunTest]: %s" % self.__str__()
        res = g_common_obj2.root_on_device()
        print res
        modalias2='usbbbbb'
        self.hal.halctl_a(modalias2)
        halctl_i_search=self.hal.halctl_i(modalias2).count('usbbbbb')
        assert halctl_i_search>0, "[ERROR]: halctl -i %s  is fail"%modalias2 
        print "[INFO]: halctl -i %s is success"%modalias2