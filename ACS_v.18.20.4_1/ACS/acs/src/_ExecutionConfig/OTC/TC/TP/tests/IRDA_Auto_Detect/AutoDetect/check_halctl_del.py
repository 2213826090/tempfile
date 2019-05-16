"""
@summary: check haclctl --del
@casename:/System_IRDA_Feature/Auto_detect halctl --del
@since: 11/04/2014
@author: Yuhui Xu(yuhuix.xu@intel.com)
"""
import os
from testlib.common.common import g_common_obj2
from testlib.util.uiatestbase import UIATestBase
from testlib.autodetect.autodetect_impl import AutodetectImpl


class CheckHalctl_del(UIATestBase):
    """
    check haclctl -l
    """

    def setUp(self):
        super(CheckHalctl_del, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'system_domain')
        self.cfg = self.config.read(cfg_file, 'sensors_module_alias')
        self.hal = AutodetectImpl(self.config.read(cfg_file, 'system_domain'))
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckHalctl_del, self).tearDown()
        self.cfg = None

    def testCheckHalctl_del(self):
        """
        This test case is to check : adb shell haclctl --del sensors_module_alias

        Test Case Precondition:
        None

        Test Case Step:
        1. Run "adb shell haclctl -l"

        Expect Result:
        1. It shows more than 100 modalias

        """

        print "[RunTest]: %s" % self.__str__()
        res = g_common_obj2.root_on_device()
        print res
       
        modalias2='usbbbbb'
        self.hal.halctl_a(modalias2)
        halctl_i_search=self.hal.halctl_i(modalias2).count('usbbbbb')
        assert halctl_i_search>0, "[ERROR]: halctl -i %s  is fail"%modalias2
        print "[INFO]: halctl -i %s is success"%modalias2       
        self.hal.halctl_s(modalias2)
        halctl_i_search=self.hal.halctl_i(modalias2).count('usbbbbb')
        assert halctl_i_search==0, "[ERROR]: halctl -i %s  is fail"%modalias2 
        print "[INFO]: halctl -i %s is success"%modalias2

