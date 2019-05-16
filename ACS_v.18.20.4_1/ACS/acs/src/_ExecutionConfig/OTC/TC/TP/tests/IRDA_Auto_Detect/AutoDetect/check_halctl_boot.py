"""
@summary: check dmesg after boot
@casename:/System_IRDA_Feature/Auto_detect Boot
@since: 11/11/2014
@author: Yuhui Xu(yuhuix.xu@intel.com)
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase


class Check_Halctl_Boot(UIATestBase):
    """
    check haclctl -l
    """

    def setUp(self):
        super(Check_Halctl_Boot, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.hal = AutodetectImpl(self.config.read(cfg_file, 'system_domain'))
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_Halctl_Boot, self).tearDown()
        self.cfg = None
        self.hal=None

    def testCheck_Halctl_Boot(self):
        """
        This test case is to check : adb shell haclctl -l

        Test Case Precondition:
        None

        Test Case Step:
1    Reboot the device
2    Connect to the device using adb for a root session
3    Issue dmesg command and check its output


        Expect Result:
1    Device boots successfully.
2    Root session available
3    No HAL errors are identified

        """

        print "[RunTest]: %s" % self.__str__()
        g_common_obj2.system_reboot()
        res = g_common_obj2.root_on_device()
        print 'root is :',res
        result=self.hal.search_in_dmesg('hal').count('error')
        print result
        assert result==0, "[ERROR]: halctl boot is fail"
        print "[INFO]: halctl boot is success"

