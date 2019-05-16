"""
@summary: check libhalbindings
@casename:/System_IRDA_Feature/Auto_detect libhalbindings - verbose mode
@since: 11/12/2014
@author: Yuhui Xu(yuhuix.xu@intel.com)
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase


class Check_Halctl_libhal_log_set_level(UIATestBase):
    """
#     check haclctl -l
    """

    def setUp(self):
        super(Check_Halctl_libhal_log_set_level, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.hal = AutodetectImpl(self.config.read(cfg_file, 'system_domain'))
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_Halctl_libhal_log_set_level, self).tearDown()
        self.cfg = None
        self.hal=None

    def testCheck_Halctl_libhal_log_set_level(self):
        """
        This test case is to check : adb shell haclctl -l

        Test Case Precondition:
        None

        Test Case Step:
        1. reboot devices
        2. Root
        3. check dmesg without hal errors

        Expect Result:
        1. dmesg without hal errors

        """

        print "[RunTest]: %s" % self.__str__()
        res = g_common_obj2.root_on_device()
        result=self.hal.halctl_cmd('-e', '7')
        print 'result is ',result
        assert result>0, "[ERROR]: halctl -e 7 is fail"
        print "[INFO]: halctl -e 7 is success"
        result1=self.hal.search_in_dmesg('hal').count('log level set to 7')
        print result1
        assert result1>0, "[ERROR]: log level set to 7 is fail"
        print "[INFO]: log level set to 7 is success"

