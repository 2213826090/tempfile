"""
@summary: check haclctl 
@casename:/System_IRDA_Feature/Auto_detect Fallback entries check
@since: 11/12/2014
@author: Yuhui Xu(yuhuix.xu@intel.com)
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase


class Check_Halctl_fallback(UIATestBase):
    """
    check haclctl fallback
    """

    def setUp(self):
        super(Check_Halctl_fallback, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg=self.config.read(cfg_file, 'system_domain')
        self.hal = AutodetectImpl()
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_Halctl_fallback, self).tearDown()
        self.cfg = None
        self.hal=None
    def testCheck_Halctl_fallback(self):
        """
        This test case is to   Check_Halctl_fallback

        Test Case Precondition:

        Test Case Step:
1    Boot device
2    Check for devices with only fallback entries in the HAL bindings array
3    Type halctl -l and grep tyep=fallback. Inspect the output.


        Expect Result:
1    Device is booted successfully
2    -
3    Fallback entry is used for the devices identified in step 2


        """

        print "[RunTest]: %s" % self.__str__()
        res = g_common_obj2.root_on_device()
        print res
        result=self.hal.halctl_cmd("-i","fallback").count("Binding")
        assert result>0, "[ERROR]: halctl -i fallback is fail"
        print "[INFO]: halctl -i fallback  is success"
