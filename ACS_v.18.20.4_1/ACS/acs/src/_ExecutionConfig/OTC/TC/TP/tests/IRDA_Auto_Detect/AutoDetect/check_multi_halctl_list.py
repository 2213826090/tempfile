"""
@summary: check haclctl -l at other user
@casename:/System_IRDA_Feature/
@since: 11/12/2014
@author: Yuhui Xu(yuhuix.xu@intel.com)
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase


class Check_Multi_Halctl_list(UIATestBase):
    """
    check haclctl -l
    """

    def setUp(self):
        super(Check_Multi_Halctl_list, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.hal = AutodetectImpl(self.config.read(cfg_file, 'system_domain'))
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_Multi_Halctl_list, self).tearDown()
        self.cfg = None
        self.hal=None
    def testCheck_Multi_Halctl_list(self):
        """
        This test case is to check : adb shell haclctl -l

        Test Case Precondition:
        None

        Test Case Step:
        1. Run "adb shell haclctl -l"

        Expect Result:
        1. It shows more than 100 modalias

        """

        print "[RunTest]: %s" % self.__str__()
        self.hal.wake_up()
        self.hal.add_one_multi_user()
        res = g_common_obj2.root_on_device()
        print res
        result=self.hal.halctl_cmd("-l",para1=None).count("modalias")
        assert result>10, "[ERROR]: halctl -list is fail"
        print "[INFO]: halctl -list is success"
        self.hal.switch_to_owner()
        
            

