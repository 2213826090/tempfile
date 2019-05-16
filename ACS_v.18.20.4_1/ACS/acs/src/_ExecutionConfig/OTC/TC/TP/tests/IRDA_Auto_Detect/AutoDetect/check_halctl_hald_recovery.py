"""
@summary: check haclctl -l
@casename:/System_IRDA_Feature/Auto_detect hald - recovery process
@since: 11/04/2014
@author: Yuhui Xu(yuhuix.xu@intel.com)
"""
import os
import time
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase


class Check_Halctl_Hald_Recovery(UIATestBase):
    """
    xxxx
    """

    def setUp(self):
        super(Check_Halctl_Hald_Recovery, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.hal = AutodetectImpl(self.config.read(cfg_file, 'system_domain'))
#         self.cfg = self.config.read(cfg_file, 'system_domain')

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_Halctl_Hald_Recovery, self).tearDown()
        self.cfg = None

    def testCheck_Halctl_Hald_Recovery(self):
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
        res = g_common_obj2.root_on_device()
        print res
          
        result=self.hal.halctl_cmd("-l",para1=None).count("modalias")
        assert result>10, "[ERROR]: halctl -list is fail"
        print "[INFO]: halctl -list is success"

        pid_name='hald'
        pid=self.hal.get_pid(pid_name)
        print pid
        assert self.hal.kill_pid(pid)==None,"[ERROR]: kill %s failed"%pid_name
        print "[INFO]: kill %s success"%pid_name
        assert self.hal.get_pid(pid_name),"[ERROR]:  get pid of %s is fail"%pid_name
        print "[INFO]: get pid of %s is success"%pid_name
        time.sleep(5)
        
        result=self.hal.halctl_cmd("-l",para1=None).count("modalias")
        print result
        assert result>10, "[ERROR]: halctl -list is fail"
        print "[INFO]: halctl -list is success"
