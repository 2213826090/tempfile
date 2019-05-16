"""
@summary: check haclctl /-h/--help
@casename:/System_IRDA_Feature/Auto_detect halctl
@since: 11/04/2014
@author: Yuhui Xu(yuhuix.xu@intel.com)
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase


class CheckHalctl_Usage(UIATestBase):
    """
    check haclctl -l
    """

    def setUp(self):
        super(CheckHalctl_Usage, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.hal = AutodetectImpl(self.config.read(cfg_file, 'system_domain'))
#          self.cfg = self.config.read(cfg_file, 'system_domain')

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckHalctl_Usage, self).tearDown()
        self.cfg = None
        self.hal=None

    def testCheckHalctl_Usage(self):
        """
        This test case is to check : adb shell haclctl usage

        Test Case Precondition:
        None

        Test Case Step:
        1. Run "adb shell haclctl  /-h/--help"

        Expect Result:
        1. It shows usage of halctl --help

        """

        print "[RunTest]: %s" % self.__str__()
        res = g_common_obj2.root_on_device()
        print res
        
        cmdstr=['','-h','--help']
        for i in cmdstr:
            print "parameter is %s"%i
            result=self.hal.halctl_cmd(para=i,para1=None).count("Usage")
            assert result>0, "[ERROR]: adb shell halctl %s is fail"%i
            print "[INFO]: adb shell halctl %s is success"%i
        
        
            

