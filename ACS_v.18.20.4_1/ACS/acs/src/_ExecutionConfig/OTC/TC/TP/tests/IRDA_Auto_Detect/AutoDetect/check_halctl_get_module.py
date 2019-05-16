"""
@summary: check haclctl --get-module
@casename:/System_IRDA_Feature/Auto_detect halctl --get-module
@since: 11/04/2014
@author: Yuhui Xu(yuhuix.xu@intel.com)
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import autodetect
from testlib.util.uiatestbase import UIATestBase


class CheckHalctl_get_module(UIATestBase):
    """
    check haclctl -l
    """

    def setUp(self):
        super(CheckHalctl_get_module, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'system_domain')

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckHalctl_get_module, self).tearDown()
        self.cfg = None

    def testCheckHalctl_get_module(self):
        """
        This test case is to check : adb shell haclctl halctl --get-module camera

        Test Case Precondition:
        None

        Test Case Step:
        1. Run "halctl --get-module"

        Expect Result:
        1.  HW module ID:     camera
            HW module name:   Intel Camera3HAL Module
            HW module author: Intel


        """

        print "[RunTest]: %s" % self.__str__()
        get_module_name=self.cfg.get("get_module_name")
        camera_g_name=self.cfg.get("camera_g_name")
        res = g_common_obj2.root_on_device()
        print res
        result=autodetect.halctl_cmd('--get-module',get_module_name)
        print result
        res=result.count(camera_g_name)
        assert res>0,"[ERROR]: adb shell halctl --get-module %s :is fail"%get_module_name
        print "[INFO]: adb shell halctl --get-module %s is pass"%get_module_name