"""
@summary: check haclctl -l
@casename:/System_IRDA_Feature/Auto_detect hald - not running as root
@since: 12/22/2014
@author: Yuhui Xu(yuhuix.xu@intel.com)
"""
import os
import time
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase


class Check_halctl_hald_not_root(UIATestBase):
    """
    xxxx
    """

    def setUp(self):
        super(Check_halctl_hald_not_root, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.hal = AutodetectImpl(self.config.read(cfg_file, 'system_domain'))
#         self.cfg = self.config.read(cfg_file, 'system_domain')

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_halctl_hald_not_root, self).tearDown()
        self.cfg = None

    def testCheck_halctl_hald_not_root(self):
        """
        This test case is to check : adb shell haclctl -l

        Test Case Precondition:
        None

        Test Case Step:
        1	Connect to device using adb. Root session
		2	Type ps and grep for hald


        Expect Result:
        1	adb root session is established
		2	Process owner is hal, not root.


        """

        print "[RunTest]: %s" % self.__str__()
        res = g_common_obj2.root_on_device()
        print res

        pid_name='hald'
        owner='hal'
        process_owner=self.hal.get_process_owner(pid_name)
        print 'process owner is %s'%process_owner
        assert process_owner==owner,"[ERROR]: Process owner is not %s"%owner
        print "[INFO]: Process owner is %s"%owner
