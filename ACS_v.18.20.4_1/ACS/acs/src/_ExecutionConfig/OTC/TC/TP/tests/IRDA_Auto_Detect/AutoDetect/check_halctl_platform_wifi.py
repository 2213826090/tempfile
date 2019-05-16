"""
@summary: check
@casename:/System_IRDA_Feature/Platform - Graphics
@since: 7/8/2015
@author: Song, GuimeiX Z <guimeix.z.song@intel.com>
"""
import os
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase


class Check_Platform_WIFI(UIATestBase):
    """
    check haclctl -l
    """

    def setUp(self):
        super(Check_Platform_WIFI, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.cfg = self.config.read(cfg_file, 'system_domain')
        self.hal = AutodetectImpl()
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Check_Platform_WIFI, self).tearDown()
        self.cfg = None
        self.hal =None
    def testCheck_Platform_WIFI(self):
        """
        This test case is to check : adb shell haclctl -l

        Test Case Precondition:
        None

        Test Case Step:
1    adb root -->adb shell
2    Type halctl -i wifi



        Expect Result:
1    Connect device and root device successful
2    Bindings list is displayed

        """

        print "[RunTest]: %s" % self.__str__()
        halctl_keyword='wifi'
        res = g_common_obj2.root_on_device()
        print res
        cmdstr="adb shell halctl -i %s"%halctl_keyword
        print cmdstr
        halctl_cmd=os.popen(cmdstr).read()
        print halctl_cmd
        search_result=halctl_cmd.count('Binding')
        assert search_result>0,"[ERROR]: halctl -i %s  is fail"%halctl_keyword
        print "[INFO]: halctl -i %s is success"%halctl_keyword

