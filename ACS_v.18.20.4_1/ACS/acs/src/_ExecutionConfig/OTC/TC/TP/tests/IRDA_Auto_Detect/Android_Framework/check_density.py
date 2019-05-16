"""
@summary: check if the density is right
@case_path:/System_Android Framework/Screen density reporting
@since: 10/27/2014
@author: Yuhui Xu(yuhuix.xu@intel.com)
"""
import os
from testlib.system_domains.system_impl import SystemImpl
from testlib.util.uiatestbase import UIATestBase


class CheckDensity(UIATestBase):
    """
    check density 
    """

    def setUp(self):
        super(CheckDensity, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_REPO_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
#         self.system = systemdomainsImpl(self.config.read(cfg_file, 'system_domain'))
        self.cfg = self.config.read(cfg_file, 'system_domain')
        self.system = SystemImpl(self.config.read(cfg_file, 'system_domain'))
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckDensity, self).tearDown()
        self.cfg = None
        self.system=None
    def testCheckDensity(self):
        """
        This test case is to check density

        Test Case Precondition:
        None

        Test Case Step:
        1. Run "adb shell getprop | grep density_info"

        Expect Result:
        [ro.sf.lcd_density_info]: [800 x 1280px 99mm x 177mm  205 dpi => density: 213]
        1. density: 213(for ECS PVT )

        The real implementation will be in android_frame_Impl class.
        """

        print "[RunTest]: %s" % self.__str__()
        density=self.cfg.get("density")
        result=self.system.getprop_grep("density").count(density)
        assert result>0, "[ERROR]: density is not "+density
        print "[INFO]: density is "+density
        
            

