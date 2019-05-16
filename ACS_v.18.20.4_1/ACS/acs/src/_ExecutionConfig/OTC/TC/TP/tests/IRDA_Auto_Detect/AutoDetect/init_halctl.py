"""
@summary: check haclctl --add
@casename:/System_IRDA_Feature/Auto_detect halctl --add
@since: 11/04/2014
@author: Sam Lan(samx.lan@intel.com)
"""
import os
import time
from testlib.common.common import g_common_obj2
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.util.uiatestbase import UIATestBase

class InitalHtlctl(UIATestBase):
    """
    check haclctl -l
    """

    def setUp(self):
        super(InitalHtlctl, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.system_domains.conf')
        self._test_name = __name__
        self.hal = AutodetectImpl(self.config.read(cfg_file, 'system_domain'))
        print "[Setup]: %s" % self._test_name
#         self.cfg = self.config.read(cfg_file, 'system_domain')

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(InitalHtlctl, self).tearDown()
        self.cfg = None
        self.hal=None

    def testInitalHtlctl(self):
        """
        This test case is to check : adb shell haclctl -add -del

        Test Case Precondition:
        NoneSystemImpl
        """
        print "[RunTest]: %s" % self.__str__()
