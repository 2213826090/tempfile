"""
@summary: Check if apks are visible in app menu after factory reset
@since: 12/1/2014
@author: Xiaoye Xu(xiaoyex.xu@intel.com)
"""
import os
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.oem.oem_impl import OEMImpl

class EXTRA_APPS(UIATestBase):

    def setUp(self):
        super(EXTRA_APPS, self).setUp()
        cfg_file = os.path.join(os.environ.get(
            'TEST_DATA_ROOT', ''), 'tests.tablet.oem.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.oem = OEMImpl(self.config.read(cfg_file, 'default'))
        self.oem.setup_connection()
        self.oem.set_orientation_n()
        self.oem.wake_up()

    def tearDown(self):
        super(EXTRA_APPS, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testEXTRA_APPS(self):
        """
        Check apks are visible in app menu after factory reset

        The test case spec is following:
        1. Connect the dvt to linux PC follow the precondition.
        2. copy apk to the IRDA device.
        3. On the IRDA device, perform a factory reset.

        The real implementation will be in OemImpl class.
        """
        print "[RunTest]: %s" % self.__str__()


        self.oem.check_apps()
