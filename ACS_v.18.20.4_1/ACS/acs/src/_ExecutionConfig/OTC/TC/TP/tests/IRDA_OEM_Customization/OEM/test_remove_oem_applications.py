"""
@summary: Check if can remove oem apks.
@since: 12/2/2014
@author: Xiaoye Xu(xiaoyex.xu@intel.com)
"""
import os
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.oem.oem_impl import OEMImpl

class REMOVE_APPS(UIATestBase):

    def setUp(self):
        super(REMOVE_APPS, self).setUp()
        cfg_file = os.path.join(os.environ.get(
            'TEST_DATA_ROOT', ''), 'tests.tablet.oem.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.oem = OEMImpl(self.config.read(cfg_file, 'default'))
        self.oem.setup_connection()
        self.oem.set_orientation_n()
        self.oem.wake_up()

    def tearDown(self):
        super(REMOVE_APPS, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testremove_apps(self):
        """
        Check if can remove OEM apks

        The test case spec is following:
        1. Connect the dvt to linux PC follow the precondition.
        2. copy apk to the IRDA device.
        3. On the IRDA device, perform a factory reset.

        The real implementation will be in OEMImpl class.
        """
        print "[RunTest]: %s" % self.__str__()

        self.oem.remove_apps()
