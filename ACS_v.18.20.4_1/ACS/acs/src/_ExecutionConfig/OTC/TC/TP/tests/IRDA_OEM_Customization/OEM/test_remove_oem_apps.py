import os
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.oem.oem_impl import OEMImpl


class TestRemoveOEMApps(UIATestBase):
    """
    Testing wifi enable
    """
    def setUp(self):
        super(TestRemoveOEMApps, self).setUp()
        cfg_file = os.path.join(os.environ.get(
            'TEST_DATA_ROOT', ''), 'tests.tablet.oem.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.oem = OEMImpl(self.config.read(cfg_file, 'default'))
        self.oem.setup_connection()
        self.oem.set_orientation_n()
        self.oem.wake_up()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.oem.switch_to_owner()
        super(TestRemoveOEMApps, self).tearDown()
        self.wifi = None

    def testRemove_oem_apps(self):
        """
        Remove OEM applications in multi-user mode
        """
        print "[RunTest] %s" % self.__str__()


        self.oem.switch_to_user()
        self.oem.delete_oem_apps()
        self.oem.switch_to_owner()
        self.oem.delete_oem_apps()