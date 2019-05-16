import os
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.oem.oem_impl import OEMImpl


class TestAppAfterFactoryReset(UIATestBase):
    """
    Testing wifi enable
    """
    def setUp(self):
        super(TestAppAfterFactoryReset, self).setUp()
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
        super(TestAppAfterFactoryReset, self).tearDown()
        self.wifi = None

    def testApp_after_factory_reset(self):
        """
        Check OEM applications after factory recovery
        """
        print "[RunTest] %s" % self.__str__()

        self.oem.check_oem_apps()
        self.oem.reset_device()
        self.oem.check_oem_apps()
