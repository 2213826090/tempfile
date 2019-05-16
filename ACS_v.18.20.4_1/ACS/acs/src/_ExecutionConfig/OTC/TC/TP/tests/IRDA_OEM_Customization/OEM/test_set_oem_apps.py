import os
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.oem.oem_impl import OEMImpl


class TestSetOEMApp(UIATestBase):
    """
    Testing wifi enable
    """
    def setUp(self):
        super(TestSetOEMApp, self).setUp()
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
        super(TestSetOEMApp, self).tearDown()
        self.wifi = None

    def testSet_oem_apps(self):
        """
        Set multi-apps by OEM
        """
        print "[RunTest] %s" % self.__str__()

        self.oem.write_oem()
        self.oem.set_oem_app()
        self.oem.unwrite_oem()
        self.oem.reset_device()

        self.oem.check_set_apps()
