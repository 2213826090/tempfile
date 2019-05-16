
'''
@summary: test calculator function after gota
@since: 12/5/2015
@author: Song, GuimeiX Z < guimeix.z.song@intel.com>
'''
import os
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.oem.oem_impl import OEMImpl


class TestOEM_device_model(UIATestBase):
    """
    Testing wifi enable
    """
    def setUp(self):
        super(TestOEM_device_model, self).setUp()
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
        super(TestOEM_device_model, self).tearDown()
        self.wifi = None

    def testOEM_device_model(self):
        """
        DEfault OEM device model
        """
        print "[RunTest] %s" % self.__str__()
        self.oem.check_oem_device_model()