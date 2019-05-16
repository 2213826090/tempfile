import os
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.oem.oem_impl import OEMImpl
from testlib.common.common import g_common_obj2
from tests.IRDA_OEM_Customization.init.dut_init import oem_init
class InitOEMCustomize(UIATestBase):
    """
    Testing wifi enable
    """
    def setUp(self):
        super(InitOEMCustomize, self).setUp()
        cfg_file = os.path.join(os.environ.get(
            'TEST_DATA_ROOT', ''), 'tests.tablet.oem.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.oem = OEMImpl(self.config.read(cfg_file, 'default'))
        self.oem.setup_connection()
        self.oem.wake_up()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(InitOEMCustomize, self).tearDown()
        self.wifi = None

    def testInitOEMCustomize(self):
        """
        Reboot DUT after Change OEM wallpaper to another
        """
        print "[RunTest] %s" % self.__str__()
        sn = g_common_obj2.getSerialNumber()
        oem=oem_init(sn)
        oem.run()
