'''
@summary: test calculator function after gota
@since: 9/1/2015
@author: Su, Chaonanx <chaonanx.su@intel.com>
'''


# -*- coding:utf-8 -*-
import os
import sys
import time
from tests.IRDA_OEM_Customization.init.func import OEMFunc
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.oem.oem_impl import OEMImpl
from testlib.util.device import TestDevice
from testlib.common.common import g_common_obj2
from testlib.util.repo import Artifactory



class TestOEMDefaltLocale_M(UIATestBase):
    """
    Testing wifi enable
    """
    def setUp(self):
        super(TestOEMDefaltLocale_M, self).setUp()
        self.func = OEMFunc(g_common_obj2.getSerialNumber())
        cfg_file = os.path.join(os.environ.get(
            'TEST_DATA_ROOT', ''), 'tests.tablet.oem.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.oem = OEMImpl(self.config.read(cfg_file, 'default'))
        self.oem.setup_connection()
        self.oem.set_orientation_n()
        self.oem.wake_up()
        u = self.config.read('/etc/oat/sys.conf', 'artifactory').get('location')
        print u
        f = u + 'IRDA_OEM_Customization/oem_local_m/'
        print f
        self.filePath = Artifactory(f).get('oem.prop')
        self.func.mount_device()
        g_common_obj.adb_cmd("mv /oem/oem.prop  /oem/oem.prop.bak")
        g_common_obj.adb_cmd("ls /oem")
        g_common_obj.push_file(self.filePath, "/oem/")
        g_common_obj.adb_cmd("chmod 644 /oem/oem.prop")

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(TestOEMDefaltLocale_M, self).tearDown()
        self.wifi = None
        g_common_obj.adb_cmd("mv /oem/oem.prop.bak  /oem/oem.prop")
        self.func.factory_reset_local()
        self.func.check_reset()
        self.func.wait_for_android_os()
        g_common_obj.root_on_device()
        self.func.setup_connection()
        self.func.skip_initial_screen_after_factory_reset()

    def TestOEMDefalt_Locale_M(self):
        self.func.setup_connection()
        self.func.unlock_screen()
        self.func.factory_reset()
        self.func.check_reset()
        self.func.wait_for_android_os()
        g_common_obj.root_on_device()
        self.func.mount_device()
        self.func.setup_connection()
        self.oem.local_chinese()
        g_common_obj.adb_cmd_capture_msg("rm -rf /oem/oem.prop")
