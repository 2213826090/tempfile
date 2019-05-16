"""
@summary: Run Antutu Benchmark tests
@since: 10/10/2014
@author: Dongyuanx.Chen (dongyuanx.chen@intel.com)
"""

import os
from testlib.util.log import Logger
from testlib.common.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.pnp.pnp_impl import PnpImpl
from testlib.pnp.antutu_impl import AntutuImpl
from testlib.wifi.wifisetting_impl import WifiSettingImpl
#from testlib.apk.apk_install_uninstall_impl import ApkInstallUninstallImpl
from testlib.util.repo import Artifactory
#from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.util.config import TestConfig

class AnTuTuTest(UIATestBase):
    """
    @summary: The basic function to test antutu
    """

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(AnTuTuTest, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        cfg = config.read(cfg_file, 'content_antutu')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        g_common_obj.adb_cmd_common('install ' + file_path)

    @classmethod
    def tearDownClass(self):
        """
        uninstall apk
        """
        super(AnTuTuTest, self).tearDownClass()

    def setUp(self):
        super(AnTuTuTest, self).setUp()
        self.logger = Logger.getlogger(__name__)
        self._test_name = __name__
        self.logger.info("[Setup]: %s" % self._test_name)
        cfg_file = 'tests.tablet.pnp.conf'

        #init function object
        self.wifi = WifiSettingImpl(self.config.read(cfg_file, 'wifisetting'))
        self.pnp = PnpImpl()
        self.antutu = AntutuImpl(self.config.read(cfg_file, 'antutu'))
        #self.apk = ApkInstallUninstallImpl(self.config.read(cfg_file, 'antutu'))

        #self.apk_name = self.apk.cfg.get("antutu_apk_name")
        #init local var
        self._init_env()

    def tearDown(self):
        self.logger.info("[Teardown]: %s" % self._test_name)
        super(AnTuTuTest, self).tearDown()
        #uninstall antutu
        #self.apk.apk_uninstall(self.apk_name)
        self.pnp.freeze_rotation()

    def testAnTuTu(self):
        """
        This test used to test AnTuTu
        """
        self.logger.info("[RunTest]: %s" % self.__str__())
        self.antutu.test()

    def _init_env(self):
        """
        init test env
        """
        #apk_filename = self.apk.cfg.get("antutu_apk_file")
        #apk_website = self.apk.cfg.get("antutu_apk_website")
        #check device status
        self.pnp.check_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        #turn off wifi
        self.wifi.launch_from_am()
        self.wifi.turn_off_wifi()
        #uninstall old app
        #if self.apk.apk_check(self.apk_name):
        #    self.apk.apk_uninstall(self.apk_name)
        self.pnp.rotate_device_n()
        #self.apk.apk_install(apk_filename, self.apk_name, apk_website)
