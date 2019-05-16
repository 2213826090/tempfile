from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.bat_impl import BatImpl
from testlib.util.repo import Artifactory
from testlib.util.config import TestConfig
from testlib.graphics.tools import ConfigHandle
import os


class TestGraphic(InstrumentationTestBase):
    '''
    Graphic Test Class
    '''
    @classmethod
    def setUpClass(cls):
        """
        install apk
        """
        super(InstrumentationTestBase, cls).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg_apk = config.read(cfg_file, 'content_bat')
        cls.arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_apk.get("name")
        file_path = cls.arti.get(binary_name)
        result = config_handle.check_apps("com.intel.tests.bat")
        if result == 0:
            os.system('adb install -r -g ' + file_path) # Fix apk not installed issue.
        cls.bat = BatImpl()
        cls.bat.intialize()

    @classmethod
    def tearDownClass(cls):
        """
        uninstall apk
        """
        cls.bat.finalize()
        super(InstrumentationTestBase, cls).tearDownClass()

    def testActivityBrightnessChange(self):
        self.bat.instr_run_class('GraphicTest#testActivityBrightnessChange')

    def testSystemBrightnessChange(self):
        self.bat.instr_run_class('GraphicTest#testSystemBrightnessChange')
