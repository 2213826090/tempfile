import sys
import os

from testlib.util.log import Logger
from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.graphic_impl import GraphicImpl
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.graphics.tools import ConfigHandle
from testlib.util.config import TestConfig

class TestGraphics(InstrumentationTestBase):
    """
    Graphic Test Class
    """
    @classmethod
    def setUpClass(cls):
        """
        install apk
        """

        super(TestGraphics, cls).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        cfg_apk = config.read(cfg_file, 'texture_view')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg_apk.get("name")
        file_path = arti.get(binary_name)
        result = config_handle.check_apps("com.android.cts.graphics2")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)
        cls.graphic = GraphicImpl()
        cls.graphic.intialize()

    @classmethod
    def tearDownClass(cls):
        """
        uninstall apk
        """
        super(TestGraphics, cls).tearDownClass()
        cls.graphic.finalize()
        # cls.graphic = None

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(TestGraphics, self).setUp()
        self.logger = Logger.getlogger(__name__)
        self._test_name = __name__
        self.logger.info("[Setup]: %s" % self._test_name)

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(TestGraphics, self).tearDown()

    def testGraphicTextureView(self):
        """testTextureView"""
        self.logger.info("[RunTest]: %s" % self.__str__())
        self.graphic.testTextureView()
