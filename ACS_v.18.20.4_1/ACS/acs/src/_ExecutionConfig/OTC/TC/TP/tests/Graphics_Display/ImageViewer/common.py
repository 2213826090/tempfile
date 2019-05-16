from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.graphics_impl import GraphicsImpl
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig


class GraphicsTestBase(InstrumentationTestBase):
    @classmethod
    def setUpClass(cls):
        """
        install apk
        """
        super(GraphicsTestBase, cls).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_image')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg.get("name")
        file_path = arti.get(binary_name)
        result = config_handle.check_apps("com.android.graphicstest.image")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)
        cls.graphics = GraphicsImpl()
        cls.graphics.intialize()

    @classmethod
    def tearDownClass(cls):
        """
        uninstall apk
        """
        cls.graphics.finalize()
        super(GraphicsTestBase, cls).tearDownClass()
