from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.bat_impl import BatImpl
from testlib.util.repo import Artifactory
from testlib.util.common import g_common_obj


class BatTestBase(InstrumentationTestBase):
    @classmethod
    def setUpClass(cls):
        """
        install apk
        """
        super(BatTestBase, cls).setUpClass()
        cfg_file = 'tests.tablet.artifactory.conf'
        cls.arti = Artifactory(cls.config.getConfValue(
            cfg_file, 'artifactory', 'location'))
        apk_file = cls.arti.get('abat/com.intel.tests.bat.apk')
        g_common_obj.adb_cmd_common('install ' + apk_file)
        cls.bat = BatImpl()
        cls.bat.intialize()

    @classmethod
    def tearDownClass(cls):
        """
        uninstall apk
        """
        cls.bat.finalize()
        super(BatTestBase, cls).tearDownClass()
