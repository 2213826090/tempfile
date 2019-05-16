from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.location_impl import LocationImpl
from testlib.system_domains.system_impl import system_domains
from testlib.util.repo import Artifactory
from testlib.util.common import g_common_obj


class TestLocation(InstrumentationTestBase):
    """
    Bluetooth Test Class
    """
    @classmethod
    def setUpClass(cls):
        """
        install apk
        """

        super(TestLocation, cls).setUpClass()
        cfg_file = 'tests.tablet.artifactory.conf'
        cls.arti = Artifactory(cls.config.getConfValue(
            cfg_file, 'artifactory', 'location'))
        APK_file = cls.arti.get('abat/CtsLocationTestCases.apk')
        g_common_obj.adb_cmd_common('install ' + APK_file)

        # for CtsLocationTest, must enable Mock Location first
        system_domains.developr_option_check("Allow mock locations", True)

        cls.location = LocationImpl()
        cls.location.intialize()

    @classmethod
    def tearDownClass(cls):
        """
        uninstall apk
        """
        super(TestLocation, cls).tearDownClass()
        system_domains.developr_option_check("Allow mock locations", False)
        cls.location.finalize()

    def testGpsLocation(self):
        self.location.instr_run_cts_class('GpsStatusTest')
        self.location.instr_run_cts_class('GpsSatelliteTest')
        self.location.instr_run_cts_class(
            'LocationManagerTest#testGetGpsStatus')
        self.location.instr_run_cts_class(
            'LocationManagerTest#testEnterProximity')
