import sys
import os

from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.os_impl import OsImpl
from testlib.util.repo import Artifactory
from testlib.util.common import g_common_obj

class TestOS(InstrumentationTestBase):
    """
    Os Test Class
    """
    @classmethod
    def setUpClass(cls):
        """
        install apk
        """

        super(TestOS, cls).setUpClass()
        cfg_file ='tests.tablet.artifactory.conf'
        cls.arti=Artifactory(cls.config.getConfValue(cfg_file,'artifactory','location'))
        APK_file=cls.arti.get('abat/CtsOsTestCases.apk')
        g_common_obj.adb_cmd_common('install ' + APK_file)
        cls.os = OsImpl()
        cls.os.intialize()


    @classmethod
    def tearDownClass(cls):
        """
        uninstall apk
        """
        super(TestOS, cls).tearDownClass()
        cls.os.finalize()
        #cls.os = None

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(TestOS, self).setUp()


    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(TestOS, self).tearDown()

    def testOsPowerManager_WakeLock(self):
        """testOsPowerManagerWakeLock"""
        self.os.testOsPowerManagerWakeLock()

    def testOsPowerManger(self):
        """testOsPowerManager"""
        self.os.testOsPowerManager()

