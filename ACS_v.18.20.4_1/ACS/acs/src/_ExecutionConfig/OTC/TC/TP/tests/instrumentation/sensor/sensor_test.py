import sys
import os

from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.sensor_impl  import HardwareImpl
from testlib.util.repo import Artifactory
from testlib.util.common import g_common_obj

class TestSensors(InstrumentationTestBase):
    """
    Sensor Test Class
    """
    @classmethod
    def setUpClass(cls):
        """
        install apk
        """
        super(TestSensors, cls).setUpClass()
        cfg_file ='tests.tablet.artifactory.conf'
        cls.arti=Artifactory(cls.config.getConfValue(cfg_file,'artifactory','location'))
        APK_file=cls.arti.get('abat/CtsHardwareTestCases.apk')
        g_common_obj.adb_cmd_common('install ' + APK_file)
        cls.sensor = HardwareImpl()
        cls.sensor.intialize()


    @classmethod
    def tearDownClass(cls):
        """
        uninstall apk
        """
        super(TestSensors, cls).tearDownClass()
        cls.sensor.finalize()
        #cls.sensor = None

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(TestSensors, self).setUp()


    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(TestSensors, self).tearDown()

    def testSensorValues(self):
        """testValuesForAllSensors"""
        self.sensor.testValuesForAllSensors()

    def testSensorOperations(self):
        """testSensorOperations"""
        self.sensor.testSensorOperations()

