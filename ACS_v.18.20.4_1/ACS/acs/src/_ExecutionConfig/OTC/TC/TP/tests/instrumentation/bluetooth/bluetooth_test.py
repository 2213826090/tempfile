import sys
import os

from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.bluetooth_impl import BluetoothImpl
from testlib.util.repo import Artifactory
from testlib.util.common import g_common_obj

class TestBluetooth(InstrumentationTestBase):
    """
    Bluetooth Test Class
    """
    @classmethod
    def setUpClass(cls):
        """
        install apk
        """

        super(TestBluetooth, cls).setUpClass()
        cfg_file ='tests.tablet.artifactory.conf'
        cls.arti=Artifactory(cls.config.getConfValue(cfg_file,'artifactory','location'))
        APK_file=cls.arti.get('abat/CtsBluetoothTestCases.apk')
        g_common_obj.adb_cmd_common('install ' + APK_file)
        cls.bluetooth = BluetoothImpl()
        cls.bluetooth.intialize()


    @classmethod
    def tearDownClass(cls):
        """
        uninstall apk
        """
        super(TestBluetooth, cls).tearDownClass()
        cls.bluetooth.finalize()
        #cls.bluetooth = None

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(TestBluetooth, self).setUp()


    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(TestBluetooth, self).tearDown()

    def testBluetoothAdapter(self):
        """Bluetooth testBasicAdapter"""
        self.bluetooth.testBasicAdapter()
