import sys
import os

from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.graphic_impl import GraphicImpl
from testlib.util.repo import Artifactory
from testlib.util.common import g_common_obj

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
        cfg_file ='tests.tablet.artifactory.conf'
        cls.arti=Artifactory(cls.config.getConfValue(cfg_file,'artifactory','location'))
        APK_file=cls.arti.get('abat/CtsGraphics2TestCases.apk')
        g_common_obj.adb_cmd_common('install ' + APK_file)
        cls.graphic = GraphicImpl()
        cls.graphic.intialize()


    @classmethod
    def tearDownClass(cls):
        """
        uninstall apk
        """
        super(TestGraphics, cls).tearDownClass()
        cls.graphic.finalize()
        #cls.graphic = None

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(TestGraphics, self).setUp()


    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(TestGraphics, self).tearDown()

    def testGraphicTextureView(self):
        """testTextureView"""
        self.graphic.testTextureView()
