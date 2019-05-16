import sys
import os

from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.mediastress_impl  import MediaPlayerStressExImpl


class TestMediaPlayerEx(InstrumentationTestBase):
    """
    Media Test Class
    """
    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(TestMediaPlayerEx, self).setUp()
        self.media = MediaPlayerStressExImpl()
        self.media.intialize()

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(TestMediaPlayerEx, self).tearDown()
        self.media.finalize()
        self.media = None

    def testStressPlayEx(self):
        """test enhanced short-time play"""
        self.media.do_scanningShortPlay()


