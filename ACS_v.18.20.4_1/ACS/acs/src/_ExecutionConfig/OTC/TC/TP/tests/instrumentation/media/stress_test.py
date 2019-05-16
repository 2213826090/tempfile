import sys
import os

from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.mediastress_impl  import MediaPlayerStressImpl


class TestMediaPlayer(InstrumentationTestBase):
    """
    Media Test Class
    """
    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(TestMediaPlayer, self).setUp()
        self.media = MediaPlayerStressImpl()
        self.media.intialize()

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(TestMediaPlayer, self).tearDown()
        self.media.finalize()
        self.media = None

    def testStressPlay(self):
        """test long-time play and short-time play"""
        self.media.do_H263QcifShortPlay()
        self.media.do_H263QcifLongPlay()


