import sys
import os

from testlib.util.instrumentedtestbase import InstrumentationTestBase
from testlib.instrumentation.media_impl import AudioManagerImpl
from testlib.instrumentation.media_impl import MediaPlayerExImpl


class TestAudioManager(InstrumentationTestBase):
    """
    Media Test Class
    """
    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(TestAudioManager, self).setUp()
        self.audioMgr = AudioManagerImpl()
        self.audioMgr.intialize()

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(TestAudioManager, self).tearDown()
        self.audioMgr.finalize()
        self.audioMgr = None

    def testAllSettings(self):
        """test all settings of audioMgr"""
        self.audioMgr.do_AccessMode()
        self.audioMgr.do_AccessRingMode()
        self.audioMgr.do_AndroidTestCaseSetupProperly()
        self.audioMgr.do_MicrophoneMute()
        self.audioMgr.do_MusicActive()
        self.audioMgr.do_Routing()
        self.audioMgr.do_SetInvalidRingerMode()
        self.audioMgr.do_SoundEffects()
        self.audioMgr.do_VibrateNotification()
        self.audioMgr.do_VibrateRinger()
        self.audioMgr.do_Volume()


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
        self.player = MediaPlayerExImpl()
        self.player.intialize()

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(TestMediaPlayerEx, self).tearDown()
        self.player.finalize()
        self.player = None

    def testMidiPlayback(self):
        """ test playing MIDI playback in different file extension """
        self.player.do_PlayAudioMidiMid()
        self.player.do_PlayAudioMidiImy()
        self.player.do_PlayAudioMidiMxmf()
        self.player.do_PlayAudioMidiOta()
        self.player.do_PlayAudioMidiRtttl()
        self.player.do_PlayAudioMidiRtx()
        self.player.do_PlayAudioMidiXmf()
