import sys
import os

from testlib.util.instrumentedtestbase  import InstrumentedCtsBaseImpl


class AudioManagerImpl(InstrumentedCtsBaseImpl):
    """
    Instrumented Test Suite
    """
    test_pkg = 'com.android.cts.media'
    pkg_names = ['com.android.cts.media']
    pkg_files = ['CtsMediaTestCases.apk']
    apk_repo_path = os.path.join(os.path.split(os.path.realpath(__file__))[0], '../../../support/prebuild')

    def do_AccessMode(self):
        self.instr_run('android.media.cts.AudioManagerTest#testAccessMode')

    def do_AccessRingMode(self):
        self.instr_run('android.media.cts.AudioManagerTest#testAccessRingMode')

    def do_AndroidTestCaseSetupProperly(self):
        self.instr_run('android.media.cts.AudioManagerTest#testAndroidTestCaseSetupProperly')

    def do_MicrophoneMute(self):
        self.instr_run('android.media.cts.AudioManagerTest#testMicrophoneMute')

    def do_MusicActive(self):
        self.instr_run('android.media.cts.AudioManagerTest#testMusicActive')

    def do_Routing(self):
        self.instr_run('android.media.cts.AudioManagerTest#testRouting')

    def do_SetInvalidRingerMode(self):
        self.instr_run('android.media.cts.AudioManagerTest#testSetInvalidRingerMode')

    def do_SoundEffects(self):
        self.instr_run('android.media.cts.AudioManagerTest#testSoundEffects')

    def do_VibrateNotification(self):
        self.instr_run('android.media.cts.AudioManagerTest#testVibrateNotification')

    def do_VibrateRinger(self):
        self.instr_run('android.media.cts.AudioManagerTest#testVibrateRinger')

    def do_Volume(self):
        self.instr_run('android.media.cts.AudioManagerTest#testVolume')


class MediaPlayerExImpl(InstrumentedCtsBaseImpl):
    """
    MediaPlayerEx Instrumented Test Suite
    """
    test_pkg = 'com.android.cts.mediaex'
    pkg_names = ['com.android.cts.mediaex']
    pkg_files = ['CtsMediaTestCasesEx.apk']
    apk_repo_path = os.path.join(os.path.split(os.path.realpath(__file__))[0],
                                 '../../../support/prebuild')
    cts_class = 'android.mediaex.cts.MediaPlayerTest'

    def do_PlayAudioMidiMid(self):
        self.instr_run(self.cts_class + '#testLocalAudio_midi_mid')

    def do_PlayAudioMidiImy(self):
        self.instr_run(self.cts_class + '#testLocalAudio_midi_imy')

    def do_PlayAudioMidiMxmf(self):
        self.instr_run(self.cts_class + '#testLocalAudio_midi_mxmf')

    def do_PlayAudioMidiOta(self):
        self.instr_run(self.cts_class + '#testLocalAudio_midi_ota')

    def do_PlayAudioMidiRtttl(self):
        self.instr_run(self.cts_class + '#testLocalAudio_midi_rtttl')

    def do_PlayAudioMidiRtx(self):
        self.instr_run(self.cts_class + '#testLocalAudio_midi_rtx')

    def do_PlayAudioMidiXmf(self):
        self.instr_run(self.cts_class + '#testLocalAudio_midi_xmf')
