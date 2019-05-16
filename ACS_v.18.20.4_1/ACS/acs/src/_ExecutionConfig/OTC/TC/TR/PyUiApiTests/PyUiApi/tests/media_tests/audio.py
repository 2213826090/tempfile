from PyUiApi.app_utils.audio.audio_utils import *
from PyUiApi.app_utils.audio.audio_player import *
from PyUiApi.common.system_utils import *


class AudioPlaybackQualityTests(unittest.TestCase):
    audio_player = AudioPlayer()
    host_audio_recorder = HostAudioRecorder()
    audio_utils = AudioAnalysisUtils()
    delete_recordings = True

    def setUp(self):
        LOG.info("setup")

    def tearDown(self):
        if AudioPlaybackQualityTests.delete_recordings:
            AudioPlaybackQualityTests.host_audio_recorder.delete_all_recordings()
        LOG.info("tear down")

    def audio_playback_quality(self):
        pass
