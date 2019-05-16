# coding: utf-8
import time
from testlib.audio.recorder_impl import RecorderImpl
from testlib.audio import resource
from testlib.audio.audio_test_base import AudioStubTestBase


class AudioRecordIterative(AudioStubTestBase):
    """
    @summary: AudioPlayback used to test audio playback function
    """
    CONFIG_FILE = "tests.tablet.mum_auto_audio.conf"

    def setUp(self):
        """
        @summary: setup
        @return: None
        """
        super(AudioRecordIterative, self).setUp()
        self.recorder = RecorderImpl()

    def tearDown(self):
        super(AudioRecordIterative, self).tearDown()

    def appPrepare(self, case_name):
        self.cfg = self.config.read(self.CONFIG_FILE, case_name)
        self.logger.debug("[Download]: download apk...")
        apk = resource.get_app(self.cfg.get("app_name"))
        self.recorder.install(apk)

    def do_record_iterative(self, case_name):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch record
        2. Play record
        """
        self.logger.debug( "run case is " + str(case_name))
        self.appPrepare(case_name)
        self.recorder.launch()
        # if any configure exists
        str_cfg = self.cfg.get("configure", "")
        if str_cfg.strip():
            cfg = {}
            items = str_cfg.split(";")
            for item in items:
                tmp = item.split(":")
                key = tmp[0].strip()
                val = tmp[1].strip()
                cfg[key] = val
            self.recorder.configure(cfg)
        loops = int(self.cfg.get("test_loops", "0")) or 1
        for i in range(loops):
            record_time = int(self.cfg.get("record_time"))
            self.recorder.record(record_time)
            self.recorder.playback(record_time)
            time.sleep(2)
            self.recorder.delete()
        self.recorder.quit()

    def testAudioRecord_Iteration_EmbdMicCapture(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch record
        2. Play record
        """
        self.do_record_iterative('test_capture_audio_embedded_mic_iterative')

    def testAudioRecord_Iteration_AMR_NB(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch record
        2. Play record
        """
        self.do_record_iterative(
            'test_encode_voice_memo_record_amrnb_codec_iterative')

    def testAudioRecord_Longlasting_AMR_NB(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch record
        2. Play record
        """
        self.do_record_iterative(
            'test_encode_voice_memo_record_amrnb_codec_long_lasting')

    def testAudioRecord_Longlasting_AMIC(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch record
        2. Play record
        """
        self.do_record_iterative(
            'test_capture_audio_streo_wshs_amic_long_lasting')

    def testAudioRecord_Iteration_AMIC_wsHS(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch record
        2. Play record
        """
        self.do_record_iterative('test_capture_audio_streo_wshs_amic_iterative')

    def testAudioRecord_Longlasting_EmbdMicCapture(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch record
        2. Play record
        """
        self.do_record_iterative('test_capture_audio_embedded_mic_long_lasting')
