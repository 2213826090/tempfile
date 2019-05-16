# coding: utf-8
from testlib.audio import resource
from testlib.audio.audio_test_base import AudioStubTestBase
import os
import time


def alias_name(name):
    def wrapper(func):
        setattr(func, "alias_name", name)
        return func
    return wrapper


#class AudioPlayback(UIATestBase):
class AudioPlayback(AudioStubTestBase):
    """
    @summary: AudioPlayback used to test audio playback function
    """
    CONFIG_FILE = "tests.tablet.mum_auto_audio.conf"

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(AudioPlayback, self).setUp()
        self._test_name = __name__
        self.logger.debug("[Setup]: %s" % self._test_name)
        self.audio.cleanUpData()
        self.audio.set_orientation_n()


    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(AudioPlayback, self).tearDown()
        self.logger.debug("[Teardown]: %s" % self._test_name)
        self.test_device.gmusic.cleanUpData()
        time.sleep(3)
        self.test_device.adb_cmd_capture_msg(self.audio.cfg.get("remove_audio"))
        time.sleep(10)
        self.test_device.adb_cmd_capture_msg(self.audio.cfg.get("refresh_sd"))

    def appPrepare(self, case_name):
        self.audio.cfg = self.config.read(self.CONFIG_FILE, case_name)
        self.test_device.adb_cmd_capture_msg(self.audio.cfg.get("remove_audio"))
        self.file_name = self.audio.cfg.get("push_audio").split("/")[-1].replace("\"","")
        self.push_path = self.audio.cfg.get("push_audio").split("\" \"")[1].replace("\"","")
        ret_file = resource.get_media_content(self.file_name)
        self.test_device.adb_cmd_common("push \"" + ret_file + "\" " + "\"" + self.push_path + "\"")
        if "extra_list" in self.audio.cfg:
            for each in self.audio.cfg.get("extra_list").split(","):
                each = each.strip()
                ret_file = resource.get_media_content(each)
                extra_push_path = self.push_path.replace(self.file_name, each)
                self.file_name = each
                assert ret_file, "download file failed: " + each
                self.test_device.adb_cmd_common("push \"" + ret_file + "\" " + "\"" + extra_push_path + "\"")
        self.test_device.adb_cmd_capture_msg(self.audio.cfg.get("refresh_sd"))

    def audioPlaybackLonglasting(self, case_name):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music
        """
        self.logger.debug("run case is " + str(case_name))
        self.appPrepare(case_name)
        self.audio.enterPlayPageFromHome()
        self.audio.enterSongsPage()
        self.audio.playMusicLongLasting(self.audio.cfg.get("audio_name"), \
                                        self.audio.cfg.get("duration_time"), \
                                        self.audio.cfg.get("click_repeat"))

    # Add for need change volume while audio is playing
    # 2014-12-05 graceyix@intel.com
    def audioPlaybackLonglastingWithLoops(self,case_name):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music with loops
        """
        self.logger.debug("run case is " + str(case_name))
        self.appPrepare(case_name)
        self.audio.enterPlayPageFromHome()
        self.audio.enterSongsPage()
        self.audio.playMusicLongLastingWithLoops(self.audio.cfg.get("audio_name"), \
                                                 self.audio.cfg.get("duration_time", 5), \
                                                 self.audio.cfg.get("click_repeat", 0),\
                                                 self.audio.cfg.get("test_loops", 1))
    def audioPlay(self, case_name):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music
        """
        self.logger.debug("run case is " + str(case_name))
        self.appPrepare(case_name)
        self.audio.enterPlayPageFromHome()
        self.audio.enterSongsPage()
        self.audio.playMusic(self.audio.cfg.get("audio_name"))
        self.audio.setRepeat(self.audio.cfg.get("click_repeat"))

    def audioPlayback(self, case_name):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music
        """
        self.logger.debug("run case is " + str(case_name))
        self.appPrepare(case_name)
        self.audio.enterPlayPageFromHome()
        self.audio.enterSongsPage()
        self.audio.playMusic(self.audio.cfg.get("audio_name"))
        self.audio.checkMusicPlayBack(stopTime=self.audio.cfg.get("stop_time"))
        self.logger.debug("case " + str(case_name) + " is pass")

    @alias_name("testAudioPlayback_FLAC_16khz_16bit_stereo")
    def testAudioPlayback_001(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('FLAC codec in flac container at 16Ksample/S')
        """
        self.audioPlayback('test_audio_playback_001')

    @alias_name("testAudioPlayback_LPCM_24khz_16bit_stereo")
    def testAudioPlayback_002(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('LPCM codec in wav container at 24Ksample/S')
        """
        self.audioPlayback('test_audio_playback_002')

    @alias_name("testAudioPlayback_AMR_AMR-NB_mono_8KHz_12.2Kbps")
    def testAudioPlayback_005(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AMR NB Codec 8k sample per second 12.2 Kbps')
        """
        self.audioPlayback('test_audio_playback_005')

    @alias_name("testAudioPlayback_WAV_LPCM_22.05khz_16bit_stereo")
    def testAudioPlayback_007(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('LPCM codec in wav container at 22.05Ksample/S')
        """
        self.audioPlayback('test_audio_playback_007')

    @alias_name("testAudioPlayback_3GP_HE-AAC_16bits")
    def testAudioPlayback_008(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC HE v1 Codec For .3gp container')
        """
        self.audioPlayback('test_audio_playback_008')

    @alias_name("testAudioPlayback_3GP_AAC_ELD_12khz_VBR")
    def testAudioPlayback_010(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC ELD With 16K Sampling frequency')
        """
        self.audioPlayback('test_audio_playback_010')

    @alias_name("testAudioPlayback_MKA_HE-AAC_v2_48KHz_stereo_37.2Kbps_VBR")
    def testAudioPlayback_014(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('HE AAC V2 Codec At 48 Ksample per Second')
        """
        self.audioPlayback('test_audio_playback_014')

    @alias_name("testAudioPlayback_FLAC_32khz_16bit_stereo")
    def testAudioPlayback_015(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('FLAC codec in flac container at 32Ksample/S')
        """
        self.audioPlayback('test_audio_playback_015')

    @alias_name("testAudioPlayback_FLAC_192khz_16bit_stereo")
    def testAudioPlayback_016(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('FLAC codec in flac container at 192Ksample/S')
        """
        self.audioPlayback('test_audio_playback_016')

    @alias_name("testAudioPlayback_3GP_AAC_ELD_44100Hz_CBR")
    def testAudioPlayback_020(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC ELD with 3gp as container')
        """
        self.audioPlayback('test_audio_playback_020')

    @alias_name("testAudioPlayback_WAV_LPCM_44.1khz_16bit_stereo")
    def testAudioPlayback_021(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('LPCM codec in wav container at 44.1Ksample/S')
        """
        self.audioPlayback('test_audio_playback_021')

    @alias_name("testAudioPlayback_AAC_HE-AAC2_44.1khz_stereo_VBR")
    def testAudioPlayback_022(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('HE AAC V2 Codec For .aac container')
        """
        self.audioPlayback('test_audio_playback_022')

    @alias_name("testAudioPlayback_3GP_HE-AAC_44.1KHz_mono_64Kbps_CBR")
    def testAudioPlayback_023(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC HE v1 Decoder Mono')
        """
        self.audioPlayback('test_audio_playback_023')

    @alias_name("testAudioPlayback_3GP_AAC_ELD_16khz_VBR")
    def testAudioPlayback_024(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC ELD with 16k sample rate')
        """
        self.audioPlayback('test_audio_playback_024')

    @alias_name("testAudioPlayback_3GP_HE-AAC_32KHz_stereo_80Kbps_CBR")
    def testAudioPlayback_025(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC HE v1 Decoder At 32Ksample/S')
        """
        self.audioPlayback('test_audio_playback_025')

    @alias_name("testAudioPlayback_3GP_HE-AAC_v2_24KHz_stereo_37Kbps_VBR")
    def testAudioPlayback_027(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('HE AAC V2 Codec At 24 Ksample per Second')
        """
        self.audioPlayback('test_audio_playback_027')

    @alias_name("testAudioPlayback_WAV_LPCM_2min_48khz_16bit_stereo")
    def testAudioPlayback_028(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('LPCM codec in wav container at 48Ksample/S')
        """
        self.audioPlayback('test_audio_playback_028')

    @alias_name("testAudioPlayback_M4A_AAC_ELD_48khz_Mono_VBR")
    def testAudioPlayback_029(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('mono AAC ELD')
        """
        self.audioPlayback('test_audio_playback_029')

    @alias_name("testAudioPlayback_3GP_AAC_ELD_12khz_VBR")
    def testAudioPlayback_031(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC ELD with 32k sample rate')
        """
        self.audioPlayback('test_audio_playback_031')

    @alias_name("testAudioPlayback_M4A_AAC_ELD_24khz")
    def testAudioPlayback_036(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC ELD with 24k sample rate')
        """
        self.audioPlayback('test_audio_playback_036')

    @alias_name("testAudioPlayback_WAV_LPCM_2min_8khz_16bit_stereo")
    def testAudioPlayback_037(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('LPCM codec in wav container at 8Ksample/S')
        """
        self.audioPlayback('test_audio_playback_037')

    @alias_name("testAudioPlayback_MKA_HE-AAC_12KHz_stereo_64Kbps_CBR")
    def testAudioPlayback_038(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC HE v1 Decoder Stereo')
        """
        self.audioPlayback('test_audio_playback_038')

    @alias_name("testAudioPlayback_M4A_E-AAC_v2_44.1KHz_stereo_16Kbps_CBR")
    def testAudioPlayback_039(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC V2 Codec For .m4a container')
        """
        self.audioPlayback('test_audio_playback_039')

    @alias_name("testAudioPlayback_AMR_AMR-WB_mono_16KHz_18.25Kbps")
    def testAudioPlayback_040(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AMR WB 19.85 vocoder')
        """
        self.audioPlayback('test_audio_playback_040')

    @alias_name("testAudioPlayback_MKA_HE-AAC_v2_44.1KHz_stereo_16Kbps_CBR")
    def testAudioPlayback_041(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('HE AAC V2 Codec At 44.1 Ksample per Second')
        """
        self.audioPlayback('test_audio_playback_041')

    @alias_name("testAudioPlayback_3GP_HE-AAC_v2_32KHz_stereo_12.4Kbps_VBR")
    def testAudioPlayback_043(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('HE AAC V2 Codec Stereo')
        """
        self.audioPlayback('test_audio_playback_043')

    @alias_name("testAudioPlayback_M4A_AAC_ELD_48khz_VBR")
    def testAudioPlayback_046(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC ELD with 48k sample rate')
        """
        self.audioPlayback('test_audio_playback_046')

    @alias_name("testAudioPlayback_WAV_LPCM_44.1khz_16bit_mono")
    def testAudioPlayback_048(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('LPCM Mono')
        """
        self.audioPlayback('test_audio_playback_048')

    @alias_name("testAudioPlayback_M4A_HE-AAC_v2_32KHz_stereo_12.4Kbps_VBR")
    def testAudioPlayback_049(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('HE AAC V2 Codec At 32 Ksample per Second')
        """
        self.audioPlayback('test_audio_playback_049')

    @alias_name("testAudioPlayback_3GP_AMR-WB_mono_16KHz_8.85Kbps")
    def testAudioPlayback_052(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AMR WB Codec 16k sample per second 8.85 Kbps')
        """
        self.audioPlayback('test_audio_playback_052')

    @alias_name("testAudioPlayback_AMR_AMR-WB_mono_16KHz_23.05Kbps")
    def testAudioPlayback_053(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AMR WB Codec 16k sample per second 23.05 Kbps')
        """
        self.audioPlayback('test_audio_playback_053')

    @alias_name("testAudioPlayback_3GP_HE-AAC_v2_16KHz_stereo_32Kbps_CBR")
    def testAudioPlayback_057(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('HE AAC V2 Codec At 16 Ksample per Second')
        """
        self.audioPlayback('test_audio_playback_057')

    @alias_name("testAudioPlayback_MP4_MPEG2_4_AAC_LC_96K_44100Hz_Stereo")
    def testAudioPlayback_059(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC LC and mp4 as container')
        """
        self.audioPlayback('test_audio_playback_059')

    @alias_name("testAudioPlayback_AMR_AMR-WB_mono_16KHz_23.85Kbps")
    def testAudioPlayback_060(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AMR WB Codec 16k sample per second 23.85 Kbps')
        """
        self.audioPlayback('test_audio_playback_060')

    @alias_name("testAudioPlayback_3GP_HE-AAC_24KHz_stereo_24Kbps_CBR")
    def testAudioPlayback_061(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC HE v1 Decoder At 24Ksample/S')
        """
        self.audioPlayback('test_audio_playback_061')

    @alias_name("testAudioPlayback_WAV_LPCM_32khz_16bit_stereo")
    def testAudioPlayback_062(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('LPCM codec in wav container at 32Ksample/S')
        """
        self.audioPlayback('test_audio_playback_062')

    @alias_name("testAudioPlayback_3GP_HE-AAC_48KHz_mono_55Kbps_VBR")
    def testAudioPlayback_064(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC HE v1 Decoder At 48Ksample/S')
        """
        self.audioPlayback('test_audio_playback_064')

    @alias_name("testAudioPlayback_M4A_AAC-LC_M4A_44.1khz_mono_VBR_q0.5")
    def testAudioPlayback_065(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC LC mono')
        """
        self.audioPlayback('test_audio_playback_065')

    @alias_name("testAudioPlayback_3GP_AAC_LC_48KHz_stereo_256Kbps")
    def testAudioPlayback_069(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC LC Stereo')
        """
        self.audioPlayback('test_audio_playback_069')

    @alias_name("testAudioPlayback_WAV_LPCM_2min_16khz_16bit_stereo")
    def testAudioPlayback_070(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('LPCM codec in wav container at 16Ksample/S')
        """
        self.audioPlayback('test_audio_playback_070')

    @alias_name("testAudioPlayback_3GP_HE-AAC_44.1KHz_stereo_32Kbps_CBR")
    def testAudioPlayback_071(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music(AAC HE v1 Decoder At 44.1Ksample/S)
        """
        self.audioPlayback('test_audio_playback_071')

    @alias_name("testAudioPlayback_3GP_HE-AAC_16KHz_stereo_22Kbps_VBR")
    def testAudioPlayback_072(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC HE v1 Decoder At 16Ksample/S')
        """
        self.audioPlayback('test_audio_playback_072')

    @alias_name("testAudioPlayback_3GP_AAC_ELD_44.1khz_VBR-3gp")
    def testAudioPlayback_073(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AAC ELD with 44.1k sample rate')
        """
        self.audioPlayback('test_audio_playback_073')

    @alias_name("testAudioPlayback_3GP_AMR-NB_mono_8KHz_4.75kbps")
    def testAudioPlayback_075(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AMR NB Codec 8khz 4.75kbps')
        """
        self.audioPlayback('test_audio_playback_075')

    @alias_name("testAudioPlayback_WAV_LPCM_2min_11.025khz_16bit_stereo")
    def testAudioPlayback_077(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('LPCM codec in wav container at 11.025Ksample/S')
        """
        self.audioPlayback('test_audio_playback_077')

    @alias_name("testAudioPlayback_WAV_PCM_16bit_Mono_8KHz_128Kbps")
    def testAudioPlayback_078(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('16bit with 8khz PCM Codec')
        """
        self.audioPlayback('test_audio_playback_078')

    @alias_name("testAudioPlayback_FLAC_24khz_16bit_stereo")
    def testAudioPlayback_084(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('LPCM codec in wav container at 24Ksample/S')
        """
        self.audioPlayback('test_audio_playback_084')

    @alias_name("testAudioPlaybackLonglasting_FLAC_16khz_16bit_stereo")
    def testAudioPlaybackLonglasting_001(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('FLAC codec in flac container at 16Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_001')

    @alias_name("testAudioPlaybackLonglasting_FLAC_24khz_16bit_stereo")
    def testAudioPlaybackLonglasting_002(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('LPCM codec in wav container at 24Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_002')

    @alias_name("testAudioPlaybackLonglasting_WAV_PCM_24bit_Stereo_48KHz")
    def testAudioPlaybackLonglasting_003(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('24bit PCM Codec')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_003')

    @alias_name("testAudioPlaybackLonglasting_AMR_AMR-NB_mono_8KHz_12.2Kbps")
    def testAudioPlaybackLonglasting_005(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AMR NB Codec 8k sample per second 12.2 Kbps')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_005')

    @alias_name("testAudioPlaybackLonglasting_WAV_LPCM_22.05khz_16bit_stereo")
    def testAudioPlaybackLonglasting_007(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('LPCM codec in wav container at 22.05Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_007')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_16bits")
    def testAudioPlaybackLonglasting_008(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC HE v1 Codec For .3gp container')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_008')

    @alias_name("testAudioPlaybackLonglasting_XMF_MIDI")
    def testAudioPlaybackLonglasting_009(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('MIDI Play With Xmf container')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_009')

    @alias_name("testAudioPlaybackLonglasting_3GP_AAC_ELD_12khz_VBR")
    def testAudioPlaybackLonglasting_010(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC ELD With 16K Sampling frequency')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_010')

    @alias_name("testAudioPlaybackLonglasting_OGG_24kHz_128kbps_Stereo")
    def testAudioPlaybackLonglasting_011(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('Vorbis Decoder With Bits Per Sample : 24')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_011')

    @alias_name("testAudioPlaybackLonglasting_M4A_HE-AAC_8KHz_stereo_64Kbps_CBR")
    def testAudioPlaybackLonglasting_012(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC HE v1 Decoder At 8Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_012')

    @alias_name("testAudioPlaybackLonglasting_MKA_HE-AAC_v2_32bits_per_sample")
    def testAudioPlaybackLonglasting_013(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec with 32 bits per sample')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_013')

    @alias_name("testAudioPlaybackLonglasting_MKA_HE-AAC_v2_48KHz_stereo_37.2Kbps_VBR")
    def testAudioPlaybackLonglasting_014(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec At 48 Ksample per Second')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_014')

    @alias_name("testAudioPlaybackLonglasting_FLAC_32khz_16bit_stereo")
    def testAudioPlaybackLonglasting_015(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('FLAC codec in flac container at 32Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_015')

    @alias_name("testAudioPlaybackLonglasting_FLAC_192khz_16bit_stereo")
    def testAudioPlaybackLonglasting_016(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('FLAC codec in flac container at 192Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_016')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_v2_16bits")
    def testAudioPlaybackLonglasting_017(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec with 16 bits per sample')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_017')

    @alias_name("testAudioPlaybackLonglasting_OTA_234bytes")
    def testAudioPlaybackLonglasting_018(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('ota music file')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_018')

    @alias_name("testAudioPlaybackLonglasting_3GP_AAC_ELD_8khz_VBR")
    def testAudioPlaybackLonglasting_019(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC ELD .3gp with 8k sample rate')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_019')

    @alias_name("testAudioPlaybackLonglasting_3GP_AAC_ELD_44100Hz_CBR")
    def testAudioPlaybackLonglasting_020(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC ELD with 3gp as container')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_020')

    @alias_name("testAudioPlaybackLonglasting_WAV_LPCM_44.1khz_16bit_stereo")
    def testAudioPlaybackLonglasting_021(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('LPCM codec in wav container at 44.1Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_021')

    @alias_name("testAudioPlaybackLonglasting_AAC_HE-AAC2_44.1khz_stereo_VBR")
    def testAudioPlaybackLonglasting_022(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec For .aac container')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_022')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_44.1KHz_mono_64Kbps_CBR")
    def testAudioPlaybackLonglasting_023(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC HE v1 Decoder Mono')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_023')

    @alias_name("testAudioPlaybackLonglasting_3GP_AAC_ELD_16khz_VBR")
    def testAudioPlaybackLonglasting_024(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC ELD with 16k sample rate')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_024')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_32KHz_stereo_80Kbps_CBR")
    def testAudioPlaybackLonglasting_025(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC HE v1 Decoder At 32Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_025')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_v2_11.025KHz_stereo_24Kbps_CBR")
    def testAudioPlaybackLonglasting_026(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec At 11.025 Ksample per Second')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_026')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_v2_24KHz_stereo_37Kbps_VBR")
    def testAudioPlaybackLonglasting_027(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec At 24 Ksample per Second')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_027')

    @alias_name("testAudioPlaybackLonglasting_WAV_LPCM_2min_48khz_16bit_stereo")
    def testAudioPlaybackLonglasting_028(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('LPCM codec in wav container at 48Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_028')

    @alias_name("testAudioPlaybackLonglasting_M4A_AAC_ELD_48khz_Mono_VBR")
    def testAudioPlaybackLonglasting_029(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('mono AAC ELD')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_029')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_V2_16bit_48khz_320kbps_CBR")
    def testAudioPlaybackLonglasting_030(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec at 320 Kbps per channel @48 ks/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_030')

    @alias_name("testAudioPlaybackLonglasting_3GP_AAC_ELD_12khz_VBR")
    def testAudioPlaybackLonglasting_031(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC ELD with 32k sample rate')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_031')

    @alias_name("testAudioPlaybackLonglasting_RTTTL")
    def testAudioPlaybackLonglasting_032(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('rtttl music file')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_032')

    @alias_name("testAudioPlaybackLonglasting_3GP_AAC_ELD_11.025khz_VBR")
    def testAudioPlaybackLonglasting_033(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC ELD with 11.025k sample rate')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_033')

    @alias_name("testAudioPlaybackLonglasting_3GP_AAC-LC_3GP_44.1khz_stereo_128kbps_CBR")
    def testAudioPlaybackLonglasting_034(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC LC Stereo CBR')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_034')

    @alias_name("testAudioPlaybackLonglasting_M4A_AAC_ELD_24khz")
    def testAudioPlaybackLonglasting_036(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC ELD with 24k sample rate')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_036')

    @alias_name("testAudioPlaybackLonglasting_WAV_LPCM_2min_8khz_16bit_stereo")
    def testAudioPlaybackLonglasting_037(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('LPCM codec in wav container at 8Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_037')

    @alias_name("testAudioPlaybackLonglasting_MKA_HE-AAC_12KHz_stereo_64Kbps_CBR")
    def testAudioPlaybackLonglasting_038(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC HE v1 Decoder Stereo')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_038')

    @alias_name("testAudioPlaybackLonglasting_M4A_E-AAC_v2_44.1KHz_stereo_16Kbps_CBR")
    def testAudioPlaybackLonglasting_039(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC V2 Codec For .m4a container')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_039')

    @alias_name("testAudioPlaybackLonglasting_AMR_AMR-WB_mono_16KHz_18.25Kbps")
    def testAudioPlaybackLonglasting_040(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AMR WB 19.85 vocoder')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_040')

    @alias_name("testAudioPlaybackLonglasting_MKA_HE-AAC_v2_44.1KHz_stereo_16Kbps_CBR")
    def testAudioPlaybackLonglasting_041(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec At 44.1 Ksample per Second')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_041')

    @alias_name("testAudioPlaybackLonglasting_WAV_LPCM_2min_12khz_16bit_stereo")
    def testAudioPlaybackLonglasting_042(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('LPCM codec in wav container at 12Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_042')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_v2_32KHz_stereo_12.4Kbps_VBR")
    def testAudioPlaybackLonglasting_043(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec Stereo')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_043')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_v2_8KHz_stereo_24kbps_CBR")
    def testAudioPlaybackLonglasting_044(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec At 8 Ksample per Second')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_044')

    @alias_name("testAudioPlaybackLonglasting_3GP_AMR-NB_16bit_mono_7.4Khz_8Kbps")
    def testAudioPlaybackLonglasting_045(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AMR-NB For .3GPv5 Decode AMR-NB_16bit_mono_7.4Khz_8Kbps_60sec')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_045')

    @alias_name("testAudioPlaybackLonglasting_M4A_AAC_ELD_48khz_VBR")
    def testAudioPlaybackLonglasting_046(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC ELD with 48k sample rate')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_046')

    @alias_name("testAudioPlaybackLonglasting_M4A_HE-AAC_22.05KHz_stereo_64Kbps_VBR")
    def testAudioPlaybackLonglasting_047(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC HE v1 Decoder At 22.05Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_047')

    @alias_name("testAudioPlaybackLonglasting_WAV_LPCM_44.1khz_16bit_mono")
    def testAudioPlaybackLonglasting_048(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('LPCM Mono')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_048')

    @alias_name("testAudioPlaybackLonglasting_M4A_HE-AAC_v2_32KHz_stereo_12.4Kbps_VBR")
    def testAudioPlaybackLonglasting_049(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec At 32 Ksample per Second')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_049')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_v2_48KHz_mono_34.8Kbps_VBR")
    def testAudioPlaybackLonglasting_050(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec For .3gp container')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_050')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_24bits_per_sample")
    def testAudioPlaybackLonglasting_051(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec with 24 bits per sample')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_051')

    @alias_name("testAudioPlaybackLonglasting_3GP_AMR-WB_mono_16KHz_8.85Kbps")
    def testAudioPlaybackLonglasting_052(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AMR WB Codec 16k sample per second 8.85 Kbps')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_052')

    @alias_name("testAudioPlaybackLonglasting_AMR_AMR-WB_mono_16KHz_23.05Kbps")
    def testAudioPlaybackLonglasting_053(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AMR WB Codec 16k sample per second 23.05 Kbps')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_053')

    @alias_name("testAudioPlaybackLonglasting_HE-AAC_12KHz_stereo_64Kbps_CBR")
    def testAudioPlaybackLonglasting_054(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC HE v1 Decoder At 12Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_054')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_11.025KHz_stereo_64Kbps_CBR")
    def testAudioPlaybackLonglasting_055(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC HE v1 Decoder At 11.025Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_055')

    @alias_name("testAudioPlaybackLonglasting_M4A_HE-AAC_v2_44.1KHz_mono_16Kbps_CBR")
    def testAudioPlaybackLonglasting_056(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC HE v1 Decoder At 11.025Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_056')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_v2_16KHz_stereo_32Kbps_CBR")
    def testAudioPlaybackLonglasting_057(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec At 16 Ksample per Second')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_057')

    @alias_name("testAudioPlaybackLonglasting_M4A_AAC_ELD_48_khz_128kbps_VBR")
    def testAudioPlaybackLonglasting_058(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC ELD with 128kbps')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_058')

    @alias_name("testAudioPlaybackLonglasting_MP4_MPEG2_4_AAC_LC_96K_44100Hz_Stereo")
    def testAudioPlaybackLonglasting_059(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC LC and mp4 as container')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_059')

    @alias_name("testAudioPlaybackLonglasting_AMR_AMR-WB_mono_16KHz_23.85Kbps")
    def testAudioPlaybackLonglasting_060(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AMR WB Codec 16k sample per second 23.85 Kbps')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_060')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_24KHz_stereo_24Kbps_CBR")
    def testAudioPlaybackLonglasting_061(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC HE v1 Decoder At 24Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_061')

    @alias_name("testAudioPlaybackLonglasting_WAV_LPCM_32khz_16bit_stereo")
    def testAudioPlaybackLonglasting_062(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('LPCM codec in wav container at 32Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_062')

    @alias_name("testAudioPlayLonglastingback_IMY")
    def testAudioPlaybackLonglasting_063(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('imy music file')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_063')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_48KHz_mono_55Kbps_VBR")
    def testAudioPlaybackLonglasting_064(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC HE v1 Decoder At 48Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_064')

    @alias_name("testAudioPlaybackLonglasting_M4A_AAC-LC_M4A_44.1khz_mono_VBR_q0.5")
    def testAudioPlaybackLonglasting_065(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC LC mono')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_065')

    @alias_name("testAudioPlaybackLonglasting_RTX")
    def testAudioPlaybackLonglasting_066(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('rtx music file')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_066')

    @alias_name("testAudioPlaybackLonglasting_M4A_HE-AAC_v2_12KHz_stereo_24Kbps_CBR")
    def testAudioPlaybackLonglasting_067(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec At 12 Ksample per Second')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_067')

    @alias_name("testAudioPlaybackLonglasting_M4A_HE-AAC_v2_22.05KHz_stereo_34Kbps_VBR")
    def testAudioPlaybackLonglasting_068(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('HE AAC V2 Codec At 22.05 Ksample per Second')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_068')

    @alias_name("testAudioPlaybackLonglasting_3GP_AAC_LC_48KHz_stereo_256Kbps")
    def testAudioPlaybackLonglasting_069(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC LC Stereo')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_069')

    @alias_name("testAudioPlaybackLonglasting_WAV_LPCM_2min_16khz_16bit_stereo")
    def testAudioPlaybackLonglasting_070(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('LPCM codec in wav container at 16Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_070')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_44.1KHz_stereo_32Kbps_CBR")
    def testAudioPlaybackLonglasting_071(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting(AAC HE v1 Decoder At 44.1Ksample/S)
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_071')

    @alias_name("testAudioPlaybackLonglasting_3GP_HE-AAC_16KHz_stereo_22Kbps_VBR")
    def testAudioPlaybackLonglasting_072(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC HE v1 Decoder At 16Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_072')

    @alias_name("testAudioPlaybackLonglasting_3GP_AAC_ELD_44.1khz_VBR-3gp")
    def testAudioPlaybackLonglasting_073(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC ELD with 44.1k sample rate')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_073')

    @alias_name("testAudioPlaybackLonglasting_M4A_AAC_ELD_8khz_VBR-m4a")
    def testAudioPlaybackLonglasting_074(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('AAC ELD with 8k sample rate')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_074')

    @alias_name("testAudioPlaybackLonglasting_MIDI_32_Polyphony_Max")
    def testAudioPlaybackLonglasting_076(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('MIDI Play With 32 Polyphony Max')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_076')

    @alias_name("testAudioPlaybackLonglasting_WAV_PCM_16bit_Mono_8KHz_128Kbps")
    def testAudioPlaybackLonglasting_078(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('16bit with 8khz PCM Codec')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_078')

    @alias_name("testAudioPlaybackLonglasting_MP3_MPEG-2.5_L3_8khz_stereo_8kbps_CBR")
    def testAudioPlaybackLonglasting_079(self):
        """
        This test used to test Audio playback longlasting
        The test case spec is following:
        1. Launch music
        2. Play music longlasting('MPEG 2.5 Layer 3 (Mp3) 8khz CBR Codec_Intel-optimized')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_079')

    # 2014-12-05 graceyix@intel.com
    def testAudioPlayback_Longlasting_3GP_HE_AAC_8KHz_stereo_64Kbps_CBR_wsHS(self):
        """
        Summary: DECODE Audio playback AAC HE on wsHS Long Lasting

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(HE-AAC_8KHz_stereo_64Kbps_CBR.3gp)
        """
        self.audioPlaybackLonglasting('test_audio_mum_stress_001')

    def testAudioPlayback_Iteration_MKA_AAC_LC_MKA_44_1khz_stereo_128kbps_CBR(self):
        """
        Summary: DECODE Audio playback AAC LC on wsHS Iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(AAC-LC_MKA_44.1khz_stereo_128kbps_CBR.mka)
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_002')

    def testAudioPlayback_Longlasting_MP3_MPEG1_L3_44_1khz_stereo_128kbps(self):
        """
        Summary: DECODE Audio playback MP3 on wsHS Long Lasting

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(LongDuration_MPEG1_L3_44.1khz_stereo_128kbps_2hrs_l_auberge_espanole.mp3)
        """
        self.audioPlaybackLonglasting('test_audio_mum_stress_003')

    def testAudioPlayback_Iteration_M4A_AAC_LC_44_1KHz_mono_256Kbps_IHF(self):
        """
        Summary: MUSIC_PB Music playback on IHF Iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(AAC_LC_44.1KHz_mono_256Kbps.m4a)
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_004')

    def testAudioPlayback_Iteration_WAV_MP3_PCM_16bit_Stereo_48KHz_1536Kbps(self):
        """
        Summary: Small duration audio mp3 & PCM files iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(PCM_16bit_Stereo_48KHz_1536Kbps_5sec.wav.mp3)
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_005')

    def testAudioPlayback_Longlasting_3GP_AAC_HE_v2_44_1khz_stereo_VBR(self):
        """
        Summary: MUSIC_PB Music playback on IHF Long Lasting

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(ST_AUDIO-MUM_MUSIC_020)
        """
        self.audioPlaybackLonglasting('test_audio_mum_stress_006')

    def testAudioPlayback_Iteration_AMR_NB_mono_8KHz_7_95Kbps(self):
        """
        Summary: DECODE Audio playback AMR NB Iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(AMR-NB_mono_8KHz_7.95Kbps.AMR)
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_007')

    def testAudioPlayback_Iteration_MIDI_96Polyphony(self):
        """
        Summary: DECODE Audio playback Midi Iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(96Polyphony.mid)
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_008')

    def testAudioPlayback_Iteration_VORBIS_44_1khz_stereo_128kbps(self):
        """
        Summary: DECODE Audio playback Vorbis Iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(LongDuration_vorbis_44.1khz_stereo_128kbps_2hrs_l_auberge_espanole.ogg)
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_009')

    def testAudioPlayback_Longlasting_AMR_NB_8khz_mono_7_4kbps(self):
        """
        Summary: DECODE Audio playback AMR NB Long Lasting

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(LongDuration_AMR_NB_8khz_mono_7.4kbps_2hrs_l_auberge_espanole.3gp)
        """
        self.audioPlaybackLonglasting('test_audio_mum_stress_010')

    def testAudioPlayback_Longlasting_AMR_WB_16khz_stereo_12_65kbps(self):
        """
        Summary: DECODE Audio playback AMR WB Long Lasting

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(LongDuration_AMR_WB_16khz_stereo_12.65kbps_2hrs_l_auberge_espanole.wav.3gp)
        """
        self.audioPlaybackLonglasting('test_audio_mum_stress_011')

    def testAudioPlayback_Iteration_MPEG_1_L3_44_1khz_stereo_320kbps_CBR(self):
        """
        Summary: DECODE Audio playback MP3 on wsHS Iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(MPEG-1_L3_44.1khz_stereo_320kbps_CBR_1min.mp3)
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_012')

    def testAudioPlayback_Iteration_AMR_WB_mono_16KHz_23_05Kbps(self):
        """
        Summary: DECODE Audio playback AMR WB Iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(AMR-WB_mono_16KHz_23.05Kbps.AMR)
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_013')

    def testAudioPlayback_Iteration_3GP_AMR_NB_mono_8KHz_4_75kbps(self):
        """
        Summary: MUSIC_PB Music playback on wsHS Iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(AMR-NB_mono_8KHz_4.75kbps.3gp)
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_014')

    def testAudioPlayback_Longlasting_FLAC_2min_48khz_16bit_mono(self):
        """
        Summary: DECODE Audio playback long lasting Audio playback FLAC decoder with flac container

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(FLAC_2min_48khz_16bit_mono.flac)
        """
        self.audioPlaybackLonglasting('test_audio_mum_stress_015')

    def testAudioPlayback_Longlasting_VORBIS_44_1khz_Stereo_128kbps(self):
        """
        Summary: DECODE Audio playback Vorbis Long Lasting

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(LongDuration_vorbis_44.1khz_stereo_128kbps_2hrs_l_auberge_espanole.ogg)
        """
        self.audioPlaybackLonglasting('test_audio_mum_stress_016')

    def testAudioPlayback_Longlasting_MIDI(self):
        """
        Summary: DECODE Audio playback Midi Long Lasting

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(LongDuration_MIDI_5m23s.mid)
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_019')

    def testAudioPlayback_Longlasting_AAC_HE_44_1khz_stereo_VBR(self):
        """
        Summary: MUSIC_PB Music playback on wsHS Long Lasting

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(LongDuration_AAC_HE_44.1khz_stereo_VBR_2hrs_l_auberge_espanole.3gp)
        """
        self.audioPlaybackLonglasting('test_audio_mum_stress_020')

    def testAudioPlayback_Longlasting_MP3_LoopPlay(self):
        """
        Summary: mp3 play in loop mode for 24 hours

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(MP3s_44khz_6ch_192kbps_122s_ID.mp3)
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_023')

    def testAudioPlayback_Longlasting_MP3_PCM_Short_LoopPlay(self):
        """
        Summary: Small duration audio mp3 & PCM files in a loop for 100 Hours.

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(PCM_2.1ch_8b_11khz_Electronic.wav)
        """
        case_name = "test_audio_mum_stress_024"
        self.logger.debug("run case is %s"%case_name)
        self.appPrepare(case_name)
        audio_list = [self.audio.cfg.get("audio_name")] +\
                     [".".join(_.strip().split(".")[:-1]) for _ in self.audio.cfg.get("extra_list").split(",")]
        self.playMusicList( audio_list, \
                            repeat_time = int(self.audio.cfg.get("click_repeat")),\
                            playtime = int(self.audio.cfg.get("duration_time")))

    def testAudioPlayback_Iteration_3GP_HE_AAC_8KHz_stereo_64Kbps_CBR(self):
        """
        Summary: DECODE Audio playback AAC HE on wsHS Iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(HE-AAC_8KHz_stereo_64Kbps_CBR.3gp)
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_025')

    def testAudioPlayback_Longlasting_AAC_LC_32khz_192kbps_VBR(self):
        """
        Summary: DECODE Audio playback AAC LC on wsHS Long Lasting

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(AAC_LC_2min_32khz_192kbps_VBR.aac)
        """
        self.audioPlaybackLonglasting('test_audio_mum_stress_026')

    def testAudioPlayback_Iteration_MP3_AudioEffectOn(self):
        """
        Summary: Music Playback with effect enabled Iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play mp3 audio iteration with audio effect on
        """
        case_name = "test_audio_mum_stress_028"
        self.logger.debug("run case is " + str(case_name))
        self.appPrepare(case_name)
        self.audio.enterPlayPageFromHome()
        self.audio.setEffect()
        self.audio.enterSongsPage()
        self.audio.playMusicLongLasting(self.audio.cfg.get("audio_name"), \
                                        self.audio.cfg.get("duration_time"), \
                                        self.audio.cfg.get("click_repeat"))
        self.audio.back()
        self.audio.turnOffEffect()



    def testAudioPlayback_Iteration_AMR_NB(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('AMR NB Codec 8khz 4.75kbps')
        """
        self.audioPlaybackLonglastingWithLoops('test_audio_mum_stress_010')

    def testAudioPlayback_Longlasting_WAV_LPCM(self):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music('LPCM codec in wav container at 11.025Ksample/S')
        """
        self.audioPlaybackLonglasting('test_audio_playbacklonglasting_077')

    def testAudioPlayback_Iteration_VolUpDown_100cycles(self):
        """
        Summary: Music Playback with effect enabled Iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(AAC_LC_2min_32khz_192kbps_VBR.aac)
        3. Change the Volume
        """
        self.audioPlay('test_audio_mum_stress_volume_change')
        repeat_time = int(self.audio.cfg.get("volume_repeat"))
        self.audio.playMusicSetVolumeUpDown(repeat_time)
        self.audio.checkMusicPlayBack(self.audio.cfg.get("duration_time"))

    def testAudioPlayback_Iteration_VolMaxMin_100cycles(self):
        """
        Summary: Music Playback with effect enabled Iterative

        This test used to test Audio playback Stress
        The test case spec is following:
        1. Launch music
        2. Play music(AAC_LC_2min_32khz_192kbps_VBR.aac)
        3. Change the Volume
        """
        self.audioPlay('test_audio_mum_stress_volume_allchange')
        repeat_time = int(self.audio.cfg.get("volume_repeat"))
        self.audio.playMusicSetVolume(repeat_time)
        self.audio.checkMusicPlayBack(self.audio.cfg.get("duration_time"))
        self.audio.resetVolume(self.audio.cfg.get("volume"))

    def playMusic(self, case_name):
        """
        This test used to test Audio playback
        The test case spec is following:
        1. Launch music
        2. Play music
        """
        self.logger.debug("run case is " + str(case_name))
        self.appPrepare(case_name)
        self.audio.enterPlayPageFromHome()
        self.audio.enterSongsPage()
        self.audio.playMusic(self.audio.cfg.get("audio_name"))

    def loops(self, action):
        for i in range(int(self.audio.cfg.get("times"))):
            self.logger.debug("loops: %d"%(i+1))
            action(i)

    def testAudioPlayback_Iteration_ClickProgressbar_200cycles(self):
        self.playMusic("click_progress")
        def clickProgress(i):
            self.audio.clickProgress((i%2+1.0)/3.0)
            self.assertTrue(self.audio.isMusicPlaying())
        self.loops(clickProgress)

    def testAudioPlayback_Iteration_PauseThenHomeScreen_200cycles(self):
        self.playMusic("test_play_audio_pause_then_home")
        self.audio.musicRepeat(2)
        def pauseThenHome(i):
            self.audio.clickPause()
            self.audio.home()
            #self.audio.enterPlayPageFromHome()
            self.audio.launch_music_from_home()
            assert not self.audio.isMusicPlaying(), "Music still playing"
            self.audio.clickPause() # Resume
            time.sleep(3)
        self.loops(pauseThenHome)

    def testAudioPlayback_Iteration_PausePrevNext_100cycles(self):
        self.playMusic("pause_click_last_and_next")
        self.audio.musicRepeat(1)
        def clickLastAndNext(i):
            self.audio.clickPause()
            time.sleep(3)
            self.audio.clickPrevious()
            time.sleep(3)
            self.audio.clickNext()
            time.sleep(3)
        self.loops(clickLastAndNext)

    def testAudioPlayback_Iteration_RotateScreen_200cycles(self):
        self.playMusic("play_audio_and_rotate")
        self.audio.musicRepeat(2)
        routateList = ["l", "r"]
        def setRotate(i):
            self.audio.setOrientation(routateList[i%2])
            time.sleep(2)
            self.assertTrue(self.audio.isMusicPlaying())
        self.loops(setRotate)
        self.audio.setOrientation()

    def testPlayback_DefaultApp_RotateScreen(self):
        self.playMusic("play_audio_and_rotate")
        self.audio.musicRepeat(2)
        for o in ["l", "r"]:
            self.audio.setOrientation(o)
            time.sleep(4)
        self.audio.setOrientation()

    def testAudioPlayback_Iteration_50SongsLoop(self):
        self.playMusic("play_from_end_to_start")
        self.audio.musicRepeat(1)
        def playPrev(i):
            self.audio.clickNext()
            time.sleep(5)
            self.assertTrue(self.audio.isMusicPlaying())
        self.loops(playPrev)

    def testAudioPlayback_Iteration_50SongsShuffle_100cycles(self):
        self.playMusic("play_shuffle")
        self.audio.musicRepeat(1)
        self.audio.clickShuffle()  # enter suffle mode
        def playShuffle(i):
            self.audio.clickNext()
            time.sleep(2)
            self.assertTrue(self.audio.isMusicPlaying())
        self.loops(playShuffle)

    def testAudioDecode_Playlist_Shuffle(self):
        #self.appPrepare("play_shuffle")
        self.test_device.clean_play_music_content()  # helping rerun single case.
        self.audio.cfg = self.config.read(self.CONFIG_FILE, "play_shuffle")
        file_name = self.audio.cfg.get("push_audio").split("/")[-1].replace("\"","")
        play_list = sorted(self.test_device.DEFAULT_CONTENTS + [file_name])
        self.test_device.deploy_play_music_content(files = play_list)
        play_list = [_.split(".")[0] for _ in play_list]
        self.audio.enterPlayPageFromHome()
        self.audio.enterSongsPage()
        self.audio.playMusic(play_list[0])
        self.audio.musicRepeat(1)
        self.audio.clickShuffle()
        defaut_sequence = reduce(lambda x,y: str(x)+str(y), range(len(play_list)))
        self.logger.debug("Default play sequence is %s"%defaut_sequence)
        sequence = ""
        for i in range(2*len(play_list)):
            self.audio.clickNext()
            time.sleep(2)
            song = self.audio.getTrackName()
            sequence += str(play_list.index(song))
        self.logger.debug("Actual sequence is %s"%sequence)

        assert defaut_sequence not in sequence, "Play list not shuffle"

    def testAudioPlayback_Longlasting_AudioEffectOn(self):
        case_name = "play_with_effect_long_lasting"
        self.logger.debug("run case is " + str(case_name))
        self.appPrepare(case_name)
        self.audio.enterPlayPageFromHome()
        self.audio.setEffect()
        self.audio.enterSongsPage()
        self.audio.playMusicLongLasting(self.audio.cfg.get("audio_name"), \
                                        self.audio.cfg.get("duration_time"), \
                                        self.audio.cfg.get("click_repeat"))
        self.audio.back()
        self.audio.turnOffEffect()

    def pushTmpFile(self, fileSize=1):
        filename = "push_%dMB.tmp"%fileSize
        if not hasattr(self, "tmpFile"):
            fileSize = int(fileSize)
            from testlib.common.base import getTmpDir
            self.tmpFile = os.path.join(getTmpDir(),filename)
            os.system("dd if=/dev/urandom of=%s bs=1048576 count=%d"%(self.tmpFile,fileSize))
        self.test_device.adb_cmd_common(r'push "%s" "%s"'%(self.tmpFile,self.push_path))
        return os.path.join(self.push_path, filename)

    def adbRm(self, f):
        self.test_device.adb_cmd(r'rm "%s"'%f)

    def testAudioPlayback_Longlasting_MP3PlayWithFileTransfer(self):
        #root dut
        self.test_device.root_on_device()
        #start music play back
        self.playMusic("play_and_push_file")
        self.audio.musicRepeat(2)
        s = time.time()
        while time.time()-s < int(self.audio.cfg.get("playtime")):
            f = self.pushTmpFile(int(self.audio.cfg.get("filesize")))
            assert self.audio.isInPlayPage()
            self.adbRm(f)

    def playMusicList(self, audio_list, repeat_time = 1, playtime = 60 ):
        self.audio.enterPlayPageFromHome()
        self.audio.enterSongsPage()
        repeat = False
        for a in audio_list:
            self.audio.playMusic(a)
            if not repeat:
                self.audio.musicRepeat(repeat_time)
                repeat = True
            time.sleep(playtime)
            self.audio.back()
