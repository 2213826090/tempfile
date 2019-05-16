# coding: utf-8
from testlib.domains.mum_settings_impl import SettingImpl
from testlib.photos.mum_photos_impl import PhotosImpl
from testlib.util.common import g_common_obj
from testlib.multimedia.testcasebase import TestCaseBase
from testlib.multimedia.multimedia_setting import MultiMediaSetting, MultiMediaHandle
import os
import time
from testlib.multimedia.multimedia_log import MultimediaLogger
logger = MultimediaLogger.instance()


class VideoPlayBackIteration(TestCaseBase):
    """
    @summary: Test Video PlayBack
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(VideoPlayBackIteration, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        g_common_obj.stop_app_am("com.google.android.apps.plus")
        g_common_obj.stop_app_am("com.google.android.apps.photos")
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(VideoPlayBackIteration, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()
        time.sleep(3)

    def appPrepare(self, case_name):
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_video.conf')
        self.video = PhotosImpl(\
            self.config.read(cfg_file, case_name))
        self.setting = SettingImpl(\
            self.config.read(cfg_file, case_name))
        g_common_obj.adb_cmd_capture_msg(" rm -rf /sdcard/DCIM/Camera/*")
        
        self.multimedia_handle = MultiMediaHandle()
        self.multimedia_setting = MultiMediaSetting(cfg_file)
        self.multimedia_setting.clearFolder(self.video.cfg.get("remove_video"))
        self.multimedia_setting.install_apk("video_apk")
        time.sleep(2)
        self.push_path = self.multimedia_setting.push_file(self.video.cfg.get("push_video"), self.video.cfg.get("datapath"))
        
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))
        self.video.set_orientation_n()
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def videoPlayBackIteration(self, case_name, bigfileskiptime=0):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        push_folder = os.path.split(os.path.split(self.push_path)[0])[-1]
        for iteration in range(int(self.video.cfg.get("iteration_times"))):
            logger.info("Playback video iteration: {0} ".format(iteration + 1))
            if self.multimedia_setting.get_android_version() == "O":
                self.multimedia_handle.launchVideoApp()
                self.multimedia_handle.videoPlayBack(self.push_path)
                assert self.multimedia_handle.checkVideoPlayBackWithComparePicture(60), 'video play failed'
            else:
                self.video.launchPhotos(push_folder)
                time.sleep(2)
                self.multimedia_handle.checkVideoPlayBackWithPhotoApp(int(self.video.cfg.get("iteration_last_time")), bigfileskiptime)
                self.setting.recent_app()
                self.setting.remove_recent_app("OtcVideoPlayer")
        print "case " + str(case_name) + " is pass"

    def testVideoPlayback_Iteration_3GP_H263_20cycles(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('H263 for 3GP continuouly play 20 times')
        3. Former name: test_video_playback_iteration_mum_001
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_001")

    def testVideoPlayback_Iteration_MP4_H264_HP_1080P_30fps_50Mbps_AAC_LC_48KHz_320Kbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('H263 for 3GP continuouly play 20 times')
        3. Former name: test_video_playback_iteration_mum_002
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_002")

    def testVideoPlayback_Iteration_MP4_H264_HP_1080P_30fps_25Mbps_AAC_ELD_48KHz_192kbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('Iterative Video playback: H264_HP_1080P_30fps_25Mbps_AAC_ELD_48KHz_192kbps.mp4, iteratively')
        3. Former name: test_video_playback_iteration_mum_003
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_003", 20)

    def test_video_playback_iteration_mum_004(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('H263 for 3GP continuouly play 20 times')
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_004")

    def test_video_playback_iteration_mum_005(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('MPEG4 part 2 SP (Simple Profile) and (Advanced Simple Profile) play 20 times continuoously')
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_005")

    def testVideoPlayback_Iteration_3GP_H264_Part2v4_20cycles(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('H264 Part 2 For .3GPv4 continuouly play 20 times')
        3. Former name: test_video_playback_iteration_mum_006
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_006")

    def testVideoPlayback_Iteration_H264_L3_0_L4_0_HP_20cycles(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('Video Playback H.264 Level 3.0, 4.0 High Profile play contnuously 20 times')
        3. Former name: test_video_playback_iteration_mum_007
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_007")

    def testVideoPlayback_Iteration_H264_L1_0_L2_0_BP_20cycles(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('Video Playback H.264 Level 1.0, 2.0 Baseline Profile play continuously 20 times')
        3. Former name: test_video_playback_iteration_mum_008
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_008")

    def testVideoPlayback_Iteration_MP4_H263_20cycles(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('Video Codec : H263 For .Mp4 play 20 times')
        3. Former name: test_video_playback_iteration_mum_009
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_009")

    def testVideoPlayback_Iteration_3GP_MPEG4_Part2_3GPv5_20cycles(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('MPEG4 Part 2 For .3GPv5 continuouly play 20 times')
        3. Former name: test_video_playback_iteration_mum_010
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_010")

    def test_video_playback_iteration_mum_011(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('MPEG4 Part 2 For .3GPv5 continuouly play 20 times')
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_011")

    def test_video_playback_iteration_mum_012(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('MPEG4 Part 2 For .3GPv5 continuouly play 20 times')
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_012")

    def test_video_playback_iteration_mum_013(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('MPEG4 Part 2 For .3GPv5 continuouly play 20 times')
        """
        self.videoPlayBackIteration("mum_test_video_playback_iteration_013")

    def test_video_playback_iteration_001(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('H263 Decoder, H263_720x450_30fps_no_audio')
        """
        self.videoPlayBackIteration("test_video_playback_iteration_001")

    def test_video_playback_iteration_004(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_004")

    def test_video_playback_iteration_005(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_005")

    def test_video_playback_iteration_006(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_006")

    def test_video_playback_iteration_007(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_007")

    def test_video_playback_iteration_008(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_008")

    def test_video_playback_iteration_010(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_010")

    def test_video_playback_iteration_011(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_011")

    def test_video_playback_iteration_012(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_012")

    def test_video_playback_iteration_013(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_013")

    def test_video_playback_iteration_014(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_014")

    def test_video_playback_iteration_015(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_015")

    def test_video_playback_iteration_017(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_017")

    def test_video_playback_iteration_018(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_018")

    def test_video_playback_iteration_019(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_019")

    def test_video_playback_iteration_021(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_021")

    def test_video_playback_iteration_022(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_022")

    def test_video_playback_iteration_024(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_024")

    def test_video_playback_iteration_025(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_025")

    def test_video_playback_iteration_026(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_026")

    def test_video_playback_iteration_027(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_027")

    def test_video_playback_iteration_028(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_028")

    def test_video_playback_iteration_029(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_029")

    def test_video_playback_iteration_030(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_030")

    def test_video_playback_iteration_031(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_031")

    def test_video_playback_iteration_032(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_032")

    def test_video_playback_iteration_033(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_033")

    def test_video_playback_iteration_034(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_034")

    def test_video_playback_iteration_035(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_035")

    def test_video_playback_iteration_036(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_036")

    def test_video_playback_iteration_037(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_037")

    def test_video_playback_iteration_038(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_038")

    def test_video_playback_iteration_039(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_039")

    def test_video_playback_iteration_040(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_040")

    def test_video_playback_iteration_041(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_041")

    def test_video_playback_iteration_042(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_042")

    def test_video_playback_iteration_043(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_043")

    def test_video_playback_iteration_044(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_044")

    def test_video_playback_iteration_045(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_045")

    def test_video_playback_iteration_046(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBackIteration("test_video_playback_iteration_046")
