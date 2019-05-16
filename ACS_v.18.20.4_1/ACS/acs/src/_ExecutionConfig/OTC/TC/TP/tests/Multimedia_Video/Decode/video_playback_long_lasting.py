# coding: utf-8
from testlib.domains.mum_settings_impl import SettingImpl
from testlib.photos.mum_photos_impl import PhotosImpl
from testlib.util.common import g_common_obj
from testlib.multimedia.testcasebase import TestCaseBase
from testlib.multimedia.multimedia_setting import MultiMediaSetting, MultiMediaHandle
from testlib.multimedia.multimedia_setting import CheckPhotoApp
import os
import time


class VideoPlayBackLonglasting(TestCaseBase):
    """
    @summary: Test Video PlayBack
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(VideoPlayBackLonglasting, self).setUp()
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
        super(VideoPlayBackLonglasting, self).tearDown()
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
        self.check_photo_app = CheckPhotoApp(cfg_file)
        self.multimedia_setting.clearFolder(self.video.cfg.get("remove_video"))
        time.sleep(2)
        self.push_path = self.multimedia_setting.push_file(self.video.cfg.get("push_video"), self.video.cfg.get("datapath"))
        
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))
        self.video.set_orientation_n()
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def videoPlayBackLonglasting(self, case_name, bigfileskiptime=0):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        lasting_time = int(self.video.cfg.get("lasting_time"))
        push_folder = os.path.split(os.path.split(self.push_path)[0])[-1]
        self.check_photo_app.check_photo_app()
        self.video.launchPhotos(push_folder)
        time.sleep(2)
        self.multimedia_handle.checkVideoPlayBackWithPhotoApp(lasting_time, bigfileskiptime)
        print "case " + str(case_name) + " is pass"

    def videoPlayBackLonglastingNoTime(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        lasting_time = int(self.video.cfg.get("lasting_time"))
        push_folder = os.path.split(os.path.split(self.push_path)[0])[-1]
        self.video.launchPhotos(push_folder)
        time.sleep(2)
#         self.multimedia_handle.launchVideoApp()
#         self.multimedia_handle.videoPlayBack(self.push_path)
        self.multimedia_handle.checkVideoPlayBackWithPhotoApp(stoptime=lasting_time)
#         start_time = time.time()
#         while lasting_time > 0:
#             self.video.playback_video_un_QR_code_photoplus(playTime=self.video.cfg.get("wait_time"), stoptime=self.video.cfg.get("stop_time"), flag=True)
#             playback_time = time.time() - start_time
#             print "play back time is "
#             lasting_time = lasting_time - playback_time
#             print "lasting_time is ", lasting_time
        print "case " + str(case_name) + " is pass"

    def test_video_playback_long_lasting_mum_001(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('Verify that user can play 3D video file for 30 minutes')
        """
        self.videoPlayBackLonglastingNoTime("mum_test_video_playback_long_lasting_001")

    def testVideoPlayback_Longlasting_MP4_H264_L4_0_MP_720x424_25fps_AAC(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('Playback of very long duration video files (2+ hours)')
        3. Former name: test_video_playback_long_lasting_mum_002
        """
        self.videoPlayBackLonglasting("mum_test_video_playback_long_lasting_002", 20)

    def testVideoPlayback_Longlasting_WEBM_VP8_VGA_20fps_2Mbps_Vorbis_48KHz_128kbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('Long-time Video playback: VP8_VGA_20fps_2Mbps_Vorbis_48KHz_128kbps.webm (3 hours)')
        3. Former name: test_video_playback_long_lasting_mum_003
        """
        self.videoPlayBackLonglasting("mum_test_video_playback_long_lasting_003", 20)

    def testVideoPlayback_Longlasting_MKV_H264_HP_L4_1_720P_25fps_1_9Mbps_NA(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('Long-time Video playback: H264_HP_L4.1_720P_25fps_1.9Mbps_na.mkv (5 hours)')
        3. Former name: test_video_playback_long_lasting_mum_004
        """
        self.videoPlayBackLonglasting("mum_test_video_playback_long_lasting_004", 20)

    def testVideoPlayback_Longlasting_TS_H264_HP_L3_2_720P_60fps_4Mbps_NA(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('Long-time Video playback: H264_HP_L3.2_720P_60fps_4Mbps_na.ts (2 hours)')
        3. Former name: test_video_playback_long_lasting_mum_005
        """
        self.videoPlayBackLonglasting("mum_test_video_playback_long_lasting_005", 20)

    def testVideoPlayback_Longlasting_MP4_H264_HP_L4_1_1080P_30fps_4_5Mbps_NA(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('Long-time Video playback: H264_HP_L4.1_1080P_30fps_4.5Mbps_na.mp4 (2 hours)')
        3. Former name: test_video_playback_long_lasting_mum_006
        """
        self.videoPlayBackLonglasting("mum_test_video_playback_long_lasting_006", 20)
