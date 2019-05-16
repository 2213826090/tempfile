# coding: utf-8
import os
import time
from testlib.photos.mum_photos_impl import PhotosImpl
from testlib.util.common import g_common_obj
from testlib.multimedia.testcasebase import TestCaseBase
from testlib.multimedia.multimedia_setting import MultiMediaSetting, MultiMediaHandle, MultiMediaBasicTestCase


class VideoPlayBack(TestCaseBase):
    """
    @summary: Test Video PlayBack
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(VideoPlayBack, self).setUp()
        self.d = g_common_obj.get_device()
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
        super(VideoPlayBack, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()
        time.sleep(3)

    def appPrepare(self, case_name):
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_video.conf')
        self.video = PhotosImpl(\
            self.config.read(cfg_file, case_name))
        self.file_name = self.video.cfg.get("push_video").split("/")[-1].replace("\"","")
        self.push_path = self.video.cfg.get("push_video").split("\" \"")[1].replace("\"","")
        
        self.multimedia_handle = MultiMediaHandle()
        self.multimedia_setting = MultiMediaSetting(cfg_file)
        self.multimedia_setting.clearFolder(self.video.cfg.get("remove_video"))
        time.sleep(2)
        self.multimedia_setting.push_file(self.video.cfg.get("push_video"), self.video.cfg.get("datapath"))
        self.video.set_orientation_n()
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def appPrepare2(self, case_name, model=1):
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_video.conf')
        self.video = PhotosImpl(\
            self.config.read(cfg_file, case_name))

        self.multimedia_handle = MultiMediaHandle()
        self.multimedia_setting = MultiMediaSetting(cfg_file)
        self.multimedia_setting.clearFolder(self.video.cfg.get("remove_video"))
        time.sleep(2)
        if model == 1:
            self.push_path = self.multimedia_setting.push_file(self.video.cfg.get("push_video"), self.video.cfg.get("datapath"))
        
        self.multimedia_setting.install_apk("video_apk")
        self.multimedia_setting.install_apk("alarm_apk")
        self.video.set_orientation_n()
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def videoPlayBack(self, case_name, t_bigfileskiptime=0):
        print "run case is " + str(case_name)
        self.appPrepare2(case_name)
        push_folder = os.path.split(os.path.split(self.push_path)[0])[-1]
        if self.multimedia_setting.get_android_version() == "O":
            self.multimedia_handle.launchVideoApp()
            self.multimedia_handle.videoPlayBack(self.push_path)
            assert self.multimedia_handle.checkVideoPlayBack(), 'video play failed'
        else:
            self.video.launchPhotos(push_folder)
            time.sleep(2)
            self.multimedia_handle.checkVideoPlayBackWithPhotoApp(stoptime=self.video.cfg.get("stop_time"), bigfileskiptime=t_bigfileskiptime)
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBack(self, case_name):
        MultiMediaBasicTestCase().testVideoPlayBack(case_name)

    def checkVideoPlayBack(self, s=60):
        return self.multimedia_handle.checkVideoPlayBack(s)

    def checkVideoPlayBackWithComparePicture(self, stoptime, bigfileskiptime=0):
        return self.multimedia_handle.checkVideoPlayBackWithComparePicture(stoptime, bigfileskiptime)

    def checkVideoPlayBackComplete(self, s=900):
        tt = -1
        for _ in range(s/10):
            time.sleep(10)
            if self.d(textContains="Completed").exists:
                return
            else:
                try:
                    if tt == -1:
                        for _ in range(10):
                            if self.d(resourceId="android:id/time").exists:
                                break
                            self.d.click(self.x/2,self.y/2)
                        tt = self.d(resourceId="android:id/time").text
                        tt = self.setTimeToSec(tt)
                    for _ in range(10):
                        if self.d(resourceId="android:id/time_current").exists:
                            break
                        self.d.click(self.x/2,self.y/2)
                    ct = self.d(resourceId="android:id/time_current").text
                    ct = self.setTimeToSec(ct)
                except Exception as e:
                    if self.d(textContains="Completed").exists:
                        return
                    else:
                        assert False, e
                if ct == tt:
                    assert not self.d(textContains="error").exists or not self.d(textContains="fail").exists,"Play back error! please check it."
                    return

    def setTimeToSec(self, time):
        time = time.split(":")
        i = 1
        temp = 0
        for s in time[::-1]:
            temp += int(s) * i
            i *= 60
        return int(temp)

    def videoPlayBackNoTime(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare2(case_name)
        push_folder = os.path.split(os.path.split(self.push_path)[0])[-1]
        self.video.launchPhotos(push_folder)
        time.sleep(2)
#         self.multimedia_handle.launchVideoApp()
#         self.multimedia_handle.videoPlayBack(self.push_path)
        self.multimedia_handle.checkVideoPlayBackWithPhotoApp(stoptime=self.video.cfg.get("stop_time"))
#         self.video.playback_video_un_QR_code_photoplus(playTime=self.video.cfg.get("wait_time"), stoptime=self.video.cfg.get("stop_time"), flag=True)
        print "case " + str(case_name) + " is pass"

    def testVideoPlayback_3GP_H263_720x450_30fps_no_audio(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('H263 Decoder, H263_720x450_30fps_no_audio')
        3. Former name: testVideoPlayBack_001
        """
        self.videoPlayBack("test_video_playback_001")

    def testVideoPlayback_3GP_H264_128_94_25fps_145kbps_AAC_LC_22_05kHz_48kbps_Stereo(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_002
        """
        self.videoPlayBack("test_video_playback_002")

    def testVideoPlayback_MP4_H264_176_144_12_5fps_627kbps_ALAC_44_1kHz_352kbps_Stereo(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_003
        """
        self.videoPlayBack("test_video_playback_003")

    def testVideoPlayBack_004(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_004")

    def testVideoPlayBack_005(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_005")

    def testVideoPlayBack_006(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_006")

    def testVideoPlayBack_007(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_007")

    def testVideoPlayBack_008(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_008")

    def testVideoPlayback_MP4_H264_320_240_25fps_3566kbps_AAC_Main_48kHz_165kbps_Stereo(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_009
        """
        self.videoPlayBack("test_video_playback_009")

    def testVideoPlayBack_010(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_010")

    def testVideoPlayBack_011(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_011")

    def testVideoPlayBack_012(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_012")

    def testVideoPlayback_MP4_H264_800_600_30fps_1641kbps_AAC_LC_Stereo_44_1kHz_152kbps_Stereo(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_013
        """
        self.videoPlayBack("test_video_playback_013")

    def testVideoPlayback_MP4_L1_0_SP_QCIF_15fps_AACplus_112kbp_32KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_014
        """
        self.videoPlayBack("test_video_playback_014")

    def testVideoPlayback_MP4_L2_0_SP_QVGA_15fps_eAACplus_96kbp_48KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_015
        """
        self.videoPlayBack("test_video_playback_015")

    def testVideoPlayBack_016(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_016")

    def testVideoPlayback_3GP_MPEG4_SP_480P_30fps_AAC_ELD_48KHz_192kbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_017
        """
        self.videoPlayBack("test_video_playback_017")

    def testVideoPlayback_3GP_MPEG4_SP_720x480_30fps_AMR_NB_8KHz_12_2kbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_018
        """
        self.videoPlayBack("test_video_playback_018")

    def testVideoPlayback_MP4_MPEG4_SP_L1_1080_30fps_AACplus_160kbps_48KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_019
        """
        self.videoPlayBack("test_video_playback_019")

    def testVideoPlayBack_020(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_020")

    def testVideoPlayback_MP4_H264_L1_3_HP_CIF_30fps_eAACplus_128kb_44KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_021
        """
        self.videoPlayBack("test_video_playback_021")

    def testVideoPlayback_MP4_H264_L3_1_BP_720p_30fps_AAC_96kb_32KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_022
        """
        self.videoPlayBack("test_video_playback_022")

    def testVideoPlayback_MP4_H264_852_480_30fps_7763kbps_ALAC_44_1kHz_352kbps_Stereo(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_023
        """
        self.videoPlayBack("test_video_playback_023", 20)

    def testVideoPlayBack_024(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_024")

    def testVideoPlayback_MP4_H264_L1_0_BP_QCIF_10fps_AAC_64kb_32KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_025
        """
        self.videoPlayBack("test_video_playback_025")

    def testVideoPlayback_MP4_H264_L2_2_MP_VGA_15fps_AAC_112kb_48K(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_026
        """
        self.videoPlayBack("test_video_playback_026")

    def testVideoPlayback_MP4_H264_L3_0_HP_D1SD_30fps_eAACplus_128kb_48KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_027
        """
        self.videoPlayBack("test_video_playback_027")

    def testVideoPlayBack_028(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_028")

    def testVideoPlayback_TS_H264_BP_720x576_25fps_AACplus_128kbps_22_05KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_029
        """
        self.videoPlayBack("test_video_playback_029")

    def testVideoPlayback_TS_H264_HP_1280x960_51fps_AACplus_192kbps_44_1KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_030
        """
        self.videoPlayBack("test_video_playback_030")

    def testVideoPlayback_TS_H264_HP_1408x1152_38fps_eAACplus_128kbps_48KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_031
        """
        self.videoPlayBack("test_video_playback_031")

    def testVideoPlayback_3GP_H264_HP_L3_2_720P_60fps_AAC_LC_48KHz_320kbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_032
        """
        self.videoPlayBack("test_video_playback_032")

    def testVideoPlayback_MKV_VP8_30fps_852_480_AAC_LC_44_1kHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_033
        """
        self.videoPlayBack("test_video_playback_033")

    def testVideoPlayback_MKV_VP8_640x360_25fps_Vorbis_44KHz_128Kbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_034
        """
        self.videoPlayBack("test_video_playback_034")

    def testVideoPlayback_MP4_H264_1920x1080_30fps_24_7Mbps_AAC_352_8Kbps_44_1KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_035
        """
        self.videoPlayBack("test_video_playback_035")

    def testVideoPlayBack_036(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_036")

    def testVideoPlayBack_037(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_037")

    def testVideoPlayBack_038(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_038")

    def testVideoPlayBack_039(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_039")

    def testVideoPlayback_MP4_MPEG4_800_600_30fps_2075kbps_AAC_LC_48kHz_165kbps_Stereo(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_040
        """
        self.videoPlayBack("test_video_playback_040")

    def testVideoPlayBack_041(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_041")

    def testVideoPlayback_WEBM_VP8_QCIF_25fps_Vorbis_44_1KHz_128kbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_042
        """
        self.videoPlayBack("test_video_playback_042")

    def testVideoPlayBack_043(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_043")

    def testVideoPlayBack_044(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_044")

    def testVideoPlayBack_045(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_045")

    def testVideoPlayback_WEBM_VP9_936Kbps_640x360_30fps_vorbis_224kbps_48khz_2channel(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_046
        """
        self.videoPlayBack("test_video_playback_046")

    def testVideoPlayback_MP4_H264_BP_1080P_24fps_25Mbps_AAC_ELD_48KHz_192kbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_047
        """
        self.videoPlayBack("test_video_playback_047")

    def testVideoPlayBack_048(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_048")

    def testVideoPlayBack_049(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_049")

    def testVideoPlayback_MP4_H264_L3_1_HP_720p_30fps_AAC_160kb_48KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_050
        """
        self.videoPlayBack("test_video_playback_050")

    def testVideoPlayback_3GP_H263_Decoder_Level_30_CIF_352_288_30fps_384_Kbs(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_051
        """
        self.videoPlayBack("test_video_playback_051")

    def testVideoPlayback_3GP_H263_Level_40_0_None_352_288_30_0fps_2048kbps_aac_stereo_48khz_192kbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_052
        """
        self.videoPlayBack("test_video_playback_052")

    def testVideoPlayback_WEBM_VP8_CIF_25fps_Vorbis_44_1KHz_128kbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_053
        """
        self.videoPlayBack("test_video_playback_053")

    def testVideoPlayback_MP4_H264_L4_1_HP_720p_30fps_eAACplus_128kb_48KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_054
        """
        self.videoPlayBack("test_video_playback_054")

    def testVideoPlayBack_055(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_055")

    def testVideoPlayBack_056(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_056")

    def testVideoPlayback_MP4_H264_Primary_Output_Video_Playback_Content_QVGA_15_fps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        3. Former name: testVideoPlayBack_057
        """
        self.videoPlayBack("test_video_playback_057")

    def testVideoPlayBack_058(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_058")

    def testVideoPlayBack_059(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_059")

    def testVideoPlayBack_060(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_060")

    def testVideoPlayBack_061(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_061")

    def testVideoPlayBack_062(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_062")

    def testVideoPlayBack_063(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_063")

    def testVideoPlayBack_064(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_064")

    def testVideoPlayBack_065(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_065")

    def testVideoPlayBack_066(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_066")

    def testVideoPlayBack_067(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_067")

    def testVideoPlayBack_068(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_068")

    def testVideoPlayBack_069(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_069")

    def testVideoPlayBack_070(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_070")

    def testVideoPlayBack_071(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_071")

    def testVideoPlayBack_072(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_072")

    def testVideoPlayBack_073(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_073")

    def testVideoPlayBack_074(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_074")

    def testVideoPlayBack_075(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_075")

    def testVideoPlayBack_076(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_076")

    def testVideoPlayBack_077(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_077")

    def testVideoPlayBack_078(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_078")

    def testVideoPlayBack_079(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_079")

    def testVideoPlayBack_080(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_080")

    def testVideoPlayBack_081(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_081")

    def testVideoPlayBack_082(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_082")

    def testVideoPlayBack_083(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_083")

    def testVideoPlayBack_084(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_084")

    def testVideoPlayBack_085(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_085")

    def testVideoPlayBack_086(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_086")

    def testVideoPlayBack_087(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_087")

    def testVideoPlayBack_088(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_088")

    def testVideoPlayBack_089(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_089")

    def testVideoPlayBack_090(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_090")

    def testVideoPlayBack_091(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_091")

    def testVideoPlayBack_092(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_092")

    def testVideoPlayBack_093(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_093")

    def testVideoPlayBack_094(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_094")

    def testVideoPlayBack_095(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_095")

    def testVideoPlayBack_096(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_096")

    def testVideoPlayBack_097(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_097")

    def testVideoPlayBack_098(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_098")

    def testVideoPlayBack_099(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_099")

    def testVideoPlayBack_100(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_100")

    def testVideoPlayBack_101(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_101")

    def testVideoPlayBack_102(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_102")

    def testVideoPlayBack_103(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_103")

    def testVideoPlayBack_104(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_104")

    def testVideoPlayBack_105(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_105")

    def testVideoPlayBack_106(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_106")

    def testVideoPlayBack_107(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_107")

    def testVideoPlayBack_108(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_108")

    def testVideoPlayBack_109(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("test_video_playback_109")

    def testVideoPlayback_MKV_MPEG4_ASP_720P_30fps_4_5Mbps_AAC_128kbps_32KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video
        """
        self.videoPlayBack("mum_test_video_playback_047")

    def testMPEG4_352_288_24fps_144kbps_aaclc_22_05kHz_38kbps_Mono_3GP(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_008")

    def testMPEG4_part_2_SP_Simple_Profile_Level3_CIF_352x288_30fps_NO_audio(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_009")

    def testVideo_Playback_Mp3_Mkv(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_video_playback_110")

    def testVideo_Playback_HEAAC_V2_Mp4(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_video_playback_111")

