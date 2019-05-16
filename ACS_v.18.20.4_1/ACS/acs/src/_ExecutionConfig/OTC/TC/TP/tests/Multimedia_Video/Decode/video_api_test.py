# coding: utf-8
import os
import sys
import time
from testlib.photos.mum_photos_impl import PhotosImpl
from testlib.multimedia.testcasebase import TestCaseBase
from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_setting import MultiMediaSetting, MultiMediaHandle, MultiMediaBasicTestCase
from testlib.multimedia.multimedia_log import MultimediaLogger
logger = MultimediaLogger.instance()

class VideoAPITest(TestCaseBase):
    """
    @summary: Test Video PlayBack
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(VideoAPITest, self).setUp()
        self.d = g_common_obj.get_device()
        self._test_name = __name__
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]
        print "[Setup]: %s" % self._test_name
        g_common_obj.stop_app_am("videoplayer.app.instrument.otc.intel.com.otcvideoplayer")
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")
        self.cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
                                'tests.tablet.mum_auto_video.conf')

        self.multimedia_setting = MultiMediaSetting(self.cfg_file)
        self.hardware = self.multimedia_setting.get_paltform_hardware()
        self.tag = "[Decode API] "

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(VideoAPITest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()
        time.sleep(3)
        g_common_obj.stop_app_am("videoplayer.app.instrument.otc.intel.com.otcvideoplayer")
        time.sleep(3)

    def appPrepare(self, case_name, model=1):
        # cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
        #     'tests.tablet.mum_auto_video.conf')
        self.video = PhotosImpl(\
            self.config.read(self.cfg_file, case_name))
        g_common_obj.adb_cmd_capture_msg(" rm -rf /sdcard/DCIM/Camera/*")
        
        self.multimedia_handle = MultiMediaHandle()
        # self.multimedia_setting = MultiMediaSetting(cfg_file)
        self.multimedia_setting.clearFolder(self.video.cfg.get("remove_video"))
        time.sleep(2)
        if model == 1:
            self.push_path = self.multimedia_setting.push_file(self.video.cfg.get("push_video"), self.video.cfg.get("datapath"))
            
        self.multimedia_setting.install_apk("video_apk")
        self.multimedia_setting.install_apk("alarm_apk")
        
        self.video.set_orientation_n()
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def setTimeToSec(self, time):
        time = time.split(":")
        i = 1
        temp = 0
        for s in time[::-1]:
            temp += int(s) * i
            i *= 60
        return int(temp)

    def setRotation(self, mode):
        self.d(className="android.widget.ImageButton").click()
        time.sleep(1)
        self.d(text=mode).click()
        time.sleep(1)

    def videoPlayBack(self, push_path=""):
        if push_path == "":
            push_path = self.push_path
        return self.multimedia_handle.videoPlayBack(push_path)

    def streamingVideoPlayBack(self, path="", flag=1):
        if path == "":
            path = self.video.cfg.get("video_path")
        return self.multimedia_handle.streamingVideoPlayBack(path, flag)

    def checkVideoPlayBack(self, s=30):
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

    def checkVideoNotSupport(self, s=60):
        for _ in range(s):
            if self.d(text="Can't play this video.").exists:
                return
            assert not self.d(resourceId="android:id/time_current").exists and not self.d(resourceId="android:id/time").exists, "error!can play it!"
            time.sleep(1)
            self.d.click(self.x/2,self.y/2)

    def testVideoPlayBack(self, case_name):
        assert MultiMediaBasicTestCase().testVideoPlayBack(case_name), 'video play failed'

    def testVideoPlayBackAudio(self, case_name):
        assert MultiMediaBasicTestCase().testVideoPlayBack(case_name, check_hang=False), 'video play failed'

    def testVideoPlayBackViaVLC(self, case_name, change_file_flag=False):
        MultiMediaBasicTestCase().testVideoPlayBackViaVLC(case_name, change_file_flag)

    def testVideoPlayBackViaGallery(self, case_name):
        MultiMediaBasicTestCase().testVideoPlayBackViaGallery(case_name)

    def testVideoPlayBackLongIteration(self, case_name):
        MultiMediaBasicTestCase().testVideoPlayBackLongIterationTimes(case_name)

    def testVideoPlayBackWithManyTimes(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_handle.launchVideoApp()
        assert self.checkVideoPlayBack(), 'video play failed'
        time.sleep(2)
        for _ in range(100):
            self.videoPlayBack()
            self.checkVideoPlayBack()
            self.d.press("back")
            time.sleep(2)
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBackWithManyTimes_ASP_file(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        for _ in range(20):
            self.videoPlayBack()
            assert self.checkVideoPlayBack(), "video play failed"
            self.d.press("back")
            self.d.press("back")
            time.sleep(2)
            if not self.d(textContains="OtcVideoPlayer").exists:
                self.multimedia_handle.launchVideoApp()
        self.push_path = self.multimedia_setting.push_file(self.video.cfg.get("push_video_2"), self.video.cfg.get("datapath"))
        for _ in range(20):
            self.videoPlayBack()
            assert self.checkVideoPlayBack(), "video play failed"
            self.d.press("back")
            self.d.press("back")
            time.sleep(2)
            if not self.d(textContains="OtcVideoPlayer").exists:
                self.multimedia_handle.launchVideoApp()
        print "case " + str(case_name) + " is pass"

    def interuptionVideoPlayBackSwitchHome(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        time.sleep(1)
        self.d.press.home()
        self.multimedia_setting.click_recent_app("OtcVideoPlayer")
        time.sleep(1)
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testStreamingVideoPlayBack(self, case_name):
        MultiMediaBasicTestCase().testStreamingVideoPlayBack(case_name)

    def interuptionStreamingVideoPlayBackSwitchHome(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        assert self.checkVideoPlayBack(), "Video play failed"
        time.sleep(1)
        self.d.press.home()
        self.multimedia_setting.click_recent_app("OtcVideoPlayer")
        time.sleep(1)
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def videoPlayControlProcess(self, case_name):
        logger.debug(self.tag + 'run case name is ' + sys._getframe().f_back.f_code.co_name)
        MultiMediaBasicTestCase().videoPlayControlProcess(case_name)

    def videoPlayVP8CodecProcess(self, case_name):
        logger.debug(self.tag + 'run case name is ' + sys._getframe().f_back.f_code.co_name)
        MultiMediaBasicTestCase().videoPlayCodecProcess(case_name,msg='OMX.*.vp8')

    def videoPlayControlProcess_complte_pasue(self, case_name):
        assert MultiMediaBasicTestCase().videoPlayControlProcess_Complete_pasue(case_name)

    def videoPlayControlProcessWithVolume(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        assert self.checkVideoPlayBack(), "video play failed"
        self.multimedia_setting.set_play_time(0.5)
        assert self.checkVideoPlayBack(), "video play failed"
        self.multimedia_handle.clickOtcVideoPlayer_Control(resourceId="android:id/rew")
        self.multimedia_handle.clickOtcVideoPlayer_Control(resourceId="android:id/ffwd")
        self.multimedia_setting.set_play_time_with_swipe(0.2)
        assert self.checkVideoPlayBack(), "video play failed"
        self.multimedia_handle.clickOtcVideoPlayer_Control(resourceId="android:id/pause")
        self.d.press.volume_up()
        self.checkVideoPlayBack()
        self.d.press.volume_down()
        assert self.checkVideoPlayBack(), "video play failed"
        print "case " + str(case_name) + " is pass"

    def videoDecodeSecondDisplay(self, case_name):
        logger.debug(self.tag + 'run case name is ' + sys._getframe().f_back.f_code.co_name)
        self.appPrepare(case_name)
        g_common_obj.adb_cmd_capture_msg('am force-stop com.android.gallery3d')
        assert self.checkSecondDisplay(), 'Cannot find the second display screen'
        assert self.playVideoSecondDisplay(self.push_path), 'cannot launch the video'
        g_common_obj.adb_cmd_capture_msg('am force-stop com.android.gallery3d')
        assert self.playVideoMainDisplay(self.push_path), 'cannot launch the video'
        logger.debug(self.tag + 'run case ' + sys._getframe().f_back.f_code.co_name + ' is pass')

    def checkSecondDisplay(self):
        """
        check the DUT has two display screen
        :return: True, has two display screen
        """
        display2_out = g_common_obj.adb_cmd_capture_msg("dumpsys |grep mExternalTouchViewport")
        if 'valid=false' in display2_out:
            logger.warning(self.tag + "Cannot find the second display screen:%s"%display2_out)
            return False
        else:
            logger.debug(self.tag + "The second display screen exits.")
            return True

    def playVideoSecondDisplay(self, file_path):
        """
        Play the video using the second display in Gallery
        :return: True, launch success
        """
        g_common_obj.adb_cmd_common("logcat -c")
        play_cmd = 'am start --display 1 -a android.intent.action.VIEW -d "file://%s" -t "video/*" -n com.android.gallery3d/.app.MovieActivity'%file_path
        g_common_obj.adb_cmd_capture_msg(play_cmd)
        time.sleep(5)
        gallery_play_act = 'com.android.gallery3d/com.android.gallery3d.app.MovieActivity'
        display1_surface = g_common_obj.adb_cmd_capture_msg("dumpsys SurfaceFlinger|grep  -A10 'Display 0 HWC layers' ")
        display2_surface = g_common_obj.adb_cmd_capture_msg("dumpsys SurfaceFlinger|grep  -A10 'Display 1 HWC layers' ")
        logger.debug(self.tag + "Video play failed, display1_surface:\n {0},\ndisplay2_surface:\n{1}".format(display1_surface,display2_surface))
        if gallery_play_act not in display1_surface and gallery_play_act in display2_surface:
            logger.debug(self.tag + "Video play in the second display")
        else:
            logger.debug(self.tag + "Video play failed")
            return False
        # check video play
        return self.checkVideoInfoFromLogcat()

    def playVideoMainDisplay(self, file_path):
        """
        Play the video using the second display in Gallery
        :return: True, launch success
        """
        g_common_obj.adb_cmd_common("logcat -c")
        play_cmd = 'am start --display 0 -a android.intent.action.VIEW -d "file://%s" -t "video/*" -n com.android.gallery3d/.app.MovieActivity' % file_path
        g_common_obj.adb_cmd_capture_msg(play_cmd)
        time.sleep(6)
        gallery_play_act = 'com.android.gallery3d/com.android.gallery3d.app.MovieActivity'
        display1_surface = g_common_obj.adb_cmd_capture_msg("dumpsys SurfaceFlinger|grep  -A10 'Display 0 HWC layers' ")
        display2_surface = g_common_obj.adb_cmd_capture_msg("dumpsys SurfaceFlinger|grep  -A10 'Display 1 HWC layers' ")
        logger.debug(self.tag + "Video play failed, display1_surface:\n {0} \ndisplay2_surface:\n{1}".format(display1_surface,display2_surface))
        if gallery_play_act in display1_surface and gallery_play_act in display2_surface:
            logger.debug(self.tag + "Video play in the main display")
        else:
            logger.debug(self.tag + "Video play failed")
            return False
        return self.checkVideoInfoFromLogcat()


    def checkVideoInfoFromLogcat(self):
        hardware_decode_msg = g_common_obj.adb_cmd_capture_msg("logcat -d |grep 'I libva'")
        ur_decode_msg = g_common_obj.adb_cmd_capture_msg("logcat -d |grep 'OMXMaster: makeComponentInstance'")
        logger.debug(self.tag + "Video play failed, hardware_decode_msg:\n {0},\nur_decode_msg:\n{1}".format(hardware_decode_msg, ur_decode_msg))
        if not hardware_decode_msg and not ur_decode_msg:
            logger.debug(self.tag + "cannot find the libva or OMXMaster, video play abnormal")
            return False
        else:
            return True


    def testVideoNotSupport(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack(flag=2)
        self.checkVideoNotSupport()
        print "case " + str(case_name) + " is pass"

    def testPlayback_VP8_640x480_20fps_Vorbis_48KHz_192Kbps_mkv_check_pause_resume_seek_rotate_sleep(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.videoPlayControlProcess("test_API_video_playback_033")

    def testPlayback_H264_MP_1080P_60fps_50Mbps_AAC_LC_48KHz_320Kbps_mp4_check_pause_resume_seek_rotate_sleep(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.videoPlayControlProcess("test_API_video_playback_034")

    def testPlayback_H264_HP_1080P_60fps_50Mbps_AAC_LC_48KHz_320Kbps_mp4_check_pause_resume_seek_rotate_sleep(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.videoPlayControlProcess("test_API_video_playback_035")

    def testPlayback_H264_BP_1080P_60fps_50Mbps_AAC_LC_48KHz_320Kbps_mp4_check_pause_resume_seek_rotate_sleep(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.videoPlayControlProcess("test_API_video_playback_036")

    def testh265_HEVC_With_10Bit_2160P_30Fps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.videoPlayControlProcess_complte_pasue("test_API_video_playback_128")

    def testh265_HEVC_With_10Bit_1080p_30Fps_8Mbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.videoPlayControlProcess_complte_pasue("test_API_video_playback_129")

    def testh265_HEVC_With_10Bit_2160P_30Fps_60Mbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.videoPlayControlProcess_complte_pasue("test_API_video_playback_130")

    def testCheck_SeparateCard_for_each_apps_with_multi_tasking(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Switch to Home, then back from Home
        """
        self.interuptionVideoPlayBackSwitchHome("test_API_video_playback_038")

    def testPlay_H263_MP4_container(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_043")

    def testSHALL_support_m4v_extension_MP4_video_file(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.videoPlayControlProcessWithVolume("test_API_video_playback_045")

    def testPlaying_streaming_Music_Home(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Switch to Home, then back from Home
        """
        self.interuptionStreamingVideoPlayBackSwitchHome("test_API_video_playback_058")

    def testSupport_native_display_resolution_FWVGA_854x480_30fps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_074")

    def testPlayback_H263_3GPv5_L1_0_BP_QCIF_15fps_AMR_WB_256kb_16KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_075")

    def testPlayback_H264_Level1_0_Main_Profile_QCIF_176x144_10fps_AAC_Multimedia_96kb_48KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_076")

    def testIterative_video_playback_H264_HP_1080P_60fps_50Mbps_AAC_LC_48KHz_320Kbps_mp4_iteratively(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackLongIteration("test_API_video_playback_077")

    def testPlayback_H264_Level4_0_Main_Profile_HD_1920x1080_30fps_AAC_Multimedia_128kb_48KHz_and_pause_resume(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_078")

    def testH263_352_288_30fps_858kbps_No_Audio_mp4(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_079")

    def testH263_Level20_Sub_QCIF_30fps_128_KbS(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_080")

    def testPlay_H263_20F_QQVGA_AMRNB_12_2k(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_131")

    def testPlayback_VP8_CIF_25fps_Vorbis_44_1KHz_128kbps_webm_iteratively(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackLongIteration("test_API_video_playback_081")

    def testVideo_H264_29_970fps_5mn29s_640x480_MP4_RTP_streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_088")

    def testPlayback_MP4_h264_HiP_4_0_1080p_30FPS_25Mbps_AAC_LC_48KHz_256Kbps_1Min_179MB_BBB_mp4(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.videoPlayControlProcessWithVolume("test_API_video_playback_101")

    def testPlayback_MPEG4_ASP_1080P_30fps_6Mbps_AAC_LC_48KHz_128Kbps(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_111")

    def testPlayback_MPEG4_ASP_Lv5_720X480_30fps_AAC_ST_160kbp_48KHZ(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_112")

    def testPlayback_MPEG4_SP_and_MEPG4_ASP_Continuously(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithManyTimes_ASP_file("test_API_video_playback_114")

    def testCorrupt_File_Playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoNotSupport("test_API_video_playback_115")

    def testPlayback_MPEG4_ASP_Lv0_176X144_30fps_AAC_Plus_ST_64kbp_32KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_116")

    def testH264_480X320_Playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_118")

    def testH264_854x480_Playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_119")

    def testH265_480X320_Playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_120")

    def testH265_854x480_Playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_121")

    def testh265_simple_10bit_lowbitrat_mkv(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch VLC app
            2. Play video
            """
        self.testVideoPlayBackViaVLC("test_API_video_playback_140")

    def testh265_HEVC_With_10Bit_medium_mp4(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch VLC app
            2. Play video
            """
        self.testVideoPlayBackViaVLC("test_API_video_playback_141")

    def testVideo_Playback_MPEG4_SPL0_QCIF_176x144_15Fps(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch VLC app
            2. Play video
            """
        if self.hardware == "gordon_peak":
            self.testVideoPlayBackViaGallery("test_API_video_playback_142")
        elif self.hardware in ["androidia_64", "r2_cht_mrd" ]:
            self.testVideoPlayBack("test_API_video_playback_142")
        else:
            self.testVideoPlayBackViaVLC("test_API_video_playback_142")

    def testPlayback_H264_4K_External_Storage(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch VLC app
            2. Play video
            """
        if self.hardware in ["androidia_64", "r2_cht_mrd" ]:
            self.testVideoPlayBack("test_API_video_playback_143")
        else:
            self.testVideoPlayBackViaVLC("test_API_video_playback_143", change_file_flag=True)


    def testVideo_Playback_DIVX5_720p_30Fps(self):
        """
        This test used to test DIVX Video playback
        he test case spec is following:
        1. Launch VLC app
        2. Play video
        """
        self.testVideoPlayBackViaVLC("test_API_video_playback_144")

    def testVideo_Playback_DIVX6_1080p_23Fps(self):
        """
        This test used to test DIVX Video playback
        he test case spec is following:
        1. Launch VLC app
        2. Play video
        """
        self.testVideoPlayBackViaVLC("test_API_video_playback_145")

    def testVideo_Playback_Xvid_960x720_30Fps(self):
        """
        This test used to test DIVX Video playback
        he test case spec is following:
        1. Launch VLC app
        2. Play video
        """
        self.testVideoPlayBackViaVLC("test_API_video_playback_153")

    def testVP8_480X320_Playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_122")

    def testVP8_854x480_Playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_123")

    def testVP8_HW_4K_Decode(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.videoPlayVP8CodecProcess("test_API_video_playback_151")

    def testVP9_480X320_Playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_124")

    def testVP9_854x480_Playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_125")

    def testMPEG4_480X320_Playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_126")

    def testMPEG4_854x480_Playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_127")


    def testVideo_Playback_H264_HP_1408x1152_Ts_Pause_Resume_Rotate_Sleep(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch play video app
            2. Play video and check
            3. rotate screen and press power key to suspeng/resume
        """
        self.videoPlayControlProcess("test_video_playback_031")

    def testVideo_Playback_H264_1080P_Ts_Pause_Resume_Rotate_Sleep(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch play video app
            2. Play video and check
            3. rotate screen and press power key to suspeng/resume
        """
        self.videoPlayControlProcess("test_API_video_playback_146")

    def testVideo_Playback_H264_1080_Mkv_Local(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch play video app
            2. Play video and check
            3. rotate screen and press power key to suspeng/resume
        """
        self.videoPlayControlProcess("test_API_video_playback_154")

    def testVideo_Playback_HEAAC_V2_Mp4(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch play video app
            2. Play video and check
        """
        self.testVideoPlayBackAudio("test_API_video_playback_152")

    def testVideo_Playback_Mp3_Mkv(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch play video app
            2. Play video and check
        """
        self.testVideoPlayBackAudio("test_video_playback_110")

    def testVideo_Playback_MPEG4_SPL2_QVGA_15Fps(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch play video app
            2. Play video and check
            3. rotate screen and press power key to suspeng/resume
        """
        self.videoPlayControlProcess("test_video_playback_015")

    def testVideo_Playback_MPEG4_SP_VGA_30Fps(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch play video app
            2. Play video and check
            3. rotate screen and press power key to suspeng/resume
        """
        self.videoPlayControlProcess("test_video_playback_098")

    def testVideo_Playback_MPEG4_SP2_CIF_3gp(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch play video app
            2. Play video and check
            3. rotate screen and press power key to suspeng/resume
        """
        self.videoPlayControlProcess("test_video_playback_101")

    def testVideo_Playback_MPEG4_SP1_CIF(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Launch play video app
            2. Play video and check
            3. rotate screen and press power key to suspeng/resume
        """
        self.videoPlayControlProcess("test_API_video_playback_132")

    def testVideo_Decode_Second_Display_H264_1080P(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Connect two screen.
            2. Play video on the second screen.
        """
        self.videoDecodeSecondDisplay("test_API_video_playback_147")

    def testVideo_Decode_Second_Display_VP8_720P(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Connect two screen.
            2. Play video on the second screen.
        """
        self.videoDecodeSecondDisplay("test_API_video_playback_148")

    def testVideo_Decode_Second_Display_MPEG4_480P(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Connect two screen.
            2. Play video on the second screen.
        """
        self.videoDecodeSecondDisplay("test_video_playback_017")

    def testVideo_Decode_Second_Display_H264_720P_60fps(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Connect two screen.
            2. Play video on the second screen.
        """
        self.videoDecodeSecondDisplay("test_API_video_playback_149")

    def testVideo_Decode_Second_Display_H265_480x360(self):
        """
            This test used to test Video playback
            The test case spec is following:
            1. Connect two screen.
            2. Play video on the second screen.
        """
        self.videoDecodeSecondDisplay("test_API_video_playback_150")