# coding: utf-8
import os
import time
import re
import sys
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.multimedia.multimedia_drm_helper import MultiMediaDRMHelper, MultiMediaVideoQualityHelper
from testlib.systemui.systemui_impl import SystemUI
from testlib.util.common import g_common_obj as adb

from testlib.util.log import Logger
logger = Logger.getlogger()

class ExoPlayerTest(UIATestBase):
    """
    @summary: Test Widevine
    """
    
    config = TestConfig()

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(ExoPlayerTest, self).setUp()
        self.d = g_common_obj.get_device()
        self._test_name = __name__
        self.playback_state_str = "playbackState"
        self.video_button_index_dict={"Paused":1, "Play":1, "Forward":2, "Backward":0 }
        
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(ExoPlayerTest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()

    def appPrepare(self, model=1):
        self.cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_drm.conf')
        self.cfg = self.config.read(self.cfg_file, self.case_name) # read config file

        self.multimedia_setting = MultiMediaSetting(self.cfg_file)
        self.multimedia_drm_helper = MultiMediaDRMHelper()
        self.multimedia_video_quality_helper = MultiMediaVideoQualityHelper()
        
        self.multimedia_setting.install_apk("exoplayer_apk")#install exoplayer apk
        self.exoplayer_package_name, self.exoplayer_activity_name = self.multimedia_setting.get_package_and_activity_name("exoplayer_apk")#get app info

#         g_common_obj.set_vertical_screen()
        g_common_obj.stop_app_am(self.exoplayer_package_name)
        g_common_obj.adb_cmd_capture_msg("input keyevent 82") #Unlock screen

    def clickScreen(self):
        self.multimedia_setting.clickScreen()

    def launchEXOPlayerApp(self):
        SystemUI().unlock_screen()
#         g_common_obj.stop_app_am(self.widevine_package_name)
        g_common_obj.launch_app_am(self.exoplayer_package_name, self.exoplayer_activity_name)
        time.sleep(5)
        for _ in range(5):
            if self.d(textContains="ExoPlayer Demo").exists:
                return
            time.sleep(5)
        assert self.d(textContains="ExoPlayer Demo").exists, "launch Exoplayer App failed!"

    def get_playback_state_pattern(self):
        if "playback_state_pattern" not in dir(self):
            GET_PLAYBACKSTATE_PATTERN = "playbackState=(.*)"
            self.playback_state_pattern = re.compile(GET_PLAYBACKSTATE_PATTERN)
        return self.playback_state_pattern

    def getPlaybackState(self):
        playback_state_str=self.playback_state_str
        for _ in range(3):
            if not self.d(textContains=playback_state_str).exists:
                self.clickScreen()
            time.sleep(1)
        assert self.d(textContains=playback_state_str).exists, "playbackState not exist! please check it!"
        status = self.d(textContains=playback_state_str).text
        logger.debug("all_status=%s" % status)
        playback_state_pattern = self.get_playback_state_pattern()
        playback_state = playback_state_pattern.findall(status)
        logger.debug("playback_state=%s" % str(playback_state))
        return playback_state[0]

    def checkPlaybackState(self):
        for _ in range(5):
            playback_state = self.getPlaybackState()
            if playback_state == "ready":
                return 0
            time.sleep(5)
        assert 0, "Error! playback_state=%s" % playback_state

    def clickVideoButton(self, t_str):
        logger.debug("clickVideoButton---t_str=%s" % t_str)
        assert t_str in self.video_button_index_dict, "%s not in video_button_index_dict!" % t_str
        t_index = self.video_button_index_dict[t_str]
        if not self.d(className="android.widget.ImageButton", index=t_index).exists:
            self.clickScreen()
        self.d(className="android.widget.ImageButton", index=t_index).click()

    def wait_boot_completed(self, timeout=1000):
        ''' wait Android boot_completed
    
        args: timeout -- optional timeout in second, default 180s
        '''
        count = 0
        sleep_time = 5
        while count < timeout:
            prop_val = adb.adb_cmd_capture_msg('getprop sys.boot_completed')
            if '1' in prop_val:
                print 'boot_completed'
                return
            count += sleep_time
            time.sleep(sleep_time)
        raise Exception('%ds timeout waiting for boot_completed' % timeout)

    def clickRetryButton(self):
        retry_str = "Retry"
        if not self.d(text=retry_str).exists:
            self.clickScreen()
            time.sleep(3)
            if not self.d(text=retry_str).exists:
                self.clickVideoButton("Paused")
                time.sleep(1)
                self.clickVideoButton("Play")
                time.sleep(5)
        if self.d(text=retry_str).exists:
            logger.debug("Click retry button!")
            self.d(text=retry_str).click()
            time.sleep(1)
        else:
            logger.debug("Can't find retry button!")

    def clickRecentApp(self, app_name):
        self.multimedia_setting.click_recent_app(app_name)

    def checkVideoPlayback(self, t_time, expect_resolution=532):
        timeNow = time.time()
        if t_time >= 20:
            self.multimedia_video_quality_helper.checkSurfaceViewResolutionWithThread_start(expect_resolution)
        while time.time() - timeNow < t_time:
            playback_state = self.getPlaybackState()
            if playback_state == "end":
                logger.debug("Video playback finish!")
                break
            elif playback_state != "ready":
                logger.debug("checkVideoPlayback-- playback_state=%s" % playback_state)
                self.clickRetryButton()
            time.sleep(3)
        if t_time >= 20:
            assert self.multimedia_video_quality_helper.checkSurfaceViewResolutionWithThread_stop() == 1, "Looks like Video didn't reach HD(%d)..." % expect_resolution

    def exoplayerPlayBack(self, mode=""):
        if mode == "":
            mode = "WV: Secure video path required"
        elif mode == "HDCP":
            mode = "WV: HDCP + secure video path required"
        self.d(scrollable=True).scroll.vert.to(text=mode)
        self.d(text=mode).click()
        time.sleep(5)
        self.checkPlaybackState()

    def test_video_playback(self, t_time=60, mode="", expect_resolution=532):
        self.exoplayerPlayBack(mode)# play video with exoplayer app
        time.sleep(20)
        self.checkVideoPlayback(t_time, expect_resolution)# check play status of video

    def test_video_playback_with_pause(self, t_time=60):
        self.exoplayerPlayBack()# play video with exoplayer app
        time.sleep(20)
        self.checkVideoPlayback(20)
        self.clickVideoButton("Paused")#click paused button
        time.sleep(t_time)
        self.clickVideoButton("Play")#click play button
        self.checkVideoPlayback(20)# check play status of video

    def test_video_playback_with_control(self):
        self.exoplayerPlayBack()
        time.sleep(20)
        self.checkVideoPlayback(20)
        self.clickVideoButton("Paused")
        time.sleep(5)
        self.clickVideoButton("Play")
        time.sleep(5)
        self.clickVideoButton("Forward")
        time.sleep(5)
        self.clickVideoButton("Backward")
        time.sleep(10)

    def test_video_playback_with_repeatedly(self, click_list, t_time, mode=""):
        self.exoplayerPlayBack(mode)
        time.sleep(10)
        self.checkVideoPlayback(20)
        for i in range(t_time):
            logger.debug("clickVideoButton---%d time" % i)
            for t_str in click_list:
                self.clickVideoButton(t_str)
                time.sleep(2)
        time.sleep(5)

    def test_video_playback_with_background(self):
        self.exoplayerPlayBack()
        time.sleep(20)
        self.checkVideoPlayback(20)
        self.clickVideoButton("Paused")
        time.sleep(5)
        self.clickVideoButton("Play")
        time.sleep(5)
        self.d.press.home()
        time.sleep(3)
        self.clickRecentApp("ExoPlayer Demo")

    def test_video_playback_with_swipe(self, t_time, percent, mode=""):
        self.exoplayerPlayBack(mode)
        time.sleep(20)
        self.checkVideoPlayback(t_time)
        self.multimedia_setting.set_play_time(percent)
        self.checkVideoPlayback(20)

    def test_video_playback_with_sleep_mode(self, mode=""):
        self.exoplayerPlayBack(mode)
        time.sleep(20)
        self.checkVideoPlayback(10)
        self.d.press.power()
        time.sleep(5)
        self.d.press.power()
        SystemUI().unlock_screen()
        self.checkVideoPlayback(20)

    def test_video_playback_with_orientation(self, orientation_list, t_time, mode=""):
        self.exoplayerPlayBack(mode)
        time.sleep(10)
        
        self.checkVideoPlayback(20)
        for i in range(t_time):
            logger.debug("change orientation---%d time" % i)
            for t_str in orientation_list:
                self.d.orientation = t_str
                time.sleep(5)
        self.d.freeze_rotation(False)
        time.sleep(5)

    def test_video_playback_with_reboot(self, t_time):
        self.case_name = sys._getframe().f_back.f_code.co_name
        self.appPrepare()
        self.launchEXOPlayerApp()
        self.exoplayerPlayBack()
        time.sleep(20)
        self.checkVideoPlayback(10)
        for _ in range(t_time):
            adb.reboot_device()
            self.wait_boot_completed()
        time.sleep(20)
        self.lock = SystemUI()
        self.lock.unlock_screen()
        self.launchEXOPlayerApp()
        self.exoplayerPlayBack()
        time.sleep(20)
        self.checkVideoPlayback(10)
        self.d.press.back()
        self.d.press.home()

    def test_video_playback_with_check_S0I3(self):
        self.case_name = sys._getframe().f_back.f_code.co_name
        self.appPrepare()
        after_S0I3_str = self.multimedia_setting.getS0I3()
        self.launchEXOPlayerApp()
        self.exoplayerPlayBack()
        time.sleep(20)
        self.checkVideoPlayback(10)
        self.d.press.power()
        logger.debug("Step: sleep 800s, please wait...")
        time.sleep(800)
        logger.debug("Step: sleep 800s, complete!")
        self.d.press.power()
        SystemUI().unlock_screen()
        self.d.press.back()
        self.d.press.home()
        before_S0I3_str = self.multimedia_setting.getS0I3()
        assert after_S0I3_str != before_S0I3_str, "after_S0I3_str=%s, before_S0I3_str=%s" % (after_S0I3_str, before_S0I3_str)

    def test_video_playback_with_check_log(self, mode="", check_str="CopyBuffer"):
        self.multimedia_setting.clearLogs()
        self.multimedia_setting.checkLogs_start(check_str)
        self.exoplayerPlayBack(mode)
        time.sleep(20)
        self.checkVideoPlayback(20)
        self.multimedia_setting.checkLogs_end(check_str)

    def test_video_playback_with_pause_1hour(self,delay_time=3600):
        '''

        This test used to pause play 1 hours and resume
        test cases spec is following:
        1.pause palying video
        2.stop record video
        3.resume video
        4.start record the second video

        '''
        self.exoplayerPlayBack()
        time.sleep(20)
        self.clickVideoButton("Paused")
        self.multimedia_drm_helper.stopRecord()
        logger.debug("Step: sleep 1 hours, please wait...")
        time.sleep(delay_time)
        self.multimedia_drm_helper.startRecord(self.case_name+'_2')
        time.sleep(10)
        self.clickVideoButton("Paused")
        time.sleep(5)
        self.checkVideoPlayback(20)


    def exoplayer_main_test(self, sub_func_name="", *arg, **keywords):
        """
        This test used to test Exoplayer App
        The test case spec is following:
        1. Start record video
        2. do sub_func()
        3. Stop record video
        """
        self.case_name = sys._getframe().f_back.f_code.co_name
        if sub_func_name == "":
            sub_func_name = "%s_sub_func" % self.case_name
        logger.debug("case_name=%s" % self.case_name)
        logger.debug("exoplayer_main_test---sub_func_name=%s" % sub_func_name)
        self.appPrepare()
        try:
            self.multimedia_drm_helper.startRecord(self.case_name)#start record
            self.launchEXOPlayerApp()#launch exoplayer app
            
            logger.debug("Arbitrary parameter is %s" % str(arg))
            logger.debug("keywords parameter is %s" % str(keywords))
            getattr(self, sub_func_name)(*arg, **keywords)
            
            time.sleep(10)
            self.d.press.back()
            self.d.press.home()
            time.sleep(1)
            self.multimedia_drm_helper.stopRecord()#stop record
        except Exception as e:
            self.multimedia_drm_helper.stopRecord()
            assert 0, e
        
        assert 0, "Playback complete! Please check video!"
        logger.debug("Case %s is pass!" % self.case_name)

    def testWVModular_Exoplayer_Display(self):
        self.exoplayer_main_test("test_video_playback", 20)

    def testWVModular_Exoplayer_Video_Playback(self):
        self.exoplayer_main_test("test_video_playback", 20)

    def testWVModualr_Exoplayer_Audio_Video_Synchronization(self):
        self.exoplayer_main_test("test_video_playback", 20)

    def testWVModular_Exoplayer_Play(self):
        self.exoplayer_main_test("test_video_playback", 20)

    def testWVModular_Exoplayer_End(self):
        self.exoplayer_main_test("test_video_playback", 600)

    def testWVModular_Exoplayer_Secure_HD_MP4_H264(self):
        self.exoplayer_main_test("test_video_playback", 20, "WV: Secure HD (MP4,H264)", 500)

    def testWVModular_ExoPlayer_Control(self):
        self.exoplayer_main_test("test_video_playback_with_control")

    def testWVModular_Exoplayer_Pause_Play_Repeatedly(self):
        self.exoplayer_main_test("test_video_playback_with_repeatedly", ["Paused", "Play"], 20)

    def testWVModular_Exoplayer_Forward_Rewind_Repeatedly(self):
        self.exoplayer_main_test("test_video_playback_with_repeatedly", ["Forward", "Backward"], 20)

    def testWVModular_Exoplayer_Forward_Rewind_Paused(self):
        self.exoplayer_main_test("test_video_playback_with_repeatedly", ["Paused", "Forward", "Backward", "Play"], 1)

    def testWVModular_Exoplayer_Forward_Rewind(self):
        self.exoplayer_main_test("test_video_playback_with_repeatedly", ["Forward", "Backward"], 1)

    def testWVModular_Exoplayer_Rewind_Less_10s(self):
        self.exoplayer_main_test("test_video_playback_with_repeatedly", ["Forward", "Backward"], 1, "HDCP")

    def testWVModular_Exoplayer_Pause_One_Minute(self):
        self.exoplayer_main_test("test_video_playback_with_pause")

    def testWVModular_Exoplayer_Resume_Background(self):
        self.exoplayer_main_test("test_video_playback_with_background")

    def testRewind_WVModular_Exoplayer(self):
        self.exoplayer_main_test("test_video_playback_with_swipe", 10, 0.01)

    def testWVModular_Exoplayer_Forward_Till_End(self):
        self.exoplayer_main_test("test_video_playback_with_swipe", 10, 0.98)

    def testWVModular_Exoplayer_Sleep_Paused(self):
        self.exoplayer_main_test("test_video_playback_with_sleep_mode")

    def testWVModular_Exoplayer_Power_Button(self):
        self.exoplayer_main_test("test_video_playback_with_sleep_mode", "HDCP")

    def testWVModular_Exoplayer_Changeorientation(self):
        self.exoplayer_main_test("test_video_playback_with_orientation", ["l", "r", "n", "l"], 1, "HDCP")

    def testWVModular_Verify_WV10_Copy_Buffer(self):
        self.exoplayer_main_test("test_video_playback_with_check_log", "HDCP", "CopyBuffer")

    def testWVModular_HDCP_Capability(self):
        self.exoplayer_main_test("test_video_playback_with_check_log", "HDCP", "GetHDCPCapability")

    def testWVClassic_Reboot_Multi_Times(self):
        self.test_video_playback_with_reboot(20)

    def testWVModular_Exoplayer_Resume_S0i3(self):
        self.test_video_playback_with_check_S0I3()
   
    def testWVModular_Resume_1hour_Paused(self):
        self.exoplayer_main_test("test_video_playback_with_pause_1hour",delay_time=3600)
