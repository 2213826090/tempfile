# coding: utf-8
import os
import time
import sys
import thread
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.multimedia.multimedia_setting import MultiMediaSetting, MultiMediaHandle
from testlib.multimedia.multimedia_drm_helper import MultiMediaDRMHelper, MultiMediaVideoQualityHelper
from testlib.systemui.systemui_impl import SystemUI
from testlib.util.common import g_common_obj as adb

from testlib.util.log import Logger
logger = Logger.getlogger()

class NetflixTest(UIATestBase):
    """
    @summary: Test Widevine
    """
    
    config = TestConfig()

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(NetflixTest, self).setUp()
        self.d = g_common_obj.get_device()
        self._test_name = __name__
        self.playback_state_str = "playbackState"
        self.video_button_widget_dict={"play":"com.netflix.mediaclient:id/player",
                                       "pause":"com.netflix.mediaclient:id/player",
                                       "back_30s":"com.netflix.mediaclient:id/skip_back",
                                       "current_time":"com.netflix.mediaclient:id/current_timeLabel",
                                       "remain_time":"com.netflix.mediaclient:id/label_duration",
                                       "seek_bar":"com.netflix.mediaclient:id/timeline",
                                       "volume_up":"volume_up",
                                       "volume_down":"volume_down",
                                       "back":"back"}
        
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(NetflixTest, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()

    def appPrepare(self, model=1):
        self.cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_drm.conf')
        self.cfg = self.config.read(self.cfg_file, self.case_name)

        self.multimedia_setting = MultiMediaSetting(self.cfg_file)
        self.multimedia_drm_helper = MultiMediaDRMHelper()
        self.multimedia_video_quality_helper = MultiMediaVideoQualityHelper()
        
        self.multimedia_setting.install_apk("netflix_apk")
        self.netflix_package_name, self.netflix_activity_name = self.multimedia_setting.get_package_and_activity_name("netflix_apk")

#         g_common_obj.set_vertical_screen()
        g_common_obj.stop_app_am(self.netflix_package_name)
        SystemUI().unlock_screen()

    def clickScreen(self):
        self.multimedia_setting.clickScreen()

    def registerNetflix(self):
        t_email = "amt.test2014@gmail.com"
        t_password = "amt.test"
        if self.d(description="Choose a Credential").exists:
            if self.d(textContains=t_email).exists:
                self.d(textContains=t_email).click()
                for _ in range(10):
                    if self.d(textContains="Kids").exists:
                        self.d(textContains="Kids").click()
                        time.sleep(10)
                        return
                    time.sleep(3)
            else:
                self.d(textContains="None of the above").click()
                time.sleep(2)
        else:
            self.d(textContains="Sign In").click()
            time.sleep(5)
        self.d(className="android.widget.EditText", resourceId="com.netflix.mediaclient:id/login_email").set_text(t_email)
        self.d(className="android.widget.EditText", resourceId="com.netflix.mediaclient:id/login_password").set_text(t_password)
        self.d(textContains="Sign In").click()
        time.sleep(10) #wait DUT sign in
            

    def launchNetflixApp(self):
        SystemUI().unlock_screen()
        for _ in range(2):
            g_common_obj.launch_app_am(self.netflix_package_name, self.netflix_activity_name)
            time.sleep(5)
            for _ in range(6):
                if self.d(description="Search").exists:
                    return
                if self.d(textContains="New version of application").exists:
                    time.sleep(2)
                    self.d(textContains="Cancel").click(2)
                if self.d(textContains="Sign In").exists or self.d(description="Choose a Credential").exists:
                    self.registerNetflix()
                time.sleep(5)
                if self.d(description="test").exists:
                    self.d(description="test").click(3)
                if self.d(textContains="Notifications may include newly-added movi").exists:
                    self.d(textContains="No Thanks").click(2)
        assert self.d(description="Search").exists, "launch Netflix App failed!"

    def clickNetflixMenu(self, menu_str="Action"):
        self.d(description="Navigate up").click() #menu button
        time.sleep(2)
        self.d(className="android.widget.ScrollView").scroll.vert.to(textContains=menu_str)
        self.d(textContains=menu_str).click()
        time.sleep(5)

    def get_widget(self, t_str):
        logger.debug("get_widget---t_str=%s" % t_str)
        assert t_str in self.video_button_widget_dict, "%s not in video_button_widget_dict!" % t_str
        t_resource_id = self.video_button_widget_dict[t_str]
        if t_resource_id == t_str:
            return t_resource_id
        else:
            return self.d(resourceId=t_resource_id)

    @staticmethod
    def setTimeToSec(time):
        time = time.split(":")
        i = 1
        temp = 0
        for s in time[::-1]:
            temp += int(s) * i
            i *= 60
        return int(temp)

    def get_widget_text(self, t_widget):
        try:
            self.s_t_text = t_widget.text
        except Exception as e:
            logger.debug("get_widget_text error:%s" % e)
            self.s_t_text = -2

    def click_widget(self, t_widget):
        try:
            t_widget.click()
            self.s_t_text = 0
        except Exception as e:
            logger.debug("click_widget error:%s" % e)
            self.s_t_text = -2
            assert 0, "click_widget error:%s" % e

    def get_widget_bounds(self, t_widget):
        try:
            bounds = t_widget.info["bounds"]
            progress_bar_bounds = {}
            progress_bar_bounds["y"] = bounds["top"] + (bounds["bottom"] - bounds["top"])/2
            progress_bar_bounds["x_start"] = bounds["left"] + 10
            progress_bar_bounds["x_end"] = bounds["right"] - 10
            self.s_t_text = progress_bar_bounds
        except Exception as e:
            logger.debug("get_widget_text error:%s" % e)
            self.s_t_text = -2

    def widget_operation_with_thread(self, t_str, t_operation):
        self.s_t_text = -1
        t_widget = self.get_widget(t_str)
        if t_operation == "get_text":
            thread.start_new_thread(self.get_widget_text, (t_widget, ))
        elif t_operation == "click":
            thread.start_new_thread(self.click_widget, (t_widget, ))
        elif t_operation == "get_bounds":
            thread.start_new_thread(self.get_widget_bounds, (t_widget, ))
        else:
            assert 0, "Error operation!"
        while self.s_t_text == -1:
            if not t_widget.exists:
                self.clickScreen()
            time.sleep(1)
        return self.s_t_text

    def click_button(self, t_str):
        logger.debug("click_button---t_str=%s" % t_str)
#         t_widget = self.get_widget(t_str)
        t_widget = self.video_button_widget_dict[t_str]
        if t_str != t_widget:
            self.widget_operation_with_thread(t_str, "click")
        else:
            getattr(self.d.press, t_widget)()

    def get_play_time_coordinate(self, percent):
        if "progress_bar_bounds" not in dir(self):
            self.progress_bar_bounds = self.widget_operation_with_thread("seek_bar", "get_bounds")
        x = self.progress_bar_bounds["x_start"] + (self.progress_bar_bounds["x_end"] - self.progress_bar_bounds["x_start"]) * percent
        x = int(x)
        y = self.progress_bar_bounds["y"]
        y = int(y)
        logger.debug("progress_bar_bounds=%s, percent=%s" % (self.progress_bar_bounds, percent))
        return x, y

    def set_play_time(self, percent=0.5):
        x, y = self.get_play_time_coordinate(percent)
        seek_bar_widget = self.get_widget("seek_bar")
        if not seek_bar_widget.exists:
            self.clickScreen()
        self.d.click(x,y)

    def set_play_time_with_swipe(self, percent):
        ct, tt = self.get_play_time()
        ct = ct + 2
        if ct > tt:
            ct = tt
        start_x, start_y = self.get_play_time_coordinate(ct / float(tt))
        end_x, end_y = self.get_play_time_coordinate(percent)
        self.d.swipe(start_x, start_y, end_x, end_y)

    def get_play_time(self, t_time=60):
        logger.debug("get_play_time start")
        timeNow = time.time()
        while time.time() - timeNow < t_time:
            current_time_widget = self.get_widget("current_time")
            if current_time_widget.exists:
                ct = self.widget_operation_with_thread("current_time", "get_text")
                rt = self.widget_operation_with_thread("remain_time", "get_text")
                ct = self.setTimeToSec(ct)
                rt = self.setTimeToSec(rt)
                logger.debug("ct---tt:%s, %s" % (ct, ct + rt))
                return ct, ct + rt
            else:
                logger.debug("%s times, don't find current time or total time!" % str(time.time() - timeNow))
                assert not self.d(textContains="error").exists, "Play error!"
                time.sleep(1)
                self.clickScreen()
        assert not self.d(textContains="OTC Alarm is triggered").exists, "OTC Alarm is triggered! wait time=%d" % (t_time) 
        assert 0, "Play error! playback timeout %d s, network problem." % (t_time)

    def checkVideoPlayback(self, t_time, skip_flag=0, expect_resolution=480):
        flag = 0
        timeNow = time.time()
        if t_time >= 20 and skip_flag == 0:
            self.multimedia_video_quality_helper.checkSurfaceViewResolutionWithThread_start(expect_resolution)
        while time.time() - timeNow < t_time:
            if flag == 1 or self.d(textContains="Loading").exists:
                time.sleep(2)
            elif self.get_widget("current_time").exists:
                logger.debug("%s times, find current time and total time!!" % str(time.time() - timeNow))
                flag = 1
                if skip_flag:
                    break
            else:
                logger.debug("%s times, don't find current time or total time!!" % str(time.time() - timeNow))
                assert not self.d(textContains="error").exists, "Play error!"
                time.sleep(1)
                self.clickScreen()
        if t_time >=20 and skip_flag == 0 :
            assert self.multimedia_video_quality_helper.checkSurfaceViewResolutionWithThread_stop() == 1, "Looks like Video didn't reach HD(%d)..." % expect_resolution
        assert not self.d(textContains="OTC Alarm is triggered").exists, "OTC Alarm is triggered! wait time=%d" % (time.time() - timeNow) 
        assert flag, "Play error! playback timeout %d s, network problem.." % (t_time)

    def netflixPlayBack(self, t_index=0):
        #instance
        self.d(className="android.widget.ImageView", index=t_index).click()
        time.sleep(3)
        for _ in range(10):
            if self.d(className="android.widget.ImageView").exists:
                break
            time.sleep(3)
        self.d(className="android.widget.ImageView").click()
        time.sleep(5)
        for _ in range(20):
            if not self.d(textContains="Loading").exists:
                break
            time.sleep(5)
        assert not self.d(textContains="Loading").exists, "Error! loading %d s" % (5*20)

    def netflixLongPlayBack(self):
        self.d(scrollable=True).scroll.vert.to(description="The Lost World: Jurassic Park")
        for _ in range(10):
            if self.d(description="The Lost World: Jurassic Park").exists:
                break
            time.sleep(3)
        self.d(description="The Lost World: Jurassic Park").click()
        self.d(description="Play Video The Lost World: Jurassic Park").click()
        time.sleep(5)
        for _ in range(20):
            if not self.d(textContains="Loading").exists:
                break
            time.sleep(5)
        assert not self.d(textContains="Loading").exists, "Error! loading %d s" % (5 * 20)

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

    def clickRecentApp(self, app_name):
        self.multimedia_setting.click_recent_app(app_name)

    def test_video_playback(self, t_time=60):
        self.clickNetflixMenu()#open netflix menu
        self.netflixPlayBack()#play video with netflix app
        self.checkVideoPlayback(t_time)# check play status of video

    def test_video_playback_with_back(self, t_time=1):
        self.clickNetflixMenu()
        self.netflixPlayBack()
        self.checkVideoPlayback(60)
        for _ in range(t_time):
            self.click_button("back")
            self.d(className="android.widget.ImageView").click()
            time.sleep(5)
            self.checkVideoPlayback(60, 1)
            self.checkVideoPlayback(30, 0, 384)

    def test_video_playback_with_app_change(self):
        self.multimedia_setting.install_apk("video_apk")
        self.clickNetflixMenu()
        self.netflixPlayBack()
        self.checkVideoPlayback(60, 1)
        self.multimedia_handle = MultiMediaHandle()
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.clickRecentApp("Netflix")
        self.d(className="android.widget.ImageView").click()
        time.sleep(5)
        self.checkVideoPlayback(60, 1)
        self.checkVideoPlayback(20)

    def test_video_playback_with_sleep_mode(self):
        self.clickNetflixMenu()
        self.netflixPlayBack()
        self.checkVideoPlayback(40, 1)
        ct, tt = self.get_play_time()
        if tt - ct <= 300:
            self.set_play_time(0.01)
            self.checkVideoPlayback(30, 1)
        self.click_button("pause")
        self.d.press.power()
        time.sleep(10)
        self.d.press.power()
        SystemUI().unlock_screen()
        time.sleep(5)
        self.click_button("play")
        self.checkVideoPlayback(20)

    def test_video_playback_with_repeatedly(self, click_list, t_time, step_skip_time=2, begin_check_time=30):
        self.clickNetflixMenu()
        self.netflixPlayBack()
        self.checkVideoPlayback(30, 1)
        ct, tt = self.get_play_time()
        logger.debug("ct, tt---%d, %d" % (ct, tt))
        if tt - ct <= 300:
            self.set_play_time(0.01)
            self.checkVideoPlayback(30, 1)
        for i in range(t_time):
            logger.debug("click_button---%d time" % i)
            for t_str in click_list:
                if t_str == "sleep":
                    time.sleep(5)
                elif "swipe" in t_str:
                    self.set_play_time_with_swipe(float(t_str.split(" ")[1]))
                    self.checkVideoPlayback(30, 1)
                else:
                    self.click_button(t_str)
                    time.sleep(step_skip_time)
        time.sleep(5)

    def test_video_playback_with_longtime(self,t_time=7200):
        '''
         This test used to test netflix app over 2 hours
         The case spec is following:
         1.select HD(or better) protected content clip
         2.Watch movices for about 2 hours
         3.Play Another Netflix movices for 10 minutes
         '''

        self.clickNetflixMenu('Home')
        self.netflixLongPlayBack()
        self.checkVideoPlayback(120)
        self.multimedia_drm_helper.stopRecord()
        time.sleep(5)
        logger.debug("Step 2: Watch movices for about 2 hours")
        self.set_play_time(0.01)
        ct, tt = self.get_play_time()
        while ct<= t_time:
            self.multimedia_drm_helper.startRecord(self.case_name + '_step2_' + str(ct/60) + '_mins')
            logger.debug("Current palytime is :%s seconds" %ct)
            self.checkVideoPlayback(1800)
            ct, tt = self.get_play_time()
            self.multimedia_drm_helper.stopRecord()
        logger.debug("Play time is %s seconds" % (tt-ct))
        logger.debug("Step 3: Play Another Netflix movices for 10 minutes")
        self.multimedia_drm_helper.startRecord(self.case_name + '_step3')
        for _ in range(2):
          self.d.press.back()
        #self.launchNetflixApp()
        self.clickNetflixMenu()
        self.netflixPlayBack(2)
        self.checkVideoPlayback(600)

    def test_video_playback_with_check_WVinfo(self,check_str='WVCdm'):
        '''
        This test used to test netflix app and check WVinfo
        The case spec is following:
        1.Play HD video via netflix
        2.Check its logs "adb logcat |grep WVCdm"
        '''
        self.multimedia_setting.clearLogs()
        self.launchNetflixApp()
        logger.debug("get WVCdm info:")
        self.multimedia_setting.checkLogs_start(check_str)
        self.clickNetflixMenu()  # open netflix menu
        self.netflixPlayBack()  # play video with netflix app
        self.checkVideoPlayback(120)  # check play status of video
        self.multimedia_setting.checkLogs_end(check_str)



    def netflix_main_test(self, sub_func_name="", *arg, **keywords):
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
        logger.debug("netflix_main_test---sub_func_name=%s" % sub_func_name)
        self.appPrepare()
        try:
            self.multimedia_drm_helper.startRecord(self.case_name)#start record
            self.launchNetflixApp()#launch netflix app
            
            logger.debug("Arbitrary parameter is %s" % str(arg))
            logger.debug("keywords parameter is %s" % str(keywords))
            getattr(self, sub_func_name)(*arg, **keywords)
            
            time.sleep(10)
            self.d.press.back()
            self.d.press.home()
            time.sleep(1)
            self.multimedia_drm_helper.stopRecord()#stop record
        except Exception as e:
            self.multimedia_drm_helper.stopRecord()#stop record
            assert 0, e
        
        assert 0, "Playback complete! Please check video!"
        logger.debug("Case %s is pass!" % self.case_name)

    def testWVModular_Netflix_HD(self):
        self.netflix_main_test("test_video_playback", 120)

    def testWVModular_NetFlix_SD(self):
        self.netflix_main_test("test_video_playback", 120)

    def testWVModular_Netflix_Menu(self):
        self.netflix_main_test("test_video_playback", 120)

    def testWVModular_Netflix_Pause(self):
        self.netflix_main_test("test_video_playback_with_repeatedly", ["pause", "play"], 2, 4)

    def testWVModular_NetFlix_Volume_Change(self):
        self.netflix_main_test("test_video_playback_with_repeatedly", ["volume_up"]*4 + ["sleep"]*2 +["volume_down"]*4 , 1, 1)

    def testWVModular_Netflix_Volume_Pause(self):
        self.netflix_main_test("test_video_playback_with_repeatedly", ["pause"] + ["volume_up"]*4 + ["play"] + ["sleep"]*2 + 
                                                                      ["pause"] + ["volume_down"]*4 + ["play"] + ["sleep"], 1, 1)

    def testWVModular_NetFlix_AV_Synced(self):
        self.netflix_main_test("test_video_playback_with_repeatedly", ["pause"] + ["volume_up"]*4 + ["play"] + ["sleep"]*2 + 
                                                                      ["pause"] + ["volume_down"]*4 + ["play"] + ["sleep"], 1, 1)

    def testWVModular_Netflix_Rewind_10(self):
        self.netflix_main_test("test_video_playback_with_repeatedly", ["back_30s"], 1, 20, 40)

    def testWVModular_NetFlix_Resume_Sleep(self):
        self.netflix_main_test("test_video_playback_with_sleep_mode")

    def testWVModular_NetFlix_Power_Button(self):
        self.netflix_main_test("test_video_playback_with_sleep_mode")

    def testWVModular_Netflix_Back(self):
        self.netflix_main_test("test_video_playback_with_back", 1)

    def testWVModular_Netflix_Back_Stress(self):
        self.netflix_main_test("test_video_playback_with_back", 10)

    def testWVModular_NetFlix_Apps_Change(self):
        self.netflix_main_test("test_video_playback_with_app_change")

    def testWVModular_Netflix_Scrub(self):
        self.netflix_main_test("test_video_playback_with_repeatedly", ["swipe 0.7"] + ["sleep"]*2 + ["swipe 0.2"], 1, 1)

    def testWVModualr_Netflix_HD_Control(self):
        self.netflix_main_test("test_video_playback_with_repeatedly", ["pause", "play", "swipe 0.7"] + ["sleep"]*2 + ["swipe 0.2"], 1, 1)


    def testWVModular_Netflix_HD_WVinfo(self):
        self.netflix_main_test("test_video_playback_with_check_WVinfo")

    def testWVModular_Netflix_Long_Playback(self):
        self.netflix_main_test("test_video_playback_with_longtime", 7200)