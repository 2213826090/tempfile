# coding: utf-8
import os
from testlib.photos.mum_photos_impl import PhotosImpl
from testlib.multimedia.testcasebase import TestCaseBase
from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_setting import MultiMediaSetting, MultiMediaHandle, MultiMediaBasicTestCase
from testlib.systemui.systemui_impl import SystemUI
from testlib.util.common import g_common_obj as adb
import time


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
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]
        print "[Setup]: %s" % self._test_name
        g_common_obj.stop_app_am("videoplayer.app.instrument.otc.intel.com.otcvideoplayer")
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
        g_common_obj.stop_app_am("videoplayer.app.instrument.otc.intel.com.otcvideoplayer")
        time.sleep(3)

    def appPrepare(self, case_name, model=1):
        self.cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_video.conf')
        self.video = PhotosImpl(\
            self.config.read(self.cfg_file, case_name))
        g_common_obj.adb_cmd_capture_msg(" rm -rf /sdcard/DCIM/Camera/*")
        
        self.multimedia_handle = MultiMediaHandle()
        self.multimedia_setting = MultiMediaSetting(self.cfg_file)
        self.multimedia_setting.clearFolder(self.video.cfg.get("remove_video"))
        time.sleep(2)
        if model == 1:
            self.push_path = self.multimedia_setting.push_file(self.video.cfg.get("push_video"), self.video.cfg.get("datapath"))
            
        self.multimedia_setting.install_apk("video_apk")
        self.multimedia_setting.install_apk("alarm_apk")
        
        self.video.set_orientation_n()
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def getNormalWifi(self):
        conf = self.config.read('tests.tablet.dut_init.conf','wifisetting')
        ssid = conf.get("ssid")
        passwd = conf.get("passwd")
        security = conf.get("security")
        print ssid, passwd, security
        return ssid, passwd, security

    def getRTPWifi(self):
        rtp_wifisetting = PhotosImpl(self.config.read(self.cfg_file, "rtp_wifisetting"))
        ssid = rtp_wifisetting.cfg.get("ssid")
        passwd = rtp_wifisetting.cfg.get("passwd")
        security = rtp_wifisetting.cfg.get("security")
        print ssid, passwd, security
        return ssid, passwd, security

    def launchRecordAPP(self):
        SystemUI().unlock_screen()
        for _ in range(3):
            g_common_obj.launch_app_am("com.intel.vpg.tool", \
                                   "com.intel.vpg.tool.ConfActivity")
            time.sleep(3)
            if self.d(textContains="Vpg Media Tool").exists:
                return
        assert self.d(textContains="Vpg Media Tool").exists, "launch record app failed!"

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

    def clickRecordButton(self):
        self.d(className="android.widget.Button").click()

    def deleteCameraRecordFile(self):
        cmd = "shell rm /storage/emulated/legacy/Android/data/com.example.android.camera2video/files/video.mp4"
        g_common_obj.adb_cmd_common(cmd)

    def checkCameraRecordFile(self, flag=1):
        cmd = "shell ls /storage/emulated/legacy/Android/data/com.example.android.camera2video/files/video.mp4"
        result = g_common_obj.adb_cmd_common(cmd)
        assert ("No such file or directory" not in result.stdout.read()) == flag, "Record file status error! Flag=%s" % (flag)

    def enterBrowseVideo(self, path):
        self.d(className="android.widget.ImageButton").click()
        time.sleep(1)
        self.d(text="Browse Video").click()
        for _ in range(10):
            if self.d(textContains="Open").exists:
                break
            time.sleep(1)
        time.sleep(3)
        self.d.press.back()
        time.sleep(1)
        if self.multimedia_setting.getcheckIputMethod() != '':
            self.d.press.back()
        self.d(className="android.widget.EditText").set_text(path)
        time.sleep(1)
        if self.multimedia_setting.getcheckIputMethod() != '':
            self.d.press.back()
        self.d(text="OK").click()
        for _ in range(5):
            if self.d(text=path).exists:
                break
            time.sleep(5)
        assert self.d(text=path).exists, "can't enter path:%s" % path

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

    def checkVideoPlayBack(self, s=60):
        return self.multimedia_handle.checkVideoPlayBack(s)

    def checkVideoPlayBackWithComparePicture(self, stoptime, bigfileskiptime=0):
        return self.multimedia_handle.checkVideoPlayBackWithComparePicture(stoptime, bigfileskiptime)

    def checkVideoPlayBackComplete(self, s=900):
        return self.multimedia_handle.checkVideoPlayBackComplete(s)

    def checkVideoFileExist(self, file_name):
        assert self.d(text=file_name).exists, file_name + "file not find!"

    def deleteVideoFile(self, file_name):
        assert self.d(text=file_name).exists, file_name + "file not find!"
        self.d(text=file_name).long_click()
        time.sleep(1)
        self.d(text="Delete").click()
        time.sleep(1)
        self.d(text="Yes").click()
        time.sleep(1)
        assert not self.d(text=file_name).exists, file_name + "file delete failed!"

    def checkVideoDetail(self, file_name):
        assert self.d(text=file_name).exists, file_name + "file not find!"
        self.d(text=file_name).long_click()
        time.sleep(1)
        self.d(text="Detail").click()
        time.sleep(1)
        assert self.d(textContains=file_name).exists, file_name + "check detail failed!"

    def renameVideoFile(self, file_name, new_name):
        assert self.d(text=file_name).exists, file_name + "file not find!"
        self.d(text=file_name).long_click()
        time.sleep(1)
        self.d(text="Rename").click()
        for _ in range(10):
            if self.d(textContains="Open").exists:
                break
            time.sleep(1)
        time.sleep(3)
        if self.multimedia_setting.getcheckIputMethod() != '':
            self.d.press.back()
        self.d(className="android.widget.EditText").set_text(new_name)
        time.sleep(1)
        if self.multimedia_setting.getcheckIputMethod() != '':
            self.d.press.back()
        self.d(text="OK").click()
        time.sleep(1)
        assert self.d(textContains=new_name).exists, file_name + "check detail failed!"

    def interuptionVideoPlayBack(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        time.sleep(1)
        hardware = self.multimedia_setting.get_paltform_hardware()
        if 'bxtp_abl' or 'gordon_peak' in hardware:
            #  For BXT use relay card press power to sleep
            self.multimedia_setting.pressPowerKey()
            time.sleep(10)
            self.multimedia_setting.pressPowerKey()
        # elif 'gordon_peak' in hardware:
        #     self.logger.error("Case failed due OAM-48191: adb lost in host when resume from S3")
        #     assert False, "case " + str(case_name) + " is Fail"
        else:
            self.d.press.power()
            time.sleep(2)
            self.d.press.power()
        self.lock = SystemUI()
        self.lock.unlock_screen()
        time.sleep(1)
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def interuptionStreamingVideoPlayBack(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        time.sleep(1)
        hardware = self.multimedia_setting.get_paltform_hardware()
        if 'bxtp_abl' or 'gordon_peak' in hardware:
            #  For BXT use relay card press power to sleep
            self.multimedia_setting.pressPowerKey()
            time.sleep(10)
            self.multimedia_setting.pressPowerKey()
        # elif 'gordon_peak' in hardware:
        #     self.logger.error("Case failed due OAM-48191: adb lost in host when resume from S3")
        #     assert False, "case " + str(case_name) + " is Fail"
        else:
            self.d.press.power()
            time.sleep(2)
            self.d.press.power()
        self.lock = SystemUI()
        self.lock.unlock_screen()
        time.sleep(1)
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBack(self, case_name):
        MultiMediaBasicTestCase().testVideoPlayBack(case_name)

    def testStreamingVideoPlayBack(self, case_name):
        MultiMediaBasicTestCase().testStreamingVideoPlayBack(case_name)

    def testRTPStreamingVideoPlayBack(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testStreamingVideoPlayBackWithPowerOff(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        adb.reboot_device()
        self.wait_boot_completed()
        time.sleep(20)
        self.lock = SystemUI()
        self.lock.unlock_screen()
        print "case " + str(case_name) + " is pass"

    def streamingVideoPlayControlProcess(self, case_name):
        MultiMediaBasicTestCase().streamingVideoPlayControlProcessLong(case_name)

    def lowAndHighFramestreamingVideoPlayControlProcess(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        time.sleep(10)
        self.checkVideoPlayBack()
        self.d.press.volume_up()
        self.checkVideoPlayBack()
        self.d.press.volume_down()
        self.checkVideoPlayBack()
        self.multimedia_setting.set_play_time(0.5)
        self.checkVideoPlayBack()
        self.d(resourceId="android:id/rew").click()
        self.checkVideoPlayBack()
#         self.d(resourceId="android:id/ffwd").click()
#         self.checkVideoPlayBack()
        self.d(resourceId="android:id/pause").click()
        self.checkVideoPlayBack()
        self.d.press.home()
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack(self.video.cfg.get("video_path_2"))
        time.sleep(10)
        self.checkVideoPlayBack()
        self.d.press.volume_up()
        self.checkVideoPlayBack()
        self.d.press.volume_down()
        self.checkVideoPlayBack()
        self.multimedia_setting.set_play_time(0.5)
        self.checkVideoPlayBack()
        self.d(resourceId="android:id/rew").click()
        self.checkVideoPlayBack()
#         self.d(resourceId="android:id/ffwd").click()
#         self.checkVideoPlayBack()
        self.d(resourceId="android:id/pause").click()
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testVideoDurationThumbnail(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.enterBrowseVideo(os.path.split(self.push_path)[0])
        time.sleep(2)
        file_name = os.path.split(self.video.cfg.get("push_video"))[1].strip("\"")
        self.checkVideoFileExist(file_name)
        self.d(text=file_name).click()
        time.sleep(1)
        self.checkVideoPlayBack()
        self.d.press.back()
        if not self.d(text=os.path.split(self.push_path)[0]).exists:
            self.d.press.back()
        time.sleep(1)
        self.checkVideoFileExist(file_name)
        print "case " + str(case_name) + " is pass"

    def testStreamingVideoDurationThumbnail(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        self.d.press.back()
        assert self.d(text="OtcVideoPlayer").exists, "launch video app failed!"
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

    def interuptionStreamingVideoPlayBackSwitchHome(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        time.sleep(1)
        self.d.press.home()
        self.multimedia_setting.click_recent_app("OtcVideoPlayer")
        time.sleep(1)
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def deleteVideoFileCase(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_setting.push_file(self.video.cfg.get("push_video_2"), self.video.cfg.get("datapath_2"))
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.enterBrowseVideo(os.path.split(self.push_path)[0])
        time.sleep(2)
        self.deleteVideoFile(os.path.split(self.video.cfg.get("push_video"))[1].strip("\""))
        self.deleteVideoFile(os.path.split(self.video.cfg.get("push_video_2"))[1].strip("\""))
        print "case " + str(case_name) + " is pass"

    def CheckVideoFileInThumbnail(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_setting.push_file(self.video.cfg.get("push_video_2"), self.video.cfg.get("datapath"))
        self.multimedia_setting.push_file(self.video.cfg.get("push_video_3"), self.video.cfg.get("datapath"))
        self.multimedia_setting.push_file(self.video.cfg.get("push_video_4"), self.video.cfg.get("datapath"))
        self.multimedia_setting.push_file(self.video.cfg.get("push_video_5"), self.video.cfg.get("datapath"))
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.enterBrowseVideo(os.path.split(self.push_path)[0])
        time.sleep(2)
        self.checkVideoFileExist(os.path.split(self.video.cfg.get("push_video"))[1].strip("\""))
        self.checkVideoFileExist(os.path.split(self.video.cfg.get("push_video_2"))[1].strip("\""))
        self.checkVideoFileExist(os.path.split(self.video.cfg.get("push_video_3"))[1].strip("\""))
        self.checkVideoFileExist(os.path.split(self.video.cfg.get("push_video_4"))[1].strip("\""))
        self.checkVideoFileExist(os.path.split(self.video.cfg.get("push_video_5"))[1].strip("\""))
        print "case " + str(case_name) + " is pass"

    def CheckVideoFileInformationInThumbnail(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.enterBrowseVideo(os.path.split(self.push_path)[0])
        time.sleep(2)
        self.checkVideoDetail(os.path.split(self.video.cfg.get("push_video"))[1].strip("\""))
        print "case " + str(case_name) + " is pass"

    def RenameVideoFileInThumbnail(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.enterBrowseVideo(os.path.split(self.push_path)[0])
        time.sleep(2)
        self.renameVideoFile(os.path.split(self.video.cfg.get("push_video"))[1].strip("\""), self.video.cfg.get("new_name"))
        print "case " + str(case_name) + " is pass"

    def videoPlayBackWithAlarm(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_setting.launchAlarmAPP()
        self.multimedia_setting.setAlarmTime(40)
        self.multimedia_handle.launchVideoApp()
        time.sleep(1)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        time.sleep(1)
        self.multimedia_setting.waitAlarmTriiggered(60, "Snooze")
        self.multimedia_setting.setAlarmTime(30)
        self.multimedia_setting.click_recent_app("OtcVideoPlayer")
        self.checkVideoPlayBack()
        self.multimedia_setting.waitAlarmTriiggered(50, "Dismiss")
        self.multimedia_setting.click_recent_app("OtcVideoPlayer")
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def streamingVideoPlayBackWithAlarm(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_setting.launchAlarmAPP()
        self.multimedia_setting.setAlarmTime(60)
        self.multimedia_handle.launchVideoApp()
        time.sleep(1)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        time.sleep(1)
        self.multimedia_setting.waitAlarmTriiggered(80, "Snooze")
        self.multimedia_setting.setAlarmTime(60)
        self.multimedia_setting.click_recent_app("OtcVideoPlayer")
        self.checkVideoPlayBack()
        self.multimedia_setting.waitAlarmTriiggered(80, "Dismiss")
        self.multimedia_setting.click_recent_app("OtcVideoPlayer")
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testVideoPlaybackRotation(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        self.d.click(self.x/2,self.y/2)
        self.setRotation("Landscape View")
        self.checkVideoPlayBack()
        self.d.click(self.x/2,self.y/2)
        self.setRotation("Portrait View")
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testStreamingVideoPlaybackRotation(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        self.d.click(self.x/2,self.y/2)
        self.setRotation("Landscape View")
        self.checkVideoPlayBack()
        self.d.click(self.x/2,self.y/2)
        self.setRotation("Portrait View")
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testStreamingVideoPlayBackWithDownloadFile(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        self.push_path = self.multimedia_setting.push_file(self.video.cfg.get("push_video"), self.video.cfg.get("datapath"))
        time.sleep(3)
        self.checkVideoPlayBack()
        self.d.press.home()
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def cameraRecordWithAlarm(self, case_name):
        print "run case is " + str(case_name)
        self.deleteCameraRecordFile()
        self.checkCameraRecordFile(0)
        self.multimedia_setting.launchAlarmAPP()
        self.multimedia_setting.setAlarmTime(30)
        self.launchRecordAPP()
        time.sleep(1)
        self.clickRecordButton()
        self.multimedia_setting.waitAlarmTriiggered(50, "Dismiss")
        self.multimedia_setting.click_recent_app("Camera2Video")
        time.sleep(1)
        self.clickRecordButton()
        self.checkCameraRecordFile()
        print "case " + str(case_name) + " is pass"

    def videoPlayBackLonglasting(self, case_name, bigfileskiptime=0):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        lasting_time = int(self.video.cfg.get("lasting_time"))
        push_folder = os.path.split(os.path.split(self.push_path)[0])[-1]
        self.video.launchPhotos(push_folder)
        time.sleep(2)
        self.multimedia_handle.checkVideoPlayBackWithPhotoApp(lasting_time, bigfileskiptime, 1)
        print "case " + str(case_name) + " is pass"

    def testPlaying_streaming_Music_Back(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Press back
        """
        self.testStreamingVideoDurationThumbnail("test_API_video_playback_012")

    def testSHALL_support_H264_video_stream_no_frame_rate_limitation(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Start/Stop
        """
        self.lowAndHighFramestreamingVideoPlayControlProcess("test_API_video_playback_026")

    def testHTML5_video_playback_MP4_H264_BP(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.streamingVideoPlayControlProcess("test_API_video_playback_028")

    def testPFT_7098_VPG_HW_acceleratiof_HTML5_video_tag_H264(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_029")

    def testHTML5_video_playback_MP4_H264_HP(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.streamingVideoPlayControlProcess("test_API_video_playback_037")

    def testLongTime_HTML5_video_playback_MP4_H264(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.streamingVideoPlayControlProcess("test_API_video_playback_042")

    def testPseudo_streaming_clock_alarm(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Alarm expired
        4. Snooze the alarm
        5. Dismiss the alarm
        """
        self.streamingVideoPlayBackWithAlarm("test_API_video_playback_047")

    def testPseudo_streaming_HTTP_H264_640x480_60fps_NoAudio_mkv(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_048")

    def testPseudo_streaming_HTTP_H264_L4_0_HP_720x576_30fps_AAC_160kbps_48KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_049")

    def testPseudo_streaming_HTTP_VP8_176x144_15fps_334Kbps_NoAudio_webm(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_050")

    def testPlaying_streaming_Music_Power_Off(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBackWithPowerOff("test_API_video_playback_051")

    def testSHALL_support_H264_video_stream_with_no_size_limitation(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Start/Stop
        """
        self.lowAndHighFramestreamingVideoPlayControlProcess("test_API_video_playback_055")

    def testPFT_7102_Network_streaming_protocols_HTTP_S_Live_Streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_086")

    def testVideo_MPEG4_ASP_L1_720p_30fps_AAC_128kb_48KHz_RTP_streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_089")

    def testVideo_H263_CIF_30FPS_AAC_44KHz_127Kbps_RTP_streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_090")

    def testVideo_H264_f4v_MP_L3_1_640x480_30fps_AAC_44_1KHz_128kbps_over_WLAN(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_091")

    def testVideo_H264_L2_1_BP_CIF_30fps_AAC_Multimedia_56kb_48KHz_RTP_streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_092")

    def testPseudo_streaming_HTTP_H264_MP_320x240_15fps_211Kbps_NoAudio_3gp(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_093")

    def testPseudo_streaming_HTTP_VP8_CIF_25fps_Vorbis_44_1KHz_128kbps_webm_rotate_sleep_resume(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.streamingVideoPlayControlProcess("test_API_video_playback_094")

    def testPseudo_streaming_file_downloading_in_background(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBackWithDownloadFile("test_API_video_playback_103")

    def testPFT_7100_Network_streaming_protocols_RTSP_RTP_SDP(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testRTPStreamingVideoPlayBack("test_API_video_playback_109")

    def testLongTime_video_playback_H264_HP_L4_1_1080P_30fps_4_5Mbps_noAudio_mp4(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('Long-time Video playback: H264_HP_L4.1_1080P_30fps_4.5Mbps_na.mp4 (10 hours)')
        3. Former name: test_video_playback_long_lasting_mum_007
        """
        self.videoPlayBackLonglasting("mum_test_video_playback_long_lasting_007", 20)

    def testLongTime_video_playback_VP8_VGA_20fps_2Mbps_Vorbis_48KHz_128kbps_webm(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch photos
        2. Play video('Long-time Video playback: VP8_VGA_20fps_2Mbps_Vorbis_48KHz_128kbps.webm (10 hours)')
        3. Former name: test_video_playback_long_lasting_mum_008
        """
        self.videoPlayBackLonglasting("mum_test_video_playback_long_lasting_008", 20)