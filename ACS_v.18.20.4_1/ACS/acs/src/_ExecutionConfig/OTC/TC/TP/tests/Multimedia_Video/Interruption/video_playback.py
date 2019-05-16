# coding: utf-8
import os
import sys
from testlib.photos.mum_photos_impl import PhotosImpl
from testlib.multimedia.testcasebase import TestCaseBase
from testlib.util.common import g_common_obj
from testlib.multimedia.multimedia_setting import MultiMediaSetting, MultiMediaHandle, MultiMediaBasicTestCase
from testlib.multimedia.multimedia_camera_helper import MultiMediaCameraHelper
from testlib.systemui.systemui_impl import SystemUI
from testlib.util.common import g_common_obj as adb
from testlib.wifi.wifisetting_impl import WifiSettingImpl
from testlib.autodetect.autodetect_impl import AutodetectImpl
from testlib.audio.audio_impl import AudioImpl
from testlib.camera.CameraCommon import CameraCommon
import time
import datetime
import re
import serial
import serial.tools.list_ports
from testlib.multimedia.multimedia_log import MultimediaLogger
logger = MultimediaLogger.instance()

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
        self.serial = self.d.server.adb.device_serial()
        self.tag = "[Interruption] "
        logger.debug(self.tag + 'run case name is ' + sys._getframe().f_back.f_code.co_name)
        self.camera_common = CameraCommon()


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

    def launchRecordAPP(self):
        SystemUI().unlock_screen()
        for _ in range(3):
            g_common_obj.launch_app_am("com.intel.vpg.tool", \
                                   "com.intel.vpg.tool.ConfActivity")
            time.sleep(3)
            if self.d(textContains="Vpg Media Tool").exists:
                return
        assert self.d(textContains="Vpg Media Tool").exists, "launch record app failed!"

    def clickRecordButton(self, record_file_name):
        self.d(className="android.widget.Spinner").click()
        self.d(text="VIDEO RECORDER").click()
        self.d(className="android.widget.EditText").set_text(record_file_name)
        self.d(className="android.widget.ScrollView").scroll.to(text="Apply Configuration")
        self.d(text="Apply Configuration").click()
        self.d.click(self.x/2,self.y/2)

    def deleteCameraRecordFile(self, file_name):
        cmd = "shell rm /sdcard/Pictures/VpgMediaTool/%s.mp4" % (file_name)
        g_common_obj.adb_cmd_common(cmd)

    def checkCameraRecordFile(self, file_name, flag=1):
        cmd = "shell ls /sdcard/Pictures/VpgMediaTool/%s.mp4" % (file_name)
        result = g_common_obj.adb_cmd_common(cmd)
        print result
        assert ("No such file or directory" not in result) == flag, "Record file status error! result=%s" % (result)

    def enterBrowseVideo(self, path):
        self.d(className="android.widget.ImageButton").click()
        time.sleep(1)
        self.d(text="Browse Video").click()
        for _ in range(10):
            if self.d(textContains="Open").exists:
                break
            time.sleep(1)
        time.sleep(3)
        if self.multimedia_setting.getcheckIputMethod() != '':
            self.d.press.back()
        time.sleep(1)
        self.d(className="android.widget.EditText").set_text(path)
        time.sleep(1)
        if self.multimedia_setting.getcheckIputMethod() != '':
            self.d.press.back()
        self.d(text="OK").click()
        # for _ in range(20):
        #     if self.d(text=path).exists:
        #         break
        #     time.sleep(5)
        logger.debug("wait for files display")
        self.d(text=path).wait.exists(timeout=180000)
        logger.debug("media files have displayed")
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
        time.sleep(2)
        self.d(className="android.widget.ImageButton").click()
        time.sleep(2)
        self.d(text=mode).click()
        time.sleep(1)

    def wait_boot_completed(self, timeout=500):
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

    def checkVideoNotSupport(self, s=60):
        for _ in range(s):
            if self.d(text="Can't play this video.").exists or self.d(textContains="Error:").exists:
                logger.debug("success,video play failed")
                return
            assert not self.d(resourceId="android:id/time_current").exists and not self.d(resourceId="android:id/time").exists, "error!can play it!"
            time.sleep(1)
            self.d.click(self.x/2,self.y/2)

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
        return self.multimedia_handle.checkVideoPlayBackComplete(s)

    def checkBatteryLevel(self):
        cmd = "shell dumpsys battery | grep level"
        result = g_common_obj.adb_cmd_common(cmd)
        print result
        return int(result.strip("level: "))

    def getSerialPort(self):
        port_list = list(serial.tools.list_ports.comports())
        assert len(port_list) > 0,"The Serial port can't find!"
        port_serial = 0
        for port in port_list:
            print port
            if "xhci_hcd xHCI Host Controller" in port[1]:
                port_serial = port[0]
        assert port_serial != 0, "can't find USB Serial Port!!"
#         port_serial = "/dev/ttyUSB4"
        ser = serial.Serial(port_serial,9600,timeout = 60)
        print "check which port was really used >",ser.name
        return ser

    def useSerialPort(self, t_str, ser=""):
        if ser == "":
            ser = self.ser
        
        if t_str == "press down":
            ser.write('\xFE\x05\x00\x00\xFF\x00\x98\x35')
            time.sleep(2)
        elif t_str == "release up":
            ser.write('\xFE\x05\x00\x00\x00\x00\xD9\xC5')
            time.sleep(2)

    def setSleepTime(self, t_time):
        logger.debug("interruption : set sleep time to :" + t_time)
        g_common_obj.adb_cmd_capture_msg("am start -S com.android.settings/.Settings")
        time.sleep(1)
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.vert.to(textContains="Display")
        self.d(text="Display").click()
        self.d(text="Sleep").click()
        time.sleep(1)
        self.d(text=t_time).click()

    def checkVideoFileExist(self, file_name):
        assert self.d(text=file_name).exists, file_name + "file not find!"

    def deleteVideoFile(self, file_name):
        assert self.d(text=file_name).exists, file_name + "file not find!"
        self.d(text=file_name).long_click()
        time.sleep(1)
        self.d(textContains="Delete").click()
        time.sleep(1)
        self.d(textContains="Yes").click()
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
        self.d(textContains="OK").click()
        time.sleep(1)
        assert self.d(textContains=new_name).exists, file_name + "check detail failed!"

    def getCpuConsumption(self):
        cmd = "shell top -m 1 -n 1"
        t_pattern = re.compile("User (.*)%, System (.*)%, IOW.*")
        result = g_common_obj.adb_cmd_common(cmd)
        print result
        if t_pattern.findall(result) != []:
            (t_user, t_system)= t_pattern.findall(result)[0]
            return int(t_user) + int(t_system)
        else:
            return 0

    def getNormalWifi(self):
        conf = self.config.read('tests.tablet.dut_init.conf','wifisetting')
        ssid = conf.get("ssid")
        passwd = conf.get("passwd")
        security = conf.get("security")
        print ssid, passwd, security
        return ssid, passwd, security

    def check_ap_connect(self, check_time=60):
        logger.debug("check ap connect")
        conf = self.config.read('tests.tablet.dut_init.conf', 'wifisetting')
        ssid = conf.get("ssid")
        check_connect_cmd = "adb -s {0} shell dumpsys connectivity | grep 'CONNECTED/CONNECTED.*{1}' \
            > /dev/null 2>&1".format(self.serial, ssid)
        for i in range(int(check_time)):
            time.sleep(1)
            if os.system(check_connect_cmd) == 0:
                logger.debug("ap connect,time:{0}".format(i))
                return True
        logger.debug("ap is not connected")
        return False

    def getRTPWifi(self):
        rtp_wifisetting = PhotosImpl(self.config.read(self.cfg_file, "rtp_wifisetting"))
        ssid = rtp_wifisetting.cfg.get("ssid")
        passwd = rtp_wifisetting.cfg.get("passwd")
        security = rtp_wifisetting.cfg.get("security")
        print ssid, passwd, security
        return ssid, passwd, security

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
        else:
            self.d.press.power()
            time.sleep(2)
            self.d.press.power()
        self.lock = SystemUI()
        self.lock.unlock_screen()
        time.sleep(1)
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def get_S0i3_value(self):
        s3_value = int(g_common_obj.adb_cmd_capture_msg("cat /d/suspend_stats |grep success").split(":")[-1])
        logger.debug("Suspend value:%s"%s3_value)
        return s3_value

    def videoplay_enter_S0i3(self, case_name):
        logger.debug("run case is " + str(case_name))
        self.appPrepare(case_name)
        sleep_count = self.get_S0i3_value()
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        time.sleep(1)
        MultiMediaBasicTestCase().suspend_ressmue(300)
        cur_sleep_count = self.get_S0i3_value()
        assert cur_sleep_count > sleep_count , "DUT enter S0i3 failed"
        logger.debug("case " + str(case_name) + " is pass")

    def interuptionStreamingVideoPlayBack(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        time.sleep(1)
        if 'bxtp_abl' in self.multimedia_setting.get_paltform_hardware():
            #  For BXT use relay card press power to sleep
            self.multimedia_setting.pressPowerKey()
            time.sleep(10)
            self.multimedia_setting.pressPowerKey()
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

    def testVideoPlayBackWithRemoveSDcard(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_setting.set_enable_MTP(False)
        try:
            self.multimedia_handle.launchVideoApp()
            time.sleep(2)
            self.videoPlayBack()
            self.checkVideoPlayBack()
            self.d.press.home()
            self.multimedia_setting.mount_SD(False)
            self.multimedia_setting.click_recent_app("OtcVideoPlayer")
            time.sleep(1)
            self.checkVideoNotSupport()
            self.multimedia_setting.mount_SD(True)
        except Exception as e:
            self.multimedia_setting.set_enable_MTP(True)
            assert False, e
        finally:
            self.multimedia_setting.set_enable_MTP(True)
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBackWithMusicPlayBackgroud(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.audio = AudioImpl(\
            self.config.read(self.cfg_file, case_name))
        self.multimedia_setting.push_file(self.video.cfg.get("push_audio"), self.video.cfg.get("datapath_2"))
        self.push_path = self.multimedia_setting.push_file(self.video.cfg.get("push_video"), self.video.cfg.get("datapath"))
        self.audio.cleanUpData()
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))
        if self.multimedia_setting.get_android_version() == "O":
            audio_name = self.video.cfg.get("push_audio").split('"')[3]
            self.play_audio_with_aospmusic(audio_name)
            self.multimedia_setting.press_home_car()
            self.multimedia_handle.launchVideoApp()
            self.videoPlayBack()
            self.checkVideoPlayBack()
            if not self.check_audio_with_aospmusic(audio_name):
                logger.error("audio is not pasued")
                return False
        else:
            self.audio.launch_music_am()
            self.audio.enterSongsPage()
            self.audio.playMusic(self.video.cfg.get("audio_name"))
            self.audio.checkMusicPlayBack(stopTime=self.video.cfg.get("stop_time"))
            ct = self.d(resourceId="com.google.android.music:id/currenttime").text
            ct = self.setTimeToSec(ct)
            starttime = datetime.datetime.now()
            self.d.press.home()
            self.multimedia_handle.launchVideoApp()
            time.sleep(1)
            self.videoPlayBack()
            self.checkVideoPlayBack()
            self.multimedia_setting.click_recent_app("Play Music")
            if self.d(resourceId="com.google.android.music:id/play").exists:
                self.d(resourceId="com.google.android.music:id/play").click()
            time.sleep(2)
            self.audio.checkMusicPlayBack(1)
            tt = self.d(resourceId="com.google.android.music:id/currenttime").text
            tt = self.setTimeToSec(tt)
            endtime = datetime.datetime.now()
            pt = (endtime - starttime).seconds
            if self.d(resourceId="com.google.android.music:id/pause").exists:
                self.d(resourceId="com.google.android.music:id/pause").click()
            assert tt <= ct + pt, "time error!tt=%d,ct=%d,pt=%d" % (tt, ct , pt)
        print "case " + str(case_name) + " is pass"

    def testStreamingVideoPlayBackWithMusicPlayBackgroud(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.audio = AudioImpl(\
            self.config.read(self.cfg_file, case_name))
        self.multimedia_setting.push_file(self.video.cfg.get("push_audio"), self.video.cfg.get("datapath_2"))
        self.audio.cleanUpData()
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))
        #self.audio.enterPlayPageFromHome()
        if self.multimedia_setting.get_android_version() == "O":
            audio_name = self.video.cfg.get("push_audio").split('"')[3]
            self.play_audio_with_aospmusic(audio_name)
            self.multimedia_setting.press_home_car()
            self.multimedia_handle.launchVideoApp()
            self.streamingVideoPlayBack()
            self.checkVideoPlayBack()
            if not self.check_audio_with_aospmusic(audio_name):
                logger.error("audio is not pasued")
                return False
        else:
            self.audio.launch_music_am()
            self.audio.enterSongsPage()
            self.audio.playMusic(self.video.cfg.get("audio_name"))
            self.audio.checkMusicPlayBack(stopTime=self.video.cfg.get("stop_time"))
            ct = self.d(resourceId="com.google.android.music:id/currenttime").text
            ct = self.setTimeToSec(ct)
            starttime = datetime.datetime.now()
            self.d.press.home()
            self.multimedia_handle.launchVideoApp()
            time.sleep(1)
            self.streamingVideoPlayBack()
            self.checkVideoPlayBack()
            self.multimedia_setting.click_recent_app("Play Music")
            if self.d(resourceId="com.google.android.music:id/play").exists:
                self.d(resourceId="com.google.android.music:id/play").click()
            self.audio.checkMusicPlayBack(1)
            tt = self.d(resourceId="com.google.android.music:id/currenttime").text
            tt = self.setTimeToSec(tt)
            endtime = datetime.datetime.now()
            pt = (endtime - starttime).seconds
            if self.d(resourceId="com.google.android.music:id/pause").exists:
                self.d(resourceId="com.google.android.music:id/pause").click()
            assert tt <= ct + pt, "time error!tt=%d,ct=%d,pt=%d" % (tt, ct , pt)
        print "case " + str(case_name) + " is pass"

    def testRTPStreamingVideoPlayBackWithMusicPlayBackgroud(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.audio = AudioImpl(\
            self.config.read(self.cfg_file, case_name))
        self.multimedia_setting.push_file(self.video.cfg.get("push_audio"), self.video.cfg.get("datapath_2"))
        self.audio.cleanUpData()
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))
        #self.audio.enterPlayPageFromHome()
        if self.multimedia_setting.get_android_version() == "O":
            audio_name = self.video.cfg.get("push_audio").split('"')[3]
            self.play_audio_with_aospmusic(audio_name)
            self.multimedia_setting.press_home_car()
            self.multimedia_handle.launchVideoApp()
            self.multimedia_handle.launchVideoApp()
            self.streamingVideoPlayBack()
            #self.checkVideoPlayBack()
            if not self.check_audio_with_aospmusic(audio_name):
                logger.error("audio is not pasued")
                return False
        else:
            self.audio.launch_music_am()
            self.audio.enterSongsPage()
            self.audio.playMusic(self.video.cfg.get("audio_name"))
            self.audio.checkMusicPlayBack(stopTime=self.video.cfg.get("stop_time"))
            ct = self.d(resourceId="com.google.android.music:id/currenttime").text
            ct = self.setTimeToSec(ct)
            starttime = datetime.datetime.now()
            self.d.press.home()
            self.multimedia_handle.launchVideoApp()
            time.sleep(2)
            self.streamingVideoPlayBack()
            #self.checkVideoPlayBack()
            self.multimedia_setting.click_recent_app("Play Music")
            if self.d(text="Listen Now").exists:
                self.d(resourceId="com.google.android.music:id/trackname").click()
            if self.d(resourceId="com.google.android.music:id/pause").exists:
                self.d(resourceId="com.google.android.music:id/pause").click()
            self.audio.checkMusicPlayBack(3)
            tt = self.d(resourceId="com.google.android.music:id/currenttime").text
            tt = self.setTimeToSec(tt)
            endtime = datetime.datetime.now()
            pt = (endtime - starttime).seconds
            if self.d(resourceId="com.google.android.music:id/pause").exists:
                self.d(resourceId="com.google.android.music:id/pause").click()
            assert tt <= ct + pt, "time error!tt=%d,ct=%d,pt=%d" % (tt, ct , pt)
        print "case " + str(case_name) + " is pass"

    def testRTPStreamingVideoPlayBackWithDownloadFile(self, case_name):
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

    def play_audio_with_aospmusic(self, audioname):
        """
        Args: Play audio file in device via AOSP Muisc.
        Returns: True or False
        """
        clear_logcat = "logcat -c"
        g_common_obj.adb_cmd_common(clear_logcat)
        cmd = "am start -a android.intent.action.VIEW -d file://{0} -n com.android.music/.MediaPlaybackActivity".format(audioname)
        g_common_obj.adb_cmd(cmd)
        if self.multimedia_setting.check_logs("NuPlayerDriver: start"):
            logger.debug("Play {0}  success.".format(audioname))
            return True
        else:
            logger.error("Play {0}  failed.".format(audioname))
            return False

    def check_audio_with_aospmusic(self, audioname):
        """
        Args:
        Returns:True or False
        """
        if self.multimedia_setting.check_logs("MediaPlaybackService: AudioFocus: received AUDIOFOCUS_LOSS"):
            logger.debug("{0} is pause".format(audioname))
        else:
            logger.error("{0} is not pasue".format(audioname))
            return False

    def record_video(self):
        self.multimedia_camera_helper = MultiMediaCameraHelper()
        self.multimedia_camera_helper.camera.startCameraApp()

        self.multimedia_camera_helper.camera.selectMode("Video")
        self.multimedia_camera_helper.camera.clickRecordBtn()
        time.sleep(10)
        self.multimedia_camera_helper.camera.clickRecordBtn()

    def testRecordingWithMusicPlayBackgroud(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.audio = AudioImpl(\
            self.config.read(self.cfg_file, case_name))
        self.multimedia_setting.push_file(self.video.cfg.get("push_audio"), self.video.cfg.get("datapath_2"))
        self.audio.cleanUpData()
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))
        if self.multimedia_setting.get_android_version() == "O":
            audio_name = self.video.cfg.get("push_audio").split('"')[3]
            self.play_audio_with_aospmusic(audio_name)
            self.multimedia_setting.press_home_car()
            self.record_video()
            if not self.check_audio_with_aospmusic(audio_name):
                logger.error("audio is not pasued")
                return False
        else:
            self.audio = AudioImpl(\
                self.config.read(self.cfg_file, case_name))
            self.multimedia_setting.push_file(self.video.cfg.get("push_audio"), self.video.cfg.get("datapath_2"))
            self.audio.cleanUpData()
            g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))
            self.audio.launch_music_am()
            self.audio.enterSongsPage()
            self.audio.playMusic(self.video.cfg.get("audio_name"))
            self.audio.checkMusicPlayBack(stopTime=self.video.cfg.get("stop_time"))
            self.d.press.home()
            self.record_video()
            self.multimedia_setting.click_recent_app("Play Music")
            time.sleep(2)
            if not self.d(resourceId="com.google.android.music:id/totaltime").exists:
                self.d(text=self.video.cfg.get("audio_name")).click()
                time.sleep(2)
            if self.d(resourceId="com.google.android.music:id/play").exists:
                self.d(resourceId="com.google.android.music:id/play").click()
            self.audio.checkMusicPlayBack(1)
            ct = self.d(resourceId="com.google.android.music:id/currenttime").text
            ct = self.setTimeToSec(ct)
            assert ct <=65, "time error!ct=%d" % ct
            if self.d(resourceId="com.google.android.music:id/pause").exists:
                self.d(resourceId="com.google.android.music:id/pause").click()
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBackWithReboot(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        adb.reboot_device()
        self.wait_boot_completed(1000)
        if self.d(textContains="Drive safely").exists:
            logger.debug(self.tag + "Drive saftely exist , click owner")
            self.d(text="Owner").click()
            time.sleep(3)
        SystemUI().unlock_screen()
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBackWithBatteryLevel(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        battery_before = self.checkBatteryLevel()
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        assert self.checkVideoPlayBack(), 'Video play failed'
        logger.debug(self.tag + "sleep 120 seconds")
        time.sleep(120)
        battery_after = self.checkBatteryLevel()
        assert abs(battery_before - battery_after) <= 10, "Battery Level error! battery_before=%d, battery_after=%d" % (battery_before, battery_after)
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBackWithPlugUnplugCharger(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        ser = self.getSerialPort()
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        self.useSerialPort("release up", ser)
        self.useSerialPort("press down", ser)
        ser.close()
        time.sleep(10)
        self.wait_boot_completed()
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testVideoNotSupport(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack(flag=2)
        self.checkVideoNotSupport()
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBackWithDisconnectWifi(self, case_name):
        def connect_AP():
            cfg_file = 'tests.tablet.dut_init.conf'
            from testlib.util.config import TestConfig
            from testlib.dut_init.dut_init_impl import Function
            self.config = TestConfig()
            self.conf = self.config.read(cfg_file,'wifisetting')
            self.ssid = self.conf.get("ssid")
            self.passwd = self.conf.get("passwd")
            self.security = self.conf.get("security")
            self.func = Function()
            succeed = False
            for _ in range(3):
                try:
                    self.func.connect_AP(self.ssid, self.passwd, self.security)
                    succeed = True
                    break
                except Exception as e:
                    print e
            assert succeed, "connect AP failed!"
        
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        self.wifi = WifiSettingImpl(0)
        self.wifi.launch_from_am()
        try:
            self.wifi.turn_off_wifi()
            self.d.press.home()
            self.multimedia_setting.click_recent_app("OtcVideoPlayer")
            self.checkVideoNotSupport()
            self.wifi.launch_from_am()
            if not self.wifi.turn_on_wifi(False):
                connect_AP()
        except Exception as e:
                print e
#                 self.wifi.turn_on_wifi(False)
                assert False, e
        time.sleep(20)
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBackWithCpuOverload(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.hal = AutodetectImpl()
        self.hal.make_device_cpu_overload()
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack(900)
        adb.reboot_device()
        adb.root_on_device()
        self.wait_boot_completed(1000)
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBackWithCpuCheck(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        cpu_1 = self.getCpuConsumption()
        self.d.press.home()
        time.sleep(10)
        #SystemUI().unlock_screen()
        cpu_2 = self.getCpuConsumption()
        print "cpu_1=%s, cpu_2=%s" % (cpu_1, cpu_2)
        assert cpu_1 >= cpu_2, "Cpu Consumption error! cpu_1=%s, cpu_2=%s" % (cpu_1, cpu_2)
        print "case " + str(case_name) + " is pass"

    def testStreamingVideoPlayBackWithCpuCheck(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        cpu_1 = self.getCpuConsumption()
        self.d.press.home()
        time.sleep(10)
        #SystemUI().unlock_screen()
        cpu_2 = self.getCpuConsumption()
        print "cpu_1=%s, cpu_2=%s" % (cpu_1, cpu_2)
        assert cpu_1 + 10 >= cpu_2, "Cpu Consumption error! cpu_1=%s, cpu_2=%s" % (cpu_1, cpu_2)
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBackWithCpuOverload_OneByOne(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.hal = AutodetectImpl()
        self.hal.make_device_cpu_overload()
        for _ in range(5):
            self.d.press.home()
            self.multimedia_handle.launchVideoApp()
            time.sleep(2)
            self.streamingVideoPlayBack()
            self.checkVideoPlayBack()
            self.d.press.home()
            self.multimedia_handle.launchVideoApp()
            time.sleep(2)
            self.streamingVideoPlayBack(self.video.cfg.get("video_path_2"))
            self.checkVideoPlayBack()
        adb.reboot_device()
        self.wait_boot_completed()
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBackAfterRebootDevice(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        for _ in range(20):
            adb.reboot_device()
            self.wait_boot_completed()
        time.sleep(20)
        self.lock = SystemUI()
        self.lock.unlock_screen()
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testVideoPlayBackLongTime(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.setSleepTime("15 seconds")
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        time.sleep(50)
        self.checkVideoPlayBack()
        self.setSleepTime("30 minutes")
        print "case " + str(case_name) + " is pass"

    def testStreamingVideoPlayBack(self, case_name):
        MultiMediaBasicTestCase().testStreamingVideoPlayBack(case_name)

    def testStreamingVideoPlayBackWithFillStorage(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        from testlib.camera.CameraCommon import CameraCommon
        try:
            CameraCommon().fillStorage(25)
            CameraCommon().unlockScreen()
            self.multimedia_handle.launchVideoApp()
            time.sleep(2)
            self.streamingVideoPlayBack()
            self.checkVideoPlayBack(900)
            CameraCommon().removeBigFile()
        except Exception as e:
            CameraCommon().removeBigFile()
            assert False, 'Exception: {0}'.format(e)
        print "case " + str(case_name) + " is pass"

    def testStreamingVideoPlayBackWithLockScreen(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        time.sleep(1)
        self.d.press.power()
        time.sleep(2)
        self.d.press.power()
        SystemUI().unlock_screen()
        if self.d(resourceId="android:id/play").exists:
            self.d(resourceId="android:id/play").click()
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testStreamingVideoPlayBackWithReboot(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        adb.reboot_device()
        self.wait_boot_completed(1000)
        if self.d(textContains="Drive safely").exists:
            logger.debug(self.tag + "Drive saftely exist , click owner")
            self.d(text="Owner").click()
            time.sleep(3)
        SystemUI().unlock_screen()
        self.check_ap_connect()
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testStreamingVideoPlayBackWithFileTransfer(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        self.push_path = self.multimedia_setting.push_file(self.video.cfg.get("push_video"), self.video.cfg.get("datapath"))
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def testStreamingVideoPlayBackTenTimes(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        for iteration in range(20):
            if self.d(resourceId="android:id/pause").exists:
                self.d(resourceId="android:id/pause").click()
            elif self.d(resourceId="android:id/play").exists:
                self.d(resourceId="android:id/play").click()
            self.checkVideoPlayBack(90)
            logger.debug("click button {0} times".format(iteration+1))
        print "case " + str(case_name) + " is pass"

    def videoPlayControlProcess(self, case_name, resume=True):
        logger.debug(self.tag + 'run case name is ' + sys._getframe().f_back.f_code.co_name)
        MultiMediaBasicTestCase().videoPlayControlProcess(case_name, resume)

    def streamingVideoPlayControlProcess(self, case_name):
        MultiMediaBasicTestCase().streamingVideoPlayControlProcess(case_name)

    def lowAndHighFramestreamingVideoPlayControlProcess(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack()
        time.sleep(10)
        self.checkVideoPlayBack()
        self.multimedia_setting.set_play_time(0.5)
        self.checkVideoPlayBack()
        self.d(resourceId="android:id/rew").click()
        self.checkVideoPlayBack()
        self.d(resourceId="android:id/ffwd").click()
        self.checkVideoPlayBack()
        self.d(resourceId="android:id/pause").click()
        self.checkVideoPlayBack()
        self.d.press.home()
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.streamingVideoPlayBack(self.video.cfg.get("video_path_2"))
        time.sleep(10)
        self.checkVideoPlayBack()
        self.multimedia_setting.set_play_time(0.5)
        self.checkVideoPlayBack()
        self.d(resourceId="android:id/rew").click()
        self.checkVideoPlayBack()
        self.d(resourceId="android:id/ffwd").click()
        self.checkVideoPlayBack()
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
        # press.home not work on IVI O.Car
        # self.d.press.home()
        self.multimedia_setting.click_recent_app("OtcVideoPlayer")
        time.sleep(1)
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def interuptionVideoPlayBackWithTask(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        time.sleep(1)
        self.d.press.home()
        assert not self.d(text="OtcVideoPlayer").exists, "Recent Apps buttons are not hidden!!"
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
        time.sleep(5)
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

    def videoPlayBackWithAlarmThreeTimes(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        for _ in range(0, 3):
            self.multimedia_setting.launchAlarmAPP()
            self.multimedia_setting.setAlarmTime(40)
            self.multimedia_handle.launchVideoApp()
            time.sleep(1)
            self.videoPlayBack()
            self.checkVideoPlayBack()
            self.multimedia_setting.waitAlarmTriiggered(60, "Dismiss")
            self.multimedia_setting.click_recent_app("OtcVideoPlayer")
            self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def streamingVideoPlayBackWithAlarm(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_setting.launchAlarmAPP()
        self.multimedia_setting.setAlarmTime(70)
        self.multimedia_handle.launchVideoApp()
        time.sleep(1)
        self.streamingVideoPlayBack()
        self.checkVideoPlayBack()
        time.sleep(1)
        self.multimedia_setting.waitAlarmTriiggered(90, "Snooze")
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
        self.d.click(self.y/2,self.x/2)
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
        self.d.click(self.y/2,self.x/2)
        self.setRotation("Portrait View")
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def cameraRecordWithAlarm(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_camera_helper = MultiMediaCameraHelper()
        self.multimedia_setting.launchAlarmAPP()
        self.multimedia_setting.setAlarmTime(50)
        self.multimedia_camera_helper.camera.startCameraApp()
        self.multimedia_camera_helper.camera.selectMode("Video")
        self.multimedia_camera_helper.camera.switchRearOrFront()
        self.multimedia_camera_helper.camera.clickRecordBtn()
        self.multimedia_setting.waitAlarmTriiggered(70, "Dismiss")
        print "case " + str(case_name) + " is pass"

    def interuptionVideoPlayBackWithRecording(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_camera_helper = MultiMediaCameraHelper()
        self.multimedia_handle.launchVideoApp()
        time.sleep(2)
        self.videoPlayBack()
        self.checkVideoPlayBack()
        self.multimedia_camera_helper.camera.startCameraApp()
        self.multimedia_camera_helper.camera.selectMode("Video")
        self.multimedia_camera_helper.camera.switchRearOrFront()
        self.multimedia_camera_helper.camera.clickRecordBtn()
        time.sleep(10)
        self.multimedia_setting.click_recent_app("OtcVideoPlayer")
        self.checkVideoPlayBack()
        print "case " + str(case_name) + " is pass"

    def recordingAfterDownloadFile(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_camera_helper = MultiMediaCameraHelper()
        self.multimedia_camera_helper.camera.startCameraApp()
        self.multimedia_camera_helper.camera.selectMode("Video")
        self.multimedia_camera_helper.camera.switchRearOrFront()
        self.multimedia_camera_helper.camera.clickRecordBtn()
        self.multimedia_setting.push_file(self.video.cfg.get("push_video"), self.video.cfg.get("datapath"))
        time.sleep(10)
        self.multimedia_camera_helper.camera.clickRecordBtn()
        time.sleep(5)
        self.multimedia_camera_helper.camera.reviewPhotoAndVideo()
        print "case " + str(case_name) + " is pass"

    def recordingWithLandscapePortrait(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name, 2)
        self.multimedia_camera_helper = MultiMediaCameraHelper()
        self.multimedia_camera_helper.camera.startCameraApp()
        self.multimedia_camera_helper.camera.selectMode("Video")
        self.multimedia_camera_helper.camera.switchRearOrFront()
        self.multimedia_camera_helper.camera.clickRecordBtn()
        time.sleep(10)
        self.d.orientation = "r"
        time.sleep(2)
        self.video.set_orientation_n()
        time.sleep(2)
        self.multimedia_camera_helper.camera.clickRecordBtn()
        time.sleep(5)
        self.multimedia_camera_helper.camera.reviewPhotoAndVideo()
        print "case " + str(case_name) + " is pass"


    def videoPlaySuspendWithPwd(self, case_name):
        self.appPrepare(case_name, 1)
        try:

            self.camera_common = CameraCommon()
            self.camera_common.setLockScreenWithPasswd()
            self.multimedia_handle.launchVideoApp()
            time.sleep(2)
            self.videoPlayBack()
            self.checkVideoPlayBack()
            time.sleep(1)
            hardware = self.multimedia_setting.get_paltform_hardware()
            if 'bxtp_abl' in hardware:
                #  For BXT use relay card press power to sleep
                self.multimedia_setting.pressPowerKey()
                time.sleep(10)
                self.multimedia_setting.pressPowerKey()
            else:
                self.d.press.power()
                time.sleep(2)
                self.d.press.power()
            self.camera_common.unlockScreen()
            assert self.checkVideoPlayBack(), "cannot paly video after resmue"
            logger.debug(self.tag + "case " + str(case_name) + " is pass")
        except Exception as e:
            logger.error(self.tag + "run case " + str(case_name) + " failed ,exception:" + str(e))
        finally:
            self.camera_common.setLockScreenWithPasswd(False)


    def testSuspend_Resume_DUT_Playing_Videos_By_Pressing_Power(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Press Power
        """
        self.interuptionVideoPlayBack("test_API_video_playback_001")

    def testSuspend_S0i3_Video_Playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Press Power
        """
        self.videoplay_enter_S0i3("test_API_video_playback_001")


    def testExit_Playing_Video_From_via_Home(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Switch to Home, then back from Home
        """
        self.interuptionVideoPlayBackSwitchHome("test_API_video_playback_002")

    def testDelete_Video(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Delete some video files
        """
        self.deleteVideoFileCase("test_API_video_playback_003")

    def testOne_Video_Thumbnail(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Check some video files in thumbnail
        """
        self.CheckVideoFileInThumbnail("test_API_video_playback_004")

    def testPlaying_streaming_Video_Locked(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Press Power
        """
        self.interuptionStreamingVideoPlayBack("test_API_video_playback_005")

    def testVideo_Duration_Playing(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video(10:00)
        """
        self.testVideoPlayBack("test_API_video_playback_006")

    def testVideo_Duration_Thumbnail(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Back to thumbnail
        """
        self.testVideoDurationThumbnail("test_API_video_playback_007")

    def testVideoPlayer_Video_Information(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Check the video files in thumbnail
        3. Check the video Details.
        """
        self.CheckVideoFileInformationInThumbnail("test_API_video_playback_010")

    def testView_Video_Detailed_Info(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Check the video files in thumbnail
        3. Rename file
        """
        self.RenameVideoFileInThumbnail("test_API_video_playback_011")

    def testPlaying_streaming_Music_Back_to_Streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Switch to Home, then back from Home
        """
        self.interuptionStreamingVideoPlayBackSwitchHome("test_API_video_playback_013")

    def testPlaying_streaming_Music_Locked(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Press Power
        """
        self.interuptionStreamingVideoPlayBack("test_API_video_playback_014")

    def testPlaying_streaming_Video_Home(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Switch to Home, then back from Home
        """
        self.interuptionStreamingVideoPlayBackSwitchHome("test_API_video_playback_015")

    def testPlaying_streaming_Video_Back_to_Video_Streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Switch to Home, then back from Home
        """
        self.interuptionStreamingVideoPlayBackSwitchHome("test_API_video_playback_016")

    def testPlaying_Video_Home(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Switch to Home, then back from Home
        """
        self.interuptionVideoPlayBackSwitchHome("test_API_video_playback_017")

    def testAlarm_Ring_When_Local_Video_Playing(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Alarm expired
        4. Snooze the alarm
        5. Dismiss the alarm
        """
        self.videoPlayBackWithAlarm("test_API_video_playback_018")

    def testAlarm_Ring_When_Playing_Video_Streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Alarm expired
        4. Snooze the alarm
        5. Dismiss the alarm
        """
        self.streamingVideoPlayBackWithAlarm("test_API_video_playback_019")

    def testPlaying_streaming_Music_Alarm_Calendar(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Alarm expired
        4. Snooze the alarm
        5. Dismiss the alarm
        """
        self.streamingVideoPlayBackWithAlarm("test_API_video_playback_020")

    def testPlaying_streaming_Video_Alarm_Calendar(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Alarm expired
        4. Snooze the alarm
        5. Dismiss the alarm
        """
        self.streamingVideoPlayBackWithAlarm("test_API_video_playback_021")

    def testVideo_streaming_Calendar_reminder_with_alarm_during_video_streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Alarm expired
        4. Snooze the alarm
        5. Dismiss the alarm
        """
        self.streamingVideoPlayBackWithAlarm("test_API_video_playback_022")

    def testLocal_Video_Playback_Control_Process(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.videoPlayControlProcess("test_API_video_playback_023", resume=False)

    def testStreaming_Video_Playback_Control_Process(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.streamingVideoPlayControlProcess("test_API_video_playback_024")

    def testVideo_Playback_Rotation(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Rotation Screen
        """
        self.testVideoPlaybackRotation("test_API_video_playback_025")

    def testSHALL_support_H264_video_stream_no_frame_rate_limitation(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Start/Stop
        """
        self.lowAndHighFramestreamingVideoPlayControlProcess("test_API_video_playback_026")

    def testRotate_device_during_streaming_playing(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Rotation Screen
        """
        self.testStreamingVideoPlaybackRotation("test_API_video_playback_027")

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

    def testClock_alarm_when_progressive_streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Alarm expired
        4. Snooze the alarm
        5. Dismiss the alarm
        """
        self.streamingVideoPlayBackWithAlarm("test_API_video_playback_030")

    def testSHALL_support_video_operatiseek_during_paused(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.streamingVideoPlayControlProcess("test_API_video_playback_031")
        
    def testVideo_recording_clock_alarm(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Camera record
        2. Alarm expired
        """
        self.cameraRecordWithAlarm("test_API_video_playback_032")

    def testCamera_video_recording_clock_alarm(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Camera record
        2. Alarm expired
        """
        self.cameraRecordWithAlarm("test_API_video_playback_039")

    def testDUT_doesnot_automatically_suspend_when_playing_video_file(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackLongTime("test_API_video_playback_040")

    def testMultiMedia_VideoPlayer_Switch_Application(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Switch to Home, then back from Home
        """
        self.interuptionVideoPlayBackSwitchHome("test_API_video_playback_041")

    def testST_Alarm_remains_when_play_download_video(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Alarm expired
        5. Dismiss the alarm
        """
        self.videoPlayBackWithAlarmThreeTimes("test_API_video_playback_044")

    def testReboot_ManyTimes_VideoPlay_Back(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Reboot Device
        2. Launch play video app
        3. Play video
        """
        self.testVideoPlayBackAfterRebootDevice("test_API_video_playback_046")

    def testNewly_Added_Video_Loading(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_052")

    def testVideo_streaming_Video_recording_while_video_streaming_paused(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Recording
        """
        self.interuptionVideoPlayBackWithRecording("test_API_video_playback_053")

    def testVerify_Stress_pause_function(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Play/Pause 10 times
        """
        self.testStreamingVideoPlayBackTenTimes("test_API_video_playback_054")

    def testPseudo_streaming_not_supported_format_or_features(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoNotSupport("test_API_video_playback_056")

    def testVideo_streaming_during_file_transfer_over_USB(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBackWithFileTransfer("test_API_video_playback_057")

    def testPseudo_streaming_cannot_reach_destination_via_wifi_network(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoNotSupport("test_API_video_playback_059")

    def testVideo_recording_file_downloading_background(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch recording app
        3. Recording
        """
        self.recordingAfterDownloadFile("test_API_video_playback_061")

    def testVideo_Record_Landscape_Portrait(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch recording app
        3. Recording
        """
        self.recordingWithLandscapePortrait("test_API_video_playback_062")

    def testDisconnecting_from_wifi_network_during_pseudo_streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithDisconnectWifi("test_API_video_playback_063")

    def testDisconnecting_from_wifi_network_during_RTP_streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithDisconnectWifi("test_API_video_playback_064")

    def testHTTP_while_CPU_loading_User_System_more_than_50Percent(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithCpuOverload("test_API_video_playback_065")

    def testPlay_AudioVideo_file_continuous_OneByOne_while_CPU_loading_User_System_is_more_than_50Percent(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithCpuOverload_OneByOne("test_API_video_playback_066")

    def testPlay_Video_HTTP_and_RTSP_while_CPU_loading_User_System_more_than_50Percent(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithCpuOverload("test_API_video_playback_067")

    def testPlay_AudioOnly_HTTP_and_RTSP_while_CPU_loading_User_System_more_than_50Percent(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithCpuOverload("test_API_video_playback_068")

    def testVideo_Streaming_Playback_CPU_Check(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBackWithCpuCheck("test_API_video_playback_069")

    def testVideo_Local_Playback_CPU_Check(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithCpuCheck("test_API_video_playback_070")

    def testVideo_streaming_USB_Charger(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_071")

    def testPlug_unplug_wall_Charger_when_play_video_file(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithPlugUnplugCharger("test_API_video_playback_072")

    def testEnable_PTP_during_video_playing_plugout_USB_cable(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithPlugUnplugCharger("test_API_video_playback_073")

    def testVideo_Playback_External_Storage(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBack("test_API_video_playback_082")

    def testSuspend_Mode_During_Streaming_Pressing_Power_Button(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        3. Press Power
        """
        self.interuptionVideoPlayBack("test_API_video_playback_083")

    def testPFT_7101_Network_streaming_protocols_HTTPS_progressive_streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_084")

    def testPseudo_streaming_HTTP_rotation_25fps_h264_720x576_mp4(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_087")

    def testTask_Lock(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.interuptionVideoPlayBackWithTask("test_API_video_playback_095")

    def testCheck_PowerChange_VideoPlayback_H264_1280x720p_30fps_AAC_128kb_44KHz(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithBatteryLevel("test_API_video_playback_096")

    def testPlaying_streaming_Video_Power_Off(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBackWithReboot("test_API_video_playback_099")

    def testPlaying_Video_Power_Off(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithReboot("test_API_video_playback_100")

    def testPseudo_streaming_wifi_connected_but_needs_web_authentication(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBack("test_API_video_playback_102")

    def testMusic_playing_background_when_local_video_playback(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithMusicPlayBackgroud("test_API_video_playback_104")

    def testPseudo_streaming_music_playing_background(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBackWithMusicPlayBackgroud("test_API_video_playback_105")

    def testVideo_recording_music_playing_background(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testRecordingWithMusicPlayBackgroud("test_API_video_playback_106")

    def testMusic_playing_background_when_RTP_streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testRTPStreamingVideoPlayBackWithMusicPlayBackgroud("test_API_video_playback_107")

    def testFile_downloading_background_when_RTP_streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testRTPStreamingVideoPlayBackWithDownloadFile("test_API_video_playback_108")

    def testScreen_dim_during_streaming(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBackWithLockScreen("test_API_video_playback_110")

    def testRemoving_SDcard_during_local_video_playback_from_the_same_SDcard(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testVideoPlayBackWithRemoveSDcard("test_API_video_playback_114")

    def testRTSP_while_Storage_Full(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.testStreamingVideoPlayBackWithFillStorage("test_API_video_playback_117")

    def testSuspend_Video_Playback_Screen_Unlock(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch play video app
        2. Play video
        """
        self.videoPlaySuspendWithPwd("test_API_video_playback_001")