# coding: utf-8
from testlib.camera.mum_camera_impl import CameraImpl
from testlib.photos.mum_photos_impl import PhotosImpl
from testlib.util.common import g_common_obj
from testlib.multimedia.testcasebase import TestCaseBase
from testlib.systemui.systemui_impl import SystemUI
from testlib.multimedia.multimedia_setting import MultiMediaSetting
from testlib.multimedia.multimedia_setting import MultiMediaHandle
from testlib.multimedia.multimedia_camera_helper import MultiMediaCameraHelper
from testlib.multimedia.multimedia_log import MultimediaLogger
logger = MultimediaLogger.instance()
import os
import time
import sys


class VideoEncode(TestCaseBase):
    """
    @summary: Test Video encode
    """

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(VideoEncode, self).setUp()
        self.d = g_common_obj.get_device()
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]
        self._test_name = __name__
        self.tag = "[Video Encode] "
        logger.debug(self.tag + 'run case name is ' + sys._getframe().f_back.f_code.co_name)
        logger.debug(self.tag + '[Setup]: ' + self._test_name)
        g_common_obj.stop_app_am("com.google.android.GoogleCamera")
        # Unlock screen
        g_common_obj.adb_cmd_capture_msg("input keyevent 82")

    def tearDown(self):
        """
        @summary: tearDown
        @return: None
        """
        super(VideoEncode, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        g_common_obj.stop_exp_handle()
        time.sleep(3)
        g_common_obj.stop_app_am("com.google.android.GoogleCamera")
        time.sleep(3)
        g_common_obj.adb_cmd_capture_msg(" rm -rf /sdcard/DCIM/Camera/*")
        time.sleep(10)
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))

    def appPrepare(self, case_name):
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_video.conf')
        self.video = PhotosImpl(\
            self.config.read(cfg_file, case_name))
        self.camera = CameraImpl(\
            self.config.read(cfg_file, case_name))
        self.multimedia_handle = MultiMediaHandle()
        g_common_obj.adb_cmd_capture_msg(" rm -rf /sdcard/DCIM/Camera/*")
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("refresh_sd"))
        
        self.multimedia_setting = MultiMediaSetting(cfg_file)
        self.multimedia_setting.install_apk("video_apk")
        self.multimedia_setting.install_apk("alarm_apk")
        self.multimedia_setting.install_apk("ffmpegCLI_apk")
        self.multimedia_setting.install_apk("EncodeAndMux_4K_Encode_apk")
        self.video.set_orientation_n()
        self.camera.clean_up_camera_data()
        if self.video.cfg.get("push_video") != None:
            self.multimedia_setting.clearFolder(self.video.cfg.get("remove_video"))
            self.push_path = self.multimedia_setting.push_file(self.video.cfg.get("push_video"), self.video.cfg.get("datapath"))

    def launchRecordAPP(self):
        SystemUI().unlock_screen()
        for _ in range(3):
            g_common_obj.launch_app_am("com.intel.vpg.tool", \
                                   "com.intel.vpg.tool.ConfActivity")
            time.sleep(3)
            if self.d(textContains="Vpg Media Tool").exists:
                return
        assert self.d(textContains="Vpg Media Tool").exists, "launch record app failed!"

    def launchVideoRecTestAPP(self):
        logger.debug(self.tag + "Start to launch VideoRecTest")
        for _ in range(3):
            g_common_obj.launch_app_am("com.intel.mchen33.videorectest", \
                                       ".MainActivity")
            time.sleep(3)
            if self.d(resourceId="com.intel.mchen33.videorectest:id/surfaceViewPreview").exists:
                return
        assert self.d(resourceId="com.intel.mchen33.videorectest:id/surfaceViewPreview").exists, "launch VideoRecTest app failed!"

    def videoRectTestRecod(self, encode_type='ST_VIDEO_REC_ENCODE_001'):
        for _ in range(3):
            self.d(resourceId="android:id/text1").click()
            self.d(scrollable=True).scroll.to(text=encode_type)
            self.d(text=encode_type).click()
            if self.d(text=encode_type).exists:
                logger.debug(self.tag + "set the encode type to %s" % encode_type)
                break
        try:
            self.d(text="Start").click()
            self.multimedia_setting.getScreenshotToHost("Video_Rec_Encode.png", g_common_obj.get_user_log_dir())
            time.sleep(2)
            assert not self.d(text="Failed").exists, "Cannot recoder this video"
            logger.debug(self.tag + "Start to record 15 seconds")
            time.sleep(15)
            self.d(text="Stop").click()
        except Exception as e:
            logger.error(self.tag + "Record fail, Exception:%s" % e)
            return False
        logger.debug(self.tag + "Record success")
        return True

    def checkVideoRecTestfile(self, encode_type='ST_VIDEO_REC_ENCODE_001', flag=True):
        import re
        size_pattern = re.compile("sdcard_rw(.*)20")
        file_path = '/sdcard/' + encode_type + '*'
        cmd = "shell ls -l %s" % (file_path)
        # result = g_common_obj.adb_cmd_capture_msg(cmd)
        result = self.multimedia_setting.execute_adb_command(cmd)
        size = re.findall(size_pattern, result)
        assert ("No such file or directory" not in result) == flag, "Record file status error! result=%s" % (result)
        if size != []:
            assert str(0) != size[0], "Record file size is zero, error!"
        g_common_obj.pull_file(g_common_obj.get_user_log_dir(), file_path)
        logger.debug(self.tag + "record file:" + result)
        logger.debug(self.tag + "record file exist")
        return True


    def clickRecordButton(self, record_file_name, record_type):
        self.d(className="android.widget.Spinner").click()
        self.d(textContains="VIDEO RECORDER").click()
        self.d(className="android.widget.EditText").set_text(record_file_name)
        self.d(className="android.widget.ScrollView").scroll.to(textContains="Use MediaCodec Encoder")
        self.d(textContains="Use MediaCodec Encoder").click()
        self.d(className="android.widget.ScrollView").scroll.to(textContains="APPLY")
        self.d(textContains="video/").click()
        self.d(text=record_type).click()
        self.d(className="android.widget.ScrollView").scroll.to(textContains="APPLY")
        self.d(textContains="APPLY").click()
        time.sleep(2)
        self.d.click(self.x/2,self.y/2)

    def deleteCameraRecordFile(self, file_name):
        cmd = "shell rm /sdcard/Pictures/VpgMediaTool/%s" % (file_name)
        g_common_obj.adb_cmd_common(cmd)

    def checkCameraRecordFile(self, file_name, flag=True):
        cmd = "shell ls /sdcard/Pictures/VpgMediaTool/%s" % (file_name)
        #result = g_common_obj.adb_cmd_capture_msg(cmd)
        result = self.multimedia_setting.execute_adb_command(cmd)
        logger.debug("camera file:" + result)
        assert ("No such file or directory" not in result) == flag, "Record file status error! result=%s" % (result)

    def changeCameraTypeStr(self, camera_type):
        if "Back" in camera_type:
            return "Back"
        return "Front"
            

    def videoEncodeThenPlayback(self, case_name):
        """
        This test used to test video encode then playback
        The test case spec is following:
        1. Launch camera
        2. record video
        3. playback
        """
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_camera_helper = MultiMediaCameraHelper()
        self.multimedia_camera_helper.camera.startCameraApp()
        self.multimedia_camera_helper.camera.selectMode("Video")
        camera_type = self.changeCameraTypeStr(self.video.cfg.get("camera_type"))
        self.multimedia_camera_helper.camera.switchRearOrFront(camera_type)
        self.multimedia_camera_helper.camera.clickRecordBtn()
        time.sleep(int(self.video.cfg.get("record_time")))
        self.multimedia_camera_helper.camera.clickRecordBtn()
        time.sleep(5)
        self.multimedia_camera_helper.camera.reviewPhotoAndVideo()
        print "case " + str(case_name) + " is pass"

    def recordingWithVPGTool(self, case_name):
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_setting.install_apk("vpg_apk")
        file_name = self.video.cfg.get("file_name")
        record_type = self.video.cfg.get("record_type")
        self.deleteCameraRecordFile(file_name)
        self.checkCameraRecordFile(file_name, False)
        self.launchRecordAPP()
        self.clickRecordButton(file_name.split(".")[0], record_type)
        time.sleep(2)
        assert not self.d(text="Unfortunately, Vpg Media Tool has stopped.").exists, "Vpg app error!!!"
        time.sleep(10)
        self.d.click(self.x/2,self.y/2)
        time.sleep(3)
        self.checkCameraRecordFile(file_name)
        print "case " + str(case_name) + " is pass"

    def recordingWithVideoRectest(self, case_name):
        logger.debug(self.tag + 'run case name is ' + sys._getframe().f_back.f_code.co_name)
        self.appPrepare(case_name)
        self.multimedia_setting.install_apk("rec_apk")
        encode_type = self.video.cfg.get("encode_type")
        g_common_obj.adb_cmd_capture_msg(self.video.cfg.get("remove_video"))
        #file_name = self.video.cfg.get("file_name")
        self.launchVideoRecTestAPP()
        assert self.videoRectTestRecod(encode_type) , "Record failed"
        assert self.checkVideoRecTestfile(encode_type), "Record file not exist"
        logger.debug(self.tag + 'run %s is pass '%sys._getframe().f_back.f_code.co_name)


    def videoEncodeSettingResolutionThenPlayback(self, case_name):
        """
        This test used to test video encode then playback
        The test case spec is following:
        1. Launch camera
        2. set resolution
        3. record video
        4. playback
        """
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_camera_helper = MultiMediaCameraHelper()
        self.multimedia_camera_helper.camera.startCameraApp()
        self.multimedia_camera_helper.camera.selectMode("Video")
        camera_type = self.changeCameraTypeStr(self.video.cfg.get("camera_type"))
        print camera_type
        if "bxtp" in self.multimedia_setting.get_paltform_hardware():
            logger.debug("For BXT, do not switch Rear/Front")
        else:
            self.multimedia_camera_helper.camera.switchRearOrFront(camera_type)
        resolution = self.multimedia_camera_helper.changeResolution(self.camera.cfg.get("resolution"))
        self.multimedia_camera_helper.camera.setVideoResolution(resolution, camera_type)
        self.multimedia_camera_helper.camera.recordVideo(1, int(self.video.cfg.get("record_time")))
        self.multimedia_camera_helper.camera.reviewPhotoAndVideo()
        print "case " + str(case_name) + " is pass"

    def videoEncodeSettingResolution(self, case_name):
        """
        This test used to test video encode then playback
        The test case spec is following:
        1. Launch camera
        2. set resolution
        3. record video
        4. playback
        """
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.camera.enter_camera_from_home()
        self.camera.switch_module_in_camera()
        self.camera.change_front_back_camera(self.video.cfg.get("camera_type"))
        self.camera.click_camera_menu_setting_vert()
        self.camera.enter_camera_setting_video_quality()
        self.camera.set_camera_setting_video_quality(self.video.cfg.get("resolution"),self.video.cfg.get("camera_type"))
        self.camera.press_back()
        self.camera.press_back()
        self.camera.capture_video_camera_initial_page(recordTime=self.video.cfg.get("record_time"))
        print "case " + str(case_name) + " is pass"

    def videoEncodeCapturePauseMultipleTimes(self, case_name):
        """
        This test used to test video encode then playback
        The test case spec is following:
        1. Launch camera
        2. set resolution
        3. Start/stop capturing video for multiple times quickly
        """
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.camera.enter_camera_from_home()
        self.camera.switch_module_in_camera()
        self.camera.change_front_back_camera(self.video.cfg.get("camera_type"))
        self.camera.click_camera_menu_setting_vert()
        self.camera.enter_camera_setting_video_quality()
        self.camera.set_camera_setting_video_quality(self.video.cfg.get("resolution"),self.video.cfg.get("camera_type"))
        self.camera.press_back()
        self.camera.press_back()
        self.camera.camera_video_capture_pause(self.video.cfg.get("click_times"))
        print "case " + str(case_name) + " is pass"

    def videoEncodeLongLasting(self, case_name):
        """
        This test used to test video encode then playback
        The test case spec is following:
        1. Launch camera
        2. record video
        """
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.camera.enter_camera_from_home()
        self.camera.switch_module_in_camera()
        self.camera.change_front_back_camera(self.camera.cfg.get("camera_type"))
        self.camera.click_camera_menu_setting_vert()
        self.camera.enter_camera_setting_video_quality()
        self.camera.set_camera_setting_video_quality(self.camera.cfg.get("resolution"),self.camera.cfg.get("camera_type"))
        self.camera.press_back()
        self.camera.press_back()
        start_time = time.time()
        record_time = int(self.camera.cfg.get("record_time"))
        while record_time > 0:
            record_time = record_time - (time.time() - start_time)
            total, used, free = self.multimedia_setting.get_sdcard_memory()
            if free < (total/30):
                g_common_obj.adb_cmd_capture_msg(" rm -rf /sdcard/DCIM/Camera/*")
                g_common_obj.adb_cmd_capture_msg(self.camera.cfg.get("refresh_sd"))
            self.camera.capture_video_camera_initial_page(record_time)
            print "case " + str(case_name) + " is pass"
 
    def videoEncodeIteration(self, case_name):
        """
        This test used to test video encode then playback
        The test case spec is following:
        1. Launch camera
        2. record video
        3. playback
        """
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_camera_helper = MultiMediaCameraHelper()
        self.multimedia_camera_helper.camera.startCameraApp()
        self.multimedia_camera_helper.camera.selectMode("Video")
        camera_type = self.changeCameraTypeStr(self.video.cfg.get("camera_type"))
        if "bxtp" in self.multimedia_setting.get_paltform_hardware():
            logger.debug("For BXT, do not switch Rear/Front")
        else:
            self.multimedia_camera_helper.camera.switchRearOrFront(camera_type)
        resolution = self.multimedia_camera_helper.changeResolution(self.camera.cfg.get("resolution"))
        self.multimedia_camera_helper.camera.setVideoResolution(resolution, camera_type)
        record_times = int(self.camera.cfg.get("record_times"))
        record_time = int(self.camera.cfg.get("record_time"))
        for index in range(record_times):
            print "record_times is ", index
            total, used, free = self.multimedia_setting.get_sdcard_memory()
            if free < (total/30):
                g_common_obj.adb_cmd_capture_msg(" rm -rf /sdcard/DCIM/Camera/*")
                g_common_obj.adb_cmd_capture_msg(self.camera.cfg.get("refresh_sd"))
                break
            self.multimedia_camera_helper.camera.recordVideo(1, record_time)
        print "case " + str(case_name) + " is pass"

    def videoEncodeTillFull(self, case_name):
        """
        This test used to test video encode then playback
        The test case spec is following:
        1. Launch camera
        2. record video till memory full
        3. playback
        """
        print "run case is " + str(case_name)
        self.appPrepare(case_name)
        self.multimedia_camera_helper = MultiMediaCameraHelper()
        self.multimedia_camera_helper.camera.startCameraApp()
        self.multimedia_camera_helper.camera.selectMode("Video")
        camera_type = self.changeCameraTypeStr(self.video.cfg.get("camera_type"))
        if "bxtp" in self.multimedia_setting.get_paltform_hardware():
            logger.debug("For BXT, do not switch Rear/Front")
        else:
            self.multimedia_camera_helper.camera.switchRearOrFront(camera_type)
        resolution = self.multimedia_camera_helper.changeResolution(self.camera.cfg.get("resolution"))
        self.multimedia_camera_helper.camera.setVideoResolution(resolution, camera_type)
        self.multimedia_camera_helper.camera.clickRecordBtn()
        time.sleep(5)
        record_time= int(self.video.cfg.get("lasting_time"))
        startTime=time.time()
        while True:
            if time.time() - startTime < record_time:
                if not self.multimedia_camera_helper.camera.isRecordTimeExists():
                    break
            else:
                self.multimedia_camera_helper.camera.clickRecordBtn()
                break
            time.sleep(10)
        time.sleep(5)
        self.multimedia_camera_helper.camera.reviewPhotoAndVideo()
        print "case " + str(case_name) + " is pass"

    def launchFFmpeg(self):
        SystemUI().unlock_screen()
        for _ in range(3):
            g_common_obj.launch_app_am("com.silentlexx.ffmpeggui", ".Gui")
            time.sleep(2)
            if self.d(textContains="Allow FFmpeg Media Encoder to access photo").exists:
                self.d(resourceId="com.android.packageinstaller:id/permission_allow_button").click()
            if self.d(text="FFmpeg Media Encoder").exists:
                return
        assert self.d(text="FFmpeg Media Encoder").exists, "launch FFmpeg app failed!"

    def encodevideo(self, in_file_path='', e_resolution='1080x720', e_type='3gp' ):
        input_file= self.d(resourceId='com.silentlexx.ffmpeggui:id/in_file')
        if input_file.exists and (in_file_path != input_file.text):
            input_file.click()
            while input_file.text != '':

                g_common_obj.adb_cmd_common("shell input keyevent DEL")
            g_common_obj.adb_cmd_common("shell input text %s" % in_file_path)

        output_type = self.d(resourceId='com.silentlexx.ffmpeggui:id/sp_ext').child(resourceId='android:id/text1')
        output_type.click()
        try:
            self.d(scrollable=True).scroll.to(text=e_type).click()
        except:
            self.d(text=e_type).click()

        encode_type = self.d(resourceId='com.silentlexx.ffmpeggui:id/preset').child(resourceId='android:id/text1')
        encode_type.click()
        try:
            self.d(scrollable=True).scroll.to(text="Video 3gp (h263/aac/qcif)").click()
        except:
            self.d(text='Video 3gp (h263/aac/qcif)').click()

        enable_setres = self.d(resourceId='com.silentlexx.ffmpeggui:id/setres')
        input_x = self.d(resourceId='com.silentlexx.ffmpeggui:id/rx')
        input_y = self.d(resourceId='com.silentlexx.ffmpeggui:id/ry')
        if input_x.exists and not input_file.click():
            enable_setres.click()

        iter_x = 5
        iter_y = 5
        if input_x.text !=  e_resolution.split('x')[0]:
            while input_x.text != '' and iter_x > 0:

                g_common_obj.adb_cmd_common("shell input keyevent DEL")
                iter_y -= 1
            input_x.click()
            g_common_obj.adb_cmd_common("shell input text %s" % e_resolution.split('x')[0])

        if input_y.text !=  e_resolution.split('y')[0]:
            while input_y.text != '' and iter_y > 0:
                g_common_obj.adb_cmd_common("shell input keyevent DEL")
                iter_y -= 1
            input_y.click()
            g_common_obj.adb_cmd_common("shell input text %s" % e_resolution.split('x')[1])

        self.d(resourceId='com.silentlexx.ffmpeggui:id/run').click()

    def recordingWithWithFFmpeg(self, casename=''):
        self.appPrepare(casename)
        self.launchFFmpeg()
        self.encodevideo(self.video.cfg.get("push_video").split('"')[-2], self.video.cfg.get("resolution"), \
                         self.video.cfg.get("encode_type"))

    def launchFFmpegCLI(self):
        SystemUI().unlock_screen()
        for _ in range(3):
            g_common_obj.launch_app_am("org.magiclen.ffmpeg.cli", ".activities.CommandActivity")
            time.sleep(2)
            if self.d(textContains="Allow FFmpeg CLI to access").exists:
                self.d(resourceId="com.android.packageinstaller:id/permission_allow_button").click()
            if self.d(text="FFmpeg CLI").exists:
                logger.debug(self.tag + "launch FFmpeg CLI app success!")
                return
        assert self.d(text="FFmpeg CLI").exists, "launch FFmpeg CLI app failed!"

    def encodevideoViaFFmpegCLI(self, encode_cmd=''):
        logger.debug(self.tag + "start to encode video via FFmpeg CLI APP!")
        encode_cmd_box= self.d(resourceId='org.magiclen.ffmpeg.cli:id/etCLI')
        if encode_cmd_box.exists :
            encode_cmd_box.click()
            if ("Input FFmpeg's parameters here." != encode_cmd_box.text):
                while encode_cmd_box.text != '':
                    g_common_obj.adb_cmd_common("shell input keyevent DEL")
            g_common_obj.adb_cmd_common("shell input text %s" % encode_cmd)

        try:
            self.d(resourceId='org.magiclen.ffmpeg.cli:id/bCLI').click()
        except Exception as e:
            logger.error(self.tag + "Click Run button failed ,Excetpion:%s" % e)

        out_screen = self.d(resourceId='org.magiclen.ffmpeg.cli:id/tvScreen')
        for _ in range(15):
            time.sleep(3)
            if "Qavg: 0.00" in out_screen.text:
                logger.debug(self.tag + "encode done")
                return True
        return False

    def checkEncodeVideo(self, play_file='', check_hang=True):
        if g_common_obj.adb_cmd_common("shell getprop ro.hardware") in ['gordon_peak']:
            assert self.multimedia_handle.playVideoviaOGallery(play_file), 'launch encode video failed'
            if check_hang:
                return self.multimedia_handle.checkVideoPlayHang()
            else:
                return True
        else:
            self.multimedia_handle.launchVideoApp()
            self.multimedia_handle.videoPlayBack(play_file)
            return self.multimedia_handle.checkVideoPlayBack()

    def recordingWithWithFFmpegCLI(self, casename=''):
        self.appPrepare(casename)
        self.launchFFmpegCLI()
        assert self.encodevideoViaFFmpegCLI(self.video.cfg.get("encode_cmd")), 'encode video failed'
        assert self.checkEncodeVideo(self.video.cfg.get("output_file")), "check encode video failed"


    def recordingWithWith4KEncode(self, casename=''):
        self.appPrepare(casename)
        self.launch4KEncode()
        self.encodevideoVia4KEncode(Encode_type=self.video.cfg.get("encode_type"))
        assert self.checkEncodeVideo(self.video.cfg.get("output_file"),check_hang=False), "check encode video failed"

    def launch4KEncode(self):
        """
        A method to launch testEncodeAndMux_4K_Encode.apk
        :return: True, launch success.
        """
        SystemUI().unlock_screen()
        for _ in range(3):
            g_common_obj.launch_app_am("com.intel.mchen33.testencodeandmux", ".MainActivity")
            time.sleep(1)
            if self.d(text="testEncodeAndMux").exists:
                logger.debug(self.tag + "launch testEncodeAndMux_4K_Encode app success!")
                return True
        assert self.d(text="testEncodeAndMux").exists, "launch testEncodeAndMux_4K_Encode app failed!"

    def encodevideoVia4KEncode(self,Encode_type='GO VP8 3840x2160 30fps'):
        """
        A method to encode via testEncodeAndMux_4K_Encode.apk
        :return: True, Encode finished.
        """
        if Encode_type=='VP8':
            if self.d(text='GO H.264 3840x2160 30fps').exists:
                logger.debug(self.tag + "try to encode GO H.264 3840x2160 30fps" )
                self.d(text='GO VP8 3840x2160 30fps').click()
        else:
            if self.d(text='GO H.264 3840x2160 30fps').exists:
                logger.debug(self.tag + "try to encode GO H.264 3840x2160 30fps " )
                self.d(text='GO VP8 3840x2160 30fps').click()

        for _ in range(40):
            time.sleep(1)
            if not self.d(text="testEncodeAndMux").exists:
                logger.debug(self.tag + "launch testEncodeAndMux_4K_Encode app success!")
                return True
        return False

    def testVideoEncode_Longlasting_FrontCamera_480p_30mins(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_001
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_001')

    def testVideoEncode_Longlasting_RearCamera_480p_30mins(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_002
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_002')

    def testVideoEncode_Longlasting_RearCamera_720p_1280x720_30mins(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_003
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_003')

    def testVideoEncode_Longlasting_RearCamera_1080p_1920x1080_30mins(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_004
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_004')

    def test_video_encode_playback_mum_005(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        """
        self.videoEncodeLongLasting('mum_test_video_encode_playback_005')

    def testVideoEncode_Longlasting_FrontCamera_480p_60mins(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_006
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_006')

    def test_video_encode_playback_mum_007(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        """
        self.videoEncodeCapturePauseMultipleTimes('mum_test_video_encode_playback_007')

    def testVideoEncode_Longlasting_RearCamera_720p_PlaybackCheck(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_008
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_008')

    def test_video_encode_playback_mum_009(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_009')

    def test_video_encode_playback_mum_010(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_010')

    def testVideoEncode_Longlasting_RearCamera_720p_PlaybackInParallel(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_011
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_011')

    def testVideoEncode_Longlasting_RearCamera_720p_30mins(self):
        """
        This test used to test video record and playback
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_012
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_012')

    def test_video_encode_playback_mum_013(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        """
        self.videoEncodeCapturePauseMultipleTimes('mum_test_video_encode_playback_013')

    def test_video_encode_playback_mum_014(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        """
        self.videoEncodeIteration('mum_test_video_encode_playback_014')

    def test_video_encode_playback_mum_015(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        """
        self.videoEncodeTillFull('mum_test_video_encode_playback_015')

    def testVideoEncode_Longlasting_FrontCamera_480p_Storagefull_PlaybackCheck(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_016
        """
        self.videoEncodeTillFull('mum_test_video_encode_playback_016')

    def testVideoEncode_Longlasting_RearCamera_720p_Storagefull_PlaybackCheck(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_017
        """
        self.videoEncodeTillFull('mum_test_video_encode_playback_017')

    def testVideoEncode_Longlasting_RearCamera_480p_Storagefull_PlaybackCheck(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_018
        """
        self.videoEncodeTillFull('mum_test_video_encode_playback_018')

    def testVideoEncode_Iteration_FrontCamera_RecordStartStop_AudibleTones(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_019
        """
        self.videoEncodeIteration('mum_test_video_encode_playback_019')

    def testVideoEncode_Longlasting_RearCamera_1080p_10mins_PlaybackCheck(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_020
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_020')

    def testVideoEncode_Longlasting_FrontCamera_480p_10mins_PlaybackCheck(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        1. Former name: test_video_encode_playback_mum_021
        """
        self.videoEncodeSettingResolutionThenPlayback('mum_test_video_encode_playback_021')

    def testAudible_tones_are_heard_video_record_start_and_stop_default_settings_rear_camera(self):
        """
        This test used to test video record long lasting
        The test case spec is following:
        1. Former name: test_API_video_playback_060
        """
        self.videoEncodeThenPlayback('test_API_video_playback_060')

    def testSimultaneous_encode_2way_video_H264(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch recording app
        3. Recording
        """
        self.recordingWithVPGTool("test_API_video_playback_085")

    def testSimultaneous_encode_2way_video_H263(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch recording app
        3. Recording
        """
        self.recordingWithVPGTool("test_API_video_playback_097")

    def testSimultaneous_encode_2way_video_VP8(self):
        """
        This test used to test Video playback
        The test case spec is following:
        1. Launch recording app
        3. Recording
        """
        self.recordingWithVPGTool("test_API_video_playback_098")

    def testVideo_Encode_H264_1920x1080_30fps_mp4(self):
        """
            This test used to encode with videoRectest app
            The test case spec is following:
            1. Launch recording app
            3. Recording
        """
        self.recordingWithVideoRectest("test_API_video_playback_133")

    def testVideo_Encode_H264_320x240_30fps_mp4(self):
        """
            This test used to encode with videoRectest app
            The test case spec is following:
            1. Launch recording app
            3. Recording
        """
        self.recordingWithVideoRectest("test_API_video_playback_134")

    def testVideo_Encode_H264_352x288_30fps_mp4(self):
        """
            This test used to encode with videoRectest app
            The test case spec is following:
            1. Launch recording app
            3. Recording
        """
        self.recordingWithVideoRectest("test_API_video_playback_135")

    def testVideo_Encode_H264_1280x720_30fps_mp4(self):
        """
            This test used to encode with videoRectest app
            The test case spec is following:
            1. Launch recording app
            3. Recording
        """
        self.recordingWithVideoRectest("test_API_video_playback_136")

    def testVideo_Encode_H264_720x480_30fps_3gp(self):
        """
            This test used to encode with videoRectest app
            The test case spec is following:
            1. Launch recording app
            3. Recording
        """
        self.recordingWithVideoRectest("test_API_video_playback_137")

    def testVideo_Rec_Encode_VP8_1920x1080_30fps_webm(self):
        """
            This test used to encode with videoRectest app
            The test case spec is following:
            1. Launch recording app
            3. Recording
        """
        self.recordingWithVideoRectest("test_API_video_playback_138")

    def testVideo_Rec_Encode_VP8_720x480_30fps_webm(self):
        """
            This test used to encode with videoRectest app
            The test case spec is following:
            1. Launch recording app
            3. Recording
        """
        self.recordingWithVideoRectest("test_API_video_playback_139")

    def testEncode_H264_3gp(self):
        """
            This test used to encode with videoRectest app
            The test case spec is following:
            1. Launch recording app
            3. Recording
        """
        self.recordingWithVideoRectest("mum_test_video_encode_playback_025")

    def testVideo_Encode_H263_128x96_30fps_3gp(self):
        """
            This test used to encode with FFmpeg CLI app
            The test case spec is following:
            1. Launch recording app
            3. Recording
        """
        self.recordingWithWithFFmpegCLI("mum_test_video_encode_playback_022")

    def testVideo_Encode_H263_352x288_30fps_3gp(self):
        """
            This test used to encode with FFmpeg CLI app
            The test case spec is following:
            1. Launch recording app
            3. Recording
        """
        self.recordingWithWithFFmpegCLI("mum_test_video_encode_playback_023")

    def testVideo_Encode_H263_176x144_30fps_3gp(self):
        """
            This test used to encode with FFmpeg CLI app
            The test case spec is following:
            1. Launch recording app
            3. Recording
        """
        self.recordingWithWithFFmpegCLI("mum_test_video_encode_playback_024")

    def testVP8_HW_4K_Encode(self):
        """
            This test used to encode with 4k Encode app
            The test case spec is following:
            1. Launch recording app
            3. Recording
        """
        self.recordingWithWith4KEncode("mum_test_video_encode_playback_026")